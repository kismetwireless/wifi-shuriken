// scanner_platform_esp32_monitor.cpp
#include "scanner_platform_esp32_monitor.h"

#include <stdlib.h>
#include <string.h>

#include <Arduino.h>
#include <WiFi.h>
#include <esp_cpu.h>
#include <esp_wifi.h>

#include "../../include/capture_ring.h"
#include "wifi_result_utils.h"

namespace {

constexpr uint32_t RING_BYTES = 65536;

uint8_t* ring_storage = nullptr;
CaptureRing ring;
bool active = false;
uint8_t current_channel = 0;
uint8_t current_band = 0;

// Capture configuration, written with promiscuous RX quiesced or as single
// aligned stores the WiFi task reads per frame.
volatile uint16_t snaplen[3] = {};
volatile uint8_t type_mask = 0;

// Counters written only by the promiscuous callback.
volatile uint32_t cb_packets[3] = {};
volatile uint32_t cb_bytes = 0;
volatile uint32_t cb_copied = 0;
volatile uint32_t cb_drops = 0;
volatile uint32_t cb_cycles = 0;
volatile uint32_t cb_max_cycles = 0;
volatile uint8_t last_frame_channel = 0;

uint32_t ring_high_water = 0;
bool sim_drain = false;
bool synthetic = false;
uint16_t synthetic_len = 0;
uint32_t synthetic_seq = 0;
uint32_t synthetic_budget = 0;
uint32_t synthetic_last_us = 0;
uint8_t synthetic_body[PKT_RECORD_MAX_CAP];
uint32_t drain_kib_per_s = 0;
uint32_t drain_budget = 0;
uint32_t last_budget_ms = 0;
uint32_t started_ms = 0;
uint8_t error_stage = MONITOR_STAGE_NONE;
int32_t error_code = ESP_OK;

// Scanner-side channel hopping. The list is staged while chunks arrive and
// swapped in once complete.
bool hopping = false;
uint16_t hop_dwell_ms = 0;
uint8_t hop_count = 0;
uint8_t hop_index = 0;
uint32_t hop_next_ms = 0;
MonitorHopEntry hop_list[MONITOR_HOP_MAX_ENTRIES] = {};
MonitorHopEntry hop_staging[MONITOR_HOP_MAX_ENTRIES] = {};
uint8_t hop_staged = 0;
uint16_t hop_switches = 0;
uint16_t hop_switch_max_us = 0;

// Batch awaiting acknowledgement. Its records stay in the ring (peeked, not
// consumed) until CMD_PKT_READ acknowledges batch_seq.
bool batch_valid = false;
uint16_t batch_seq = 0;
uint16_t batch_records = 0;
uint16_t batch_record_bytes = 0;
uint16_t batch_wire_len = 0;
uint32_t batch_crc = 0;

int8_t startFailed(uint8_t stage, esp_err_t err) {
  esp_wifi_set_promiscuous(false);
  active = false;
  error_stage = stage;
  error_code = err;
  return SCANNER_STATUS_START_FAILED;
}

void IRAM_ATTR promiscuousCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
  const uint32_t t0 = esp_cpu_get_cycle_count();
  if (type > WIFI_PKT_DATA || !(type_mask & (1u << type))) {
    return;
  }
  const wifi_promiscuous_pkt_t* pkt = static_cast<const wifi_promiscuous_pkt_t*>(buf);
  // sig_len counts four trailing bytes that are not a valid FCS on the C5
  // (verified against captured management frames), so frames are recorded
  // without them: dump_len when the driver supplies it, else sig_len - 4.
  const uint16_t sig_len = pkt->rx_ctrl.sig_len;
  uint16_t len = pkt->rx_ctrl.dump_len;
  if (len == 0 || len > sig_len) {
    len = sig_len > 4 ? static_cast<uint16_t>(sig_len - 4) : 0;
  }
  if (len == 0) {
    return;
  }
  const uint16_t snap = snaplen[type];
  uint16_t cap = (snap != 0 && snap < len) ? snap : len;
  if (cap > PKT_RECORD_MAX_CAP) {
    cap = PKT_RECORD_MAX_CAP;
  }

  MonitorPacketRecord rec;
  rec.record_len = static_cast<uint16_t>(sizeof(rec) + cap);
  rec.orig_len = len;
  rec.timestamp_us = pkt->rx_ctrl.timestamp;
  rec.rssi = pkt->rx_ctrl.rssi;
  rec.noise = pkt->rx_ctrl.noise_floor;
  rec.channel = pkt->rx_ctrl.channel;
  // Derive the band from the channel the frame was received on, not the
  // current tuning, so frames that land mid-hop are labelled correctly.
  rec.band = rec.channel <= 14 ? WIFI_BAND_24_GHZ : WIFI_BAND_5_GHZ;
  rec.rate = pkt->rx_ctrl.rate;
  rec.sig_mode = pkt->rx_ctrl.cur_bb_format;
  rec.flags = 0;
  if (cap < len) {
    rec.flags |= MONITOR_PKT_FLAG_TRUNCATED;
  }
  rec.type = static_cast<uint8_t>(type);
  last_frame_channel = rec.channel;

  cb_packets[type] = cb_packets[type] + 1;
  cb_bytes = cb_bytes + len;
  if (ring.tryWrite(&rec, sizeof(rec), pkt->payload, cap)) {
    cb_copied = cb_copied + rec.record_len;
  } else {
    cb_drops = cb_drops + 1;
  }

  const uint32_t dt = esp_cpu_get_cycle_count() - t0;
  cb_cycles = cb_cycles + dt;
  if (dt > cb_max_cycles) {
    cb_max_cycles = dt;
  }
}

void applyFilter() {
  wifi_promiscuous_filter_t filter = {};
  if (type_mask & MONITOR_TYPE_MGMT) filter.filter_mask |= WIFI_PROMIS_FILTER_MASK_MGMT;
  if (type_mask & MONITOR_TYPE_CTRL) filter.filter_mask |= WIFI_PROMIS_FILTER_MASK_CTRL;
  if (type_mask & MONITOR_TYPE_DATA) filter.filter_mask |= WIFI_PROMIS_FILTER_MASK_DATA;
  esp_wifi_set_promiscuous_filter(&filter);
  wifi_promiscuous_filter_t ctrl = {};
  ctrl.filter_mask = WIFI_PROMIS_CTRL_FILTER_MASK_ALL;
  esp_wifi_set_promiscuous_ctrl_filter(&ctrl);
}

void resetCounters() {
  for (uint8_t i = 0; i < 3; i++) {
    cb_packets[i] = 0;
  }
  cb_bytes = 0;
  cb_copied = 0;
  cb_drops = 0;
  cb_cycles = 0;
  cb_max_cycles = 0;
  ring_high_water = 0;
  started_ms = millis();
}

// Produce synthetic records against a byte budget that accrues at
// drain_kib_per_s; the main loop is the ring's only producer in this mode.
void runSyntheticGenerator() {
  const uint32_t now = micros();
  const uint64_t accrued =
      static_cast<uint64_t>(now - synthetic_last_us) * drain_kib_per_s * 1024u / 1000000u;
  synthetic_last_us = now;
  const uint32_t cap = drain_kib_per_s * 1024u / 20u;  // At most 50 ms of credit.
  synthetic_budget = static_cast<uint32_t>(
      synthetic_budget + accrued > cap ? cap : synthetic_budget + accrued);

  const uint32_t record_len = sizeof(MonitorPacketRecord) + synthetic_len;
  while (synthetic_budget >= record_len) {
    synthetic_budget -= record_len;
    MonitorPacketRecord rec = {};
    rec.record_len = static_cast<uint16_t>(record_len);
    rec.orig_len = synthetic_len;
    rec.timestamp_us = now;
    rec.channel = current_channel;
    rec.band = current_band;
    rec.flags = MONITOR_PKT_FLAG_SYNTHETIC;
    rec.type = WIFI_PKT_DATA;
    memcpy(synthetic_body, &synthetic_seq, sizeof(synthetic_seq));
    cb_packets[WIFI_PKT_DATA] = cb_packets[WIFI_PKT_DATA] + 1;
    cb_bytes = cb_bytes + synthetic_len;
    if (ring.tryWrite(&rec, sizeof(rec), synthetic_body, synthetic_len)) {
      cb_copied = cb_copied + record_len;
    } else {
      cb_drops = cb_drops + 1;
    }
    // Sequence advances even on a drop so the reader sees the gap.
    synthetic_seq++;
  }
}

// Retune without touching promiscuous mode or the ring. Band mode is pinned
// per band, so it only changes when a hop crosses bands.
esp_err_t tuneTo(uint8_t band, uint8_t channel) {
  if (band != current_band) {
    esp_wifi_set_band_mode(band == WIFI_BAND_5_GHZ ? WIFI_BAND_MODE_5G_ONLY
                                                    : WIFI_BAND_MODE_2G_ONLY);
    current_band = band;
  }
  const esp_err_t err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  if (err == ESP_OK) {
    current_channel = channel;
  }
  return err;
}

void runHopper() {
  const uint32_t now = millis();
  if ((int32_t)(now - hop_next_ms) < 0) {
    return;
  }
  hop_index = static_cast<uint8_t>((hop_index + 1) % hop_count);
  // Schedule from the previous deadline so the average dwell holds even
  // when the main loop is late; resynchronise after a long stall.
  hop_next_ms += hop_dwell_ms;
  if ((int32_t)(now - hop_next_ms) >= 0) {
    hop_next_ms = now + hop_dwell_ms;
  }
  const uint32_t t0 = micros();
  tuneTo(hop_list[hop_index].band, hop_list[hop_index].channel);
  const uint32_t dt = micros() - t0;
  if (dt > hop_switch_max_us) {
    hop_switch_max_us = static_cast<uint16_t>(dt > UINT16_MAX ? UINT16_MAX : dt);
  }
  hop_switches++;
}

void noteRingUsage() {
  const uint32_t used = ring.used();
  if (used > ring_high_water) {
    ring_high_water = used;
  }
}

// Copy whole records from the front of the ring into out (after the batch
// header) without consuming them.
void buildBatch(uint8_t* out) {
  const uint32_t limit = PKT_BATCH_MAX_BYTES - sizeof(PktBatchHeader);
  uint8_t* records = out + sizeof(PktBatchHeader);
  uint32_t offset = 0;
  uint16_t count = 0;
  while (true) {
    MonitorPacketRecord rec;
    if (ring.peek(reinterpret_cast<uint8_t*>(&rec), sizeof(rec), offset) != sizeof(rec)) {
      break;
    }
    if (offset + rec.record_len > limit) {
      break;
    }
    ring.peek(records + offset, rec.record_len, offset);
    offset += rec.record_len;
    count++;
  }

  batch_seq++;
  batch_records = count;
  batch_record_bytes = static_cast<uint16_t>(offset);
  batch_crc = pktCrc32(records, offset);
  batch_wire_len = pktBatchWireLen(batch_record_bytes);

  PktBatchHeader header = {};
  header.magic = PKT_BATCH_MAGIC;
  header.seq = batch_seq;
  header.record_bytes = batch_record_bytes;
  header.records = batch_records;
  header.crc32 = batch_crc;
  memcpy(out, &header, sizeof(header));
  // Zero the alignment padding so the wire image is deterministic.
  memset(records + offset, 0, batch_wire_len - sizeof(PktBatchHeader) - offset);
  batch_valid = true;
}

}  // namespace

int8_t scannerMonitorStart(const MonitorStartCommand& command) {
  if (!wifiBandChannelValid(command.band, command.channel) ||
      (command.type_mask & (MONITOR_TYPE_MGMT | MONITOR_TYPE_CTRL | MONITOR_TYPE_DATA)) == 0) {
    return SCANNER_STATUS_INVALID;
  }
  if (ring_storage == nullptr) {
    ring_storage = static_cast<uint8_t*>(malloc(RING_BYTES));
    if (ring_storage == nullptr) {
      return startFailed(MONITOR_STAGE_ALLOC, ESP_ERR_NO_MEM);
    }
  }
  // scannerRadioInit() leaves STA stopped until the first scan, and
  // promiscuous mode needs the driver running.
  if (!WiFi.enableSTA(true)) {
    return startFailed(MONITOR_STAGE_WIFI_START, ESP_FAIL);
  }
  error_stage = MONITOR_STAGE_NONE;
  error_code = ESP_OK;
  // A scan the controller abandoned may still be running in the driver.
  esp_wifi_scan_stop();

  // Quiesce the callback while the ring and counters are reset. Anything
  // still queued, including an unacknowledged batch, belongs to the old
  // channel and is discarded.
  esp_wifi_set_promiscuous(false);
  ring.init(ring_storage, RING_BYTES);
  batch_valid = false;
  resetCounters();
  type_mask = command.type_mask;
  snaplen[0] = command.snaplen_mgmt;
  snaplen[1] = command.snaplen_ctrl;
  snaplen[2] = command.snaplen_data;
  sim_drain = (command.flags & MONITOR_START_FLAG_SIM_DRAIN) != 0;
  synthetic = (command.flags & MONITOR_START_FLAG_SYNTHETIC) != 0;
  synthetic_len = command.snaplen_data;
  if (synthetic_len < sizeof(uint32_t)) {
    synthetic_len = sizeof(uint32_t);
  } else if (synthetic_len > PKT_RECORD_MAX_CAP) {
    synthetic_len = PKT_RECORD_MAX_CAP;
  }
  synthetic_seq = 0;
  synthetic_budget = 0;
  synthetic_last_us = micros();
  for (uint16_t i = 0; i < sizeof(synthetic_body); i++) {
    synthetic_body[i] = static_cast<uint8_t>(i);
  }
  drain_kib_per_s = command.drain_kib_per_s;
  drain_budget = 0;
  last_budget_ms = millis();
  current_band = command.band;
  hopping = false;
  hop_staged = 0;
  hop_switches = 0;
  hop_switch_max_us = 0;

  if (synthetic) {
    // The radio stays out of it; nothing else may produce into the ring.
    current_channel = command.channel;
    last_frame_channel = 0;
    active = true;
    return SCANNER_STATUS_OK;
  }

  esp_wifi_set_ps(WIFI_PS_NONE);
  // Pin the band explicitly; AUTO lets the driver pick which band a bare
  // channel number applies to.
  esp_wifi_set_band_mode(command.band == WIFI_BAND_5_GHZ ? WIFI_BAND_MODE_5G_ONLY
                                                          : WIFI_BAND_MODE_2G_ONLY);
  applyFilter();
  esp_wifi_set_promiscuous_rx_cb(promiscuousCallback);
  esp_err_t err = esp_wifi_set_promiscuous(true);
  if (err != ESP_OK) {
    return startFailed(MONITOR_STAGE_PROMISCUOUS, err);
  }
  err = esp_wifi_set_channel(command.channel, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK) {
    return startFailed(MONITOR_STAGE_CHANNEL, err);
  }
  current_channel = command.channel;
  last_frame_channel = 0;
  active = true;
  return SCANNER_STATUS_OK;
}

void scannerMonitorStop() {
  hopping = false;
  if (!active) {
    return;
  }
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_promiscuous_rx_cb(nullptr);
  // Capture pins the band; scanning needs both bands back.
  esp_wifi_set_band_mode(WIFI_BAND_MODE_AUTO);
  active = false;
  batch_valid = false;
}

bool scannerMonitorActive() {
  return active;
}

void scannerMonitorService() {
  if (!active) {
    return;
  }
  if (synthetic) {
    runSyntheticGenerator();
  } else if (hopping) {
    runHopper();
  }
  noteRingUsage();
  if (!sim_drain) {
    return;
  }

  const uint32_t now = millis();
  if (drain_kib_per_s != 0 && now != last_budget_ms) {
    const uint32_t add = (now - last_budget_ms) * drain_kib_per_s * 1024u / 1000u;
    const uint32_t cap = drain_kib_per_s * 1024u / 10u;  // At most 100 ms of credit.
    drain_budget = (drain_budget + add > cap) ? cap : drain_budget + add;
  }
  last_budget_ms = now;

  uint32_t allowance = drain_kib_per_s != 0 ? drain_budget : UINT32_MAX;
  while (allowance > 0) {
    const uint8_t* span = nullptr;
    uint32_t n = ring.readSpan(&span);
    if (n == 0) {
      break;
    }
    if (n > allowance) {
      n = allowance;
    }
    ring.consume(n);
    allowance -= n;
    if (drain_kib_per_s != 0) {
      drain_budget -= n;
    }
  }
}

int8_t scannerMonitorSetHop(const MonitorHopCommand& command) {
  if (!active || synthetic) {
    return SCANNER_STATUS_INVALID;
  }
  if (command.total_count == 0) {
    hopping = false;
    hop_staged = 0;
    return SCANNER_STATUS_OK;
  }
  if (command.total_count > MONITOR_HOP_MAX_ENTRIES ||
      command.entry_count == 0 || command.entry_count > MONITOR_HOP_ENTRIES_PER_FRAME ||
      command.first_index + command.entry_count > command.total_count ||
      command.start_index >= command.total_count ||
      command.dwell_ms < MONITOR_HOP_MIN_DWELL_MS) {
    return SCANNER_STATUS_INVALID;
  }
  if (command.first_index == 0) {
    hop_staged = 0;
  } else if (command.first_index != hop_staged) {
    // A chunk went missing; the controller resends the list from the start.
    return SCANNER_STATUS_INVALID;
  }
  for (uint8_t i = 0; i < command.entry_count; i++) {
    const MonitorHopEntry& entry = command.entries[i];
    if (!wifiBandChannelValid(entry.band, entry.channel)) {
      return SCANNER_STATUS_INVALID;
    }
    hop_staging[command.first_index + i] = entry;
  }
  hop_staged = static_cast<uint8_t>(command.first_index + command.entry_count);
  if (hop_staged < command.total_count) {
    return SCANNER_STATUS_OK;
  }

  // Complete: swap the list in and tune to the starting entry now.
  memcpy(hop_list, hop_staging, command.total_count * sizeof(MonitorHopEntry));
  hop_count = command.total_count;
  hop_dwell_ms = command.dwell_ms;
  hop_index = command.start_index;
  hop_staged = 0;
  const esp_err_t err = tuneTo(hop_list[hop_index].band, hop_list[hop_index].channel);
  if (err != ESP_OK) {
    hopping = false;
    error_stage = MONITOR_STAGE_CHANNEL;
    error_code = err;
    return SCANNER_STATUS_START_FAILED;
  }
  const uint16_t first_delay = command.first_hop_delay_ms != 0 &&
                                       command.first_hop_delay_ms <= command.dwell_ms
                                   ? command.first_hop_delay_ms
                                   : command.dwell_ms;
  hop_next_ms = millis() + first_delay;
  hopping = hop_count > 1;
  return SCANNER_STATUS_OK;
}

uint16_t scannerMonitorPrepareBatch(const PktReadCommand& command, uint8_t* batch_out,
                                    PktReadReply& reply) {
  memset(&reply, 0, sizeof(reply));
  if (!active || sim_drain) {
    reply.status = SCANNER_STATUS_INVALID;
    return 0;
  }
  noteRingUsage();
  if (batch_valid && (command.flags & PKT_READ_FLAG_ACK) && command.ack_seq == batch_seq) {
    ring.consume(batch_record_bytes);
    batch_valid = false;
  }
  // An unacknowledged batch is resent unchanged (same sequence) so the master
  // can recover a corrupt transfer; otherwise build the next one.
  if (!batch_valid && ring.used() >= sizeof(MonitorPacketRecord)) {
    buildBatch(batch_out);
  }

  reply.status = SCANNER_STATUS_OK;
  reply.channel = current_channel;
  reply.ring_drops = cb_drops;
  const uint32_t used = ring.used();
  if (!batch_valid) {
    reply.pending_bytes = used;
    return 0;
  }
  reply.seq = batch_seq;
  reply.wire_len = batch_wire_len;
  reply.records = batch_records;
  reply.record_bytes = batch_record_bytes;
  reply.pending_bytes = used - batch_record_bytes;
  reply.crc32 = batch_crc;
  return batch_wire_len;
}

void scannerMonitorFillStats(MonitorStatsReply& out) {
  memset(&out, 0, sizeof(out));
  out.status = SCANNER_STATUS_OK;
  out.active = active ? 1 : 0;
  out.channel = current_channel;
  out.cpu_mhz = static_cast<uint8_t>(getCpuFrequencyMhz());
  out.packets_mgmt = cb_packets[0];
  out.packets_ctrl = cb_packets[1];
  out.packets_data = cb_packets[2];
  out.air_bytes = cb_bytes;
  out.copied_bytes = cb_copied;
  out.ring_drops = cb_drops;
  out.cb_cycles = cb_cycles;
  const uint32_t max_cycles = cb_max_cycles;
  cb_max_cycles = 0;
  const uint32_t max_us = max_cycles / (out.cpu_mhz ? out.cpu_mhz : 1);
  out.cb_max_us = static_cast<uint16_t>(max_us > UINT16_MAX ? UINT16_MAX : max_us);
  out.ring_kib = static_cast<uint16_t>(RING_BYTES / 1024u);
  out.ring_high_water = ring_high_water;
  ring_high_water = 0;
  out.free_heap = ESP.getFreeHeap();
  out.uptime_ms = millis() - started_ms;
  out.error_stage = error_stage;
  out.error_code = error_code;
  uint8_t primary = 0;
  wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
  if (esp_wifi_get_channel(&primary, &second) == ESP_OK) {
    out.radio_channel = primary;
  }
  out.last_frame_channel = last_frame_channel;
  out.hops = hop_switches;
  out.hop_switch_max_us = hop_switch_max_us;
  hop_switch_max_us = 0;
}
