#include "controller_usb_capture.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <pico/mutex.h>
#include <pico/time.h>

#include <string.h>

#include "Configuration.h"
#include "capture_ring.h"
#include "class/vendor/vendor_device.h"
#include "controller_scanner_runtime.h"
#include "dot11_filter.h"
#include "radio_channels.h"
#include "spi_protocol_shared.h"

// This module owns the host-facing packet capture interface: the vendor-class
// USB descriptor, host command parsing, per-slot capture configuration, the
// packet/control rings, and the pump that drains them into the bulk IN
// endpoint. Packets come from the synthetic beacon generator (core0) or, in a
// radio session, from the scanner runtime on core1 through
// controllerUsbCaptureQueuePacket(); the two never produce at the same time.

extern mutex_t __usb_mutex;

namespace {

#if SCANNER_USE_SHIFTREG_CS
constexpr uint8_t SLOT_COUNT = SCANNER_SHIFTREG_OUTPUTS;
#else
constexpr uint8_t SLOT_COUNT = 1;
#endif
static_assert(SLOT_COUNT <= USB_CAPTURE_MAX_SLOTS, "Too many scanner slots for the capture protocol");
constexpr uint8_t ALL_SLOTS_MASK = static_cast<uint8_t>((1u << SLOT_COUNT) - 1u);

// GET_RADIO_INFO copies scanner width bits straight into USB fields.
static_assert(RADIO_WIDTH_20 == USB_CAPTURE_WIDTH_20 && RADIO_WIDTH_40 == USB_CAPTURE_WIDTH_40 &&
                  RADIO_WIDTH_80 == USB_CAPTURE_WIDTH_80 && RADIO_WIDTH_160 == USB_CAPTURE_WIDTH_160,
              "Radio and USB width bits must match");
static_assert(RADIO_CHAN_WIDTH_20 == USB_CAPTURE_CHAN_20 &&
                  RADIO_CHAN_WIDTH_40_ABOVE == USB_CAPTURE_CHAN_40_ABOVE &&
                  RADIO_CHAN_WIDTH_40_BELOW == USB_CAPTURE_CHAN_40_BELOW &&
                  RADIO_CHAN_WIDTH_80 == USB_CAPTURE_CHAN_80,
              "Radio and USB per-channel width bits must match");
static_assert(DOT11_DATA_PASS_ALL == USB_CAPTURE_DATA_PASS_ALL &&
                  DOT11_DATA_TRUNCATE_ALL == USB_CAPTURE_DATA_TRUNCATE_ALL &&
                  DOT11_DATA_PASS_OPEN == USB_CAPTURE_DATA_PASS_OPEN &&
                  DOT11_DATA_PASS_HANDSHAKE == USB_CAPTURE_DATA_PASS_HANDSHAKE &&
                  DOT11_DATA_MODE_COUNT == USB_CAPTURE_DATA_PASS_HANDSHAKE + 1,
              "Frame filter and USB data modes must match");
static_assert(RADIO_BAND_24GHZ == USB_CAPTURE_BAND_24GHZ && RADIO_BAND_5GHZ == USB_CAPTURE_BAND_5GHZ,
              "Radio and USB band values must match");

static_assert(CaptureRing::capacityValid(USB_CAPTURE_PACKET_RING_BYTES),
              "USB_CAPTURE_PACKET_RING_BYTES must be a power of two");
static_assert(CaptureRing::capacityValid(USB_CAPTURE_CONTROL_RING_BYTES),
              "USB_CAPTURE_CONTROL_RING_BYTES must be a power of two");

constexpr uint16_t FS_BULK_PACKET_SIZE = 64;
constexpr uint32_t SYNTHETIC_MAX_RATE_PPS = 50000;
constexpr uint64_t SYNTHETIC_MAX_BACKLOG_NS = 100000000ull;
constexpr uint16_t HOP_MIN_DWELL_MS = 10;

// Default per-slot channels until the host configures them.
constexpr uint8_t DEFAULT_CHANNELS[USB_CAPTURE_MAX_SLOTS] = {1, 6, 11, 36, 44, 149, 157, 165};
// Whole data frames by default: the data mode does the trimming.
constexpr uint16_t DEFAULT_SNAPLEN_DATA = 0;

class UsbCaptureInterface : public Adafruit_USBD_Interface {
 public:
  bool begin() {
    setStringDescriptor(USB_CAPTURE_IFACE_NAME);
    return TinyUSBDevice.addInterface(*this);
  }

  uint16_t getInterfaceDescriptor(uint8_t, uint8_t* buf, uint16_t bufsize) override {
    if (buf == nullptr) {
      return TUD_VENDOR_DESC_LEN;
    }
    if (bufsize < TUD_VENDOR_DESC_LEN) {
      return 0;
    }

    const uint8_t itf = TinyUSBDevice.allocInterface(1);
    const uint8_t ep_out = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
    const uint8_t ep_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
    // TUD_VENDOR_DESCRIPTOR() hardcodes subclass/protocol 0. Spell the
    // descriptor out so hosts can find the capture interface by its class
    // triple without reading string descriptors.
    const uint8_t desc[] = {
        9, TUSB_DESC_INTERFACE, itf, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC,
        USB_CAPTURE_IFACE_SUBCLASS, USB_CAPTURE_IFACE_PROTOCOL, _strid,
        7, TUSB_DESC_ENDPOINT, ep_out, TUSB_XFER_BULK, U16_TO_U8S_LE(FS_BULK_PACKET_SIZE), 0,
        7, TUSB_DESC_ENDPOINT, ep_in, TUSB_XFER_BULK, U16_TO_U8S_LE(FS_BULK_PACKET_SIZE), 0};
    static_assert(sizeof(desc) == TUD_VENDOR_DESC_LEN, "Vendor descriptor length mismatch");
    memcpy(buf, desc, sizeof(desc));
    return sizeof(desc);
  }
};

struct SlotConfig {
  bool hopping;
  uint8_t band;
  uint8_t channel;
  uint8_t hop_count;
  uint8_t hop_offset;
  uint16_t dwell_ms;
  UsbCaptureHopEntry hop[USB_CAPTURE_MAX_HOP_CHANNELS];
  uint8_t type_mask;
  uint16_t snaplen_mgmt;
  uint16_t snaplen_ctrl;
  uint16_t snaplen_data;
  uint8_t data_mode;
};

struct __attribute__((packed)) PacketRecordPrefix {
  UsbCaptureRecordHeader record;
  UsbCapturePacketHeader packet;
};

UsbCaptureInterface capture_interface;

uint8_t packet_ring_storage[USB_CAPTURE_PACKET_RING_BYTES] __attribute__((aligned(4)));
uint8_t control_ring_storage[USB_CAPTURE_CONTROL_RING_BYTES] __attribute__((aligned(4)));
CaptureRing packet_ring;
CaptureRing control_ring;

SlotConfig slot_config[SLOT_COUNT] = {};

// Session state is owned by core0; the packet producer only reads it.
volatile bool session_active = false;
// The session streams real radio frames: core1 owns the scanners and is the
// packet ring's producer until the hand-off reports the stop applied.
bool radio_session = false;
uint8_t session_slot_mask = 0;
uint32_t last_host_ms = 0;
uint32_t next_status_ms = 0;
// Set when queued packets must be dropped; the pump applies it at the next
// record boundary so a half-sent record never gets cut off mid-stream.
volatile bool discard_packets_pending = false;
bool was_mounted = false;

// Host command reassembly. Twice the largest command so a full command still
// fits after resynchronizing past garbage bytes.
uint8_t cmd_buf[USB_CAPTURE_MAX_COMMAND_BYTES * 2];
uint16_t cmd_len = 0;

// USB TX pump: the record currently being streamed and its unsent bytes.
CaptureRing* tx_ring = nullptr;
uint32_t tx_remaining = 0;

// Counters. Producer-side counters are written only by the packet producer.
uint32_t tx_records = 0;
uint32_t tx_bytes = 0;
volatile uint32_t ring_drop_records = 0;
volatile uint32_t ring_drop_bytes = 0;
uint16_t packet_seq = 0;
uint16_t control_seq = 0;
volatile uint32_t slot_packets[SLOT_COUNT] = {};

// Counter values when the current session started. The producer may be core1,
// so core0 never resets those counters; status reports the difference.
uint32_t session_base_tx_records = 0;
uint32_t session_base_tx_bytes = 0;
uint32_t session_base_ring_drop_records = 0;
uint32_t session_base_ring_drop_bytes = 0;
uint32_t session_base_slot_packets[SLOT_COUNT] = {};

void markSessionCounters() {
  session_base_tx_records = tx_records;
  session_base_tx_bytes = tx_bytes;
  session_base_ring_drop_records = ring_drop_records;
  session_base_ring_drop_bytes = ring_drop_bytes;
  for (uint8_t s = 0; s < SLOT_COUNT; s++) {
    session_base_slot_packets[s] = slot_packets[s];
  }
}

#if USB_CAPTURE_SYNTHETIC_ENABLED
// Synthetic beacon layout; byte offsets patched per frame.
constexpr uint16_t SYNTH_OFF_SA_LAST = 15;
constexpr uint16_t SYNTH_OFF_BSSID_LAST = 21;
constexpr uint16_t SYNTH_OFF_SEQ_CTRL = 22;
constexpr uint16_t SYNTH_OFF_TSF = 24;
constexpr uint16_t SYNTH_OFF_SSID_SLOT = 48;
constexpr uint16_t SYNTH_OFF_DS_CHANNEL = 61;
constexpr uint16_t SYNTH_BASE_LEN = 62;
constexpr uint16_t SYNTH_MIN_PAD_IE = 6;  // Vendor IE: id, len, OUI, type.

bool synthetic_active = false;
uint16_t synthetic_frame_len = 0;
uint64_t synthetic_period_ns = 0;
uint64_t synthetic_next_due_ns = 0;
uint8_t synthetic_next_slot = 0;
uint16_t synthetic_frame_seq = 0;
uint8_t synthetic_frame[USB_CAPTURE_MAX_PACKET_BYTES];
#endif

uint32_t nowMs() {
  return to_ms_since_boot(get_absolute_time());
}

uint8_t slotRank(uint8_t mask, uint8_t slot) {
  uint8_t rank = 0;
  for (uint8_t s = 0; s < slot; s++) {
    if (mask & (1u << s)) {
      rank++;
    }
  }
  return rank;
}

uint8_t slotPopcount(uint8_t mask) {
  return slotRank(mask, SLOT_COUNT);
}

void resetSlotDefaults() {
  for (uint8_t s = 0; s < SLOT_COUNT; s++) {
    SlotConfig& cfg = slot_config[s];
    cfg = {};
    cfg.channel = DEFAULT_CHANNELS[s];
    cfg.band = (cfg.channel <= 14) ? USB_CAPTURE_BAND_24GHZ : USB_CAPTURE_BAND_5GHZ;
    cfg.type_mask = USB_CAPTURE_FILTER_MGMT | USB_CAPTURE_FILTER_DATA;
    cfg.snaplen_data = DEFAULT_SNAPLEN_DATA;
    cfg.data_mode = USB_CAPTURE_DATA_MODE_DEFAULT;
  }
}

void slotCurrentChannel(uint8_t slot, uint32_t now_ms, uint8_t& band, uint8_t& channel) {
  const SlotConfig& cfg = slot_config[slot];
  if (!cfg.hopping || cfg.hop_count == 0) {
    band = cfg.band;
    channel = cfg.channel;
    return;
  }
  const uint32_t index = ((now_ms / cfg.dwell_ms) + cfg.hop_offset) % cfg.hop_count;
  band = cfg.hop[index].band;
  channel = cfg.hop[index].channel;
}

bool queueControlRecord(uint8_t type, uint16_t seq, const void* body, uint16_t body_len) {
  UsbCaptureRecordHeader hdr = {};
  usbCaptureFillHeader(hdr, type, static_cast<uint16_t>(sizeof(hdr) + body_len), seq);
  return control_ring.tryWrite(&hdr, sizeof(hdr), body, body_len);
}

void sendReply(uint8_t command, uint16_t seq, int8_t status,
               const void* payload = nullptr, uint16_t payload_len = 0) {
  uint8_t body[sizeof(UsbCaptureReplyHeader) + sizeof(UsbCaptureInfo)] = {};
  if (payload_len > sizeof(body) - sizeof(UsbCaptureReplyHeader)) {
    payload_len = 0;
  }
  UsbCaptureReplyHeader reply = {};
  reply.command = command;
  reply.status = status;
  memcpy(body, &reply, sizeof(reply));
  if (payload_len > 0) {
    memcpy(body + sizeof(reply), payload, payload_len);
  }
  queueControlRecord(USB_CAPTURE_REC_REPLY, seq, body,
                     static_cast<uint16_t>(sizeof(reply) + payload_len));
}

void sendStatus() {
  uint8_t body[sizeof(UsbCaptureStatusHeader) + SLOT_COUNT * sizeof(UsbCaptureSlotStatus)] = {};
  UsbCaptureStatusHeader status = {};
  status.active = session_active ? 1 : 0;
  status.slot_count = SLOT_COUNT;
  status.uptime_ms = nowMs();
  status.tx_records = tx_records - session_base_tx_records;
  status.tx_bytes = tx_bytes - session_base_tx_bytes;
  status.ring_drop_records = ring_drop_records - session_base_ring_drop_records;
  status.ring_drop_bytes = ring_drop_bytes - session_base_ring_drop_bytes;
  memcpy(body, &status, sizeof(status));

  const uint32_t now_ms = nowMs();
  for (uint8_t s = 0; s < SLOT_COUNT; s++) {
    UsbCaptureSlotStatus slot = {};
    if (session_active && (session_slot_mask & (1u << s))) {
      slot.flags |= USB_CAPTURE_SLOT_ENABLED;
    }
    if (radio_session) {
      ControllerMonitorSlotStatus radio = {};
      controllerScannerMonitorSlotStatus(s, radio);
      if (radio.present) {
        slot.flags |= USB_CAPTURE_SLOT_PRESENT;
      }
      if (radio.monitoring) {
        slot.flags |= USB_CAPTURE_SLOT_MONITOR;
      }
      if (radio.hopping) {
        slot.flags |= USB_CAPTURE_SLOT_HOPPING;
      }
      if (radio.monitoring) {
        slot.band = radio.band;
        slot.channel = radio.channel;
      } else {
        // Not capturing yet (or failed): report the channel requested rather
        // than whatever the slot was last on.
        slotCurrentChannel(s, now_ms, slot.band, slot.channel);
      }
      slot.radio_drops = radio.radio_drops;
      slot.transport_errors = radio.transport_errors;
    } else {
      if (slot_config[s].hopping) {
        slot.flags |= USB_CAPTURE_SLOT_HOPPING;
      }
      slotCurrentChannel(s, now_ms, slot.band, slot.channel);
    }
    slot.packets = slot_packets[s] - session_base_slot_packets[s];
    memcpy(body + sizeof(status) + s * sizeof(slot), &slot, sizeof(slot));
  }
  queueControlRecord(USB_CAPTURE_REC_STATUS, control_seq++, body, sizeof(body));
}

// Ask core1 to capture (or stop) according to the current slot config.
void publishRadioRequest(bool active) {
  // ~1.2 KB with hop lists; built in static storage rather than on the stack.
  static ControllerMonitorRequest request;
  request = {};
  request.active = active;
  request.slot_mask = active ? session_slot_mask : 0;
  for (uint8_t s = 0; s < SLOT_COUNT && s < CONTROLLER_MONITOR_MAX_SLOTS; s++) {
    const SlotConfig& cfg = slot_config[s];
    ControllerMonitorSlotConfig& out = request.slot[s];
    out.band = cfg.band;
    out.channel = cfg.channel;
    // USB filter bits for mgmt/ctrl/data match MONITOR_TYPE_*.
    out.type_mask = cfg.type_mask & (MONITOR_TYPE_MGMT | MONITOR_TYPE_CTRL | MONITOR_TYPE_DATA);
    out.snaplen_mgmt = cfg.snaplen_mgmt;
    out.snaplen_ctrl = cfg.snaplen_ctrl;
    // Trimming every data frame to its header needs no more than the longest
    // header from the scanner, which saves SPI bandwidth.
    out.snaplen_data = dot11ScannerDataSnaplen(cfg.data_mode, cfg.snaplen_data);
    out.data_mode = cfg.data_mode;
    out.hopping = cfg.hopping && cfg.hop_count > 0;
    if (out.hopping) {
      out.hop_count = cfg.hop_count > CONTROLLER_MONITOR_MAX_HOP ? CONTROLLER_MONITOR_MAX_HOP
                                                                 : cfg.hop_count;
      out.hop_offset = cfg.hop_offset;
      out.hop_dwell_ms = cfg.dwell_ms;
      for (uint8_t i = 0; i < out.hop_count; i++) {
        out.hop[i].band = cfg.hop[i].band;
        out.hop[i].channel = cfg.hop[i].channel;
      }
    }
  }
  controllerScannerMonitorRequest(request);
}

void stopSession() {
  session_active = false;
  session_slot_mask = 0;
  discard_packets_pending = true;
#if USB_CAPTURE_SYNTHETIC_ENABLED
  synthetic_active = false;
#endif
  if (radio_session) {
    radio_session = false;
    publishRadioRequest(false);
  }
}

// Drop everything queued toward the host, including bytes already sitting in
// the TinyUSB FIFO. Caller holds __usb_mutex.
void resetTransportLocked() {
  tud_vendor_write_clear();
  tx_ring = nullptr;
  tx_remaining = 0;
  packet_ring.discardAll();
  control_ring.discardAll();
  discard_packets_pending = false;
}

#if USB_CAPTURE_SYNTHETIC_ENABLED
void buildSyntheticFrame(uint16_t frame_len) {
  static const uint8_t base[SYNTH_BASE_LEN] = {
      0x80, 0x00,                          // Frame control: beacon.
      0x00, 0x00,                          // Duration.
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  // DA: broadcast.
      0x02, 0x53, 0x48, 0x4b, 0x00, 0x00,  // SA: locally administered, slot in last byte.
      0x02, 0x53, 0x48, 0x4b, 0x00, 0x00,  // BSSID.
      0x00, 0x00,                          // Sequence control.
      0, 0, 0, 0, 0, 0, 0, 0,              // TSF.
      0x64, 0x00,                          // Beacon interval: 100 TU.
      0x01, 0x04,                          // Capabilities: ESS, short slot.
      0x00, 11, 's', 'h', 'u', 'r', 'i', 'k', 'e', 'n', '-', 's', '0',
      0x01, 8, 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24,
      0x03, 1, 0x00,                       // DS parameter set: channel.
  };
  memcpy(synthetic_frame, base, sizeof(base));

  // Pad with vendor IEs. Never leave a remainder too short for a valid IE.
  uint16_t pad = frame_len - SYNTH_BASE_LEN;
  if (pad < SYNTH_MIN_PAD_IE) {
    pad = 0;
  }
  synthetic_frame_len = static_cast<uint16_t>(SYNTH_BASE_LEN + pad);
  uint16_t pos = SYNTH_BASE_LEN;
  uint8_t fill = 0;
  while (pad > 0) {
    uint16_t body = (pad - 2 > 255) ? 255 : static_cast<uint16_t>(pad - 2);
    const uint16_t leftover = pad - 2 - body;
    if (leftover > 0 && leftover < SYNTH_MIN_PAD_IE) {
      body = static_cast<uint16_t>(body - (SYNTH_MIN_PAD_IE - leftover));
    }
    synthetic_frame[pos++] = 0xdd;
    synthetic_frame[pos++] = static_cast<uint8_t>(body);
    synthetic_frame[pos++] = 0x02;
    synthetic_frame[pos++] = 0x53;
    synthetic_frame[pos++] = 0x48;
    synthetic_frame[pos++] = 0x01;
    for (uint16_t i = 4; i < body; i++) {
      synthetic_frame[pos++] = fill++;
    }
    pad = static_cast<uint16_t>(pad - 2 - body);
  }
}

void emitSyntheticFrame(uint32_t now_ms) {
  uint8_t slot = synthetic_next_slot;
  for (uint8_t i = 0; i < SLOT_COUNT; i++) {
    if (session_slot_mask & (1u << slot)) {
      break;
    }
    slot = static_cast<uint8_t>((slot + 1u) % SLOT_COUNT);
  }
  synthetic_next_slot = static_cast<uint8_t>((slot + 1u) % SLOT_COUNT);

  const SlotConfig& cfg = slot_config[slot];
  if (!(cfg.type_mask & USB_CAPTURE_FILTER_MGMT)) {
    return;
  }

  UsbCapturePacketHeader hdr = {};
  slotCurrentChannel(slot, now_ms, hdr.band, hdr.channel);
  const uint64_t tsf = time_us_64();
  const uint16_t seq_ctrl = static_cast<uint16_t>(synthetic_frame_seq++ << 4);
  synthetic_frame[SYNTH_OFF_SA_LAST] = slot;
  synthetic_frame[SYNTH_OFF_BSSID_LAST] = slot;
  memcpy(&synthetic_frame[SYNTH_OFF_SEQ_CTRL], &seq_ctrl, sizeof(seq_ctrl));
  memcpy(&synthetic_frame[SYNTH_OFF_TSF], &tsf, sizeof(tsf));
  synthetic_frame[SYNTH_OFF_SSID_SLOT] = static_cast<uint8_t>('0' + slot);
  synthetic_frame[SYNTH_OFF_DS_CHANNEL] = hdr.channel;

  hdr.slot = slot;
  hdr.rssi = static_cast<int8_t>(-40 - 5 * slot);
  hdr.noise = -95;
  hdr.flags = USB_CAPTURE_PKT_SYNTHETIC;
  hdr.radio_timestamp_us = static_cast<uint32_t>(tsf);
  hdr.orig_len = synthetic_frame_len;
  hdr.cap_len = synthetic_frame_len;
  if (cfg.snaplen_mgmt != 0 && cfg.snaplen_mgmt < synthetic_frame_len) {
    hdr.cap_len = cfg.snaplen_mgmt;
    hdr.flags |= USB_CAPTURE_PKT_TRUNCATED;
  }
  controllerUsbCaptureQueuePacket(hdr, synthetic_frame);
}

void runSyntheticGenerator() {
  if (!session_active || !synthetic_active || discard_packets_pending) {
    return;
  }
  const uint64_t now_ns = time_us_64() * 1000ull;
  // A stalled loop should not unleash an unbounded burst afterwards.
  if (now_ns > synthetic_next_due_ns + SYNTHETIC_MAX_BACKLOG_NS) {
    synthetic_next_due_ns = now_ns;
  }
  const uint32_t now_ms = nowMs();
  uint16_t emitted = 0;
  while (emitted < USB_CAPTURE_SYNTHETIC_MAX_PER_LOOP && now_ns >= synthetic_next_due_ns) {
    emitSyntheticFrame(now_ms);
    synthetic_next_due_ns += synthetic_period_ns;
    emitted++;
  }
}
#endif

void handleHello(const UsbCaptureRecordHeader& hdr) {
  // A HELLO starts a fresh host session: stop any capture a previous host left
  // running, drop its queued output so the reply is the next thing read, and
  // forget its per-slot channel/hop/filter settings so every session starts
  // from the same defaults.
  stopSession();
  resetTransportLocked();
  resetSlotDefaults();

  UsbCaptureInfo info = {};
  info.proto_version = USB_CAPTURE_PROTO_VERSION;
  info.slot_count = SLOT_COUNT;
  info.max_packet_bytes = USB_CAPTURE_MAX_PACKET_BYTES;
  info.capabilities = USB_CAPTURE_CAP_CHANNEL_HOP | USB_CAPTURE_CAP_RADIO_MONITOR;
#if USB_CAPTURE_SYNTHETIC_ENABLED
  info.capabilities |= USB_CAPTURE_CAP_SYNTHETIC;
#endif
  strncpy(info.firmware_version, APP_VERSION, sizeof(info.firmware_version) - 1);
  sendReply(hdr.type, hdr.seq, USB_CAPTURE_REPLY_OK, &info, sizeof(info));
}

int8_t handleStart(const uint8_t* body, uint16_t len) {
  UsbCaptureStartCommand start = {};
  if (len < sizeof(start)) {
    return USB_CAPTURE_REPLY_INVALID;
  }
  memcpy(&start, body, sizeof(start));
  const uint8_t mask = start.slot_mask & ALL_SLOTS_MASK;
  if (mask == 0) {
    return USB_CAPTURE_REPLY_INVALID;
  }
  if (!(start.flags & USB_CAPTURE_START_SYNTHETIC)) {
#if USB_CAPTURE_SYNTHETIC_ENABLED
    synthetic_active = false;
#endif
    // Restarting drops packets from the previous session at a record boundary.
    discard_packets_pending = true;
    markSessionCounters();
    session_slot_mask = mask;
    session_active = true;
    radio_session = true;
    next_status_ms = nowMs();
    publishRadioRequest(true);
    return USB_CAPTURE_REPLY_OK;
  }
  if (radio_session || !controllerScannerMonitorApplied()) {
    // core1 may still be producing radio packets from the last session.
    return USB_CAPTURE_REPLY_BUSY;
  }
#if !USB_CAPTURE_SYNTHETIC_ENABLED
  return USB_CAPTURE_REPLY_UNSUPPORTED;
#else
  if (start.synthetic_rate_pps == 0) {
    return USB_CAPTURE_REPLY_INVALID;
  }
  uint32_t rate = start.synthetic_rate_pps;
  if (rate > SYNTHETIC_MAX_RATE_PPS) {
    rate = SYNTHETIC_MAX_RATE_PPS;
  }
  uint16_t frame_len = start.synthetic_frame_len;
  if (frame_len < SYNTH_BASE_LEN) {
    frame_len = SYNTH_BASE_LEN;
  }
  if (frame_len > USB_CAPTURE_MAX_PACKET_BYTES) {
    frame_len = USB_CAPTURE_MAX_PACKET_BYTES;
  }
  buildSyntheticFrame(frame_len);
  synthetic_period_ns = 1000000000ull / rate;
  synthetic_next_due_ns = time_us_64() * 1000ull;
  synthetic_next_slot = 0;
  synthetic_active = true;

  // Restarting drops packets from the previous session at a record boundary.
  discard_packets_pending = true;
  markSessionCounters();
  session_slot_mask = mask;
  session_active = true;
  next_status_ms = nowMs();
  return USB_CAPTURE_REPLY_OK;
#endif
}

int8_t handleSetChannel(const uint8_t* body, uint16_t len) {
  UsbCaptureSetChannelCommand cmd = {};
  if (len < sizeof(cmd)) {
    return USB_CAPTURE_REPLY_INVALID;
  }
  memcpy(&cmd, body, sizeof(cmd));
  const uint8_t mask = cmd.slot_mask & ALL_SLOTS_MASK;
  if (mask == 0 || !usbCaptureChannelValid(cmd.band, cmd.channel)) {
    return USB_CAPTURE_REPLY_INVALID;
  }
  for (uint8_t s = 0; s < SLOT_COUNT; s++) {
    if (mask & (1u << s)) {
      slot_config[s].hopping = false;
      slot_config[s].band = cmd.band;
      slot_config[s].channel = cmd.channel;
    }
  }
  if (radio_session) {
    // Retune live; core1 restarts capture on the changed slots.
    publishRadioRequest(true);
  }
  return USB_CAPTURE_REPLY_OK;
}

int8_t handleSetHop(const uint8_t* body, uint16_t len) {
  UsbCaptureSetHopCommand cmd = {};
  if (len < sizeof(cmd)) {
    return USB_CAPTURE_REPLY_INVALID;
  }
  memcpy(&cmd, body, sizeof(cmd));
  const uint8_t mask = cmd.slot_mask & ALL_SLOTS_MASK;
  if (mask == 0 || cmd.count == 0 || cmd.count > USB_CAPTURE_MAX_HOP_CHANNELS ||
      cmd.dwell_ms < HOP_MIN_DWELL_MS ||
      len < sizeof(cmd) + cmd.count * sizeof(UsbCaptureHopEntry)) {
    return USB_CAPTURE_REPLY_INVALID;
  }
  UsbCaptureHopEntry entries[USB_CAPTURE_MAX_HOP_CHANNELS];
  memcpy(entries, body + sizeof(cmd), cmd.count * sizeof(UsbCaptureHopEntry));
  for (uint8_t i = 0; i < cmd.count; i++) {
    if (!usbCaptureChannelValid(entries[i].band, entries[i].channel)) {
      return USB_CAPTURE_REPLY_INVALID;
    }
  }

  const uint8_t slots = slotPopcount(mask);
  for (uint8_t s = 0; s < SLOT_COUNT; s++) {
    if (!(mask & (1u << s))) {
      continue;
    }
    SlotConfig& cfg = slot_config[s];
    cfg.hopping = true;
    cfg.dwell_ms = cmd.dwell_ms;
    cfg.hop_count = cmd.count;
    memcpy(cfg.hop, entries, cmd.count * sizeof(UsbCaptureHopEntry));
    cfg.hop_offset = (cmd.flags & USB_CAPTURE_HOP_STAGGER)
        ? static_cast<uint8_t>((slotRank(mask, s) * cmd.count) / slots)
        : 0;
  }
  if (radio_session) {
    // The scanners take over the new pattern; unchanged slots keep running.
    publishRadioRequest(true);
  }
  return USB_CAPTURE_REPLY_OK;
}

int8_t handleSetFilter(const uint8_t* body, uint16_t len) {
  UsbCaptureSetFilterCommand cmd = {};
  if (len < USB_CAPTURE_SET_FILTER_V1_BYTES) {
    return USB_CAPTURE_REPLY_INVALID;
  }
  memcpy(&cmd, body, len < sizeof(cmd) ? len : sizeof(cmd));
  if (len < sizeof(cmd)) {
    cmd.data_mode = USB_CAPTURE_DATA_MODE_DEFAULT;
  }
  const uint8_t mask = cmd.slot_mask & ALL_SLOTS_MASK;
  if (mask == 0 || cmd.data_mode >= DOT11_DATA_MODE_COUNT) {
    return USB_CAPTURE_REPLY_INVALID;
  }
  for (uint8_t s = 0; s < SLOT_COUNT; s++) {
    if (mask & (1u << s)) {
      slot_config[s].type_mask = cmd.type_mask;
      slot_config[s].snaplen_mgmt = cmd.snaplen_mgmt;
      slot_config[s].snaplen_ctrl = cmd.snaplen_ctrl;
      slot_config[s].snaplen_data = cmd.snaplen_data;
      slot_config[s].data_mode = cmd.data_mode;
    }
  }
  if (radio_session) {
    publishRadioRequest(true);
  }
  return USB_CAPTURE_REPLY_OK;
}

// GET_RADIO_INFO: per slot, what the scanner reported plus the channels it
// accepts, each tagged with the widths it can be used at.
void handleGetRadioInfo(const UsbCaptureRecordHeader& hdr) {
  constexpr uint16_t kMaxChannels = 14 + RADIO_5G_CHANNEL_COUNT;
  constexpr uint16_t kMaxPayload =
      sizeof(UsbCaptureRadioInfoHeader) +
      SLOT_COUNT * (sizeof(UsbCaptureRadioSlot) + kMaxChannels * sizeof(UsbCaptureChannelInfo));
  static uint8_t body[sizeof(UsbCaptureReplyHeader) + kMaxPayload];
  static_assert(sizeof(UsbCaptureRecordHeader) + sizeof(body) <= USB_CAPTURE_CONTROL_RING_BYTES,
                "Radio info reply must fit the control ring");

  uint16_t len = 0;
  UsbCaptureReplyHeader reply = {};
  reply.command = hdr.type;
  reply.status = USB_CAPTURE_REPLY_OK;
  memcpy(body + len, &reply, sizeof(reply));
  len += sizeof(reply);
  UsbCaptureRadioInfoHeader info_header = {};
  info_header.slot_count = SLOT_COUNT;
  memcpy(body + len, &info_header, sizeof(info_header));
  len += sizeof(info_header);

  for (uint8_t s = 0; s < SLOT_COUNT; s++) {
    RadioInfoReply info = {};
    const bool present = controllerScannerRadioInfo(s, info);
    const bool valid = present && (info.flags & RADIO_INFO_VALID);
    UsbCaptureRadioSlot slot = {};
    slot.slot = s;
    slot.flags = (present ? USB_CAPTURE_RADIO_PRESENT : 0) | (valid ? USB_CAPTURE_RADIO_INFO : 0);
    const uint16_t slot_offset = len;
    len += sizeof(slot);
    if (valid) {
      memcpy(slot.country, info.country, sizeof(slot.country));
      // RADIO_WIDTH_* and USB_CAPTURE_WIDTH_* share bit positions.
      slot.widths_24_now = info.widths24_now;
      slot.widths_5_now = info.widths5_now;
      slot.widths_24_capable = info.widths24_capable;
      slot.widths_5_capable = info.widths5_capable;
      slot.protocols_24 = info.protocols24;
      slot.protocols_5 = info.protocols5;
      const bool country = info.flags & RADIO_INFO_COUNTRY_VALID;
      const uint32_t country5 = radioCountry5gMask(info.country_5g_mask);
      if (country) {
        slot.country_schan = info.country_schan;
        slot.country_nchan = info.country_nchan;
        slot.country_policy = info.policy;
        slot.max_tx_power = info.max_tx_power;
        slot.country_5g_mask = country5;
      }
      const RadioChannelSet set = {info.chan24_mask, info.chan5_mask};
      auto add = [&](uint8_t band, uint8_t channel, uint8_t band_widths) {
        // Any accepted channel works at 20 MHz even if the width probe failed.
        const uint8_t widths = radioChannelWidths(set, band, channel, band_widths | RADIO_WIDTH_20);
        if (widths == 0) {
          return;
        }
        UsbCaptureChannelInfo entry = {};
        entry.band = band;
        entry.channel = channel;
        // RADIO_CHAN_WIDTH_* and USB_CAPTURE_CHAN_* share bit positions.
        entry.widths = widths;
        const RadioRegulatory reg =
            country ? radioChannelRegulatory(band, channel, info.country_schan,
                                             info.country_nchan, country5)
                    : RADIO_REG_UNKNOWN;
        if (reg == RADIO_REG_ALLOWED) {
          entry.flags |= USB_CAPTURE_CHAN_REG_ALLOWED;
        } else if (reg == RADIO_REG_UNKNOWN) {
          entry.flags |= USB_CAPTURE_CHAN_REG_UNKNOWN;
        }
        memcpy(body + len, &entry, sizeof(entry));
        len += sizeof(entry);
        slot.channel_count++;
      };
      for (uint8_t ch = 1; ch <= 14; ch++) {
        add(USB_CAPTURE_BAND_24GHZ, ch, info.widths24_capable);
      }
      for (uint8_t i = 0; i < RADIO_5G_CHANNEL_COUNT; i++) {
        add(USB_CAPTURE_BAND_5GHZ, RADIO_5G_CHANNELS[i], info.widths5_capable);
      }
    }
    memcpy(body + slot_offset, &slot, sizeof(slot));
  }
  queueControlRecord(USB_CAPTURE_REC_REPLY, hdr.seq, body, len);
}

// Caller holds __usb_mutex (HELLO resets the TinyUSB FIFO).
void handleCommand(const UsbCaptureRecordHeader& hdr, const uint8_t* body, uint16_t len) {
  last_host_ms = nowMs();
  if (hdr.proto_version != USB_CAPTURE_PROTO_VERSION) {
    sendReply(hdr.type, hdr.seq, USB_CAPTURE_REPLY_UNSUPPORTED);
    return;
  }

  int8_t status = USB_CAPTURE_REPLY_OK;
  switch (hdr.type) {
    case USB_CAPTURE_CMD_HELLO:
      handleHello(hdr);
      return;
    case USB_CAPTURE_CMD_START:
      status = handleStart(body, len);
      break;
    case USB_CAPTURE_CMD_STOP:
      stopSession();
      break;
    case USB_CAPTURE_CMD_KEEPALIVE:
      status = session_active ? USB_CAPTURE_REPLY_OK : USB_CAPTURE_REPLY_NOT_ACTIVE;
      break;
    case USB_CAPTURE_CMD_SET_CHANNEL:
      status = handleSetChannel(body, len);
      break;
    case USB_CAPTURE_CMD_SET_HOP:
      status = handleSetHop(body, len);
      break;
    case USB_CAPTURE_CMD_SET_FILTER:
      status = handleSetFilter(body, len);
      break;
    case USB_CAPTURE_CMD_GET_STATUS:
      sendReply(hdr.type, hdr.seq, USB_CAPTURE_REPLY_OK);
      sendStatus();
      return;
    case USB_CAPTURE_CMD_GET_RADIO_INFO:
      handleGetRadioInfo(hdr);
      return;
    default:
      status = USB_CAPTURE_REPLY_UNKNOWN_COMMAND;
      break;
  }
  sendReply(hdr.type, hdr.seq, status);
}

void dropCommandBytes(uint16_t count) {
  memmove(cmd_buf, cmd_buf + count, cmd_len - count);
  cmd_len = static_cast<uint16_t>(cmd_len - count);
}

// Caller holds __usb_mutex.
void serviceCommandsLocked() {
  while (cmd_len < sizeof(cmd_buf) && tud_vendor_available() > 0) {
    const uint32_t n = tud_vendor_read(cmd_buf + cmd_len, sizeof(cmd_buf) - cmd_len);
    if (n == 0) {
      break;
    }
    cmd_len = static_cast<uint16_t>(cmd_len + n);
  }

  while (cmd_len >= sizeof(UsbCaptureRecordHeader)) {
    UsbCaptureRecordHeader hdr = {};
    memcpy(&hdr, cmd_buf, sizeof(hdr));
    if (hdr.magic != USB_CAPTURE_MAGIC ||
        hdr.length < sizeof(hdr) ||
        hdr.length > USB_CAPTURE_MAX_COMMAND_BYTES) {
      // Not a record start: slide forward one byte and look again.
      dropCommandBytes(1);
      continue;
    }
    if (cmd_len < hdr.length) {
      break;
    }
    handleCommand(hdr, cmd_buf + sizeof(hdr), static_cast<uint16_t>(hdr.length - sizeof(hdr)));
    dropCommandBytes(hdr.length);
  }
}

// Move queued records into the TinyUSB FIFO. Records are streamed one at a
// time so control and packet records never interleave mid-record; control
// records go first so replies are not stuck behind a packet backlog.
// Caller holds __usb_mutex.
void pumpTxLocked() {
  while (true) {
    if (tx_remaining == 0) {
      tx_ring = nullptr;
      if (discard_packets_pending) {
        packet_ring.discardAll();
        discard_packets_pending = false;
      }
      CaptureRing* next = nullptr;
      if (control_ring.used() > 0) {
        next = &control_ring;
      } else if (packet_ring.used() > 0) {
        next = &packet_ring;
      } else {
        break;
      }
      UsbCaptureRecordHeader hdr = {};
      if (next->peek(&hdr, sizeof(hdr)) < sizeof(hdr) ||
          hdr.magic != USB_CAPTURE_MAGIC ||
          hdr.length < sizeof(hdr)) {
        // Producers only write whole records, so this means corruption.
        next->discardAll();
        continue;
      }
      tx_ring = next;
      tx_remaining = hdr.length;
      tx_records++;
    }

    const uint32_t writable = tud_vendor_write_available();
    if (writable == 0) {
      break;
    }
    const uint8_t* span = nullptr;
    uint32_t n = tx_ring->readSpan(&span);
    if (n > tx_remaining) {
      n = tx_remaining;
    }
    if (n > writable) {
      n = writable;
    }
    const uint32_t written = tud_vendor_write(span, n);
    tx_ring->consume(written);
    tx_remaining -= written;
    tx_bytes += written;
    if (written < n || written == 0) {
      break;
    }
  }
  tud_vendor_write_flush();
}

}  // namespace

bool controllerUsbCaptureBegin() {
  packet_ring.init(packet_ring_storage, sizeof(packet_ring_storage));
  control_ring.init(control_ring_storage, sizeof(control_ring_storage));
  resetSlotDefaults();
#if USB_CAPTURE_ENABLED
  return capture_interface.begin();
#else
  return true;
#endif
}

bool controllerUsbCaptureActive() {
  return session_active;
}

bool controllerUsbCaptureQueuePacket(UsbCapturePacketHeader& header, const uint8_t* frame) {
  if (!session_active) {
    return false;
  }
  if (header.cap_len > USB_CAPTURE_MAX_PACKET_BYTES) {
    header.cap_len = USB_CAPTURE_MAX_PACKET_BYTES;
    header.flags |= USB_CAPTURE_PKT_TRUNCATED;
  }
  header.controller_timestamp_us = time_us_64();

  PacketRecordPrefix prefix = {};
  const uint16_t length = static_cast<uint16_t>(sizeof(prefix) + header.cap_len);
  usbCaptureFillHeader(prefix.record, USB_CAPTURE_REC_PACKET, length, packet_seq);
  prefix.packet = header;
  if (!packet_ring.tryWrite(&prefix, sizeof(prefix), frame, header.cap_len)) {
    ring_drop_records++;
    ring_drop_bytes += length;
    return false;
  }
  packet_seq++;
  if (header.slot < SLOT_COUNT) {
    slot_packets[header.slot]++;
  }
  return true;
}

void controllerUsbCaptureTask() {
#if USB_CAPTURE_ENABLED
  const uint32_t now_ms = nowMs();
  if (session_active &&
      (int32_t)(now_ms - last_host_ms) > (int32_t)USB_CAPTURE_KEEPALIVE_TIMEOUT_MS) {
    // The host stopped talking without a STOP; don't capture into the void.
    stopSession();
  }
  if (session_active && (int32_t)(now_ms - next_status_ms) >= 0) {
    sendStatus();
    next_status_ms = now_ms + USB_CAPTURE_STATUS_INTERVAL_MS;
  }

#if USB_CAPTURE_SYNTHETIC_ENABLED
  runSyntheticGenerator();
#endif

  // core1 never touches USB, so this only fails while the USB IRQ task runs.
  if (!mutex_try_enter(&__usb_mutex, nullptr)) {
    return;
  }
  if (!tud_vendor_mounted()) {
    if (was_mounted) {
      // Host went away (unplug, reset, or deconfigure): end the session.
      was_mounted = false;
      stopSession();
      resetTransportLocked();
      cmd_len = 0;
    }
    mutex_exit(&__usb_mutex);
    return;
  }
  was_mounted = true;
  serviceCommandsLocked();
  pumpTxLocked();
  mutex_exit(&__usb_mutex);
#endif
}
