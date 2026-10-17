#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Shared SPI protocol between Pico master and ESP32 slave.
// Keep this file as the single source of truth for on-wire framing.
static constexpr uint8_t PROTO_VERSION = 1;
static constexpr uint8_t SCANNER_TYPE_ESP32_C5 = 1;
static constexpr uint8_t CAP_BAND_24GHZ = 0x01;
static constexpr uint8_t CAP_BAND_5GHZ = 0x02;
static constexpr uint8_t CAP_BAND_6GHZ = 0x04;
// Scanner implements CMD_MONITOR_* (promiscuous capture probe).
static constexpr uint8_t CAP_MONITOR = 0x08;
static constexpr uint8_t CAP_OTA_UPDATE = 0x80;
static constexpr uint8_t PROTO_MAX_RESULTS = 127;
static constexpr uint8_t OTA_DATA_BYTES_PER_FRAME = 48;

// WiFiResult.capabilities bit flags (AuthMode/ciphers/AKM).
#define CAP_ESS              (1u << 0)
#define CAP_WEP              (1u << 1)
#define CAP_WPA              (1u << 2)
#define CAP_WPA2             (1u << 3)
#define CAP_WPA3             (1u << 4)
#define CAP_PSK              (1u << 5)
#define CAP_EAP              (1u << 6)
#define CAP_TKIP             (1u << 7)
#define CAP_CCMP             (1u << 8)

#ifndef LOG_DEDUPE_HITS
#define LOG_DEDUPE_HITS 0
#endif

#ifndef LOG_DEDUPE_HITS_MAX_PER_SCAN
#define LOG_DEDUPE_HITS_MAX_PER_SCAN 8
#endif

enum SpiCommand : uint8_t {
  CMD_NOP          = 0x00,
  CMD_ID           = 0x01,
  CMD_SCAN         = 0x02,
  CMD_RESULT_COUNT = 0x03,
  CMD_RESULT_GET   = 0x04,
  CMD_DEDUPE_RESET = 0x05,
  CMD_OTA_BEGIN    = 0x10,
  CMD_OTA_DATA     = 0x11,
  CMD_OTA_END      = 0x12,
  CMD_OTA_ABORT    = 0x13,
  CMD_MONITOR_START = 0x20,
  CMD_MONITOR_STOP  = 0x21,
  CMD_MONITOR_STATS = 0x22,
  CMD_PKT_READ      = 0x23,
  CMD_PKT_DIAG      = 0x24,
  CMD_MONITOR_HOP   = 0x25,
  CMD_RADIO_INFO    = 0x26,
};

enum ResultType : uint8_t {
  RESULT_WIFI = 0x00,
  RESULT_BUSY = 0xFD,
  RESULT_END  = 0xFF,
};

// On-wire scan/status codes returned by CMD_SCAN and CMD_RESULT_COUNT.
enum ScannerStatus : int8_t {
  SCANNER_STATUS_OK = 0,
  SCANNER_STATUS_BUSY = -1,
  SCANNER_STATUS_INVALID = -2,
  SCANNER_STATUS_START_FAILED = -3,
};

enum OtaStatus : int8_t {
  OTA_STATUS_OK = 0,
  OTA_STATUS_BUSY = -1,
  OTA_STATUS_INVALID = -2,
  OTA_STATUS_NOT_ACTIVE = -3,
  OTA_STATUS_SEQUENCE_MISMATCH = -4,
  OTA_STATUS_CRC_MISMATCH = -5,
  OTA_STATUS_WRITE_FAILED = -6,
  OTA_STATUS_SIZE_MISMATCH = -7,
  OTA_STATUS_HASH_MISMATCH = -8,
  OTA_STATUS_BEGIN_FAILED = -9,
  OTA_STATUS_END_FAILED = -10,
};

static constexpr size_t SPI_FRAME_SIZE = 64;
static constexpr size_t SPI_FRAME_TAG_INDEX = SPI_FRAME_SIZE - 1;

struct __attribute__((packed)) WiFiResult {
  char ssid[33];
  uint8_t bssid[6];
  int8_t rssi;
  uint8_t channel;
  uint8_t band;
  uint16_t capabilities;
};

struct __attribute__((packed)) WiFiResultPacket {
  uint8_t result_type;
  WiFiResult result;
  uint8_t _pad[3];
};

struct __attribute__((packed)) ScanCommand {
  uint8_t cmd;
  uint8_t band;
  uint8_t channel;
  uint8_t reserved;
};

struct __attribute__((packed)) DeviceIdReply {
  uint8_t proto_version;
  uint8_t scanner_type;
  uint8_t capabilities;
  uint8_t max_results;
  uint8_t reserved[4];
};

struct __attribute__((packed)) OtaBeginCommand {
  uint8_t cmd;
  uint8_t packet_data_bytes;
  uint16_t reserved0;
  uint32_t image_size;
  uint64_t image_hash;
  uint8_t reserved1[48];
};

struct __attribute__((packed)) OtaDataCommand {
  uint8_t cmd;
  uint8_t length;
  uint16_t sequence;
  uint32_t offset;
  uint32_t crc32;
  uint8_t data[OTA_DATA_BYTES_PER_FRAME];
  uint8_t reserved[4];
};

struct __attribute__((packed)) OtaEndCommand {
  uint8_t cmd;
  uint8_t reserved0;
  uint16_t reserved1;
  uint32_t image_size;
  uint64_t image_hash;
  uint8_t reserved2[48];
};

struct __attribute__((packed)) OtaAck {
  int8_t status;
  uint8_t active;
  uint16_t next_sequence;
  uint32_t offset;
  uint32_t detail;
  uint64_t image_hash;
};

// Monitor mode filter bits for MonitorStartCommand.type_mask.
static constexpr uint8_t MONITOR_TYPE_MGMT = 0x01;
static constexpr uint8_t MONITOR_TYPE_CTRL = 0x02;
static constexpr uint8_t MONITOR_TYPE_DATA = 0x04;

// MonitorStartCommand.flags: the scanner discards captured frames itself at
// drain_kib_per_s (0 = as fast as possible) instead of holding them for
// CMD_PKT_READ. Used to load-test the radio without the SPI path.
static constexpr uint8_t MONITOR_START_FLAG_SIM_DRAIN = 0x01;
// MonitorStartCommand.flags: ignore the radio and generate records in the
// scanner main loop at drain_kib_per_s, each snaplen_data body bytes long and
// starting with a little-endian uint32 sequence number (from 0 per START), to
// measure the SPI path's capacity and end-to-end loss.
static constexpr uint8_t MONITOR_START_FLAG_SYNTHETIC = 0x02;

// Starts (or retunes) promiscuous capture. Snaplen 0 keeps whole frames.
struct __attribute__((packed)) MonitorStartCommand {
  uint8_t cmd;
  uint8_t band;
  uint8_t channel;
  uint8_t type_mask;
  uint16_t snaplen_mgmt;
  uint16_t snaplen_ctrl;
  uint16_t snaplen_data;
  uint16_t drain_kib_per_s;
  uint8_t flags;
};

// ---------- Scanner-side channel hopping ----------
// CMD_MONITOR_HOP installs a hop list on a scanner already in capture mode.
// The list arrives in chunks (MONITOR_HOP_ENTRIES_PER_FRAME entries per
// frame, consecutive first_index values) and takes effect when the chunk
// holding the last entry lands; total_count 0 turns hopping off. The scanner
// then retunes every dwell_ms on its own, starting at start_index, without
// resetting its capture ring; the first retune happens first_hop_delay_ms
// after the list completes, which lets the controller line every slot up on
// shared hop boundaries. Replies with a ScannerStatus byte.
static constexpr uint8_t MONITOR_HOP_MAX_ENTRIES = 64;
static constexpr uint8_t MONITOR_HOP_ENTRIES_PER_FRAME = 24;
static constexpr uint16_t MONITOR_HOP_MIN_DWELL_MS = 10;

struct __attribute__((packed)) MonitorHopEntry {
  uint8_t band;
  uint8_t channel;
};

struct __attribute__((packed)) MonitorHopCommand {
  uint8_t cmd;
  uint8_t total_count;
  uint16_t dwell_ms;
  uint8_t start_index;
  uint8_t first_index;
  uint8_t entry_count;
  uint8_t reserved;
  MonitorHopEntry entries[MONITOR_HOP_ENTRIES_PER_FRAME];
  uint16_t first_hop_delay_ms;
};

// ---------- Radio capabilities ----------
// CMD_RADIO_INFO reports what the scanner measured at boot: which channels the
// radio accepts on each band and which channel widths it can be set to.

// Channel width bits (RadioInfoReply.widths_*).
static constexpr uint8_t RADIO_WIDTH_20 = 0x01;
static constexpr uint8_t RADIO_WIDTH_40 = 0x02;
static constexpr uint8_t RADIO_WIDTH_80 = 0x04;
static constexpr uint8_t RADIO_WIDTH_160 = 0x08;

// RadioInfoReply.flags.
static constexpr uint8_t RADIO_INFO_VALID = 0x01;          // Probe ran; fields are meaningful.
static constexpr uint8_t RADIO_INFO_COUNTRY_VALID = 0x02;  // esp_wifi_get_country succeeded.

// 5 GHz channel for each bit of RadioInfoReply.chan5_mask (bit i), matching
// ESP-IDF's wifi_5g_channel_bit_t (which uses bit i + 1).
static constexpr uint8_t RADIO_5G_CHANNEL_COUNT = 28;
static constexpr uint8_t RADIO_5G_CHANNELS[RADIO_5G_CHANNEL_COUNT] = {
    36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120,
    124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165, 169, 173, 177};

struct __attribute__((packed)) RadioInfoReply {
  int8_t status;
  uint8_t flags;
  char country[3];        // ESP-IDF country code, e.g. "01" (world) plus env char.
  uint8_t policy;         // wifi_country_policy_t: 0 auto, 1 manual.
  int8_t max_tx_power;    // Country limit in dBm (wifi_country_t; not the 0.25 dBm
                          // units esp_wifi_set_max_tx_power takes).
  uint8_t country_schan;  // 2.4 GHz range from the country record.
  uint8_t country_nchan;
  uint16_t chan24_mask;   // Bit n set: 2.4 GHz channel n accepted (1..14).
  uint32_t chan5_mask;    // Bit i set: RADIO_5G_CHANNELS[i] accepted.
  uint32_t country_5g_mask;  // Raw wifi_5g_channel_mask (0: regulatory default).
  uint8_t widths24_now;      // Widths settable with the protocols as configured.
  uint8_t widths5_now;
  uint8_t widths24_capable;  // Widths settable with 11n-only protocols.
  uint8_t widths5_capable;
  uint16_t protocols24;      // WIFI_PROTOCOL_* bits as configured.
  uint16_t protocols5;
  uint8_t channel_errors;    // Channels the driver rejected while probing.
};

// ---------- Captured packet batches ----------
// CMD_PKT_READ moves captured frames in one long SPI transaction:
//   1. Master sends CMD_PKT_READ (64-byte frame), acknowledging the previous
//      batch by sequence number.
//   2. Master pulls a NOP; the reply frame (tag CMD_PKT_READ) is a
//      PktReadReply giving the batch sequence and wire length.
//   3. Master clocks exactly wire_len bytes: PktBatchHeader + records.
// The scanner only releases a batch from its ring when a later CMD_PKT_READ
// acknowledges that sequence, so a corrupt or missed batch is resent with
// the same sequence and the master drops duplicates. A 64-byte transaction
// arriving while the scanner waits for the bulk read is handled as an
// ordinary command, which is how the master abandons a batch.
static constexpr size_t PKT_BATCH_MAX_BYTES = 4096;
static constexpr uint16_t PKT_BATCH_MAGIC = 0x4250;  // "PB" on the wire.
static constexpr uint8_t PKT_READ_FLAG_ACK = 0x01;

// Record flags.
static constexpr uint8_t MONITOR_PKT_FLAG_FCS_INCLUDED = 0x01;
static constexpr uint8_t MONITOR_PKT_FLAG_TRUNCATED = 0x02;
static constexpr uint8_t MONITOR_PKT_FLAG_SYNTHETIC = 0x04;

struct __attribute__((packed)) PktReadCommand {
  uint8_t cmd;
  uint8_t flags;
  uint16_t ack_seq;
};

struct __attribute__((packed)) PktReadReply {
  int8_t status;
  // Channel the radio is tuned to right now (it may be hopping).
  uint8_t channel;
  uint16_t seq;
  // Bytes the master must clock for the batch; 0 when nothing is queued.
  uint16_t wire_len;
  uint16_t records;
  uint32_t record_bytes;
  // Ring bytes still waiting behind this batch.
  uint32_t pending_bytes;
  uint32_t ring_drops;
  uint32_t crc32;
};

// CMD_PKT_DIAG reply: how the scanner classified completed transactions while
// waiting for a bulk read, with the raw received bit counts.
struct __attribute__((packed)) PktDiagReply {
  int8_t status;
  uint8_t bulk_state;
  uint16_t wire_len;
  uint32_t bulk_full;
  uint32_t bulk_command;
  uint32_t bulk_partial;
  uint32_t header_abandoned;
  uint32_t last_full_bits;
  uint32_t last_command_bits;
  uint32_t last_partial_bits;
  uint32_t batches_built;
};

struct __attribute__((packed)) PktBatchHeader {
  uint16_t magic;
  uint16_t seq;
  uint16_t record_bytes;
  uint16_t records;
  uint32_t crc32;  // pktCrc32() over the record bytes.
};

// One captured frame inside a batch, followed by cap_len frame bytes where
// cap_len = record_len - sizeof(MonitorPacketRecord).
struct __attribute__((packed)) MonitorPacketRecord {
  uint16_t record_len;
  uint16_t orig_len;
  uint32_t timestamp_us;
  int8_t rssi;
  int8_t noise;
  uint8_t channel;
  uint8_t band;
  uint8_t rate;
  uint8_t sig_mode;
  uint8_t flags;
  uint8_t type;
};

// Longest frame body that fits in one batch.
static constexpr uint16_t PKT_RECORD_MAX_CAP =
    PKT_BATCH_MAX_BYTES - sizeof(PktBatchHeader) - sizeof(MonitorPacketRecord);

// Shortest bulk read. The scanner classifies a completed bulk transaction by
// its bit count, which the ESP32-C5 driver reports with a few bits of slop
// (a 64-byte frame reads back as 520 bits), so a batch must be far longer
// than one command frame to be told apart from the master abandoning it.
static constexpr uint16_t PKT_BATCH_MIN_WIRE_BYTES = 2 * SPI_FRAME_SIZE;

// Bulk length on the wire: word aligned for DMA and at least
// PKT_BATCH_MIN_WIRE_BYTES.
static inline uint16_t pktBatchWireLen(uint16_t record_bytes) {
  uint32_t len = sizeof(PktBatchHeader) + record_bytes;
  len = (len + 3u) & ~3u;
  if (len < PKT_BATCH_MIN_WIRE_BYTES) {
    len = PKT_BATCH_MIN_WIRE_BYTES;
  }
  return static_cast<uint16_t>(len);
}

// Walk the records of a batch, checking every length. Returns false if the
// records do not exactly tile record_bytes or the count disagrees.
static inline bool pktBatchRecordsValid(const uint8_t* records, uint16_t record_bytes,
                                        uint16_t expected_records) {
  uint32_t offset = 0;
  uint16_t count = 0;
  while (offset < record_bytes) {
    if (record_bytes - offset < sizeof(MonitorPacketRecord)) {
      return false;
    }
    MonitorPacketRecord rec;
    memcpy(&rec, records + offset, sizeof(rec));
    if (rec.record_len < sizeof(MonitorPacketRecord) || rec.record_len > record_bytes - offset) {
      return false;
    }
    if (rec.record_len - sizeof(MonitorPacketRecord) > rec.orig_len) {
      return false;
    }
    offset += rec.record_len;
    count++;
  }
  return count == expected_records;
}

enum MonitorStartStage : uint8_t {
  MONITOR_STAGE_NONE = 0,
  MONITOR_STAGE_ALLOC = 1,
  MONITOR_STAGE_WIFI_START = 2,
  MONITOR_STAGE_PROMISCUOUS = 3,
  MONITOR_STAGE_CHANNEL = 4,
};

// Cumulative counters since CMD_MONITOR_START; the reader computes rates.
// cb_max_us and ring_high_water reset each time stats are read.
struct __attribute__((packed)) MonitorStatsReply {
  int8_t status;
  uint8_t active;
  uint8_t channel;
  uint8_t cpu_mhz;
  uint32_t packets_mgmt;
  uint32_t packets_ctrl;
  uint32_t packets_data;
  uint32_t air_bytes;
  uint32_t copied_bytes;
  uint32_t ring_drops;
  uint32_t cb_cycles;
  uint16_t cb_max_us;
  uint16_t ring_kib;
  uint32_t ring_high_water;
  uint32_t free_heap;
  uint32_t uptime_ms;
  // Last CMD_MONITOR_START failure: MonitorStartStage and its esp_err_t.
  uint8_t error_stage;
  // Channel the driver reports (esp_wifi_get_channel) and the channel field
  // of the most recent captured frame, to confirm the radio actually retuned.
  uint8_t radio_channel;
  int32_t error_code;
  uint8_t last_frame_channel;
  // Channel changes made by hopping since START, and the slowest one.
  uint16_t hops;
  uint16_t hop_switch_max_us;
};

static constexpr uint32_t OTA_CRC32_INIT = 0xffffffffUL;
static constexpr uint64_t OTA_HASH64_INIT = 14695981039346656037ULL;
static constexpr uint64_t OTA_HASH64_PRIME = 1099511628211ULL;

static inline uint32_t otaCrc32Update(uint32_t crc, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 1U) {
        crc = (crc >> 1) ^ 0xedb88320UL;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static inline uint32_t otaCrc32(const uint8_t* data, size_t len) {
  return ~otaCrc32Update(OTA_CRC32_INIT, data, len);
}

static inline uint64_t otaHash64Update(uint64_t hash, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    hash ^= data[i];
    hash *= OTA_HASH64_PRIME;
  }
  return hash;
}

// Table-driven CRC-32 (IEEE, reflected) for packet batches; the bitwise OTA
// helper above is too slow for multi-kilobyte batches on the scanner.
struct PktCrc32Table {
  uint32_t entry[256];
};

static constexpr PktCrc32Table makePktCrc32Table() {
  PktCrc32Table table = {};
  for (uint32_t i = 0; i < 256; i++) {
    uint32_t crc = i;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 1U) ? (crc >> 1) ^ 0xedb88320UL : crc >> 1;
    }
    table.entry[i] = crc;
  }
  return table;
}

static constexpr PktCrc32Table PKT_CRC32_TABLE = makePktCrc32Table();

static inline uint32_t pktCrc32Update(uint32_t crc, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    crc = PKT_CRC32_TABLE.entry[(crc ^ data[i]) & 0xffU] ^ (crc >> 8);
  }
  return crc;
}

static inline uint32_t pktCrc32(const uint8_t* data, size_t len) {
  return ~pktCrc32Update(0xffffffffUL, data, len);
}

static_assert(sizeof(WiFiResultPacket) == 48, "WiFiResultPacket must be 48 bytes");
static_assert(sizeof(DeviceIdReply) == 8, "DeviceIdReply must be 8 bytes");
static_assert(sizeof(OtaBeginCommand) == SPI_FRAME_SIZE, "OtaBeginCommand must be one SPI frame");
static_assert(sizeof(OtaDataCommand) == SPI_FRAME_SIZE, "OtaDataCommand must be one SPI frame");
static_assert(sizeof(OtaEndCommand) == SPI_FRAME_SIZE, "OtaEndCommand must be one SPI frame");
static_assert(sizeof(OtaAck) == 20, "OtaAck must be 20 bytes");
static_assert(sizeof(MonitorStartCommand) == 13, "MonitorStartCommand must be 13 bytes");
static_assert(sizeof(PktReadReply) <= SPI_FRAME_TAG_INDEX, "PktReadReply must fit before the tag byte");
static_assert(sizeof(PktBatchHeader) == 12, "PktBatchHeader must be 12 bytes");
static_assert(sizeof(PktDiagReply) <= SPI_FRAME_TAG_INDEX, "PktDiagReply must fit before the tag byte");
static_assert(sizeof(MonitorPacketRecord) == 16, "MonitorPacketRecord must be 16 bytes");
static_assert(PKT_BATCH_MAX_BYTES % 4 == 0, "Batches must stay word aligned for DMA");
static_assert(sizeof(MonitorStatsReply) <= SPI_FRAME_TAG_INDEX, "MonitorStatsReply must fit before the tag byte");
static_assert(sizeof(MonitorHopCommand) <= SPI_FRAME_TAG_INDEX, "MonitorHopCommand must fit before the tag byte");
static_assert(sizeof(RadioInfoReply) <= SPI_FRAME_TAG_INDEX, "RadioInfoReply must fit before the tag byte");
