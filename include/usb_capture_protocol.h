#pragma once

#include <stddef.h>
#include <stdint.h>

// Host-facing packet capture protocol carried on the controller's vendor-class
// USB interface. Keep this file as the single source of truth for the USB wire
// format; tools/shuriken_capture.py mirrors these definitions.
//
// Both directions are a byte stream of records. Every record starts with
// UsbCaptureRecordHeader; `length` covers the header and body, so a reader can
// skip record types it does not understand. All fields are little-endian.
//
// Host -> device (bulk OUT): command records (USB_CAPTURE_CMD_*).
// Device -> host (bulk IN): reply, packet, and status records.

static constexpr uint16_t USB_CAPTURE_MAGIC = 0x4853;  // "SH" on the wire.
static constexpr uint8_t USB_CAPTURE_PROTO_VERSION = 1;

// Interface identification so hosts can find the capture interface without
// reading string descriptors.
static constexpr uint8_t USB_CAPTURE_IFACE_SUBCLASS = 0x53;
static constexpr uint8_t USB_CAPTURE_IFACE_PROTOCOL = 0x01;

static constexpr uint8_t USB_CAPTURE_MAX_SLOTS = 8;
static constexpr uint8_t USB_CAPTURE_MAX_HOP_CHANNELS = 64;
static constexpr uint16_t USB_CAPTURE_MAX_PACKET_BYTES = 2400;
static constexpr uint16_t USB_CAPTURE_MAX_COMMAND_BYTES = 256;

enum UsbCaptureRecordType : uint8_t {
  // Host -> device.
  USB_CAPTURE_CMD_HELLO       = 0x01,
  USB_CAPTURE_CMD_START       = 0x02,
  USB_CAPTURE_CMD_STOP        = 0x03,
  USB_CAPTURE_CMD_KEEPALIVE   = 0x04,
  USB_CAPTURE_CMD_SET_CHANNEL = 0x05,
  USB_CAPTURE_CMD_SET_HOP     = 0x06,
  USB_CAPTURE_CMD_SET_FILTER  = 0x07,
  USB_CAPTURE_CMD_GET_STATUS  = 0x08,
  USB_CAPTURE_CMD_GET_RADIO_INFO = 0x09,
  // Device -> host.
  USB_CAPTURE_REC_REPLY       = 0x80,
  USB_CAPTURE_REC_PACKET      = 0x81,
  USB_CAPTURE_REC_STATUS      = 0x82,
};

enum UsbCaptureReplyStatus : int8_t {
  USB_CAPTURE_REPLY_OK = 0,
  USB_CAPTURE_REPLY_UNKNOWN_COMMAND = -1,
  USB_CAPTURE_REPLY_INVALID = -2,
  USB_CAPTURE_REPLY_UNSUPPORTED = -3,
  USB_CAPTURE_REPLY_NOT_ACTIVE = -4,
  // The previous radio session is still being torn down; retry shortly.
  USB_CAPTURE_REPLY_BUSY = -5,
};

// Capability bits reported in UsbCaptureInfo.capabilities.
static constexpr uint32_t USB_CAPTURE_CAP_SYNTHETIC = (1u << 0);
static constexpr uint32_t USB_CAPTURE_CAP_RADIO_MONITOR = (1u << 1);
static constexpr uint32_t USB_CAPTURE_CAP_CHANNEL_HOP = (1u << 2);

// UsbCaptureStartCommand.flags.
static constexpr uint8_t USB_CAPTURE_START_SYNTHETIC = (1u << 0);

// UsbCaptureSetHopCommand.flags: offset each slot's starting hop index so
// radios sharing one pattern cover different channels at the same time.
static constexpr uint8_t USB_CAPTURE_HOP_STAGGER = (1u << 0);

// UsbCaptureSetFilterCommand.type_mask.
static constexpr uint8_t USB_CAPTURE_FILTER_MGMT = (1u << 0);
static constexpr uint8_t USB_CAPTURE_FILTER_CTRL = (1u << 1);
static constexpr uint8_t USB_CAPTURE_FILTER_DATA = (1u << 2);
static constexpr uint8_t USB_CAPTURE_FILTER_FCS_FAIL = (1u << 3);

// UsbCaptureSetFilterCommand.data_mode: how the controller trims data frames.
// Truncated frames keep orig_len and set USB_CAPTURE_PKT_TRUNCATED, so a pcap
// records them as cut short rather than as short frames.
enum UsbCaptureDataMode : uint8_t {
  USB_CAPTURE_DATA_PASS_ALL = 0,        // Every data frame whole.
  USB_CAPTURE_DATA_TRUNCATE_ALL = 1,    // Every data frame cut to its 802.11 header.
  USB_CAPTURE_DATA_PASS_OPEN = 2,       // Unprotected whole, protected cut to the header.
  USB_CAPTURE_DATA_PASS_HANDSHAKE = 3,  // EAPOL (WPA handshakes) whole, the rest cut.
};
static constexpr uint8_t USB_CAPTURE_DATA_MODE_DEFAULT = USB_CAPTURE_DATA_PASS_HANDSHAKE;

// UsbCapturePacketHeader.flags.
static constexpr uint8_t USB_CAPTURE_PKT_FCS_INCLUDED = (1u << 0);
static constexpr uint8_t USB_CAPTURE_PKT_TRUNCATED = (1u << 1);
static constexpr uint8_t USB_CAPTURE_PKT_BAD_FCS = (1u << 2);
static constexpr uint8_t USB_CAPTURE_PKT_SYNTHETIC = (1u << 3);

// UsbCaptureRadioSlot.flags.
static constexpr uint8_t USB_CAPTURE_RADIO_PRESENT = (1u << 0);  // Scanner answered on SPI.
static constexpr uint8_t USB_CAPTURE_RADIO_INFO = (1u << 1);     // Capabilities reported.

// UsbCaptureRadioSlot.widths_*: widths the radio can be set to on that band.
static constexpr uint8_t USB_CAPTURE_WIDTH_20 = (1u << 0);
static constexpr uint8_t USB_CAPTURE_WIDTH_40 = (1u << 1);
static constexpr uint8_t USB_CAPTURE_WIDTH_80 = (1u << 2);
static constexpr uint8_t USB_CAPTURE_WIDTH_160 = (1u << 3);

// UsbCaptureChannelInfo.flags: what the country record says about the channel.
static constexpr uint8_t USB_CAPTURE_CHAN_REG_ALLOWED = (1u << 0);  // Permitted by the record.
static constexpr uint8_t USB_CAPTURE_CHAN_REG_UNKNOWN = (1u << 1);  // Record does not say.

// UsbCaptureRadioSlot.country_policy (wifi_country_policy_t).
static constexpr uint8_t USB_CAPTURE_COUNTRY_POLICY_AUTO = 0;
static constexpr uint8_t USB_CAPTURE_COUNTRY_POLICY_MANUAL = 1;

// UsbCaptureChannelInfo.widths: what this channel can be used as.
static constexpr uint8_t USB_CAPTURE_CHAN_20 = (1u << 0);
static constexpr uint8_t USB_CAPTURE_CHAN_40_ABOVE = (1u << 1);  // HT40+
static constexpr uint8_t USB_CAPTURE_CHAN_40_BELOW = (1u << 2);  // HT40-
static constexpr uint8_t USB_CAPTURE_CHAN_80 = (1u << 3);

// Band values match WIFI_BAND_* in wifi_result_utils.h.
static constexpr uint8_t USB_CAPTURE_BAND_24GHZ = 2;
static constexpr uint8_t USB_CAPTURE_BAND_5GHZ = 5;

struct __attribute__((packed)) UsbCaptureRecordHeader {
  uint16_t magic;
  uint8_t type;
  uint8_t proto_version;
  // Total record length in bytes, including this header.
  uint16_t length;
  // Commands carry a host-chosen sequence number that the reply echoes.
  // Packet and status records use it as a wrapping per-type counter.
  uint16_t seq;
};

// ---- Commands ----

struct __attribute__((packed)) UsbCaptureStartCommand {
  uint8_t slot_mask;
  uint8_t flags;
  // Synthetic generator settings; ignored without USB_CAPTURE_START_SYNTHETIC.
  uint16_t synthetic_frame_len;
  uint32_t synthetic_rate_pps;
};

struct __attribute__((packed)) UsbCaptureSetChannelCommand {
  uint8_t slot_mask;
  uint8_t band;
  uint8_t channel;
  uint8_t reserved;
};

struct __attribute__((packed)) UsbCaptureHopEntry {
  uint8_t band;
  uint8_t channel;
};

// Followed by `count` UsbCaptureHopEntry values.
struct __attribute__((packed)) UsbCaptureSetHopCommand {
  uint8_t slot_mask;
  uint8_t flags;
  uint16_t dwell_ms;
  uint8_t count;
  uint8_t reserved[3];
};

// A snaplen of 0 captures the whole frame; data_mode applies within it. The
// 8-byte form without data_mode (protocol 1 hosts) selects
// USB_CAPTURE_DATA_MODE_DEFAULT.
struct __attribute__((packed)) UsbCaptureSetFilterCommand {
  uint8_t slot_mask;
  uint8_t type_mask;
  uint16_t snaplen_mgmt;
  uint16_t snaplen_ctrl;
  uint16_t snaplen_data;
  uint8_t data_mode;
  uint8_t reserved[3];
};
static constexpr uint16_t USB_CAPTURE_SET_FILTER_V1_BYTES = 8;

// ---- Device records ----

// Followed by a command-specific payload (UsbCaptureInfo for HELLO).
struct __attribute__((packed)) UsbCaptureReplyHeader {
  uint8_t command;
  int8_t status;
  uint16_t reserved;
};

struct __attribute__((packed)) UsbCaptureInfo {
  uint8_t proto_version;
  uint8_t slot_count;
  uint16_t max_packet_bytes;
  uint32_t capabilities;
  char firmware_version[16];
};

// Followed by cap_len bytes of 802.11 frame.
struct __attribute__((packed)) UsbCapturePacketHeader {
  uint8_t slot;
  uint8_t band;
  uint8_t channel;
  int8_t rssi;
  int8_t noise;
  uint8_t rate;
  uint8_t sig_mode;
  uint8_t flags;
  // Radio-local receive timestamp.
  uint32_t radio_timestamp_us;
  // Controller monotonic time when the packet was queued for USB.
  uint64_t controller_timestamp_us;
  uint16_t orig_len;
  uint16_t cap_len;
};

// UsbCaptureSlotStatus.flags.
static constexpr uint8_t USB_CAPTURE_SLOT_ENABLED = (1u << 0);  // In the session slot mask.
static constexpr uint8_t USB_CAPTURE_SLOT_PRESENT = (1u << 1);  // Scanner identified on SPI.
static constexpr uint8_t USB_CAPTURE_SLOT_MONITOR = (1u << 2);  // Radio in monitor mode.
static constexpr uint8_t USB_CAPTURE_SLOT_HOPPING = (1u << 3);  // Following a hop pattern.

struct __attribute__((packed)) UsbCaptureSlotStatus {
  uint8_t flags;
  uint8_t band;
  uint8_t channel;
  uint8_t reserved;
  uint32_t packets;
  uint32_t radio_drops;
  uint32_t transport_errors;
};

// GET_RADIO_INFO reply payload: this header, then slot_count blocks of
// UsbCaptureRadioSlot each followed by its channel_count UsbCaptureChannelInfo.
struct __attribute__((packed)) UsbCaptureRadioInfoHeader {
  uint8_t slot_count;
  uint8_t reserved[3];
};

struct __attribute__((packed)) UsbCaptureRadioSlot {
  uint8_t slot;
  uint8_t flags;
  char country[3];        // ESP-IDF country code ("01" = world-safe default).
  uint8_t channel_count;
  // Widths settable with protocols as configured (802.11ax limits the C5 to
  // 20 MHz) and with 802.11n-only protocols.
  uint8_t widths_24_now;
  uint8_t widths_5_now;
  uint8_t widths_24_capable;
  uint8_t widths_5_capable;
  // WIFI_PROTOCOL_* bits configured per band (11b=1 11g=2 11n=4 11a=16
  // 11ac=32 11ax=64).
  uint16_t protocols_24;
  uint16_t protocols_5;
  // Regulatory data from the radio's ESP-IDF country record.
  uint8_t country_schan;   // 2.4 GHz channels schan .. schan + nchan - 1.
  uint8_t country_nchan;
  uint8_t country_policy;  // USB_CAPTURE_COUNTRY_POLICY_*.
  int8_t max_tx_power;     // Country limit in dBm.
  uint8_t reserved[2];
  // Bit i: RADIO_5G_CHANNELS[i] (36, 40, ... 177) permitted. 0 means the
  // record leaves 5 GHz to the driver's built-in regulatory defaults.
  uint32_t country_5g_mask;
};

// Per-channel widths assume the capable widths and require every channel the
// wider channel spans to be one the radio accepts.
// channels lists every channel the radio accepted when probed, which can be
// wider than the country record allows; flags says which ones it permits.
struct __attribute__((packed)) UsbCaptureChannelInfo {
  uint8_t band;
  uint8_t channel;
  uint8_t widths;
  uint8_t flags;
};

// Followed by slot_count UsbCaptureSlotStatus values.
struct __attribute__((packed)) UsbCaptureStatusHeader {
  uint8_t active;
  uint8_t slot_count;
  uint16_t reserved;
  uint32_t uptime_ms;
  uint32_t tx_records;
  uint32_t tx_bytes;
  uint32_t ring_drop_records;
  uint32_t ring_drop_bytes;
};

static_assert(sizeof(UsbCaptureRecordHeader) == 8, "UsbCaptureRecordHeader must be 8 bytes");
static_assert(sizeof(UsbCaptureStartCommand) == 8, "UsbCaptureStartCommand must be 8 bytes");
static_assert(sizeof(UsbCaptureSetChannelCommand) == 4, "UsbCaptureSetChannelCommand must be 4 bytes");
static_assert(sizeof(UsbCaptureHopEntry) == 2, "UsbCaptureHopEntry must be 2 bytes");
static_assert(sizeof(UsbCaptureSetHopCommand) == 8, "UsbCaptureSetHopCommand must be 8 bytes");
static_assert(sizeof(UsbCaptureSetFilterCommand) == 12, "UsbCaptureSetFilterCommand must be 12 bytes");
static_assert(sizeof(UsbCaptureReplyHeader) == 4, "UsbCaptureReplyHeader must be 4 bytes");
static_assert(sizeof(UsbCaptureInfo) == 24, "UsbCaptureInfo must be 24 bytes");
static_assert(sizeof(UsbCapturePacketHeader) == 24, "UsbCapturePacketHeader must be 24 bytes");
static_assert(sizeof(UsbCaptureSlotStatus) == 16, "UsbCaptureSlotStatus must be 16 bytes");
static_assert(sizeof(UsbCaptureStatusHeader) == 24, "UsbCaptureStatusHeader must be 24 bytes");
static_assert(sizeof(UsbCaptureRadioInfoHeader) == 4, "UsbCaptureRadioInfoHeader must be 4 bytes");
static_assert(sizeof(UsbCaptureRadioSlot) == 24, "UsbCaptureRadioSlot must be 24 bytes");
static_assert(offsetof(UsbCaptureRadioSlot, country_5g_mask) == 20,
              "country_5g_mask must stay 4-byte aligned in the reply");
static_assert(sizeof(UsbCaptureChannelInfo) == 4, "UsbCaptureChannelInfo must be 4 bytes");
static_assert(sizeof(UsbCaptureRecordHeader) + sizeof(UsbCaptureSetHopCommand) +
                  USB_CAPTURE_MAX_HOP_CHANNELS * sizeof(UsbCaptureHopEntry) <=
                  USB_CAPTURE_MAX_COMMAND_BYTES,
              "Largest SET_HOP command must fit the command buffer");

static inline void usbCaptureFillHeader(UsbCaptureRecordHeader& hdr,
                                        uint8_t type,
                                        uint16_t length,
                                        uint16_t seq) {
  hdr.magic = USB_CAPTURE_MAGIC;
  hdr.type = type;
  hdr.proto_version = USB_CAPTURE_PROTO_VERSION;
  hdr.length = length;
  hdr.seq = seq;
}

// Accepts the 802.11 channels the ESP32-C5 radios can tune.
static inline bool usbCaptureChannelValid(uint8_t band, uint8_t channel) {
  if (band == USB_CAPTURE_BAND_24GHZ) {
    return channel >= 1 && channel <= 14;
  }
  if (band == USB_CAPTURE_BAND_5GHZ) {
    return channel >= 32 && channel <= 177;
  }
  return false;
}
