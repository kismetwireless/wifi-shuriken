#pragma once

#include <stdint.h>

// Minimal 802.11 frame classification and the capture filter built on it.
// Pure logic shared by the controller (applied to every captured frame on its
// way to USB) and the native tests.

enum Dot11FrameClass : uint8_t {
  DOT11_CLASS_MALFORMED = 0,  // Too short to hold its own header.
  DOT11_CLASS_MGMT,
  DOT11_CLASS_CTRL,
  DOT11_CLASS_EXTENSION,      // Type 3 (DMG beacon, S1G): passed like mgmt.
  DOT11_CLASS_DATA_NULL,      // Data subtypes that carry no body.
  DOT11_CLASS_DATA_OPEN,      // Unprotected data with a body.
  DOT11_CLASS_DATA_PROTECTED, // Protected (encrypted) data.
  DOT11_CLASS_DATA_EAPOL,     // Unprotected data carrying 802.1X (WPA handshakes).
  DOT11_CLASS_COUNT,
};

// What to do with data frames.
enum Dot11DataMode : uint8_t {
  DOT11_DATA_PASS_ALL = 0,       // Every data frame whole.
  DOT11_DATA_TRUNCATE_ALL = 1,   // Every data frame cut to its 802.11 header.
  DOT11_DATA_PASS_OPEN = 2,      // Unprotected whole; protected cut to the header.
  DOT11_DATA_PASS_HANDSHAKE = 3, // EAPOL whole; everything else cut to the header.
  DOT11_DATA_MODE_COUNT,
};

enum Dot11FilterAction : uint8_t {
  DOT11_ACTION_PASS = 0,
  DOT11_ACTION_TRUNCATE,
  DOT11_ACTION_DROP,
};

// Frame type bits, matching MONITOR_TYPE_* and USB_CAPTURE_FILTER_*.
static constexpr uint8_t DOT11_TYPE_MGMT = 0x01;
static constexpr uint8_t DOT11_TYPE_CTRL = 0x02;
static constexpr uint8_t DOT11_TYPE_DATA = 0x04;

// Longest data frame header: 3-address header (24) + addr4 (6) + QoS (2) +
// HT control (4).
static constexpr uint16_t DOT11_MAX_DATA_HEADER = 36;
// Bytes needed to tell EAPOL from other unprotected data: the longest header
// plus an LLC/SNAP header.
static constexpr uint16_t DOT11_CLASSIFY_BYTES = DOT11_MAX_DATA_HEADER + 8;

struct Dot11Info {
  uint8_t cls;         // Dot11FrameClass.
  uint8_t header_len;  // MAC header length (data frames; 0 otherwise).
};

static inline Dot11Info dot11Classify(const uint8_t* frame, uint16_t len) {
  Dot11Info info = {DOT11_CLASS_MALFORMED, 0};
  if (len < 2) {
    return info;
  }
  const uint8_t fc0 = frame[0];
  const uint8_t fc1 = frame[1];
  const uint8_t type = (fc0 >> 2) & 0x3;
  const uint8_t subtype = fc0 >> 4;
  if (type == 0) {
    info.cls = DOT11_CLASS_MGMT;
    return info;
  }
  if (type == 1) {
    info.cls = DOT11_CLASS_CTRL;
    return info;
  }
  if (type == 3) {
    info.cls = DOT11_CLASS_EXTENSION;
    return info;
  }

  // Data. Header: 24 bytes, +6 with both DS bits (WDS/mesh), +2 for QoS
  // subtypes, +4 HT control when a QoS frame sets the order bit.
  const bool qos = (subtype & 0x8) != 0;
  uint16_t hdr = 24;
  if ((fc1 & 0x03) == 0x03) {
    hdr += 6;
  }
  if (qos) {
    hdr += 2;
    if (fc1 & 0x80) {
      hdr += 4;
    }
  }
  if (len < hdr) {
    return info;
  }
  info.header_len = static_cast<uint8_t>(hdr);
  if (subtype & 0x4) {
    // Null, CF-Ack/Poll without data, QoS Null.
    info.cls = DOT11_CLASS_DATA_NULL;
    return info;
  }
  if (fc1 & 0x40) {
    info.cls = DOT11_CLASS_DATA_PROTECTED;
    return info;
  }
  info.cls = DOT11_CLASS_DATA_OPEN;
  // An A-MSDU body starts with subframe headers, not LLC.
  const bool amsdu = qos && (frame[hdr - (fc1 & 0x80 ? 6 : 2)] & 0x80);
  if (!amsdu && len >= hdr + 8) {
    const uint8_t* llc = frame + hdr;
    if (llc[0] == 0xAA && llc[1] == 0xAA && llc[2] == 0x03 && llc[3] == 0x00 &&
        llc[4] == 0x00 && llc[5] == 0x00 && llc[6] == 0x88 && llc[7] == 0x8E) {
      info.cls = DOT11_CLASS_DATA_EAPOL;
    }
  }
  return info;
}

struct Dot11FilterResult {
  uint8_t cls;     // Dot11FrameClass.
  uint8_t action;  // Dot11FilterAction.
  uint16_t cap_len;
};

// type_mask: DOT11_TYPE_* allowed through (extension frames ride with mgmt).
// Frames too short to classify are passed unchanged.
static inline Dot11FilterResult dot11Filter(const uint8_t* frame, uint16_t cap_len,
                                            uint8_t type_mask, uint8_t data_mode) {
  const Dot11Info info = dot11Classify(frame, cap_len);
  Dot11FilterResult result = {info.cls, DOT11_ACTION_PASS, cap_len};
  uint8_t type_bit = 0;
  bool cut = false;
  switch (info.cls) {
    case DOT11_CLASS_MALFORMED:
      return result;
    case DOT11_CLASS_MGMT:
    case DOT11_CLASS_EXTENSION:
      type_bit = DOT11_TYPE_MGMT;
      break;
    case DOT11_CLASS_CTRL:
      type_bit = DOT11_TYPE_CTRL;
      break;
    case DOT11_CLASS_DATA_NULL:
      type_bit = DOT11_TYPE_DATA;
      break;
    case DOT11_CLASS_DATA_OPEN:
      type_bit = DOT11_TYPE_DATA;
      cut = data_mode == DOT11_DATA_TRUNCATE_ALL || data_mode == DOT11_DATA_PASS_HANDSHAKE;
      break;
    case DOT11_CLASS_DATA_PROTECTED:
      type_bit = DOT11_TYPE_DATA;
      cut = data_mode != DOT11_DATA_PASS_ALL;
      break;
    case DOT11_CLASS_DATA_EAPOL:
      type_bit = DOT11_TYPE_DATA;
      cut = data_mode == DOT11_DATA_TRUNCATE_ALL;
      break;
    default:
      return result;
  }
  if (!(type_mask & type_bit)) {
    result.action = DOT11_ACTION_DROP;
    result.cap_len = 0;
    return result;
  }
  if (cut && cap_len > info.header_len) {
    result.action = DOT11_ACTION_TRUNCATE;
    result.cap_len = info.header_len;
  }
  return result;
}

// Scanner-side data snaplen that still gives the controller what the data
// mode needs. host_snaplen 0 means whole frames.
static inline uint16_t dot11ScannerDataSnaplen(uint8_t data_mode, uint16_t host_snaplen) {
  if (data_mode != DOT11_DATA_TRUNCATE_ALL) {
    return host_snaplen;
  }
  // Every data frame ends up as its header: ship no more than that.
  return (host_snaplen != 0 && host_snaplen < DOT11_MAX_DATA_HEADER) ? host_snaplen
                                                                     : DOT11_MAX_DATA_HEADER;
}
