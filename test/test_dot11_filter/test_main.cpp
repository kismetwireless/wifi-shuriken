#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "dot11_filter.h"
#include "spi_protocol_shared.h"
#include "usb_capture_protocol.h"

void setUp() {}
void tearDown() {}

static const uint8_t ALL_TYPES = DOT11_TYPE_MGMT | DOT11_TYPE_CTRL | DOT11_TYPE_DATA;

// Build a data frame: fc0/fc1, the header that implies, then an optional
// LLC/SNAP header with the given ethertype and body_len bytes in total.
static uint16_t buildData(uint8_t* buf, uint8_t fc0, uint8_t fc1, uint16_t ethertype,
                          uint16_t body_len) {
  memset(buf, 0x5a, 1600);
  buf[0] = fc0;
  buf[1] = fc1;
  const Dot11Info info = dot11Classify(buf, 1600);
  const uint16_t hdr = info.header_len;
  if (fc0 & 0x80) {
    // QoS control: clear the A-MSDU bit unless a test sets it.
    buf[hdr - ((fc1 & 0x80) ? 6 : 2)] = 0x00;
  }
  if (ethertype != 0 && body_len >= 8) {
    const uint8_t llc[8] = {0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00,
                            static_cast<uint8_t>(ethertype >> 8),
                            static_cast<uint8_t>(ethertype)};
    memcpy(buf + hdr, llc, sizeof(llc));
  }
  return static_cast<uint16_t>(hdr + body_len);
}

void test_type_constants_match_protocols() {
  TEST_ASSERT_EQUAL_HEX8(MONITOR_TYPE_MGMT, DOT11_TYPE_MGMT);
  TEST_ASSERT_EQUAL_HEX8(MONITOR_TYPE_CTRL, DOT11_TYPE_CTRL);
  TEST_ASSERT_EQUAL_HEX8(MONITOR_TYPE_DATA, DOT11_TYPE_DATA);
  TEST_ASSERT_EQUAL_HEX8(USB_CAPTURE_FILTER_CTRL, DOT11_TYPE_CTRL);
  TEST_ASSERT_EQUAL_UINT8(USB_CAPTURE_DATA_PASS_HANDSHAKE, DOT11_DATA_PASS_HANDSHAKE);
}

void test_classify_mgmt_ctrl_extension() {
  const uint8_t beacon[24] = {0x80, 0x00};
  const uint8_t ack[10] = {0xd4, 0x00};
  const uint8_t rts[16] = {0xb4, 0x00};
  const uint8_t ext[16] = {0x0c, 0x00};
  TEST_ASSERT_EQUAL_UINT8(DOT11_CLASS_MGMT, dot11Classify(beacon, sizeof(beacon)).cls);
  TEST_ASSERT_EQUAL_UINT8(DOT11_CLASS_CTRL, dot11Classify(ack, sizeof(ack)).cls);
  TEST_ASSERT_EQUAL_UINT8(DOT11_CLASS_CTRL, dot11Classify(rts, sizeof(rts)).cls);
  TEST_ASSERT_EQUAL_UINT8(DOT11_CLASS_EXTENSION, dot11Classify(ext, sizeof(ext)).cls);
  TEST_ASSERT_EQUAL_UINT8(DOT11_CLASS_MALFORMED, dot11Classify(beacon, 1).cls);
}

void test_data_header_lengths() {
  uint8_t buf[1600];
  buildData(buf, 0x08, 0x01, 0, 0);
  TEST_ASSERT_EQUAL_UINT8(24, dot11Classify(buf, 100).header_len);
  buildData(buf, 0x08, 0x03, 0, 0);  // WDS: addr4.
  TEST_ASSERT_EQUAL_UINT8(30, dot11Classify(buf, 100).header_len);
  buildData(buf, 0x88, 0x02, 0, 0);  // QoS.
  TEST_ASSERT_EQUAL_UINT8(26, dot11Classify(buf, 100).header_len);
  buildData(buf, 0x88, 0x82, 0, 0);  // QoS + HT control.
  TEST_ASSERT_EQUAL_UINT8(30, dot11Classify(buf, 100).header_len);
  buildData(buf, 0x88, 0x83, 0, 0);  // QoS + addr4 + HT control.
  TEST_ASSERT_EQUAL_UINT8(DOT11_MAX_DATA_HEADER, dot11Classify(buf, 100).header_len);
  buildData(buf, 0x08, 0x80, 0, 0);  // Order bit without QoS: no HT control.
  TEST_ASSERT_EQUAL_UINT8(24, dot11Classify(buf, 100).header_len);
}

void test_classify_data_kinds() {
  uint8_t buf[1600];
  uint16_t len = buildData(buf, 0x88, 0x01, 0x0800, 200);
  TEST_ASSERT_EQUAL_UINT8(DOT11_CLASS_DATA_OPEN, dot11Classify(buf, len).cls);
  len = buildData(buf, 0x88, 0x41, 0, 200);
  TEST_ASSERT_EQUAL_UINT8(DOT11_CLASS_DATA_PROTECTED, dot11Classify(buf, len).cls);
  len = buildData(buf, 0x88, 0x02, 0x888E, 121);
  TEST_ASSERT_EQUAL_UINT8(DOT11_CLASS_DATA_EAPOL, dot11Classify(buf, len).cls);
  len = buildData(buf, 0x08, 0x01, 0x888E, 99);
  TEST_ASSERT_EQUAL_UINT8(DOT11_CLASS_DATA_EAPOL, dot11Classify(buf, len).cls);
  len = buildData(buf, 0x48, 0x11, 0, 0);  // Null function.
  TEST_ASSERT_EQUAL_UINT8(DOT11_CLASS_DATA_NULL, dot11Classify(buf, len).cls);
  len = buildData(buf, 0xc8, 0x11, 0, 0);  // QoS Null.
  TEST_ASSERT_EQUAL_UINT8(DOT11_CLASS_DATA_NULL, dot11Classify(buf, len).cls);
  TEST_ASSERT_EQUAL_UINT8(26, dot11Classify(buf, len).header_len);
}

void test_eapol_needs_full_llc_and_no_amsdu() {
  uint8_t buf[1600];
  uint16_t len = buildData(buf, 0x88, 0x02, 0x888E, 121);
  // Cut inside the SNAP header: just open data.
  TEST_ASSERT_EQUAL_UINT8(DOT11_CLASS_DATA_OPEN, dot11Classify(buf, 26 + 7).cls);
  // A-MSDU bit set in QoS control: the body is not LLC.
  buf[24] = 0x80;
  TEST_ASSERT_EQUAL_UINT8(DOT11_CLASS_DATA_OPEN, dot11Classify(buf, len).cls);
  // A protected frame is never classed as EAPOL, even with a matching body.
  len = buildData(buf, 0x88, 0x42, 0x888E, 121);
  TEST_ASSERT_EQUAL_UINT8(DOT11_CLASS_DATA_PROTECTED, dot11Classify(buf, len).cls);
}

void test_short_data_is_malformed() {
  uint8_t buf[1600];
  buildData(buf, 0x88, 0x01, 0, 0);
  TEST_ASSERT_EQUAL_UINT8(DOT11_CLASS_MALFORMED, dot11Classify(buf, 25).cls);
  const Dot11FilterResult r = dot11Filter(buf, 25, ALL_TYPES, DOT11_DATA_TRUNCATE_ALL);
  TEST_ASSERT_EQUAL_UINT8(DOT11_ACTION_PASS, r.action);
  TEST_ASSERT_EQUAL_UINT16(25, r.cap_len);
}

void test_ctrl_filtered_by_type_mask() {
  const uint8_t ack[10] = {0xd4, 0x00};
  Dot11FilterResult r = dot11Filter(ack, sizeof(ack), DOT11_TYPE_MGMT | DOT11_TYPE_DATA,
                                    DOT11_DATA_PASS_ALL);
  TEST_ASSERT_EQUAL_UINT8(DOT11_ACTION_DROP, r.action);
  TEST_ASSERT_EQUAL_UINT16(0, r.cap_len);
  r = dot11Filter(ack, sizeof(ack), ALL_TYPES, DOT11_DATA_PASS_ALL);
  TEST_ASSERT_EQUAL_UINT8(DOT11_ACTION_PASS, r.action);
  TEST_ASSERT_EQUAL_UINT16(sizeof(ack), r.cap_len);
}

// Expected cap length for each class under each data mode.
struct ModeCase {
  uint8_t mode;
  bool open_whole;
  bool protected_whole;
  bool eapol_whole;
};

void test_data_modes() {
  static const ModeCase cases[] = {
      {DOT11_DATA_PASS_ALL, true, true, true},
      {DOT11_DATA_TRUNCATE_ALL, false, false, false},
      {DOT11_DATA_PASS_OPEN, true, false, true},
      {DOT11_DATA_PASS_HANDSHAKE, false, false, true},
  };
  uint8_t open[1600], prot[1600], eapol[1600], null_data[1600];
  const uint16_t open_len = buildData(open, 0x88, 0x01, 0x0800, 1400);
  const uint16_t prot_len = buildData(prot, 0x88, 0x41, 0, 1400);
  const uint16_t eapol_len = buildData(eapol, 0x88, 0x02, 0x888E, 121);
  const uint16_t null_len = buildData(null_data, 0xc8, 0x11, 0, 0);
  for (const ModeCase& c : cases) {
    Dot11FilterResult r = dot11Filter(open, open_len, ALL_TYPES, c.mode);
    TEST_ASSERT_EQUAL_UINT16(c.open_whole ? open_len : 26, r.cap_len);
    TEST_ASSERT_EQUAL_UINT8(c.open_whole ? DOT11_ACTION_PASS : DOT11_ACTION_TRUNCATE, r.action);
    r = dot11Filter(prot, prot_len, ALL_TYPES, c.mode);
    TEST_ASSERT_EQUAL_UINT16(c.protected_whole ? prot_len : 26, r.cap_len);
    r = dot11Filter(eapol, eapol_len, ALL_TYPES, c.mode);
    TEST_ASSERT_EQUAL_UINT16(c.eapol_whole ? eapol_len : 26, r.cap_len);
    r = dot11Filter(null_data, null_len, ALL_TYPES, c.mode);
    TEST_ASSERT_EQUAL_UINT8(DOT11_ACTION_PASS, r.action);
    TEST_ASSERT_EQUAL_UINT16(null_len, r.cap_len);
  }
}

void test_data_dropped_without_data_type() {
  uint8_t buf[1600];
  const uint16_t len = buildData(buf, 0x88, 0x02, 0x888E, 121);
  const Dot11FilterResult r = dot11Filter(buf, len, DOT11_TYPE_MGMT, DOT11_DATA_PASS_ALL);
  TEST_ASSERT_EQUAL_UINT8(DOT11_ACTION_DROP, r.action);
}

void test_mgmt_never_truncated() {
  uint8_t beacon[300] = {0x80, 0x00};
  const Dot11FilterResult r =
      dot11Filter(beacon, sizeof(beacon), ALL_TYPES, DOT11_DATA_TRUNCATE_ALL);
  TEST_ASSERT_EQUAL_UINT8(DOT11_ACTION_PASS, r.action);
  TEST_ASSERT_EQUAL_UINT16(sizeof(beacon), r.cap_len);
}

void test_scanner_snaplen_pushdown() {
  TEST_ASSERT_EQUAL_UINT16(0, dot11ScannerDataSnaplen(DOT11_DATA_PASS_ALL, 0));
  TEST_ASSERT_EQUAL_UINT16(256, dot11ScannerDataSnaplen(DOT11_DATA_PASS_HANDSHAKE, 256));
  TEST_ASSERT_EQUAL_UINT16(0, dot11ScannerDataSnaplen(DOT11_DATA_PASS_OPEN, 0));
  TEST_ASSERT_EQUAL_UINT16(DOT11_MAX_DATA_HEADER,
                           dot11ScannerDataSnaplen(DOT11_DATA_TRUNCATE_ALL, 0));
  TEST_ASSERT_EQUAL_UINT16(DOT11_MAX_DATA_HEADER,
                           dot11ScannerDataSnaplen(DOT11_DATA_TRUNCATE_ALL, 256));
  TEST_ASSERT_EQUAL_UINT16(20, dot11ScannerDataSnaplen(DOT11_DATA_TRUNCATE_ALL, 20));
}

void test_truncate_all_after_scanner_snaplen() {
  // The scanner already cut a 4-address QoS+HTC frame to the longest header;
  // the controller still trims it to its real header and keeps classifying.
  uint8_t buf[1600];
  buildData(buf, 0x88, 0x01, 0x0800, 1400);
  const Dot11FilterResult r =
      dot11Filter(buf, DOT11_MAX_DATA_HEADER, ALL_TYPES, DOT11_DATA_TRUNCATE_ALL);
  TEST_ASSERT_EQUAL_UINT8(DOT11_ACTION_TRUNCATE, r.action);
  TEST_ASSERT_EQUAL_UINT16(26, r.cap_len);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_type_constants_match_protocols);
  RUN_TEST(test_classify_mgmt_ctrl_extension);
  RUN_TEST(test_data_header_lengths);
  RUN_TEST(test_classify_data_kinds);
  RUN_TEST(test_eapol_needs_full_llc_and_no_amsdu);
  RUN_TEST(test_short_data_is_malformed);
  RUN_TEST(test_ctrl_filtered_by_type_mask);
  RUN_TEST(test_data_modes);
  RUN_TEST(test_data_dropped_without_data_type);
  RUN_TEST(test_mgmt_never_truncated);
  RUN_TEST(test_scanner_snaplen_pushdown);
  RUN_TEST(test_truncate_all_after_scanner_snaplen);
  return UNITY_END();
}
