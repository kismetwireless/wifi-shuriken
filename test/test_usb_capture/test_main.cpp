#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "capture_ring.h"
#include "usb_capture_protocol.h"

void setUp() {}
void tearDown() {}

// Drain the whole ring via readSpan()/consume(), the way the USB pump does.
static uint32_t drainAll(CaptureRing& ring, uint8_t* out, uint32_t out_len) {
  uint32_t total = 0;
  while (true) {
    const uint8_t* span = nullptr;
    uint32_t n = ring.readSpan(&span);
    if (n == 0) {
      break;
    }
    if (n > out_len - total) {
      n = out_len - total;
    }
    memcpy(out + total, span, n);
    ring.consume(n);
    total += n;
  }
  return total;
}

void test_protocol_values_are_stable() {
  TEST_ASSERT_EQUAL_HEX16(0x4853, USB_CAPTURE_MAGIC);
  TEST_ASSERT_EQUAL_UINT8(1, USB_CAPTURE_PROTO_VERSION);
  TEST_ASSERT_EQUAL_HEX8(0x01, USB_CAPTURE_CMD_HELLO);
  TEST_ASSERT_EQUAL_HEX8(0x02, USB_CAPTURE_CMD_START);
  TEST_ASSERT_EQUAL_HEX8(0x03, USB_CAPTURE_CMD_STOP);
  TEST_ASSERT_EQUAL_HEX8(0x04, USB_CAPTURE_CMD_KEEPALIVE);
  TEST_ASSERT_EQUAL_HEX8(0x05, USB_CAPTURE_CMD_SET_CHANNEL);
  TEST_ASSERT_EQUAL_HEX8(0x06, USB_CAPTURE_CMD_SET_HOP);
  TEST_ASSERT_EQUAL_HEX8(0x07, USB_CAPTURE_CMD_SET_FILTER);
  TEST_ASSERT_EQUAL_HEX8(0x08, USB_CAPTURE_CMD_GET_STATUS);
  TEST_ASSERT_EQUAL_HEX8(0x80, USB_CAPTURE_REC_REPLY);
  TEST_ASSERT_EQUAL_HEX8(0x81, USB_CAPTURE_REC_PACKET);
  TEST_ASSERT_EQUAL_HEX8(0x82, USB_CAPTURE_REC_STATUS);
}

void test_record_header_wire_layout() {
  UsbCaptureRecordHeader hdr = {};
  usbCaptureFillHeader(hdr, USB_CAPTURE_REC_PACKET, 0x1234, 0xBEEF);
  uint8_t raw[sizeof(hdr)];
  memcpy(raw, &hdr, sizeof(hdr));
  const uint8_t expected[] = {0x53, 0x48, 0x81, 0x01, 0x34, 0x12, 0xEF, 0xBE};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, raw, sizeof(expected));
}

void test_packet_header_field_offsets() {
  TEST_ASSERT_EQUAL_UINT32(0, offsetof(UsbCapturePacketHeader, slot));
  TEST_ASSERT_EQUAL_UINT32(7, offsetof(UsbCapturePacketHeader, flags));
  TEST_ASSERT_EQUAL_UINT32(8, offsetof(UsbCapturePacketHeader, radio_timestamp_us));
  TEST_ASSERT_EQUAL_UINT32(12, offsetof(UsbCapturePacketHeader, controller_timestamp_us));
  TEST_ASSERT_EQUAL_UINT32(20, offsetof(UsbCapturePacketHeader, orig_len));
  TEST_ASSERT_EQUAL_UINT32(22, offsetof(UsbCapturePacketHeader, cap_len));
}

void test_channel_validation() {
  TEST_ASSERT_TRUE(usbCaptureChannelValid(USB_CAPTURE_BAND_24GHZ, 1));
  TEST_ASSERT_TRUE(usbCaptureChannelValid(USB_CAPTURE_BAND_24GHZ, 14));
  TEST_ASSERT_FALSE(usbCaptureChannelValid(USB_CAPTURE_BAND_24GHZ, 0));
  TEST_ASSERT_FALSE(usbCaptureChannelValid(USB_CAPTURE_BAND_24GHZ, 36));
  TEST_ASSERT_TRUE(usbCaptureChannelValid(USB_CAPTURE_BAND_5GHZ, 36));
  TEST_ASSERT_TRUE(usbCaptureChannelValid(USB_CAPTURE_BAND_5GHZ, 165));
  TEST_ASSERT_FALSE(usbCaptureChannelValid(USB_CAPTURE_BAND_5GHZ, 6));
  TEST_ASSERT_FALSE(usbCaptureChannelValid(0, 6));
}

void test_ring_capacity_validation() {
  TEST_ASSERT_TRUE(CaptureRing::capacityValid(64));
  TEST_ASSERT_TRUE(CaptureRing::capacityValid(131072));
  TEST_ASSERT_FALSE(CaptureRing::capacityValid(0));
  TEST_ASSERT_FALSE(CaptureRing::capacityValid(100));
}

void test_ring_two_segment_write_round_trips() {
  uint8_t storage[64];
  CaptureRing ring;
  ring.init(storage, sizeof(storage));
  const uint8_t a[] = {1, 2, 3};
  const uint8_t b[] = {4, 5, 6, 7};
  TEST_ASSERT_TRUE(ring.tryWrite(a, sizeof(a), b, sizeof(b)));
  TEST_ASSERT_EQUAL_UINT32(7, ring.used());

  uint8_t out[16] = {};
  TEST_ASSERT_EQUAL_UINT32(7, drainAll(ring, out, sizeof(out)));
  const uint8_t expected[] = {1, 2, 3, 4, 5, 6, 7};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
  TEST_ASSERT_EQUAL_UINT32(0, ring.used());
}

void test_ring_write_is_all_or_nothing_when_full() {
  uint8_t storage[16];
  CaptureRing ring;
  ring.init(storage, sizeof(storage));
  uint8_t fill[12] = {};
  TEST_ASSERT_TRUE(ring.tryWrite(fill, sizeof(fill)));
  uint8_t big[5] = {};
  TEST_ASSERT_FALSE(ring.tryWrite(big, sizeof(big)));
  TEST_ASSERT_EQUAL_UINT32(12, ring.used());
  uint8_t fits[4] = {};
  TEST_ASSERT_TRUE(ring.tryWrite(fits, sizeof(fits)));
  TEST_ASSERT_EQUAL_UINT32(0, ring.freeSpace());
}

void test_ring_wraps_and_read_span_stops_at_wrap() {
  uint8_t storage[16];
  CaptureRing ring;
  ring.init(storage, sizeof(storage));
  uint8_t first[12];
  memset(first, 0xAA, sizeof(first));
  TEST_ASSERT_TRUE(ring.tryWrite(first, sizeof(first)));
  uint8_t discard[12];
  TEST_ASSERT_EQUAL_UINT32(12, ring.peek(discard, sizeof(discard)));
  ring.consume(12);

  // This record straddles the end of storage.
  const uint8_t rec[] = {10, 11, 12, 13, 14, 15, 16, 17};
  TEST_ASSERT_TRUE(ring.tryWrite(rec, sizeof(rec)));
  const uint8_t* span = nullptr;
  TEST_ASSERT_EQUAL_UINT32(4, ring.readSpan(&span));

  uint8_t peeked[8] = {};
  TEST_ASSERT_EQUAL_UINT32(8, ring.peek(peeked, sizeof(peeked)));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(rec, peeked, sizeof(rec));

  uint8_t out[8] = {};
  TEST_ASSERT_EQUAL_UINT32(8, drainAll(ring, out, sizeof(out)));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(rec, out, sizeof(rec));
}

void test_ring_peek_with_offset_and_partial_consume() {
  uint8_t storage[32];
  CaptureRing ring;
  ring.init(storage, sizeof(storage));
  const uint8_t rec[] = {1, 2, 3, 4, 5, 6};
  TEST_ASSERT_TRUE(ring.tryWrite(rec, sizeof(rec)));

  uint8_t out[4] = {};
  TEST_ASSERT_EQUAL_UINT32(2, ring.peek(out, sizeof(out), 4));
  TEST_ASSERT_EQUAL_UINT8(5, out[0]);
  TEST_ASSERT_EQUAL_UINT8(6, out[1]);
  TEST_ASSERT_EQUAL_UINT32(0, ring.peek(out, sizeof(out), 6));

  ring.consume(2);
  TEST_ASSERT_EQUAL_UINT32(4, ring.used());
  TEST_ASSERT_EQUAL_UINT32(4, ring.peek(out, sizeof(out)));
  TEST_ASSERT_EQUAL_UINT8(3, out[0]);
}

void test_ring_discard_all_empties_ring() {
  uint8_t storage[32];
  CaptureRing ring;
  ring.init(storage, sizeof(storage));
  uint8_t rec[10] = {};
  TEST_ASSERT_TRUE(ring.tryWrite(rec, sizeof(rec)));
  ring.discardAll();
  TEST_ASSERT_EQUAL_UINT32(0, ring.used());
  TEST_ASSERT_EQUAL_UINT32(32, ring.freeSpace());
  TEST_ASSERT_TRUE(ring.tryWrite(rec, sizeof(rec)));
  TEST_ASSERT_EQUAL_UINT32(10, ring.used());
}

void test_ring_survives_index_wraparound() {
  uint8_t storage[16];
  CaptureRing ring;
  ring.init(storage, sizeof(storage));
  // Push enough traffic that the free-running indices wrap many times over
  // the storage and verify each record still reads back intact.
  for (uint32_t i = 0; i < 1000; i++) {
    uint8_t rec[5];
    for (uint8_t j = 0; j < sizeof(rec); j++) {
      rec[j] = static_cast<uint8_t>(i + j);
    }
    TEST_ASSERT_TRUE(ring.tryWrite(rec, 2, rec + 2, 3));
    uint8_t out[5] = {};
    TEST_ASSERT_EQUAL_UINT32(5, drainAll(ring, out, sizeof(out)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(rec, out, sizeof(rec));
  }
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_protocol_values_are_stable);
  RUN_TEST(test_record_header_wire_layout);
  RUN_TEST(test_packet_header_field_offsets);
  RUN_TEST(test_channel_validation);
  RUN_TEST(test_ring_capacity_validation);
  RUN_TEST(test_ring_two_segment_write_round_trips);
  RUN_TEST(test_ring_write_is_all_or_nothing_when_full);
  RUN_TEST(test_ring_wraps_and_read_span_stops_at_wrap);
  RUN_TEST(test_ring_peek_with_offset_and_partial_consume);
  RUN_TEST(test_ring_discard_all_empties_ring);
  RUN_TEST(test_ring_survives_index_wraparound);
  return UNITY_END();
}
