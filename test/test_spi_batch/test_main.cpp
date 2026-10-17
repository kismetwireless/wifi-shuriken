#include <stdint.h>
#include <string.h>

#include <unity.h>

#include "spi_protocol_shared.h"

void setUp() {}
void tearDown() {}

// Append one record with cap_len body bytes; returns the new offset.
static uint32_t appendRecord(uint8_t* buf, uint32_t offset, uint16_t cap_len, uint16_t orig_len) {
  MonitorPacketRecord rec = {};
  rec.record_len = static_cast<uint16_t>(sizeof(rec) + cap_len);
  rec.orig_len = orig_len;
  memcpy(buf + offset, &rec, sizeof(rec));
  memset(buf + offset + sizeof(rec), 0xA5, cap_len);
  return offset + rec.record_len;
}

void test_crc32_matches_ieee_check_value() {
  const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX32(0xCBF43926, pktCrc32(check, sizeof(check)));
  // The table and bitwise OTA implementations must agree.
  TEST_ASSERT_EQUAL_HEX32(otaCrc32(check, sizeof(check)), pktCrc32(check, sizeof(check)));
}

void test_crc32_incremental_equals_one_shot() {
  uint8_t data[300];
  for (uint16_t i = 0; i < sizeof(data); i++) {
    data[i] = static_cast<uint8_t>(i * 7 + 3);
  }
  uint32_t crc = 0xffffffffUL;
  crc = pktCrc32Update(crc, data, 100);
  crc = pktCrc32Update(crc, data + 100, 200);
  TEST_ASSERT_EQUAL_HEX32(pktCrc32(data, sizeof(data)), ~crc);
}

void test_wire_len_is_aligned_and_longer_than_a_frame() {
  TEST_ASSERT_EQUAL_UINT16(PKT_BATCH_MIN_WIRE_BYTES, pktBatchWireLen(0));
  TEST_ASSERT_EQUAL_UINT16(PKT_BATCH_MIN_WIRE_BYTES, pktBatchWireLen(20));
  TEST_ASSERT_EQUAL_UINT16(PKT_BATCH_MIN_WIRE_BYTES,
                           pktBatchWireLen(PKT_BATCH_MIN_WIRE_BYTES - sizeof(PktBatchHeader)));
  // 12-byte header + 125 bytes = 137, rounded up to 140.
  TEST_ASSERT_EQUAL_UINT16(140, pktBatchWireLen(125));
  // A frame plus the driver's reporting slop must stay well short of a batch.
  TEST_ASSERT_TRUE(SPI_FRAME_SIZE * 8u + 32u < PKT_BATCH_MIN_WIRE_BYTES * 8u - 32u);
  TEST_ASSERT_EQUAL_UINT16(sizeof(PktBatchHeader) + 1000, pktBatchWireLen(1000));
  TEST_ASSERT_EQUAL_UINT16(sizeof(PktBatchHeader) + 1004, pktBatchWireLen(1001));
  const uint16_t max_records = PKT_BATCH_MAX_BYTES - sizeof(PktBatchHeader);
  TEST_ASSERT_EQUAL_UINT16(PKT_BATCH_MAX_BYTES, pktBatchWireLen(max_records));
}

void test_largest_record_fits_in_one_batch() {
  const uint32_t record = sizeof(MonitorPacketRecord) + PKT_RECORD_MAX_CAP;
  TEST_ASSERT_EQUAL_UINT32(PKT_BATCH_MAX_BYTES, sizeof(PktBatchHeader) + record);
}

void test_records_valid_accepts_well_formed_batch() {
  uint8_t buf[512];
  uint32_t offset = 0;
  offset = appendRecord(buf, offset, 24, 24);
  offset = appendRecord(buf, offset, 0, 40);
  offset = appendRecord(buf, offset, 256, 1500);
  TEST_ASSERT_TRUE(pktBatchRecordsValid(buf, static_cast<uint16_t>(offset), 3));
  TEST_ASSERT_TRUE(pktBatchRecordsValid(buf, 0, 0));
}

void test_records_valid_rejects_count_mismatch() {
  uint8_t buf[256];
  uint32_t offset = appendRecord(buf, 0, 10, 10);
  offset = appendRecord(buf, offset, 10, 10);
  TEST_ASSERT_FALSE(pktBatchRecordsValid(buf, static_cast<uint16_t>(offset), 3));
  TEST_ASSERT_FALSE(pktBatchRecordsValid(buf, static_cast<uint16_t>(offset), 1));
}

void test_records_valid_rejects_overrun_and_short_tail() {
  uint8_t buf[256];
  uint32_t offset = appendRecord(buf, 0, 40, 40);
  // Total claims fewer bytes than the record says it uses.
  TEST_ASSERT_FALSE(pktBatchRecordsValid(buf, static_cast<uint16_t>(offset - 1), 1));
  // Trailing bytes too short to hold another record header.
  TEST_ASSERT_FALSE(pktBatchRecordsValid(buf, static_cast<uint16_t>(offset + 5), 1));
}

void test_records_valid_rejects_bad_lengths() {
  uint8_t buf[256];
  memset(buf, 0, sizeof(buf));
  MonitorPacketRecord rec = {};
  // record_len below the header size would never advance.
  rec.record_len = 4;
  memcpy(buf, &rec, sizeof(rec));
  TEST_ASSERT_FALSE(pktBatchRecordsValid(buf, sizeof(rec), 1));
  // Captured more bytes than the frame had.
  appendRecord(buf, 0, 50, 49);
  TEST_ASSERT_FALSE(pktBatchRecordsValid(buf, sizeof(rec) + 50, 1));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crc32_matches_ieee_check_value);
  RUN_TEST(test_crc32_incremental_equals_one_shot);
  RUN_TEST(test_wire_len_is_aligned_and_longer_than_a_frame);
  RUN_TEST(test_largest_record_fits_in_one_batch);
  RUN_TEST(test_records_valid_accepts_well_formed_batch);
  RUN_TEST(test_records_valid_rejects_count_mismatch);
  RUN_TEST(test_records_valid_rejects_overrun_and_short_tail);
  RUN_TEST(test_records_valid_rejects_bad_lengths);
  return UNITY_END();
}
