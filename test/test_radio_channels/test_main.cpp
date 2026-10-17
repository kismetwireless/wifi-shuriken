#include <stdint.h>

#include <unity.h>

#include "radio_channels.h"

void setUp() {}
void tearDown() {}

static RadioChannelSet allChannels() {
  RadioChannelSet set = {};
  for (int ch = 1; ch <= 14; ch++) {
    set.mask24 |= static_cast<uint16_t>(1U << ch);
  }
  for (int i = 0; i < RADIO_5G_CHANNEL_COUNT; i++) {
    set.mask5 |= (1UL << i);
  }
  return set;
}

static void remove5g(RadioChannelSet& set, uint8_t channel) {
  set.mask5 &= ~(1UL << radio5gIndex(channel));
}

static constexpr uint8_t ALL_WIDTHS = RADIO_WIDTH_20 | RADIO_WIDTH_40 | RADIO_WIDTH_80;

void test_5g_table_matches_idf_bit_order() {
  TEST_ASSERT_EQUAL_INT(0, radio5gIndex(36));
  TEST_ASSERT_EQUAL_INT(7, radio5gIndex(64));
  TEST_ASSERT_EQUAL_INT(8, radio5gIndex(100));
  TEST_ASSERT_EQUAL_INT(20, radio5gIndex(149));
  TEST_ASSERT_EQUAL_INT(27, radio5gIndex(177));
  TEST_ASSERT_EQUAL_INT(-1, radio5gIndex(38));
}

void test_disallowed_channel_has_no_widths() {
  RadioChannelSet set = allChannels();
  set.mask24 &= ~(1U << 13);
  TEST_ASSERT_EQUAL_HEX8(0, radioChannelWidths(set, RADIO_BAND_24GHZ, 13, ALL_WIDTHS));
  TEST_ASSERT_EQUAL_HEX8(0, radioChannelWidths(set, RADIO_BAND_5GHZ, 38, ALL_WIDTHS));
  TEST_ASSERT_EQUAL_HEX8(0, radioChannelWidths(set, 7, 1, ALL_WIDTHS));
}

void test_20mhz_only_radio() {
  const RadioChannelSet set = allChannels();
  TEST_ASSERT_EQUAL_HEX8(RADIO_CHAN_WIDTH_20,
                         radioChannelWidths(set, RADIO_BAND_24GHZ, 6, RADIO_WIDTH_20));
  TEST_ASSERT_EQUAL_HEX8(RADIO_CHAN_WIDTH_20,
                         radioChannelWidths(set, RADIO_BAND_5GHZ, 36, RADIO_WIDTH_20));
}

void test_24ghz_ht40_secondary_rules() {
  const RadioChannelSet set = allChannels();
  // Channel 1: only above (5); channel 13: only below (9); 7: both.
  TEST_ASSERT_EQUAL_HEX8(RADIO_CHAN_WIDTH_20 | RADIO_CHAN_WIDTH_40_ABOVE,
                         radioChannelWidths(set, RADIO_BAND_24GHZ, 1, ALL_WIDTHS));
  TEST_ASSERT_EQUAL_HEX8(RADIO_CHAN_WIDTH_20 | RADIO_CHAN_WIDTH_40_BELOW,
                         radioChannelWidths(set, RADIO_BAND_24GHZ, 13, ALL_WIDTHS));
  TEST_ASSERT_EQUAL_HEX8(RADIO_CHAN_WIDTH_20 | RADIO_CHAN_WIDTH_40_ABOVE | RADIO_CHAN_WIDTH_40_BELOW,
                         radioChannelWidths(set, RADIO_BAND_24GHZ, 7, ALL_WIDTHS));
  // Channel 14 never takes part in HT40, and 80 MHz never applies on 2.4 GHz.
  TEST_ASSERT_EQUAL_HEX8(RADIO_CHAN_WIDTH_20,
                         radioChannelWidths(set, RADIO_BAND_24GHZ, 14, ALL_WIDTHS));
  // Channel 10 would need 14 above; it only gets below.
  TEST_ASSERT_EQUAL_HEX8(RADIO_CHAN_WIDTH_20 | RADIO_CHAN_WIDTH_40_BELOW,
                         radioChannelWidths(set, RADIO_BAND_24GHZ, 10, ALL_WIDTHS));
}

void test_24ghz_ht40_needs_partner_allowed() {
  RadioChannelSet set = allChannels();
  set.mask24 &= ~(1U << 5);
  TEST_ASSERT_EQUAL_HEX8(RADIO_CHAN_WIDTH_20,
                         radioChannelWidths(set, RADIO_BAND_24GHZ, 1, ALL_WIDTHS));
}

void test_5ghz_ht40_pairs() {
  const RadioChannelSet set = allChannels();
  const uint8_t w40 = RADIO_WIDTH_20 | RADIO_WIDTH_40;
  TEST_ASSERT_EQUAL_HEX8(RADIO_CHAN_WIDTH_20 | RADIO_CHAN_WIDTH_40_ABOVE,
                         radioChannelWidths(set, RADIO_BAND_5GHZ, 36, w40));
  TEST_ASSERT_EQUAL_HEX8(RADIO_CHAN_WIDTH_20 | RADIO_CHAN_WIDTH_40_BELOW,
                         radioChannelWidths(set, RADIO_BAND_5GHZ, 40, w40));
  TEST_ASSERT_EQUAL_HEX8(RADIO_CHAN_WIDTH_20 | RADIO_CHAN_WIDTH_40_ABOVE,
                         radioChannelWidths(set, RADIO_BAND_5GHZ, 149, w40));
  TEST_ASSERT_EQUAL_HEX8(RADIO_CHAN_WIDTH_20 | RADIO_CHAN_WIDTH_40_BELOW,
                         radioChannelWidths(set, RADIO_BAND_5GHZ, 161, w40));
  TEST_ASSERT_EQUAL_HEX8(RADIO_CHAN_WIDTH_20 | RADIO_CHAN_WIDTH_40_BELOW,
                         radioChannelWidths(set, RADIO_BAND_5GHZ, 144, w40));
}

void test_5ghz_80mhz_blocks() {
  RadioChannelSet set = allChannels();
  TEST_ASSERT_TRUE(radioChannelWidths(set, RADIO_BAND_5GHZ, 44, ALL_WIDTHS) & RADIO_CHAN_WIDTH_80);
  TEST_ASSERT_TRUE(radioChannelWidths(set, RADIO_BAND_5GHZ, 161, ALL_WIDTHS) & RADIO_CHAN_WIDTH_80);
  // Losing one member of the 36-48 block removes 80 MHz from all of it, and
  // 40 MHz only from the affected pair.
  remove5g(set, 48);
  TEST_ASSERT_EQUAL_HEX8(RADIO_CHAN_WIDTH_20 | RADIO_CHAN_WIDTH_40_ABOVE,
                         radioChannelWidths(set, RADIO_BAND_5GHZ, 36, ALL_WIDTHS));
  TEST_ASSERT_EQUAL_HEX8(RADIO_CHAN_WIDTH_20,
                         radioChannelWidths(set, RADIO_BAND_5GHZ, 44, ALL_WIDTHS));
  TEST_ASSERT_TRUE(radioChannelWidths(set, RADIO_BAND_5GHZ, 52, ALL_WIDTHS) & RADIO_CHAN_WIDTH_80);
}

void test_country_5g_mask_shifts_idf_bits() {
  // ESP-IDF: bit 1 is channel 36, bit 28 is 177.
  const uint32_t idf = (1UL << 1) | (1UL << 28);
  TEST_ASSERT_EQUAL_HEX32((1UL << 0) | (1UL << 27), radioCountry5gMask(idf));
}

void test_regulatory_24ghz_range() {
  // World-safe "01": channels 1..11.
  TEST_ASSERT_EQUAL_UINT8(RADIO_REG_ALLOWED, radioChannelRegulatory(RADIO_BAND_24GHZ, 1, 1, 11, 0));
  TEST_ASSERT_EQUAL_UINT8(RADIO_REG_ALLOWED, radioChannelRegulatory(RADIO_BAND_24GHZ, 11, 1, 11, 0));
  TEST_ASSERT_EQUAL_UINT8(RADIO_REG_NOT_ALLOWED,
                          radioChannelRegulatory(RADIO_BAND_24GHZ, 12, 1, 11, 0));
  TEST_ASSERT_EQUAL_UINT8(RADIO_REG_NOT_ALLOWED,
                          radioChannelRegulatory(RADIO_BAND_24GHZ, 14, 1, 13, 0));
  TEST_ASSERT_EQUAL_UINT8(RADIO_REG_UNKNOWN, radioChannelRegulatory(RADIO_BAND_24GHZ, 6, 1, 0, 0));
}

void test_regulatory_5ghz_mask() {
  const uint32_t unii1 = (1UL << radio5gIndex(36)) | (1UL << radio5gIndex(40)) |
                         (1UL << radio5gIndex(44)) | (1UL << radio5gIndex(48));
  TEST_ASSERT_EQUAL_UINT8(RADIO_REG_ALLOWED, radioChannelRegulatory(RADIO_BAND_5GHZ, 44, 1, 11, unii1));
  TEST_ASSERT_EQUAL_UINT8(RADIO_REG_NOT_ALLOWED,
                          radioChannelRegulatory(RADIO_BAND_5GHZ, 149, 1, 11, unii1));
  // A zero mask leaves 5 GHz to the driver's defaults: unknown, not allowed.
  TEST_ASSERT_EQUAL_UINT8(RADIO_REG_UNKNOWN, radioChannelRegulatory(RADIO_BAND_5GHZ, 149, 1, 11, 0));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_5g_table_matches_idf_bit_order);
  RUN_TEST(test_disallowed_channel_has_no_widths);
  RUN_TEST(test_20mhz_only_radio);
  RUN_TEST(test_24ghz_ht40_secondary_rules);
  RUN_TEST(test_24ghz_ht40_needs_partner_allowed);
  RUN_TEST(test_5ghz_ht40_pairs);
  RUN_TEST(test_5ghz_80mhz_blocks);
  RUN_TEST(test_country_5g_mask_shifts_idf_bits);
  RUN_TEST(test_regulatory_24ghz_range);
  RUN_TEST(test_regulatory_5ghz_mask);
  return UNITY_END();
}
