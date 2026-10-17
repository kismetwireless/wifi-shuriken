#pragma once

#include <stdint.h>

#include "spi_protocol_shared.h"

// Which channel widths a channel can use, given the set of channels a radio
// accepts and the widths the radio can be set to. Pure logic shared by the
// controller (for the USB channel list) and the native tests.

// Per-channel width flags.
static constexpr uint8_t RADIO_CHAN_WIDTH_20 = 0x01;
static constexpr uint8_t RADIO_CHAN_WIDTH_40_ABOVE = 0x02;  // HT40+: secondary channel above.
static constexpr uint8_t RADIO_CHAN_WIDTH_40_BELOW = 0x04;  // HT40-: secondary channel below.
static constexpr uint8_t RADIO_CHAN_WIDTH_80 = 0x08;

static constexpr uint8_t RADIO_BAND_24GHZ = 2;
static constexpr uint8_t RADIO_BAND_5GHZ = 5;

struct RadioChannelSet {
  uint16_t mask24;  // Bit n: 2.4 GHz channel n (1..14).
  uint32_t mask5;   // Bit i: RADIO_5G_CHANNELS[i].
};

static inline int radio5gIndex(uint8_t channel) {
  for (uint8_t i = 0; i < RADIO_5G_CHANNEL_COUNT; i++) {
    if (RADIO_5G_CHANNELS[i] == channel) {
      return i;
    }
  }
  return -1;
}

static inline bool radioChannelAllowed(const RadioChannelSet& set, uint8_t band, int channel) {
  if (band == RADIO_BAND_24GHZ) {
    return channel >= 1 && channel <= 14 && (set.mask24 & (1U << channel));
  }
  if (band == RADIO_BAND_5GHZ && channel > 0 && channel < 256) {
    const int index = radio5gIndex(static_cast<uint8_t>(channel));
    return index >= 0 && (set.mask5 & (1UL << index));
  }
  return false;
}

// First channel of the 80 MHz block holding a 5 GHz channel, or 0 if none.
static inline uint8_t radio5g80BlockStart(uint8_t channel) {
  static const uint8_t kStarts[] = {36, 52, 100, 116, 132, 149, 165};
  for (uint8_t start : kStarts) {
    if (channel >= start && channel <= start + 12) {
      return start;
    }
  }
  return 0;
}

// What a country record says about a channel.
enum RadioRegulatory : uint8_t {
  RADIO_REG_NOT_ALLOWED = 0,
  RADIO_REG_ALLOWED = 1,
  RADIO_REG_UNKNOWN = 2,  // The record does not list it (ESP-IDF 5 GHz mask 0).
};

// ESP-IDF's wifi_5g_channel_mask uses bit i + 1 for RADIO_5G_CHANNELS[i];
// convert it to the RadioChannelSet layout (bit i).
static inline uint32_t radioCountry5gMask(uint32_t idf_mask) {
  return idf_mask >> 1;
}

// schan/nchan: the country record's 2.4 GHz range. country5_mask is in
// RadioChannelSet layout; 0 means the record leaves 5 GHz to the driver's
// regulatory defaults, which it does not expose.
static inline RadioRegulatory radioChannelRegulatory(uint8_t band, uint8_t channel,
                                                     uint8_t schan, uint8_t nchan,
                                                     uint32_t country5_mask) {
  if (band == RADIO_BAND_24GHZ) {
    if (nchan == 0) {
      return RADIO_REG_UNKNOWN;
    }
    return (channel >= schan && channel < schan + nchan) ? RADIO_REG_ALLOWED
                                                         : RADIO_REG_NOT_ALLOWED;
  }
  if (band == RADIO_BAND_5GHZ) {
    if (country5_mask == 0) {
      return RADIO_REG_UNKNOWN;
    }
    const int index = radio5gIndex(channel);
    return (index >= 0 && (country5_mask & (1UL << index))) ? RADIO_REG_ALLOWED
                                                             : RADIO_REG_NOT_ALLOWED;
  }
  return RADIO_REG_NOT_ALLOWED;
}

// band_widths: RADIO_WIDTH_* bits the radio can be set to on this band.
static inline uint8_t radioChannelWidths(const RadioChannelSet& set, uint8_t band,
                                         uint8_t channel, uint8_t band_widths) {
  if (!radioChannelAllowed(set, band, channel)) {
    return 0;
  }
  uint8_t widths = 0;
  if (band_widths & RADIO_WIDTH_20) {
    widths |= RADIO_CHAN_WIDTH_20;
  }
  if (band_widths & RADIO_WIDTH_40) {
    if (band == RADIO_BAND_24GHZ) {
      // HT40 is not permitted on channel 14; the secondary must stay in 1..13.
      if (channel <= 13) {
        if (channel + 4 <= 13 && radioChannelAllowed(set, band, channel + 4)) {
          widths |= RADIO_CHAN_WIDTH_40_ABOVE;
        }
        if (channel >= 5 && radioChannelAllowed(set, band, channel - 4)) {
          widths |= RADIO_CHAN_WIDTH_40_BELOW;
        }
      }
    } else {
      // Fixed 40 MHz pairs: 36/40, 44/48, ... and 149/153, 157/161, ...
      const uint8_t base = channel >= 149 ? 149 : 36;
      const bool lower = ((channel - base) % 8) == 0;
      const int partner = lower ? channel + 4 : channel - 4;
      if (radioChannelAllowed(set, band, partner)) {
        widths |= lower ? RADIO_CHAN_WIDTH_40_ABOVE : RADIO_CHAN_WIDTH_40_BELOW;
      }
    }
  }
  if ((band_widths & RADIO_WIDTH_80) && band == RADIO_BAND_5GHZ) {
    const uint8_t start = radio5g80BlockStart(channel);
    if (start != 0 && radioChannelAllowed(set, band, start) &&
        radioChannelAllowed(set, band, start + 4) &&
        radioChannelAllowed(set, band, start + 8) &&
        radioChannelAllowed(set, band, start + 12)) {
      widths |= RADIO_CHAN_WIDTH_80;
    }
  }
  return widths;
}
