// scanner_platform_esp32_radio.cpp
#include "scanner_platform_esp32_radio.h"

#include <string.h>

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "wifi_result_utils.h"

// Map ESP-IDF auth modes into transport-stable capability bits.
// This intentionally captures security-family intent; detailed RSN IE parsing
// is out of scope for this backend abstraction.
static uint16_t scannerCapabilitiesFromAuthMode(wifi_auth_mode_t auth) {
  uint16_t caps = CAP_ESS;

  switch (auth) {
    case WIFI_AUTH_OPEN:
      break;
    case WIFI_AUTH_WEP:
      caps |= CAP_WEP;
      break;
    case WIFI_AUTH_WPA_PSK:
      caps |= (CAP_WPA | CAP_PSK | CAP_TKIP);
      break;
    case WIFI_AUTH_WPA2_PSK:
      caps |= (CAP_WPA2 | CAP_PSK | CAP_CCMP);
      break;
    case WIFI_AUTH_WPA_WPA2_PSK:
      caps |= (CAP_WPA | CAP_WPA2 | CAP_PSK | CAP_TKIP | CAP_CCMP);
      break;
    case WIFI_AUTH_WPA2_ENTERPRISE:
      caps |= (CAP_WPA2 | CAP_EAP | CAP_CCMP);
      break;
    case WIFI_AUTH_WPA3_PSK:
      caps |= (CAP_WPA3 | CAP_PSK | CAP_CCMP);
      break;
    case WIFI_AUTH_WPA2_WPA3_PSK:
      caps |= (CAP_WPA2 | CAP_WPA3 | CAP_PSK | CAP_CCMP);
      break;
#if defined(WIFI_AUTH_WPA3_ENT_192)
    case WIFI_AUTH_WPA3_ENT_192:
      caps |= (CAP_WPA3 | CAP_EAP | CAP_CCMP);
      break;
#endif
#if defined(WIFI_AUTH_WPA3_EXT_PSK)
    case WIFI_AUTH_WPA3_EXT_PSK:
      caps |= (CAP_WPA3 | CAP_PSK | CAP_CCMP);
      break;
#endif
#if defined(WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE)
    case WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE:
      caps |= (CAP_WPA2 | CAP_WPA3 | CAP_PSK | CAP_CCMP);
      break;
#endif
#if defined(WIFI_AUTH_OWE)
    case WIFI_AUTH_OWE:
      // Enhanced open; no matching flag in current protocol bitset.
      break;
#endif
#if defined(WIFI_AUTH_WAPI_PSK)
    case WIFI_AUTH_WAPI_PSK:
      // No WAPI bit in current protocol bitset.
      break;
#endif
    default:
      break;
  }

  return caps;
}

bool scannerRadioInit() {
  // STA mode only; disconnect to force clean scan behavior after reboot.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  return true;
}

// Widths esp_wifi_set_bandwidth accepts in the current band mode and
// protocol, leaving the original bandwidth in place.
static uint8_t probeWidths() {
  wifi_bandwidth_t original = WIFI_BW_HT20;
  const bool have_original = esp_wifi_get_bandwidth(WIFI_IF_STA, &original) == ESP_OK;
  static const struct {
    wifi_bandwidth_t bw;
    uint8_t bit;
  } kWidths[] = {
      {WIFI_BW_HT20, RADIO_WIDTH_20},
      {WIFI_BW_HT40, RADIO_WIDTH_40},
      {WIFI_BW80, RADIO_WIDTH_80},
      {WIFI_BW160, RADIO_WIDTH_160},
  };
  uint8_t mask = 0;
  for (const auto& w : kWidths) {
    if (esp_wifi_set_bandwidth(WIFI_IF_STA, w.bw) == ESP_OK) {
      mask |= w.bit;
    }
  }
  if (have_original) {
    esp_wifi_set_bandwidth(WIFI_IF_STA, original);
  }
  return mask;
}

// Probe one band: channels the driver accepts, the configured protocols, and
// widths both as configured and with an 11n-only protocol set.
static void probeBand(wifi_band_mode_t mode, uint8_t n_only_protocols, RadioInfoReply& out) {
  if (esp_wifi_set_band_mode(mode) != ESP_OK) {
    return;
  }
  const bool five = (mode == WIFI_BAND_MODE_5G_ONLY);
  if (five) {
    for (uint8_t i = 0; i < RADIO_5G_CHANNEL_COUNT; i++) {
      if (esp_wifi_set_channel(RADIO_5G_CHANNELS[i], WIFI_SECOND_CHAN_NONE) == ESP_OK) {
        out.chan5_mask |= (1UL << i);
      } else {
        out.channel_errors++;
      }
    }
  } else {
    for (uint8_t ch = 1; ch <= 14; ch++) {
      if (esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE) == ESP_OK) {
        out.chan24_mask |= static_cast<uint16_t>(1U << ch);
      } else {
        out.channel_errors++;
      }
    }
  }

  uint8_t protocols = 0;
  const bool have_protocols = esp_wifi_get_protocol(WIFI_IF_STA, &protocols) == ESP_OK;
  const uint8_t now = probeWidths();
  uint8_t capable = now;
  if (have_protocols && esp_wifi_set_protocol(WIFI_IF_STA, n_only_protocols) == ESP_OK) {
    capable |= probeWidths();
    esp_wifi_set_protocol(WIFI_IF_STA, protocols);
  }
  if (five) {
    out.protocols5 = protocols;
    out.widths5_now = now;
    out.widths5_capable = capable;
  } else {
    out.protocols24 = protocols;
    out.widths24_now = now;
    out.widths24_capable = capable;
  }
}

void scannerRadioProbeCapabilities(RadioInfoReply& out) {
  memset(&out, 0, sizeof(out));
  out.status = 0;
  if (!WiFi.enableSTA(true)) {
    return;
  }

  wifi_country_t country = {};
  if (esp_wifi_get_country(&country) == ESP_OK) {
    memcpy(out.country, country.cc, sizeof(out.country));
    out.policy = static_cast<uint8_t>(country.policy);
    out.max_tx_power = country.max_tx_power;
    out.country_schan = country.schan;
    out.country_nchan = country.nchan;
    out.country_5g_mask = country.wifi_5g_channel_mask;
    out.flags |= RADIO_INFO_COUNTRY_VALID;
  }

  wifi_band_mode_t original_mode = WIFI_BAND_MODE_AUTO;
  esp_wifi_get_band_mode(&original_mode);
  probeBand(WIFI_BAND_MODE_2G_ONLY, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N, out);
  probeBand(WIFI_BAND_MODE_5G_ONLY, WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N, out);
  esp_wifi_set_band_mode(original_mode);
  out.flags |= RADIO_INFO_VALID;

  // Back to the state scannerRadioInit leaves: STA stopped until a scan or
  // capture brings it up.
  WiFi.disconnect(true, true);
}

int scannerRadioStartAsyncScan(uint8_t /*band*/, uint8_t channel, bool passive, int dwell_ms) {
  // band is currently implicit via channel on this backend.
  const int ret = WiFi.scanNetworks(true, true, passive, dwell_ms, channel);
  if (ret == WIFI_SCAN_FAILED) {
    return SCANNER_RADIO_SCAN_FAILED;
  }
  return ret;
}

int scannerRadioPollAsyncScan() {
  // scanComplete() returns WIFI_SCAN_RUNNING while active, >=0 when complete.
  const int ret = WiFi.scanComplete();
  if (ret == WIFI_SCAN_RUNNING) {
    return SCANNER_RADIO_SCAN_RUNNING;
  }
  if (ret < 0) {
    return SCANNER_RADIO_SCAN_FAILED;
  }
  return ret;
}

void scannerRadioDeleteScanResults() {
  WiFi.scanDelete();
}

bool scannerRadioReadResult(int index, WiFiResult& out) {
  if (index < 0) {
    return false;
  }

  memset(&out, 0, sizeof(out));
  String ssid = WiFi.SSID(index);
  ssid.toCharArray(out.ssid, sizeof(out.ssid));
  WiFi.BSSID(index, out.bssid);
  out.rssi = static_cast<int8_t>(WiFi.RSSI(index));
  out.channel = static_cast<uint8_t>(WiFi.channel(index));
  out.band = wifiBandFromChannel(out.channel);
  out.capabilities = scannerCapabilitiesFromAuthMode(WiFi.encryptionType(index));
  return true;
}
