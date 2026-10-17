// Standalone controller firmware (env xiao_rp2350_dot11_bench) that times the
// USB capture frame filter on the RP2350. It runs dot11Filter() plus the copy
// into a capture buffer, the same work core1 does per frame on its way to USB,
// over a set of representative frames and data modes, and prints the results
// to the USB console every few seconds. Reflash the normal controller
// firmware afterwards; the scanners are not touched.

#include <Arduino.h>
#include <pico/time.h>
#include <string.h>

#include "dot11_filter.h"

namespace {

constexpr uint32_t ITERATIONS = 20000;
constexpr uint8_t ALL_TYPES = DOT11_TYPE_MGMT | DOT11_TYPE_CTRL | DOT11_TYPE_DATA;
constexpr uint8_t NO_CTRL = DOT11_TYPE_MGMT | DOT11_TYPE_DATA;

struct BenchFrame {
  const char* name;
  uint16_t len;
  uint8_t data[1600];
};

BenchFrame frames[] = {
    {"beacon 300B", 300, {}},
    {"ack 10B", 10, {}},
    {"rts 16B", 16, {}},
    {"qos null 26B", 26, {}},
    {"open data 600B", 600, {}},
    {"prot data 1500B", 1500, {}},
    {"eapol 121B", 26 + 8 + 121, {}},
};
constexpr size_t FRAME_COUNT = sizeof(frames) / sizeof(frames[0]);

// A plausible busy-channel mix, by index into frames[].
const uint8_t MIX[] = {0, 0, 0, 0, 1, 1, 1, 2, 3, 3, 4, 4, 5, 5, 5, 5, 5, 5, 6, 0};
constexpr size_t MIX_COUNT = sizeof(MIX) / sizeof(MIX[0]);

const char* const MODE_NAMES[DOT11_DATA_MODE_COUNT] = {"pass-all", "truncate-all", "pass-open",
                                                        "handshake"};

uint8_t ring[2048] __attribute__((aligned(4)));
volatile uint32_t sink = 0;

void buildFrames() {
  for (BenchFrame& f : frames) {
    for (uint16_t i = 0; i < sizeof(f.data); i++) {
      f.data[i] = static_cast<uint8_t>(i * 7 + 3);
    }
  }
  frames[0].data[0] = 0x80;  // Beacon.
  frames[0].data[1] = 0x00;
  frames[1].data[0] = 0xd4;  // ACK.
  frames[1].data[1] = 0x00;
  frames[2].data[0] = 0xb4;  // RTS.
  frames[2].data[1] = 0x00;
  frames[3].data[0] = 0xc8;  // QoS Null, to-DS.
  frames[3].data[1] = 0x11;
  for (size_t i = 4; i <= 6; i++) {
    frames[i].data[0] = 0x88;  // QoS data.
    frames[i].data[24] = 0x00; // QoS control: no A-MSDU.
  }
  frames[4].data[1] = 0x01;  // To-DS, open, IPv4 SNAP.
  const uint8_t ipv4[8] = {0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00};
  memcpy(frames[4].data + 26, ipv4, sizeof(ipv4));
  frames[5].data[1] = 0x41;  // To-DS, protected.
  frames[6].data[1] = 0x02;  // From-DS, open, EAPOL SNAP.
  const uint8_t eapol[8] = {0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8E};
  memcpy(frames[6].data + 26, eapol, sizeof(eapol));
}

// One frame's worth of delivery: filter, then copy what survives.
inline uint16_t deliver(const BenchFrame& f, uint8_t types, uint8_t mode) {
  const Dot11FilterResult r = dot11Filter(f.data, f.len, types, mode);
  if (r.action != DOT11_ACTION_DROP) {
    memcpy(ring, f.data, r.cap_len);
  }
  return r.cap_len;
}

inline uint16_t copyOnly(const BenchFrame& f) {
  memcpy(ring, f.data, f.len);
  return f.len;
}

// Nanoseconds per call of fn over ITERATIONS.
template <typename Fn>
uint32_t timeNs(Fn fn) {
  uint32_t acc = 0;
  const uint64_t start = time_us_64();
  for (uint32_t i = 0; i < ITERATIONS; i++) {
    // Make every pass reread the frame so nothing is hoisted out of the loop.
    asm volatile("" ::: "memory");
    acc += fn(i);
  }
  const uint64_t elapsed = time_us_64() - start;
  sink = sink + acc;
  return static_cast<uint32_t>(elapsed * 1000u / ITERATIONS);
}

void runBench() {
  Serial.printf("\n=== dot11 filter benchmark: %lu MHz, %lu iterations per cell ===\n",
                static_cast<unsigned long>(rp2040.f_cpu() / 1000000),
                static_cast<unsigned long>(ITERATIONS));
  Serial.printf("ns per frame: classify only | copy only | filter+copy per data mode "
                "(ctrl filtered)\n");
  Serial.printf("%-16s %8s %8s", "frame", "classify", "copy");
  for (uint8_t m = 0; m < DOT11_DATA_MODE_COUNT; m++) {
    Serial.printf(" %12s", MODE_NAMES[m]);
  }
  Serial.printf("   cap_len per mode\n");
  for (const BenchFrame& f : frames) {
    const uint32_t classify =
        timeNs([&](uint32_t) { return dot11Classify(f.data, f.len).cls; });
    const uint32_t copy = timeNs([&](uint32_t) { return copyOnly(f); });
    Serial.printf("%-16s %8lu %8lu", f.name, static_cast<unsigned long>(classify),
                  static_cast<unsigned long>(copy));
    uint16_t caps[DOT11_DATA_MODE_COUNT];
    for (uint8_t m = 0; m < DOT11_DATA_MODE_COUNT; m++) {
      const uint32_t ns = timeNs([&](uint32_t) { return deliver(f, NO_CTRL, m); });
      caps[m] = dot11Filter(f.data, f.len, NO_CTRL, m).cap_len;
      Serial.printf(" %12lu", static_cast<unsigned long>(ns));
    }
    Serial.printf("   %u/%u/%u/%u\n", caps[0], caps[1], caps[2], caps[3]);
  }

  // Mixed traffic: average cost and bytes kept per frame.
  uint32_t mix_bytes = 0;
  for (size_t i = 0; i < MIX_COUNT; i++) {
    mix_bytes += frames[MIX[i]].len;
  }
  const uint32_t copy_mix =
      timeNs([&](uint32_t i) { return copyOnly(frames[MIX[i % MIX_COUNT]]); });
  Serial.printf("\nmix of %u frames, %lu B avg: copy only %lu ns/frame\n",
                static_cast<unsigned>(MIX_COUNT), static_cast<unsigned long>(mix_bytes / MIX_COUNT),
                static_cast<unsigned long>(copy_mix));
  const uint8_t type_sets[] = {NO_CTRL, ALL_TYPES};
  for (uint8_t types : type_sets) {
    for (uint8_t m = 0; m < DOT11_DATA_MODE_COUNT; m++) {
      uint32_t kept = 0;
      for (size_t i = 0; i < MIX_COUNT; i++) {
        kept += dot11Filter(frames[MIX[i]].data, frames[MIX[i]].len, types, m).cap_len;
      }
      const uint32_t ns =
          timeNs([&](uint32_t i) { return deliver(frames[MIX[i % MIX_COUNT]], types, m); });
      Serial.printf("  ctrl %-6s data %-12s %5lu ns/frame, keeps %3lu%% of bytes "
                    "(%lu Kframes/s per core)\n",
                    types & DOT11_TYPE_CTRL ? "pass" : "filter", MODE_NAMES[m],
                    static_cast<unsigned long>(ns),
                    static_cast<unsigned long>(kept * 100u / mix_bytes),
                    static_cast<unsigned long>(ns ? 1000000u / ns : 0));
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  buildFrames();
}

void loop() {
  static uint32_t next_ms = 3000;
  if ((int32_t)(millis() - next_ms) >= 0) {
    runBench();
    next_ms = millis() + 10000;
  }
}
