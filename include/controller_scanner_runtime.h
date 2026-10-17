#pragma once

#include <Arduino.h>
#include <SdFat.h>
#include <stdint.h>

class Adafruit_NeoPixel;

#include "pico/util/queue.h"
#include "wifi_dedupe.h"

// Cross-core resources owned by controller_main.cpp and consumed by the
// scanner runtime on core1.
struct ControllerScannerRuntimeContext {
  // Queue of unique scan results produced on core1 and drained on core0.
  queue_t* scan_result_queue;
  // Drop counters updated by the runtime when queueing or dedupe rejects fail.
  uint32_t* scan_queue_drops;
  uint32_t* dedupe_drops;
  // Sweep timing stats surfaced by periodic controller status output.
  volatile uint32_t* sweep_cycles_completed;
  volatile uint32_t* last_full_sweep_ms;
  // Controller-wide dedupe table shared across all scanner slots.
  WiFiDedupeTable* master_dedupe_table;
  // Core1 cannot print directly; queue diagnostic lines for core0 instead.
  void (*serial_queue_try)(const char* text);
  // Watchdog heartbeat: core1 writes the current time (ms) each main loop
  // iteration so core0 can verify both cores are alive before kicking.
  volatile uint32_t* wd_core1_last_ms;
};

// Initialize the SPI1 bus and any scanner CS hardware.
void controllerScannerRuntimeInitBus();

// Request a controller-wide dedupe reset from core0. Core1 applies the reset
// to the master dedupe table and re-arms per-scanner dedupe handshakes.
void controllerScannerRuntimeRequestDedupeReset();

// Push /scanner.bin to attached ESP32-C5 scanners during the boot-time quiet
// window before controllerScannerRuntimeRun() starts normal scan scheduling.
bool controllerScannerRuntimeHandleScannerUpdatesFromSd(SdFat& sd,
                                                        bool sd_ready,
                                                        Stream& serial,
                                                        Adafruit_NeoPixel& pixels);

// Run the scanner service loop on core1 forever.
void controllerScannerRuntimeRun(const ControllerScannerRuntimeContext& context);

// ---------- Monitor mode hand-off (core0 -> core1) ----------
// The USB capture session (core0) asks core1 to take scanners out of scanning
// and into promiscuous capture; core1 applies the request between slot
// services, streams captured frames into the USB packet ring, and reports
// per-slot state back.
static constexpr uint8_t CONTROLLER_MONITOR_MAX_SLOTS = 8;
static constexpr uint8_t CONTROLLER_MONITOR_MAX_HOP = 64;

struct ControllerMonitorHopEntry {
  uint8_t band;
  uint8_t channel;
};

struct ControllerMonitorSlotConfig {
  uint8_t band;
  uint8_t channel;
  uint8_t type_mask;  // MONITOR_TYPE_* bits.
  uint16_t snaplen_mgmt;
  uint16_t snaplen_ctrl;
  uint16_t snaplen_data;
  // When hopping, the scanner retunes through hop[] every hop_dwell_ms on its
  // own, starting at hop_offset; band/channel are then ignored.
  bool hopping;
  uint8_t hop_count;
  uint8_t hop_offset;
  uint16_t hop_dwell_ms;
  ControllerMonitorHopEntry hop[CONTROLLER_MONITOR_MAX_HOP];
};

struct ControllerMonitorRequest {
  bool active;
  uint8_t slot_mask;
  ControllerMonitorSlotConfig slot[CONTROLLER_MONITOR_MAX_SLOTS];
};

struct ControllerMonitorSlotStatus {
  bool present;
  bool monitoring;
  bool hopping;
  // Channel the radio is on now (follows the hops).
  uint8_t band;
  uint8_t channel;
  // Cumulative since the slot last entered monitor mode.
  uint32_t radio_drops;
  uint32_t transport_errors;
};

// core0: replace the current request. Takes effect asynchronously.
void controllerScannerMonitorRequest(const ControllerMonitorRequest& request);

// core0: true once core1 has applied the most recent request.
bool controllerScannerMonitorApplied();

// core0: latest state core1 reported for a slot.
void controllerScannerMonitorSlotStatus(uint8_t slot, ControllerMonitorSlotStatus& out);
