#include "controller_scanner_runtime.h"

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/time.h"
#include "channel_scheduler.h"
#include "controller_usb_capture.h"
#include "pico/critical_section.h"
#include "pico_logging.h"
#include "spi_protocol_shared.h"
#include "wifi_result_utils.h"
#include "Configuration.h"

// This module owns the controller-side scanner transport and scheduler that run
// on core1: SPI framing, slot identification, shared sweep progression, and
// controller-wide dedupe before results are handed back to core0.

using QueuedScanResult = pico_logging::QueuedScanResult;

// Transport tuning constants for the Pico<->scanner SPI protocol.
static constexpr size_t SCANNER_FRAME_SIZE = SPI_FRAME_SIZE;
static constexpr uint32_t SCANNER_SPI_HZ = SCANNER_SPI_CLOCK;
static constexpr uint32_t SCANNER_SPI_BULK_HZ = SCANNER_SPI_BULK_CLOCK;
static constexpr size_t SCANNER_FRAME_TAG_INDEX = SPI_FRAME_TAG_INDEX;
static constexpr uint8_t SCANNER_SCAN_RESPONSE_PULLS = 24;
// RESULT_GET replies should normally be ready on the very next pull; keep the
// worst-case wait bounded so result-heavy scans do not feel hung.
static constexpr uint8_t SCANNER_RESULT_RESPONSE_PULLS = 8;
// Let a slot drain a little deeper each pass so buffered results clear faster.
static constexpr uint8_t SCANNER_RESULT_DRAIN_BUDGET = 16;
// Fast-fail status polling so one missing slot does not stall the round-robin scheduler.
static constexpr uint8_t SCANNER_RESULT_COUNT_RESPONSE_PULLS = 8;
// The ESP32 SPI slave needs a short gap after each command transaction to
// process the just-received frame and queue the response for the next pull.
// uint32_t, not uint16_t: both delays are documented as overridable via build
// flags and scannerCommandWithResponse() already takes a uint32_t, so a
// narrower type here silently wraps any override above 65535us into a much
// shorter wait rather than failing the build.
static constexpr uint32_t SCANNER_STATUS_FIRST_PULL_US = SCANNER_STATUS_FIRST_PULL_DELAY_US;
static constexpr uint8_t SCANNER_SCAN_CMD_RETRIES = 2;
static constexpr uint8_t SCANNER_QUERY_CMD_RETRIES = 2;
static constexpr uint8_t SCANNER_RESULT_COUNT_CMD_RETRIES = 2;
static constexpr uint8_t SCANNER_RESULT_GET_CMD_RETRIES = 2;
// Keep a short guard gap after CMD_SCAN before the first response pull.
static constexpr uint32_t SCANNER_SCAN_FIRST_PULL_US = SCANNER_SCAN_FIRST_PULL_DELAY_US;
static constexpr uint8_t SCANNER_SCAN_BUSY_CONFIRM_POLLS = 2;
static constexpr uint16_t SCANNER_SCAN_BUSY_CONFIRM_DELAY_US = 2500;
static constexpr uint32_t SCANNER_OTA_SPI_HZ = SCANNER_SPI_UPDATE_CLOCK;
static constexpr uint16_t SCANNER_OTA_FIRST_PULL_US = SCANNER_SPI_UPDATE_FIRST_PULL_US;
static constexpr uint16_t SCANNER_OTA_RESPONSE_PULLS = SCANNER_SPI_UPDATE_RESPONSE_PULLS;
static constexpr uint16_t SCANNER_OTA_INTERFRAME_US = SCANNER_SPI_UPDATE_INTERFRAME_US;
static constexpr uint32_t SCANNER_OTA_PROGRESS_BYTES = 32768;
static constexpr uint16_t SCANNER_OTA_POST_SLOT_DELAY_MS = 250;
static constexpr uint16_t SCANNER_OTA_FLASH_SECTOR_BYTES = SCANNER_SPI_UPDATE_FLASH_SECTOR_BYTES;
// Image verification and commit on the scanner has been seen to take longer
// than 5 s on some units.
static constexpr uint32_t SCANNER_OTA_END_TIMEOUT_MS = 15000;
static constexpr uint32_t SCANNER_OTA_BEGIN_TIMEOUT_MS = 1000;
// A scanner that just committed an image restarts on its next SPI frame and
// needs a moment before it answers ID again.
static constexpr uint16_t SCANNER_OTA_ROUND_DELAY_MS = 1500;
static constexpr uint8_t SCANNER_OTA_ROUNDS = 2;
static constexpr uint8_t SCANNER_UPDATE_LED_RED = 80;
static constexpr uint8_t SCANNER_UPDATE_LED_BLUE = 80;
// Scanner transport hardware config (mapped from legacy config macro names).
static constexpr int SCANNER_PIN_SCK = ESP_PIN_SCK;
static constexpr int SCANNER_PIN_MOSI = ESP_PIN_MOSI;
static constexpr int SCANNER_PIN_MISO = ESP_PIN_MISO;
#if defined(ESP_PIN_CS)
static constexpr int SCANNER_PIN_CS = ESP_PIN_CS;
#endif
static constexpr uint32_t SCANNER_INTERFRAME_US = ESP_INTERFRAME_US;
static constexpr uint8_t SCANNER_DEVICE_TYPE = SCANNER_TYPE_ESP32_C5;
// Local transport status (not on-wire protocol).
static constexpr int8_t SCANNER_STATUS_TRANSPORT_TIMEOUT = -4;
static constexpr uint16_t SCANNER_SLOT_REPROBE_MS = 2000;
static constexpr uint8_t SCANNER_SLOT_TIMEOUT_REPROBE_THRESHOLD = 8;
static constexpr uint16_t SCANNER_SLOT_TIMEOUT_BACKOFF_BASE_MS = 10;
static constexpr uint16_t SCANNER_SLOT_TIMEOUT_BACKOFF_MAX_MS = 200;
static constexpr uint16_t SCANNER_DEDUPE_RESET_RETRY_MS = 100;
static constexpr uint8_t SCANNER_DEDUPE_RESET_MAX_ATTEMPTS = 3;
#if SCANNER_USE_SHIFTREG_CS
static constexpr uint8_t SCANNER_SLOT_COUNT = SCANNER_SHIFTREG_OUTPUTS;
#else
static constexpr uint8_t SCANNER_SLOT_COUNT = 1;
#endif

struct ScannerSlotState {
  // Cached identity state for this physical slot.
  bool id_valid;
  // One-shot log suppressors while the slot remains in same state.
  bool miss_logged;
  bool unsupported_logged;
  DeviceIdReply id;
  // Re-probe scheduling so absent slots do not get hammered every loop.
  uint32_t next_probe_ms;
  // Backoff for temporary transport misses while slot remains identified.
  uint32_t next_service_ms;
  // One-time per-identification scanner dedupe reset handshake.
  bool dedupe_reset_done;
  uint32_t next_dedupe_reset_ms;
  uint8_t dedupe_reset_failures;
  // Runtime dedupe resets are deferred until the slot is idle so they do not
  // preempt normal scan/result transport after GNSS lock transitions.
  bool runtime_dedupe_reset_pending;
  uint32_t next_runtime_dedupe_reset_ms;
  uint8_t runtime_dedupe_reset_failures;
  // Transport timeout streak for targeted stale-frame draining.
  uint8_t transport_timeouts;
};

// Persistent core1 runtime state. controller_main.cpp owns the shared queues and
// counters, but the scanner runtime owns the active slot machine and sweep state.
static ControllerScannerRuntimeContext scanner_runtime_context = {};
static uint8_t scanner_cs_shiftreg_state = 0xFF;
static uint8_t scanner_active_slot = SCANNER_INITIAL_SLOT;
static bool scanner_dedupe_reset_requested = false;
#if SCANNER_USE_SHIFTREG_CS
static bool scanner_shiftreg_outputs_enabled = false;
#endif
static ScannerSlotState scanner_slot_state[SCANNER_SLOT_COUNT] = {};
// A single global channel dispatch plan is shared across all scanner slots.
static ChannelScheduleState scanner_schedule_state = {};
static bool scanner_sweep_timing_active = false;
static uint32_t scanner_sweep_started_ms = 0;
static uint8_t scanner_slot = SCANNER_INITIAL_SLOT;
static bool scanner_ota_transport_active = false;
// Guards the monitor-mode hand-off between core0 and core1.
static critical_section_t monitor_link_lock;

// SPI1 carries both the shift-register CS updates and scanner traffic at
// different clocks. SPIClass::beginTransaction re-initialises (resets) the
// peripheral whenever the clock changes, which cost ~68 us per switch and
// dominated every frame, so SPI1 clocks are switched by writing precomputed
// prescaler registers instead.
struct SpiClockRegs {
  uint32_t cpsr;
  uint32_t scr;
};

static SpiClockRegs spi_clock_shiftreg = {};
static SpiClockRegs spi_clock_scanner = {};
static SpiClockRegs spi_clock_ota = {};
static SpiClockRegs spi_clock_bulk = {};
static const SpiClockRegs* spi_clock_current = nullptr;

static SpiClockRegs scannerSpiCaptureClock(uint32_t hz) {
  spi_set_baudrate(spi1, hz);
  const spi_hw_t* hw = spi_get_hw(spi1);
  return {hw->cpsr, hw->cr0 & SPI_SSPCR0_SCR_BITS};
}

static void scannerSpiClockInit() {
  // One Arduino transaction puts SPI1 in 8-bit mode 0; after that the clock
  // is only ever changed through scannerSpiSelectClock().
  SPI1.beginTransaction(SPISettings(SCANNER_SPI_CLOCK, MSBFIRST, SPI_MODE0));
  SPI1.endTransaction();
  spi_clock_shiftreg = scannerSpiCaptureClock(SCANNER_SHIFTREG_SPI_HZ);
  spi_clock_ota = scannerSpiCaptureClock(SCANNER_SPI_UPDATE_CLOCK);
  spi_clock_bulk = scannerSpiCaptureClock(SCANNER_SPI_BULK_CLOCK);
  spi_clock_scanner = scannerSpiCaptureClock(SCANNER_SPI_CLOCK);
  spi_clock_current = &spi_clock_scanner;
}

static inline void scannerSpiSelectClock(const SpiClockRegs& clock) {
  if (spi_clock_current == &clock) {
    return;
  }
  spi_hw_t* const hw = spi_get_hw(spi1);
  while (spi_is_busy(spi1)) {
  }
  hw->cpsr = clock.cpsr;
  hw_write_masked(&hw->cr0, clock.scr, SPI_SSPCR0_SCR_BITS);
  spi_clock_current = &clock;
}

static void scannerSerialQueueTry(const char* text) {
  if (scanner_runtime_context.serial_queue_try != nullptr) {
    scanner_runtime_context.serial_queue_try(text);
  }
}

static void scannerSerialPrintlnTry(const char* s) {
#if !CORE1_SERIAL_LOG
  (void)s;
  return;
#else
  char buf[192];
  snprintf(buf, sizeof(buf), "%s\n", s);
  scannerSerialQueueTry(buf);
#endif
}

static void scannerSerialPrintfTry(const char* fmt, ...) {
#if !CORE1_SERIAL_LOG
  (void)fmt;
  return;
#else
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  scannerSerialQueueTry(buf);
#endif
}

void controllerScannerRuntimeRequestDedupeReset() {
  __atomic_store_n(&scanner_dedupe_reset_requested, true, __ATOMIC_RELEASE);
}

// Sweep timing starts with the first scheduled channel after the previous
// coverage cycle completes.
static inline void sweepTimingNoteStart(bool& timing_active,
                                        uint32_t& started_ms) {
  if (!timing_active) {
    started_ms = millis();
    timing_active = true;
  }
}

static inline void scannerSetActiveSlot(uint8_t slot) {
#if SCANNER_USE_SHIFTREG_CS
  // The active slot selects which 74HC595 output will assert CS for the next
  // transfer. With direct CS wiring there is only one logical slot.
  scanner_active_slot = (uint8_t)(slot % SCANNER_SLOT_COUNT);
#else
  (void)slot;
  scanner_active_slot = 0;
#endif
}

#if SCANNER_USE_SHIFTREG_CS
static inline void scannerShiftRegSetOutputsEnabled(bool enabled) {
  scanner_shiftreg_outputs_enabled = enabled;
  // 74HC595 OE is active-low.
  digitalWrite(SCANNER_SHIFTREG_OE_PIN, enabled ? LOW : HIGH);
}

static inline void scannerShiftRegWriteCsState(uint8_t state, bool force) {
  // Updating the shift register itself uses SPI1, so temporarily disable the
  // scanner outputs to avoid clocking garbage into a selected slave.
  if (!force && scanner_cs_shiftreg_state == state) {
    return;
  }
  const bool restore_outputs = scanner_shiftreg_outputs_enabled;
  if (restore_outputs) {
    // Prevent SPI clocks for shift-register updates from reaching any selected scanner.
    scannerShiftRegSetOutputsEnabled(false);
  }
  scanner_cs_shiftreg_state = state;
  scannerSpiSelectClock(spi_clock_shiftreg);
  spi_write_blocking(spi1, &scanner_cs_shiftreg_state, 1);
  digitalWrite(SCANNER_SHIFTREG_LATCH_PIN, HIGH);
  digitalWrite(SCANNER_SHIFTREG_LATCH_PIN, LOW);
  if (restore_outputs) {
    scannerShiftRegSetOutputsEnabled(true);
  }
}
#endif

static inline void advanceSweepChannelAndLog(ChannelScheduleState& schedule_state,
                                             bool& timing_active,
                                             uint32_t& started_ms) {
  // The scheduler is global across all slots. A coverage cycle completes once
  // both bands have wrapped at least once under the mixed-band dispatch plan.
  if (channelScheduleAdvance(schedule_state)) {
    if (timing_active) {
      const uint32_t elapsed_ms = millis() - started_ms;
      *scanner_runtime_context.last_full_sweep_ms = elapsed_ms;
      (*scanner_runtime_context.sweep_cycles_completed)++;
      scannerSerialPrintfTry("Completed full 2.4G + 5G coverage cycle in %lums\n",
                             (unsigned long)elapsed_ms);
      timing_active = false;
      return;
    }
    scannerSerialPrintlnTry("Completed full 2.4G + 5G coverage cycle");
  }
}

// SPI1 data moves by DMA: one channel feeds the TX FIFO, one drains RX. Core1
// waits for completion, but no longer spends a call per byte.
static int scanner_dma_tx = -1;
static int scanner_dma_rx = -1;
static uint8_t scanner_dma_zero = 0;
static uint8_t scanner_dma_sink = 0;

static void scannerSpiDmaInit() {
  scanner_dma_tx = dma_claim_unused_channel(true);
  scanner_dma_rx = dma_claim_unused_channel(true);
}

// tx == nullptr clocks zeros (NOP bytes); rx == nullptr discards.
static void scannerSpiDmaTransfer(const uint8_t* tx, uint8_t* rx, size_t len) {
  spi_inst_t* const spi = spi1;
  spi_hw_t* const hw = spi_get_hw(spi);
  while (spi_is_readable(spi)) {
    (void)hw->dr;
  }
  hw->icr = SPI_SSPICR_RORIC_BITS;

  dma_channel_config c = dma_channel_get_default_config(scanner_dma_tx);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_read_increment(&c, tx != nullptr);
  channel_config_set_write_increment(&c, false);
  channel_config_set_dreq(&c, spi_get_dreq(spi, true));
  dma_channel_configure(scanner_dma_tx, &c, &hw->dr, tx ? tx : &scanner_dma_zero, len, false);

  c = dma_channel_get_default_config(scanner_dma_rx);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, rx != nullptr);
  channel_config_set_dreq(&c, spi_get_dreq(spi, false));
  dma_channel_configure(scanner_dma_rx, &c, rx ? rx : &scanner_dma_sink, &hw->dr, len, false);

  hw->dmacr = SPI_SSPDMACR_TXDMAE_BITS | SPI_SSPDMACR_RXDMAE_BITS;
  dma_start_channel_mask((1u << scanner_dma_tx) | (1u << scanner_dma_rx));
  // RX completing means every byte has been clocked both ways.
  dma_channel_wait_for_finish_blocking(scanner_dma_rx);
  hw->dmacr = 0;
}

// A scanner re-arms its slave transaction from its main loop after every
// completed one; a frame arriving inside that window is lost. The protocol
// used to get this spacing for free from slow per-frame SPI setup, so enforce
// it explicitly per slot (other slots may use the bus meanwhile).
static constexpr uint32_t SCANNER_MIN_FRAME_GAP_US = 150;
static uint32_t scanner_slot_last_xfer_us[SCANNER_SLOT_COUNT] = {};

static void scannerTransferBytes(const uint8_t* tx, uint8_t* rx, size_t len,
                                 const SpiClockRegs& clock) {
  const uint8_t gap_slot = scanner_active_slot < SCANNER_SLOT_COUNT ? scanner_active_slot : 0;
  while (time_us_32() - scanner_slot_last_xfer_us[gap_slot] < SCANNER_MIN_FRAME_GAP_US) {
  }
#if SCANNER_USE_SHIFTREG_CS
  // Active-low CS via shift-register output slot.
  const uint8_t assert_state = (uint8_t)(0xFFu & ~(1u << scanner_active_slot));
  scannerShiftRegWriteCsState(assert_state, false);
#else
  digitalWrite(SCANNER_PIN_CS, LOW);
#endif
  scannerSpiSelectClock(clock);
  scannerSpiDmaTransfer(tx, rx, len);
  scanner_slot_last_xfer_us[gap_slot] = time_us_32();
#if SCANNER_USE_SHIFTREG_CS
  // Deassert all scanner CS outputs.
  const uint8_t deassert_state = 0xFFu;
  scannerShiftRegWriteCsState(deassert_state, false);
#else
  digitalWrite(SCANNER_PIN_CS, HIGH);
#endif
}

// Every command and poll is exactly one 64-byte full-duplex frame.
static void scannerTransferFrame(const uint8_t* tx, uint8_t* rx) {
  scannerTransferBytes(tx, rx, SCANNER_FRAME_SIZE,
                       scanner_ota_transport_active ? spi_clock_ota : spi_clock_scanner);
}

// A scanner left waiting for a batch read (for example by a controller reset
// mid-exchange) has a transaction of up to PKT_BATCH_MAX_BYTES queued. Clocking
// past that length completes it by length, which the scanner always accepts,
// and costs an idle scanner only one NOP frame.
static void scannerReleaseBulkWait() {
  scannerTransferBytes(nullptr, nullptr, PKT_BATCH_MAX_BYTES + SCANNER_FRAME_SIZE,
                       spi_clock_scanner);
  delayMicroseconds(200);
}

static inline void scannerInterframeWait() {
  // Some scanner boards need a short turnaround gap between command and poll
  // frames to let the response buffer settle.
  if (SCANNER_INTERFRAME_US > 0) {
    delayMicroseconds(SCANNER_INTERFRAME_US);
  }
}

static inline void scannerOtaInterframeWait() {
  if (SCANNER_OTA_INTERFRAME_US > 0) {
    delayMicroseconds(SCANNER_OTA_INTERFRAME_US);
  } else {
    scannerInterframeWait();
  }
}

// Issue command frame, then pull NOP frames until response tag matches expected cmd.
static bool scannerCommandWithResponse(const uint8_t* cmd_frame,
                                       uint8_t expected_cmd_tag,
                                       uint8_t* response_frame,
                                       int max_pulls = 8,
                                       uint32_t first_pull_wait_us = SCANNER_STATUS_FIRST_PULL_US) {
  // Most commands are acknowledged asynchronously: send once, then keep pulling
  // NOP frames until the scanner echoes the expected command tag.
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  nop[0] = CMD_NOP;
  scannerTransferFrame(cmd_frame, rx);
  if (first_pull_wait_us > 0) {
    delayMicroseconds(first_pull_wait_us);
  }

  for (int i = 0; i < max_pulls; i++) {
    scannerTransferFrame(nop, response_frame);
    if (response_frame[SCANNER_FRAME_TAG_INDEX] == expected_cmd_tag) {
      return true;
    }
    scannerInterframeWait();
  }
  return false;
}

static void scannerDrainStaleFrames(int pulls = 4) {
  // Drain leftover reply frames when we switch slots or recover from malformed
  // traffic so a stale response is less likely to be mis-read as the next ACK.
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  nop[0] = CMD_NOP;
  for (int i = 0; i < pulls; i++) {
    scannerTransferFrame(nop, rx);
    scannerInterframeWait();
  }
}

static void handlePendingDedupeResetRequest() {
  const bool was_set = __atomic_exchange_n(&scanner_dedupe_reset_requested, false, __ATOMIC_ACQ_REL);
  if (!was_set) {
    return;
  }

  if (scanner_runtime_context.master_dedupe_table != nullptr) {
    wifiDedupeTableReset(scanner_runtime_context.master_dedupe_table);
  }

  for (uint8_t slot = 0; slot < SCANNER_SLOT_COUNT; slot++) {
    ScannerSlotState& state = scanner_slot_state[slot];
    state.runtime_dedupe_reset_pending = true;
    state.next_runtime_dedupe_reset_ms = 0;
  }

  scannerSerialPrintlnTry("Controller dedupe reset requested (deferred until slot idle)");
}

void controllerScannerRuntimeInitBus() {
  // Bring the scanner bus to a safe idle state before core1 starts polling.
#if SCANNER_USE_SHIFTREG_CS
  pinMode(SCANNER_SHIFTREG_LATCH_PIN, OUTPUT);
  pinMode(SCANNER_SHIFTREG_OE_PIN, OUTPUT);
  digitalWrite(SCANNER_SHIFTREG_LATCH_PIN, LOW);
  // Active-low OE: keep outputs disabled while shift register state is initialized.
  scannerShiftRegSetOutputsEnabled(false);
#else
  digitalWrite(SCANNER_PIN_CS, HIGH);
  pinMode(SCANNER_PIN_CS, OUTPUT);
#endif
#if SCANNER_MISO_PULLUP
  gpio_pull_up(SCANNER_PIN_MISO);
#endif
  SPI1.setSCK(SCANNER_PIN_SCK);
  SPI1.setTX(SCANNER_PIN_MOSI);
  SPI1.setRX(SCANNER_PIN_MISO);
  SPI1.begin();
  scannerSpiDmaInit();
  scannerSpiClockInit();
  critical_section_init(&monitor_link_lock);
#if SCANNER_MISO_PULLUP
  gpio_pull_up(SCANNER_PIN_MISO);
#endif
#if SCANNER_USE_SHIFTREG_CS
  scannerSetActiveSlot(SCANNER_INITIAL_SLOT);
  scannerShiftRegWriteCsState(0xFFu, true);
  // Keep outputs enabled in idle once all scanner CS lines are safely deasserted.
  scannerShiftRegSetOutputsEnabled(true);
#endif
}

static bool scannerGetDeviceId(DeviceIdReply& out) {
  uint8_t tx[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};

  // Empty or still-booting slots can return garbage; probe a few times before
  // deciding the slot is absent.
  for (int attempt = 0; attempt < 8; attempt++) {
    tx[0] = CMD_ID;
    if (scannerCommandWithResponse(tx, CMD_ID, rx, 10)) {
      memcpy(&out, rx, sizeof(out));
      if (out.proto_version == PROTO_VERSION) {
        return true;
      }
    }
    delay(1);
  }
  return false;
}

static const char* scannerOtaStatusToString(int8_t status) {
  switch (status) {
    case OTA_STATUS_OK: return "OK";
    case OTA_STATUS_BUSY: return "BUSY";
    case OTA_STATUS_INVALID: return "INVALID";
    case OTA_STATUS_NOT_ACTIVE: return "NOT_ACTIVE";
    case OTA_STATUS_SEQUENCE_MISMATCH: return "SEQUENCE_MISMATCH";
    case OTA_STATUS_CRC_MISMATCH: return "CRC_MISMATCH";
    case OTA_STATUS_WRITE_FAILED: return "WRITE_FAILED";
    case OTA_STATUS_SIZE_MISMATCH: return "SIZE_MISMATCH";
    case OTA_STATUS_HASH_MISMATCH: return "HASH_MISMATCH";
    case OTA_STATUS_BEGIN_FAILED: return "BEGIN_FAILED";
    case OTA_STATUS_END_FAILED: return "END_FAILED";
    default: return "UNKNOWN";
  }
}

static void showScannerUpdateLed(Adafruit_NeoPixel& pixels) {
  pixels.setPixelColor(0, pixels.Color(SCANNER_UPDATE_LED_RED,
                                       0,
                                       SCANNER_UPDATE_LED_BLUE));
  pixels.show();
}

static bool scannerOtaCommandWithAck(const uint8_t* cmd_frame,
                                     uint8_t expected_cmd_tag,
                                     OtaAck& ack,
                                     uint16_t max_pulls = SCANNER_OTA_RESPONSE_PULLS,
                                     uint32_t expected_offset = UINT32_MAX,
                                     uint16_t expected_next_sequence = UINT16_MAX,
                                     bool require_tag = false) {
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  nop[0] = CMD_NOP;

  auto frameLooksLikeAck = [&](const uint8_t* frame) -> bool {
    OtaAck candidate = {};
    memcpy(&candidate, frame, sizeof(candidate));
    const bool tag_match = frame[SCANNER_FRAME_TAG_INDEX] == expected_cmd_tag;
    const bool payload_match =
        candidate.active != 0 &&
        candidate.status <= OTA_STATUS_OK &&
        candidate.status >= OTA_STATUS_END_FAILED &&
        (expected_offset == UINT32_MAX || candidate.offset == expected_offset) &&
        (expected_next_sequence == UINT16_MAX ||
         candidate.next_sequence == expected_next_sequence);

    if (tag_match || (payload_match && !require_tag)) {
      ack = candidate;
      return true;
    }
    return false;
  };

  scannerTransferFrame(cmd_frame, rx);
  if (frameLooksLikeAck(rx)) {
    return true;
  }
  if (SCANNER_OTA_FIRST_PULL_US > 0) {
    delayMicroseconds(SCANNER_OTA_FIRST_PULL_US);
  }

  for (uint16_t pull = 0; pull < max_pulls; pull++) {
    scannerTransferFrame(nop, rx);
    if (frameLooksLikeAck(rx)) {
      return true;
    }
    scannerOtaInterframeWait();
  }

  return false;
}

static bool scannerOtaAbortActiveTransfer() {
  uint8_t tx[SCANNER_FRAME_SIZE] = {0};
  OtaAck ack = {};
  tx[0] = CMD_OTA_ABORT;
  return scannerOtaCommandWithAck(tx, CMD_OTA_ABORT, ack, 80) &&
         ack.status == OTA_STATUS_OK;
}

static bool scannerOtaReadImageHash(FsFile& source,
                                    uint32_t image_size,
                                    uint64_t& image_hash,
                                    Stream& serial) {
  if (!source.seekSet(0)) {
    serial.println("scanner.bin seek failed before hashing");
    return false;
  }

  uint8_t buffer[SCANNER_SPI_UPDATE_FILE_CHUNK_BYTES] = {};
  uint64_t hash = OTA_HASH64_INIT;
  uint32_t total_read = 0;
  while (total_read < image_size) {
    const uint32_t remaining = image_size - total_read;
    const size_t want = (remaining < sizeof(buffer)) ? remaining : sizeof(buffer);
    const int bytes_read = source.read(buffer, want);
    if (bytes_read <= 0) {
      serial.println("scanner.bin read failed while hashing");
      return false;
    }
    hash = otaHash64Update(hash, buffer, static_cast<size_t>(bytes_read));
    total_read += static_cast<uint32_t>(bytes_read);
  }

  image_hash = hash;
  return source.seekSet(0);
}

// Poll NOPs on a wall-clock deadline until a frame tagged with cmd_tag arrives.
static bool scannerOtaAwaitTaggedAck(uint8_t cmd_tag, OtaAck& ack, uint32_t timeout_ms) {
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  nop[0] = CMD_NOP;
  const uint32_t deadline = millis() + timeout_ms;
  while ((int32_t)(millis() - deadline) < 0) {
    delay(2);
    scannerTransferFrame(nop, rx);
    if (rx[SCANNER_FRAME_TAG_INDEX] == cmd_tag) {
      memcpy(&ack, rx, sizeof(ack));
      return true;
    }
  }
  return false;
}

// A scanner left in monitor mode by a previous controller session services SPI
// on a short timeout; return it to normal mode before starting an update.
static void scannerOtaStopMonitor() {
  uint8_t tx[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  tx[0] = CMD_MONITOR_STOP;
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    if (scannerCommandWithResponse(tx, CMD_MONITOR_STOP, rx, 20, SCANNER_STATUS_FIRST_PULL_US)) {
      return;
    }
    delay(5);
  }
}

static bool scannerOtaSendBegin(uint32_t image_size,
                                uint64_t image_hash,
                                Stream& serial,
                                uint8_t slot) {
  OtaBeginCommand begin = {};
  begin.cmd = CMD_OTA_BEGIN;
  begin.packet_data_bytes = OTA_DATA_BYTES_PER_FRAME;
  begin.image_size = image_size;
  begin.image_hash = image_hash;

  // BEGIN is safe to resend: the scanner aborts any partial session first.
  OtaAck ack = {};
  bool acked = false;
  for (uint8_t attempt = 0; attempt < 3 && !acked; attempt++) {
    acked = scannerOtaCommandWithAck(reinterpret_cast<const uint8_t*>(&begin),
                                     CMD_OTA_BEGIN,
                                     ack,
                                     SCANNER_OTA_RESPONSE_PULLS,
                                     0,
                                     0,
                                     true) ||
            scannerOtaAwaitTaggedAck(CMD_OTA_BEGIN, ack, SCANNER_OTA_BEGIN_TIMEOUT_MS);
  }
  if (!acked) {
    serial.print("S");
    serial.print(slot);
    serial.println(" scanner OTA begin timed out");
    return false;
  }
  if (ack.status != OTA_STATUS_OK) {
    serial.print("S");
    serial.print(slot);
    serial.print(" scanner OTA begin rejected: ");
    serial.println(scannerOtaStatusToString(ack.status));
    return false;
  }
  return true;
}

static bool scannerOtaShouldLogPacket(uint32_t offset, uint8_t length);

static bool scannerOtaSendDataPacket(const OtaDataCommand& packet,
                                     uint32_t next_offset,
                                     Stream& serial,
                                     uint8_t slot) {
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  nop[0] = CMD_NOP;

  for (uint8_t attempt = 0; attempt < SCANNER_SPI_UPDATE_PACKET_RETRIES; attempt++) {
    uint8_t rx[SCANNER_FRAME_SIZE] = {0};
    OtaAck ack = {};
    const auto frameLooksLikeDataAck = [&](const uint8_t* frame) -> bool {
      OtaAck candidate = {};
      memcpy(&candidate, frame, sizeof(candidate));
      const bool tag_match = frame[SCANNER_FRAME_TAG_INDEX] == CMD_OTA_DATA;
      const bool payload_match =
          candidate.active != 0 &&
          candidate.status <= OTA_STATUS_OK &&
          candidate.status >= OTA_STATUS_END_FAILED &&
          candidate.offset == next_offset &&
          (candidate.next_sequence == static_cast<uint16_t>(packet.sequence + 1) ||
           candidate.status == OTA_STATUS_SEQUENCE_MISMATCH);
      if (tag_match || payload_match) {
        ack = candidate;
        return true;
      }
      return false;
    };

    scannerTransferFrame(reinterpret_cast<const uint8_t*>(&packet), rx);
    if (frameLooksLikeDataAck(rx)) {
      if (ack.status == OTA_STATUS_OK && ack.offset == next_offset) {
        return true;
      }
      if (ack.status == OTA_STATUS_SEQUENCE_MISMATCH && ack.offset == next_offset) {
        return true;
      }
      // Same command tag is used for every OTA data packet, so a delayed ACK
      // for the prior packet can arrive here. Keep polling unless the scanner
      // reports a non-recoverable error for this transfer.
      if (ack.status != OTA_STATUS_OK &&
          ack.status != OTA_STATUS_CRC_MISMATCH &&
          ack.status != OTA_STATUS_SEQUENCE_MISMATCH) {
        serial.print("S");
        serial.print(slot);
        serial.print(" scanner OTA data failed at offset ");
        serial.print(packet.offset);
        serial.print(": ");
        serial.println(scannerOtaStatusToString(ack.status));
        return false;
      }
    }

    if (SCANNER_OTA_FIRST_PULL_US > 0) {
      delayMicroseconds(SCANNER_OTA_FIRST_PULL_US);
    }

    for (uint16_t pull = 0; pull < SCANNER_OTA_RESPONSE_PULLS; pull++) {
      scannerTransferFrame(nop, rx);
      if (!frameLooksLikeDataAck(rx)) {
        scannerOtaInterframeWait();
        continue;
      }

      if (ack.status == OTA_STATUS_OK && ack.offset == next_offset) {
        if (scannerOtaShouldLogPacket(packet.offset, packet.length)) {
          serial.print("S");
          serial.print(slot);
          serial.print(" scanner OTA ACK data seq=");
          serial.print(packet.sequence);
          serial.print(" offset=");
          serial.print(packet.offset);
          serial.print(" after_pulls=");
          serial.println(pull + 1);
        }
        return true;
      }

      // A lost ACK can make the controller retry a packet the scanner already
      // accepted. Treat the scanner's advanced offset as idempotent success.
      if (ack.status == OTA_STATUS_SEQUENCE_MISMATCH && ack.offset == next_offset) {
        return true;
      }

      if (ack.status == OTA_STATUS_CRC_MISMATCH ||
          ack.status == OTA_STATUS_SEQUENCE_MISMATCH ||
          (ack.status == OTA_STATUS_OK && ack.offset < next_offset)) {
        scannerOtaInterframeWait();
        continue;
      }

      serial.print("S");
      serial.print(slot);
      serial.print(" scanner OTA data failed at offset ");
      serial.print(packet.offset);
      serial.print(": ");
      serial.println(scannerOtaStatusToString(ack.status));
      return false;
    }
  }

  serial.print("S");
  serial.print(slot);
  serial.print(" scanner OTA data timed out at offset ");
  serial.println(packet.offset);
  return false;
}

// The scanner verifies and commits the image inside its END handler before it
// re-arms SPI, which takes far longer than the fast pull budget. Keep polling
// on a wall-clock deadline for the END-tagged verdict.
static bool scannerOtaAwaitEndAck(OtaAck& ack) {
  return scannerOtaAwaitTaggedAck(CMD_OTA_END, ack, SCANNER_OTA_END_TIMEOUT_MS);
}

static bool scannerOtaSendEnd(uint32_t image_size,
                              uint64_t image_hash,
                              Stream& serial,
                              uint8_t slot) {
  OtaEndCommand end = {};
  end.cmd = CMD_OTA_END;
  end.image_size = image_size;
  end.image_hash = image_hash;

  // A repeated DATA ack for the final packet carries the same offset, so a
  // payload-only match would mistake it (often SEQUENCE_MISMATCH from a
  // retried last packet) for the END verdict. Only accept the END tag.
  OtaAck ack = {};
  if (!scannerOtaCommandWithAck(reinterpret_cast<const uint8_t*>(&end),
                                CMD_OTA_END,
                                ack,
                                SCANNER_OTA_RESPONSE_PULLS,
                                image_size,
                                UINT16_MAX,
                                true) &&
      !scannerOtaAwaitEndAck(ack)) {
    serial.print("S");
    serial.print(slot);
    serial.println(" scanner OTA end timed out");
    return false;
  }
  if (ack.status != OTA_STATUS_OK) {
    serial.print("S");
    serial.print(slot);
    serial.print(" scanner OTA end rejected: ");
    serial.println(scannerOtaStatusToString(ack.status));
    return false;
  }
  return true;
}

static bool scannerOtaShouldLogPacket(uint32_t offset, uint8_t length) {
  if (offset < 512) {
    return true;
  }
#if SCANNER_SPI_UPDATE_FLASH_SECTOR_BYTES == 0
  (void)length;
  return false;
#else
  const uint32_t sector_offset = offset % SCANNER_OTA_FLASH_SECTOR_BYTES;
  const uint32_t end_offset = (offset + length) % SCANNER_OTA_FLASH_SECTOR_BYTES;
  return sector_offset < 128 ||
         sector_offset >= (SCANNER_OTA_FLASH_SECTOR_BYTES - 128) ||
         end_offset < 128;
#endif
}

static bool scannerOtaTransferImageToSlot(SdFat& sd,
                                          uint8_t slot,
                                          uint32_t image_size,
                                          uint64_t image_hash,
                                          Stream& serial) {
  scanner_ota_transport_active = true;

  FsFile source = sd.open(SCANNER_SPI_UPDATE_PATH, O_RDONLY);
  if (!source) {
    serial.println("scanner.bin disappeared before scanner OTA");
    scanner_ota_transport_active = false;
    return false;
  }
  if (!source.seekSet(0)) {
    serial.println("scanner.bin seek failed before scanner OTA");
    source.close();
    scanner_ota_transport_active = false;
    return false;
  }

  scannerSetActiveSlot(slot);
  scannerDrainStaleFrames(2);

  if (!scannerOtaSendBegin(image_size, image_hash, serial, slot)) {
    source.close();
    scanner_ota_transport_active = false;
    return false;
  }

  uint32_t offset = 0;
  uint16_t sequence = 0;
  uint32_t next_progress = SCANNER_OTA_PROGRESS_BYTES;
  while (offset < image_size) {
    OtaDataCommand packet = {};
    packet.cmd = CMD_OTA_DATA;
    packet.sequence = sequence;
    packet.offset = offset;

    const uint32_t remaining = image_size - offset;
    uint32_t packet_limit = OTA_DATA_BYTES_PER_FRAME;
#if SCANNER_SPI_UPDATE_FLASH_SECTOR_BYTES != 0
    const uint32_t sector_offset = offset % SCANNER_OTA_FLASH_SECTOR_BYTES;
    if (sector_offset != 0) {
      const uint32_t bytes_to_sector_end =
          SCANNER_OTA_FLASH_SECTOR_BYTES - sector_offset;
      if (bytes_to_sector_end < packet_limit) {
        packet_limit = bytes_to_sector_end;
      }
    }
#endif
    const uint8_t want = (remaining < packet_limit)
      ? static_cast<uint8_t>(remaining)
      : static_cast<uint8_t>(packet_limit);
    const int bytes_read = source.read(packet.data, want);
    if (bytes_read != want) {
      serial.print("S");
      serial.print(slot);
      serial.println(" scanner.bin read failed during scanner OTA");
      scannerOtaAbortActiveTransfer();
      source.close();
      scanner_ota_transport_active = false;
      return false;
    }

    packet.length = want;
    packet.crc32 = otaCrc32(packet.data, packet.length);
    const uint32_t next_offset = offset + packet.length;
    if (scannerOtaShouldLogPacket(packet.offset, packet.length)) {
      serial.print("S");
      serial.print(slot);
      serial.print(" scanner OTA TX data seq=");
      serial.print(packet.sequence);
      serial.print(" offset=");
      serial.print(packet.offset);
      serial.print(" len=");
      serial.print(packet.length);
#if SCANNER_SPI_UPDATE_FLASH_SECTOR_BYTES != 0
      serial.print(" sector_off=");
      serial.print(packet.offset % SCANNER_OTA_FLASH_SECTOR_BYTES);
#endif
      serial.print(" next=");
      serial.print(next_offset);
      serial.print(" crc=0x");
      serial.println(packet.crc32, HEX);
    }
    if (!scannerOtaSendDataPacket(packet, next_offset, serial, slot)) {
      scannerOtaAbortActiveTransfer();
      source.close();
      scanner_ota_transport_active = false;
      return false;
    }

    offset = next_offset;
    sequence++;

    if (offset >= next_progress || offset == image_size) {
      serial.print("S");
      serial.print(slot);
      serial.print(" scanner OTA progress ");
      serial.print(offset);
      serial.print("/");
      serial.println(image_size);
      while (next_progress <= offset) {
        next_progress += SCANNER_OTA_PROGRESS_BYTES;
      }
    }
  }

  source.close();

  if (!scannerOtaSendEnd(image_size, image_hash, serial, slot)) {
    scanner_ota_transport_active = false;
    return false;
  }

  delay(SCANNER_OTA_POST_SLOT_DELAY_MS);
  scanner_ota_transport_active = false;
  return true;
}

static bool buildScannerAppliedPath(SdFat& sd, char* out, size_t out_len) {
  const char* const applied_base = SCANNER_SPI_UPDATE_APPLIED_PATH;
  if (!sd.exists(applied_base)) {
    snprintf(out, out_len, "%s", applied_base);
    return true;
  }

  const char* const slash = strrchr(applied_base, '/');
  const char* const dot = strrchr(applied_base, '.');
  const bool has_extension = (dot != nullptr) && (slash == nullptr || dot > slash);

  for (uint16_t suffix = 1; suffix < 100; suffix++) {
    if (has_extension) {
      const int stem_len = static_cast<int>(dot - applied_base);
      snprintf(out, out_len, "%.*s_%02u%s",
               stem_len, applied_base, (unsigned)suffix, dot);
    } else {
      snprintf(out, out_len, "%s_%02u", applied_base, (unsigned)suffix);
    }

    if (!sd.exists(out)) {
      return true;
    }
  }

  out[0] = '\0';
  return false;
}

bool controllerScannerRuntimeHandleScannerUpdatesFromSd(SdFat& sd,
                                                        bool sd_ready,
                                                        Stream& serial,
                                                        Adafruit_NeoPixel& pixels) {
#if !SCANNER_SPI_UPDATE_ENABLED
  (void)sd;
  (void)sd_ready;
  (void)serial;
  (void)pixels;
  return false;
#else
  if (!sd_ready || !sd.exists(SCANNER_SPI_UPDATE_PATH)) {
    return false;
  }

  FsFile source = sd.open(SCANNER_SPI_UPDATE_PATH, O_RDONLY);
  if (!source) {
    serial.println("scanner.bin exists but could not be opened");
    return false;
  }

  const uint32_t image_size = static_cast<uint32_t>(source.size());
  if (image_size == 0) {
    serial.println("scanner.bin is empty; skipping scanner OTA");
    source.close();
    return false;
  }

  uint64_t image_hash = 0;
  if (!scannerOtaReadImageHash(source, image_size, image_hash, serial)) {
    source.close();
    return false;
  }
  source.close();

  showScannerUpdateLed(pixels);

  serial.print("Scanner SPI OTA image: ");
  serial.print(image_size);
  serial.print(" bytes hash=0x");
  serial.print(static_cast<uint32_t>(image_hash >> 32), HEX);
  serial.println(static_cast<uint32_t>(image_hash & 0xffffffffUL), HEX);

  // A slot that does not answer ID may be empty, or may be a scanner that is
  // restarting (one that committed an image restarts on its next SPI frame).
  // Neither can be told apart at probe time, so run a second round after a
  // pause: it picks up late responders and retries failures. Only slots that
  // stay silent through both rounds are treated as empty.
  enum class SlotOtaResult : uint8_t { ABSENT, UNSUPPORTED, UPDATED, FAILED };
  SlotOtaResult results[SCANNER_SLOT_COUNT] = {};
  for (uint8_t round = 0; round < SCANNER_OTA_ROUNDS; round++) {
    bool pending = false;
    for (uint8_t slot = 0; slot < SCANNER_SLOT_COUNT; slot++) {
      if (results[slot] == SlotOtaResult::ABSENT || results[slot] == SlotOtaResult::FAILED) {
        pending = true;
      }
    }
    if (!pending) {
      break;
    }
    if (round > 0) {
      delay(SCANNER_OTA_ROUND_DELAY_MS);
    }

    for (uint8_t slot = 0; slot < SCANNER_SLOT_COUNT; slot++) {
      if (results[slot] == SlotOtaResult::UPDATED ||
          results[slot] == SlotOtaResult::UNSUPPORTED) {
        continue;
      }
      scannerSetActiveSlot(slot);
      scannerReleaseBulkWait();
      scannerDrainStaleFrames(2);

      DeviceIdReply id = {};
      if (!scannerGetDeviceId(id)) {
        if (results[slot] == SlotOtaResult::FAILED) {
          serial.print("S");
          serial.print(slot);
          serial.println(" scanner stopped answering after a failed OTA");
        }
        continue;
      }
      if ((id.capabilities & CAP_OTA_UPDATE) == 0) {
        serial.print("S");
        serial.print(slot);
        serial.println(" scanner does not advertise SPI OTA; skipping");
        results[slot] = SlotOtaResult::UNSUPPORTED;
        continue;
      }

      if ((id.capabilities & CAP_MONITOR) != 0) {
        scannerOtaStopMonitor();
      }
      serial.print("S");
      serial.print(slot);
      serial.print(round == 0 ? " scanner OTA starting" : " scanner OTA retry starting");
      serial.println(results[slot] == SlotOtaResult::ABSENT && round > 0
                         ? " (did not answer in the first round)"
                         : "");
      if (scannerOtaTransferImageToSlot(sd, slot, image_size, image_hash, serial)) {
        results[slot] = SlotOtaResult::UPDATED;
        serial.print("S");
        serial.print(slot);
        serial.println(" scanner OTA complete");
      } else {
        results[slot] = SlotOtaResult::FAILED;
        serial.print("S");
        serial.print(slot);
        serial.println(" scanner OTA failed");
      }
    }
  }

  uint8_t updated_slots = 0;
  uint8_t failed_slots = 0;
  for (uint8_t slot = 0; slot < SCANNER_SLOT_COUNT; slot++) {
    if (results[slot] == SlotOtaResult::UPDATED) {
      updated_slots++;
    } else if (results[slot] == SlotOtaResult::FAILED) {
      failed_slots++;
    }
  }

  if (updated_slots == 0 && failed_slots == 0) {
    serial.println("No SPI OTA-capable scanners found; leaving scanner.bin in place");
    return false;
  }

  if (failed_slots != 0) {
    serial.print("Scanner OTA failed on ");
    serial.print(failed_slots);
    serial.println(" scanner(s); leaving scanner.bin in place for retry");
    return updated_slots != 0;
  }

  char applied_path[56] = {};
  if (!buildScannerAppliedPath(sd, applied_path, sizeof(applied_path))) {
    serial.println("Scanner OTA applied, but no free applied filename was available");
    return true;
  }
  if (!sd.rename(SCANNER_SPI_UPDATE_PATH, applied_path)) {
    serial.print("Scanner OTA applied, but rename failed: ");
    serial.println(applied_path);
    return true;
  }

  serial.print("Scanner OTA applied successfully; renamed SD image to ");
  serial.println(applied_path);
  return true;
#endif
}

static bool ensureScannerSlotIdentity(uint8_t slot, ScannerSlotState& state) {
  // Slot identity is cached until transport failures force a re-probe. This
  // keeps empty slots from consuming controller bandwidth every pass.
  if (state.id_valid) {
    return true;
  }

  const uint32_t now = millis();
  if ((int32_t)(now - state.next_probe_ms) < 0) {
    return false;
  }

  scannerReleaseBulkWait();
  DeviceIdReply id = {};
  if (!scannerGetDeviceId(id)) {
    state.id_valid = false;
    state.next_service_ms = 0;
    state.dedupe_reset_done = false;
    state.next_dedupe_reset_ms = 0;
    state.dedupe_reset_failures = 0;
    state.runtime_dedupe_reset_pending = false;
    state.next_runtime_dedupe_reset_ms = 0;
    state.runtime_dedupe_reset_failures = 0;
    state.transport_timeouts = 0;
    state.next_probe_ms = now + SCANNER_SLOT_REPROBE_MS;
    if (!state.miss_logged) {
      scannerSerialPrintfTry("S%u no valid scanner ID; skipping slot\n", (unsigned)slot);
      state.miss_logged = true;
    }
    return false;
  }

  // Only accept slots that speak our protocol and report sane limits.
  state.id = id;
  state.miss_logged = false;

  // Scanners keep monitor mode across a controller reboot and refuse SCAN
  // while in it, so release any capture a previous session left running.
  if ((id.capabilities & CAP_MONITOR) != 0) {
    scannerOtaStopMonitor();
  }

  const bool proto_ok = (id.proto_version == PROTO_VERSION);
  const bool type_ok = (id.scanner_type == SCANNER_DEVICE_TYPE);
  const bool max_ok = (id.max_results > 0 && id.max_results <= PROTO_MAX_RESULTS);
  if (!(proto_ok && type_ok && max_ok)) {
    state.id_valid = false;
    state.next_service_ms = 0;
    state.dedupe_reset_done = false;
    state.next_dedupe_reset_ms = 0;
    state.dedupe_reset_failures = 0;
    state.runtime_dedupe_reset_pending = false;
    state.next_runtime_dedupe_reset_ms = 0;
    state.runtime_dedupe_reset_failures = 0;
    state.transport_timeouts = 0;
    state.next_probe_ms = now + SCANNER_SLOT_REPROBE_MS;
    if (!state.unsupported_logged) {
      scannerSerialPrintfTry("S%u unsupported SPI device: proto=%u type=%u caps=0x%02X max=%u; skipping slot\n",
                             (unsigned)slot,
                             id.proto_version, id.scanner_type, id.capabilities, id.max_results);
      state.unsupported_logged = true;
    }
    return false;
  }

  state.id_valid = true;
  state.transport_timeouts = 0;
  state.next_service_ms = 0;
  state.dedupe_reset_done = false;
  state.next_dedupe_reset_ms = 0;
  state.dedupe_reset_failures = 0;
  state.runtime_dedupe_reset_pending = false;
  state.next_runtime_dedupe_reset_ms = 0;
  state.runtime_dedupe_reset_failures = 0;
  state.next_probe_ms = 0;
  state.unsupported_logged = false;
  scannerSerialPrintfTry("S%u scanner ready: proto=%u type=%u caps=0x%02X max=%u\n",
                         (unsigned)slot,
                         id.proto_version, id.scanner_type, id.capabilities, id.max_results);
  scannerDrainStaleFrames(1);
  return true;
}

static int8_t scannerQueryStatusWithRetry(uint8_t cmd,
                                          uint8_t arg1,
                                          uint8_t arg2,
                                          uint8_t response_pulls,
                                          uint8_t retries,
                                          int8_t min_valid_status,
                                          int8_t max_valid_status) {
  // SCAN needs a longer first wait than lightweight status commands because
  // the scanner must hand work to the Wi-Fi driver before an ACK is ready.
  uint32_t first_pull_wait_us = SCANNER_STATUS_FIRST_PULL_US;
  if (cmd == CMD_SCAN) {
    first_pull_wait_us = SCANNER_SCAN_FIRST_PULL_US;
  }

  for (uint8_t attempt = 0; attempt < retries; attempt++) {
    uint8_t tx[SCANNER_FRAME_SIZE] = {0};
    uint8_t rx[SCANNER_FRAME_SIZE] = {0};
    tx[0] = cmd;
    tx[1] = arg1;
    tx[2] = arg2;
    if (scannerCommandWithResponse(tx, cmd, rx, response_pulls, first_pull_wait_us)) {
      const int8_t status = (int8_t)rx[0];
      if (status >= min_valid_status && status <= max_valid_status) {
        return status;
      }
      // Corrupt/unknown status payload; retry in-place before giving up.
      continue;
    }
    // Lost/misaligned response; retry in-place before giving up.
  }
  return SCANNER_STATUS_TRANSPORT_TIMEOUT;
}

static bool scannerGetResult(WiFiResultPacket& pkt) {
  // RESULT_GET is a two-phase exchange: request the next packet, then keep
  // pulling until the scanner returns a tagged result payload.
  uint8_t tx[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  tx[0] = CMD_RESULT_GET;
  nop[0] = CMD_NOP;

  for (int attempt = 0; attempt < SCANNER_RESULT_GET_CMD_RETRIES; attempt++) {
    scannerTransferFrame(tx, rx);
    if (SCANNER_STATUS_FIRST_PULL_US > 0) {
      delayMicroseconds(SCANNER_STATUS_FIRST_PULL_US);
    }

    for (int pull = 0; pull < SCANNER_RESULT_RESPONSE_PULLS; pull++) {
      scannerTransferFrame(nop, rx);
      if (rx[SCANNER_FRAME_TAG_INDEX] != CMD_RESULT_GET) {
        scannerInterframeWait();
        continue;
      }

      WiFiResultPacket candidate = {};
      memcpy(&candidate, rx, sizeof(candidate));
      if (candidate.result_type == RESULT_WIFI ||
          candidate.result_type == RESULT_BUSY ||
          candidate.result_type == RESULT_END) {
        pkt = candidate;
        return true;
      }

      // Valid tag but invalid payload; keep pulling to find a sane packet.
      scannerInterframeWait();
    }

    // Retry full RESULT_GET transaction before failing this record.
  }
  return false;
}

static void startScanForCurrentChannel(uint8_t slot,
                                       ChannelScheduleState& schedule_state,
                                       bool& sweep_timing_active,
                                       uint32_t& sweep_started_ms) {
  // Every slot draws work from the same shared dispatch plan. A slot that
  // successfully starts a scan advances the global plan for the next slot.
  const ChannelScheduleEntry scheduled = channelScheduleCurrent(schedule_state);
  const uint8_t band = scheduled.band;
  const uint8_t channel = scheduled.channel;
  sweepTimingNoteStart(sweep_timing_active, sweep_started_ms);

  const int8_t status = scannerQueryStatusWithRetry(CMD_SCAN,
                                                    band,
                                                    channel,
                                                    SCANNER_SCAN_RESPONSE_PULLS,
                                                    SCANNER_SCAN_CMD_RETRIES,
                                                    SCANNER_STATUS_START_FAILED,
                                                    SCANNER_STATUS_OK);
  if (status == SCANNER_STATUS_OK) {
#if SCAN_LOG_NORMAL_STARTS
    scannerSerialPrintfTry("S%u Scan started (band=%u ch=%u)\n",
                           (unsigned)slot, band, channel);
#endif
    advanceSweepChannelAndLog(schedule_state,
                              sweep_timing_active,
                              sweep_started_ms);
    return;
  }
  if (status == SCANNER_STATUS_BUSY) {
    // If RESULT_COUNT said idle but SCAN says busy, the prior start likely latched
    // and we missed/garbled the ACK. Advance to avoid re-requesting same channel.
    scannerSerialPrintfTry("S%u Scan already active (band=%u ch=%u); assuming prior start latched\n",
                           (unsigned)slot, band, channel);
    advanceSweepChannelAndLog(schedule_state,
                              sweep_timing_active,
                              sweep_started_ms);
    return;
  }
  if (status == SCANNER_STATUS_TRANSPORT_TIMEOUT) {
    // If SCAN ACK was lost but scanner is now busy, treat scan as started.
    for (uint8_t confirm = 0; confirm < SCANNER_SCAN_BUSY_CONFIRM_POLLS; confirm++) {
      const int8_t c = scannerQueryStatusWithRetry(CMD_RESULT_COUNT,
                                                   0,
                                                   0,
                                                   SCANNER_RESULT_COUNT_RESPONSE_PULLS,
                                                   SCANNER_RESULT_COUNT_CMD_RETRIES,
                                                   SCANNER_STATUS_BUSY,
                                                   static_cast<int8_t>(PROTO_MAX_RESULTS));
      if (c == SCANNER_STATUS_BUSY) {
        scannerSerialPrintfTry("S%u Scan started (band=%u ch=%u) via BUSY confirm\n",
                               (unsigned)slot, band, channel);
        advanceSweepChannelAndLog(schedule_state,
                                  sweep_timing_active,
                                  sweep_started_ms);
        return;
      }
      if (SCANNER_SCAN_BUSY_CONFIRM_DELAY_US > 0) {
        delayMicroseconds(SCANNER_SCAN_BUSY_CONFIRM_DELAY_US);
      }
    }
    // Timeout with no BUSY confirm: move on to avoid getting pinned on one channel.
    scannerSerialPrintfTry("S%u SCAN timeout (band=%u ch=%u); moving on\n",
                           (unsigned)slot, band, channel);
    advanceSweepChannelAndLog(schedule_state,
                              sweep_timing_active,
                              sweep_started_ms);
    return;
  }

  // Unexpected scan status: move on to the next channel.
  scannerSerialPrintfTry("S%u SCAN unexpected status=%d (band=%u ch=%u); moving on\n",
                         (unsigned)slot, status, band, channel);
  advanceSweepChannelAndLog(schedule_state,
                            sweep_timing_active,
                            sweep_started_ms);
}

// Per-slot scan pump with a shared channel scheduler:
// 1) Poll RESULT_COUNT.
// 2) If busy -> return.
// 3) If zero -> request scan on current channel.
// 4) If >0 -> drain that many packets and enqueue uniques.
static void processScannerSlot(uint8_t slot,
                               ScannerSlotState& slot_state,
                               ChannelScheduleState& schedule_state,
                               bool& sweep_timing_active,
                               uint32_t& sweep_started_ms) {
  // Each service pass handles one slot only. This keeps the round-robin fair
  // across populated scanners even when one slot has a large result backlog.
  const uint32_t now = millis();
  if ((int32_t)(now - slot_state.next_service_ms) < 0) {
    return;
  }

  scannerSetActiveSlot(slot);

  if (!slot_state.dedupe_reset_done) {
    // Reset scanner-local dedupe once per identification cycle so the slave
    // starts each attach with a clean buffer model.
    if ((int32_t)(now - slot_state.next_dedupe_reset_ms) < 0) {
      return;
    }

    const int8_t reset_status = scannerQueryStatusWithRetry(CMD_DEDUPE_RESET,
                                                            0,
                                                            0,
                                                            SCANNER_RESULT_COUNT_RESPONSE_PULLS,
                                                            SCANNER_QUERY_CMD_RETRIES,
                                                            SCANNER_STATUS_BUSY,
                                                            SCANNER_STATUS_OK);
    if (reset_status == SCANNER_STATUS_OK) {
      slot_state.dedupe_reset_done = true;
      slot_state.next_dedupe_reset_ms = 0;
      slot_state.dedupe_reset_failures = 0;
      slot_state.transport_timeouts = 0;
      slot_state.next_service_ms = 0;
      scannerSerialPrintfTry("S%u scanner dedupe reset\n", (unsigned)slot);
      scannerDrainStaleFrames(1);
    } else {
      if (slot_state.dedupe_reset_failures < 0xFF) {
        slot_state.dedupe_reset_failures++;
      }
      slot_state.next_dedupe_reset_ms = now + SCANNER_DEDUPE_RESET_RETRY_MS;
      if (reset_status == SCANNER_STATUS_TRANSPORT_TIMEOUT &&
          slot_state.transport_timeouts < 0x0F) {
        slot_state.transport_timeouts++;
      }
      if (slot_state.dedupe_reset_failures >= SCANNER_DEDUPE_RESET_MAX_ATTEMPTS) {
        slot_state.dedupe_reset_done = true;
        slot_state.next_dedupe_reset_ms = 0;
        slot_state.dedupe_reset_failures = 0;
        scannerSerialPrintfTry("S%u scanner dedupe reset failed status=%d; continuing without reset\n",
                               (unsigned)slot,
                               reset_status);
      }
      return;
    }
  }

  const int8_t count = scannerQueryStatusWithRetry(CMD_RESULT_COUNT,
                                                   0,
                                                   0,
                                                   SCANNER_RESULT_COUNT_RESPONSE_PULLS,
                                                   SCANNER_RESULT_COUNT_CMD_RETRIES,
                                                   SCANNER_STATUS_BUSY,
                                                   static_cast<int8_t>(PROTO_MAX_RESULTS));

  // Active scan on slave; poll this slot again next round.
  if (count == SCANNER_STATUS_BUSY) {
    slot_state.transport_timeouts = 0;
    slot_state.next_service_ms = 0;
    return;
  }

  if (count == SCANNER_STATUS_TRANSPORT_TIMEOUT) {
    if (slot_state.transport_timeouts < 0x0F) {
      slot_state.transport_timeouts++;
    }
    const uint8_t streak = slot_state.transport_timeouts;
    const uint8_t shift = (streak <= 1) ? 0 : ((streak >= 5) ? 4 : (uint8_t)(streak - 1));
    uint32_t backoff_ms = (uint32_t)SCANNER_SLOT_TIMEOUT_BACKOFF_BASE_MS << shift;
    if (backoff_ms > SCANNER_SLOT_TIMEOUT_BACKOFF_MAX_MS) {
      backoff_ms = SCANNER_SLOT_TIMEOUT_BACKOFF_MAX_MS;
    }
    slot_state.next_service_ms = now + backoff_ms;

    if ((streak == SCANNER_SLOT_TIMEOUT_REPROBE_THRESHOLD) ||
        ((streak % SCANNER_SLOT_TIMEOUT_REPROBE_THRESHOLD) == 0)) {
      scannerSerialPrintfTry("S%u RESULT_COUNT timeout x%u; backing off %lums\n",
                             (unsigned)slot,
                             (unsigned)streak,
                             (unsigned long)backoff_ms);
    }
    return;
  }
  slot_state.transport_timeouts = 0;
  slot_state.next_service_ms = 0;

  if (count < 0) {
    // Unexpected status: advance global scheduler so one bad slot does not stall coverage.
    scannerSerialPrintfTry("S%u RESULT_COUNT unexpected=%d; moving on\n",
                           (unsigned)slot, count);
    advanceSweepChannelAndLog(schedule_state,
                              sweep_timing_active,
                              sweep_started_ms);
    return;
  }

  // No results pending: request next scan from the global scheduler.
  if (count == 0) {
    if (slot_state.runtime_dedupe_reset_pending) {
      if ((int32_t)(now - slot_state.next_runtime_dedupe_reset_ms) < 0) {
        return;
      }

      const int8_t reset_status = scannerQueryStatusWithRetry(CMD_DEDUPE_RESET,
                                                              0,
                                                              0,
                                                              SCANNER_RESULT_COUNT_RESPONSE_PULLS,
                                                              SCANNER_QUERY_CMD_RETRIES,
                                                              SCANNER_STATUS_BUSY,
                                                              SCANNER_STATUS_OK);
      if (reset_status == SCANNER_STATUS_OK) {
        slot_state.runtime_dedupe_reset_pending = false;
        slot_state.next_runtime_dedupe_reset_ms = 0;
        slot_state.runtime_dedupe_reset_failures = 0;
        slot_state.transport_timeouts = 0;
        slot_state.next_service_ms = 0;
        scannerSerialPrintfTry("S%u runtime dedupe reset\n", (unsigned)slot);
        scannerDrainStaleFrames(1);
      } else {
        if (slot_state.runtime_dedupe_reset_failures < 0xFF) {
          slot_state.runtime_dedupe_reset_failures++;
        }
        slot_state.next_runtime_dedupe_reset_ms = now + SCANNER_DEDUPE_RESET_RETRY_MS;
        if (reset_status == SCANNER_STATUS_TRANSPORT_TIMEOUT &&
            slot_state.transport_timeouts < 0x0F) {
          slot_state.transport_timeouts++;
        }
        if (slot_state.runtime_dedupe_reset_failures >= SCANNER_DEDUPE_RESET_MAX_ATTEMPTS) {
          slot_state.runtime_dedupe_reset_pending = false;
          slot_state.next_runtime_dedupe_reset_ms = 0;
          slot_state.runtime_dedupe_reset_failures = 0;
          scannerSerialPrintfTry("S%u runtime dedupe reset failed status=%d; continuing without reset\n",
                                 (unsigned)slot,
                                 reset_status);
        }
      }
      return;
    }

    startScanForCurrentChannel(slot,
                               schedule_state,
                               sweep_timing_active,
                               sweep_started_ms);
    return;
  }

  // Results pending: drain up to reported count for this slot.
  bool scanner_busy_during_drain = false;
  const int drain_budget = (count > (int)SCANNER_RESULT_DRAIN_BUDGET)
    ? (int)SCANNER_RESULT_DRAIN_BUDGET
    : count;
  uint16_t batch_dedupe_hits = 0;
  uint16_t batch_dedupe_logs = 0;
  for (int i = 0; i < drain_budget; i++) {
    WiFiResultPacket pkt = {};
    if (!scannerGetResult(pkt)) {
      if (slot_state.transport_timeouts < 0x0F) {
        slot_state.transport_timeouts++;
      }
      slot_state.next_service_ms = now + SCANNER_SLOT_TIMEOUT_BACKOFF_BASE_MS;
      break;
    }

    if (pkt.result_type == RESULT_WIFI) {
      if (!wifiResultIsValidForDedupe(pkt.result)) {
        const bool zero_bssid = wifiBssidIsZero(pkt.result.bssid);
        const bool ff_bssid = wifiBssidIsBroadcast(pkt.result.bssid);
        scannerSerialPrintfTry("S%u Dropped invalid WIFI result: ch=%u band=%u rssi=%d ssid0=0x%02X bssid_zero=%u bssid_ff=%u\n",
                               (unsigned)slot,
                               pkt.result.channel,
                               pkt.result.band,
                               pkt.result.rssi,
                               (uint8_t)pkt.result.ssid[0],
                               zero_bssid ? 1 : 0,
                               ff_bssid ? 1 : 0);
        continue;
      }

      WiFiDedupeHash hash = {};
      wifiDedupeHashFromResult(pkt.result, hash);
      // Controller-side dedupe suppresses duplicates across all scanners, not
      // just within the reporting slot.
      if (!wifiDedupeTableRemember(scanner_runtime_context.master_dedupe_table, &hash)) {
        (*scanner_runtime_context.dedupe_drops)++;
        batch_dedupe_hits++;
#if LOG_DEDUPE_HITS
#if CONTROLLER_LOG_INDIVIDUAL_DEDUPE_HITS
        if (batch_dedupe_logs < LOG_DEDUPE_HITS_MAX_PER_SCAN) {
          char mac[18] = {};
          pico_logging::formatBssid(pkt.result.bssid, mac, sizeof(mac));
          scannerSerialPrintfTry("[MASTER S%u] dedupe hit ssid='%s' bssid=%s ch=%u band=%u rssi=%d\n",
                                 (unsigned)slot,
                                 pkt.result.ssid,
                                 mac,
                                 pkt.result.channel,
                                 pkt.result.band,
                                 pkt.result.rssi);
          batch_dedupe_logs++;
        }
#endif
#endif
        continue;
      }

      QueuedScanResult q = {};
      strncpy(q.ssid, pkt.result.ssid, sizeof(q.ssid) - 1);
      memcpy(q.bssid, pkt.result.bssid, sizeof(q.bssid));
      q.rssi = pkt.result.rssi;
      q.channel = pkt.result.channel;
      q.band = pkt.result.band;
      q.capabilities = pkt.result.capabilities;
      if (!queue_try_add(scanner_runtime_context.scan_result_queue, &q)) {
        (*scanner_runtime_context.scan_queue_drops)++;
      }
      continue;
    }

    if (pkt.result_type == RESULT_END) {
      break;
    }
    if (pkt.result_type == RESULT_BUSY) {
      scanner_busy_during_drain = true;
      break;
    }

    scannerSerialPrintfTry("S%u Unknown result type: 0x%02X; draining stale frames\n",
                           (unsigned)slot, pkt.result_type);
    scannerDrainStaleFrames(2);
    break;
  }

#if LOG_DEDUPE_HITS
  if (batch_dedupe_hits > batch_dedupe_logs) {
    scannerSerialPrintfTry("[MASTER S%u] dedupe hits suppressed: %u (total=%u)\n",
                           (unsigned)slot,
                           (unsigned)(batch_dedupe_hits - batch_dedupe_logs),
                           (unsigned)batch_dedupe_hits);
  }
#endif

  if (scanner_busy_during_drain) {
    return;
  }
}

// ---------- Monitor mode packet engine ----------
// Shared by the USB capture hand-off and the standalone monitor probe build:
// per-slot capture control plus overlapped, need-based CMD_PKT_READ polling.
static constexpr uint8_t MONITOR_PROBE_CHANNELS[] = {1, 6, 11, 40, 48, 161, 36};
static constexpr uint8_t MONITOR_PROBE_CHANNEL_COUNT = sizeof(MONITOR_PROBE_CHANNELS);
static constexpr uint32_t MONITOR_PROBE_RESPONSE_TIMEOUT_MS = 300;
// A command can land while the scanner is still re-arming after the previous
// transaction and be lost; resend it rather than pulling for the full timeout.
static constexpr uint32_t MONITOR_PROBE_RESEND_MS = 30;
// Quiet time on a slot after any transfer before sending it a new command.
static constexpr uint16_t SLOT_COMMAND_GAP_US = 300;
static constexpr uint32_t MONITOR_PROBE_REPROBE_MS = 5000;

// Batch polling: a slot with nothing queued is re-polled at this interval to
// learn about new data; otherwise the slot with the most queued bytes (plus
// an age bonus so no slot starves) is read next.
static constexpr uint32_t PKT_IDLE_POLL_MS = 10;
static constexpr uint32_t PKT_AGE_WEIGHT_BYTES_PER_MS = 256;
// The scanner builds the batch (copy + CRC) before it re-arms, so the reply
// takes up to about a millisecond; other slots are serviced in the meantime.
static constexpr uint16_t PKT_REPLY_FIRST_WAIT_US = 300;
// Minimum spacing between frames to the same slot while waiting for a reply.
// A scanner re-arms its slave transaction from its main loop; frames that
// arrive before it has re-armed are lost, and back-to-back pulls only add to
// that, so pull each slot at a bounded rate instead of as fast as the bus goes.
static constexpr uint16_t PKT_REPLY_PULL_GAP_US = 250;
static constexpr uint32_t PKT_REPLY_TIMEOUT_US = 15000;
// Delay before each bulk read attempt: the first covers the scanner re-arming
// its transaction after the header pull, the rest back off after a miss.
static constexpr uint16_t PKT_BULK_WAIT_US[] = {30, 200, 1000, 3000};

struct MonitorSlotState {
  bool present;
  // Scanner confirmed in capture mode; only these slots are polled for batches.
  bool monitoring;
  uint8_t band;
  uint8_t channel;
  uint32_t next_probe_ms;
  uint32_t last_ms;
  MonitorStatsReply last;
  // Totals for the current dwell, printed when the slot retunes.
  uint32_t dwell_packets;
  uint32_t dwell_air_bytes;
  uint32_t dwell_drops;
  uint32_t dwell_ms;
  // Batch transport state.
  bool request_outstanding;
  uint32_t request_sent_us;
  uint32_t last_pull_us;
  uint32_t pending_bytes;
  uint32_t last_poll_ms;
  bool ack_pending;
  uint16_t ack_seq;
  bool delivered_any;
  uint16_t last_delivered_seq;
  // Batch transport counters since the last report.
  uint32_t spi_record_bytes;
  uint32_t spi_records;
  uint32_t spi_batches;
  uint32_t spi_polls;
  uint32_t spi_crc_errors;
  uint32_t spi_bulk_retries;
  uint32_t spi_duplicates;
  uint32_t spi_aborts;
  uint32_t spi_reply_timeouts;
  uint32_t spi_busy_replies;
  uint32_t spi_busy_us;
  // Synthetic mode: next expected record sequence and missing records seen.
  uint32_t synth_next_seq;
  uint32_t spi_synth_gaps;
  // Reply timeouts, abandoned batches and CRC failures since monitor start.
  uint32_t transport_errors_total;
  // Capture settings the scanner is running with (monitor-link sessions), so
  // an unchanged slot is not restarted when another slot is retuned.
  ControllerMonitorSlotConfig applied_cfg;
  uint32_t scanner_ring_drops;
};

static MonitorSlotState monitor_slots[SCANNER_SLOT_COUNT] = {};

// core1 only: a USB capture session owns the scanners and wants their frames.
static bool monitor_link_streaming = false;

static inline void probeHeartbeat() {
  if (scanner_runtime_context.wd_core1_last_ms != nullptr) {
    *scanner_runtime_context.wd_core1_last_ms = to_ms_since_boot(get_absolute_time());
  }
}

static inline void slotNoteTransfer() {
  scanner_slot_last_xfer_us[scanner_active_slot] = time_us_32();
}

static inline bool slotReadyForCommand() {
  return time_us_32() - scanner_slot_last_xfer_us[scanner_active_slot] >= SLOT_COMMAND_GAP_US;
}

static inline void slotWaitForCommandGap() {
  while (!slotReadyForCommand()) {
  }
}
// Bulk reads clock a little past wire_len so the scanner's transaction always
// ends on its own length rather than on chip-select.
static constexpr uint16_t PKT_BULK_OVERCLOCK_BYTES = 4;
static uint8_t pkt_bulk_rx[PKT_BATCH_MAX_BYTES + PKT_BULK_OVERCLOCK_BYTES] __attribute__((aligned(4)));

// Monitor commands touch the WiFi driver before the scanner re-arms its SPI
// transaction (a first start or a cross-band retune takes tens of ms), so
// poll on a wall-clock deadline rather than a fixed pull count.
static bool monitorSlotCommand(const uint8_t* tx, uint8_t* rx) {
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  uint8_t discard[SCANNER_FRAME_SIZE] = {0};
  slotWaitForCommandGap();
  scannerTransferFrame(tx, discard);
  uint32_t sent_ms = millis();
  const uint32_t deadline = sent_ms + MONITOR_PROBE_RESPONSE_TIMEOUT_MS;
  while ((int32_t)(millis() - deadline) < 0) {
    delay(1);
    probeHeartbeat();
    scannerTransferFrame(nop, rx);
    if (rx[SCANNER_FRAME_TAG_INDEX] == tx[0]) {
      slotNoteTransfer();
      return true;
    }
    if (millis() - sent_ms >= MONITOR_PROBE_RESEND_MS) {
      // Every probe control command is safe to repeat.
      scannerTransferFrame(tx, discard);
      sent_ms = millis();
    }
  }
  slotNoteTransfer();
  return false;
}

// Synthetic ramp: per-slot generation rate for the current dwell, KiB/s.
static uint32_t monitor_probe_synthetic_kibps = 0;

// Send CMD_MONITOR_START to the active slot. START is idempotent, so a lost
// command or reply is simply retried.
static int8_t monitorSlotStart(MonitorStartCommand start) {
  uint8_t tx[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  start.cmd = CMD_MONITOR_START;
  memcpy(tx, &start, sizeof(start));
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    if (monitorSlotCommand(tx, rx)) {
      return static_cast<int8_t>(rx[0]);
    }
  }
  return SCANNER_STATUS_TRANSPORT_TIMEOUT;
}

static bool monitorSlotStop() {
  uint8_t tx[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  tx[0] = CMD_MONITOR_STOP;
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    if (monitorSlotCommand(tx, rx)) {
      return true;
    }
  }
  return false;
}

#if SCANNER_MONITOR_PROBE
static int8_t monitorProbeStart(uint8_t channel) {
  MonitorStartCommand start = {};
  start.cmd = CMD_MONITOR_START;
  start.band = wifiBandFromChannel(channel);
  start.channel = channel;
  start.type_mask = SCANNER_MONITOR_PROBE_TYPE_MASK;
  start.snaplen_data = SCANNER_MONITOR_PROBE_SNAPLEN_DATA;
  start.drain_kib_per_s = SCANNER_MONITOR_PROBE_DRAIN_KIBPS;
  start.flags = SCANNER_MONITOR_PROBE_SPI_DRAIN ? 0 : MONITOR_START_FLAG_SIM_DRAIN;
  if (SCANNER_MONITOR_PROBE_SYNTHETIC_KIBPS != 0) {
    start.flags |= MONITOR_START_FLAG_SYNTHETIC;
    start.drain_kib_per_s = static_cast<uint16_t>(monitor_probe_synthetic_kibps);
  }
  return monitorSlotStart(start);
}

static bool monitorProbeReadStats(MonitorStatsReply& out) {
  uint8_t tx[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  tx[0] = CMD_MONITOR_STATS;
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    if (monitorSlotCommand(tx, rx)) {
      memcpy(&out, rx, sizeof(out));
      return out.status == SCANNER_STATUS_OK;
    }
  }
  return false;
}

static void monitorProbeTune(uint8_t slot, MonitorSlotState& state, uint8_t channel, uint32_t now) {
  if (state.dwell_ms >= 1000) {
    const uint32_t secs = state.dwell_ms / 1000;
    scannerSerialPrintfTry("MON s%u == ch %3u summary: %lus avg %lu pkt/s %lu KiB/s drops %lu\n",
                           slot, state.channel, (unsigned long)secs,
                           (unsigned long)(state.dwell_packets / secs),
                           (unsigned long)(state.dwell_air_bytes / secs / 1024),
                           (unsigned long)state.dwell_drops);
  }
  const int8_t status = monitorProbeStart(channel);
  if (status != SCANNER_STATUS_OK) {
    MonitorStatsReply stats = {};
    monitorProbeReadStats(stats);
    scannerSerialPrintfTry("MON s%u start ch %u failed: %d (stage %u err 0x%lx)\n", slot, channel,
                           status, stats.error_stage, (unsigned long)stats.error_code);
    state.present = false;
    state.next_probe_ms = now + MONITOR_PROBE_REPROBE_MS;
    return;
  }
  state.channel = channel;
  state.band = wifiBandFromChannel(channel);
  state.monitoring = true;
  state.last = {};
  state.last_ms = now;
  // START discards the scanner's ring, including any unacknowledged batch.
  state.synth_next_seq = 0;
  state.request_outstanding = false;
  state.ack_pending = false;
  state.pending_bytes = 0;
  state.dwell_packets = 0;
  state.dwell_air_bytes = 0;
  state.dwell_drops = 0;
  state.dwell_ms = 0;
}

static void monitorProbeReport(uint8_t slot, MonitorSlotState& state, uint32_t now) {
  MonitorStatsReply stats = {};
  if (!monitorProbeReadStats(stats)) {
    scannerSerialPrintfTry("MON s%u stats read failed\n", slot);
    return;
  }
  if (!stats.active) {
    scannerSerialPrintfTry("MON s%u scanner reports monitor inactive; restarting\n", slot);
    state.present = false;
    return;
  }
  const MonitorStatsReply& last = state.last;
  const uint32_t dt_ms = now - state.last_ms;
  const uint32_t dm = stats.packets_mgmt - last.packets_mgmt;
  const uint32_t dc = stats.packets_ctrl - last.packets_ctrl;
  const uint32_t dd = stats.packets_data - last.packets_data;
  const uint32_t dair = stats.air_bytes - last.air_bytes;
  const uint32_t dcopied = stats.copied_bytes - last.copied_bytes;
  const uint32_t ddrops = stats.ring_drops - last.ring_drops;
  const uint32_t dcycles = stats.cb_cycles - last.cb_cycles;
  // Integer math only: float printf on core1 is not safe alongside core0.
  if (dt_ms == 0) {
    return;
  }
  auto perSecond = [dt_ms](uint32_t value) {
    return static_cast<unsigned>(static_cast<uint64_t>(value) * 1000u / dt_ms);
  };
  // Callback CPU share in tenths of a percent.
  const uint32_t cb_permille =
      stats.cpu_mhz > 0
          ? static_cast<uint32_t>(static_cast<uint64_t>(dcycles) / (stats.cpu_mhz * dt_ms))
          : 0;
  const uint32_t ring_bytes = static_cast<uint32_t>(stats.ring_kib) * 1024u;
  scannerSerialPrintfTry(
      "MON s%u ch %3u/%3u/%3u %5u pkt/s (m%u c%u d%u) air %4u KiB/s copied %4u KiB/s drops %lu "
      "cb %lu.%lu%% max %uus ring hw %lu%% heap %lu\n",
      slot, stats.channel, stats.radio_channel, stats.last_frame_channel, perSecond(dm + dc + dd), perSecond(dm), perSecond(dc), perSecond(dd),
      perSecond(dair) / 1024, perSecond(dcopied) / 1024, (unsigned long)ddrops,
      (unsigned long)(cb_permille / 10), (unsigned long)(cb_permille % 10), stats.cb_max_us,
      (unsigned long)(ring_bytes ? 100u * stats.ring_high_water / ring_bytes : 0),
      (unsigned long)stats.free_heap);
  state.dwell_packets += dm + dc + dd;
  state.dwell_air_bytes += dair;
  state.dwell_drops += ddrops;
  state.dwell_ms += dt_ms;
  state.last = stats;
  state.last_ms = now;
}

#endif  // SCANNER_MONITOR_PROBE

static bool pktBatchMatches(const PktReadReply& reply) {
  PktBatchHeader header;
  memcpy(&header, pkt_bulk_rx, sizeof(header));
  if (header.magic != PKT_BATCH_MAGIC || header.seq != reply.seq ||
      header.record_bytes != reply.record_bytes || header.records != reply.records ||
      header.crc32 != reply.crc32 ||
      sizeof(PktBatchHeader) + header.record_bytes > reply.wire_len) {
    return false;
  }
  const uint8_t* records = pkt_bulk_rx + sizeof(PktBatchHeader);
  return pktCrc32(records, header.record_bytes) == header.crc32 &&
         pktBatchRecordsValid(records, header.record_bytes, header.records);
}

// Step 1 of an exchange: send CMD_PKT_READ (acknowledging the previous batch)
// and return without waiting; the scanner builds the batch meanwhile.
static void pktSendRequest(MonitorSlotState& state, uint32_t now_ms) {
  uint8_t tx[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  PktReadCommand command = {};
  command.cmd = CMD_PKT_READ;
  command.flags = state.ack_pending ? PKT_READ_FLAG_ACK : 0;
  command.ack_seq = state.ack_seq;
  memcpy(tx, &command, sizeof(command));
  scannerTransferFrame(tx, rx);
  slotNoteTransfer();
  state.request_outstanding = true;
  state.request_sent_us = time_us_32();
  state.last_pull_us = state.request_sent_us;
  state.last_poll_ms = now_ms;
  state.spi_polls++;
}

// Synthetic records carry a running sequence number; any jump is loss
// somewhere between the scanner's generator and here.
static void pktCheckSyntheticSequence(MonitorSlotState& state, uint16_t record_bytes) {
  const uint8_t* records = pkt_bulk_rx + sizeof(PktBatchHeader);
  uint32_t offset = 0;
  while (offset + sizeof(MonitorPacketRecord) <= record_bytes) {
    MonitorPacketRecord rec;
    memcpy(&rec, records + offset, sizeof(rec));
    if ((rec.flags & MONITOR_PKT_FLAG_SYNTHETIC) &&
        rec.record_len >= sizeof(rec) + sizeof(uint32_t)) {
      uint32_t seq = 0;
      memcpy(&seq, records + offset + sizeof(rec), sizeof(seq));
      if (seq != state.synth_next_seq) {
        state.spi_synth_gaps += seq > state.synth_next_seq ? seq - state.synth_next_seq : 1;
      }
      state.synth_next_seq = seq + 1;
    }
    offset += rec.record_len;
  }
}

// Hand every record of a validated batch to the USB capture ring.
static void pktDeliverToUsb(const MonitorSlotState& state, uint16_t record_bytes) {
  const uint8_t slot = static_cast<uint8_t>(&state - monitor_slots);
  const uint8_t* records = pkt_bulk_rx + sizeof(PktBatchHeader);
  uint32_t offset = 0;
  while (offset + sizeof(MonitorPacketRecord) <= record_bytes) {
    MonitorPacketRecord rec;
    memcpy(&rec, records + offset, sizeof(rec));
    UsbCapturePacketHeader hdr = {};
    hdr.slot = slot;
    hdr.band = rec.band;
    hdr.channel = rec.channel;
    hdr.rssi = rec.rssi;
    hdr.noise = rec.noise;
    hdr.rate = rec.rate;
    hdr.sig_mode = rec.sig_mode;
    if (rec.flags & MONITOR_PKT_FLAG_FCS_INCLUDED) hdr.flags |= USB_CAPTURE_PKT_FCS_INCLUDED;
    if (rec.flags & MONITOR_PKT_FLAG_TRUNCATED) hdr.flags |= USB_CAPTURE_PKT_TRUNCATED;
    if (rec.flags & MONITOR_PKT_FLAG_SYNTHETIC) hdr.flags |= USB_CAPTURE_PKT_SYNTHETIC;
    hdr.radio_timestamp_us = rec.timestamp_us;
    hdr.orig_len = rec.orig_len;
    hdr.cap_len = static_cast<uint16_t>(rec.record_len - sizeof(rec));
    // Ring overflow is counted by the capture layer; never block here.
    controllerUsbCaptureQueuePacket(hdr, records + offset + sizeof(rec));
    offset += rec.record_len;
  }
}

// Clock the bulk read for a reply already in hand and account for it.
static void pktReadBulk(MonitorSlotState& state, const PktReadReply& reply) {
  bool ok = false;
  for (uint8_t attempt = 0; attempt < sizeof(PKT_BULK_WAIT_US) / sizeof(PKT_BULK_WAIT_US[0]);
       attempt++) {
    delayMicroseconds(PKT_BULK_WAIT_US[attempt]);
    scannerTransferBytes(nullptr, pkt_bulk_rx, reply.wire_len + PKT_BULK_OVERCLOCK_BYTES,
                         spi_clock_bulk);
    if (pktBatchMatches(reply)) {
      ok = true;
      break;
    }
    PktBatchHeader header;
    memcpy(&header, pkt_bulk_rx, sizeof(header));
    if (header.magic == PKT_BATCH_MAGIC && header.seq == reply.seq) {
      state.spi_crc_errors++;
      state.transport_errors_total++;
    }
    state.spi_bulk_retries++;
  }
  if (!ok) {
    // One plain frame takes the scanner out of its bulk wait; the batch stays
    // unacknowledged and is offered again on the next read.
    state.spi_aborts++;
    state.transport_errors_total++;
    scannerDrainStaleFrames(1);
    return;
  }

  if (state.delivered_any && reply.seq == state.last_delivered_seq) {
    // Resent after our ack was lost; already counted.
    state.spi_duplicates++;
  } else {
    state.spi_batches++;
    state.spi_records += reply.records;
    state.spi_record_bytes += reply.record_bytes;
    pktCheckSyntheticSequence(state, reply.record_bytes);
    if (monitor_link_streaming) {
      pktDeliverToUsb(state, reply.record_bytes);
    }
  }
  state.delivered_any = true;
  state.last_delivered_seq = reply.seq;
  state.ack_pending = true;
  state.ack_seq = reply.seq;
}

// Step 2: pull once for the reply. On success clock the batch; give up after
// PKT_REPLY_TIMEOUT_US. Returns true once the exchange is finished.
static bool pktTryComplete(MonitorSlotState& state) {
  const uint32_t now_us = time_us_32();
  const uint32_t elapsed = now_us - state.request_sent_us;
  if (elapsed < PKT_REPLY_FIRST_WAIT_US || now_us - state.last_pull_us < PKT_REPLY_PULL_GAP_US) {
    return false;
  }
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  state.last_pull_us = now_us;
  scannerTransferFrame(nop, rx);
  if (rx[SCANNER_FRAME_TAG_INDEX] != CMD_PKT_READ) {
    if (elapsed >= PKT_REPLY_TIMEOUT_US) {
      // The ack (if any) may not have landed; it is resent next time and the
      // scanner ignores a stale one.
      state.spi_reply_timeouts++;
      state.transport_errors_total++;
      state.request_outstanding = false;
      return true;
    }
    return false;
  }

  state.request_outstanding = false;
  PktReadReply reply = {};
  memcpy(&reply, rx, sizeof(reply));
  if (reply.status != SCANNER_STATUS_OK) {
    // BUSY: a bulk read from an abandoned exchange is still draining.
    state.spi_busy_replies++;
    return true;
  }
  // The scanner has processed the ack by the time it replies.
  state.ack_pending = false;
  state.scanner_ring_drops = reply.ring_drops;
  state.pending_bytes = reply.pending_bytes;
  if (reply.channel != 0) {
    // Follows scanner-side hopping.
    state.channel = reply.channel;
    state.band = reply.channel <= 14 ? WIFI_BAND_24_GHZ : WIFI_BAND_5_GHZ;
  }
  if (reply.wire_len == 0) {
    return true;
  }
  if (reply.wire_len > PKT_BATCH_MAX_BYTES || reply.wire_len < PKT_BATCH_MIN_WIRE_BYTES) {
    state.spi_aborts++;
    state.transport_errors_total++;
    scannerDrainStaleFrames(1);
    return true;
  }
  pktReadBulk(state, reply);
  return true;
}

// Finish an outstanding exchange on the active slot before control traffic.
static void pktFinishSlot(MonitorSlotState& state) {
  const uint32_t started = time_us_32();
  while (state.request_outstanding && !pktTryComplete(state)) {
    delayMicroseconds(PKT_REPLY_PULL_GAP_US);
    probeHeartbeat();
  }
  slotNoteTransfer();
  state.spi_busy_us += time_us_32() - started;
}

static uint32_t pktSlotScore(const MonitorSlotState& state, uint32_t now_ms) {
  return state.pending_bytes + (now_ms - state.last_poll_ms) * PKT_AGE_WEIGHT_BYTES_PER_MS;
}

// One scheduling pass over all slots, neediest first: advance every
// outstanding exchange by one pull and start a request on each slot that is
// due. Returns true if any slot was touched.
static bool pktServicePass(uint32_t now_ms) {
  uint8_t order[SCANNER_SLOT_COUNT];
  uint8_t count = 0;
  for (uint8_t slot = 0; slot < SCANNER_SLOT_COUNT; slot++) {
    if (!monitor_slots[slot].present || !monitor_slots[slot].monitoring) {
      continue;
    }
    // Insertion sort by descending score; at most a handful of slots.
    const uint32_t score = pktSlotScore(monitor_slots[slot], now_ms);
    uint8_t pos = count++;
    while (pos > 0 && pktSlotScore(monitor_slots[order[pos - 1]], now_ms) < score) {
      order[pos] = order[pos - 1];
      pos--;
    }
    order[pos] = slot;
  }

  bool worked = false;
  for (uint8_t i = 0; i < count; i++) {
    const uint8_t slot = order[i];
    MonitorSlotState& state = monitor_slots[slot];
    const uint32_t started = time_us_32();
    if (state.request_outstanding) {
      const uint32_t now_us = time_us_32();
      if (now_us - state.request_sent_us < PKT_REPLY_FIRST_WAIT_US ||
          now_us - state.last_pull_us < PKT_REPLY_PULL_GAP_US) {
        continue;
      }
      scannerSetActiveSlot(slot);
      pktTryComplete(state);
      slotNoteTransfer();
    } else {
      const uint32_t age = now_ms - state.last_poll_ms;
      if (state.pending_bytes == 0 && !state.ack_pending && age < PKT_IDLE_POLL_MS) {
        continue;
      }
      scannerSetActiveSlot(slot);
      if (!slotReadyForCommand()) {
        continue;
      }
      pktSendRequest(state, now_ms);
    }
    state.spi_busy_us += time_us_32() - started;
    worked = true;
  }
  return worked;
}

#if SCANNER_MONITOR_PROBE
static void pktReportDiag(uint8_t slot) {
  uint8_t tx[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  tx[0] = CMD_PKT_DIAG;
  if (!monitorSlotCommand(tx, rx)) {
    scannerSerialPrintfTry("DIAG s%u no reply\n", slot);
    return;
  }
  PktDiagReply diag = {};
  memcpy(&diag, rx, sizeof(diag));
  scannerSerialPrintfTry(
      "DIAG s%u state %u wire %u built %lu full %lu (bits %lu) cmd %lu (bits %lu) partial %lu "
      "(bits %lu) hdr_abandon %lu\n",
      slot, diag.bulk_state, diag.wire_len, (unsigned long)diag.batches_built,
      (unsigned long)diag.bulk_full, (unsigned long)diag.last_full_bits,
      (unsigned long)diag.bulk_command, (unsigned long)diag.last_command_bits,
      (unsigned long)diag.bulk_partial, (unsigned long)diag.last_partial_bits,
      (unsigned long)diag.header_abandoned);
}

static void pktReportSlot(uint8_t slot, MonitorSlotState& state, uint32_t dt_ms) {
  if (dt_ms == 0) {
    return;
  }
  auto perSecond = [dt_ms](uint32_t value) {
    return static_cast<unsigned>(static_cast<uint64_t>(value) * 1000u / dt_ms);
  };
  scannerSerialPrintfTry(
      "SPI s%u %4u KiB/s %4u rec/s %3u bat/s polls %4u busy %3u.%u%% pend %5lu crc %lu rty %lu dup %lu "
      "abort %lu tmo %lu busy_rsp %lu sdrops %lu gap %lu\n",
      slot, perSecond(state.spi_record_bytes) / 1024, perSecond(state.spi_records),
      perSecond(state.spi_batches), perSecond(state.spi_polls),
      (unsigned)(state.spi_busy_us / (dt_ms * 10)), (unsigned)(state.spi_busy_us / dt_ms % 10),
      (unsigned long)state.pending_bytes, (unsigned long)state.spi_crc_errors,
      (unsigned long)state.spi_bulk_retries, (unsigned long)state.spi_duplicates,
      (unsigned long)state.spi_aborts, (unsigned long)state.spi_reply_timeouts,
      (unsigned long)state.spi_busy_replies, (unsigned long)state.scanner_ring_drops,
      (unsigned long)state.spi_synth_gaps);
}

#if SCANNER_USE_SHIFTREG_CS
// One-shot timing of the transport's pieces, to see where per-transaction
// time goes. Talks only to slot 0 with NOP frames.
static void pktBenchTransport() {
  constexpr uint16_t N = 200;
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  scannerSetActiveSlot(0);

  uint32_t t0 = time_us_32();
  for (uint16_t i = 0; i < N; i++) {
    scannerTransferFrame(nop, rx);
  }
  const uint32_t frame_us = time_us_32() - t0;

  t0 = time_us_32();
  for (uint16_t i = 0; i < N; i++) {
    scannerShiftRegWriteCsState(0xFEu, true);
    scannerShiftRegWriteCsState(0xFFu, true);
  }
  const uint32_t cs_pair_us = time_us_32() - t0;

  t0 = time_us_32();
  for (uint16_t i = 0; i < N; i++) {
    scannerSpiSelectClock(spi_clock_shiftreg);
    scannerSpiSelectClock(spi_clock_scanner);
  }
  const uint32_t clock_switch_us = time_us_32() - t0;

  // Raw DMA with every chip-select released, so no scanner sees the clocks.
  scannerSpiSelectClock(spi_clock_scanner);
  t0 = time_us_32();
  for (uint16_t i = 0; i < N; i++) {
    scannerSpiDmaTransfer(nullptr, rx, SCANNER_FRAME_SIZE);
  }
  const uint32_t dma64_us = time_us_32() - t0;
  t0 = time_us_32();
  for (uint16_t i = 0; i < 20; i++) {
    scannerSpiDmaTransfer(nullptr, pkt_bulk_rx, PKT_BATCH_MAX_BYTES);
  }
  const uint32_t dma4k_us = time_us_32() - t0;
  scannerSpiSelectClock(spi_clock_bulk);
  t0 = time_us_32();
  for (uint16_t i = 0; i < 20; i++) {
    scannerSpiDmaTransfer(nullptr, pkt_bulk_rx, PKT_BATCH_MAX_BYTES);
  }
  const uint32_t dma4k_bulk_us = time_us_32() - t0;
  scannerSpiSelectClock(spi_clock_scanner);

  scannerSerialPrintfTry(
      "BENCH frame %lu.%02lu us, cs assert+release %lu.%02lu us, clock switch pair %lu.%02lu us, "
      "dma 64B %lu.%02lu us, dma 4KB %lu us, dma 4KB at bulk clock %lu us\n",
      (unsigned long)(frame_us / N), (unsigned long)(frame_us % N * 100 / N),
      (unsigned long)(cs_pair_us / N), (unsigned long)(cs_pair_us % N * 100 / N),
      (unsigned long)(clock_switch_us / N), (unsigned long)(clock_switch_us % N * 100 / N),
      (unsigned long)(dma64_us / N), (unsigned long)(dma64_us % N * 100 / N),
      (unsigned long)(dma4k_us / 20), (unsigned long)(dma4k_bulk_us / 20));
}
#endif

[[noreturn]] static void runMonitorProbe() {
  if (SCANNER_MONITOR_PROBE_SPI_DRAIN) {
    scannerSerialPrintfTry("Monitor probe: dwell %lu ms, type mask 0x%02X, data snaplen %u, "
                           "SPI batch drain at %lu Hz\n",
                           (unsigned long)SCANNER_MONITOR_PROBE_DWELL_MS,
                           SCANNER_MONITOR_PROBE_TYPE_MASK, SCANNER_MONITOR_PROBE_SNAPLEN_DATA,
                           (unsigned long)SCANNER_SPI_BULK_HZ);
  } else {
    scannerSerialPrintfTry("Monitor probe: dwell %lu ms, type mask 0x%02X, data snaplen %u, "
                           "simulated drain %u KiB/s\n",
                           (unsigned long)SCANNER_MONITOR_PROBE_DWELL_MS,
                           SCANNER_MONITOR_PROBE_TYPE_MASK, SCANNER_MONITOR_PROBE_SNAPLEN_DATA,
                           SCANNER_MONITOR_PROBE_DRAIN_KIBPS);
  }
#if SCANNER_USE_SHIFTREG_CS
  pktBenchTransport();
#endif
  uint32_t rotation = 0;
  uint32_t next_rotate_ms = 0;
  uint32_t next_report_ms = 0;
  uint32_t last_report_ms = to_ms_since_boot(get_absolute_time());
  while (true) {
    const uint32_t now = to_ms_since_boot(get_absolute_time());
    if (scanner_runtime_context.wd_core1_last_ms != nullptr) {
      *scanner_runtime_context.wd_core1_last_ms = now;
    }
    const bool rotate = (int32_t)(now - next_rotate_ms) >= 0;
    if (rotate) {
      next_rotate_ms = now + SCANNER_MONITOR_PROBE_DWELL_MS;
      monitor_probe_synthetic_kibps = SCANNER_MONITOR_PROBE_SYNTHETIC_KIBPS * (rotation + 1);
    }
    const bool report = (int32_t)(now - next_report_ms) >= 0;
    if (report) {
      next_report_ms = now + 1000;
    }
    if (!rotate && !report) {
      // Between control passes, spend the bus on the slots that need it.
      if (SCANNER_MONITOR_PROBE_SPI_DRAIN && pktServicePass(now)) {
        continue;
      }
      delayMicroseconds(50);
      continue;
    }

    uint32_t total_bytes = 0;
    uint32_t total_busy_us = 0;
    const uint32_t report_dt_ms = now - last_report_ms;
    for (uint8_t slot = 0; slot < SCANNER_SLOT_COUNT; slot++) {
      MonitorSlotState& state = monitor_slots[slot];
      // Each slot starts on a different channel and steps one channel per dwell.
      const uint8_t channel =
          MONITOR_PROBE_CHANNELS[(slot + rotation) % MONITOR_PROBE_CHANNEL_COUNT];
      scannerSetActiveSlot(slot);
      probeHeartbeat();
      if (state.request_outstanding) {
        // Control commands share the slot's frame stream; close the exchange.
        pktFinishSlot(state);
      }
      if (!state.present) {
        if ((int32_t)(now - state.next_probe_ms) < 0) {
          continue;
        }
        scannerReleaseBulkWait();
        scannerDrainStaleFrames(2);
        DeviceIdReply id = {};
        if (!scannerGetDeviceId(id) || (id.capabilities & CAP_MONITOR) == 0) {
          state.next_probe_ms = now + MONITOR_PROBE_REPROBE_MS;
          continue;
        }
        state = {};
        state.present = true;
        scannerSerialPrintfTry("MON s%u monitor-capable scanner found\n", slot);
        monitorProbeTune(slot, state, channel, now);
        continue;
      }
      if (rotate && state.channel != channel) {
        monitorProbeTune(slot, state, channel, now);
      } else if (report) {
        monitorProbeReport(slot, state, now);
      }
      if (report && SCANNER_MONITOR_PROBE_SPI_DRAIN) {
        pktReportSlot(slot, state, report_dt_ms);
        pktReportDiag(slot);
        total_bytes += state.spi_record_bytes;
        total_busy_us += state.spi_busy_us;
        state.spi_record_bytes = 0;
        state.spi_records = 0;
        state.spi_batches = 0;
        state.spi_polls = 0;
        state.spi_crc_errors = 0;
        state.spi_bulk_retries = 0;
        state.spi_duplicates = 0;
        state.spi_aborts = 0;
        state.spi_reply_timeouts = 0;
        state.spi_busy_replies = 0;
        state.spi_busy_us = 0;
        state.spi_synth_gaps = 0;
      }
    }
    if (report) {
      if (SCANNER_MONITOR_PROBE_SPI_DRAIN && report_dt_ms > 0) {
        scannerSerialPrintfTry("SPI total %lu KiB/s bus busy %lu%% synth target %lu KiB/s/slot\n",
                               (unsigned long)(static_cast<uint64_t>(total_bytes) * 1000u /
                                               report_dt_ms / 1024),
                               (unsigned long)(total_busy_us / (report_dt_ms * 10)),
                               (unsigned long)monitor_probe_synthetic_kibps);
      }
      last_report_ms = now;
    }
    if (rotate) {
      rotation++;
    }
  }
}
#endif

// ---------- Monitor mode hand-off ----------
// core0 writes the request and reads status; core1 reads the request and
// writes status. Both sides copy under monitor_link_lock (declared with the
// bus state because controllerScannerRuntimeInitBus() initialises it).
static ControllerMonitorRequest monitor_link_request = {};
static volatile uint32_t monitor_link_generation = 0;
static volatile uint32_t monitor_link_applied_generation = 0;
static ControllerMonitorSlotStatus monitor_link_status[CONTROLLER_MONITOR_MAX_SLOTS] = {};
static constexpr uint32_t MONITOR_LINK_STATUS_INTERVAL_MS = 200;

void controllerScannerMonitorRequest(const ControllerMonitorRequest& request) {
  critical_section_enter_blocking(&monitor_link_lock);
  monitor_link_request = request;
  monitor_link_generation = monitor_link_generation + 1;
  critical_section_exit(&monitor_link_lock);
}

bool controllerScannerMonitorApplied() {
  return monitor_link_applied_generation == monitor_link_generation;
}

void controllerScannerMonitorSlotStatus(uint8_t slot, ControllerMonitorSlotStatus& out) {
  out = {};
  if (slot >= CONTROLLER_MONITOR_MAX_SLOTS) {
    return;
  }
  critical_section_enter_blocking(&monitor_link_lock);
  out = monitor_link_status[slot];
  critical_section_exit(&monitor_link_lock);
}

static void monitorLinkPublish() {
  ControllerMonitorSlotStatus status[SCANNER_SLOT_COUNT] = {};
  for (uint8_t slot = 0; slot < SCANNER_SLOT_COUNT; slot++) {
    const MonitorSlotState& state = monitor_slots[slot];
    status[slot].present = state.present;
    status[slot].monitoring = state.monitoring;
    status[slot].hopping = state.monitoring && state.applied_cfg.hopping;
    status[slot].band = state.band;
    status[slot].channel = state.channel;
    status[slot].radio_drops = state.scanner_ring_drops;
    status[slot].transport_errors = state.transport_errors_total;
  }
  critical_section_enter_blocking(&monitor_link_lock);
  for (uint8_t slot = 0; slot < SCANNER_SLOT_COUNT && slot < CONTROLLER_MONITOR_MAX_SLOTS; slot++) {
    monitor_link_status[slot] = status[slot];
  }
  critical_section_exit(&monitor_link_lock);
}

// Identify the active slot as a monitor-capable scanner if not already known.
static bool monitorLinkEnsurePresent(MonitorSlotState& state) {
  if (state.present) {
    return true;
  }
  scannerReleaseBulkWait();
  scannerDrainStaleFrames(2);
  DeviceIdReply id = {};
  if (!scannerGetDeviceId(id) || (id.capabilities & CAP_MONITOR) == 0) {
    return false;
  }
  state.present = true;
  return true;
}

static bool monitorSlotConfigEqual(const ControllerMonitorSlotConfig& a,
                                   const ControllerMonitorSlotConfig& b) {
  if (a.type_mask != b.type_mask || a.snaplen_mgmt != b.snaplen_mgmt ||
      a.snaplen_ctrl != b.snaplen_ctrl || a.snaplen_data != b.snaplen_data ||
      a.hopping != b.hopping) {
    return false;
  }
  if (!a.hopping) {
    return a.band == b.band && a.channel == b.channel;
  }
  if (a.hop_count != b.hop_count || a.hop_offset != b.hop_offset ||
      a.hop_dwell_ms != b.hop_dwell_ms) {
    return false;
  }
  for (uint8_t i = 0; i < a.hop_count; i++) {
    if (a.hop[i].band != b.hop[i].band || a.hop[i].channel != b.hop[i].channel) {
      return false;
    }
  }
  return true;
}

// Session-wide hop epoch (controller ms). Every hopping slot switches on
// epoch + n * dwell, so staggered slots stay on different channels instead of
// overlapping for part of each dwell.
static uint32_t monitor_hop_epoch_ms = 0;

// Install a hop list on the active slot (already in capture mode), in as many
// CMD_MONITOR_HOP frames as it takes. Any rejected chunk aborts the list.
static int8_t monitorSlotSetHop(const ControllerMonitorSlotConfig& cfg) {
  const uint8_t total = cfg.hop_count > MONITOR_HOP_MAX_ENTRIES ? MONITOR_HOP_MAX_ENTRIES
                                                                : cfg.hop_count;
  const uint32_t dwell = cfg.hop_dwell_ms ? cfg.hop_dwell_ms : MONITOR_HOP_MIN_DWELL_MS;
  uint8_t first = 0;
  do {
    MonitorHopCommand hop = {};
    hop.cmd = CMD_MONITOR_HOP;
    hop.total_count = total;
    hop.dwell_ms = cfg.hop_dwell_ms;
    // Join the shared schedule: the entry this slot should be on now, and the
    // time left until the next shared boundary. Computed per frame so the
    // last chunk (the one that commits) carries current values.
    const uint32_t elapsed = millis() - monitor_hop_epoch_ms;
    const uint32_t dwells = elapsed / dwell;
    hop.start_index = total ? static_cast<uint8_t>((cfg.hop_offset + dwells) % total) : 0;
    uint32_t delay = dwell - elapsed % dwell;
    if (delay < 2) {
      // Too close to a boundary to make it; start one entry later instead.
      hop.start_index = total ? static_cast<uint8_t>((hop.start_index + 1) % total) : 0;
      delay += dwell;
    }
    hop.first_hop_delay_ms = static_cast<uint16_t>(delay > dwell ? dwell : delay);
    hop.first_index = first;
    const uint8_t left = static_cast<uint8_t>(total - first);
    hop.entry_count = left > MONITOR_HOP_ENTRIES_PER_FRAME ? MONITOR_HOP_ENTRIES_PER_FRAME : left;
    for (uint8_t i = 0; i < hop.entry_count; i++) {
      hop.entries[i].band = cfg.hop[first + i].band;
      hop.entries[i].channel = cfg.hop[first + i].channel;
    }
    uint8_t tx[SCANNER_FRAME_SIZE] = {0};
    uint8_t rx[SCANNER_FRAME_SIZE] = {0};
    memcpy(tx, &hop, sizeof(hop));
    bool acked = false;
    for (uint8_t attempt = 0; attempt < 3 && !acked; attempt++) {
      acked = monitorSlotCommand(tx, rx);
    }
    if (!acked) {
      return SCANNER_STATUS_TRANSPORT_TIMEOUT;
    }
    if (static_cast<int8_t>(rx[0]) != SCANNER_STATUS_OK) {
      return static_cast<int8_t>(rx[0]);
    }
    first = static_cast<uint8_t>(first + hop.entry_count);
  } while (first < total);
  return SCANNER_STATUS_OK;
}

// Bring every slot in line with the request: start, retune or stop capture.
// Slots already capturing with the requested settings are left running, so
// retuning one slot does not flush the others' scanner rings.
static void monitorLinkApply(const ControllerMonitorRequest& request, uint32_t now_ms) {
  if (request.active && !monitor_link_streaming) {
    // New session: hop boundaries are counted from here. Slots configured
    // later in this pass join the schedule at their current position.
    monitor_hop_epoch_ms = now_ms;
  }
  uint8_t wanted = 0;
  uint8_t capturing = 0;
  uint8_t restarted = 0;
  for (uint8_t slot = 0; slot < SCANNER_SLOT_COUNT; slot++) {
    MonitorSlotState& state = monitor_slots[slot];
    scannerSetActiveSlot(slot);
    probeHeartbeat();
    if (state.request_outstanding) {
      pktFinishSlot(state);
    }
    const bool want = request.active && slot < CONTROLLER_MONITOR_MAX_SLOTS &&
                      (request.slot_mask & (1u << slot));
    if (!want) {
      if (state.monitoring) {
        monitorSlotStop();
      }
      state.monitoring = false;
      continue;
    }
    wanted++;
    if (!monitorLinkEnsurePresent(state)) {
      scannerSerialPrintfTry("S%u no monitor-capable scanner; not capturing\n", slot);
      state.monitoring = false;
      continue;
    }
    const ControllerMonitorSlotConfig& cfg = request.slot[slot];
    if (state.monitoring && monitorSlotConfigEqual(state.applied_cfg, cfg)) {
      capturing++;
      continue;
    }
    MonitorStartCommand start = {};
    start.band = cfg.band;
    start.channel = cfg.channel;
    if (cfg.hopping && cfg.hop_count > 0) {
      // Start near where the shared schedule has this slot; the hop list then
      // corrects to the exact entry.
      const uint32_t dwell = cfg.hop_dwell_ms ? cfg.hop_dwell_ms : MONITOR_HOP_MIN_DWELL_MS;
      const uint32_t dwells = (millis() - monitor_hop_epoch_ms) / dwell;
      const ControllerMonitorHopEntry& first =
          cfg.hop[(cfg.hop_offset + dwells) % cfg.hop_count];
      start.band = first.band;
      start.channel = first.channel;
    }
    start.type_mask = cfg.type_mask;
    start.snaplen_mgmt = cfg.snaplen_mgmt;
    start.snaplen_ctrl = cfg.snaplen_ctrl;
    start.snaplen_data = cfg.snaplen_data;
    int8_t status = monitorSlotStart(start);
    if (status == SCANNER_STATUS_OK && cfg.hopping && cfg.hop_count > 1) {
      status = monitorSlotSetHop(cfg);
    }
    if (status != SCANNER_STATUS_OK) {
      scannerSerialPrintfTry("S%u monitor start on ch %u failed: %d\n", slot, start.channel, status);
      if (state.present) {
        monitorSlotStop();
      }
      state.monitoring = false;
      continue;
    }
    // START discards the scanner's ring, including any unacknowledged batch.
    state.monitoring = true;
    state.applied_cfg = cfg;
    state.band = start.band;
    state.channel = start.channel;
    state.request_outstanding = false;
    state.ack_pending = false;
    state.pending_bytes = 0;
    state.last_poll_ms = now_ms;
    state.synth_next_seq = 0;
    state.scanner_ring_drops = 0;
    state.transport_errors_total = 0;
    capturing++;
    restarted++;
  }

  const bool was_streaming = monitor_link_streaming;
  monitor_link_streaming = request.active;
  if (request.active) {
    scannerSerialPrintfTry(
        "Monitor capture: %u of %u requested slots capturing (%u restarted); CSV logging paused\n",
        capturing, wanted, restarted);
  } else if (was_streaming) {
    // Scanning re-identifies every slot, which also clears any leftover
    // capture state on the scanner side.
    for (uint8_t slot = 0; slot < SCANNER_SLOT_COUNT; slot++) {
      scanner_slot_state[slot].id_valid = false;
      scanner_slot_state[slot].next_probe_ms = 0;
    }
    scannerSerialPrintfTry("Monitor capture stopped; scanning and CSV logging resumed\n");
  }
  monitorLinkPublish();
}

// One core1 step of monitor mode. Returns true while monitor mode owns the
// scanners (the scan scheduler must not run).
static bool monitorLinkService() {
  static uint32_t next_publish_ms = 0;
  const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
  if (monitor_link_generation != monitor_link_applied_generation) {
    // ~1.2 KB with hop lists: keep it off core1's small stack.
    static ControllerMonitorRequest request;
    critical_section_enter_blocking(&monitor_link_lock);
    request = monitor_link_request;
    const uint32_t generation = monitor_link_generation;
    critical_section_exit(&monitor_link_lock);
    monitorLinkApply(request, now_ms);
    monitor_link_applied_generation = generation;
  }
  if (!monitor_link_streaming) {
    return false;
  }
  if (!pktServicePass(now_ms)) {
    delayMicroseconds(50);
  }
  if ((int32_t)(now_ms - next_publish_ms) >= 0) {
    monitorLinkPublish();
    next_publish_ms = now_ms + MONITOR_LINK_STATUS_INTERVAL_MS;
  }
  return true;
}

void controllerScannerRuntimeRun(const ControllerScannerRuntimeContext& context) {
  // core1 lives here forever after startup. controller_main.cpp only builds the
  // context and launches this loop on the second core.
  scanner_runtime_context = context;
  scanner_slot = SCANNER_INITIAL_SLOT;

  scannerSerialPrintlnTry("Core1 SPI loop started");
#if SCANNER_USE_SHIFTREG_CS
  for (uint8_t s = 0; s < SCANNER_SLOT_COUNT; s++) {
    scannerSetActiveSlot(s);
    scannerDrainStaleFrames(1);
  }
  scannerSetActiveSlot(scanner_slot);
#else
  scannerDrainStaleFrames();
#endif

#if SCANNER_MONITOR_PROBE
  runMonitorProbe();
#endif

  while (true) {
    if (scanner_runtime_context.wd_core1_last_ms != nullptr) {
      *scanner_runtime_context.wd_core1_last_ms =
          to_ms_since_boot(get_absolute_time());
    }
    if (monitorLinkService()) {
      continue;
    }
    handlePendingDedupeResetRequest();
    // Each iteration services exactly one slot; round-robin when shift-reg CS is enabled.
    scannerSetActiveSlot(scanner_slot);
    if (ensureScannerSlotIdentity(scanner_slot, scanner_slot_state[scanner_slot])) {
      processScannerSlot(scanner_slot,
                         scanner_slot_state[scanner_slot],
                         scanner_schedule_state,
                         scanner_sweep_timing_active,
                         scanner_sweep_started_ms);
    }
#if SCANNER_USE_SHIFTREG_CS
    scanner_slot = (uint8_t)((scanner_slot + 1u) % SCANNER_SLOT_COUNT);
#endif
  }
}
