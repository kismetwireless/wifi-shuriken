// scanner_platform_esp32_spi.cpp
#include "scanner_platform_esp32_spi.h"
#include "ScannerConfiguration.h"

#include <driver/gpio.h>
#include <driver/spi_slave.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

// Queued transactions live in persistent storage so timeouts never leave the
// SPI ISR holding a pointer to a stack-local transaction object. Results come
// back in queue order, so g_next_result walks the same array.
static spi_slave_transaction_t g_trans[SCANNER_SPI_MAX_QUEUED] = {};
static uint8_t g_queued = 0;
static uint8_t g_next_result = 0;
static bool g_spi_initialized = false;

static void resetQueueState() {
  memset(g_trans, 0, sizeof(g_trans));
  g_queued = 0;
  g_next_result = 0;
}

bool scannerSpiSlaveInit(int pin_sck,
                         int pin_mosi,
                         int pin_miso,
                         int pin_cs,
                         size_t max_transfer_bytes,
                         uint8_t queue_size) {
  // ESP32-C5 slave mode runs on SPI2 host.
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = pin_mosi;
  buscfg.miso_io_num = pin_miso;
  buscfg.sclk_io_num = pin_sck;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = max_transfer_bytes;

  spi_slave_interface_config_t slvcfg = {};
  slvcfg.spics_io_num = pin_cs;
  slvcfg.queue_size = queue_size < SCANNER_SPI_MAX_QUEUED ? SCANNER_SPI_MAX_QUEUED : queue_size;
  slvcfg.mode = 0;

  const esp_err_t ret = spi_slave_initialize(
    SPI2_HOST, &buscfg, &slvcfg, static_cast<spi_dma_chan_t>(SCANNER_SPI_DMA_CH));
  if (ret != ESP_OK) {
    return false;
  }

  const esp_err_t drive_ret = gpio_set_drive_capability(
    static_cast<gpio_num_t>(pin_miso),
    static_cast<gpio_drive_cap_t>(SCANNER_SPI_MISO_DRIVE_CAP));
  if (drive_ret != ESP_OK) {
    return false;
  }

  resetQueueState();
  g_spi_initialized = true;
  return true;
}

uint8_t scannerSpiSlavePending() {
  return static_cast<uint8_t>(g_queued - g_next_result);
}

bool scannerSpiSlaveQueue(const ScannerSpiTransaction* transactions, uint8_t count) {
  if (!g_spi_initialized || scannerSpiSlavePending() != 0 || count == 0 ||
      count > SCANNER_SPI_MAX_QUEUED) {
    return false;
  }
  resetQueueState();
  for (uint8_t i = 0; i < count; i++) {
    spi_slave_transaction_t& trans = g_trans[i];
    trans.length = transactions[i].length * 8;
    trans.tx_buffer = transactions[i].tx_buf;
    trans.rx_buffer = transactions[i].rx_buf;
    if (spi_slave_queue_trans(SPI2_HOST, &trans, 0) != ESP_OK) {
      // Earlier entries are already with the driver and still complete in
      // order; only report what made it in.
      g_queued = i;
      return false;
    }
    g_queued = static_cast<uint8_t>(i + 1);
  }
  return true;
}

int scannerSpiSlaveWait(uint32_t timeout_ms, uint8_t* index, size_t* received_bits) {
  if (!g_spi_initialized || scannerSpiSlavePending() == 0) {
    return SCANNER_SPI_TRANSFER_ERROR;
  }

  spi_slave_transaction_t* completed = nullptr;
  const esp_err_t ret = spi_slave_get_trans_result(
    SPI2_HOST, &completed, pdMS_TO_TICKS(timeout_ms));
  if (ret == ESP_ERR_TIMEOUT) {
    return SCANNER_SPI_TRANSFER_TIMEOUT;
  }
  if (ret != ESP_OK || completed != &g_trans[g_next_result]) {
    resetQueueState();
    return SCANNER_SPI_TRANSFER_ERROR;
  }

  if (index != nullptr) {
    *index = g_next_result;
  }
  if (received_bits != nullptr) {
    *received_bits = completed->trans_len;
  }
  g_next_result++;
  return SCANNER_SPI_TRANSFER_OK;
}

int scannerSpiSlaveTransfer(const uint8_t* tx_buf,
                            uint8_t* rx_buf,
                            size_t length,
                            uint32_t timeout_ms,
                            size_t* received_bits) {
  if (scannerSpiSlavePending() == 0) {
    const ScannerSpiTransaction trans = {tx_buf, rx_buf, length};
    if (!scannerSpiSlaveQueue(&trans, 1)) {
      return SCANNER_SPI_TRANSFER_ERROR;
    }
  }
  return scannerSpiSlaveWait(timeout_ms, nullptr, received_bits);
}
