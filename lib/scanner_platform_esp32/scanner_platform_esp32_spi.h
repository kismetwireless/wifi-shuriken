// scanner_platform_esp32_spi.h
#pragma once

#include <stddef.h>
#include <stdint.h>

enum ScannerSpiTransferResult : int {
  SCANNER_SPI_TRANSFER_ERROR = -1,
  SCANNER_SPI_TRANSFER_TIMEOUT = 0,
  SCANNER_SPI_TRANSFER_OK = 1,
};

// Most transactions the platform keeps queued at once.
static constexpr uint8_t SCANNER_SPI_MAX_QUEUED = 3;

struct ScannerSpiTransaction {
  const uint8_t* tx_buf;
  uint8_t* rx_buf;
  size_t length;
};

// max_transfer_bytes bounds the longest transaction (packet batches); every
// buffer handed to this layer must be word aligned in DMA RAM.
bool scannerSpiSlaveInit(int pin_sck,
                         int pin_mosi,
                         int pin_miso,
                         int pin_cs,
                         size_t max_transfer_bytes,
                         uint8_t queue_size);

// Number of queued transactions not yet returned by scannerSpiSlaveWait().
uint8_t scannerSpiSlavePending();

// Queue transactions to run back to back; the driver arms each one as soon as
// the previous completes, without waiting for the caller. Only valid when
// nothing is pending; count must be 1..SCANNER_SPI_MAX_QUEUED.
bool scannerSpiSlaveQueue(const ScannerSpiTransaction* transactions, uint8_t count);

// Wait for the oldest pending transaction. On OK, *index is its position in
// the scannerSpiSlaveQueue() call and *received_bits how many bits the master
// clocked (the C5 driver reports a few bits more than were sent).
int scannerSpiSlaveWait(uint32_t timeout_ms, uint8_t* index, size_t* received_bits);

// Convenience for the single-frame case: queue one transaction if nothing is
// pending, then wait for it.
int scannerSpiSlaveTransfer(const uint8_t* tx_buf,
                            uint8_t* rx_buf,
                            size_t length,
                            uint32_t timeout_ms,
                            size_t* received_bits = nullptr);
