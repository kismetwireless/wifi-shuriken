// scanner_platform_esp32_monitor.h
#pragma once

#include <stdint.h>
#include "../../include/spi_protocol_shared.h"

// Promiscuous-mode capture. Frames are copied from the WiFi driver callback
// into a CaptureRing as MonitorPacketRecords and leave it in CMD_PKT_READ
// batches. With MONITOR_START_FLAG_SIM_DRAIN the ring is instead discarded
// locally, optionally rate limited, to load-test the radio without SPI.

// Start or retune capture. Returns a ScannerStatus code.
int8_t scannerMonitorStart(const MonitorStartCommand& command);

void scannerMonitorStop();

bool scannerMonitorActive();

// Track ring usage and run the simulated drain; call from the main loop.
void scannerMonitorService();

// Handle CMD_MONITOR_HOP: stage a chunk of the hop list, and start hopping
// once the last chunk arrives. Returns a ScannerStatus code.
int8_t scannerMonitorSetHop(const MonitorHopCommand& command);

// Handle CMD_PKT_READ: release the acknowledged batch, then build (or keep,
// if still unacknowledged) the next one in batch_out, which must hold
// PKT_BATCH_MAX_BYTES. Returns the wire length to clock, 0 if none.
uint16_t scannerMonitorPrepareBatch(const PktReadCommand& command, uint8_t* batch_out,
                                    PktReadReply& reply);

void scannerMonitorFillStats(MonitorStatsReply& out);
