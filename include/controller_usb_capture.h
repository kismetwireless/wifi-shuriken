#pragma once

#include <stdint.h>

#include "usb_capture_protocol.h"

// Register the vendor-class capture interface before USB enumeration starts.
bool controllerUsbCaptureBegin();

// Service host commands, the synthetic generator, and the USB TX pump. Call
// from the core0 loop; never blocks on the host.
void controllerUsbCaptureTask();

// True while a host capture session is running.
bool controllerUsbCaptureActive();

// Queue one captured frame for the host. Single producer only. Fills in the
// controller timestamp. Returns false when no session is active, or when the
// packet ring is full (counted as a ring drop).
bool controllerUsbCaptureQueuePacket(UsbCapturePacketHeader& header, const uint8_t* frame);
