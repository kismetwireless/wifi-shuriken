/*
 * Project TinyUSB override for RP2040/RP2350 builds.
 * Enables a second CDC interface, read-write MTP storage access, and the
 * vendor-class packet capture interface.
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU OPT_MCU_RP2040
#endif

#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE
#define CFG_TUSB_OS OPT_OS_PICO

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE 64
#endif

//------------- CLASS -------------//
#define CFG_TUD_HID (2)
#define CFG_TUD_CDC (2)
#define CFG_TUD_MSC (0)
#define CFG_TUD_MTP (1)
#define CFG_TUD_MIDI (1)
#define CFG_TUD_VENDOR (1)

#define CFG_TUD_CDC_RX_BUFSIZE (256)
#define CFG_TUD_CDC_TX_BUFSIZE (256)

// Packet capture vendor interface. The TX transfer buffer spans many 64-byte
// full-speed packets so one transfer callback can move a large chunk; the FIFO
// absorbs short core0 stalls in front of the controller's own packet ring.
#define CFG_TUD_VENDOR_RX_BUFSIZE (512)
#define CFG_TUD_VENDOR_TX_BUFSIZE (8192)
#define CFG_TUD_VENDOR_RX_EPSIZE (64)
#define CFG_TUD_VENDOR_TX_EPSIZE (2048)

// Larger than the 64-byte full-speed bulk packet so TinyUSB can stream many
// packets per transfer callback and SdFat can read larger chunks from the card.
#define CFG_TUD_MTP_EP_BUFSIZE (16384)
#define CFG_TUD_MTP_EP_CONTROL_BUFSIZE (16)

#define CFG_TUD_MTP_DEVICEINFO_EXTENSIONS "microsoft.com: 1.0; "
#define CFG_TUD_MTP_DEVICEINFO_SUPPORTED_OPERATIONS \
  MTP_OP_GET_DEVICE_INFO, MTP_OP_OPEN_SESSION, MTP_OP_CLOSE_SESSION, \
  MTP_OP_GET_STORAGE_IDS, MTP_OP_GET_STORAGE_INFO, \
  MTP_OP_GET_DEVICE_PROP_DESC, MTP_OP_GET_DEVICE_PROP_VALUE, \
  MTP_OP_GET_OBJECT_HANDLES, MTP_OP_GET_OBJECT_INFO, \
  MTP_OP_GET_OBJECT, MTP_OP_GET_PARTIAL_OBJECT, \
  MTP_OP_DELETE_OBJECT, MTP_OP_SEND_OBJECT_INFO, MTP_OP_SEND_OBJECT
#define CFG_TUD_MTP_DEVICEINFO_SUPPORTED_EVENTS
#define CFG_TUD_MTP_DEVICEINFO_SUPPORTED_DEVICE_PROPERTIES \
  MTP_DEV_PROP_DEVICE_FRIENDLY_NAME
#define CFG_TUD_MTP_DEVICEINFO_CAPTURE_FORMATS \
  MTP_OBJ_FORMAT_UNDEFINED, MTP_OBJ_FORMAT_ASSOCIATION, MTP_OBJ_FORMAT_TEXT
#define CFG_TUD_MTP_DEVICEINFO_PLAYBACK_FORMATS \
  MTP_OBJ_FORMAT_UNDEFINED, MTP_OBJ_FORMAT_ASSOCIATION, MTP_OBJ_FORMAT_TEXT

#define CFG_TUD_HID_EP_BUFSIZE (64)

#define CFG_TUD_MIDI_RX_BUFSIZE (64)
#define CFG_TUD_MIDI_TX_BUFSIZE (64)

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
