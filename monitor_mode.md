# Monitor mode switching

How a USB host moves the WiFi Shuriken from its normal WiGLE scanning mode into
promiscuous (monitor) capture and back, and what each part of the system does
along the way.

The wire formats live in two headers, which are the source of truth:

- `include/usb_capture_protocol.h` — host ↔ controller (USB vendor interface)
- `include/spi_protocol_shared.h` — controller ↔ scanner (SPI)

`tools/shuriken_capture.py` is the reference host implementation.

## Parts involved

```
 host ──USB vendor bulk──▶ controller core0 ──hand-off──▶ controller core1 ──SPI──▶ scanners (×6)
      ◀── packet/status ──   (USB, CSV, GPS)  ◀─ status ─   (scanner bus)    ◀────
```

- **Host** — opens the capture interface (vendor class, subclass `0x53`,
  protocol `0x01`) and sends command records.
- **Controller core0** — owns USB, the capture session, CSV logging and GPS.
  It decides *what* the radios should do.
- **Controller core1** — owns the scanner SPI bus. It decides *how*: it runs
  either the scan scheduler or the monitor batch engine, never both.
- **Scanners** — ESP32-C5 radios that either scan for networks or capture in
  promiscuous mode, on command.

## Modes

```
                 START (radio)                    STOP / HELLO / keepalive loss / unplug
   ┌──────────┐ ───────────────▶ ┌──────────────┐ ───────────────────────────────────▶ ┌──────────┐
   │ Scanning │                  │ Radio capture│                                       │ Scanning │
   └──────────┘ ◀─────────────── └──────────────┘                                       └──────────┘
        │  ▲          (same exits)       ▲  │ SET_CHANNEL / SET_HOP / SET_FILTER
        │  │                             └──┘ (live retune, session stays up)
        │  │
        ▼  │ STOP / HELLO / keepalive loss / unplug
   ┌───────────────────┐
   │ Synthetic capture │   START with USB_CAPTURE_START_SYNTHETIC (controller-generated
   └───────────────────┘   beacons, scanners keep scanning; for testing the USB path)
```

| Mode | Scanners | CSV logging | USB packet source |
|---|---|---|---|
| Scanning (default) | Active scans per channel schedule | Running | none |
| Radio capture | Promiscuous capture | Paused | core1, from scanner batches |
| Synthetic capture | Scanning | Running | core0 beacon generator |

The device boots into scanning and returns to it whenever a radio session ends.

## Host side: session commands

All records are little-endian and start with `UsbCaptureRecordHeader`
(magic `"SH"`, type, protocol version, length, sequence). Every command gets a
reply record echoing its sequence number and a status byte.

| Command | Effect |
|---|---|
| `HELLO` (0x01) | Starts a fresh host session: stops any session a previous host left running, drops queued output, and **restores every slot's default channel, hop and filter settings**. Replies with `UsbCaptureInfo` (slot count, max packet size, capabilities). |
| `SET_CHANNEL` (0x05) | Fixed channel for the slots in `slot_mask`; clears hopping on them. |
| `SET_HOP` (0x06) | Hop list (≤ 64 entries) and dwell (≥ 10 ms) for the slots in `slot_mask`. With `USB_CAPTURE_HOP_STAGGER`, each slot starts at a different point in the list. |
| `SET_FILTER` (0x07) | Frame types (mgmt/ctrl/data) and per-type snaplen (0 = whole frame). The `FCS_FAIL` bit is ignored for radio capture; scanners only deliver frames received intact. |
| `START` (0x02) | Without `USB_CAPTURE_START_SYNTHETIC`: radio capture on the slots in `slot_mask`. With it: synthetic capture. |
| `KEEPALIVE` (0x04) | Must arrive (or any other command) at least every 5 s while a session runs. |
| `GET_STATUS` (0x08) | Replies, then queues a status record immediately. |
| `STOP` (0x03) | Ends the session; the device returns to scanning. |

Defaults after `HELLO`: slots on channels 1, 6, 11, 36, 44, 149; management and
data frames; data snaplen 256; no hopping.

### What ends a radio session

Any of these returns the device to scanning, with the same teardown:

- `STOP`
- `HELLO` from any host
- no command for `USB_CAPTURE_KEEPALIVE_TIMEOUT_MS` (5 s)
- the host deconfigures or the cable is unplugged

### Reply statuses that matter for switching

| Status | When |
|---|---|
| `OK` (0) | Accepted. For `START`, capture begins asynchronously (see below). |
| `INVALID` (−2) | Bad slot mask, channel, hop list or lengths. |
| `BUSY` (−5) | A synthetic `START` arrived while core1 is still tearing down the last radio session. Retry after a few tens of milliseconds; it has taken ~60 ms in testing. |

`START` replies as soon as the request is recorded. Slots come up over the next
tens to hundreds of milliseconds; watch the status records to see each slot
reach monitor mode.

### Status records

Sent every second during a session and on `GET_STATUS`. Per slot:

| Field | Meaning |
|---|---|
| `flags` | `ENABLED` (in the session mask), `PRESENT` (scanner identified), `MONITOR` (capturing), `HOPPING` |
| `band`, `channel` | Channel in use now (follows hopping). Before a slot is capturing, the channel it has been asked for. |
| `packets` | Packets queued to USB from this slot this session |
| `radio_drops` | Frames the scanner dropped because its capture ring was full |
| `transport_errors` | SPI reply timeouts, abandoned batches and CRC failures |

The header carries session totals: records/bytes sent and USB ring drops. All
counters are per session (reported against a baseline taken at `START`).

### Live changes

`SET_CHANNEL`, `SET_HOP` and `SET_FILTER` during a radio session take effect
without stopping it. Only slots whose settings changed are restarted; the
others keep capturing undisturbed. A retuned slot delivers its first frame on
the new channel within ~0.05–0.2 s.

## Controller: core0 → core1 hand-off

core0 never touches the scanner bus. It publishes a **monitor request**
(`ControllerMonitorRequest` in `include/controller_scanner_runtime.h`):

- `active` — capture on or off
- `slot_mask`
- per slot: band, channel, type mask, snaplens, and hop list / dwell / stagger
  offset when hopping

Each publish bumps a generation counter. core1 checks the counter between slot
services; when it changes, core1 copies the request and **applies** it, then
marks that generation applied. `controllerScannerMonitorApplied()` lets core0
tell when core1 has caught up (this is what gates `BUSY`). core1 publishes
per-slot status back the same way.

### Applying a request (core1)

For each slot:

1. Finish any batch exchange in flight on that slot.
2. **Slot wanted and settings unchanged** → leave it running.
3. **Slot wanted, new or changed** →
   - identify it (`CMD_ID`, monitor capability) if not already known,
   - `CMD_MONITOR_START` with its band/channel/filter,
   - if hopping, `CMD_MONITOR_HOP` with the hop list.
4. **Slot not wanted but capturing** → `CMD_MONITOR_STOP`.

While any request is active, core1 runs the monitor batch engine instead of the
scan scheduler and turns every received frame into a USB packet record.

When the request goes inactive, core1 stops every slot, marks every slot for
re-identification, and resumes the scan scheduler.

### CSV logging

CSV logging is paused by construction: core1 produces no scan results while
monitoring, so core0 has nothing to log. Nothing is buffered or replayed; when
scanning resumes, new results are logged as usual (still subject to the normal
dedupe). The console reports both transitions:

```
Monitor capture: 6 of 6 requested slots capturing (6 restarted); CSV logging paused
Monitor capture stopped; scanning and CSV logging resumed
```

### Single producer for USB packets

The USB packet ring has one producer at a time: core1 during a radio session,
core0's generator during a synthetic session. A synthetic `START` is refused
with `BUSY` until core1 has applied the stop of the previous radio session, so
the two never overlap.

## Controller → scanner: SPI commands

All commands are 64-byte frames; the reply arrives tagged with the command byte
in the last byte of a later frame.

| Command | Scanner behaviour |
|---|---|
| `CMD_MONITOR_START` (0x20) | Aborts any scan in progress and drops unread results, then enters promiscuous mode on the given band/channel with the given filter and snaplens. Resets the capture ring and stops any hopping. Idempotent: the controller retries it freely. |
| `CMD_MONITOR_HOP` (0x25) | Installs a hop list in chunks of up to 24 entries; it takes effect when the chunk holding the last entry arrives. `total_count` 0 turns hopping off. Does not reset the capture ring. |
| `CMD_MONITOR_STOP` (0x21) | Leaves promiscuous mode and restores band mode `AUTO` so scanning can use both bands. |
| `CMD_PKT_READ` (0x23) | Captured-frame transport; see `spi_protocol_shared.h`. |
| `CMD_MONITOR_STATS` (0x22), `CMD_PKT_DIAG` (0x24) | Diagnostics (used by the monitor probe build). |

Scanners never leave monitor mode on their own and keep their state across a
controller reboot. The controller therefore sends `CMD_MONITOR_STOP` whenever it
identifies a slot for normal scanning, and before starting a scanner firmware
update. An update always takes precedence over capture.

## Hopping and stagger

Hopping runs on the scanners: each retunes itself every dwell, without touching
its capture ring, and labels every frame with the band and channel it was
actually received on (so frames that land mid-hop are correct).

To keep staggered slots on different channels at the same time, all slots hop
on **shared boundaries**. When a radio session starts, core1 fixes an epoch;
each slot switches at `epoch + n × dwell`. When core1 sends a hop list it
computes, from the epoch:

- `start_index` — the entry this slot should be on now (its stagger offset plus
  the dwells elapsed), and
- `first_hop_delay_ms` — the time left until the next shared boundary.

Slots configured later, or retuned live, join the same schedule. Measured with
seven channels at 200 ms dwell: hops in list order on every slot, revisit
period within ~0.5% of nominal, and two slots on the same channel in ~3% of
samples (largely timestamp jitter: pcap timestamps are when the controller
queued the frame, not air time).

## Unaffected while monitoring

- **GPS NMEA passthrough** (second CDC port) — tested across a capture: 15
  sentences/s throughout, no missed fixes, no checksum errors.
- **Console** (first CDC port) and **MTP** SD access.

## Example host session

```
HELLO                                   -> OK, info (6 slots, caps: synthetic, monitor, hop)
SET_FILTER  slots=0x3f  mgmt+data, data snaplen 256
SET_HOP     slots=0x3f  stagger, dwell 200 ms, [1 6 11 36 40 48 161]
START       slots=0x3f                  -> OK
  ... packet records, status record every 1 s, KEEPALIVE every 1 s ...
SET_CHANNEL slots=0x01  ch 48           -> OK   (slot 0 stops hopping; others untouched)
STOP                                    -> OK   (scanning and CSV logging resume)
```

With the reference tool:

```
tools/shuriken_capture.py -w out.pcap --split --hop 1,6,11,36,40,48,161 --dwell 200
tools/shuriken_capture.py -w - | tshark -i - -l
```
