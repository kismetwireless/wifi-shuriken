#!/usr/bin/env python3
"""Capture packets from a WiFi Shuriken over its vendor USB interface into pcap.

The wire format is defined in include/usb_capture_protocol.h; the constants and
struct layouts below mirror it.

Requires pyusb (`pip install pyusb`) and a libusb backend. On Linux, give your
user access to the device, e.g. with a udev rule matching the controller's
VID/PID. No kernel driver binds the vendor interface, so nothing needs detaching.

Capture real traffic, on the device's default per-slot channels, one fixed
channel, or a hop pattern the scanners follow themselves (staggered across
slots unless --no-stagger):

    tools/shuriken_capture.py -w out.pcap
    tools/shuriken_capture.py -w out.pcap --slots 0,1 --channel 6
    tools/shuriken_capture.py -w out.pcap --split --hop 1,6,11,36,40,48,161 --dwell 200
    tools/shuriken_capture.py -w out.pcap --split --types mgmt,data

A .pcapng output (or --pcapng) declares one capture interface per radio,
named shuriken01..shuriken06, so each packet shows which radio heard it:

    tools/shuriken_capture.py -w out.pcapng
    tools/shuriken_capture.py -w - --pcapng | tshark -i - -T fields -e frame.interface_name

Or exercise the transport with the controller's synthetic beacon generator:

    tools/shuriken_capture.py -w out.pcap --synthetic 2000 --synthetic-len 400
    tools/shuriken_capture.py -w out.pcap --synthetic 500 --hop 1,6,11,36,149 --dwell 250
"""

import argparse
import os
import signal
import struct
import sys
import time

try:
    import usb.core
    import usb.util
except ImportError:
    sys.exit("pyusb is required: pip install pyusb")

MAGIC = 0x4853
PROTO_VERSION = 1
IFACE_CLASS = 0xFF
IFACE_SUBCLASS = 0x53
IFACE_PROTOCOL = 0x01

CMD_HELLO = 0x01
CMD_START = 0x02
CMD_STOP = 0x03
CMD_KEEPALIVE = 0x04
CMD_SET_CHANNEL = 0x05
CMD_SET_HOP = 0x06
CMD_SET_FILTER = 0x07
CMD_GET_STATUS = 0x08
REC_REPLY = 0x80
REC_PACKET = 0x81
REC_STATUS = 0x82

REPLY_STATUS_NAMES = {
    0: "OK",
    -1: "UNKNOWN_COMMAND",
    -2: "INVALID",
    -3: "UNSUPPORTED",
    -4: "NOT_ACTIVE",
    -5: "BUSY",
}

CAP_SYNTHETIC = 1 << 0
CAP_RADIO_MONITOR = 1 << 1
CAP_CHANNEL_HOP = 1 << 2

START_SYNTHETIC = 1 << 0
HOP_STAGGER = 1 << 0

FILTER_MGMT = 1 << 0
FILTER_CTRL = 1 << 1
FILTER_DATA = 1 << 2
FILTER_FCS_FAIL = 1 << 3

PKT_FCS_INCLUDED = 1 << 0
PKT_TRUNCATED = 1 << 1
PKT_BAD_FCS = 1 << 2

BAND_24GHZ = 2
BAND_5GHZ = 5

RECORD_HEADER = struct.Struct("<HBBHH")        # magic, type, version, length, seq
REPLY_HEADER = struct.Struct("<BbH")           # command, status, reserved
INFO = struct.Struct("<BBHI16s")               # proto, slots, max_pkt, caps, fw
PACKET_HEADER = struct.Struct("<BBBbbBBBIQHH")
STATUS_HEADER = struct.Struct("<BBHIIIII")
SLOT_STATUS = struct.Struct("<BBBBIII")

LINKTYPE_IEEE802_11_RADIOTAP = 127
# Radiotap: TSFT, Flags, Channel, dBm antenna signal, dBm antenna noise.
RADIOTAP_PRESENT = (1 << 0) | (1 << 1) | (1 << 3) | (1 << 5) | (1 << 6)
RADIOTAP = struct.Struct("<BBHIQBxHHbb")
RT_FLAG_FCS = 0x10
RT_FLAG_BADFCS = 0x40
RT_CHAN_2GHZ = 0x0080
RT_CHAN_5GHZ = 0x0100


def band_for_channel(channel):
    return BAND_24GHZ if channel <= 14 else BAND_5GHZ


def channel_freq(band, channel):
    if band == BAND_24GHZ:
        return 2484 if channel == 14 else 2407 + 5 * channel
    return 5000 + 5 * channel


def radiotap_header(pkt):
    rt_flags = 0
    if pkt["flags"] & PKT_FCS_INCLUDED:
        rt_flags |= RT_FLAG_FCS
    if pkt["flags"] & PKT_BAD_FCS:
        rt_flags |= RT_FLAG_BADFCS
    chan_flags = RT_CHAN_2GHZ if pkt["band"] == BAND_24GHZ else RT_CHAN_5GHZ
    return RADIOTAP.pack(0, 0, RADIOTAP.size, RADIOTAP_PRESENT,
                         pkt["controller_ts"], rt_flags,
                         channel_freq(pkt["band"], pkt["channel"]), chan_flags,
                         pkt["rssi"], pkt["noise"])


def radio_name(slot):
    """Capture interface name for a scanner slot: shuriken01 for slot 0."""
    return "shuriken%02d" % (slot + 1)


class PcapWriter:
    """Classic pcap: one radiotap link for the whole file, no per-radio labels."""

    def __init__(self, path):
        # "-" streams to stdout (e.g. into tshark -i -), flushed per packet.
        self.stream = path == "-"
        self.closed_by_reader = False
        self.f = sys.stdout.buffer if self.stream else open(path, "wb")
        self._emit(self._file_header())

    def _file_header(self):
        return struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535,
                           LINKTYPE_IEEE802_11_RADIOTAP)

    def _emit(self, data):
        self.f.write(data)
        if self.stream:
            self.f.flush()

    def write(self, ts, pkt, frame):
        if self.closed_by_reader:
            return
        try:
            self._emit(self._record(ts, pkt, radiotap_header(pkt) + frame))
        except BrokenPipeError:
            # The reader is gone. Drop everything from here on, and park
            # stdout on /dev/null so exit-time flushes stay quiet.
            self.closed_by_reader = True
            devnull = os.open(os.devnull, os.O_WRONLY)
            os.dup2(devnull, sys.stdout.fileno())
            os.close(devnull)
            raise

    def _record(self, ts, pkt, data):
        sec = int(ts)
        usec = int(round((ts - sec) * 1e6))
        if usec >= 1000000:
            sec += 1
            usec -= 1000000
        orig = RADIOTAP.size + pkt["orig_len"]
        return struct.pack("<IIII", sec, usec, len(data), orig) + data

    def close(self):
        if self.stream:
            if not self.closed_by_reader:
                try:
                    self.f.flush()
                except BrokenPipeError:
                    pass
        else:
            self.f.close()


class PcapngWriter(PcapWriter):
    """pcapng with one radiotap capture interface per scanner radio.

    Each slot gets an Interface Description Block named shuriken01,
    shuriken02, ... and every packet is written against its slot's
    interface, so Wireshark shows which radio heard it (frame.interface_name).
    """

    def __init__(self, path, slots, hardware):
        self.slots = list(slots)
        self.hardware = hardware
        self.if_index = {slot: i for i, slot in enumerate(self.slots)}
        super().__init__(path)

    @staticmethod
    def _option(code, value):
        pad = (-len(value)) % 4
        return struct.pack("<HH", code, len(value)) + value + b"\0" * pad

    @staticmethod
    def _block(block_type, body):
        total = 12 + len(body)
        return struct.pack("<II", block_type, total) + body + struct.pack("<I", total)

    def _file_header(self):
        # Section Header Block: byte-order magic, version 1.0, unknown length.
        shb_opts = (self._option(2, self.hardware.encode()) +           # shb_hardware
                    self._option(4, b"tools/shuriken_capture.py") +     # shb_userappl
                    struct.pack("<HH", 0, 0))
        out = self._block(0x0A0D0D0A,
                          struct.pack("<IHHq", 0x1A2B3C4D, 1, 0, -1) + shb_opts)
        for slot in self.slots:
            # Interface Description Block: radiotap, microsecond timestamps.
            idb_opts = (self._option(2, radio_name(slot).encode()) +    # if_name
                        self._option(3, ("WiFi Shuriken radio slot %d" % slot).encode()) +
                        self._option(9, b"\x06") +                       # if_tsresol 10^-6
                        struct.pack("<HH", 0, 0))
            out += self._block(0x00000001, struct.pack(
                "<HHI", LINKTYPE_IEEE802_11_RADIOTAP, 0, 0) + idb_opts)
        return out

    def _record(self, ts, pkt, data):
        # Enhanced Packet Block on the slot's interface.
        usec = int(round(ts * 1e6))
        orig = RADIOTAP.size + pkt["orig_len"]
        body = struct.pack("<IIIII", self.if_index[pkt["slot"]], usec >> 32,
                           usec & 0xFFFFFFFF, len(data), orig)
        body += data + b"\0" * ((-len(data)) % 4)
        return self._block(0x00000006, body)


class Shuriken:
    def __init__(self, serial=None):
        self.dev, self.intf = self._find(serial)
        usb.util.claim_interface(self.dev, self.intf.bInterfaceNumber)
        self.ep_out = usb.util.find_descriptor(
            self.intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
            == usb.util.ENDPOINT_OUT)
        self.ep_in = usb.util.find_descriptor(
            self.intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
            == usb.util.ENDPOINT_IN)
        if self.ep_out is None or self.ep_in is None:
            raise RuntimeError("capture interface is missing bulk endpoints")
        self.seq = 0
        self.rx = bytearray()
        self.resync_bytes = 0

    @staticmethod
    def _find(serial):
        for dev in usb.core.find(find_all=True):
            if serial is not None:
                try:
                    if usb.util.get_string(dev, dev.iSerialNumber) != serial:
                        continue
                except (usb.core.USBError, ValueError):
                    continue
            try:
                cfg = dev.get_active_configuration()
            except usb.core.USBError:
                continue
            for intf in cfg:
                if (intf.bInterfaceClass, intf.bInterfaceSubClass, intf.bInterfaceProtocol) == (
                        IFACE_CLASS, IFACE_SUBCLASS, IFACE_PROTOCOL):
                    return dev, intf
        raise RuntimeError("no WiFi Shuriken capture interface found")

    def close(self):
        usb.util.release_interface(self.dev, self.intf.bInterfaceNumber)
        usb.util.dispose_resources(self.dev)

    def send(self, cmd_type, body=b""):
        self.seq = (self.seq + 1) & 0xFFFF
        rec = RECORD_HEADER.pack(MAGIC, cmd_type, PROTO_VERSION,
                                 RECORD_HEADER.size + len(body), self.seq) + body
        self.ep_out.write(rec, timeout=1000)
        return self.seq

    def records(self, timeout_ms=100):
        """Read from the IN endpoint and yield every complete record."""
        try:
            self.rx += self.ep_in.read(16384, timeout=timeout_ms)
        except usb.core.USBTimeoutError:
            pass
        while len(self.rx) >= RECORD_HEADER.size:
            magic, rtype, _version, length, seq = RECORD_HEADER.unpack_from(self.rx)
            if magic != MAGIC or length < RECORD_HEADER.size:
                # Lost framing (e.g. stale bytes from an earlier host): slide
                # forward until the next record start.
                del self.rx[0]
                self.resync_bytes += 1
                continue
            if len(self.rx) < length:
                break
            body = bytes(self.rx[RECORD_HEADER.size:length])
            del self.rx[:length]
            yield rtype, seq, body

    def command(self, cmd_type, body=b"", timeout=2.0, on_record=None):
        """Send a command and wait for its reply; other records go to on_record."""
        seq = self.send(cmd_type, body)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for rtype, rseq, rbody in self.records():
                if rtype == REC_REPLY and rseq == seq:
                    command, status, _ = REPLY_HEADER.unpack_from(rbody)
                    if command != cmd_type:
                        continue
                    return status, rbody[REPLY_HEADER.size:]
                if on_record is not None:
                    on_record(rtype, rseq, rbody)
        raise RuntimeError("timed out waiting for reply to command 0x%02x" % cmd_type)

    def checked(self, name, cmd_type, body=b"", on_record=None):
        status, payload = self.command(cmd_type, body, on_record=on_record)
        if status != 0:
            raise RuntimeError("%s failed: %s" % (name, REPLY_STATUS_NAMES.get(status, status)))
        return payload


def parse_packet(body):
    (slot, band, channel, rssi, noise, rate, sig_mode, flags, radio_ts, controller_ts,
     orig_len, cap_len) = PACKET_HEADER.unpack_from(body)
    frame = body[PACKET_HEADER.size:PACKET_HEADER.size + cap_len]
    return dict(slot=slot, band=band, channel=channel, rssi=rssi, noise=noise, rate=rate,
                sig_mode=sig_mode, flags=flags, radio_ts=radio_ts,
                controller_ts=controller_ts, orig_len=orig_len, cap_len=cap_len), frame


def parse_status(body):
    fields = STATUS_HEADER.unpack_from(body)
    status = dict(zip(("active", "slot_count", "reserved", "uptime_ms", "tx_records",
                       "tx_bytes", "ring_drop_records", "ring_drop_bytes"), fields))
    slots = []
    for i in range(status["slot_count"]):
        off = STATUS_HEADER.size + i * SLOT_STATUS.size
        if off + SLOT_STATUS.size > len(body):
            break
        flags, band, channel, _r, packets, radio_drops, transport_errors = \
            SLOT_STATUS.unpack_from(body, off)
        slots.append(dict(flags=flags, band=band, channel=channel, packets=packets,
                          radio_drops=radio_drops, transport_errors=transport_errors))
    status["slots"] = slots
    return status


def parse_channel_list(text):
    entries = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        channel = int(item)
        entries.append((band_for_channel(channel), channel))
    return entries


def slot_mask_from_arg(text, slot_count):
    if text is None:
        return (1 << slot_count) - 1
    mask = 0
    for item in text.split(","):
        slot = int(item)
        if not 0 <= slot < slot_count:
            raise ValueError("slot %d out of range (device has %d)" % (slot, slot_count))
        mask |= 1 << slot
    return mask


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("-w", "--write",
                    help="pcap output path, or - for stdout (with --split: path prefix)")
    ap.add_argument("--split", action="store_true",
                    help="write one pcap per slot (<prefix>-s<N>.pcap)")
    ap.add_argument("--pcapng", action="store_true",
                    help="write pcapng with one capture interface per radio, named "
                         "shuriken01..shuriken0N (implied by a .pcapng output name)")
    ap.add_argument("--serial", help="select device by USB serial number")
    ap.add_argument("--slots", help="comma-separated slot list (default: all)")
    ap.add_argument("--channel", type=int, help="tune selected slots to one channel")
    ap.add_argument("--hop", help="comma-separated channel hop list")
    ap.add_argument("--dwell", type=int, default=200, help="hop dwell in ms (default 200)")
    ap.add_argument("--no-stagger", action="store_true",
                    help="start every slot at the same hop index")
    ap.add_argument("--types", default="mgmt,data",
                    help="frame types to capture: mgmt,ctrl,data,fcsfail (default mgmt,data)")
    ap.add_argument("--snaplen-mgmt", type=int, default=0)
    ap.add_argument("--snaplen-ctrl", type=int, default=0)
    ap.add_argument("--snaplen-data", type=int, default=256)
    ap.add_argument("--synthetic", type=int, metavar="PPS",
                    help="run the controller's synthetic beacon generator at PPS")
    ap.add_argument("--synthetic-len", type=int, default=200,
                    help="synthetic frame length in bytes (default 200)")
    ap.add_argument("--duration", type=float, help="stop after this many seconds")
    ap.add_argument("--info", action="store_true", help="print device info and exit")
    ap.add_argument("-q", "--quiet", action="store_true", help="no per-second stats")
    args = ap.parse_args()

    if args.channel is not None and args.hop is not None:
        ap.error("--channel and --hop are mutually exclusive")

    dev = Shuriken(args.serial)
    payload = dev.checked("HELLO", CMD_HELLO)
    proto, slot_count, max_pkt, caps, fw = INFO.unpack_from(payload)
    fw = fw.split(b"\0", 1)[0].decode(errors="replace")
    print("Shuriken proto=%d fw=%s slots=%d max_packet=%d caps=%s" % (
        proto, fw, slot_count, max_pkt,
        ",".join(n for b, n in ((CAP_SYNTHETIC, "synthetic"),
                                (CAP_RADIO_MONITOR, "monitor"),
                                (CAP_CHANNEL_HOP, "hop")) if caps & b) or "none"),
        file=sys.stderr)
    if args.info:
        dev.close()
        return

    if not args.write:
        ap.error("-w/--write is required unless --info is given")
    if args.write == "-" and args.split:
        ap.error("--split needs a file path prefix, not -")
    if args.synthetic is None and not caps & CAP_RADIO_MONITOR:
        ap.error("this firmware has no radio monitor mode yet; use --synthetic PPS")

    slot_mask = slot_mask_from_arg(args.slots, slot_count)

    type_bits = {"mgmt": FILTER_MGMT, "ctrl": FILTER_CTRL, "data": FILTER_DATA,
                 "fcsfail": FILTER_FCS_FAIL}
    type_mask = 0
    for name in args.types.split(","):
        if name.strip() not in type_bits:
            ap.error("unknown frame type %r" % name)
        type_mask |= type_bits[name.strip()]
    dev.checked("SET_FILTER", CMD_SET_FILTER, struct.pack(
        "<BBHHH", slot_mask, type_mask, args.snaplen_mgmt, args.snaplen_ctrl,
        args.snaplen_data))

    if args.channel is not None:
        dev.checked("SET_CHANNEL", CMD_SET_CHANNEL, struct.pack(
            "<BBBB", slot_mask, band_for_channel(args.channel), args.channel, 0))
    elif args.hop is not None:
        entries = parse_channel_list(args.hop)
        flags = 0 if args.no_stagger else HOP_STAGGER
        body = struct.pack("<BBHB3x", slot_mask, flags, args.dwell, len(entries))
        body += b"".join(struct.pack("<BB", band, ch) for band, ch in entries)
        dev.checked("SET_HOP", CMD_SET_HOP, body)

    use_pcapng = args.pcapng or args.write.lower().endswith(".pcapng")
    hardware = "WiFi Shuriken fw %s" % fw
    writers = {}

    def writer_for(slot):
        key = slot if args.split else None
        if key not in writers:
            path = args.write
            if args.split:
                root, ext = os.path.splitext(args.write)
                path = "%s-s%d%s" % (root, slot, ext or (".pcapng" if use_pcapng else ".pcap"))
            if use_pcapng:
                # One file per slot carries just that radio; otherwise every
                # radio is declared up front so interface ids match slots.
                radios = [slot] if args.split else range(slot_count)
                writers[key] = PcapngWriter(path, radios, hardware)
            else:
                writers[key] = PcapWriter(path)
        return writers[key]

    stats = dict(packets=0, bytes=0, seq_gaps=0, last_status=None)
    clock = dict(host0=None, ctrl0=None)
    expected_seq = [None]

    def on_record(rtype, seq, body):
        if rtype == REC_PACKET:
            if expected_seq[0] is not None and seq != expected_seq[0]:
                stats["seq_gaps"] += (seq - expected_seq[0]) & 0xFFFF
            expected_seq[0] = (seq + 1) & 0xFFFF
            pkt, frame = parse_packet(body)
            if clock["host0"] is None:
                clock["host0"] = time.time()
                clock["ctrl0"] = pkt["controller_ts"]
            ts = clock["host0"] + (pkt["controller_ts"] - clock["ctrl0"]) / 1e6
            writer_for(pkt["slot"]).write(ts, pkt, frame)
            stats["packets"] += 1
            stats["bytes"] += len(body)
        elif rtype == REC_STATUS:
            stats["last_status"] = parse_status(body)

    flags = 0
    rate = 0
    if args.synthetic is not None:
        flags |= START_SYNTHETIC
        rate = args.synthetic
    dev.checked("START", CMD_START,
                struct.pack("<BBHI", slot_mask, flags, args.synthetic_len, rate),
                on_record=on_record)

    running = [True]
    signal.signal(signal.SIGINT, lambda *_: running.__setitem__(0, False))
    started = time.monotonic()
    next_keepalive = started + 1.0
    next_report = started + 1.0
    last_report = dict(packets=0, bytes=0, t=started)

    try:
        while running[0]:
            try:
                for rtype, seq, body in dev.records():
                    on_record(rtype, seq, body)
            except BrokenPipeError:
                # The reader on our stdout went away (e.g. tshark -a duration).
                print("output pipe closed; stopping capture", file=sys.stderr)
                break
            now = time.monotonic()
            if now >= next_keepalive:
                dev.send(CMD_KEEPALIVE)
                next_keepalive = now + 1.0
            if now >= next_report:
                dt = now - last_report["t"]
                pps = (stats["packets"] - last_report["packets"]) / dt
                kbps = (stats["bytes"] - last_report["bytes"]) / dt / 1024
                st = stats["last_status"] or {}
                if not args.quiet:
                    print("%8.1f pkt/s %8.1f KiB/s  total=%d ring_drops=%s seq_gaps=%d resync=%d" % (
                        pps, kbps, stats["packets"], st.get("ring_drop_records", "?"),
                        stats["seq_gaps"], dev.resync_bytes), file=sys.stderr)
                last_report = dict(packets=stats["packets"], bytes=stats["bytes"], t=now)
                next_report = now + 1.0
            if args.duration is not None and now - started >= args.duration:
                break
    finally:
        try:
            dev.command(CMD_STOP, on_record=on_record)
        except (RuntimeError, usb.core.USBError) as exc:
            print("STOP failed: %s" % exc, file=sys.stderr)
        for w in writers.values():
            w.close()
        dev.close()

    elapsed = time.monotonic() - started
    st = stats["last_status"] or {}
    print("captured %d packets in %.1fs (%.1f pkt/s), ring drops=%s, seq gaps=%d" % (
        stats["packets"], elapsed, stats["packets"] / elapsed if elapsed else 0,
        st.get("ring_drop_records", "?"), stats["seq_gaps"]), file=sys.stderr)


if __name__ == "__main__":
    main()
