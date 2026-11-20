"""
Quick "is anything coming out?" check for a FreeEEG board, with no
dependencies beyond the standard library.

Connect the computer to the board's WiFi access point, then:

    python3 Client/FreeEEG/util/probe.py                 # 10 s, channel 1, gain 1
    python3 Client/FreeEEG/util/probe.py -c 1 -g 24 -t 30

It sends "start", recognises which firmware is answering from the packet
format, prints once per second the mean and the RMS (in uV) of the chosen
channels, and sends "stop" at the end (or on Ctrl+C).

Packet formats recognised:
  * v0.5 firmware (commit 1898459): raw packed C struct, always 1177 bytes,
    30 samples x 8 channels.
  * current firmware (eeg_net/wifi_app_data.c): compact format, only the
    samples and channels actually present.
  * anything else (for example the microphone test firmware, which streams
    raw I2S audio on "start") is reported as unknown.
"""
import argparse
import cmath
import socket
import struct
import time

VREF = 4.5
ADC_MAX = 8388607.0
V05_SIZE = 4 + 1 + 30 * 4 + 30 * 8 * 4 + 1 + 5 * 4 + 5 * 6 + 5 * 6 + 1 + 2 * 4 + 2  # 1177


def parse_v05(data):
    """Returns a list of samples, each a list of 8 counts."""
    num = data[4]
    rows = struct.unpack_from("<240i", data, 5 + 30 * 4)
    return [list(rows[s * 8:(s + 1) * 8]) for s in range(min(num, 30))]


def parse_compact(data):
    """Returns (samples, loff_p, loff_n), samples being a list of lists of
    num_elec counts, or None if the bytes do not follow the compact format.
    loff_* are None for firmware that does not send lead-off flags."""
    if len(data) < 8:
        return None
    _, ne, ns = struct.unpack_from("<IBB", data, 0)
    if ne > 8 or ns > 30:
        return None
    off = 6 + ns * 4
    try:
        vals = struct.unpack_from(f"<{ns * ne}i", data, off)
        off += ns * ne * 4
        ni = data[off]
        off += 1 + ni * 4 + ni * 12
        nev = data[off]
        off += 1 + nev * 5
    except (struct.error, IndexError):
        return None
    samples = [list(vals[s * ne:(s + 1) * ne]) for s in range(ns)]
    if off == len(data):
        return samples, None, None
    if off + 2 == len(data):
        return samples, data[off], data[off + 1]
    return None


def dominant_freq(col, fs):
    """Frequency (Hz) of the largest DFT bin, ignoring DC."""
    n = len(col)
    mean = sum(col) / n
    best, best_f = 0.0, 0.0
    for k in range(1, n // 2):
        acc = sum((v - mean) * cmath.exp(-2j * cmath.pi * k * i / n) for i, v in enumerate(col))
        if abs(acc) > best:
            best, best_f = abs(acc), k * fs / n
    return best_f


def main():
    ap = argparse.ArgumentParser(description="FreeEEG quick signal check")
    ap.add_argument("--ip", default="192.168.4.1")
    ap.add_argument("--port", type=int, default=69)
    ap.add_argument("-c", "--channel", type=int, action="append",
                    help="channel number 1-8 as labelled on the web page (repeatable, default 1)")
    ap.add_argument("-g", "--gain", type=int, default=1, help="gain configured for those channels")
    ap.add_argument("-t", "--time", type=float, default=10.0, help="seconds to record")
    args = ap.parse_args()
    channels = args.channel or [1]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", 6969))
    sock.settimeout(1.0)
    dev = (args.ip, args.port)

    sock.sendto(f"time {time.time():.6f}".encode(), dev)  # the board has no clock battery
    sock.sendto(b"start", dev)
    print(f"'start' sent to {args.ip}:{args.port}, listening for {args.time:.0f} s...\n")

    fmt_seen = None
    n_ch_seen = None
    contact_seen = None
    sizes = {}
    window = []
    n_packets = 0
    t_end = time.time() + args.time
    t_print = time.time() + 1.0
    try:
        while time.time() < t_end:
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                print("  (nothing received in the last second)")
                continue
            n_packets += 1
            sizes[len(data)] = sizes.get(len(data), 0) + 1

            loff = (None, None)
            if len(data) == V05_SIZE:
                fmt, samples = "v0.5 (raw struct, 8 channels)", parse_v05(data)
            else:
                parsed = parse_compact(data)
                fmt, samples = (None, None) if parsed is None else ("current (compact)", parsed[0])
                if parsed is not None:
                    loff = parsed[1:]
            if fmt is None:
                if fmt_seen is None:
                    print(f"Unknown format: {len(data)}-byte packet, starts with {data[:16].hex(' ')}")
                    fmt_seen = "unknown"
                continue
            if fmt_seen != fmt:
                print(f"Firmware format: {fmt}\n")
                fmt_seen = fmt
            if samples and len(samples[0]) != n_ch_seen:
                n_ch_seen = len(samples[0])
                print(f"Packets carry {n_ch_seen} channel(s) "
                      "(= number of channels NOT powered down in the active config)\n")
            if samples and loff[0] is not None:
                off = [f"CH{i + 1}" for i in range(8) if loff[0] >> i & 1] + (["REF"] if loff[1] else [])
                contact = "OK (or detection disabled)" if not off else "OFF: " + ", ".join(off)
                if contact != contact_seen:
                    print(f"Electrode contact: {contact}")
                    contact_seen = contact
            window.extend(samples)

            if time.time() >= t_print and window:
                t_print += 1.0
                parts = []
                for ch in channels:
                    col = [s[ch - 1] for s in window if len(s) >= ch]
                    if not col:
                        parts.append(f"CH{ch}: --")
                        continue
                    mean = sum(col) / len(col)
                    rms = (sum((v - mean) ** 2 for v in col) / len(col)) ** 0.5
                    k = VREF / (args.gain * ADC_MAX) * 1e6  # counts -> uV
                    p2p = (max(col) - min(col)) * k
                    sat = " SATURATED" if max(abs(v) for v in col) >= ADC_MAX - 1 else ""
                    f = dominant_freq(col, len(col))
                    parts.append(f"CH{ch}: mean {mean * k:10.1f} uV  rms {rms * k:8.1f} uV  "
                                 f"p2p {p2p:8.1f} uV  peak {f:5.1f} Hz{sat}")
                print(f"{len(window):4d} samples/s | " + " | ".join(parts))
                window = []
    except KeyboardInterrupt:
        pass
    finally:
        sock.sendto(b"stop", dev)
        sock.close()

    print(f"\n'stop' sent. {n_packets} packets received; sizes: {sizes or 'none'}")
    if n_packets == 0:
        print("Nothing arrived: check you are on the board's WiFi and that it shows 192.168.4.1.")
    elif fmt_seen == "unknown":
        print("This firmware does not send EEG in a known format (it may be the microphone test build).")


if __name__ == "__main__":
    main()
