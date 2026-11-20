"""
Reader for the files the board records on the SD card in offline mode
(EEG_0001.BIN, EEG_0002.BIN, ...). Format written by
Firmware/components/eeg_app/tasks.c (write_sd_header / write_sd_records):

    header, 24 bytes, little-endian:
        char    magic[4]      b"FEEG"
        uint8   version       1
        uint16  sample_rate   samples per second
        uint8   comm_ref      1 if the SRB1 common reference was enabled
        uint8   mode[8]       channel mode per channel (0 = powered down, 1 = normal, ...)
        uint8   gain[8]       PGA gain per channel
    records until end of file:
        uint16  length
        uint8   packet[length]   exactly one UDP packet (see protocol.Packet)

Convert to CSV (same layout read_csv.py understands):

    python -m FreeEEG.sdfile EEG_0001.BIN                 # -> EEG_0001.csv, raw ADC counts
    python -m FreeEEG.sdfile EEG_0001.BIN --volts         # electrode columns in volts
"""
import argparse
import struct
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from .protocol import Packet, MalformedPacket, adc_to_volts, N_ELECTRODE

HEADER_FMT = "<4sBHB8B8B"
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 24

MODE_NAMES = ["PWRDWN", "NORMAL", "SHORTED", "BIASMEAS", "MVDD", "TEMP", "TEST", "BIASDRP", "BIASDRN"]


@dataclass
class SDHeader:
    version: int
    sample_rate: int
    comm_ref: bool
    modes: list[int]
    gains: list[int]

    @property
    def active_channels(self) -> list[int]:
        """Channel numbers (1-8) that are present in the data, in order."""
        return [i + 1 for i, m in enumerate(self.modes) if m != 0]

    @property
    def active_gains(self) -> list[int]:
        return [self.gains[ch - 1] for ch in self.active_channels]


def read_sd_file(path):
    """Returns (SDHeader, [Packet, ...]). A truncated last record (battery
    died mid-write) is silently dropped."""
    data = Path(path).read_bytes()
    if len(data) < HEADER_SIZE:
        raise ValueError(f"{path}: too short to be a FreeEEG recording")

    magic, version, fs, comm_ref, *rest = struct.unpack_from(HEADER_FMT, data, 0)
    if magic != b"FEEG":
        raise ValueError(f"{path}: not a FreeEEG recording (magic {magic!r})")
    if version != 1:
        raise ValueError(f"{path}: unsupported file version {version}")
    header = SDHeader(version, fs, bool(comm_ref), list(rest[:N_ELECTRODE]), list(rest[N_ELECTRODE:]))

    packets = []
    off = HEADER_SIZE
    while off + 2 <= len(data):
        (length,) = struct.unpack_from("<H", data, off)
        off += 2
        if off + length > len(data):
            break  # truncated last record
        try:
            packets.append(Packet.parse_packet(data[off:off + length]))
        except (MalformedPacket, struct.error, IndexError) as e:
            print(f"warning: skipping bad record at byte {off - 2}: {e}")
        off += length
    return header, packets


def to_csv(header, packets, out, volts=False):
    """Writes the recording as CSV: one row per EEG sample and one per IMU
    sample, with Packet.CSV_HEADER columns (e1..e8 are the active channels in
    order), preceded by the '#' lines read_csv.py expects."""
    gains = header.active_gains
    conv = (lambda v, g: adc_to_volts(v, g)) if volts else (lambda v, g: v)

    eeg_t = [p.abs_time + t / 1e6 for p in packets for t, _ in p.elec_data]
    imu_t = [p.abs_time + t / 1e6 for p in packets for t, _, _ in p.imu_data]
    rate = lambda ts: round((len(ts) - 1) / (ts[-1] - ts[0])) if len(ts) > 1 and ts[-1] > ts[0] else 0
    # The board has no clock battery: unless the client or the web page set
    # its clock before recording, times count from power-up.
    first = packets[0].abs_time if packets else 0
    synced = first > 1_000_000_000
    start = datetime.fromtimestamp(first, tz=timezone.utc)

    with open(out, "w") as f:
        label = "date (UTC)" if synced else "date (clock not set, time since power-up)"
        f.write(f"# {label}: {start:%Y-%m-%d %H:%M:%S}\n")
        f.write("# active channels: " + ", ".join(f"CH{c}" for c in header.active_channels) + "\n")
        f.write(f"# eeg data rate: {rate(eeg_t)}\n")
        f.write(f"# imu data rate: {rate(imu_t)}\n")
        f.write(Packet.CSV_HEADER)
        for p in packets:
            for t, values in p.elec_data:
                cols = [str(conv(v, g)) for v, g in zip(values, gains)] + [""] * (N_ELECTRODE - len(values))
                f.write(f"{p.abs_time + t / 1e6}," + ",".join(cols) + ",,,,,,,\n")
            for t, (ax, ay, az), (gx, gy, gz) in p.imu_data:
                f.write(f"{p.abs_time + t / 1e6}," + "," * N_ELECTRODE + f"{ax},{ay},{az},{gx},{gy},{gz},\n")


def main():
    ap = argparse.ArgumentParser(description="Convert a FreeEEG SD recording to CSV")
    ap.add_argument("file", help="EEG_XXXX.BIN copied from the SD card")
    ap.add_argument("-o", "--output", help="CSV path (default: same name with .csv)")
    ap.add_argument("--volts", action="store_true", help="electrode columns in volts instead of ADC counts")
    args = ap.parse_args()

    header, packets = read_sd_file(args.file)
    n_eeg = sum(len(p.elec_data) for p in packets)
    n_imu = sum(len(p.imu_data) for p in packets)

    print(f"{args.file}: {header.sample_rate} SPS, common reference {'on' if header.comm_ref else 'off'}")
    for ch in range(1, N_ELECTRODE + 1):
        mode = header.modes[ch - 1]
        name = MODE_NAMES[mode] if mode < len(MODE_NAMES) else str(mode)
        print(f"  CH{ch}: {name:<8} gain {header.gains[ch - 1]}")
    duration = n_eeg / header.sample_rate if header.sample_rate else 0
    print(f"{len(packets)} records, {n_eeg} EEG samples (~{duration / 60:.1f} min), {n_imu} IMU samples")

    out = args.output or str(Path(args.file).with_suffix(".csv"))
    to_csv(header, packets, out, volts=args.volts)
    print(f"written {out}")


if __name__ == "__main__":
    main()
