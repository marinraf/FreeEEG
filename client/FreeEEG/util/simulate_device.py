"""
Fake FreeEEG device: speaks the same UDP protocol as the firmware
(Firmware/components/eeg_net/wifi_app_data.c) so the client tools can be
tested without the board.

    python -m FreeEEG.util.simulate_device            # terminal 1
    python -m FreeEEG.util.plot --ip 127.0.0.1 --port 4321 -e 2 -e 3 -g 24 --psd   # terminal 2

Simulated montage (common reference at Fp2, bias at Fp1):
    ch0 = F7 - Fp2, ch1 = F8 - Fp2, ch2 = O1 - Fp2, ch3 = O2 - Fp2
    - 10 Hz alpha on O1/O2
    - a blink every ~4 s at Fp2 (shows up, inverted, on every channel and
      cancels in derivations such as ch0-ch2)
    - horizontal eye movements: opposite sign on F7 and F8
    - 50 Hz mains common to all channels
"""
import argparse
import math
import random
import socket
import struct
import time

VREF = 4.5
ADC_MAX = 8388607


def volts_to_counts(v, gain):
    return int(max(-ADC_MAX, min(ADC_MAX, v * gain * ADC_MAX / VREF)))


def build_packet(samples, imu, n_elec, loff_p=0, loff_n=0):
    """samples: [(t_sec_float, [counts]*n_elec)], imu: [(t, (ax,ay,az), (gx,gy,gz))]"""
    t0 = samples[0][0] if samples else (imu[0][0] if imu else time.time())
    abs_time = int(t0)
    rel = lambda t: int(round((t - abs_time) * 1e6))  # firmware: tv_usec (+1e6 on second rollover)

    fmt = f"<IBB{len(samples)}I{len(samples) * n_elec}iB{len(imu)}I{len(imu) * 3}h{len(imu) * 3}hBBB"
    values = [abs_time, n_elec, len(samples)]
    values += [rel(t) for t, _ in samples]
    values += [v for _, ch in samples for v in ch]
    values += [len(imu)]
    values += [rel(t) for t, _, _ in imu]
    values += [v for _, a, _ in imu for v in a]
    values += [v for _, _, g in imu for v in g]
    values += [0]  # no events
    values += [loff_p, loff_n]  # lead-off flags
    return struct.pack(fmt, *values)


def eeg_sample(t, n_elec, gain):
    alpha = 30e-6 * math.sin(2 * math.pi * 10 * t) * (0.6 + 0.4 * math.sin(2 * math.pi * 0.1 * t))
    blink_phase = t % 4.0
    blink = 150e-6 * math.exp(-((blink_phase - 0.3) / 0.08) ** 2)        # at Fp2 (the reference)
    saccade = 60e-6 * (1 if (t % 6.0) < 3.0 else -1)                     # horizontal EOG
    mains = 20e-6 * math.sin(2 * math.pi * 50 * t)
    noise = lambda: random.gauss(0, 3e-6)

    f7 = -saccade + noise()
    f8 = saccade + noise()
    o1 = alpha + noise()
    o2 = 0.9 * alpha + noise()
    electrodes = [f7, f8, o1, o2] + [noise() for _ in range(4)]
    return [volts_to_counts(e - blink + mains, gain) for e in electrodes[:n_elec]]


def main():
    parser = argparse.ArgumentParser(description="Simulated FreeEEG device")
    parser.add_argument("--port", type=int, default=4321)
    parser.add_argument("--channels", type=int, default=4, help="number of active channels")
    parser.add_argument("--fs", type=float, default=250.0)
    parser.add_argument("--gain", type=int, default=24)
    parser.add_argument("--loff", type=int, action="append", default=[],
                        help="pretend this channel (1-8) has lost contact (repeatable)")
    args = parser.parse_args()

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("", args.port))
    print(f"Simulated FreeEEG listening on UDP {args.port} ({args.channels} channels, {args.fs} SPS, gain {args.gain})")

    while True:
        udp.settimeout(None)
        msg, client = udp.recvfrom(64)
        text = msg.decode(errors="ignore").strip()
        if text.startswith("time "):
            print(f"clock set to {float(text[5:]):.3f}")
            continue
        if text != "start":
            continue
        print(f"start from {client}")
        udp.settimeout(0)

        next_t = time.time()
        last_imu = next_t
        streaming = True
        while streaming:
            time.sleep(0.03)  # firmware sends a packet every ~30 ms
            now = time.time()

            samples = []
            while next_t <= now and len(samples) < 30:  # UDP_ELEC_BUFF_SIZE
                samples.append((next_t, eeg_sample(next_t, args.channels, args.gain)))
                next_t += 1.0 / args.fs

            imu = []
            while last_imu + 0.01 <= now and len(imu) < 5:  # UDP_IMU_BUFF_SIZE
                last_imu += 0.01
                imu.append((last_imu, (random.randint(-50, 50), random.randint(-50, 50), 2048 + random.randint(-20, 20)),
                            (random.randint(-10, 10), random.randint(-10, 10), random.randint(-10, 10))))

            if samples or imu:
                loff = sum(1 << (c - 1) for c in args.loff)
                udp.sendto(build_packet(samples, imu, args.channels, loff_p=loff), client)

            try:
                msg, _ = udp.recvfrom(64)
                if msg.decode(errors="ignore").strip() == "stop":
                    print("stop")
                    streaming = False
            except BlockingIOError:
                pass


if __name__ == "__main__":
    main()
