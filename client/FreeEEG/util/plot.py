from FreeEEG import FreeEEG, Packet, contact_report

if __name__ == '__main__':
    import signal
    import datetime
    import argparse
    from sys import argv
    from collections import deque
    import math
    
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    parser = argparse.ArgumentParser(
        prog='PlotElectrodes',
        description='Plot and save the signal from selected electrodes'
    )

    parser.add_argument('-s', '--save', action='store_true')
    parser.add_argument('-e', '--elec', type=int, choices=list(range(FreeEEG.N_ELECTRODE)), action='append', default=[],
                        help='active channel index to plot (0 = first powered-up channel). Repeatable.')
    parser.add_argument('-d', '--deriv', type=str, action='append', default=[],
                        help='derivation A-B between active channel indexes, e.g. -d 0-2 plots ch0 minus ch2. Repeatable.')
    parser.add_argument('--ip', type=str, default='192.168.4.1', help='device IP (127.0.0.1 for simulate_device.py)')
    parser.add_argument('--port', type=int, default=69, help='device UDP port (4321 for simulate_device.py)')
    parser.add_argument('-i', '--imu', type=str, choices=['a.x', 'a.y', 'a.z', 'g.x', 'g.y', 'g.z'], action='append')
    parser.add_argument('-g', '--gain', type=int)
    parser.add_argument('--psd', action='store_true')
    parser.add_argument('--fs', type=float, default=250.0)
    parser.add_argument('--notch50', action='store_true')
    
    args = parser.parse_args(argv[1:])

    # Each trace is (label, channel_a, channel_b or None): a single channel or a
    # derivation a - b computed from two channels that share the same reference.
    TRACES = {f'ch{e}': (e, None) for e in args.elec}
    for d in args.deriv:
        try:
            a, b = (int(x) for x in d.split('-'))
        except ValueError:
            parser.error(f'invalid derivation "{d}", expected A-B, e.g. 0-2')
        TRACES[f'ch{a}-ch{b}'] = (a, b)
    if not TRACES:
        parser.error('select at least one channel (-e) or derivation (-d)')

    BUFFER_SIZE = 1000
    CHANNELS_TO_PLOT = list(TRACES)
    IMU_TO_PLOT = args.imu or []

    imu_index = {
        'a.x': ('accel', 0),
        'a.y': ('accel', 1),
        'a.z': ('accel', 2),
        'g.x': ('gyro', 0),
        'g.y': ('gyro', 1),
        'g.z': ('gyro', 2),
    }

    eeg = FreeEEG(args.ip, args.port)

    data_stream = eeg.get_data_generator(blocking=False)
    generator = data_stream()

    time_data = deque(maxlen=BUFFER_SIZE)
    channel_data = {ch: deque(maxlen=BUFFER_SIZE) for ch in CHANNELS_TO_PLOT}
    
    imu_time_data = deque(maxlen=BUFFER_SIZE)
    imu_data = {name: deque(maxlen=BUFFER_SIZE) for name in IMU_TO_PLOT}

    notch_state = {
        ch: {
            "x1": 0.0,
            "x2": 0.0,
            "y1": 0.0,
            "y2": 0.0,
        }
    for ch in CHANNELS_TO_PLOT
    }

    def make_notch_coeffs(f0, fs, q=30.0):
        w0 = 2.0 * math.pi * f0 / fs
        alpha = math.sin(w0) / (2.0 * q)

        b0 = 1.0
        b1 = -2.0 * math.cos(w0)
        b2 = 1.0
        a0 = 1.0 + alpha
        a1 = -2.0 * math.cos(w0)
        a2 = 1.0 - alpha

        return b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0

    notch_b0, notch_b1, notch_b2, notch_a1, notch_a2 = make_notch_coeffs(50.0, args.fs)

    def apply_notch_50(value, ch):
        state = notch_state[ch]

        y = (
            notch_b0 * value
            + notch_b1 * state["x1"]
            + notch_b2 * state["x2"]
            - notch_a1 * state["y1"]
            - notch_a2 * state["y2"]
        )

        state["x2"] = state["x1"]
        state["x1"] = value
        state["y2"] = state["y1"]
        state["y1"] = y

        return y

    t0 = None
    last_contact = None
    
    def signal_handler(sig, frame):
        eeg.stop()
        if args.save:
            csv.close()
        plt.close('all')

    signal.signal(signal.SIGINT, signal_handler)

    n_subplots = 1 + bool(IMU_TO_PLOT) + bool(args.psd)
    fig, axes = plt.subplots(n_subplots, 1, figsize=(10, 4 * n_subplots), sharex=False)

    if n_subplots == 1:
        axes = [axes]

    ax_eeg = axes[0]
    axis_idx = 1

    ax_imu = None
    if IMU_TO_PLOT:
        ax_imu = axes[axis_idx]
        axis_idx += 1

    ax_psd = None
    if args.psd:
        ax_psd = axes[axis_idx]

    eeg_lines = {}
    for ch in CHANNELS_TO_PLOT:
        line, = ax_eeg.plot([], [], label=ch)
        eeg_lines[ch] = line

    ax_eeg.set_title('FreeEEG real-time EEG data')
    ax_eeg.set_xlabel('Time (s)')
    ax_eeg.set_ylabel('Voltage (V)' if args.gain else 'ADC counts')
    ax_eeg.grid(True)
    ax_eeg.legend(loc='upper right')

    imu_lines = {}
    if IMU_TO_PLOT:
        for name in IMU_TO_PLOT:
            line, = ax_imu.plot([], [], label=name)
            imu_lines[name] = line

        ax_imu.set_title('FreeEEG real-time IMU data')
        ax_imu.set_xlabel('Time (s)')
        ax_imu.set_ylabel('Raw IMU value')
        ax_imu.grid(True)
        ax_imu.legend(loc='upper right')

    psd_lines = {}
    if args.psd:
        for ch in CHANNELS_TO_PLOT:
            line, = ax_psd.plot([], [], label=ch)
            psd_lines[ch] = line

        ax_psd.set_title('EEG power spectral density')
        ax_psd.set_xlabel('Frequency (Hz)')
        ax_psd.set_ylabel('PSD (V²/Hz)')
        ax_psd.set_xlim(0, args.fs / 2)
        ax_psd.grid(True)
        ax_psd.legend(loc='upper right')

    if args.save:
        start = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        csv = open(f"recording_{start}.csv", "w+")
        csv.write(Packet.CSV_HEADER)

    def compute_psd(values, fs):
        y = np.asarray(values, dtype=float)

        if len(y) < 8:
            return None, None

        y = y - np.mean(y)
        window = np.hanning(len(y))
        y = y * window

        freqs = np.fft.rfftfreq(len(y), d=1.0 / fs)

        psd = np.abs(np.fft.rfft(y)) ** 2
        psd /= fs * np.sum(window ** 2)

        return freqs, psd

    def update(frame):
        global t0, csv, last_contact

        packets = []
        while (packet := next(generator)) != None:
            packets.append(packet)

        if len(packets) == 0:
            return tuple(eeg_lines.values()) + tuple(imu_lines.values()) + tuple(psd_lines.values())

        if t0 is None:
            t0 = packets[0].abs_time  # x axis in seconds since the first packet

        for packet in packets:
            if packet.elec_data:
                contact = contact_report(packet.loff_p, packet.loff_n)
                if contact != last_contact:
                    print(f"Electrode contact: {contact}")
                    last_contact = contact
            if args.save:
                csv.write(packet.as_csv(args.gain))
            for t, data in packet.elec_data:
                ts = t/1e6 + packet.abs_time - t0
                time_data.append(ts)

                for ch in CHANNELS_TO_PLOT:
                    a, b = TRACES[ch]
                    if max(a, b if b is not None else 0) >= len(data):
                        # exceptions inside the animation callback are swallowed by matplotlib
                        print(f'{ch}: the device is only sending {len(data)} active channels (indexes 0-{len(data)-1})')
                        plt.close('all')
                        return ()
                    adc_value = data[a] - (data[b] if b is not None else 0)
                    volts = adc_value
                    if args.gain != None:
                        volts = FreeEEG.adc_to_volts(adc_value, args.gain)

                    if args.notch50:
                        volts = apply_notch_50(volts, ch)

                    channel_data[ch].append(volts)

            for t, accel, gyro in packet.imu_data:
                ts = t/1e6 + packet.abs_time - t0
                imu_time_data.append(ts)

                for name in IMU_TO_PLOT:
                    sensor, axis = imu_index[name]

                    if sensor == 'accel':
                        imu_data[name].append(accel[axis])
                    else:
                        imu_data[name].append(gyro[axis])

        for ch in CHANNELS_TO_PLOT:
            eeg_lines[ch].set_data(time_data, channel_data[ch])

        for name in IMU_TO_PLOT:
            imu_lines[name].set_data(imu_time_data, imu_data[name])

        if args.psd:
            max_psd = 0

            for ch in CHANNELS_TO_PLOT:
                freqs, psd = compute_psd(channel_data[ch], args.fs)

                if freqs is None:
                    continue

                mask = freqs > 1
                psd = psd[mask]
                freqs = freqs[mask]

                psd_lines[ch].set_data(freqs, psd)
                max_psd = max(max_psd, np.max(psd))

            if max_psd > 0:
                ax_psd.set_ylim(0, max_psd * 1.1)
                ax_psd.set_xlim(1,50)

        if len(time_data) >= 2:
            ax_eeg.set_xlim(time_data[0], time_data[-1])

        if len(imu_time_data) >= 2 and ax_imu is not None:
            ax_imu.set_xlim(imu_time_data[0], imu_time_data[-1])

        eeg_values = [value for ch in CHANNELS_TO_PLOT for value in channel_data[ch]]

        if eeg_values:
            ymin = min(eeg_values)
            ymax = max(eeg_values)
            margin = 0.1 * (ymax - ymin) if ymax != ymin else 1e-6
            ax_eeg.set_ylim(ymin - margin, ymax + margin)

        imu_values = [value for name in IMU_TO_PLOT for value in imu_data[name]]

        if imu_values and ax_imu is not None:
            ymin = min(imu_values)
            ymax = max(imu_values)
            margin = 0.1 * (ymax - ymin) if ymax != ymin else 1
            ax_imu.set_ylim(ymin - margin, ymax + margin)

        return tuple(eeg_lines.values()) + tuple(imu_lines.values()) + tuple(psd_lines.values())

    eeg.start()
    print("Starting EEG...")

    # import cProfile, pstats, io
    # from pstats import SortKey
    try:    
        # pr = cProfile.Profile()
        # pr.enable()
        animation = FuncAnimation(fig, update, interval=33, blit=False, cache_frame_data=False)
        plt.show()
        # pr.disable()
    finally:
        eeg.stop()
        # s = io.StringIO()
        # sortby = SortKey.CUMULATIVE
        # ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
        # ps.print_stats()
        # print(s.getvalue())
        if args.save:
            csv.close()
