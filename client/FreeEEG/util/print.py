from FreeEEG import FreeEEG

if __name__ == '__main__':
    import time
    import signal
    import numpy as np
    import argparse
    parser = argparse.ArgumentParser(description='Print the packets received from the device')
    parser.add_argument('--ip', type=str, default='192.168.4.1', help='device IP (127.0.0.1 for simulate_device.py)')
    parser.add_argument('--port', type=int, default=69, help='device UDP port (4321 for simulate_device.py)')
    args = parser.parse_args()

    eeg = FreeEEG(args.ip, args.port)

    def signal_handler(sig, frame):
        eeg.stop()
        exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    
    data_stream = eeg.get_data_generator(blocking=True)

    eeg.start()
    print("Starting EEG...")
    for i in data_stream():
        print(i)
