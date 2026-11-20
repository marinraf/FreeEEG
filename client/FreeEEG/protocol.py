import struct
from dataclasses import dataclass


N_ELECTRODE = 8

# ADS1299 internal reference (CONFIG3.PD_REFBUF = 1). The 5 V rail is AVDD,
# the ADC full scale is +-VREF/gain.
VREF = 4.5
ADC_MAX = 8388607.0  # 2^23 - 1

def adc_to_volts(value, gain):
    return value * VREF / (gain * ADC_MAX)

class MalformedPacket(Exception):
    def __init__(self, msg):
        super().__init__(msg)

def malformed_packet(msg, x):
    raise MalformedPacket(f"{msg} is out of bounds: {x}")

def contact_report(loff_p, loff_n):
    """Human readable lead-off state. Bits are absolute channel numbers
    (bit 0 = CH1). With the common reference (SRB1) the negative inputs are the
    reference electrode, so any loff_n bit means the reference is off."""
    off = [f"CH{i + 1}" for i in range(N_ELECTRODE) if loff_p >> i & 1]
    if loff_n:
        off.append("REF/N")
    return "OK" if not off else "off: " + ", ".join(off)


@dataclass
class Packet:
    abs_time: int
    elec_data: list[tuple[int, list[int]]]
    imu_data: list[tuple[int, tuple[int,int,int], tuple[int,int,int]]]
    event_data: list[tuple[int, int]]
    # Lead-off flags (bit i = CH i+1), 0 if the firmware does not send them
    loff_p: int = 0
    loff_n: int = 0

    CSV_HEADER = "timestamp,e1,e2,e3,e4,e5,e6,e7,e8,ax,ay,az,gx,gy,gz,event\n"

    def parse_packet(data):
        global N_ELECTRODE
        __ELEC_BUFF_SIZE =  30
        __IMU_BUFF_SIZE =   5
        __EVENT_BUFF_SIZE = 2

        check_bound = lambda x, y, msg: x if x <= y else malformed_packet(msg, x)

        offset = 4

        n_elec = check_bound(data[offset], N_ELECTRODE, "# electrodes")
        offset += 1

        n_elec_samp = check_bound(data[offset], __ELEC_BUFF_SIZE, "# electrode samples")
        offset += 4*(n_elec_samp*(1+n_elec))+1

        n_imu = check_bound(data[offset], __IMU_BUFF_SIZE, "# imu samples")
        offset += 16*n_imu+1

        n_events = check_bound(data[offset], __EVENT_BUFF_SIZE, "# of events")

        struct_fmt = (
            "<"
            "I"      # abs_timestamp
            "B"      # num_elec_
            "B"      # num_elec_samp
            f"{n_elec_samp}I"
            f"{n_elec_samp * n_elec}i"
            "B"      # num_imu_samp
            f"{n_imu}I"
            f"{n_imu * 3}h"  # accel (ICM42670 two's complement)
            f"{n_imu * 3}h"  # gyro
            "B"      # num_events_samp
            f"{n_events}I"
            f"{n_events}B"
        )

        unpacked_data = struct.unpack_from(struct_fmt, data)

        # Firmware with lead-off detection appends loff_p, loff_n.
        size = struct.calcsize(struct_fmt)
        loff_p, loff_n = (data[size], data[size + 1]) if len(data) >= size + 2 else (0, 0)

        offset = 0

        abs_timestamp = unpacked_data[offset]
        offset += 3

        elec_timestamps = unpacked_data[offset:offset+n_elec_samp]
        offset += n_elec_samp
        elec_data_flat = unpacked_data[offset:offset+n_elec_samp*n_elec]
        offset += n_elec_samp*n_elec+1

        imu_timestamps = unpacked_data[offset:offset+n_imu]
        offset += n_imu

        accel_data_flat = unpacked_data[offset:offset+n_imu*3]
        offset += n_imu*3

        gyro_data_flat = unpacked_data[offset:offset+n_imu*3]
        offset += n_imu*3+1

        event_timestamps = unpacked_data[offset:offset+n_events]
        offset += n_events

        events = unpacked_data[offset:offset+n_events]
        
        elec_data = [
            (
                elec_timestamps[i],
                elec_data_flat[i * n_elec:(i + 1) * n_elec],
            )
            for i in range(n_elec_samp)
        ]
    
        imu_data = [
            (
                imu_timestamps[i],
                accel_data_flat[i * 3:(i + 1) * 3],
                gyro_data_flat[i * 3:(i + 1) * 3],
            ) for i in range(n_imu)]
    
        event_data = list(zip(event_timestamps, events))
        return Packet(abs_timestamp, elec_data, imu_data, event_data, loff_p, loff_n)        
        
    def as_csv(self, gain=None) -> str:
        """
        One CSV row per electrode sample and one per IMU sample, with the
        layout of CSV_HEADER. Electrode columns e1..e8 are the *active*
        channels in order (the device only sends powered-up channels), unused
        columns are left empty. If gain is given, values are converted to volts.
        """
        conv = (lambda v: v) if gain is None else (lambda v: adc_to_volts(v, gain))
        rows = []
        for et, ed in self.elec_data:
            elec = [str(conv(v)) for v in ed] + [""] * (N_ELECTRODE - len(ed))
            rows.append(f"{self.abs_time + et/1e6}," + ",".join(elec) + ",,,,,,,")

        for it, (ax, ay, az), (gx, gy, gz) in self.imu_data:
            rows.append(f"{self.abs_time + it/1e6}," + "," * N_ELECTRODE + f"{ax},{ay},{az},{gx},{gy},{gz},")

        return "".join(r + "\n" for r in rows)
