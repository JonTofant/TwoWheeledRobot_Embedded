#!/usr/bin/env python3
"""Capture STM32 paper telemetry from the ST-Link VCP and write plot-ready CSV.

Example:
    python Tools/paper_uart_capture.py COM3 --duration 30 --reset \
        --output Results/paper/run_01.csv --plot Results/paper/run_01.png
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
import time
from pathlib import Path


MAGIC = b"\xA5\x5A"
VERSION = 2
SAMPLE_TYPE = 1
HEADER = struct.Struct("<2sBBH")
CRC = struct.Struct("<H")

QUANTITIES = [
    "imu_read",
    "encoder_read",
    "observation",
    "inference",
    "action",
    "motor_write",
    "total",
    "control_period",
    "release_latency",
    "rs485_cycle",
    "imu_age",
    "wheel_age",
    "leg_age",
]

STATE_FLOATS = [
    "pitch_rad",
    "pitch_rate_radps",
    "roll_rad",
    "roll_rate_radps",
    "yaw_rad",
    "yaw_rate_radps",
    "yaw_rate_world_radps",
    "yaw_rate_from_quat_radps",
    "wheel_left_logical_velocity_radps",
    "wheel_right_logical_velocity_radps",
    "position_m",
    "velocity_mps",
    "wheel_left_position_rad",
    "wheel_right_position_rad",
    "wheel_left_velocity_radps",
    "wheel_right_velocity_radps",
    "wheel_left_current_command_A",
    "wheel_right_current_command_A",
    "wheel_left_current_measured_A",
    "wheel_right_current_measured_A",
    "leg_0_angle_rad",
    "leg_1_angle_rad",
    "leg_2_angle_rad",
    "leg_3_angle_rad",
    *[f"obs_{i}" for i in range(13)],
    "action_left",
    "action_right",
    "theta_deg",
    "theta_initial_deg",
    "theta_max_abs_deg",
    "theta_rms_deg",
    "current_rms_A",
    "recovery_time_s",
    "x_final_m",
    "run_time_s",
]

TIMING_FLOAT_COUNT = len(QUANTITIES) * 4
PAYLOAD = struct.Struct(f"<4I{TIMING_FLOAT_COUNT + len(STATE_FLOATS)}f")
EXPECTED_PAYLOAD_SIZE = PAYLOAD.size

POLICY_NAMES = {
    0: "point_mlp",
    1: "range_mlp",
    2: "range_gru",
}

TESTBENCH_STAGE_NAMES = {
    0: "idle",
    1: "settle",
    2: "left_leg",
    3: "neutral_after_left",
    4: "right_leg",
    5: "neutral_after_right",
    6: "checkpoint_before_forward",
    7: "forward",
    8: "hold_after_forward",
    9: "checkpoint_before_reverse",
    10: "reverse",
    11: "hold_after_reverse",
    12: "spin_positive",
    13: "hold_after_positive_spin",
    14: "spin_negative",
    15: "hold_after_negative_spin",
    16: "checkpoint_before_sine_forward",
    17: "sine_legs_forward",
    18: "checkpoint_before_sine_reverse",
    19: "sine_legs_reverse",
    20: "final_hold",
    21: "crossed_asymmetric_hold",
    22: "neutral_after_crossed",
    23: "checkpoint_before_one_leg_forward",
    24: "one_leg_sine_forward",
    25: "checkpoint_before_one_leg_reverse",
    26: "one_leg_sine_reverse",
}


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def csv_columns() -> list[str]:
    columns = [
        "sequence",
        "timestamp_us",
        "time_s",
        "flags",
        "active_policy_id",
        "active_policy_name",
        "testbench_running",
        "testbench_waiting",
        "testbench_recovering",
        "testbench_fall_count",
        "testbench_stage",
        "testbench_stage_name",
        "dropped_frames",
    ]
    for statistic in ("last", "mean", "wcet", "jitter"):
        columns.extend(f"{quantity}_{statistic}_us" for quantity in QUANTITIES)
    columns.extend(STATE_FLOATS)
    return columns


def decode_payload(payload: bytes) -> dict[str, int | float | str]:
    values = PAYLOAD.unpack(payload)
    sequence, timestamp_us, flags, dropped_frames = values[:4]
    floats = values[4:]
    policy_id = (flags >> 8) & 0x03
    testbench_stage = (flags >> 16) & 0xFF
    testbench_fall_count = (flags >> 24) & 0xFF
    row: dict[str, int | float | str] = {
        "sequence": sequence,
        "timestamp_us": timestamp_us,
        "time_s": timestamp_us * 1.0e-6,
        "flags": flags,
        "active_policy_id": policy_id,
        "active_policy_name": POLICY_NAMES.get(policy_id, f"unknown_{policy_id}"),
        "testbench_running": 1 if flags & (1 << 3) else 0,
        "testbench_waiting": 1 if flags & (1 << 4) else 0,
        "testbench_recovering": 1 if flags & (1 << 5) else 0,
        "testbench_fall_count": testbench_fall_count,
        "testbench_stage": testbench_stage,
        "testbench_stage_name": TESTBENCH_STAGE_NAMES.get(
            testbench_stage, f"unknown_{testbench_stage}"
        ),
        "dropped_frames": dropped_frames,
    }

    offset = 0
    for statistic in ("last", "mean", "wcet", "jitter"):
        for quantity in QUANTITIES:
            row[f"{quantity}_{statistic}_us"] = floats[offset]
            offset += 1
    for name in STATE_FLOATS:
        row[name] = floats[offset]
        offset += 1
    return row


def extract_frames(buffer: bytearray):
    while True:
        magic_index = buffer.find(MAGIC)
        if magic_index < 0:
            if buffer[-1:] == MAGIC[:1]:
                del buffer[:-1]
            else:
                buffer.clear()
            return
        if magic_index:
            del buffer[:magic_index]
        if len(buffer) < HEADER.size:
            return

        magic, version, packet_type, payload_size = HEADER.unpack_from(buffer)
        if magic != MAGIC or payload_size > 4096:
            del buffer[0]
            continue
        frame_size = HEADER.size + payload_size + CRC.size
        if len(buffer) < frame_size:
            return

        frame = bytes(buffer[:frame_size])
        del buffer[:frame_size]
        received_crc = CRC.unpack_from(frame, frame_size - CRC.size)[0]
        calculated_crc = crc16_ccitt(frame[2:-2])
        if received_crc != calculated_crc:
            print("warning: discarded frame with invalid CRC", file=sys.stderr)
            continue
        if version != VERSION or packet_type != SAMPLE_TYPE:
            continue
        if payload_size != EXPECTED_PAYLOAD_SIZE:
            raise RuntimeError(
                f"firmware payload is {payload_size} bytes; capture tool expects "
                f"{EXPECTED_PAYLOAD_SIZE}. Update firmware/tool together."
            )
        yield decode_payload(frame[HEADER.size:-CRC.size])


def make_plot(csv_path: Path, plot_path: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("--plot requires matplotlib: python -m pip install matplotlib") from exc

    with csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        return

    def series(name: str) -> list[float]:
        return [float(row[name]) for row in rows]

    t0 = float(rows[0]["time_s"])
    t = [value - t0 for value in series("time_s")]
    rad_to_deg = 180.0 / math.pi

    figure, axes = plt.subplots(3, 2, figsize=(12, 11), constrained_layout=True)
    axes[0, 0].plot(t, [value * rad_to_deg for value in series("pitch_rad")])
    axes[0, 0].set(title="Platform pitch", ylabel="deg")

    axes[0, 1].plot(t, series("wheel_left_current_command_A"), label="left command")
    axes[0, 1].plot(t, series("wheel_right_current_command_A"), label="right command")
    axes[0, 1].plot(t, series("wheel_left_current_measured_A"), "--", label="left measured")
    axes[0, 1].plot(t, series("wheel_right_current_measured_A"), "--", label="right measured")
    axes[0, 1].set(title="Wheel current", ylabel="A")
    axes[0, 1].legend(fontsize="small")

    axes[1, 0].plot(t, series("inference_last_us"), label="policy inference")
    axes[1, 0].plot(t, series("total_last_us"), label="total control")
    axes[1, 0].plot(t, series("release_latency_last_us"), label="release latency")
    axes[1, 0].set(title="Execution timing", ylabel="us")
    axes[1, 0].legend(fontsize="small")

    axes[1, 1].plot(t, series("imu_age_last_us"), label="IMU")
    axes[1, 1].plot(t, series("wheel_age_last_us"), label="wheel encoders")
    axes[1, 1].plot(t, series("leg_age_last_us"), label="leg joints")
    axes[1, 1].set(title="Observation age at inference", ylabel="us")
    axes[1, 1].legend(fontsize="small")

    axes[2, 0].plot(t, series("yaw_rate_radps"), label="body gyro Z (diagnostic)")
    axes[2, 0].plot(t, series("yaw_rate_world_radps"), label="gyro transformed to world Z (policy)")
    axes[2, 0].plot(t, series("yaw_rate_from_quat_radps"), label="fused-yaw derivative")
    axes[2, 0].set(title="Yaw-rate frame diagnostic", ylabel="rad/s")
    axes[2, 0].legend(fontsize="small")

    axes[2, 1].plot(t, series("wheel_left_logical_velocity_radps"), label="left logical")
    axes[2, 1].plot(t, series("wheel_right_logical_velocity_radps"), label="right logical")
    axes[2, 1].set(title="Policy-frame wheel velocity", ylabel="rad/s")
    axes[2, 1].legend(fontsize="small")

    for axis in axes.flat:
        axis.set_xlabel("time (s)")
        axis.grid(True, alpha=0.3)
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(plot_path, dpi=180)
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="serial port, for example COM3 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=921_600)
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 records until Ctrl+C")
    parser.add_argument("--output", type=Path, required=True, help="destination CSV")
    parser.add_argument("--plot", type=Path, help="optional summary PNG written after capture")
    parser.add_argument("--reset", action="store_true", help="send R to reset firmware accumulators")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: python -m pip install pyserial") from exc

    args.output.parent.mkdir(parents=True, exist_ok=True)
    buffer = bytearray()
    rows_written = 0
    last_row: dict[str, int | float | str] | None = None
    started = time.monotonic()

    with serial.Serial(args.port, args.baud, timeout=0.1) as uart, args.output.open(
        "w", newline="", encoding="utf-8"
    ) as csv_stream:
        if args.reset:
            uart.write(b"R")
            uart.flush()
        writer = csv.DictWriter(csv_stream, fieldnames=csv_columns())
        writer.writeheader()
        try:
            while not args.duration or time.monotonic() - started < args.duration:
                data = uart.read(max(uart.in_waiting, 1))
                if not data:
                    continue
                buffer.extend(data)
                for row in extract_frames(buffer):
                    writer.writerow(row)
                    last_row = row
                    rows_written += 1
                    if rows_written % 50 == 0:
                        csv_stream.flush()
        except KeyboardInterrupt:
            pass

    if args.plot:
        make_plot(args.output, args.plot)
    print(f"wrote {rows_written} samples to {args.output}")
    if last_row is not None:
        print("\nTable 9 running statistics (us):")
        print(f"{'quantity':<20} {'mean':>12} {'WCET':>12} {'jitter':>12}")
        for quantity in QUANTITIES:
            print(
                f"{quantity:<20} "
                f"{float(last_row[f'{quantity}_mean_us']):12.3f} "
                f"{float(last_row[f'{quantity}_wcet_us']):12.3f} "
                f"{float(last_row[f'{quantity}_jitter_us']):12.3f}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
