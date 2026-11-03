#!/usr/bin/env python3
"""Live payload mass + position viewer for the balancing robot (laptop side).

The robot runs the balance GRU on the STM32 and streams paper telemetry v3 over
the ST-Link VCP; this script runs the sim-trained payload estimator on every
frame and draws where the object is and how heavy it is.

    python payload_viewer.py calibrate /dev/ttyACM0 --mass 1.0   # once per robot
    python payload_viewer.py live /dev/ttyACM0 --record run.bin
    python payload_viewer.py replay run.bin                     # or a paper_uart_capture CSV

Windows ports are COM3 etc. See README.md for the full procedure.
"""

from __future__ import annotations

import argparse
import collections
import csv
import threading
import time
from pathlib import Path

import numpy as np
from payload_estimator import (FLAG_FALLEN, HERE, LEG_NAMES, POLICY_NAMES, RANGE_GRU_POLICY_ID, LegMap,
                               PayloadEstimator, extract_frames, fit_leg_map, frame_to_raw,
                               location_uncertainty_m, signature)

DEFAULT_CALIB = HERE / "calibration.json"
SHOW_POSITION_ABOVE_KG = 0.05
HISTORY_S = 30.0


# -- frame sources ----------------------------------------------------------------

def serial_rows(port: str, baud: int, record: Path | None, stop: threading.Event):
    import serial

    sink = record.open("wb") if record else None
    buffer = bytearray()
    try:
        with serial.Serial(port, baud, timeout=0.05) as uart:
            uart.reset_input_buffer()
            while not stop.is_set():
                chunk = uart.read(4096)
                if not chunk:
                    continue
                if sink:
                    sink.write(chunk)
                buffer.extend(chunk)
                yield from extract_frames(buffer)
    finally:
        if sink:
            sink.close()


def file_rows(path: Path, speed: float, stop: threading.Event):
    """Replay a raw .bin recording or a paper_uart_capture.py CSV, paced by frame time."""
    if path.suffix.lower() == ".csv":
        with path.open(newline="") as f:
            rows = [{k: (float(v) if k not in ("active_policy_name", "testbench_stage_name") else v)
                     for k, v in r.items()} for r in csv.DictReader(f)]
    else:
        rows = list(extract_frames(bytearray(path.read_bytes())))
    t0_wall, t0 = time.monotonic(), None
    for row in rows:
        if stop.is_set():
            return
        if speed > 0:
            t0 = row["time_s"] if t0 is None else t0
            wait = (row["time_s"] - t0) / speed - (time.monotonic() - t0_wall)
            if wait > 0:
                time.sleep(wait)
        yield row


# -- shared state between the reader thread and the GUI ---------------------------

class Runner:
    def __init__(self, est: PayloadEstimator, tare_s: float):
        self.est, self.tare_s = est, tare_s
        self.lock = threading.Lock()
        self.hist = collections.deque(maxlen=int(HISTORY_S / est.dt))
        self.last = None
        self.frames = self.seq_gaps = 0
        self.prev_seq = None
        self.fallen = False
        self.policy_id = None
        self.dropped = 0
        self.msg = ""
        self.done = False
        self.rate = collections.deque(maxlen=200)
        self.leg_torque_dev = np.zeros(4)

    def request_tare(self):
        with self.lock:
            self.est.start_tare(self.tare_s)
            self.msg = f"TARE: keep the deck empty for {self.tare_s:.0f} s"

    def request_reset(self):
        with self.lock:
            self.est.reset_hidden()
            self.msg = "estimator memory reset"

    def feed(self, row: dict):
        raw = frame_to_raw(row)
        with self.lock:
            if self.prev_seq is not None and raw["seq"] != self.prev_seq + 1:
                self.seq_gaps += 1
            self.prev_seq = raw["seq"]
            self.frames += 1
            self.rate.append(time.monotonic())
            self.policy_id, self.dropped = raw["policy_id"], raw["dropped"]
            fallen = bool(raw["flags"] & FLAG_FALLEN)
            if self.fallen and not fallen:
                self.est.reset_hidden()
                self.msg = "back upright: estimator memory reset -- re-tare (t) with an empty deck"
            self.fallen = fallen
            was_taring = self.est.taring
            mass, off = self.est.step(raw)
            if was_taring and not self.est.taring:
                self.msg = "tare done -- place an object"
            x = self.est.features(raw)
            self.leg_torque_dev = x[19:23] - self.est.empty_mean[19:23]
            self.last = (raw["t"], mass, off.copy())
            self.hist.append((raw["t"], mass, off[0], off[1]))

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            r = [t for t in self.rate if now - t < 1.0]
            return dict(last=self.last, hist=list(self.hist), fps=len(r), gaps=self.seq_gaps, msg=self.msg,
                        fallen=self.fallen, policy_id=self.policy_id, dropped=self.dropped,
                        taring=self.est.taring, legs=self.leg_torque_dev.copy())


def reader_thread(runner: Runner, rows, stop: threading.Event):
    try:
        for row in rows:
            runner.feed(row)
            if stop.is_set():
                break
    except Exception as exc:  # surfaced in the GUI status line
        runner.msg = f"reader stopped: {exc!r}"
    runner.done = True


# -- GUI --------------------------------------------------------------------------

def run_gui(runner: Runner, title: str, stop: threading.Event):
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Circle, Rectangle

    matplotlib.rcParams["keymap.home"] = ["h", "home"]  # free 'r' for "reset memory"
    deck = runner.est.ref["deck"]
    hx, hy = deck["size_lateral_x_m"] / 2, deck["size_foreaft_y_m"] / 2

    fig = plt.figure(figsize=(12, 6.5))
    fig.canvas.manager.set_window_title(title)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1])
    ax_deck = fig.add_subplot(gs[:, 0])
    ax_mass = fig.add_subplot(gs[0, 1])
    ax_legs = fig.add_subplot(gs[1, 1])

    # Top-down view, FRONT up, robot's LEFT on screen-left: screen = (-x, -y) in sim.
    ax_deck.add_patch(Rectangle((-hx, -hy), 2 * hx, 2 * hy, fill=False, lw=2))
    ax_deck.plot([0], [0], "+", color="0.6", ms=12)
    for txt, xy in (("FRONT", (0, hy + 0.012)), ("BACK", (0, -hy - 0.018)),
                    ("LEFT", (-hx - 0.012, 0)), ("RIGHT", (hx + 0.012, 0))):
        ax_deck.text(*xy, txt, ha="center", va="center", fontsize=9, color="0.4",
                     rotation=90 if txt in ("LEFT", "RIGHT") else 0)
    ax_deck.set_xlim(-hx - 0.03, hx + 0.03)
    ax_deck.set_ylim(-hy - 0.03, hy + 0.03)
    ax_deck.set_aspect("equal")
    ax_deck.set_xticks([])
    ax_deck.set_yticks([])
    dot = Circle((0, 0), 0.005, color="tab:blue", zorder=3)  # dashed ring = typical sim error
    ring = Circle((0, 0), 0.01, fill=False, ls="--", color="tab:blue", alpha=0.6, zorder=2)
    ax_deck.add_patch(ring)
    ax_deck.add_patch(dot)
    mass_txt = ax_deck.text(0.5, 1.02, "", transform=ax_deck.transAxes, ha="center", fontsize=22, weight="bold")
    pos_txt = ax_deck.text(0.5, -0.06, "", transform=ax_deck.transAxes, ha="center", fontsize=10)
    status = fig.text(0.01, 0.01, "", fontsize=9, family="monospace")

    (mass_line,) = ax_mass.plot([], [], lw=1.5)
    ax_mass.set_ylabel("mass estimate (kg)")
    ax_mass.set_xlabel("time (s)")
    ax_mass.grid(alpha=0.3)

    bars = ax_legs.bar(LEG_NAMES, [0, 0, 0, 0], color="tab:gray")
    ref_1kg = signature(runner.est.ref, 1.0, 0.0, 0.0)[1]
    ax_legs.plot(LEG_NAMES, ref_1kg, "k_", ms=25, mew=2, label="sim: 1 kg centred")
    ax_legs.axhline(0, color="k", lw=0.5)
    ax_legs.set_ylim(-0.7, 0.7)
    ax_legs.set_ylabel("leg torque vs empty (Nm, sim frame)")
    ax_legs.legend(loc="upper right", fontsize=8)
    fig.text(0.99, 0.01, "t = tare (empty deck)   r = reset memory   q = quit", ha="right", fontsize=9)
    fig.tight_layout(rect=(0, 0.04, 1, 1))

    def on_key(event):
        if event.key == "t":
            runner.request_tare()
        elif event.key == "r":
            runner.request_reset()

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("close_event", lambda _e: stop.set())

    def update(_frame):
        s = runner.snapshot()
        if s["last"] is not None:
            _, mass, off = s["last"]
            show = mass >= SHOW_POSITION_ABOVE_KG and not s["taring"]
            sx, sy = -off[0], -off[1]
            dot.set_center((sx, sy))
            ring.set_center((sx, sy))
            ring.set_radius(location_uncertainty_m(mass))
            dot.set_visible(show)
            ring.set_visible(show)
            mass_txt.set_text("taring..." if s["taring"] else f"{mass:.3f} kg  ({mass * 1000:.0f} g)")
            if show:
                lat = f"{abs(off[0]) * 100:.1f} cm {'left' if off[0] > 0 else 'right'}"
                fa = f"{abs(off[1]) * 100:.1f} cm {'front' if off[1] < 0 else 'back'}"
                weak = "  (position unreliable below ~0.25 kg)" if mass < 0.25 else ""
                pos_txt.set_text(f"{lat}, {fa} of deck centre{weak}")
            else:
                pos_txt.set_text("deck empty" if not s["taring"] else "")
            h = np.array(s["hist"])
            if len(h):
                mass_line.set_data(h[:, 0] - h[-1, 0], h[:, 1])
                ax_mass.set_xlim(-HISTORY_S, 0)
                ax_mass.set_ylim(-0.05, max(0.5, h[:, 1].max() * 1.15))
            for b, v in zip(bars, s["legs"]):
                b.set_height(v)
        pol = POLICY_NAMES.get(s["policy_id"], s["policy_id"])
        warn = []
        if s["policy_id"] is not None and s["policy_id"] != RANGE_GRU_POLICY_ID:
            warn.append(f"WARNING policy={pol}, estimator was trained with range_gru")
        if s["fallen"]:
            warn.append("FALLEN")
        if not runner.est.leg_map.calibrated:
            warn.append("UNCALIBRATED leg map")
        if runner.done:
            warn.append("stream ended")
        status.set_text(f"{s['fps']:3d} frames/s  seq gaps {s['gaps']}  fw dropped {s['dropped']}  policy {pol}"
                        f"  | {s['msg']}  {'  '.join(warn)}")
        return dot, ring, mass_txt, pos_txt, status, mass_line, *bars

    _anim = FuncAnimation(fig, update, interval=70, cache_frame_data=False)  # noqa: F841 (keep alive)
    plt.show()
    stop.set()


# -- commands ---------------------------------------------------------------------

def make_runner(args) -> Runner:
    leg_map = LegMap.load(args.calib)
    if args.use_gain:
        leg_map.use_gain = True
    print(f"[payload] leg map: perm {leg_map.perm} sign {leg_map.sign} "
          f"({'calibrated' if leg_map.calibrated else 'NOT calibrated: ' + leg_map.note})"
          f"{', gains applied' if leg_map.use_gain else ''}", flush=True)
    runner = Runner(PayloadEstimator(leg_map), args.tare_s)
    if not args.no_tare:
        runner.request_tare()
    return runner


def cmd_live(args):
    stop = threading.Event()
    runner = make_runner(args)
    rows = serial_rows(args.port, args.baud, args.record, stop)
    threading.Thread(target=reader_thread, args=(runner, rows, stop), daemon=True).start()
    run_gui(runner, f"payload estimator - {args.port}", stop)


def cmd_replay(args):
    stop = threading.Event()
    runner = make_runner(args)
    rows = file_rows(args.file, 0.0 if args.no_gui else args.speed, stop)
    if args.no_gui:
        out = []
        for row in rows:
            runner.feed(row)
            t, mass, off = runner.last
            out.append((t, mass, off[0], off[1]))
        if args.out:
            np.savetxt(args.out, np.array(out), delimiter=",", header="time_s,mass_kg,offset_x_m,offset_y_m",
                       comments="")
        tail = np.array(out[-int(2.0 / runner.est.dt):])
        print(f"[payload] {runner.frames} frames, seq gaps {runner.seq_gaps}; last 2 s: mass "
              f"{tail[:, 1].mean():.3f} kg, x {tail[:, 2].mean() * 100:+.1f} cm, y {tail[:, 3].mean() * 100:+.1f} cm",
              flush=True)
        return
    threading.Thread(target=reader_thread, args=(runner, rows, stop), daemon=True).start()
    run_gui(runner, f"payload estimator - replay {args.file.name}", stop)


def cmd_calibrate(args):
    """Interactive: empty deck, then the known mass at centre/front/back/left/right."""
    stop = threading.Event()
    buf: collections.deque = collections.deque(maxlen=4000)
    state = {"fallen": False, "policy": None}

    def pump():
        for row in serial_rows(args.port, args.baud, args.record, stop):
            raw = frame_to_raw(row)
            state["fallen"] = bool(raw["flags"] & FLAG_FALLEN)
            state["policy"] = raw["policy_id"]
            buf.append((time.monotonic(), raw))

    threading.Thread(target=pump, daemon=True).start()
    ref = PayloadEstimator(LegMap()).ref
    f, s = args.foreaft_m, args.lateral_m
    steps = [("EMPTY deck (remove everything)", None), ("CENTRE of the deck", (0.0, 0.0)),
             (f"FRONT: {f * 100:.0f} cm forward of centre", (0.0, -f)),
             (f"BACK: {f * 100:.0f} cm behind centre", (0.0, +f)),
             (f"LEFT: {s * 100:.0f} cm to the robot's left", (+s, 0.0)),
             (f"RIGHT: {s * 100:.0f} cm to the robot's right", (-s, 0.0))]
    print(f"\nCalibration with a {args.mass:.3f} kg object. Robot balancing in place, no joystick input.\n"
          "FRONT = the side the robot drives towards for a positive velocity command;\n"
          "LEFT/RIGHT as seen from behind the robot, facing FRONT.\n", flush=True)
    means = []
    for label, xy in steps:
        input(f"-> {label}. Press Enter when placed and the robot is steady... ")
        time.sleep(args.settle_s)
        t0 = time.monotonic()
        time.sleep(args.window_s)
        win = [r for (t, r) in list(buf) if t >= t0]
        if len(win) < 0.5 * args.window_s / 0.015:
            raise SystemExit(f"only {len(win)} frames in {args.window_s} s -- is telemetry streaming (v3, 921600)?")
        if state["fallen"]:
            raise SystemExit("robot reports fallen; restart calibration")
        a = np.mean([r["angle"] - r["target"] for r in win], axis=0)
        tq = np.mean([r["torque"] for r in win], axis=0)
        print(f"   {len(win)} frames | angle dev (mrad) {np.round(a * 1000, 2)} | torque (Nm) {np.round(tq, 3)}")
        means.append((xy, a, tq))
    stop.set()
    if state["policy"] != RANGE_GRU_POLICY_ID:
        print(f"WARNING: firmware reports policy {POLICY_NAMES.get(state['policy'], state['policy'])}, not range_gru")

    (_, a0, t0_), loaded = means[0], means[1:]
    placements = [(args.mass, xy[0], xy[1]) for xy, _, _ in loaded]
    d_a = np.stack([a - a0 for _, a, _ in loaded])
    d_t = np.stack([t - t0_ for _, _, t in loaded])
    best, scored = fit_leg_map(ref, d_a, d_t, placements)
    runner_up = scored[1][0] if len(scored) > 1 else float("nan")
    print("\nHardware change from empty (index order 0..3):")
    for (label, _), da, dt_ in zip(steps[1:], d_a, d_t):
        print(f"   {label.split(':')[0]:6s} torque {np.round(dt_, 3)} Nm   angle {np.round(da * 1000, 2)} mrad")
    stiff = -d_t.sum(0) / np.where(np.abs(d_a.sum(0)) > 1e-6, d_a.sum(0), np.nan)
    print(f"\nEffective leg stiffness -dTorque/dAngle per CyberGear (sim/firmware kp = 30): {np.round(stiff, 1)}")
    print(f"\nBest map: sim {LEG_NAMES} <- hardware index {best.perm}, sign {best.sign}")
    print(f"   residual {scored[0][0]:.3f} (runner-up {runner_up:.3f}; want best << runner-up and best < ~0.3)")
    print(f"   hardware torque reads {1 / best.torque_gain:.2f}x sim, angle {1 / best.angle_gain:.2f}x sim "
          "(1.0 = matches the sim)")
    exp = np.stack([signature(ref, *p)[1] for p in placements])
    mapped = np.array(best.sign) * d_t[:, best.perm]
    print("   per placement, sim-expected vs measured (mapped) torque, Nm:")
    for (label, _), e, m in zip(steps[1:], exp, mapped):
        print(f"     {label.split(':')[0]:6s} sim {np.round(e, 3)}   hw {np.round(m, 3)}")
    best.note = f"calibrated {time.strftime('%Y-%m-%d %H:%M')} with {args.mass} kg, residual {scored[0][0]:.3f}"
    best.save(args.out)
    print(f"\nSaved {args.out}. Gains are recorded but only applied with --use-gain.", flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp):
        sp.add_argument("--calib", type=Path, default=DEFAULT_CALIB, help="leg map from `calibrate`")
        sp.add_argument("--use-gain", action="store_true", help="apply the calibrated hardware->sim gains")
        sp.add_argument("--tare-s", type=float, default=3.0, help="tare duration; deck must be empty")
        sp.add_argument("--no-tare", action="store_true", help="skip the automatic tare at start")

    sp = sub.add_parser("live", help="read the robot over serial and show the estimate")
    sp.add_argument("port")
    sp.add_argument("--baud", type=int, default=921600)
    sp.add_argument("--record", type=Path, help="also save the raw byte stream (replayable)")
    common(sp)
    sp.set_defaults(func=cmd_live)

    sp = sub.add_parser("replay", help="replay a .bin recording or a paper_uart_capture CSV")
    sp.add_argument("file", type=Path)
    sp.add_argument("--speed", type=float, default=1.0)
    sp.add_argument("--no-gui", action="store_true", help="run as fast as possible and print a summary")
    sp.add_argument("--out", type=Path, help="with --no-gui: write the estimate trace as CSV")
    common(sp)
    sp.set_defaults(func=cmd_replay)

    sp = sub.add_parser("calibrate", help="fit the CyberGear -> sim leg mapping with a known mass")
    sp.add_argument("port")
    sp.add_argument("--mass", type=float, required=True, help="known object mass in kg (1.0 recommended)")
    sp.add_argument("--baud", type=int, default=921600)
    sp.add_argument("--foreaft-m", type=float, default=0.08)
    sp.add_argument("--lateral-m", type=float, default=0.08)
    sp.add_argument("--settle-s", type=float, default=2.0)
    sp.add_argument("--window-s", type=float, default=4.0)
    sp.add_argument("--out", type=Path, default=DEFAULT_CALIB)
    sp.add_argument("--record", type=Path)
    sp.set_defaults(func=cmd_calibrate)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
