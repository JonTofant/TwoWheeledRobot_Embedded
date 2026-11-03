#!/usr/bin/env python3
"""One-window GUI for the payload estimator: connect, calibrate, then watch mass + position live.

    python payload_gui.py                  # pick the ST-Link port in the window
    python payload_gui.py --port COM7      # preselect a port
    python payload_gui.py --replay recordings/20261103_140000.bin

Same runtime as payload_viewer.py (Runner, PayloadEstimator, fit_leg_map); this only adds the
window. Every serial session is recorded to recordings/<time>.bin (the raw byte stream,
replayable here or with `payload_viewer.py replay`) plus <time>.events.jsonl, which marks the
telemetry sequence number of every tare, calibration capture and calibration result.

Calibration windows are counted in telemetry frames (sequence numbers), not wall time, so the
wizard behaves the same on a replayed recording as on the robot.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import matplotlib
import numpy as np

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402
from payload_estimator import (FLAG_FALLEN, HERE, LEG_NAMES, POLICY_NAMES, RANGE_GRU_POLICY_ID,  # noqa: E402
                               LegMap, PayloadEstimator, extract_frames, fit_leg_map, frame_to_raw,
                               location_uncertainty_m, signature)
from payload_viewer import DEFAULT_CALIB, HISTORY_S, SHOW_POSITION_ABOVE_KG, Runner, file_rows  # noqa: E402

RECORDINGS = HERE / "recordings"
BAUD = 921600
STLINK_VID = 0x0483
TARE_S = 3.0
UI_PERIOD_MS = 100
COLORS = {"good": "#1a7f37", "warn": "#b35900", "bad": "#cf222e", "info": "#57606a"}


# -- calibration wizard (no GUI code, runs on whatever frames arrive) ---------------------

class CalibrationSession:
    """Empty deck, then the known mass at centre / front / back / left / right.

    Each step: the user clicks Capture, the next `settle` frames are skipped, the following
    `window` frames are averaged. A fall or a gappy link during a step rejects that step.
    """

    def __init__(self, mass: float, offset_m: float = 0.08, settle_s: float = 2.0,
                 window_s: float = 4.0, dt: float = 0.015):
        o = offset_m * 100
        self.mass = mass
        self.steps = [
            ("Empty deck", "Remove everything from the deck.", None),
            ("Centre", f"Put the {mass:g} kg object at the CENTRE of the deck.", (0.0, 0.0)),
            ("Front", f"Move it {o:.0f} cm to the FRONT of centre (the way a positive velocity "
                      "command drives).", (0.0, -offset_m)),
            ("Back", f"Move it {o:.0f} cm to the BACK of centre.", (0.0, +offset_m)),
            ("Left", f"Move it {o:.0f} cm to the robot's LEFT (as seen from behind, facing front).",
             (+offset_m, 0.0)),
            ("Right", f"Move it {o:.0f} cm to the robot's RIGHT (as seen from behind, facing front).",
             (-offset_m, 0.0)),
        ]
        self.settle_n = max(1, int(round(settle_s / dt)))
        self.window_n = max(1, int(round(window_s / dt)))
        self.idx = 0
        self.phase = "wait"  # wait -> settle -> capture -> wait ... -> done
        self.means: list[tuple] = []  # per finished step: (xy, angle_dev_mean[4], torque_mean[4], n)
        self.last_seq: int | None = None
        self.error = ""
        self._win: list[tuple[np.ndarray, np.ndarray]] = []
        self._fell = False
        self._click_seq = self._start_seq = self._end_seq = 0

    @property
    def current(self):
        return self.steps[self.idx] if self.idx < len(self.steps) else None

    def capture(self) -> bool:
        if self.phase != "wait" or self.last_seq is None or self.current is None:
            return False
        self._click_seq = self.last_seq
        self._start_seq = self.last_seq + 1 + self.settle_n
        self._end_seq = self._start_seq + self.window_n
        self._win, self._fell, self.error = [], False, ""
        self.phase = "settle"
        return True

    def redo_previous(self) -> None:
        if self.phase in ("wait", "done") and self.means:
            self.means.pop()
            self.idx -= 1
            self.phase, self.error = "wait", ""

    def feed(self, raw: dict) -> None:
        self.last_seq = raw["seq"]
        if self.phase not in ("settle", "capture"):
            return
        if raw["flags"] & FLAG_FALLEN:
            self._fell = True
            self._finish_step()
            return
        if raw["seq"] >= self._start_seq:
            self.phase = "capture"
            if raw["seq"] < self._end_seq:
                self._win.append((raw["angle"] - raw["target"], raw["torque"]))
        if raw["seq"] >= self._end_seq - 1:
            self._finish_step()

    def _finish_step(self) -> None:
        if self._fell:
            self.phase = "wait"
            self.error = "The robot fell during this step. Stand it up, let it balance, then capture again."
            return
        if len(self._win) < 0.5 * self.window_n:
            self.phase = "wait"
            self.error = (f"Only {len(self._win)} of {self.window_n} frames arrived (link dropping frames). "
                          "Capture again.")
            return
        a = np.mean([w[0] for w in self._win], axis=0)
        t = np.mean([w[1] for w in self._win], axis=0)
        self.means.append((self.steps[self.idx][2], a, t, len(self._win)))
        self.idx += 1
        self.phase = "done" if self.idx >= len(self.steps) else "wait"

    def progress(self) -> float:
        if self.phase not in ("settle", "capture") or self.last_seq is None:
            return 0.0
        return float(np.clip((self.last_seq - self._click_seq) / (self.settle_n + self.window_n), 0, 1))


def analyze_calibration(ref: dict, mass: float, means: list[tuple]) -> dict:
    """Same fit and numbers as `payload_viewer.py calibrate`, returned instead of printed."""
    (_, a0, t0, _), loaded = means[0], means[1:]
    placements = [(mass, xy[0], xy[1]) for xy, _, _, _ in loaded]
    d_a = np.stack([a - a0 for _, a, _, _ in loaded])
    d_t = np.stack([t - t0 for _, _, t, _ in loaded])
    best, scored = fit_leg_map(ref, d_a, d_t, placements)
    den = d_a.sum(0)
    stiff = -d_t.sum(0) / np.where(np.abs(den) > 1e-6, den, np.nan)
    exp = np.stack([signature(ref, *p)[1] for p in placements])
    mapped = np.array(best.sign) * d_t[:, best.perm]
    return dict(best=best, residual=scored[0][0], runner_up=scored[1][0] if len(scored) > 1 else float("nan"),
                d_a=d_a, d_t=d_t, stiff=stiff, exp=exp, mapped=mapped,
                torque_x_sim=1.0 / best.torque_gain, angle_x_sim=1.0 / best.angle_gain)


def judge_calibration(r: dict) -> tuple[list[tuple[str, str, str, str]], str, str]:
    """Rows of (check, value, level, meaning) plus an overall (level, conclusion)."""
    res, ru = r["residual"], r["runner_up"]
    sep = res / ru if ru > 0 else float("inf")
    tx = r["torque_x_sim"]
    stiff = r["stiff"][np.isfinite(r["stiff"])]
    k = float(np.median(stiff)) if len(stiff) else float("nan")

    def lvl(good, warn):
        return "good" if good else "warn" if warn else "bad"

    rows = [
        ("Fit residual", f"{res:.3f}", lvl(res < 0.3, res < 0.5),
         "want < ~0.3 (sim: 0.05)"),
        ("Runner-up mapping", f"{ru:.3f}  (best is {sep:.0%} of it)", lvl(sep < 0.6, sep < 0.85),
         "best should be well below the runner-up (sim: 0.28)"),
        ("Torque vs sim", f"{tx:.2f}x", lvl(0.75 <= tx <= 1.33, 0.5 <= tx <= 2.0),
         "want ~1.0x; far off -> tick 'Use calibrated gains'"),
        ("Leg stiffness", f"{k:.1f} Nm/rad  (per CG: {', '.join(f'{v:.1f}' for v in r['stiff'])})",
         lvl(20 <= k <= 40, 10 <= k <= 60), "want ~30 (firmware kp); much lower -> torque misses friction"),
    ]
    levels = [row[2] for row in rows]
    if rows[0][2] == "bad":
        overall = ("bad", "High residual for the best mapping, so for every mapping: the CyberGear torque "
                          "feedback does not track the load (gearbox stiction / hysteresis). Live estimates "
                          "will not be trustworthy. Keep this recording for debugging.")
    elif "bad" in levels or "warn" in levels:
        overall = ("warn", "A mapping was found but some checks are marginal (see orange/red rows). "
                           "You can apply it and test with known masses, but treat results with care.")
    else:
        overall = ("good", "Calibration looks good: the leg torque feedback follows the load like the sim.")
    return rows, overall[0], overall[1]


def map_text(m: LegMap) -> str:
    return ", ".join(f"{n}<-CG{p}{'+' if s > 0 else '-'}" for n, p, s in zip(LEG_NAMES, m.perm, m.sign))


# -- frame sources ------------------------------------------------------------------------

class GuiRunner(Runner):
    """Runner plus what the window needs: link health and the calibration wizard."""

    def __init__(self, est: PayloadEstimator, tare_s: float):
        super().__init__(est, tare_s)
        self.calib: CalibrationSession | None = None
        self.bytes = 0
        self.last_frame_wall: float | None = None
        self.started_wall = time.monotonic()
        self.error = ""

    def feed(self, row: dict):
        super().feed(row)
        raw = frame_to_raw(row)
        with self.lock:
            self.last_frame_wall = time.monotonic()
            if self.calib is not None:
                self.calib.feed(raw)


def serial_worker(runner: GuiRunner, port: str, sink, stop: threading.Event):
    import serial

    try:
        with serial.Serial(port, BAUD, timeout=0.05) as uart:
            uart.reset_input_buffer()
            buf = bytearray()
            while not stop.is_set():
                chunk = uart.read(4096)
                if not chunk:
                    continue
                runner.bytes += len(chunk)
                if sink:
                    sink.write(chunk)
                buf.extend(chunk)
                for row in extract_frames(buf):
                    runner.feed(row)
    except Exception as exc:  # port vanished, busy, wrong payload size, ...
        runner.error = f"{type(exc).__name__}: {exc}"
    runner.done = True


def replay_worker(runner: GuiRunner, path: Path, stop: threading.Event):
    try:
        for row in file_rows(path, 1.0, stop):
            runner.bytes += 452
            runner.feed(row)
    except Exception as exc:
        runner.error = f"{type(exc).__name__}: {exc}"
    runner.done = True


def list_ports() -> list[tuple[str, str, bool]]:
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        return []
    out = []
    for p in sorted(lp.comports(), key=lambda p: p.device):
        stlink = p.vid == STLINK_VID or "stlink" in (p.description or "").lower().replace("-", "")
        out.append((p.device, p.description or "", stlink))
    return out


# -- window -------------------------------------------------------------------------------

class App:
    def __init__(self, root: tk.Tk, args):
        self.root = root
        self.est = PayloadEstimator(LegMap.load(args.calib))
        self.calib_path = Path(args.calib)
        self.ref = self.est.ref
        self.runner: GuiRunner | None = None
        self.stop: threading.Event | None = None
        self.thread: threading.Thread | None = None
        self.sink = self.events = None
        self.session_stem: Path | None = None
        self.source_name = ""
        self.last_msg = ""
        self.calib_result: dict | None = None
        self._ports: list[tuple[str, str, bool]] = []
        self._ports_t = 0.0

        root.title("Payload estimator")
        root.geometry("1450x880")
        root.minsize(1100, 700)
        style = ttk.Style(root)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        style.configure("Big.TLabel", font=("Segoe UI", 30, "bold"))
        style.configure("Pos.TLabel", font=("Segoe UI", 13))
        style.configure("Step.TLabel", font=("Segoe UI", 12, "bold"))
        style.configure("H.TLabel", font=("Segoe UI", 10, "bold"))
        style.configure("Go.TButton", font=("Segoe UI", 11, "bold"))

        self._build_toolbar()
        body = ttk.Frame(root)
        body.pack(fill="both", expand=True, padx=8, pady=(0, 4))
        self._build_sidebar(body)
        self._build_plots(body)
        self.status = ttk.Label(root, text="", font=("Consolas", 9), anchor="w")
        self.status.pack(fill="x", padx=8, pady=(0, 6))

        root.bind("<Key>", self._on_key)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self._refresh_ports(preselect=args.port)
        self.log(f"Leg map: {map_text(self.est.leg_map)} "
                 f"({'calibrated: ' + self.est.leg_map.note if self.est.leg_map.calibrated else 'NOT calibrated'})")
        if args.replay:
            root.after(300, lambda: self.start_replay(Path(args.replay)))
        root.after(UI_PERIOD_MS, self.tick)

    # -- layout -----------------------------------------------------------------------------

    def _build_toolbar(self):
        bar = ttk.Frame(self.root)
        bar.pack(fill="x", padx=8, pady=6)
        ttk.Label(bar, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=52, state="readonly")
        self.port_box.pack(side="left", padx=4)
        ttk.Button(bar, text="Refresh", command=lambda: self._refresh_ports(force=True)).pack(side="left")
        self.connect_btn = ttk.Button(bar, text="Connect", style="Go.TButton", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=8)
        self.record_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="Record session (.bin)", variable=self.record_var).pack(side="left", padx=4)
        ttk.Button(bar, text="Replay recording...", command=self.pick_replay).pack(side="left", padx=8)
        self.health = tk.Label(bar, text="Not connected", font=("Segoe UI", 10, "bold"), fg=COLORS["info"])
        self.health.pack(side="right")

    def _build_sidebar(self, body):
        side = ttk.Frame(body, width=450)
        side.pack(side="right", fill="y", padx=(8, 0))
        side.pack_propagate(False)

        chk = ttk.LabelFrame(side, text="Checks")
        chk.pack(fill="x", pady=(0, 6))
        self.chk_link = self._check_row(chk, "Telemetry v3")
        self.chk_policy = self._check_row(chk, "Firmware policy")
        self.chk_robot = self._check_row(chk, "Robot")
        self.chk_calib = self._check_row(chk, "Leg map")

        nb = ttk.Notebook(side)
        nb.pack(fill="both", expand=True)
        self.nb = nb
        self._build_calib_tab(nb)
        self._build_live_tab(nb)
        logf = ttk.Frame(nb)
        nb.add(logf, text="Log")
        self.log_text = tk.Text(logf, wrap="word", font=("Consolas", 9), height=10)
        self.log_text.pack(fill="both", expand=True)

    def _check_row(self, parent, name):
        row = ttk.Frame(parent)
        row.pack(fill="x", padx=6, pady=1)
        ttk.Label(row, text=name, width=15).pack(side="left")
        val = tk.Label(row, text="-", anchor="w", font=("Segoe UI", 9, "bold"), fg=COLORS["info"])
        val.pack(side="left", fill="x", expand=True)
        return val

    def _build_calib_tab(self, nb):
        f = ttk.Frame(nb, padding=6)
        nb.add(f, text="1. Calibrate")
        top = ttk.Frame(f)
        top.pack(fill="x")
        ttk.Label(top, text="Known mass (kg)").grid(row=0, column=0, sticky="w")
        self.mass_var = tk.StringVar(value="1.0")
        ttk.Entry(top, textvariable=self.mass_var, width=7).grid(row=0, column=1, padx=4)
        ttk.Label(top, text="Offset (cm)").grid(row=0, column=2, sticky="w", padx=(10, 0))
        self.offset_var = tk.StringVar(value="8")
        ttk.Entry(top, textvariable=self.offset_var, width=5).grid(row=0, column=3, padx=4)
        self.calib_start_btn = ttk.Button(f, text="Start calibration", command=self.calib_start)
        self.calib_start_btn.pack(fill="x", pady=6)
        ttk.Label(f, text="Robot balancing in place, no joystick input. FRONT = direction of a positive "
                          "velocity command; LEFT/RIGHT as seen from behind.", wraplength=420,
                  foreground=COLORS["info"]).pack(fill="x")

        self.step_labels = []
        steps = ttk.Frame(f)
        steps.pack(fill="x", pady=4)
        for i, name in enumerate(["Empty deck", "Centre", "Front", "Back", "Left", "Right"]):
            lab = tk.Label(steps, text=f"o  {i + 1}. {name}", anchor="w", font=("Segoe UI", 10), fg=COLORS["info"])
            lab.grid(row=i // 3, column=i % 3, sticky="w", padx=4)
            self.step_labels.append(lab)

        self.instr = ttk.Label(f, text="Connect, let the robot balance, then press Start calibration.",
                               style="Step.TLabel", wraplength=420)
        self.instr.pack(fill="x", pady=(6, 2))
        self.calib_err = tk.Label(f, text="", fg=COLORS["bad"], wraplength=420, justify="left", anchor="w")
        self.calib_err.pack(fill="x")
        self.calib_prog = ttk.Progressbar(f, maximum=1.0)
        self.calib_prog.pack(fill="x", pady=4)
        btns = ttk.Frame(f)
        btns.pack(fill="x")
        self.capture_btn = ttk.Button(btns, text="Placed & steady: Capture", style="Go.TButton",
                                      command=self.calib_capture, state="disabled")
        self.capture_btn.pack(side="left", fill="x", expand=True)
        self.redo_btn = ttk.Button(btns, text="Redo previous", command=self.calib_redo, state="disabled")
        self.redo_btn.pack(side="left", padx=4)
        self.cancel_btn = ttk.Button(btns, text="Cancel", command=self.calib_cancel, state="disabled")
        self.cancel_btn.pack(side="left")

        self.apply_btn = ttk.Button(f, text="Remove the object, then: Apply, save & tare", style="Go.TButton",
                                    command=self.calib_apply, state="disabled")
        self.apply_btn.pack(side="bottom", fill="x", pady=(4, 0))
        ttk.Label(f, text="Result", style="H.TLabel").pack(anchor="w", pady=(8, 0))
        res = ttk.Frame(f)
        res.pack(fill="both", expand=True)
        scroll = ttk.Scrollbar(res, orient="vertical")
        scroll.pack(side="right", fill="y")
        self.result_text = tk.Text(res, wrap="word", height=6, font=("Segoe UI", 9), relief="flat",
                                   background=self.root.cget("background"), yscrollcommand=scroll.set)
        scroll.config(command=self.result_text.yview)
        for k, c in COLORS.items():
            self.result_text.tag_configure(k, foreground=c)
        self.result_text.tag_configure("b", font=("Segoe UI", 9, "bold"))
        self.result_text.pack(side="left", fill="both", expand=True)

    def _build_live_tab(self, nb):
        f = ttk.Frame(nb, padding=6)
        nb.add(f, text="2. Live")
        ttk.Label(f, text="Start with an EMPTY deck: connecting runs a 3 s tare. After a fall the "
                          "estimator resets; empty the deck and tare again.", wraplength=420,
                  foreground=COLORS["info"]).pack(fill="x")
        ttk.Button(f, text="Tare now (t)  - deck must be empty", style="Go.TButton",
                   command=self.tare).pack(fill="x", pady=(8, 4))
        ttk.Button(f, text="Reset estimator memory (r)", command=self.reset_memory).pack(fill="x")
        self.gain_var = tk.BooleanVar(value=self.est.leg_map.use_gain)
        ttk.Checkbutton(f, text="Use calibrated gains (re-tares)", variable=self.gain_var,
                        command=self.toggle_gain).pack(anchor="w", pady=8)
        ttk.Label(f, text="Leg map", style="H.TLabel").pack(anchor="w")
        self.map_label = ttk.Label(f, text="", wraplength=420)
        self.map_label.pack(fill="x")
        ttk.Button(f, text="Load calibration file...", command=self.load_calib).pack(anchor="w", pady=4)
        ttk.Label(f, text="Recording", style="H.TLabel").pack(anchor="w", pady=(10, 0))
        self.rec_label = ttk.Label(f, text="-", wraplength=420)
        self.rec_label.pack(fill="x")
        ttk.Label(f, text="Sim accuracy: ~14 g mass error anywhere on the deck; position ~1 cm at >= 0.5 kg, "
                          "unreliable below ~0.25 kg. Two objects read as one at their common centre.",
                  wraplength=420, foreground=COLORS["info"]).pack(fill="x", pady=(12, 0))

    def _build_plots(self, body):
        left = ttk.Frame(body)
        left.pack(side="left", fill="both", expand=True)
        head = ttk.Frame(left)
        head.pack(fill="x")
        self.mass_label = ttk.Label(head, text="-", style="Big.TLabel")
        self.mass_label.pack(side="left")
        self.pos_label = ttk.Label(head, text="", style="Pos.TLabel")
        self.pos_label.pack(side="left", padx=20, pady=(12, 0))

        deck = self.ref["deck"]
        hx, hy = deck["size_lateral_x_m"] / 2, deck["size_foreaft_y_m"] / 2
        fig = Figure(figsize=(10, 6.5), dpi=100)
        gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1])
        ax = fig.add_subplot(gs[:, 0])
        self.ax_mass = fig.add_subplot(gs[0, 1])
        ax_legs = fig.add_subplot(gs[1, 1])

        # Top-down, FRONT up, robot's LEFT on screen-left: screen = (-x, -y) in the sim deck frame.
        ax.add_patch(Rectangle((-hx, -hy), 2 * hx, 2 * hy, fill=False, lw=2))
        ax.plot([0], [0], "+", color="0.6", ms=12)
        for txt, xy in (("FRONT", (0, hy + 0.012)), ("BACK", (0, -hy - 0.016)),
                        ("LEFT", (-hx - 0.012, 0)), ("RIGHT", (hx + 0.012, 0))):
            ax.text(*xy, txt, ha="center", va="center", fontsize=10, color="0.35", weight="bold",
                    rotation=90 if txt in ("LEFT", "RIGHT") else 0)
        for c in (-0.08, 0.08):  # calibration placements, for orientation
            ax.plot([0, c], [c, 0], ".", color="0.85", ms=6)
        ax.set_xlim(-hx - 0.03, hx + 0.03)
        ax.set_ylim(-hy - 0.03, hy + 0.03)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title("deck, seen from above", fontsize=9, color="0.4")
        self.ring = Circle((0, 0), 0.01, fill=False, ls="--", color="tab:blue", alpha=0.6, zorder=2)
        self.dot = Circle((0, 0), 0.007, color="tab:blue", zorder=3)
        ax.add_patch(self.ring)
        ax.add_patch(self.dot)
        (self.target,) = ax.plot([], [], "x", color="tab:green", ms=22, mew=4, zorder=4)
        self.target_txt = ax.text(0, 0, "", color="tab:green", fontsize=10, weight="bold", ha="center",
                                  va="top", zorder=4)
        self.deck_note = ax.text(0, 0, "", ha="center", va="center", fontsize=15, color="0.5")

        (self.mass_line,) = self.ax_mass.plot([], [], lw=1.5)
        self.ax_mass.set_ylabel("mass estimate (kg)")
        self.ax_mass.set_xlabel("time (s)")
        self.ax_mass.set_xlim(-HISTORY_S, 0)
        self.ax_mass.grid(alpha=0.3)

        self.bars = ax_legs.bar(LEG_NAMES, [0, 0, 0, 0], color="tab:gray")
        ax_legs.plot(LEG_NAMES, signature(self.ref, 1.0, 0.0, 0.0)[1], "k_", ms=25, mew=2, label="sim: 1 kg centred")
        ax_legs.axhline(0, color="k", lw=0.5)
        ax_legs.set_ylim(-0.7, 0.7)
        ax_legs.set_ylabel("leg torque vs empty (Nm, sim frame)")
        ax_legs.legend(loc="upper right", fontsize=8)
        fig.tight_layout()
        self.fig = fig
        self.canvas = FigureCanvasTkAgg(fig, master=left)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # -- helpers ----------------------------------------------------------------------------

    def log(self, text: str):
        stamp = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"{stamp}  {text}\n")
        self.log_text.see("end")

    def event(self, kind: str, **kw):
        seq = self.runner.prev_seq if self.runner else None
        if self.events:
            self.events.write(json.dumps({"wall": time.strftime("%Y-%m-%dT%H:%M:%S"), "seq": seq,
                                          "kind": kind, **kw}, default=float) + "\n")
            self.events.flush()

    def _refresh_ports(self, preselect: str | None = None, force: bool = False):
        ports = list_ports()
        if ports == self._ports and not force and preselect is None:
            return
        self._ports = ports
        labels = [f"{dev}  -  {desc}{'   [ST-Link]' if st else ''}" for dev, desc, st in ports]
        self.port_box["values"] = labels
        cur = self.port_var.get().split()[0] if self.port_var.get() else None
        want = preselect or cur
        pick = next((lab for lab, (dev, _, _) in zip(labels, ports) if dev == want), None)
        pick = pick or next((lab for lab, (_, _, st) in zip(labels, ports) if st), None)
        if pick:
            self.port_var.set(pick)
        elif preselect:
            self.port_var.set(preselect)
        elif not labels:
            self.port_var.set("")

    def _on_key(self, e):
        if isinstance(e.widget, (tk.Entry, ttk.Entry, tk.Text)):
            return
        if e.char == "t":
            self.tare()
        elif e.char == "r":
            self.reset_memory()

    def _set(self, label: tk.Label, text: str, level: str):
        label.config(text=text, fg=COLORS[level])

    # -- connection -------------------------------------------------------------------------

    def toggle_connect(self):
        if self.runner is not None:
            self.disconnect()
            return
        sel = self.port_var.get().strip()
        if not sel:
            messagebox.showwarning("No port", "Plug in the Nucleo's ST-Link USB, then press Refresh.")
            return
        port = sel.split()[0]
        self._start(port)
        self.source_name = port
        self.thread = threading.Thread(target=serial_worker, args=(self.runner, port, self.sink, self.stop),
                                       daemon=True)
        self.thread.start()

    def _start(self, name: str, record: bool = True):
        self.stop = threading.Event()
        self.runner = GuiRunner(self.est, TARE_S)
        self.est.reset_hidden()
        self.runner.request_tare()
        if record and self.record_var.get():
            RECORDINGS.mkdir(exist_ok=True)
            self.session_stem = RECORDINGS / time.strftime("%Y%m%d_%H%M%S")
            self.sink = self.session_stem.with_suffix(".bin").open("wb")
            self.events = self.session_stem.with_suffix(".events.jsonl").open("a")
            self.log(f"Recording to {self.session_stem.with_suffix('.bin')}")
        self.event("connect", source=name, leg_map=self.est.leg_map.__dict__)
        self.event("tare_start", seconds=TARE_S, reason="auto on connect")
        self.connect_btn.config(text="Disconnect")
        self.log(f"Connected to {name}. Auto-tare {TARE_S:.0f} s: keep the deck EMPTY.")

    def start_replay(self, path: Path):
        if self.runner is not None:
            self.disconnect()
        self._start(f"replay {path.name}", record=False)
        self.source_name = f"replay {path.name}"
        self.thread = threading.Thread(target=replay_worker, args=(self.runner, path, self.stop), daemon=True)
        self.thread.start()

    def pick_replay(self):
        path = filedialog.askopenfilename(initialdir=RECORDINGS if RECORDINGS.exists() else HERE,
                                          filetypes=[("Recordings", "*.bin *.csv"), ("All", "*.*")])
        if path:
            self.start_replay(Path(path))

    def disconnect(self):
        if self.runner is None:
            return
        self.event("disconnect")
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=1.5)
        for f in (self.sink, self.events):
            if f:
                f.close()
        if self.sink:
            self.log(f"Saved {self.session_stem.with_suffix('.bin')} "
                     f"({self.session_stem.with_suffix('.bin').stat().st_size / 1e6:.1f} MB)")
        self.sink = self.events = None
        self.runner = None
        self.connect_btn.config(text="Connect")
        self.log("Disconnected.")

    def close(self):
        self.disconnect()
        self.root.destroy()

    # -- live actions -----------------------------------------------------------------------

    def tare(self):
        if self.runner:
            self.runner.request_tare()
            self.event("tare_start", seconds=TARE_S)
            self.log("Tare: keep the deck empty for 3 s.")

    def reset_memory(self):
        if self.runner:
            self.runner.request_reset()
            self.event("reset_memory")
            self.log("Estimator memory reset.")

    def toggle_gain(self):
        self.est.leg_map.use_gain = bool(self.gain_var.get())
        self.log(f"Calibrated gains {'ON' if self.est.leg_map.use_gain else 'OFF'}; re-taring.")
        self.event("use_gain", value=self.est.leg_map.use_gain)
        self.tare()

    def load_calib(self):
        path = filedialog.askopenfilename(initialdir=HERE, filetypes=[("Calibration", "*.json")])
        if path:
            self._apply_leg_map(LegMap.load(Path(path)), f"loaded {Path(path).name}")

    def _apply_leg_map(self, m: LegMap, why: str):
        m.use_gain = bool(self.gain_var.get())
        if self.runner:
            with self.runner.lock:
                self.est.leg_map = m
        else:
            self.est.leg_map = m
        self.log(f"Leg map {why}: {map_text(m)}")
        self.event("leg_map", why=why, leg_map=m.__dict__)
        self.tare()

    # -- calibration ------------------------------------------------------------------------

    def calib_start(self):
        if not self.runner or self.runner.last_frame_wall is None:
            messagebox.showwarning("No telemetry", "Connect first and make sure v3 frames are arriving.")
            return
        try:
            mass = float(self.mass_var.get())
            offset = float(self.offset_var.get()) / 100
            assert mass > 0 and 0 < offset < 0.12
        except (ValueError, AssertionError):
            messagebox.showerror("Bad input", "Mass must be > 0 kg and offset between 0 and 12 cm.")
            return
        with self.runner.lock:
            self.runner.calib = CalibrationSession(mass, offset, dt=self.est.dt)
            self.runner.calib.last_seq = self.runner.prev_seq
        self.calib_result = None
        self.result_text.delete("1.0", "end")
        self.apply_btn.config(state="disabled")
        self.event("calib_start", mass_kg=mass, offset_m=offset)
        self.log(f"Calibration started with {mass:g} kg, {offset * 100:.0f} cm offsets.")

    def calib_capture(self):
        c = self.runner.calib if self.runner else None
        if c is None:
            return
        with self.runner.lock:
            name = c.current[0] if c.current else ""
            ok = c.capture()
        if ok:
            self.event("calib_capture", step=name)
            self.log(f"Capturing '{name}': 2 s settle + 4 s average. Don't touch the robot.")

    def calib_redo(self):
        c = self.runner.calib if self.runner else None
        if c:
            with self.runner.lock:
                c.redo_previous()
            self.calib_result = None
            self.result_text.delete("1.0", "end")
            self.apply_btn.config(state="disabled")
            self.event("calib_redo", step=c.current[0] if c.current else "")

    def calib_cancel(self):
        if self.runner:
            with self.runner.lock:
                self.runner.calib = None
            self.event("calib_cancel")
            self.log("Calibration cancelled.")

    def _calib_finished(self, c: CalibrationSession):
        try:
            r = analyze_calibration(self.ref, c.mass, c.means)
        except Exception as exc:
            self.calib_err.config(text=f"Fit failed: {exc}")
            return
        self.calib_result = r
        rows, level, conclusion = judge_calibration(r)
        t = self.result_text
        t.delete("1.0", "end")
        t.insert("end", conclusion + "\n\n", (level, "b"))
        for name, val, lv, why in rows:
            t.insert("end", f"{'OK  ' if lv == 'good' else '??  ' if lv == 'warn' else 'BAD '}", (lv, "b"))
            t.insert("end", f"{name}: ", "b")
            t.insert("end", f"{val}\n", lv)
            t.insert("end", f"      {why}\n", "info")
        t.insert("end", f"\nMapping: {map_text(r['best'])}\n", "b")
        t.insert("end", f"Angle vs sim: {r['angle_x_sim']:.2f}x\n\n")
        t.insert("end", "Torque change, sim expected vs hardware (mapped), Nm  [fl fr bl br]:\n", "b")
        for (name, _, _), e, m in zip(c.steps[1:], r["exp"], r["mapped"]):
            t.insert("end", f"{name:6s} sim {np.round(e, 3)}\n       hw  {np.round(m, 3)}\n")
        policy_id = self.runner.policy_id if self.runner else None
        if policy_id != RANGE_GRU_POLICY_ID:
            t.insert("end", f"\nWARNING: firmware policy is {POLICY_NAMES.get(policy_id, policy_id)}, "
                            "not range_gru.\n", ("bad", "b"))
        self.apply_btn.config(state="normal")
        r["best"].note = (f"calibrated {time.strftime('%Y-%m-%d %H:%M')} with {c.mass} kg, "
                          f"residual {r['residual']:.3f}")
        self.event("calib_result", residual=r["residual"], runner_up=r["runner_up"],
                   torque_x_sim=r["torque_x_sim"], stiffness=r["stiff"].tolist(), leg_map=r["best"].__dict__,
                   means=[dict(xy=xy, angle=a.tolist(), torque=tq.tolist(), n=n) for xy, a, tq, n in c.means])
        if self.session_stem:
            self.session_stem.with_suffix(".calibration.txt").write_text(t.get("1.0", "end"), encoding="utf-8")
        self.log(f"Calibration done: residual {r['residual']:.3f} (runner-up {r['runner_up']:.3f}), "
                 f"torque {r['torque_x_sim']:.2f}x sim. {conclusion}")

    def calib_apply(self):
        if not self.calib_result:
            return
        if not messagebox.askokcancel("Apply calibration", "Is the deck EMPTY now? Applying saves "
                                      f"{self.calib_path.name} and runs a 3 s tare."):
            return
        best: LegMap = self.calib_result["best"]
        if self.calib_path.exists():
            shutil.copyfile(self.calib_path, self.calib_path.with_name("calibration_previous.json"))
        best.save(self.calib_path)
        if self.session_stem:
            best.save(self.session_stem.with_suffix(".calibration.json"))
        self.log(f"Saved {self.calib_path}")
        self._apply_leg_map(best, "from calibration")
        with self.runner.lock:
            self.runner.calib = None
        self.apply_btn.config(state="disabled")
        self.nb.select(1)

    # -- periodic UI update -----------------------------------------------------------------

    def tick(self):
        try:
            self._tick()
        finally:
            self.root.after(UI_PERIOD_MS, self.tick)

    def _tick(self):
        now = time.monotonic()
        if self.runner is None and now - self._ports_t > 2.0:
            self._ports_t = now
            self._refresh_ports()
        r = self.runner
        m = self.est.leg_map
        self.map_label.config(text=f"{map_text(m)}\n{m.note}{'  (gains ON)' if m.use_gain else ''}")
        self._set(self.chk_calib, "calibrated" if m.calibrated else "NOT calibrated (default guess)",
                  "good" if m.calibrated else "warn")
        if self.sink and self.session_stem:
            self.rec_label.config(text=f"{self.session_stem.with_suffix('.bin').name}  "
                                       f"{self.sink.tell() / 1e6:.1f} MB")
        elif r is None:
            self.rec_label.config(text="-")

        if r is None:
            self._set(self.health, "Not connected", "info")
            for lab in (self.chk_link, self.chk_policy, self.chk_robot):
                self._set(lab, "-", "info")
            self.status.config(text="")
            self._update_calib_ui(None)
            self.canvas.draw_idle()
            return

        s = r.snapshot()
        stale = r.last_frame_wall is None or now - r.last_frame_wall > 1.5
        if r.error:
            self._set(self.health, "Link error", "bad")
            self._set(self.chk_link, r.error[:60], "bad")
        elif stale and r.done:
            self._set(self.health, "Stream ended", "info")
            self._set(self.chk_link, "stream ended", "info")
        elif stale and r.bytes > 3000:
            self._set(self.health, "No v3 frames", "bad")
            self._set(self.chk_link, "bytes arrive but no v3 frames: old (v2) firmware flashed?", "bad")
        elif stale and now - r.started_wall > 2.5:
            self._set(self.health, "No data", "bad")
            self._set(self.chk_link, "no data: wrong port, or board not running", "bad")
        elif stale:
            self._set(self.health, "Waiting for data...", "info")
            self._set(self.chk_link, "waiting...", "info")
        else:
            lvl = "good" if s["fps"] >= 55 else "warn"
            self._set(self.health, f"{self.source_name}: {s['fps']} frames/s", lvl)
            self._set(self.chk_link, f"{s['fps']} frames/s, seq gaps {s['gaps']}", lvl)
        if r.error and r.error != self.last_msg:
            self.log(f"Link error: {r.error}")
            self.last_msg = r.error

        pid = s["policy_id"]
        if pid is None:
            self._set(self.chk_policy, "-", "info")
        elif pid == RANGE_GRU_POLICY_ID:
            self._set(self.chk_policy, "range_gru", "good")
        else:
            self._set(self.chk_policy, f"{POLICY_NAMES.get(pid, pid)}  (estimator expects range_gru!)", "bad")
        if s["last"] is not None:
            self._set(self.chk_robot, "FALLEN" if s["fallen"] else "balancing", "bad" if s["fallen"] else "good")

        if s["msg"] and s["msg"] != self.last_msg:
            self.log(s["msg"])
            self.last_msg = s["msg"]
        self.status.config(text=f"frames/s {s['fps']:3d}   seq gaps {s['gaps']}   fw dropped {s['dropped']}   "
                                f"policy {POLICY_NAMES.get(pid, pid)}   |  {s['msg']}")

        if s["last"] is not None:
            _, mass, off = s["last"]
            show = mass >= SHOW_POSITION_ABOVE_KG and not s["taring"] and not s["fallen"]
            sx, sy = -off[0], -off[1]
            self.dot.set_center((sx, sy))
            self.ring.set_center((sx, sy))
            self.ring.set_radius(location_uncertainty_m(mass))
            self.dot.set_visible(show)
            self.ring.set_visible(show)
            if s["taring"]:
                self.mass_label.config(text="taring...")
                self.pos_label.config(text="keep the deck EMPTY")
                self.deck_note.set_text("taring")
            elif s["fallen"]:
                self.mass_label.config(text="fallen")
                self.pos_label.config(text="stand it up, empty the deck, then Tare")
                self.deck_note.set_text("")
            else:
                self.mass_label.config(text=f"{mass:.3f} kg   ({mass * 1000:.0f} g)")
                self.deck_note.set_text("" if show else "empty")
                if show:
                    lat = f"{abs(off[0]) * 100:.1f} cm {'left' if off[0] > 0 else 'right'}"
                    fa = f"{abs(off[1]) * 100:.1f} cm {'front' if off[1] < 0 else 'back'}"
                    weak = "\n(position unreliable below ~0.25 kg)" if mass < 0.25 else ""
                    self.pos_label.config(text=f"{fa}, {lat} of deck centre{weak}")
                else:
                    self.pos_label.config(text="deck empty")
            h = np.array(s["hist"])
            if len(h):
                self.mass_line.set_data(h[:, 0] - h[-1, 0], h[:, 1])
                self.ax_mass.set_ylim(-0.05, max(0.5, h[:, 1].max() * 1.15))
            for b, v in zip(self.bars, s["legs"]):
                b.set_height(v)

        with r.lock:
            c = r.calib
            snap = None if c is None else dict(phase=c.phase, idx=c.idx, current=c.current, error=c.error,
                                               progress=c.progress(), steps=c.steps, means=len(c.means))
        if c is not None and snap["phase"] == "done" and self.calib_result is None:
            self._calib_finished(c)
        self._update_calib_ui(snap)
        self.canvas.draw_idle()

    def _update_calib_ui(self, c: dict | None):
        streaming = self.runner is not None and self.runner.last_frame_wall is not None
        if c is None:
            self.calib_start_btn.config(state="normal" if streaming else "disabled",
                                        text="Start calibration")
            for b in (self.capture_btn, self.redo_btn, self.cancel_btn):
                b.config(state="disabled")
            for lab in self.step_labels:
                lab.config(fg=COLORS["info"], text="o" + lab.cget("text")[1:])
            self.calib_prog["value"] = 0
            self.calib_err.config(text="")
            self.target.set_data([], [])
            self.target_txt.set_text("")
            if not self.calib_result:
                self.instr.config(text="Connect, let the robot balance (empty deck), then press Start calibration."
                                  if not streaming else "Robot streaming. Press Start calibration.")
            return
        self.calib_start_btn.config(state="normal", text="Restart calibration")
        busy = c["phase"] in ("settle", "capture")
        self.capture_btn.config(state="normal" if c["phase"] == "wait" else "disabled")
        self.redo_btn.config(state="normal" if not busy and c["means"] else "disabled")
        self.cancel_btn.config(state="normal")
        for i, lab in enumerate(self.step_labels):
            mark, col = ("v", COLORS["good"]) if i < c["idx"] else (">", COLORS["warn"]) if i == c["idx"] \
                else ("o", COLORS["info"])
            lab.config(fg=col, text=mark + lab.cget("text")[1:])
        self.calib_prog["value"] = c["progress"]
        self.calib_err.config(text=c["error"])
        cur = c["current"]
        if c["phase"] == "done":
            self.instr.config(text="All 6 placements captured. Check the result below.")
            self.target.set_data([], [])
            self.target_txt.set_text("")
            return
        name, text, xy = cur
        if busy:
            self.instr.config(text=f"Step {c['idx'] + 1}/6, {name}: "
                                   f"{'settling' if c['phase'] == 'settle' else 'measuring'}... hands off.")
        else:
            self.instr.config(text=f"Step {c['idx'] + 1}/6, {name}: {text}")
        if xy is None:
            self.target.set_data([], [])
            self.target_txt.set_text("")
        else:
            sx, sy = -xy[0], -xy[1]
            self.target.set_data([sx], [sy])
            self.target_txt.set_position((sx, sy - 0.014))
            self.target_txt.set_text("place here")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="preselect this serial port (e.g. COM7)")
    p.add_argument("--replay", type=Path, help="start by replaying a .bin recording or capture CSV")
    p.add_argument("--calib", type=Path, default=DEFAULT_CALIB, help="leg map file to load and save")
    args = p.parse_args()
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
    root = tk.Tk()
    App(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
