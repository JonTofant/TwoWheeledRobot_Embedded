"""Payload mass/position estimator runtime for the laptop side (no GUI here).

The STM32 streams paper telemetry protocol v3 (Tools/paper_uart_capture.py,
Docs/PAPER_TELEMETRY.md) every 15 ms control tick. This module turns each frame
into the 23 features the sim-trained estimator expects, steps the recurrent
ONNX model, and handles the two things the sim cannot know about the hardware:

- the CyberGear index/sign mapping onto the sim legs (fl, fr, bl, br), fitted by
  calibrate() from known placements of a known mass; and
- per-channel zero offsets, removed by a tare on an empty deck.

Sim deck frame used throughout: x = lateral, +x = robot's LEFT;
y = fore/aft, -y = FRONT (the direction a positive velocity command drives).
"""

from __future__ import annotations

import itertools
import json
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paper_uart_capture import POLICY_NAMES, extract_frames  # noqa: E402,F401

HERE = Path(__file__).resolve().parent
MODEL_DIR = HERE / "model"
LEG_NAMES = ["fl", "fr", "bl", "br"]
RANGE_GRU_POLICY_ID = 2
FLAG_FALLEN = 1 << 1

# Best guess from the firmware comments (main.c test bench: index 0 = left-back,
# 1 = left-front, 2 = right-front, 3 = right-back; every hardware leg extends in the
# opposite direction to its sim joint). UNVERIFIED -- run `calibrate` first.
DEFAULT_PERM = [1, 2, 0, 3]  # sim leg j reads hardware CyberGear index DEFAULT_PERM[j]
DEFAULT_SIGN = [-1.0, -1.0, -1.0, -1.0]


@dataclass
class LegMap:
    perm: list[int] = field(default_factory=lambda: list(DEFAULT_PERM))
    sign: list[float] = field(default_factory=lambda: list(DEFAULT_SIGN))
    torque_gain: float = 1.0  # hardware -> sim scale, applied only with use_gain
    angle_gain: float = 1.0
    use_gain: bool = False
    calibrated: bool = False
    note: str = "default guess from firmware comments, not calibrated"

    @classmethod
    def load(cls, path: Path | None) -> LegMap:
        if path is None or not Path(path).exists():
            return cls()
        return cls(**json.loads(Path(path).read_text()))

    def save(self, path: Path) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=1))

    def to_sim(self, angle_dev_hw: np.ndarray, torque_hw: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        p, s = np.array(self.perm), np.array(self.sign)
        ga, gt = (self.angle_gain, self.torque_gain) if self.use_gain else (1.0, 1.0)
        return s * angle_dev_hw[p] * ga, s * torque_hw[p] * gt


def frame_to_raw(row: dict) -> dict:
    """The hardware quantities the estimator path needs, from one decoded frame."""
    return dict(
        seq=int(row["sequence"]), t=float(row["time_s"]), flags=int(row["flags"]),
        policy_id=int(row["active_policy_id"]), dropped=int(row["dropped_frames"]),
        obs=np.array([row[f"obs_{i}"] for i in range(13)], dtype=np.float32),
        # Logical (policy-frame) currents: the raw ONNX output, before the firmware's
        # destination swap and wiring signs. wheel_*_current_command_A are physical.
        current=np.array([row["action_left"], row["action_right"]], dtype=np.float32),
        angle=np.array([row[f"leg_{i}_angle_rad"] for i in range(4)], dtype=np.float32),
        target=np.array([row[f"leg_{i}_target_rad"] for i in range(4)], dtype=np.float32),
        torque=np.array([row[f"leg_{i}_torque_Nm"] for i in range(4)], dtype=np.float32),
    )


class PayloadEstimator:
    """Feature mapping + tare + recurrent ONNX step. One instance per robot."""

    def __init__(self, leg_map: LegMap, model_dir: Path = MODEL_DIR):
        import onnxruntime as ort

        self.ref = json.loads((model_dir / "sim_reference.json").read_text())
        self.sess = ort.InferenceSession(str(model_dir / "estimator.onnx"), providers=["CPUExecutionProvider"])
        self.leg_map = leg_map
        self.empty_mean = np.array(self.ref["empty_mean"], dtype=np.float32)
        self.dt = float(self.ref["control_dt_s"])
        self.tare_offset = np.zeros(23, dtype=np.float32)
        self._tare_buf: list[np.ndarray] | None = None
        self._tare_needed = 0
        self.reset_hidden()

    # -- features ---------------------------------------------------------------

    def features(self, raw: dict, tared: bool = True) -> np.ndarray:
        """23 features in sim units/frame: obs(13), currents(2), leg angle(4), leg torque(4)."""
        angle, torque = self.leg_map.to_sim(raw["angle"] - raw["target"], raw["torque"])
        x = np.concatenate([raw["obs"], raw["current"], angle, torque]).astype(np.float32)
        return x + self.tare_offset if tared else x

    # -- tare -------------------------------------------------------------------

    def start_tare(self, seconds: float = 3.0) -> None:
        """Average the next `seconds` of an EMPTY, balancing robot; then the leg
        channels are shifted onto the sim's empty-deck mean and the GRU restarts."""
        self._tare_buf = []
        self._tare_needed = max(1, int(seconds / self.dt))

    @property
    def taring(self) -> bool:
        return self._tare_buf is not None

    def _feed_tare(self, x_untared: np.ndarray) -> None:
        self._tare_buf.append(x_untared)
        if len(self._tare_buf) >= self._tare_needed:
            mean = np.mean(self._tare_buf, axis=0)
            self.tare_offset = np.zeros(23, dtype=np.float32)
            self.tare_offset[15:23] = self.empty_mean[15:23] - mean[15:23]
            self._tare_buf = None
            self.reset_hidden()

    # -- estimator --------------------------------------------------------------

    def reset_hidden(self) -> None:
        self.h = np.zeros((1, 1, int(self.ref["hidden_size"])), dtype=np.float32)
        self.mass, self.offset_xy = 0.0, np.zeros(2, dtype=np.float32)

    def step(self, raw: dict) -> tuple[float, np.ndarray]:
        """Advance one control tick. Returns (mass_kg, offset_xy_m in the sim deck frame)."""
        if self.taring:
            self._feed_tare(self.features(raw, tared=False))
            return self.mass, self.offset_xy
        x = self.features(raw)[None, :]
        mass, off, self.h = self.sess.run(None, {"features": x, "h_in": self.h})
        self.mass = max(0.0, float(mass.reshape(-1)[0]))
        self.offset_xy = off.reshape(-1).astype(np.float32)
        return self.mass, self.offset_xy


def location_uncertainty_m(mass_kg: float) -> float:
    """Typical location error vs mass, from the closed-loop sim eval (MAE per axis):
    ~4 cm at 0.1 kg, ~2 cm at 0.25 kg, ~1 cm at 0.5 kg, ~0.5 cm at >= 1 kg."""
    return float(np.clip(0.006 * max(mass_kg, 0.05) ** -0.84, 0.0045, 0.06))


# -- calibration fit -------------------------------------------------------------

def signature(ref: dict, mass: float, x: float, y: float) -> tuple[np.ndarray, np.ndarray]:
    """Expected sim change in (leg angle[4], leg torque[4]) for `mass` at deck (x, y)."""
    sig = ref["leg_signature"]

    def one(name):
        s = sig[name]
        return s["per_kg"] * mass + s["per_kg_per_m_lateral"] * mass * x + s["per_kg_per_m_foreaft"] * mass * y

    return (np.array([one(f"leg_angle_{k}_rad") for k in LEG_NAMES]),
            np.array([one(f"leg_torque_{k}_Nm") for k in LEG_NAMES]))


def fit_leg_map(ref: dict, d_angle_hw: np.ndarray, d_torque_hw: np.ndarray,
                placements: list[tuple[float, float, float]]) -> tuple[LegMap, list[tuple[float, LegMap]]]:
    """Find the signed permutation hardware -> sim that best explains the measured changes.

    d_angle_hw, d_torque_hw: (K, 4) change from the empty deck, hardware index order,
    angles already relative to the commanded target. placements: K x (mass, x, y).
    Every one of the 4! x 2^4 = 384 signed permutations is scored by the torque
    residual after one best-fit positive gain; angles must then agree in sign.
    """
    exp_t = np.stack([signature(ref, *p)[1] for p in placements])  # (K, 4) sim order
    exp_a = np.stack([signature(ref, *p)[0] for p in placements])
    scored = []
    for perm in itertools.permutations(range(4)):
        for sign in itertools.product((1.0, -1.0), repeat=4):
            s, p = np.array(sign), list(perm)
            meas_t = s * d_torque_hw[:, p]  # hardware mapped into sim order
            g = float((meas_t * exp_t).sum() / max((exp_t * exp_t).sum(), 1e-12))
            if g <= 0:
                continue
            resid = float(np.linalg.norm(meas_t / g - exp_t) / max(np.linalg.norm(exp_t), 1e-12))
            meas_a = s * d_angle_hw[:, p]
            ga = float((meas_a * exp_a).sum() / max((exp_a * exp_a).sum(), 1e-12))
            scored.append((resid, LegMap(perm=p, sign=list(sign), torque_gain=1.0 / g,
                                         angle_gain=(1.0 / ga) if ga > 0 else float("nan"),
                                         calibrated=True)))
    scored.sort(key=lambda r: r[0])
    return scored[0][1], scored
