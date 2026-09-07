"""Check the IMU sign/frame conventions in an STM Studio log from the ERK build.

Usage:
    python Tools/check_imu_signs.py <log.txt> [<log.txt> ...]

Reads the LOG_ variables the ERK firmware exports and answers the two questions
that decide whether the robot can balance at all:

  PITCH  d(action)/d(theta) must be POSITIVE on both wheels.
         ERK reference runs: +1.4 .. +4.6.

  YAW    the yaw angle and the yaw rate fed to the policy must be coherent, i.e.
         yaw_rate_obs ~= +1.0 * d(yaw_err)/dt.
         ERK reference runs: +0.59 .. +0.89.
         A NEGATIVE slope means angle and rate are in different frames -- a
         firmware bug (see IMU_UpdateFromBNO), not a mounting question.
         Coherent but still spinning up -> flip YAW_MOUNT_SIGN.

Samples where the policy is not running (isFallen aborts ANN_Run, leaving the
obs frozen) and samples where the current command is saturated are excluded,
since neither carries information about the feedback sign.
"""

import sys
import numpy as np

PITCH_SCALE = 0.43633231  # rad, obs[2] normalisation (25 deg)
RATE_SCALE = 4.0          # rad/s, obs[3] and obs[5] normalisation
CURRENT_LIMIT = 0.5       # A, STM32_TEST_CURRENT_LIMIT_A


def load(path):
    """Parse an STM Studio 'Syntax version=4' export into (names, t, data).

    Rows are de-duplicated: STM Studio samples far faster than the 15 ms LOG_
    tick, so most rows are verbatim repeats of the previous one.
    """
    names, rows = None, []
    with open(path, errors="replace") as fh:
        for line in fh:
            if not line.startswith("D:"):
                continue
            parts = line.rstrip("\n").split("\t")
            if names is None and "LOG_" in line:
                names = ["time"] + [p for p in parts[1:] if p.startswith("LOG")]
                continue
            if len(parts) > 1 and parts[1].startswith("time"):
                continue
            try:
                rows.append([float(x) for x in parts[1:]])
            except ValueError:
                pass
    if names is None or not rows:
        raise SystemExit("%s: no LOG_ variables found" % path)
    a = np.array(rows)
    t = a[:, 0] / 1000.0
    changed = np.abs(np.diff(a[:, 1:], axis=0)).sum(axis=1)
    idx = np.concatenate(([0], np.nonzero(changed)[0] + 1))
    # The de-duplicated view is what every check below uses; the raw view is kept
    # only for tick_rate(), which must count changes against real elapsed time.
    return names, t[idx], a[idx], t, a


def active_masks(col, a):
    """(live, unsaturated) tick masks.

    live        - the policy is producing output; excludes the stretches where
                  isFallen() aborts ANN_Run and leaves the obs frozen.
    unsaturated - additionally drops ticks where a wheel command is clipped at
                  the current limit, which flattens any measured feedback gain.

    The yaw coherence test only needs `live`: whether the angle and the rate
    agree is a property of the sensor frames, not of the actuation.
    """
    frozen = np.abs(np.diff(a[:, col["LOG_obs[2]"]], prepend=np.nan)) == 0.0
    iL = a[:, col["LOG_I_L_cmd_A"]]
    iR = a[:, col["LOG_I_R_cmd_A"]]
    live = ((np.abs(iL) > 1e-9) | (np.abs(iR) > 1e-9)) & ~frozen
    unsat = live & (np.abs(iL) < CURRENT_LIMIT - 1e-4) & (np.abs(iR) < CURRENT_LIMIT - 1e-4)
    return live, unsat


def fit(x, y):
    return np.polyfit(x, y, 1)[0], np.corrcoef(x, y)[0, 1]


def tick_rate(t_raw, action_raw, live_span):
    """Actual control-loop rate, from how often the ANN output changes.

    ANN_Run() produces a new float every tick it executes, so the rate at which
    action_L changes IS the control rate. Nominal is 66.7 Hz (15 ms TIM4).
    isCYBERGEARReady is a bool, not a counter, so any superloop pass that
    overruns 15 ms silently loses that tick -- this is the only place it shows up.
    """
    t0, t1 = live_span
    m = (t_raw >= t0) & (t_raw < t1)
    if m.sum() < 100:
        return
    span = t_raw[m][-1] - t_raw[m][0]
    if span <= 0:
        return
    rate = (np.diff(action_raw[m]) != 0.0).sum() / span
    verdict = "OK" if rate > 63.0 else "*** TICKS BEING DROPPED ***"
    print("  TICK   control loop %.1f Hz (nominal 66.7)   %s" % (rate, verdict))


def freshness(t, theta, live):
    """How often the pitch the policy is fed actually changes.

    theta only refreshes when the BNO delivers a new gravity report; on ticks
    where it does not change, the policy is acting on a stale angle. The control
    loop is 66.7 Hz nominal, and the old ESP32 bridge sustained 57-65 Hz. The
    first direct-BNO build managed only 34 Hz (median 24.7 ms between refreshes,
    worst 127 ms), which doubled the balancing RMS -- see BNO08x_Init.
    """
    tt, th = t[live], theta[live]
    if len(tt) < 100:
        return
    idx = np.nonzero(np.diff(th) != 0.0)[0]
    span = tt[-1] - tt[0]
    if len(idx) < 3 or span <= 0:
        return
    gaps = np.diff(tt[idx]) * 1000.0
    rate = len(idx) / span
    verdict = "OK" if rate > 50.0 else "*** STALE -- policy is seeing a lagged angle ***"
    print("  IMU    pitch refresh %.1f Hz (want >50, loop is 66.7)   "
          "gap median %.1f ms  p90 %.1f ms  worst %.1f ms   %s"
          % (rate, np.median(gaps), np.percentile(gaps, 90), gaps.max(), verdict))


def report(path):
    names, t, a, t_raw, a_raw = load(path)
    col = {n: i for i, n in enumerate(names)}
    live, act = active_masks(col, a)

    print("=" * 72)
    print(path)
    print("  %d ticks, %.1f s; policy live on %d, of which %d unsaturated"
          % (len(t), t[-1] - t[0], live.sum(), act.sum()))
    if live.sum() < 50:
        print("  too little live data to judge -- the policy barely ran")
        return

    theta = np.deg2rad(a[:, col["LOG_theta_deg"]])

    # obs[2] must be theta/PITCH_SCALE; a mismatch here means the log and the
    # policy disagree about what theta is, which is a wiring bug in LOG_Update.
    resid = np.rad2deg(np.abs(a[live, col["LOG_obs[2]"]] * PITCH_SCALE - theta[live]))
    # The log prints 6 decimals of a float32, so a few hundredths of a degree is
    # rounding, not disagreement.
    print("  obs[2] vs LOG_theta_deg: median mismatch %.4f deg  %s"
          % (np.median(resid), "OK" if np.median(resid) < 0.05 else "*** MISMATCH ***"))

    if act.sum() >= 50:
        sL, cL = fit(theta[act], a[act, col["LOG_action_L"]])
        sR, _ = fit(theta[act], a[act, col["LOG_action_R"]])
        verdict = "OK (positive, as in ERK)" if sL > 0 and sR > 0 else "*** INVERTED ***"
        print("  PITCH  d(action)/d(theta) = %+.3f / %+.3f  (corr %+.3f, n=%d)   %s"
              % (sL, sR, cL, act.sum(), verdict))
    else:
        print("  PITCH  only %d unsaturated ticks -- not enough to judge" % act.sum())

    yaw_err = a[:, col["LOG_obs[4]"]] * np.pi
    yaw_rate = a[:, col["LOG_obs[5]"]] * RATE_SCALE

    # Central difference over +-3 ticks, dropping anything near a +-pi wrap.
    n = 3
    d = np.full_like(yaw_err, np.nan)
    d[n:-n] = (yaw_err[2 * n:] - yaw_err[:-2 * n]) / (t[2 * n:] - t[:-2 * n])
    wrap = np.abs(np.diff(yaw_err, prepend=yaw_err[0])) > 1.0
    near_wrap = np.convolve(wrap.astype(float), np.ones(2 * n + 3), mode="same") > 0
    m = live & np.isfinite(d) & ~near_wrap & (np.abs(d) > 0.15)

    tick_rate(t_raw, a_raw[:, col["LOG_action_L"]], (t[live][0], t[live][-1]))
    freshness(t, a[:, col["LOG_theta_deg"]], live)

    if m.sum() < 30:
        print("  YAW    too little heading motion to judge (n=%d) -- fine if the"
              " robot held its heading" % m.sum())
        return
    s, c = fit(d[m], yaw_rate[m])
    if s > 0.3:
        verdict = "OK (coherent, as in ERK)"
    elif s < -0.3:
        verdict = "*** ANGLE AND RATE IN DIFFERENT FRAMES ***"
    else:
        verdict = "*** INCONCLUSIVE / DECOUPLED ***"
    print("  YAW    yaw_rate_obs = %+.3f * d(yaw_err)/dt  (corr %+.3f, n=%d)   %s"
          % (s, c, m.sum(), verdict))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    for p in sys.argv[1:]:
        report(p)
