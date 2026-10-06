#!/usr/bin/env python3
"""Bounded weight-only adaptation of the hardware-proven August GRU.

Run inside isaac-lab-dev using /isaac-sim/python.sh. Keeps the corrected plant,
restores the 0.15 exploration floor, and never overwrites r2 paper results.
Exports are development candidates until independently tested on hardware.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=int, default=150)
    parser.add_argument("--seed", type=int, default=43)
    parser.add_argument("--num-envs", type=int, default=4096)
    parser.add_argument("--tag", default="gru_recovery_floor015_s43_2026-10-06")
    args = parser.parse_args()
    if args.iterations <= 0 or args.num_envs <= 0:
        parser.error("iterations and num-envs must be positive")
    if not args.tag or Path(args.tag).name != args.tag or args.tag in (".", ".."):
        parser.error("tag must be a single directory name")

    repo = Path(__file__).resolve().parents[2]
    runs = repo / "logs/rsl_rl/nn_drive_fixed_stance"
    baseline_run = runs / "2026-08-10_23-51-29_range_gru_stage5"
    baseline = baseline_run / "model_775.pt"
    r2 = runs / "2026-10-03_01-05-32_r2_range_gru_s43_stage5/model_1395.pt"
    output = repo / "logs" / args.tag
    bundle = repo / "ExportedPolicy" / args.tag
    if not baseline.is_file() or not r2.is_file():
        parser.error("required August baseline or r2 C43 checkpoint is missing")
    if output.exists() or bundle.exists():
        parser.error("tag already exists; choose a new tag to preserve previous evidence")
    output.mkdir(parents=True)
    task = "Template-Twowheeledrobot-NNDriveFixedStanceGRU-v0"
    metadata = {
        "purpose": "development recovery after reported C43 hardware regression",
        "baseline": str(baseline),
        "baseline_sha256": digest(baseline),
        "training_seed": args.seed,
        "iterations": args.iterations,
        "exploration_floor": [0.15, 0.15],
        "learning_rate": 5e-5,
        "learning_rate_schedule": "fixed",
        "restore_optimizer": False,
        "structural_fixes_preserved": True,
        "hardware_validation_complete": False,
        "git_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repo, text=True
        ).strip(),
        "commands": [],
    }
    metadata_path = output / "metadata.json"

    def save() -> None:
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")

    def run(phase: str, *command: str) -> str:
        metadata["phase"] = phase
        metadata["commands"].append(list(command))
        save()
        print(f"START {phase}", flush=True)
        log_path = output / f"{phase}.log"
        for attempt in range(1, 4):
            with log_path.open("w") as log:
                result = subprocess.run(command, cwd=repo, stdout=log, stderr=subprocess.STDOUT)
            text = log_path.read_text()
            startup_crash = (
                result.returncode == -11
                and "XOpenDisplay" in text
                and "Loading model checkpoint" not in text
                and "Learning iteration" not in text
            )
            if not startup_crash or attempt == 3:
                break
            shutil.copyfile(log_path, output / f"{phase}.startup_crash_{attempt}.log")
            print(f"RETRY {phase}: known Kit startup crash, attempt {attempt}", flush=True)
        if result.returncode:
            metadata["failed_phase"] = phase
            metadata["exit_code"] = result.returncode
            save()
            raise SystemExit(f"{phase} failed ({result.returncode}); see {log_path}")
        print(f"DONE {phase}", flush=True)
        return log_path.read_text()

    diff = subprocess.check_output(["git", "diff", "--binary"], cwd=repo)
    (output / "source_changes.diff").write_bytes(diff)
    shutil.copyfile(__file__, output / Path(__file__).name)
    for relative in [
        "scripts/rsl_rl/train.py",
        "scripts/tools/verify_retrain_changes.py",
        "scripts/tools/expand_gru_for_stm32ai.py",
        "source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/agents/rsl_rl_nn_drive_cfg.py",
    ]:
        destination = output / "source" / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(repo / relative, destination)
    preflight = run("preflight", sys.executable, "scripts/tools/verify_retrain_changes.py", "--headless")
    if "[verify] DONE failures=[]" not in preflight:
        raise SystemExit("Preflight did not confirm all fixes; refusing to train")

    previous = set(runs.iterdir())
    run(
        "train", sys.executable, "scripts/rsl_rl/train.py",
        "--task", task, "--num_envs", str(args.num_envs),
        "--max_iterations", str(args.iterations), "--seed", str(args.seed),
        "--run_name", f"{args.tag}_stage5", "--headless", "--resume",
        "--load_run", baseline_run.name, "--checkpoint", baseline.name,
        "--reset_optimizer", "env.curriculum_stage=5", "env.terrain_mode=generator",
        "agent.action_std_floor=[0.15,0.15]",
        "agent.algorithm.learning_rate=0.00005", "agent.algorithm.schedule=fixed",
    )
    new_runs = [p for p in set(runs.iterdir()) - previous if p.name.endswith(f"{args.tag}_stage5")]
    if len(new_runs) != 1:
        raise SystemExit(f"Expected exactly one new training run, got {new_runs}")
    trained = new_runs[0]
    checkpoint = max(trained.glob("model_*.pt"), key=lambda p: int(p.stem.split("_")[-1]))
    metadata.update(checkpoint=str(checkpoint), checkpoint_sha256=digest(checkpoint))
    save()
    run(
        "export_actor", sys.executable, "scripts/rsl_rl/play.py", "--task", task,
        "--checkpoint", str(checkpoint), "--num_envs", "1", "--num_steps", "1", "--headless",
    )
    bundle.mkdir()
    native = bundle / "policy_drive.onnx"
    converted = bundle / "policy_drive_stm32ai.onnx"
    run(
        "export_current", sys.executable, "scripts/export_pure_nn_current_onnx.py",
        "--policy", str(trained / "exported/policy.pt"),
        "--actor-onnx", str(trained / "exported/policy.onnx"),
        "--output", str(native), "--obs-dim", "13", "--cg-outputs", "0",
        "--i-max-a", "2.0", "--require-validation",
    )
    run(
        "convert_gru", sys.executable, "scripts/tools/expand_gru_for_stm32ai.py",
        str(native), str(converted), "--steps", "1024",
    )
    for name, model in [("old_baseline", baseline), ("r2_c43", r2), ("recovery", checkpoint)]:
        run(
            f"benchmark_{name}", sys.executable, "scripts/benchmark_nn_drive.py",
            "--task", task, "--checkpoint", str(model), "--num_envs", "256",
            "--num_steps", "1000", "--terrain", "generator", "--seed", "42",
            "--json-output", str(output / f"benchmark_{name}.json"), "--headless",
        )
        shutil.copyfile(output / f"benchmark_{name}.json", bundle / f"benchmark_{name}.json")
    metadata["phase"] = "complete"
    metadata["native_onnx_sha256"] = digest(native)
    metadata["deploy_onnx_sha256"] = digest(converted)
    save()
    shutil.copyfile(metadata_path, bundle / "manifest.json")
    shutil.copytree(trained / "params", bundle / "params")
    for phase in ("export_current", "convert_gru"):
        shutil.copyfile(output / f"{phase}.log", bundle / f"{phase}.log")
    hashes = [f"{digest(p)}  {p.relative_to(bundle)}" for p in sorted(bundle.rglob("*")) if p.is_file()]
    (bundle / "SHA256SUMS").write_text("\n".join(hashes) + "\n")
    print(f"COMPLETE candidate bundle: {bundle}; hardware validation pending", flush=True)


if __name__ == "__main__":
    main()
