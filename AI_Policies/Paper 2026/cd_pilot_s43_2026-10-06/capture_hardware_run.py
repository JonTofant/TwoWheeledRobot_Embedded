#!/usr/bin/env python3
"""Capture paper telemetry with declared C/D model identity and sidecar metadata."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def annotate_csv(path, metadata):
    columns = (
        "comparison_arm",
        "model_sha256",
        "selected_iteration",
        "repeat",
        "firmware_commit",
        "firmware_elf_sha256",
    )
    expected = 2 if metadata["arm"] == "C" else 1
    mismatches = 0
    rows = 0
    with path.open(newline="") as src:
        reader = csv.DictReader(src)
        if not reader.fieldnames:
            raise ValueError("Capture produced no CSV header")
        if set(columns) & set(reader.fieldnames):
            raise ValueError("CSV is already annotated")
        with tempfile.NamedTemporaryFile("w", newline="", dir=path.parent, delete=False) as dst:
            temporary = Path(dst.name)
            writer = csv.DictWriter(dst, fieldnames=reader.fieldnames + list(columns))
            writer.writeheader()
            for row in reader:
                if int(row.get("active_policy_id", -1)) != expected:
                    mismatches += 1
                row.update(
                    comparison_arm=metadata["arm"],
                    model_sha256=metadata["model_sha256"],
                    selected_iteration=metadata["selected_iteration"],
                    repeat=metadata["repeat"],
                    firmware_commit=metadata["firmware_commit"],
                    firmware_elf_sha256=metadata.get("firmware_elf_sha256") or "",
                )
                writer.writerow(row)
                rows += 1
    os.replace(temporary, path)
    return {"rows": rows, "unexpected_legacy_policy_id_rows": mismatches}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="ST-Link VCP, e.g. COM3 or /dev/ttyACM0")
    parser.add_argument("--arm", choices=("C", "D"), required=True)
    parser.add_argument("--repeat", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=0, help="0: record until Ctrl+C")
    parser.add_argument("--firmware-elf", type=Path)
    parser.add_argument("--firmware-commit", help="Shared source revision used to build firmware")
    parser.add_argument("--linear-mps", type=float, default=0.5)
    parser.add_argument("--yaw-radps", type=float, default=1.0)
    parser.add_argument("--leg-degrees", type=float, default=35.0)
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error("--repeat must be positive")
    if args.duration < 0:
        parser.error("--duration cannot be negative")
    sidecar = args.output.with_suffix(".metadata.json")
    if args.output.exists() or sidecar.exists():
        parser.error("Refusing to overwrite an existing capture")
    manifest = json.loads((PACKAGE / "manifest.json").read_text())
    arm = manifest["arms"][args.arm]
    model = PACKAGE / arm["directory"] / "policy_drive_stm32ai.onnx"
    if digest(model) != arm["exports"]["policy_drive_stm32ai.onnx"]:
        raise SystemExit("Model hash mismatch")
    metadata = {
        "arm": args.arm,
        "model_sha256": digest(model),
        "selected_iteration": arm["selected_iteration"],
        "repeat": args.repeat,
        "firmware_commit": args.firmware_commit or manifest["embedded_base_commit"],
        "firmware_elf_sha256": digest(args.firmware_elf) if args.firmware_elf else None,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "identity_source": "operator-declared model; UART reports legacy policy ID, not an on-device model hash",
        "testbench_settings": {
            "DBG_testbench_linear_mps": args.linear_mps,
            "DBG_testbench_yaw_rate_radps": args.yaw_radps,
            "DBG_testbench_leg_degrees": args.leg_degrees,
        },
        "settings_source": "operator-declared; capture script does not change debugger variables",
        "status": "capture starting",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    sidecar.write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Declared arm {args.arm}, model SHA256 {metadata['model_sha256']}", flush=True)
    command = [
        sys.executable,
        str(PACKAGE / "capture_tools/paper_uart_capture.py"),
        args.port,
        "--baud",
        "921600",
        "--duration",
        str(args.duration),
        "--output",
        str(args.output),
        "--reset",
    ]
    process = subprocess.Popen(command)
    try:
        returncode = process.wait()
    except KeyboardInterrupt:
        # The underlying recorder handles Ctrl+C and closes its CSV. Allow it to finish.
        try:
            returncode = process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.terminate()
            returncode = process.wait()
    metadata["capture_exit_code"] = returncode
    if args.output.exists():
        metadata.update(annotate_csv(args.output, metadata))
    metadata["finished_utc"] = datetime.now(timezone.utc).isoformat()
    metadata["status"] = "recorded" if metadata.get("rows", 0) > 0 else "empty or failed capture"
    if metadata.get("unexpected_legacy_policy_id_rows"):
        metadata["status"] = "recorded; policy ID mismatch requires investigation"
    sidecar.write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Saved {args.output} and {sidecar}", flush=True)
    if returncode or not metadata.get("rows") or metadata.get("unexpected_legacy_policy_id_rows"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
