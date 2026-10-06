#!/usr/bin/env python3
"""Check packaged hashes without external dependencies."""

import hashlib
from pathlib import Path

root = Path(__file__).resolve().parent
expected = {}
for line in (root / "SHA256SUMS").read_text().splitlines():
    digest, name = line.split("  ", 1)
    assert not Path(name).is_absolute() and ".." not in Path(name).parts
    expected[name] = digest
for name, digest in expected.items():
    path = root / name
    assert path.is_file(), f"Missing {name}"
    assert hashlib.sha256(path.read_bytes()).hexdigest() == digest, f"Hash mismatch: {name}"
print(f"PASS: {len(expected)} packaged files match SHA256SUMS")
