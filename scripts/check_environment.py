#!/usr/bin/env python3
"""Report the local environment without installing or changing anything."""

from __future__ import annotations

import importlib
import platform
import sys


MODULES = ("pinocchio", "mujoco", "numpy", "scipy", "osqp", "pytest")


def main() -> int:
    print(f"python: {sys.version.split()[0]} ({sys.executable})")
    print(f"platform: {platform.platform()}")

    missing = []
    for name in MODULES:
        try:
            module = importlib.import_module(name)
        except Exception as exc:  # environment diagnostics should show all failures
            missing.append(name)
            print(f"{name}: MISSING ({type(exc).__name__}: {exc})")
            continue
        version = getattr(module, "__version__", "installed")
        print(f"{name}: OK ({version})")

    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())

