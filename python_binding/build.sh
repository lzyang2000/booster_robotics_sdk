#!/usr/bin/env bash
# Build the `booster_lowlevel` Python module against this SDK's prebuilt library, with
# the Python given by $PYTHON (default: python3), which needs pybind11 installed.
# The module lands in build/booster_lowlevel.cpython-*.so.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
PYTHON="${PYTHON:-python3}"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPython_EXECUTABLE="$(command -v "$PYTHON")"
cmake --build build -j"$(nproc)"
ls build/booster_lowlevel.cpython-*.so
