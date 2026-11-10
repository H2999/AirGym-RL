#!/usr/bin/env bash
# Read-only initialization; source the existing activation script for GPU work.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
exec "${HARNESS_PYTHON:-python3}" tools/harness.py doctor
