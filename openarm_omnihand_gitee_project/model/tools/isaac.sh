#!/bin/bash
set -e
p=$(cd "$(dirname "$0")/.." && pwd)
isaac_runtime=${ISAAC_SIM_PATH:-/isaac-sim}
if [[ ! -x "$isaac_runtime/python.sh" ]]; then echo 'Set ISAAC_SIM_PATH to an Isaac Sim installation (tested 6.0.1).'; exit 2; fi
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
if [[ $(id -u) == 0 ]]; then export OMNI_KIT_ALLOW_ROOT=1; fi
exec "$isaac_runtime/python.sh" "$p/tools/no_can.py" "$isaac_runtime/kit/python/bin/python3" "$p/tools/isaac_demo.py" "$@"
