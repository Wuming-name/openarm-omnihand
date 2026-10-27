#!/bin/bash
set -e
p=$(cd "$(dirname "$0")/.." && pwd)
exec python3 "$p/tools/package_model.py"
