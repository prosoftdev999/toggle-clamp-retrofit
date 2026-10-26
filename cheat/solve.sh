#!/usr/bin/env bash
set -euo pipefail
mkdir -p /app/output
cp "$(dirname "$0")/design.json" /app/output/design.json
