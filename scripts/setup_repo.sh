#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

mkdir -p \
  data/datasets/raw \
  data/datasets/processed \
  data/datasets/manifests \
  data/checkpoints \
  data/logs \
  data/videos

touch data/checkpoints/.gitkeep data/logs/.gitkeep data/videos/.gitkeep

echo "Workspace preparado en ${ROOT_DIR}"
