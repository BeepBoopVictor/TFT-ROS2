#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-tfg_panda_ws_dev}"
docker exec -it "${CONTAINER_NAME}" bash
