#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_NAME="${LILI_WORKSPACE:-lili_workspace}"
HOME_DIR="${HOME:-/tmp}"
WORKSPACE_DIR="${HOME_DIR}/${WORKSPACE_NAME}"

if [[ ! -d "${WORKSPACE_DIR}" ]]; then
  echo "Error: workspace directory not found: ${WORKSPACE_DIR}" >&2
  echo "Set LILI_WORKSPACE to your workspace folder name under HOME." >&2
  exit 1
fi

echo "Building package 'lili' from: ${WORKSPACE_DIR}"
cd "${WORKSPACE_DIR}"

colcon build --symlink-install --packages-select lili
