#!/bin/zsh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_PYTHON="$SCRIPT_DIR/.venv/bin/python"

if [ ! -x "$VENV_PYTHON" ]; then
  echo "Virtual environment not found. Run src/control_station/setup_env.sh first."
  exit 1
fi

"$VENV_PYTHON" "$SCRIPT_DIR/main.py"
