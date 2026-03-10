#!/usr/bin/env bash
# Stop the test infrastructure servers started by start-test-infra.sh
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PID_FILE="$SCRIPT_DIR/.server-pids"

if [ -f "$PID_FILE" ]; then
  PIDS=$(cat "$PID_FILE")
  echo "Stopping servers: $PIDS"
  kill $PIDS 2>/dev/null || true
  rm -f "$PID_FILE"
  echo "Done."
else
  echo "No PID file found. Trying to kill by port..."
  for port in 4000 4001 4002; do
    pid=$(lsof -ti:$port 2>/dev/null || true)
    if [ -n "$pid" ]; then
      echo "  Killing PID $pid on port $port"
      kill "$pid" 2>/dev/null || true
    fi
  done
fi
