#!/usr/bin/env bash
# Start all three local test servers in the background.
# Usage: ./start-test-infra.sh
# Stop:  ./stop-test-infra.sh  (or kill the PIDs printed below)

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Starting test infrastructure..."
echo ""

# Mock IdP (port 4000)
node "$SCRIPT_DIR/mock-idp/server.js" &
IDP_PID=$!
echo "  Mock IdP         PID=$IDP_PID  http://localhost:4000"

# Callback page server (port 4001)
node "$SCRIPT_DIR/callback-page/serve.js" &
CALLBACK_PID=$!
echo "  Callback page     PID=$CALLBACK_PID  http://localhost:4001"

# Mock backend (port 4002)
node "$SCRIPT_DIR/mock-backend/server.js" &
BACKEND_PID=$!
echo "  Mock backend      PID=$BACKEND_PID  http://localhost:4002"

echo ""
echo "All servers started. PIDs: $IDP_PID $CALLBACK_PID $BACKEND_PID"
echo "Press Ctrl+C to stop all."

# Write PIDs to file for stop script
echo "$IDP_PID $CALLBACK_PID $BACKEND_PID" > "$SCRIPT_DIR/.server-pids"

# Wait for all background jobs; if one exits, kill the rest
trap "kill $IDP_PID $CALLBACK_PID $BACKEND_PID 2>/dev/null; rm -f $SCRIPT_DIR/.server-pids" EXIT
wait
