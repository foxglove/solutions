#!/bin/bash
# Script to export all Docker Compose logs to local files

LOGS_DIR="./logs"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

mkdir -p "$LOGS_DIR"

echo "Exporting logs to $LOGS_DIR at $TIMESTAMP..."

# Export logs for each service
docker compose logs --no-color roscore > "$LOGS_DIR/roscore-$TIMESTAMP.log" 2>&1
docker compose logs --no-color chatter_publisher > "$LOGS_DIR/chatter_publisher-$TIMESTAMP.log" 2>&1
docker compose logs --no-color chatter_subscriber > "$LOGS_DIR/chatter_subscriber-$TIMESTAMP.log" 2>&1
docker compose logs --no-color foxglove_bridge > "$LOGS_DIR/foxglove_bridge-$TIMESTAMP.log" 2>&1
docker compose logs --no-color rosbag_recorder > "$LOGS_DIR/rosbag_recorder-$TIMESTAMP.log" 2>&1
docker compose logs --no-color foxglove_agent > "$LOGS_DIR/foxglove_agent-$TIMESTAMP.log" 2>&1

# Export all logs combined
docker compose logs --no-color > "$LOGS_DIR/all-services-$TIMESTAMP.log" 2>&1

echo "Logs exported successfully!"
echo "Individual service logs: $LOGS_DIR/*-$TIMESTAMP.log"
echo "Combined logs: $LOGS_DIR/all-services-$TIMESTAMP.log"
