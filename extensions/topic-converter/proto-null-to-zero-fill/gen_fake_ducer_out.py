#!/usr/bin/env python3
"""
Generate an MCAP file with GROUP_ID_PRIMSCATPActuatorSensorFrame topic
logging for 10 seconds at 120Hz (1200 messages total).
"""

import json
import random
import time
from datetime import datetime, timezone
from mcap.writer import Writer

# Topic configuration
TOPIC_NAME = "GROUP_ID_PRIMSCATPActuatorSensorFrame"
FREQUENCY = 120  # Hz
DURATION = 10  # seconds
TOTAL_MESSAGES = FREQUENCY * DURATION  # 1200 messages
NUM_SENSORS = 10

# Probability that a ducer_out_N will be null (no nested data)
NULL_PROBABILITY = 0.1  # 10% chance


def generate_sensor_data():
    """Generate sensor data for one message."""
    sensors = {}
    
    for i in range(1, NUM_SENSORS + 1):
        sensor_name = f"ducer_out_{i}"
        
        # Randomly decide if this sensor should be null
        if random.random() < NULL_PROBABILITY:
            sensors[sensor_name] = None
        else:
            sensors[sensor_name] = {
                "raw_value": random.randint(0, 32767),
                "value": 0,
                "sensor_status": 0
            }
    
    return {"sensors": sensors}


def create_mcap_file(output_path: str):
    """Create the MCAP file with the specified topic and messages."""
    
    # Define schema (using JSON schema for flexible nested structure)
    schema_json = {
        "type": "object",
        "properties": {
            "sensors": {
                "type": "object",
                "additionalProperties": {
                    "oneOf": [
                        {"type": "null"},
                        {
                            "type": "object",
                            "properties": {
                                "raw_value": {"type": "integer", "minimum": 0, "maximum": 32767},
                                "value": {"type": "integer"},
                                "sensor_status": {"type": "integer"}
                            },
                            "required": ["raw_value", "value", "sensor_status"]
                        }
                    ]
                }
            }
        },
        "required": ["sensors"]
    }
    
    # Calculate time between messages (in nanoseconds)
    time_between_messages_ns = int((1.0 / FREQUENCY) * 1e9)
    
    with open(output_path, "wb") as f:
        writer = Writer(f)
        writer.start()
        
        # Register schema - returns schema_id
        schema_id = writer.register_schema(
            name="GROUP_ID_PRIMSCATPActuatorSensorFrame",
            encoding="jsonschema",
            data=json.dumps(schema_json).encode("utf-8")
        )
        
        # Register channel - returns channel_id
        channel_id = writer.register_channel(
            schema_id=schema_id,
            topic=TOPIC_NAME,
            message_encoding="json",
            metadata={}
        )
        
        # Generate and write messages
        start_time_ns = int(time.time_ns())
        
        for i in range(TOTAL_MESSAGES):
            timestamp_ns = start_time_ns + (i * time_between_messages_ns)
            data = generate_sensor_data()
            message_data = json.dumps(data).encode("utf-8")
            
            # Write message directly using add_message
            writer.add_message(
                channel_id=channel_id,
                sequence=i,
                log_time=timestamp_ns,
                publish_time=timestamp_ns,
                data=message_data
            )
            
            if (i + 1) % 120 == 0:
                print(f"Generated {i + 1}/{TOTAL_MESSAGES} messages...")
        
        writer.finish()
    
    print(f"\nMCAP file created successfully: {output_path}")
    print(f"Total messages: {TOTAL_MESSAGES}")
    print(f"Duration: {DURATION} seconds")
    print(f"Frequency: {FREQUENCY} Hz")


if __name__ == "__main__":
    output_file = "GROUP_ID_PRIMSCATPActuatorSensorFrame.mcap"
    create_mcap_file(output_file)


