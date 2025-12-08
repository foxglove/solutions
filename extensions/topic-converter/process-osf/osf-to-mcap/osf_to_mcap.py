import base64, json, math
import numpy as np
from ouster.sdk import osf
from mcap.writer import Writer

OSF_PATH = "Traffic_Intersection.osf"
MCAP_PATH = "Traffic_Intersection_raw.mcap"
TOPIC = "/ouster/lidar_scan_raw"

def b64(arr: np.ndarray) -> str:
    return base64.b64encode(arr.tobytes(order="C")).decode("ascii")

src = osf.OsfScanSource(OSF_PATH)

with open(MCAP_PATH, "wb") as f:
    w = Writer(f)
    w.start()
    
    schema_id = w.register_schema(
        name="ouster_lidar_scan_raw_v1",
        encoding="jsonschema",
        data=json.dumps({
            "type": "object",
            "properties": {
                "timestamp_ns": {"type": "integer"},
                "fields": {
                    "type": "object",
                    "properties": {
                        "NEAR_IR": {
                            "type": "object",
                            "properties": {
                                "dtype": {"type": "string"},
                                "shape": {"type": "array", "items": {"type": "integer"}},
                                "data_b64": {"type": "string"}
                            },
                            "required": ["dtype", "shape", "data_b64"]
                        },
                        "RANGE": {
                            "type": "object",
                            "properties": {
                                "dtype": {"type": "string"},
                                "shape": {"type": "array", "items": {"type": "integer"}},
                                "data_b64": {"type": "string"}
                            },
                            "required": ["dtype", "shape", "data_b64"]
                        },
                        "REFLECTIVITY": {
                            "type": "object",
                            "properties": {
                                "dtype": {"type": "string"},
                                "shape": {"type": "array", "items": {"type": "integer"}},
                                "data_b64": {"type": "string"}
                            },
                            "required": ["dtype", "shape", "data_b64"]
                        }
                    },
                    "required": ["NEAR_IR", "RANGE", "REFLECTIVITY"]
                }
            },
            "required": ["timestamp_ns", "fields"]
        }).encode("utf-8"),
    )
    
    channel_id = w.register_channel(
        schema_id=schema_id,
        topic=TOPIC,
        message_encoding="json",
    )
    
    base_ns = None
    last_ns = 0
    written = 0
    skipped = 0
    
    for item in src:
        scan = item[0] if isinstance(item, (list, tuple)) else item
        ts_val = float(scan.timestamp[0])
        
        # skip invalid / sentinel timestamps
        if ts_val == 0 or math.isnan(ts_val) or math.isinf(ts_val):
            skipped += 1
            continue
        
        # Check if timestamp is in seconds or nanoseconds
        # If > 1e12, it's likely already in nanoseconds
        if ts_val > 1e12:
            ts_ns = int(ts_val)
        else:
            ts_ns = int(ts_val * 1e9)
        
        if base_ns is None:
            base_ns = ts_ns
            print(f"Base timestamp: {base_ns} ns ({base_ns/1e9:.6f} seconds)")
        
        rel_ns = ts_ns - base_ns
        
        # Add bounds checking
        if rel_ns < 0:
            print(f"Warning: negative timestamp {rel_ns}, skipping")
            skipped += 1
            continue
        
        # guard against backward jumps
        if rel_ns < last_ns:
            print(f"Warning: backward jump from {last_ns} to {rel_ns}, skipping")
            skipped += 1
            continue
        
        # Debug output for first few messages and periodically
        if written < 3 or written % 100 == 0:
            print(f"Message {written}: ts_val={ts_val:.0f}, ts_ns={ts_ns}, rel_ns={rel_ns}")
        
        msg = {
            "timestamp_ns": rel_ns,
            "fields": {},
        }
        
        for name in ["NEAR_IR", "RANGE", "REFLECTIVITY"]:
            arr = scan.field(name)
            msg["fields"][name] = {
                "dtype": str(arr.dtype),
                "shape": list(arr.shape),
                "data_b64": b64(arr),
            }
        
        w.add_message(
            channel_id=channel_id,
            log_time=rel_ns,
            publish_time=rel_ns,
            data=json.dumps(msg).encode("utf-8"),
        )
        
        last_ns = rel_ns
        written += 1
    
    w.finish()

print(f"Wrote {written} messages, skipped {skipped}")
print("Output:", MCAP_PATH)
