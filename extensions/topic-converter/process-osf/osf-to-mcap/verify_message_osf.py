import json, base64, hashlib
from mcap.reader import make_reader

def hash_range(msg_data):
    d = json.loads(msg_data)
    b = base64.b64decode(d["fields"]["RANGE"]["data_b64"])
    return hashlib.md5(b).hexdigest()

path = "Traffic_Intersection_raw.mcap"
with open(path, "rb") as f:
    r = make_reader(f)
    hashes = []
    times = []
    for _, _, m in r.iter_messages(topics=["/ouster/lidar_scan_raw"]):
        hashes.append(hash_range(m.data))
        times.append(m.log_time)
        if len(hashes) >= 20:
            break

print("unique RANGE hashes in first 20 frames:", len(set(hashes)))
print("log time deltas (ns):", [times[i+1]-times[i] for i in range(len(times)-1)])
