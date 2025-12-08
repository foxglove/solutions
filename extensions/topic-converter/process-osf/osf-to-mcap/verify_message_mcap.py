import hashlib
from mcap.reader import make_reader

def hash_pc(msg_data):
    return hashlib.md5(msg_data).hexdigest()

path = "Traffic_Intersection_raw.mcap"
with open(path, "rb") as f:
    r = make_reader(f)
    hashes = []
    for _, _, m in r.iter_messages(topics=["/ouster/pointcloud/cartesian"]):
        hashes.append(hash_pc(m.data))
        if len(hashes) >= 20:
            break

print("unique pointcloud hashes in first 20 frames:", len(set(hashes)))
