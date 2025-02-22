import os
import csv
from dotenv import load_dotenv
load_dotenv()

from foxglove_data_platform.client import Client

def get_all_recordings(client, batch_size=2000):
    """
    Fetch all recordings using pagination.

    Args:
        client: Foxglove API client instance
        batch_size: Number of recordings to fetch per request (default: 1000)

    Returns:
        List of all recordings
    """
    all_recordings = []
    offset = 0

    while True:
        batch = client.get_recordings(limit=batch_size, offset=offset)
        if not batch:  # Stop if no more recordings
            break

        all_recordings.extend(batch)
        offset += batch_size

    return all_recordings

# Initialize client
token = os.getenv("FOXGLOVE_API_KEY")
client = Client(token=token)

# Get all recordings and devices
recordings = get_all_recordings(client)
devices = client.get_devices()

# Build the report for devices and recordings per device
report = {
    "devices": [{
        "name": device["name"],
        "id": device["id"],
        "recordings": [],
        "total_size": 0
    } for device in devices]
}
report["devices"].append({
    "name": "No Device",
    "id": "no_device",
    "recordings": [],
    "total_size": 0
})

for recording in recordings:
    if(recording["device"] is None):
        for device in report["devices"]:
            if(device["id"] == "no_device"):
                device["recordings"].append(recording["id"])
                device["total_size"] += recording["size"]
    else:
        for device in report["devices"]:
            if(device["id"] == recording["device"]["id"]):
                device["recordings"].append(recording["id"])
                device["total_size"] += recording["size"]

with open("report.csv", "w") as f:
    writer = csv.writer(f)
    writer.writerow(["Device", "Recording Count", "Total Size"])
    for device in report["devices"]:
        writer.writerow([device["name"], len(device["recordings"]), device["total_size"]])
