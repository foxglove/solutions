import os
import csv
from dotenv import load_dotenv
from datetime import datetime, timedelta, timezone
load_dotenv()

from foxglove_data_platform.client import Client

token = os.getenv("FOXGLOVE_API_KEY")
days_to_keep = os.getenv("DAYS_TO_KEEP")
delete_limit = os.getenv("DELETE_LIMIT")

latest_date = datetime.now(timezone.utc) - timedelta(days=int(days_to_keep))

if(token is None):
    raise ValueError("FOXGLOVE_API_KEY is not set")

if(days_to_keep is None):
    raise ValueError("DAYS_TO_KEEP is not set")

client = Client(token=token)

recordings = client.get_recordings()

# Delete a recording
def delete_recording(recording_id):
    client.delete_recording(recording_id=recording_id)

# List and delete recordings
delete_count = 0
for recording in recordings:
    if recording["created_at"] < latest_date:
        if recording["device"] is not None:
            device_id = recording["device"]["name"]
        else:
            device_id = "No Device"
        print(f"Deleting recording {recording['id']} from {recording['created_at']} on device {device_id}")
        delete_recording(recording["id"])
        delete_count += 1
        if delete_count >= int(delete_limit) and int(delete_limit) != 0:
            break

