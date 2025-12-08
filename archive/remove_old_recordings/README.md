# Remove Old Recordings

This utility helps manage storage by automatically removing old recordings from your Foxglove Data Platform.

## Features
- Deletes recordings older than a specified number of days
- Supports limiting the number of deletions per run
- Provides detailed logging of deleted recordings
- Handles recordings with and without device associations

## Usage

1. Set up environment variables in `.env`:
        
```
FOXGLOVE_API_KEY=your_api_key_here
DAYS_TO_KEEP=30     # Keep recordings newer than this many days
DELETE_LIMIT=100    # Maximum number of recordings to delete (0 for unlimited)
```

2. Run the script:

```
python clear_old_records.py
```

## Safety Features
- Requires explicit configuration through environment variables
- Shows preview of recordings to be deleted
- Optional deletion limit to prevent mass deletions
- Displays device information for each deleted recording

## Example Output
```
2024-02-22 19:42:04.296000+00:00
Deleting recording rec_0dJeY6aLoQp4ZoPB from 2024-11-22 19:42:04.296000+00:00 on device Moorebot
``` 