# Device Reporting

This utility generates a CSV report of all devices and their associated recordings in your Foxglove Data Platform.

## Features
- Lists all devices and their recordings
- Calculates total storage used per device
- Handles recordings not associated with any device
- Outputs results to a CSV file

## Usage

1. Set up environment variables in `.env`: 

```
FOXGLOVE_API_KEY=your_api_key_here
```

2. Run the script:  

```
python device_reporting.py
```
