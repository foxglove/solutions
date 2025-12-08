# OSF to MCAP Conversion

Python3 scripts to convert Ouster OSF (Ouster Sensor Format) files to MCAP format for use in Foxglove.

## Overview

The main conversion script (`osf_to_mcap.py`) reads lidar scan data from OSF files and converts it to MCAP format, making it compatible with Foxglove Studio.

## Prerequisites

- Python3 3.7+
- Required packages:
  ```bash
  pip install ouster-sdk mcap numpy
  ```

## Usage

1. Place your `.osf` file in this directory (or update the path in `osf_to_mcap.py`)
2. Update the file paths in `osf_to_mcap.py`:
   ```python3
   OSF_PATH = "path/to/your/file.osf"
   MCAP_PATH = "output.mcap"
   ```
3. Run the conversion:
   ```bash
   python3 osf_to_mcap.py
   ```

The script will output conversion statistics including the number of messages written and skipped.

## Technical Details

### Conversion Process

The `osf_to_mcap.py` script performs the following operations:

1. **OSF File Reading**: Uses `ouster.sdk.osf.OsfScanSource` to read lidar scans from the OSF file
2. **Data Extraction**: Extracts three main fields from each scan:
   - `RANGE`: Distance measurements in millimeters
   - `REFLECTIVITY`: Reflectivity values (uint16)
   - `NEAR_IR`: Near-infrared intensity values
3. **Data Encoding**: Converts numpy arrays to base64-encoded strings for JSON serialization
4. **Timestamp Handling**:
   - Detects timestamp format (seconds vs nanoseconds)
   - Normalizes to relative timestamps (nanoseconds from first message)
   - Validates and skips invalid timestamps
   - Guards against backward timestamp jumps
5. **MCAP Writing**: Creates MCAP file with proper schema and channel registration

### Message Schema

The converted MCAP uses the `ouster_lidar_scan_raw_v1` schema:

```json
{
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
          }
        },
        "RANGE": { /* same structure */ },
        "REFLECTIVITY": { /* same structure */ }
      }
    }
  }
}
```

**Topic**: `/ouster/lidar_scan_raw`

**Message Encoding**: JSON

### Helper Scripts

- `scan_osf.py`: Scans and inspects OSF file structure
- `list_mcap_topic.py`: Lists topics in a converted MCAP file
- `verify_message_osf.py`: Verifies message integrity from OSF source
- `verify_message_mcap.py`: Verifies message integrity in MCAP format

## File Size Considerations

**Note**: The MCAP output file is typically larger than the original OSF file. This is due to:

1. **Base64 Encoding Overhead**: Binary data is base64-encoded for JSON serialization, which increases size by approximately 33%
2. **JSON Structure**: The JSON message format adds metadata (field names, schema information) that increases file size
3. **MCAP Container Format**: The MCAP format includes additional metadata and indexing structures

**Observed Size Increase**: In testing with a ~723 MB OSF file, the resulting MCAP file was ~870 MB (approximately 20% larger).

**Scaling Concern**: The exact scaling behavior of file size increase is not yet fully characterized. The size increase may vary based on:
- Number of messages/scan frames
- Data density per frame
- MCAP indexing and metadata overhead

For very large OSF files, consider:
- Monitoring disk space during conversion
- Using compression if supported by your MCAP reader
- Processing in chunks if memory becomes a concern

## Customization

### Supporting Different Data Structures

To support different OSF file structures or additional fields:

1. **Add New Fields**: Modify the field extraction loop in `osf_to_mcap.py`:
   ```python3
   for name in ["NEAR_IR", "RANGE", "REFLECTIVITY", "YOUR_FIELD"]:
       arr = scan.field(name)
       msg["fields"][name] = {
           "dtype": str(arr.dtype),
           "shape": list(arr.shape),
           "data_b64": b64(arr),
       }
   ```

2. **Update Schema**: Add the new field to the JSON schema definition in the `register_schema` call

3. **Multiple Streams**: For multiple lidar streams, create separate channels with different topic names:
   ```python3
   channel_id_1 = w.register_channel(schema_id=schema_id, topic="/ouster/lidar_scan_raw_1", ...)
   channel_id_2 = w.register_channel(schema_id=schema_id, topic="/ouster/lidar_scan_raw_2", ...)
   ```

### Timestamp Handling

The script includes robust timestamp handling:
- Automatically detects seconds vs nanoseconds
- Skips invalid timestamps (0, NaN, Inf)
- Prevents backward jumps
- Uses relative timestamps for better compatibility

To modify timestamp behavior, adjust the timestamp processing logic around lines 77-111 in `osf_to_mcap.py`.

## Troubleshooting

- **Import Errors**: Ensure all dependencies are installed: `pip install ouster-sdk mcap numpy`
- **File Not Found**: Check that the OSF file path is correct and the file exists
- **Memory Issues**: For very large OSF files, consider processing in chunks or using streaming
- **Timestamp Warnings**: The script will skip messages with invalid timestamps and report the count

