# OSF to MCAP in Foxglove

A reference implementation for visualizing Ouster OSF (Open Sensor Format) lidar data in [Foxglove](https://app.foxglove.dev/), including conversion tools and coordinate transformation extensions.

![Example Result](example_result.gif)

## Problem Statement

[Foxglove](https://foxglove.dev) does not natively support OSF file format yet ([reference](https://docs.foxglove.dev/docs/getting-started/custom)). Additionally, Foxglove's 3D panel only supports Cartesian coordinate systems (xyz), while Ouster lidar data is stored in polar coordinates.

This repository provides a two-part solution:
1. **Data Import**: Convert OSF files to MCAP format for Foxglove compatibility
2. **Data Transformation**: Convert polar coordinates to Cartesian coordinates for 3D visualization

## Solution Overview

### OSF to MCAP Conversion ✅
Convert `.osf` files into `.mcap` format using Python scripts, allowing direct import into Foxglove.

### Coordinate Transformation ✅
A Foxglove extension that converts Ouster lidar scan data from polar coordinates to Cartesian coordinates, making it compatible with Foxglove's 3D panel.

## Project Structure

```
.
├── example-data_122250001535/    # Test data from studio.ouster.com
│   ├── Traffic_Intersection.osf   # Original OSF file
│   └── Traffic_Intersection_raw.mcap  # Converted MCAP file
│
├── osf-to-mcap/                   # Python conversion scripts
│   ├── osf_to_mcap.py            # Main conversion script
│   ├── scan_osf.py               # OSF file scanner
│   ├── list_mcap_topic.py        # MCAP topic lister
│   ├── verify_message_osf.py     # OSF message verification
│   └── verify_message_mcap.py    # MCAP message verification
│
└── ouster-cartesian-converter/    # Foxglove extension
    ├── src/
    │   └── index.ts              # Topic converter implementation
    └── package.json              # Extension configuration
```

## Prerequisites

### For OSF to MCAP Conversion
- Python 3.7+
- Required Python packages:
  ```bash
  pip install ouster-sdk mcap numpy
  ```

### For Foxglove Extension
- Node.js 16+ and npm
- [Foxglove Desktop](https://foxglove.dev/download) application

## Installation

### 1. OSF to MCAP Conversion Scripts

```bash
cd osf-to-mcap
pip install ouster-sdk mcap numpy
```

### 2. Foxglove Extension

```bash
cd ouster-cartesian-converter
npm install
```

## Usage

### Step 1: Convert OSF to MCAP

1. Place your `.osf` file in the `osf-to-mcap` directory (or update the path in `osf_to_mcap.py`)
2. Update the file paths in `osf_to_mcap.py`:
   ```python
   OSF_PATH = "path/to/your/file.osf"
   MCAP_PATH = "output.mcap"
   ```
3. Run the conversion:
   ```bash
   cd osf-to-mcap
   python osf_to_mcap.py
   ```

The script will:
- Read lidar scans from the OSF file
- Extract RANGE, REFLECTIVITY, and NEAR_IR fields
- Encode data as base64 JSON messages
- Write to MCAP format with proper timestamps
- Output conversion statistics

> [!NOTE]
> We observe a 20% file size increase when converting from OSF to MCAP. Further compression might be explored to reduce the MCAP file size.

### Step 2: Install the Topic Converter Extension

1. Build and install the extension:
   ```bash
   cd ouster-cartesian-converter
   npm run build
   npm run local-install
   ```

2. Open Foxglove Desktop (or refresh with `Ctrl+R` / `Cmd+R`)

3. The extension will automatically convert messages from `/ouster/lidar_scan_raw` to `/ouster/pointcloud/cartesian`

### Step 3: Visualize in Foxglove

1. Open your converted `.mcap` file in Foxglove
2. Add the **3D** panel
3. Subscribe to the `/ouster/pointcloud/cartesian` topic
4. The point cloud will be displayed in Cartesian coordinates

## Technical Details

For detailed technical information, see the README files in each component:

- **OSF to MCAP Conversion**: See [`osf-to-mcap/README.md`](osf-to-mcap/README.md) for conversion process, message schema, and customization options
- **Topic Converter Extension**: See [`ouster-cartesian-converter/README.md`](ouster-cartesian-converter/README.md) for conversion algorithm, sensor configuration, and extension customization

## Future Work

- [ ] Create a native OSF data loader for Foxglove
- [ ] Support for additional Ouster sensor models
- [ ] Support for multiple concurrent data streams
- [ ] Performance optimizations for large datasets

## Example Data

> [!IMPORTANT]
> Actual folder and data is not uploaded to the repo because file size exceeds 25 MB limit.

The `example-data_122250001535` folder contains sample data from [Ouster Studio's public sample library](https://studio.ouster.com/public/sample_library/public_collection), including:
- Original OSF file
- Converted MCAP file
- Related metadata files

## References

- [Foxglove Extensions Documentation](https://docs.foxglove.dev/docs/visualization/extensions/introduction)
- [Create Topic Converter Guide](https://docs.foxglove.dev/docs/extensions/guides/create-topic-converter)
- [Create Data Loader Guide](https://docs.foxglove.dev/docs/extensions/guides/create-data-loader)
- [Ouster SDK Documentation](https://static.ouster.dev/sdk-docs/)
