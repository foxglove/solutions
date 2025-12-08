# Ouster Cartesian Converter

A [Foxglove](https://foxglove.dev) extension that converts Ouster lidar scan data from polar coordinates to Cartesian coordinates, making it compatible with Foxglove's 3D panel.

[Foxglove](https://foxglove.dev) allows developers to create [extensions](https://docs.foxglove.dev/docs/visualization/extensions/introduction), or custom code that is loaded and executed inside the Foxglove application. This extension implements a [topic converter](https://docs.foxglove.dev/docs/extensions/guides/create-topic-converter) to transform lidar data in real-time.

## Overview

This extension automatically converts messages from `/ouster/lidar_scan_raw` (polar coordinate data) to `/ouster/pointcloud/cartesian` (Cartesian coordinate point cloud) that can be visualized in Foxglove's 3D panel.

## Installation

### Prerequisites

- Node.js 16+ and npm
- [Foxglove Desktop](https://foxglove.dev/download) application

### Setup

1. Install dependencies:
   ```sh
   npm install
   ```

2. Build and install the extension:
   ```sh
   npm run build
   npm run local-install
   ```

3. Open Foxglove Desktop (or refresh with `Ctrl+R` / `Cmd+R`)

The extension will automatically be available and will convert messages when you load a compatible MCAP file.

## Usage

1. Open your converted `.mcap` file in Foxglove (see `osf-to-mcap/README.md` for conversion instructions)
2. Add the **3D** panel
3. Subscribe to the `/ouster/pointcloud/cartesian` topic
4. The point cloud will be displayed in Cartesian coordinates

## Technical Details

### Topic Converter Implementation

The extension implements a topic converter that:

1. **Input Topic**: `/ouster/lidar_scan_raw`
   - Message format: `ouster_lidar_scan_raw_v1` schema
   - Contains: `timestamp_ns`, `fields` (RANGE, REFLECTIVITY, NEAR_IR)
   - Data is base64-encoded numpy arrays

2. **Output Topic**: `/ouster/pointcloud/cartesian`
   - Message format: `foxglove.PointCloud` schema
   - Contains: Cartesian coordinates (x, y, z) and intensity values
   - Compatible with Foxglove's 3D panel

### Conversion Process

The conversion follows these steps:

1. **Decode Base64 Data**: Decodes base64-encoded RANGE and REFLECTIVITY arrays
2. **Lookup Table**: Uses pre-computed lookup table for Ouster OS1-128 beam angles
   - Maps pixel index (row × columns + col) to unit direction vector
   - Calculated from beam altitude and azimuth angles
3. **Polar to Cartesian**: For each pixel:
   - Get range value from RANGE array
   - Get unit direction vector from lookup table
   - Calculate: `(x, y, z) = range × direction_vector`
   - Convert range from millimeters to meters
4. **Intensity Mapping**: Uses REFLECTIVITY values normalized to 0-1 range
5. **Point Cloud Format**: Packs data into `foxglove.PointCloud` format:
   - Fields: x, y, z, intensity (all FLOAT32)
   - Point stride: 16 bytes (4 bytes per field)

### Sensor Configuration

Currently configured for **Ouster OS1-128**:
- `PIXELS_PER_COLUMN`: 128 beams (rows)
- `COLUMNS_PER_FRAME`: 1024 azimuth samples (columns)
- `BEAM_ALTITUDE_ANGLES`: 128 elevation angles (degrees, converted to radians)
- `BEAM_AZIMUTH_ANGLES`: 128 azimuth offsets (degrees, converted to radians)

### Coordinate System

The conversion uses the following coordinate system:
- **x**: `cos(elevation) × cos(azimuth)`
- **y**: `-cos(elevation) × sin(azimuth)` (negative for proper orientation)
- **z**: `sin(elevation)`

Range values are converted from millimeters to meters before multiplication.

## Customization

### Supporting Different Sensor Models

To support different Ouster sensor models, modify the constants in `src/index.ts`:

1. **Update Beam Angles**:
   ```typescript
   const BEAM_ALTITUDE_ANGLES = [
     // Your sensor's elevation angles in degrees
   ].map((angle) => (angle * Math.PI) / 180);
   
   const BEAM_AZIMUTH_ANGLES = [
     // Your sensor's azimuth offsets in degrees
   ].map((angle) => (angle * Math.PI) / 180);
   ```

2. **Update Frame Dimensions**:
   ```typescript
   const PIXELS_PER_COLUMN = 64; // Number of beams for your sensor
   const COLUMNS_PER_FRAME = 2048; // Number of azimuth samples
   ```

3. **Rebuild the extension**:
   ```sh
   npm run build
   npm run local-install
   ```

### Supporting Multiple Data Streams

To support multiple lidar streams:

1. **Register Multiple Converters**:
   ```typescript
   extensionContext.registerMessageConverter({
     inputTopics: ["/ouster/lidar_scan_raw_1"],
     outputTopic: "/ouster/pointcloud/cartesian_1",
     // ...
   });
   
   extensionContext.registerMessageConverter({
     inputTopics: ["/ouster/lidar_scan_raw_2"],
     outputTopic: "/ouster/pointcloud/cartesian_2",
     // ...
   });
   ```

2. **Different Sensor Configurations**: Create separate lookup tables for different sensor models if needed.

### Modifying Field Processing

To use different fields or add additional processing:

1. **Decode Additional Fields**: Modify `convertToCartesian` to decode NEAR_IR or other fields
2. **Custom Intensity Mapping**: Change how intensity is calculated from reflectivity
3. **Filtering**: Add range or intensity filters to skip invalid points

## Development

Extension development uses the `npm` package manager to install development dependencies and run build scripts.

### Build

```sh
npm run build
```

### Local Install

```sh
npm run local-install
```

This installs the extension into your local Foxglove desktop app. Open Foxglove Desktop (or `ctrl-R` to refresh if it is already open).

### Linting

```sh
npm run lint
npm run lint:fix
```

## Package

Extensions are packaged into `.foxe` files. These files contain the metadata (package.json) and the build code for the extension.

Before packaging, make sure to set `name`, `publisher`, `version`, and `description` fields in _package.json_. When ready to distribute the extension, run:

```sh
npm run package
```

This command will package the extension into a `.foxe` file in the local directory.

## Publish

You can publish the extension to the public registry or privately for your organization.

See documentation here: https://docs.foxglove.dev/docs/visualization/extensions/publish/#packaging-your-extension

## References

- [Foxglove Extensions Documentation](https://docs.foxglove.dev/docs/visualization/extensions/introduction)
- [Create Topic Converter Guide](https://docs.foxglove.dev/docs/extensions/guides/create-topic-converter)
- [Ouster SDK Documentation](https://static.ouster.dev/sdk-docs/)
