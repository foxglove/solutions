import { ExtensionContext, Immutable, MessageEvent, Time } from "@foxglove/extension";

// Types for the Ouster lidar scan JSON schema
type FieldData = {
  dtype: string;
  shape: readonly number[];
  data_b64: string;
};

type OusterLidarScanRaw = {
  timestamp_ns: number;
  fields: {
    NEAR_IR: FieldData;
    RANGE: FieldData;
    REFLECTIVITY: FieldData;
  };
};

// Type for the lookup table - maps (row, col) index to cartesian coordinates
// Index is calculated as: row * columns_per_frame + col
type LookupTable = Map<number, { x: number; y: number; z: number }>;

// Ouster OS1-128 beam angles (from sensor metadata)
// These are the elevation angles for each of the 128 beams
// If your sensor is different, update these values
const BEAM_ALTITUDE_ANGLES = [
  20.44, 20.16, 19.85, 19.48, 19.21, 18.93, 18.61, 18.25, 17.96, 17.69, 17.36, 17, 16.69, 16.41,
  16.08, 15.72, 15.41, 15.12, 14.8, 14.42, 14.12, 13.81, 13.49, 13.11, 12.79, 12.49, 12.15, 11.79,
  11.46, 11.15, 10.81, 10.45, 10.12, 9.8, 9.46, 9.1, 8.77, 8.44, 8.09, 7.74, 7.41, 7.06, 6.71,
  6.36, 6.03, 5.69, 5.34, 4.97, 4.63, 4.3, 3.96, 3.58, 3.24, 2.91, 2.55, 2.19, 1.84, 1.5, 1.15,
  0.8, 0.45, 0.1, -0.26, -0.6, -0.94, -1.29, -1.66, -1.99, -2.34, -2.7, -3.05, -3.39, -3.74, -4.1,
  -4.45, -4.79, -5.12, -5.49, -5.85, -6.19, -6.51, -6.87, -7.22, -7.56, -7.89, -8.25, -8.6, -8.93,
  -9.27, -9.62, -9.98, -10.29, -10.62, -10.99, -11.33, -11.66, -11.98, -12.33, -12.67, -12.99,
  -13.31, -13.67, -14.01, -14.31, -14.62, -14.99, -15.33, -15.62, -15.93, -16.3, -16.61, -16.9,
  -17.2, -17.58, -17.89, -18.17, -18.48, -18.84, -19.16, -19.43, -19.71, -20.08, -20.39, -20.66,
  -20.95, -21.31, -21.61, -21.87,
].map((angle) => (angle * Math.PI) / 180); // Convert to radians

const BEAM_AZIMUTH_ANGLES = [
  4.15, 1.34, -1.45, -4.23, 4.16, 1.36, -1.44, -4.22, 4.16, 1.36, -1.44, -4.22, 4.16, 1.36, -1.44,
  -4.21, 4.16, 1.37, -1.43, -4.22, 4.17, 1.38, -1.43, -4.21, 4.17, 1.37, -1.43, -4.21, 4.17, 1.38,
  -1.43, -4.22, 4.17, 1.37, -1.42, -4.22, 4.16, 1.38, -1.42, -4.21, 4.17, 1.38, -1.42, -4.22,
  4.17, 1.38, -1.42, -4.22, 4.18, 1.39, -1.41, -4.21, 4.17, 1.38, -1.41, -4.22, 4.18, 1.39, -1.42,
  -4.21, 4.19, 1.38, -1.42, -4.2, 4.19, 1.4, -1.42, -4.21, 4.2, 1.4, -1.41, -4.2, 4.2, 1.39,
  -1.4, -4.19, 4.2, 1.41, -1.4, -4.2, 4.19, 1.41, -1.39, -4.19, 4.21, 1.42, -1.4, -4.19, 4.22,
  1.42, -1.39, -4.18, 4.23, 1.42, -1.37, -4.19, 4.22, 1.43, -1.37, -4.18, 4.22, 1.43, -1.36,
  -4.17, 4.24, 1.44, -1.36, -4.16, 4.24, 1.43, -1.36, -4.16, 4.24, 1.44, -1.35, -4.15, 4.25, 1.44,
  -1.36, -4.14, 4.26, 1.45, -1.36, -4.15, 4.24, 1.45, -1.36, -4.16,
].map((angle) => (angle * Math.PI) / 180); // Convert to radians

const PIXELS_PER_COLUMN = 128; // Number of beams/rows
const COLUMNS_PER_FRAME = 1024; // Number of azimuth samples/columns

// Generate lookup table from beam angles
// This creates a lookup table that maps pixel index (row * columns + col) to unit direction vector
function generateLookupTable(): LookupTable {
  const lut = new Map<number, { x: number; y: number; z: number }>();

  for (let row = 0; row < PIXELS_PER_COLUMN; row++) {
    const elevation = BEAM_ALTITUDE_ANGLES[row]!;
    const beamAzimuthOffset = BEAM_AZIMUTH_ANGLES[row]!;

    for (let col = 0; col < COLUMNS_PER_FRAME; col++) {
      // Calculate azimuth angle: 2π * (col / columns_per_frame) - π
      // Then add the beam-specific azimuth offset
      const azimuth = (2 * Math.PI * col) / COLUMNS_PER_FRAME - Math.PI + beamAzimuthOffset;

      // Calculate unit direction vector (normalized)
      // x = cos(elevation) * cos(azimuth)
      // y = cos(elevation) * sin(azimuth)
      // z = sin(elevation)
      const x = Math.cos(elevation) * Math.cos(azimuth);
      const y = - Math.cos(elevation) * Math.sin(azimuth);
      const z = Math.sin(elevation);

      const index = row * COLUMNS_PER_FRAME + col;
      lut.set(index, { x, y, z });
    }
  }

  return lut;
}

export function activate(extensionContext: ExtensionContext): void {
  // Generate the lookup table once at startup
  const lookupTable = generateLookupTable();

  // Register the topic converter
  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: ["/ouster/lidar_scan_raw"],
    outputTopic: "/ouster/pointcloud/cartesian",
    outputSchemaName: "foxglove.PointCloud",
    // No outputSchemaDescription needed - using official foxglove.PointCloud schema
    create: () => {
      // This function will be called for each input message in log time order
      return (msgEvent: MessageEvent<unknown>) => {
        const message = msgEvent.message as Immutable<OusterLidarScanRaw>;
        const eventWithLogTime = msgEvent as MessageEvent<unknown> & { logTime?: Time };
        return convertToCartesian(message, lookupTable, eventWithLogTime.logTime ?? msgEvent.receiveTime);
      };
    },
  });
}

/**
 * Decodes base64 string to Uint8Array
 * Uses atob if available (browser/Node-like environment), otherwise manual implementation
 */
function base64ToBytes(base64: string): Uint8Array {
  // Use atob if available (most common case in browser/Node-like environments)
  if (typeof atob !== "undefined") {
    const binaryString = atob(base64);
    const bytes = new Uint8Array(binaryString.length);
    for (let i = 0; i < binaryString.length; i++) {
      bytes[i] = binaryString.charCodeAt(i);
    }
    return bytes;
  }
  
  // Manual base64 decode as fallback
  const chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  let bufferLength = (base64.length * 3) / 4;
  if (base64[base64.length - 1] === "=") {
    bufferLength--;
    if (base64[base64.length - 2] === "=") {
      bufferLength--;
    }
  }
  
  const bytes = new Uint8Array(bufferLength);
  let p = 0;
  for (let i = 0; i < base64.length; i += 4) {
    const encoded1 = chars.indexOf(base64[i]!);
    const encoded2 = chars.indexOf(base64[i + 1]!);
    const encoded3 = chars.indexOf(base64[i + 2]!);
    const encoded4 = chars.indexOf(base64[i + 3]!);
    
    bytes[p++] = (encoded1 << 2) | (encoded2 >> 4);
    if (encoded3 !== -1) {
      bytes[p++] = ((encoded2 & 15) << 4) | (encoded3 >> 2);
    }
    if (encoded4 !== -1) {
      bytes[p++] = ((encoded3 & 3) << 6) | encoded4;
    }
  }
  return bytes;
}

/**
 * Decodes base64-encoded data based on dtype
 */
function decodeFieldData(field: FieldData): number[] {
  const bytes = base64ToBytes(field.data_b64);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);

  const height = field.shape[0];
  const width = field.shape[1];
  if (height === undefined || width === undefined) {
    throw new Error(`Invalid shape: ${field.shape}`);
  }
  const totalElements = height * width;

  switch (field.dtype) {
    case "uint16":
    case "<u2": {
      const result: number[] = [];
      for (let i = 0; i < totalElements; i++) {
        result.push(view.getUint16(i * 2, true)); // little-endian
      }
      return result;
    }
    case "uint32":
    case "<u4": {
      const result: number[] = [];
      for (let i = 0; i < totalElements; i++) {
        result.push(view.getUint32(i * 4, true)); // little-endian
      }
      return result;
    }
    case "float32":
    case "<f4": {
      const result: number[] = [];
      for (let i = 0; i < totalElements; i++) {
        result.push(view.getFloat32(i * 4, true)); // little-endian
      }
      return result;
    }
    case "uint8":
    case "|u1": {
      return Array.from(bytes);
    }
    default:
      throw new Error(`Unsupported dtype: ${field.dtype}`);
  }
}

/**
 * Converts a pointcloud from polar coordinates to cartesian coordinates
 * using the lookup table.
 * 
 * The conversion process:
 * 1. Decode base64-encoded RANGE and REFLECTIVITY data
 * 2. For each pixel (row, col), get the range value
 * 3. Use the lookup table to get the unit direction vector
 * 4. Multiply range by direction vector to get cartesian coordinates
 * 5. Use REFLECTIVITY as intensity
 * 6. Pack everything into foxglove.PointCloud format
 */
function convertToCartesian(
  scan: OusterLidarScanRaw,
  lookupTable: LookupTable,
  stamp: Time,
): {
  timestamp: Time;
  frame_id: string;
  pose: { position: { x: number; y: number; z: number }; orientation: { x: number; y: number; z: number; w: number } };
  point_stride: number;
  fields: Array<{ name: string; offset: number; type: number }>;
  data: Uint8Array;
} {
  // Decode the field data
  const ranges = decodeFieldData(scan.fields.RANGE);
  const reflectivities = decodeFieldData(scan.fields.REFLECTIVITY);

  const height = scan.fields.RANGE.shape[0];
  const width = scan.fields.RANGE.shape[1];
  if (height === undefined || width === undefined) {
    throw new Error(`Invalid shape: ${scan.fields.RANGE.shape}`);
  }

  const points: Array<{ x: number; y: number; z: number; intensity: number }> = [];

  // Convert each pixel from polar to cartesian
  for (let row = 0; row < height; row++) {
    for (let col = 0; col < width; col++) {
      const index = row * width + col;
      const range = ranges[index];
      const reflectivity = reflectivities[index];

      // Skip invalid ranges (typically 0 or very large values)
      if (range === undefined || range === 0 || range > 100000) {
        continue;
      }

      // Get unit direction vector from lookup table
      const direction = lookupTable.get(index);
      if (!direction) {
        continue;
      }

      // Convert range from millimeters to meters
      const rangeMeters = range * 0.001; // mm -> m

      // Convert to cartesian: multiply range by unit direction vector
      const x = rangeMeters * direction.x;
      const y = rangeMeters * direction.y;
      const z = rangeMeters * direction.z;

      // Use reflectivity as intensity (normalize to 0-1 range)
      // Reflectivity is uint16 (0-65535), so normalize it
      const intensity = reflectivity !== undefined ? reflectivity / 65535.0 : 0;

      points.push({ x, y, z, intensity });
    }
  }

  // Pack the data according to foxglove.PointCloud format
  // Fields: x (FLOAT32), y (FLOAT32), z (FLOAT32), intensity (FLOAT32)
  // Each field is 4 bytes, so point_stride = 16 bytes
  const pointStride = 16;
  const data = new ArrayBuffer(points.length * pointStride);
  const view = new DataView(data);

  let offset = 0;
  for (const point of points) {
    view.setFloat32(offset, point.x, true); // little-endian
    offset += 4;
    view.setFloat32(offset, point.y, true);
    offset += 4;
    view.setFloat32(offset, point.z, true);
    offset += 4;
    view.setFloat32(offset, point.intensity, true);
    offset += 4;
  }

  return {
    timestamp: stamp,
    frame_id: "ouster",
    pose: {
      position: { x: 0, y: 0, z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    },
    point_stride: pointStride,
    fields: [
      { name: "x", offset: 0, type: 7 }, // FLOAT32 = 7 (see foxglove.PackedElementField.NumericType)
      { name: "y", offset: 4, type: 7 },
      { name: "z", offset: 8, type: 7 },
      { name: "intensity", offset: 12, type: 7 },
    ],
    data: new Uint8Array(data),
  };
}
