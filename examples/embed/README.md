# Foxglove Embed Example

This example demonstrates how to embed Foxglove in your application using the [TypeScript SDK](https://docs.foxglove.dev/docs/embed/typescript-sdk).

## Features

This example application showcases:

- **Basic Setup**: Initialize the Foxglove viewer with organization configuration
- **File Loading**: Load local `.bag`, `.db3`, `.ulg`, `.ulog`, or `.mcap` files
- **Live Data Sources**: Connect to Foxglove WebSocket or ROS Bridge
- **Recordings**: Load recordings from your Foxglove organization
- **Remote Files**: Load files hosted on remote servers (with CORS enabled)
- **Device Connections**: Connect to devices by ID or name with time ranges
- **Layout Management**: Load, save, and manage layouts

## Installation

1. Install dependencies:

```bash
npm install
```

## Running the Example

Start the development server:

```bash
npm run dev
```

The application will open in your browser at `http://localhost:3000`.

## Usage

### Setting Organization

1. Enter your Foxglove organization slug in the "Org Slug" field
2. Click "Set Organization" to initialize the viewer with your organization

If you leave the organization slug empty, users will be able to select their organization when signing in.

### Loading Data

The example provides several buttons to load different types of data:

- **Load File**: Opens a file picker to select a local file (`.bag`, `.db3`, `.ulg`, `.ulog`, `.mcap`)
- **Connect Live (WebSocket)**: Connect to a live data source (Foxglove WebSocket or ROS Bridge)
- **Load Recording**: Load a recording by ID from your Foxglove organization
- **Load Remote File**: Load a file from a remote URL (requires CORS and range headers)
- **Connect to Device**: Connect to a device by ID or name with a time range

### Layout Management

- **Load Layout**: Load a layout from a JSON file
- **Save Layout**: Download the current layout as a JSON file

## Code Examples

### Basic Setup

```typescript
import { FoxgloveViewer } from "@foxglove/embed";

const viewer = new FoxgloveViewer({
  parent: document.getElementById("container")!,
  orgSlug: "my-org",
});
```

### Event Handling

```typescript
viewer.addEventListener("ready", () => {
  console.log("Viewer is ready");
});

viewer.addEventListener("error", (e) => {
  console.error("Error:", e.detail);
});
```

### Setting Data Sources

#### File
```typescript
viewer.setDataSource({
  type: "file",
  file: fileObject,
});
```

#### Live WebSocket
```typescript
viewer.setDataSource({
  type: "live",
  protocol: "foxglove-websocket",
  url: "ws://localhost:8765",
});
```

#### Recording
```typescript
viewer.setDataSource({
  type: "recording",
  recordingId: "rec_0dHVqSWhIQ8HUUJU",
  projectId: "prj_0dX15xqpCP0yPYNq",
});
```

#### Remote File
```typescript
viewer.setDataSource({
  type: "remote-file",
  urls: ["https://example.com/sample.mcap"],
});
```

#### Device
```typescript
viewer.setDataSource({
  type: "device",
  deviceId: "dev_0dHVqSWhIQ8HUUJU",
  start: "2025-01-01T00:00:00Z",
  end: "2025-02-01T00:00:00Z",
});
```

### Layout Management

```typescript
// Load a layout
viewer.selectLayout({
  storageKey: "layout-a",
  opaqueLayout: layoutData,
  force: true,
});

// Get current layout
const layoutData = await viewer.getLayout();
```

## Building for Production

To build the example for production:

```bash
npm run build
```

The built files will be in the `dist` directory.

## Troubleshooting

### Viewer Not Loading

- Make sure you have installed all dependencies with `npm install`
- Check the browser console for any error messages
- Verify that the `@foxglove/embed` package is installed correctly
- Ensure you're using a modern browser that supports ES modules

### Organization Sign-In Issues

- If you're having trouble signing in, try leaving the organization slug empty to allow manual selection
- Make sure you have valid credentials for the Foxglove organization
- Check that your organization slug is correct (case-sensitive)

### File Loading Issues

- For local files, ensure your browser supports the File System Access API (Chrome, Edge, or Opera)
- For remote files, verify that CORS and range request headers are enabled on the server
- Check the browser console for specific error messages

### Development Server Issues

- If port 3000 is already in use, Vite will automatically try the next available port
- Check the terminal output for the actual URL where the server is running
- Make sure you have Node.js 18+ installed

## Documentation

For more information, see the [Foxglove Embed TypeScript SDK documentation](https://docs.foxglove.dev/docs/embed/typescript-sdk).

## License

This example is licensed under the MIT License. See the [LICENSE](../../LICENSE) file for details.
