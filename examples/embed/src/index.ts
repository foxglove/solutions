import { FoxgloveViewer } from "@foxglove/embed";

// Wait for DOM to be ready
function init() {
  // Get the container element
  const container = document.getElementById("viewer-container");
  if (!container) {
    throw new Error("Container element not found");
  }

  // Initialize the viewer
  let viewer: FoxgloveViewer | null = null;
  let currentOrgSlug: string | undefined = undefined;

  // Status element
  const statusEl = document.getElementById("status");
  if (!statusEl) {
    throw new Error("Status element not found");
  }

  function setStatus(message: string, type: "info" | "success" | "error" = "info") {
    statusEl.textContent = message;
    statusEl.className = `status ${type}`;
  }

  // Initialize viewer with organization slug
  function initializeViewer(orgSlug?: string) {
    if (viewer) {
      viewer.remove();
    }

    try {
      viewer = new FoxgloveViewer({
        parent: container!,
        orgSlug: orgSlug,
      });

      // Listen to ready event
      viewer.addEventListener("ready", () => {
        setStatus("Foxglove viewer is ready", "success");
        enableButtons();
      });

      // Listen to error event
      viewer.addEventListener("error", (e) => {
        setStatus(`Error: ${e.detail}`, "error");
        console.error("FoxgloveViewer error:", e.detail);
      });

      setStatus("Initializing viewer...", "info");
      disableButtons();
    } catch (error) {
      setStatus(`Failed to initialize viewer: ${error}`, "error");
      console.error("Failed to initialize viewer:", error);
    }
  }

  // Button state management
  function enableButtons() {
    const buttons = document.querySelectorAll(".controls button");
    buttons.forEach((btn) => {
      (btn as HTMLButtonElement).disabled = false;
    });
  }

  function disableButtons() {
    const buttons = document.querySelectorAll(".controls button");
    buttons.forEach((btn) => {
      (btn as HTMLButtonElement).disabled = true;
    });
  }

  // Set organization slug
  document.getElementById("btn-set-org")?.addEventListener("click", () => {
    const orgSlugInput = document.getElementById("org-slug") as HTMLInputElement;
    const orgSlug = orgSlugInput.value.trim() || undefined;
    currentOrgSlug = orgSlug;
    initializeViewer(orgSlug);
    setStatus(`Organization set to: ${orgSlug || "none (user can select)"}`, "info");
  });

  // Load file
  document.getElementById("btn-file")?.addEventListener("click", async () => {
    if (!viewer) {
      setStatus("Viewer not initialized", "error");
      return;
    }

    try {
      const handles = await window.showOpenFilePicker({
        multiple: false,
        types: [
          {
            description: ".bag, .db3, .ulg, .ulog, .mcap",
            accept: {
              "application/octet-stream": [".bag", ".db3", ".ulg", ".ulog", ".mcap"],
            },
          },
        ],
      });

      const file = await handles[0]!.getFile();
      viewer.setDataSource({
        type: "file",
        file,
      });
      setStatus(`Loaded file: ${file.name}`, "success");
    } catch (error) {
      if (error instanceof Error && error.name !== "AbortError") {
        setStatus(`Failed to load file: ${error.message}`, "error");
        console.error("Failed to load file:", error);
      }
    }
  });

  // Connect to live WebSocket
  document.getElementById("btn-live")?.addEventListener("click", () => {
    if (!viewer) {
      setStatus("Viewer not initialized", "error");
      return;
    }

    const url = prompt("Enter WebSocket URL (e.g., ws://localhost:8765):");
    if (!url) return;

    const protocol = confirm("Use ROS Bridge? (Cancel for Foxglove WebSocket)")
      ? "rosbridge-websocket"
      : "foxglove-websocket";

    try {
      viewer.setDataSource({
        type: "live",
        protocol: protocol,
        url: url,
      });
      setStatus(`Connected to ${protocol} at ${url}`, "success");
    } catch (error) {
      setStatus(`Failed to connect: ${error}`, "error");
      console.error("Failed to connect:", error);
    }
  });

  // Load recording
  document.getElementById("btn-recording")?.addEventListener("click", () => {
    if (!viewer) {
      setStatus("Viewer not initialized", "error");
      return;
    }

    const recordingId = prompt("Enter Recording ID:");
    if (!recordingId) return;

    const projectId = prompt("Enter Project ID (optional):") || undefined;

    try {
      viewer.setDataSource({
        type: "recording",
        recordingId: recordingId,
        projectId: projectId,
      });
      setStatus(`Loading recording: ${recordingId}`, "success");
    } catch (error) {
      setStatus(`Failed to load recording: ${error}`, "error");
      console.error("Failed to load recording:", error);
    }
  });

  // Load remote file
  document.getElementById("btn-remote-file")?.addEventListener("click", () => {
    if (!viewer) {
      setStatus("Viewer not initialized", "error");
      return;
    }

    const url = prompt("Enter remote file URL (ensure CORS and range headers are enabled):");
    if (!url) return;

    try {
      viewer.setDataSource({
        type: "remote-file",
        urls: [url],
      });
      setStatus(`Loading remote file: ${url}`, "success");
    } catch (error) {
      setStatus(`Failed to load remote file: ${error}`, "error");
      console.error("Failed to load remote file:", error);
    }
  });

  // Connect to device
  document.getElementById("btn-device")?.addEventListener("click", () => {
    if (!viewer) {
      setStatus("Viewer not initialized", "error");
      return;
    }

    const deviceId = prompt("Enter Device ID (or leave empty to use device name):");
    const deviceName = deviceId ? undefined : prompt("Enter Device Name:");

    if (!deviceId && !deviceName) return;

    const start = prompt("Enter start time (ISO8601/RFC3339, e.g., 2025-01-01T00:00:00Z):");
    if (!start) return;

    const end = prompt("Enter end time (ISO8601/RFC3339, e.g., 2025-02-01T00:00:00Z):");
    if (!end) return;

    try {
      viewer.setDataSource({
        type: "device",
        ...(deviceId ? { deviceId } : { deviceName: deviceName! }),
        start: start,
        end: end,
      });
      setStatus(`Connecting to device: ${deviceId || deviceName}`, "success");
    } catch (error) {
      setStatus(`Failed to connect to device: ${error}`, "error");
      console.error("Failed to connect to device:", error);
    }
  });

  // Load layout
  document.getElementById("btn-layout")?.addEventListener("click", async () => {
    if (!viewer) {
      setStatus("Viewer not initialized", "error");
      return;
    }

    try {
      const handles = await window.showOpenFilePicker({
        multiple: false,
        types: [
          {
            description: ".json",
            accept: {
              "application/json": [".json"],
            },
          },
        ],
      });

      const file = await handles[0]!.getFile();
      const layoutData = JSON.parse(await file.text());

      const storageKey = prompt("Enter storage key for this layout (e.g., 'layout-a'):") || "default-layout";
      const force = confirm("Override local changes? (OK = yes, Cancel = no)");

      viewer.selectLayout({
        storageKey: storageKey,
        opaqueLayout: layoutData,
        force: force,
      });
      setStatus(`Loaded layout: ${storageKey}`, "success");
    } catch (error) {
      if (error instanceof Error && error.name !== "AbortError") {
        setStatus(`Failed to load layout: ${error.message}`, "error");
        console.error("Failed to load layout:", error);
      }
    }
  });

  // Save layout
  document.getElementById("btn-save-layout")?.addEventListener("click", async () => {
    if (!viewer) {
      setStatus("Viewer not initialized", "error");
      return;
    }

    try {
      const layoutData = await viewer.getLayout();
      const blob = new Blob([JSON.stringify(layoutData, null, 2)], { type: "application/json" });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = "foxglove-layout.json";
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);
      setStatus("Layout saved", "success");
    } catch (error) {
      setStatus(`Failed to save layout: ${error}`, "error");
      console.error("Failed to save layout:", error);
    }
  });

  // Initialize with no organization (user can select)
  initializeViewer();
}

// Initialize when DOM is ready
if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", init);
} else {
  init();
}
