/**
 * Mock Customer Backend API Server
 *
 * Validates Bearer tokens by calling the IdP introspection endpoint,
 * then returns sample data. Demonstrates the end-to-end flow where
 * the extension's access token is used to authenticate against the
 * customer's own backend.
 *
 * Usage: node server.js
 * Runs on http://localhost:4002
 */

const http = require("http");
const url = require("url");

const PORT = 4002;
const IDP_INTROSPECT_URL = "http://localhost:4000/introspect";

function sendJson(res, status, data) {
  res.writeHead(status, {
    "Content-Type": "application/json",
    "Access-Control-Allow-Origin": "*",
    "Access-Control-Allow-Methods": "GET, POST, OPTIONS",
    "Access-Control-Allow-Headers": "Content-Type, Authorization",
  });
  res.end(JSON.stringify(data));
}

/**
 * Validate an access token against the IdP introspection endpoint.
 * In production, you might validate a JWT signature locally instead.
 */
async function validateToken(token) {
  try {
    const body = new URLSearchParams({ token });
    const resp = await fetch(IDP_INTROSPECT_URL, {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: body.toString(),
    });
    if (!resp.ok) return null;
    const data = await resp.json();
    return data.active ? data : null;
  } catch (err) {
    console.error("[backend] Token introspection failed:", err.message);
    return null;
  }
}

const server = http.createServer(async (req, res) => {
  const parsed = url.parse(req.url, true);

  // CORS preflight
  if (req.method === "OPTIONS") {
    res.writeHead(204, {
      "Access-Control-Allow-Origin": "*",
      "Access-Control-Allow-Methods": "GET, POST, OPTIONS",
      "Access-Control-Allow-Headers": "Content-Type, Authorization",
    });
    res.end();
    return;
  }

  // Health check
  if (parsed.pathname === "/health") {
    sendJson(res, 200, { status: "ok", service: "mock-customer-backend" });
    return;
  }

  // Protected API endpoints
  if (parsed.pathname?.startsWith("/api/")) {
    // Extract Bearer token
    const authHeader = req.headers.authorization;
    if (!authHeader || !authHeader.startsWith("Bearer ")) {
      sendJson(res, 401, { error: "unauthorized", message: "Missing or invalid Authorization header" });
      return;
    }

    const token = authHeader.slice(7);
    const tokenInfo = await validateToken(token);
    if (!tokenInfo) {
      sendJson(res, 401, { error: "unauthorized", message: "Invalid or expired access token" });
      return;
    }

    // Route to specific API handlers
    if (parsed.pathname === "/api/data") {
      sendJson(res, 200, {
        message: "Authenticated API response from customer backend",
        user: tokenInfo.sub,
        scopes: tokenInfo.scope,
        timestamp: new Date().toISOString(),
        data: {
          robotStatus: "operational",
          batteryLevel: 87,
          lastMission: "warehouse-sweep-42",
          activeSensors: ["lidar", "camera_front", "imu"],
          alerts: [],
        },
      });
    } else if (parsed.pathname === "/api/missions") {
      sendJson(res, 200, {
        missions: [
          { id: "ws-42", name: "Warehouse Sweep 42", status: "completed", duration: "12m 34s" },
          { id: "ws-43", name: "Warehouse Sweep 43", status: "in_progress", duration: "3m 12s" },
          { id: "dl-07", name: "Delivery Route 7", status: "queued", duration: null },
        ],
      });
    } else {
      sendJson(res, 404, { error: "not_found", message: "Unknown API endpoint" });
    }
    return;
  }

  sendJson(res, 404, { error: "not_found" });
});

server.listen(PORT, () => {
  console.log(`[mock-backend] Customer backend API running at http://localhost:${PORT}`);
  console.log(`[mock-backend] Protected endpoints: GET /api/data, GET /api/missions`);
});
