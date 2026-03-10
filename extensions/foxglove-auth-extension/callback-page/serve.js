/**
 * Simple static file server for the callback page.
 * In production, this HTML file would be hosted at a stable URL
 * on the customer's infrastructure (e.g., https://auth.customer.com/foxglove-callback).
 *
 * Usage: node serve.js
 * Serves on http://localhost:4001
 */

const http = require("http");
const fs = require("fs");
const path = require("path");

const PORT = 4001;
const CALLBACK_FILE = path.join(__dirname, "callback.html");

const server = http.createServer((req, res) => {
  // CORS headers so the extension can load this in a popup
  res.setHeader("Access-Control-Allow-Origin", "*");
  res.setHeader("Access-Control-Allow-Methods", "GET");

  if (req.url?.startsWith("/callback.html") || req.url === "/") {
    const html = fs.readFileSync(CALLBACK_FILE, "utf-8");
    res.writeHead(200, { "Content-Type": "text/html" });
    res.end(html);
  } else {
    res.writeHead(404);
    res.end("Not found");
  }
});

server.listen(PORT, () => {
  console.log(`[callback-page] Serving callback page at http://localhost:${PORT}/callback.html`);
});
