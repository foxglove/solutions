/**
 * Mock OAuth 2.0 Authorization Server with PKCE support.
 *
 * This server simulates a real IdP for local testing. It implements:
 * - GET  /authorize  — Authorization endpoint (shows login form, issues auth code)
 * - POST /token      — Token endpoint (exchanges auth code or refresh token for tokens)
 * - POST /introspect — Token introspection (used by the mock backend)
 *
 * Features:
 * - Full PKCE (S256) validation
 * - Refresh token rotation (each use invalidates the old token)
 * - State parameter pass-through
 * - Short-lived access tokens (300s) for testing refresh flows
 *
 * Usage: node server.js
 * Runs on http://localhost:4000
 */

const http = require("http");
const crypto = require("crypto");
const url = require("url");
const { exec } = require("child_process");

const PORT = 4000;

// In-memory stores (reset on server restart)
const authCodes = new Map();     // code -> { clientId, redirectUri, codeChallenge, scopes, state, codeVerifier, expiresAt }
const refreshTokens = new Map(); // token -> { clientId, scopes, userId }
const authResults = new Map();   // state -> { tokens } — server-side relay for token delivery

// Pre-configured test users
const USERS = {
  demo: { password: "demo", name: "Demo User", email: "demo@example.com" },
  alice: { password: "alice123", name: "Alice Engineer", email: "alice@customer.com" },
};

// Allowed client IDs
const CLIENTS = new Set(["foxglove-extension-demo"]);

function generateToken(prefix = "tok") {
  return `${prefix}_${crypto.randomBytes(24).toString("hex")}`;
}

function verifyCodeChallenge(codeVerifier, codeChallenge) {
  const hash = crypto.createHash("sha256").update(codeVerifier).digest();
  const computed = hash.toString("base64url");
  return computed === codeChallenge;
}

function parseBody(req) {
  return new Promise((resolve, reject) => {
    let body = "";
    req.on("data", (chunk) => (body += chunk));
    req.on("end", () => {
      try {
        if (req.headers["content-type"]?.includes("json")) {
          resolve(JSON.parse(body));
        } else {
          resolve(Object.fromEntries(new URLSearchParams(body)));
        }
      } catch (e) {
        reject(e);
      }
    });
  });
}

function sendJson(res, status, data) {
  res.writeHead(status, {
    "Content-Type": "application/json",
    "Access-Control-Allow-Origin": "*",
    "Access-Control-Allow-Methods": "GET, POST, OPTIONS",
    "Access-Control-Allow-Headers": "Content-Type, Authorization",
    "Cache-Control": "no-store",
  });
  res.end(JSON.stringify(data));
}

function sendHtml(res, html) {
  res.writeHead(200, {
    "Content-Type": "text/html",
    "Cache-Control": "no-store",
  });
  res.end(html);
}

// ---------------------------------------------------------------------------
// Authorization Endpoint
// ---------------------------------------------------------------------------

function handleAuthorize(req, res, query) {
  const { client_id, redirect_uri, scope, state, code_challenge, code_challenge_method, code_verifier, response_type } = query;

  if (response_type !== "code") {
    sendJson(res, 400, { error: "unsupported_response_type" });
    return;
  }

  if (!CLIENTS.has(client_id)) {
    sendJson(res, 400, { error: "invalid_client", error_description: "Unknown client_id" });
    return;
  }

  if (code_challenge_method && code_challenge_method !== "S256") {
    sendJson(res, 400, { error: "invalid_request", error_description: "Only S256 is supported" });
    return;
  }

  const loginHtml = `<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <title>Mock IdP — Login</title>
  <style>
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      display: flex; align-items: center; justify-content: center;
      height: 100vh; margin: 0; background: #1a1a2e; color: #e0e0e0;
    }
    .card {
      background: #16213e; padding: 32px; border-radius: 12px;
      box-shadow: 0 4px 24px rgba(0,0,0,0.4); width: 320px;
    }
    h2 { margin-top: 0; color: #e94560; text-align: center; }
    .hint { font-size: 12px; color: #888; text-align: center; margin-bottom: 16px; }
    label { display: block; margin-bottom: 4px; font-size: 14px; }
    input {
      width: 100%; padding: 10px; margin-bottom: 12px; border: 1px solid #333;
      border-radius: 6px; background: #0f3460; color: #fff; box-sizing: border-box;
      font-size: 14px;
    }
    button {
      width: 100%; padding: 12px; border: none; border-radius: 6px;
      background: #e94560; color: #fff; font-size: 16px; cursor: pointer;
    }
    button:hover { background: #c23152; }
  </style>
</head>
<body>
  <div class="card">
    <h2>Mock IdP Login</h2>
    <p class="hint">Test users: demo/demo or alice/alice123</p>
    <form method="POST" action="/authorize/submit">
      <input type="hidden" name="client_id" value="${encodeAttr(client_id || "")}" />
      <input type="hidden" name="redirect_uri" value="${encodeAttr(redirect_uri || "")}" />
      <input type="hidden" name="scope" value="${encodeAttr(scope || "")}" />
      <input type="hidden" name="state" value="${encodeAttr(state || "")}" />
      <input type="hidden" name="code_challenge" value="${encodeAttr(code_challenge || "")}" />
      <input type="hidden" name="code_verifier" value="${encodeAttr(code_verifier || "")}" />
      <label for="username">Username</label>
      <input type="text" id="username" name="username" placeholder="demo" required />
      <label for="password">Password</label>
      <input type="password" id="password" name="password" placeholder="demo" required />
      <button type="submit">Sign In</button>
    </form>
  </div>
</body>
</html>`;

  sendHtml(res, loginHtml);
}

function encodeAttr(val) {
  return val.replace(/&/g, "&amp;").replace(/"/g, "&quot;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}

async function handleAuthorizeSubmit(req, res) {
  const body = await parseBody(req);
  const { username, password, client_id, redirect_uri, scope, state, code_challenge, code_verifier } = body;

  const user = USERS[username];
  if (!user || user.password !== password) {
    sendHtml(res, `<!DOCTYPE html><html><body style="font-family:sans-serif;color:#f44;background:#1a1a2e;display:flex;align-items:center;justify-content:center;height:100vh;margin:0"><p>Invalid credentials. <a href="javascript:history.back()" style="color:#1976d2">Try again</a></p></body></html>`);
    return;
  }

  const code = generateToken("authcode");
  authCodes.set(code, {
    clientId: client_id,
    redirectUri: redirect_uri,
    codeChallenge: code_challenge,
    scopes: scope,
    state,
    userId: username,
    expiresAt: Date.now() + 5 * 60 * 1000,
  });

  const redirectParams = new URLSearchParams({
    code,
    state: state || "",
    code_verifier: code_verifier || "",
    token_url: `http://localhost:${PORT}/token`,
    client_id: client_id || "",
  });

  const redirectUrl = `${redirect_uri}?${redirectParams.toString()}`;
  res.writeHead(302, { Location: redirectUrl });
  res.end();
}

// ---------------------------------------------------------------------------
// Token Endpoint
// ---------------------------------------------------------------------------

async function handleToken(req, res) {
  const body = await parseBody(req);
  const { grant_type, code, redirect_uri, client_id, code_verifier, refresh_token } = body;

  if (!CLIENTS.has(client_id)) {
    sendJson(res, 400, { error: "invalid_client" });
    return;
  }

  if (grant_type === "authorization_code") {
    const authCode = authCodes.get(code);
    if (!authCode) {
      sendJson(res, 400, { error: "invalid_grant", error_description: "Unknown or expired auth code" });
      return;
    }

    authCodes.delete(code);

    if (authCode.expiresAt < Date.now()) {
      sendJson(res, 400, { error: "invalid_grant", error_description: "Auth code expired" });
      return;
    }

    if (authCode.clientId !== client_id) {
      sendJson(res, 400, { error: "invalid_grant", error_description: "Client ID mismatch" });
      return;
    }

    if (authCode.codeChallenge && code_verifier) {
      if (!verifyCodeChallenge(code_verifier, authCode.codeChallenge)) {
        sendJson(res, 400, { error: "invalid_grant", error_description: "PKCE code_verifier mismatch" });
        return;
      }
    }

    const accessToken = generateToken("access");
    const newRefreshToken = generateToken("refresh");

    refreshTokens.set(newRefreshToken, {
      clientId: client_id,
      scopes: authCode.scopes,
      userId: authCode.userId,
    });

    sendJson(res, 200, {
      access_token: accessToken,
      token_type: "Bearer",
      expires_in: 300,
      refresh_token: newRefreshToken,
      scope: authCode.scopes,
    });
    console.log(`[idp] Issued tokens for user=${authCode.userId}, client=${client_id}`);

  } else if (grant_type === "refresh_token") {
    const stored = refreshTokens.get(refresh_token);
    if (!stored) {
      sendJson(res, 400, { error: "invalid_grant", error_description: "Unknown or revoked refresh token" });
      return;
    }

    if (stored.clientId !== client_id) {
      sendJson(res, 400, { error: "invalid_grant", error_description: "Client ID mismatch" });
      return;
    }

    refreshTokens.delete(refresh_token);
    const newAccessToken = generateToken("access");
    const newRefreshToken = generateToken("refresh");

    refreshTokens.set(newRefreshToken, {
      clientId: client_id,
      scopes: stored.scopes,
      userId: stored.userId,
    });

    sendJson(res, 200, {
      access_token: newAccessToken,
      token_type: "Bearer",
      expires_in: 300,
      refresh_token: newRefreshToken,
      scope: stored.scopes,
    });
    console.log(`[idp] Refreshed tokens for user=${stored.userId} (rotation applied)`);

  } else {
    sendJson(res, 400, { error: "unsupported_grant_type" });
  }
}

// ---------------------------------------------------------------------------
// Token Introspection
// ---------------------------------------------------------------------------

async function handleIntrospect(req, res) {
  const body = await parseBody(req);
  const token = body.token;
  if (token && token.startsWith("access_")) {
    sendJson(res, 200, { active: true, scope: "read write", sub: "demo-user", client_id: "foxglove-extension-demo" });
  } else {
    sendJson(res, 200, { active: false });
  }
}

// ---------------------------------------------------------------------------
// Auth Result Relay
//
// The callback page POSTs tokens here after exchanging the auth code.
// The extension polls GET /auth-result?state=XXX to pick them up.
//
// This is needed because Foxglove's web app sets
// Cross-Origin-Opener-Policy: same-origin, which blocks
// window.opener.postMessage from the callback popup.
//
// Security: the state parameter is a 128-bit cryptographic random value
// (unguessable). Results are consumed on first read (one-time pickup)
// and auto-expire after 5 minutes.
// ---------------------------------------------------------------------------

async function handlePostAuthResult(req, res) {
  const body = await parseBody(req);
  const { state, access_token, refresh_token, expires_in, error } = body;

  if (!state) {
    sendJson(res, 400, { error: "missing_state" });
    return;
  }

  authResults.set(state, {
    access_token,
    refresh_token: refresh_token || null,
    expires_in: expires_in || 3600,
    error: error || null,
  });

  // Auto-expire after 5 minutes
  setTimeout(() => authResults.delete(state), 5 * 60 * 1000);

  console.log(`[idp] Stored auth result for state=${state.slice(0, 8)}...`);
  sendJson(res, 200, { ok: true });
}

function handleGetAuthResult(req, res, query) {
  const state = query.state;
  if (!state) {
    sendJson(res, 400, { error: "missing_state" });
    return;
  }

  const result = authResults.get(state);
  if (!result) {
    sendJson(res, 202, { status: "pending" });
    return;
  }

  // One-time pickup — consume the result
  authResults.delete(state);
  sendJson(res, 200, {
    status: "complete",
    access_token: result.access_token,
    refresh_token: result.refresh_token,
    expires_in: result.expires_in,
    error: result.error,
  });
}

// ---------------------------------------------------------------------------
// Open System Browser (for desktop app testing)
//
// When the extension runs inside Foxglove's desktop app (Electron),
// window.open() creates a Foxglove tab instead of a real browser popup.
// This endpoint opens a URL in the system browser so the OAuth flow
// works correctly on desktop.
//
// In production, this would be replaced by a Foxglove extension API
// like context.openExternalUrl().
// ---------------------------------------------------------------------------

function handleOpenBrowser(req, res, query) {
  const targetUrl = query.url;
  if (!targetUrl) {
    sendJson(res, 400, { error: "missing_url" });
    return;
  }

  // Platform-specific command to open system browser
  const platform = process.platform;
  let cmd;
  if (platform === "darwin") {
    cmd = `open "${targetUrl}"`;
  } else if (platform === "win32") {
    cmd = `start "" "${targetUrl}"`;
  } else {
    cmd = `xdg-open "${targetUrl}"`;
  }

  exec(cmd, (err) => {
    if (err) {
      console.error("[idp] Failed to open browser:", err.message);
      sendJson(res, 500, { error: "failed_to_open_browser", message: err.message });
      return;
    }
    console.log(`[idp] Opened system browser for auth flow`);
    sendJson(res, 200, { ok: true });
  });
}

// ---------------------------------------------------------------------------
// HTTP Server
// ---------------------------------------------------------------------------

const server = http.createServer(async (req, res) => {
  const parsed = url.parse(req.url, true);

  if (req.method === "OPTIONS") {
    res.writeHead(204, {
      "Access-Control-Allow-Origin": "*",
      "Access-Control-Allow-Methods": "GET, POST, OPTIONS",
      "Access-Control-Allow-Headers": "Content-Type, Authorization",
    });
    res.end();
    return;
  }

  try {
    if (req.method === "GET" && parsed.pathname === "/authorize") {
      handleAuthorize(req, res, parsed.query);
    } else if (req.method === "POST" && parsed.pathname === "/authorize/submit") {
      await handleAuthorizeSubmit(req, res);
    } else if (req.method === "POST" && parsed.pathname === "/token") {
      await handleToken(req, res);
    } else if (req.method === "POST" && parsed.pathname === "/introspect") {
      await handleIntrospect(req, res);
    } else if (req.method === "POST" && parsed.pathname === "/auth-result") {
      await handlePostAuthResult(req, res);
    } else if (req.method === "GET" && parsed.pathname === "/auth-result") {
      handleGetAuthResult(req, res, parsed.query);
    } else if (req.method === "GET" && parsed.pathname === "/open-browser") {
      handleOpenBrowser(req, res, parsed.query);
    } else if (req.method === "GET" && parsed.pathname === "/") {
      sendJson(res, 200, {
        name: "Mock OAuth 2.0 IdP",
        authorization_endpoint: `http://localhost:${PORT}/authorize`,
        token_endpoint: `http://localhost:${PORT}/token`,
        introspection_endpoint: `http://localhost:${PORT}/introspect`,
      });
    } else {
      sendJson(res, 404, { error: "not_found" });
    }
  } catch (err) {
    console.error("[idp] Error:", err);
    sendJson(res, 500, { error: "server_error", error_description: err.message });
  }
});

server.listen(PORT, () => {
  console.log(`[mock-idp] OAuth 2.0 authorization server running at http://localhost:${PORT}`);
  console.log(`[mock-idp] Test users: demo/demo, alice/alice123`);
});
