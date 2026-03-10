# Foxglove Extension: OAuth 2.0 PKCE Authentication

A Foxglove extension panel that authenticates against a customer's own backend
APIs using OAuth 2.0 Authorization Code with PKCE.

Works on both the **web app** and **desktop app** with the same build — no
config changes needed. The extension auto-detects the environment: it opens a
browser popup on web, and falls back to the system browser on desktop (where
`window.open()` creates a Foxglove tab instead of a real popup). Token delivery
uses a lightweight server-side relay in both cases.

## How It Works

### The problem

The extension runs inside a sandboxed iframe in the Foxglove app. After a user
logs in via a popup window, the callback page needs to send tokens back to the
extension. The standard approach — `window.opener.postMessage` — doesn't work
because Foxglove's web app sets `Cross-Origin-Opener-Policy: same-origin`, which
nullifies `window.opener` for any cross-origin popup.

### The solution: server-side relay + auto-detection

The callback page POSTs the tokens to a **relay endpoint** keyed by the OAuth
`state` parameter. The extension polls `GET /auth-result?state=S` until the
tokens arrive. The relay is two small endpoints on the customer's existing
backend. The callback page itself is still just a static HTML file.

**Security**: The `state` parameter is a 128-bit cryptographically random value
generated fresh per login — it is unguessable. Results are consumed on first
read (one-time pickup) and auto-expire after 5 minutes. This is the same
security model the OAuth spec uses for CSRF protection.

### Desktop app support

On the Foxglove desktop app (Electron), `window.open()` creates an internal
Foxglove tab rather than a real browser popup. The extension auto-detects this
by checking whether the opened window's `document` is accessible — in a real
browser, `about:blank` inherits the opener's origin, but in Foxglove desktop
the "popup" is a cross-origin internal tab, so `popup.document` throws.

When this happens, the extension falls back to calling an `openBrowserUrl`
endpoint that launches the system browser. The relay-based token delivery works
identically regardless of how the browser was opened.

In production, this server-side `/open-browser` endpoint would ideally be
replaced by a Foxglove extension API (e.g., `context.openExternalUrl()`),
following RFC 8252 (OAuth 2.0 for Native Applications).

### Auth Flow (First Login)

```
 Extension Panel                 IdP Server              Callback Page
 (Foxglove iframe)              (customer IdP)           (static HTML)
       │                              │                        │
  1.   │── open browser ─────────────►│                        │
       │   (popup on web,             │                        │
       │    system browser on desktop)│                        │
       │   /authorize?                │                        │
       │     code_challenge=X         │                        │
       │     state=S                  │                        │
       │     code_verifier=V          │                        │
       │                              │                        │
  2.   │   (User logs in)             │                        │
       │                              │                        │
  3.   │                              │── 302 redirect ───────►│
       │                              │   /callback?code=C     │
       │                              │   &state=S&verifier=V  │
       │                              │                        │
  4.   │                              │◄── POST /token ────────│
       │                              │    code=C, verifier=V  │
       │                              │                        │
  5.   │                              │── {access_token, ─────►│
       │                              │    refresh_token}      │
       │                              │                        │
  6.   │                              │◄── POST /auth-result ──│
       │                              │    {state=S, tokens}   │
       │                              │                        │
  7.   │── GET /auth-result?state=S ─►│                        │
       │                              │                        │
  8.   │◄── {status:"complete", ──────│    (popup closes)      │
       │     access_token, ...}       │                        │
       │                              │                        │
  9.   │── GET /api/data ──────────────────────────────────────────► Customer
       │   Authorization: Bearer T                                   Backend
       │◄── {robotStatus: ...} ────────────────────────────────────┘
```

Steps 7–8 are the polling loop. The extension sends `GET /auth-result?state=S`
every second. The server returns `{"status":"pending"}` until the callback page
deposits the tokens (step 6), then returns them once and deletes them.

### Subsequent Sessions

1. Extension loads, finds a refresh token in `localStorage`
2. Extension calls the IdP token endpoint to silently exchange it for a new access token
3. No popup, no user interaction
4. If the IdP supports refresh token rotation, the new refresh token replaces the old one

## Project Structure

```
foxglove-auth-extension/
├── extension/                 # Foxglove extension panel
│   ├── src/
│   │   ├── index.ts           # Extension entry point (registers the panel)
│   │   ├── auth.ts            # PKCE auth module (all OAuth logic)
│   │   └── AuthDemoPanel.tsx  # React panel UI
│   └── package.json
│
├── mock-idp/                  # Mock OAuth 2.0 authorization server
│   └── server.js              # /authorize, /token, /auth-result, /introspect, /open-browser
│
├── callback-page/             # Customer-hosted callback page (static HTML)
│   ├── callback.html          # Exchanges auth code for tokens, POSTs to relay
│   └── serve.js               # Dev server for local testing
│
├── mock-backend/              # Mock customer backend API
│   └── server.js              # Validates access tokens, returns sample data
│
├── start-test-infra.sh        # Starts all three servers
└── stop-test-infra.sh         # Stops them
```

| Component | Port | Role |
|-----------|------|------|
| Mock IdP | 4000 | Authorization server with PKCE, token issuance, refresh rotation, `/auth-result` relay, and `/open-browser` for desktop |
| Callback page | 4001 | Static HTML that exchanges the auth code and POSTs tokens to the relay |
| Mock backend | 4002 | Customer API that validates access tokens via the IdP's introspection endpoint |

## Quick Start

```bash
# 1. Start the mock servers (IdP on :4000, callback on :4001, backend on :4002)
./start-test-infra.sh

# 2. Build the extension
cd extension
npm install
npm run package          # produces demo.auth-demo-panel-0.0.10.foxe
```

3. **Install** — drag the `.foxe` file onto a Foxglove layout (web or desktop),
   or `npm run local-install` for the desktop app.
4. Add a panel → search **"Auth Demo Panel"** → click **"Log In (PKCE Popup)"**.
5. Log in with `demo` / `demo`. The panel should flip to **"Authenticated"**.
6. Click **"Call Backend API"** to verify an authenticated request works.

```bash
# When you're done
./stop-test-infra.sh
```

The same build works on both web and desktop — no config changes needed. On
desktop, the extension auto-detects that `window.open()` produces a Foxglove
tab instead of a real popup and falls back to opening the system browser.

## Security Properties

- **PKCE S256**: The extension generates a random code verifier and sends a
  SHA-256 challenge to the IdP. The callback page sends the original verifier
  when exchanging the auth code. The IdP verifies they match.
- **State parameter (CSRF protection)**: A 128-bit random state is generated per
  login attempt. The relay endpoint validates it before accepting or returning
  tokens.
- **Relay one-time pickup**: The `/auth-result` endpoint deletes the tokens after
  the first successful GET. Combined with 5-minute auto-expiry, this prevents
  replay and limits the exposure window.
- **Refresh token rotation**: Each refresh token use at the IdP invalidates the
  old token and issues a new one.
- **Token scoping**: Tokens are scoped to the customer's backend only. They are
  never sent to Foxglove APIs.
- **Namespaced localStorage**: Refresh tokens are stored under a configurable
  prefix (e.g., `foxglove_ext_auth_demo:refresh_token`).

## Using a Real OIDC Provider

The mock IdP is only for local development. To connect to a production OIDC
provider (Auth0, Okta, Azure AD, Google, Keycloak, etc.), you need to configure
three things: the IdP, the relay + callback, and the extension config.

### 1. Register an OAuth application in your IdP

Create a "Single Page Application" or "Native/Public" client:

| Setting | Value |
|---------|-------|
| **Client type** | Public (no client secret — this is a browser-based flow) |
| **Grant type** | Authorization Code with PKCE |
| **Redirect URI** | `https://auth.yourcompany.com/foxglove-callback` (where you host the callback page) |
| **Scopes** | `openid profile` plus any custom scopes your backend requires |
| **Refresh tokens** | Enabled, with rotation enabled |
| **PKCE challenge method** | S256 |

### 2. Host the callback page and relay endpoint

Deploy `callback-page/callback.html` to a stable URL on your infrastructure.

The relay is two endpoints on your backend:

**`POST /auth-result`** — called by the callback page. Stores tokens keyed by `state`.

```json
// Request
{ "state": "...", "access_token": "...", "refresh_token": "...", "expires_in": 3600 }
// Response
{ "ok": true }
```

**`GET /auth-result?state=...`** — polled by the extension. Returns `pending`
until tokens arrive, then returns them once and deletes them.

```json
// Pending
{ "status": "pending" }
// Complete (one-time)
{ "status": "complete", "access_token": "...", "refresh_token": "...", "expires_in": 3600 }
```

**`GET /open-browser?url=...`** — called by the extension on the desktop app
(only when `window.open()` fails to produce a real popup). Opens the given URL
in the system browser. This is a simple wrapper around platform-specific
commands (`open` on macOS, `xdg-open` on Linux, `start` on Windows). On the
web app, this endpoint is never called.

```json
// Response
{ "ok": true }
```

Requirements: relay entries auto-expire after 5 minutes, entries deleted after
first GET, CORS allows requests from the Foxglove app origin and callback
origin.

### 3. Update the extension config

In `extension/src/AuthDemoPanel.tsx`, update `AUTH_CONFIG`:

```typescript
const AUTH_CONFIG: AuthConfig = {
  clientId: "your-oauth-client-id",
  authorizationUrl: "https://your-idp.com/oauth2/authorize",
  tokenUrl: "https://your-idp.com/oauth2/token",
  authResultUrl: "https://auth.yourcompany.com/api/auth-result",
  redirectUri: "https://auth.yourcompany.com/foxglove-callback",
  scopes: ["openid", "profile", "your-api-scope"],
  storagePrefix: "foxglove_ext_yourcompany",
  openBrowserUrl: "https://auth.yourcompany.com/api/open-browser",
};
```

The `openBrowserUrl` endpoint is used as a fallback for the desktop app. On the
web app, `window.open()` works normally and this endpoint is never called. See
the [Desktop app support](#desktop-app-support) section for details.

Then rebuild and redistribute the `.foxe` file.

### Provider-Specific Notes

- **Auth0**: Create a "Single Page Application". Enable "Refresh Token Rotation"
  under Application Settings. Endpoints at `https://DOMAIN.auth0.com/authorize`
  and `https://DOMAIN.auth0.com/oauth/token`.

- **Okta**: Create a "Single-Page Application". PKCE is on by default. Enable
  refresh token rotation under Security → API → Authorization Servers.

- **Azure AD / Entra ID**: Register under App Registrations with a "Single-page
  application" platform. Token endpoint at
  `https://login.microsoftonline.com/{tenant}/oauth2/v2.0/token`.

- **Google**: Create an OAuth 2.0 "Web application" client. Note: Google has
  limited refresh token support for public clients — you may need
  `access_type=offline` and `prompt=consent`.

- **Keycloak**: Create a public client with "Standard flow" and PKCE S256.
  Endpoints at `https://keycloak/realms/{realm}/protocol/openid-connect/auth`
  and `.../token`.
