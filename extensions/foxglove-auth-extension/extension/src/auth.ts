/**
 * OAuth 2.0 PKCE Authentication Module for Foxglove Extension
 *
 * Implements the full PKCE flow with automatic environment detection:
 * 1. Generates code verifier + challenge
 * 2. Tries to open a browser popup (works on web)
 * 3. If popup fails (desktop app / Electron), falls back to system browser
 * 4. Callback page exchanges code for tokens, POSTs them to a relay endpoint
 * 5. Extension polls the relay endpoint until tokens arrive
 * 6. Stores refresh token in localStorage for silent refresh
 *
 * The callback page is a static HTML file. The relay is two small endpoints
 * (POST + GET) on the customer's existing backend.
 */

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
export interface AuthConfig {
  /** OAuth client ID registered with your IdP */
  clientId: string;
  /** IdP authorization endpoint (where the popup navigates) */
  authorizationUrl: string;
  /** IdP token endpoint (used by callback page and for refresh) */
  tokenUrl: string;
  /** URL of the customer-hosted static callback page */
  redirectUri: string;
  /** OAuth scopes to request */
  scopes: string[];
  /** localStorage key prefix for this extension's tokens */
  storagePrefix: string;
  /**
   * Server-side relay endpoint for token delivery.
   *
   * The callback page POSTs tokens to this URL keyed by the OAuth state
   * parameter, and the extension polls GET <url>?state=<state> until
   * the tokens arrive.
   *
   * Security: the state parameter is a 128-bit cryptographic random value
   * (unguessable). Results are consumed on first read (one-time pickup)
   * and auto-expire after 5 minutes.
   */
  authResultUrl: string;
  /**
   * Endpoint that opens a URL in the system browser (fallback for desktop).
   *
   * The extension calls GET <openBrowserUrl>?url=<authUrl> when it detects
   * that window.open() did not produce a usable browser popup. This happens
   * automatically in the Foxglove desktop app (Electron), where window.open()
   * creates a Foxglove tab rather than a real browser popup.
   *
   * In production, this would ideally be replaced by a Foxglove extension
   * API like context.openExternalUrl().
   */
  openBrowserUrl: string;
}

export interface TokenSet {
  accessToken: string;
  refreshToken: string | null;
  expiresAt: number; // epoch ms
}

// ---------------------------------------------------------------------------
// PKCE Helpers
// ---------------------------------------------------------------------------

function generateCodeVerifier(): string {
  const array = new Uint8Array(32);
  crypto.getRandomValues(array);
  return base64UrlEncode(array);
}

async function generateCodeChallenge(verifier: string): Promise<string> {
  const encoder = new TextEncoder();
  const data = encoder.encode(verifier);
  const digest = await crypto.subtle.digest("SHA-256", data);
  return base64UrlEncode(new Uint8Array(digest));
}

function base64UrlEncode(buffer: Uint8Array): string {
  let binary = "";
  for (const byte of buffer) {
    binary += String.fromCharCode(byte);
  }
  return btoa(binary).replace(/\+/g, "-").replace(/\//g, "_").replace(/=+$/, "");
}

function generateState(): string {
  const array = new Uint8Array(16);
  crypto.getRandomValues(array);
  return base64UrlEncode(array);
}

// ---------------------------------------------------------------------------
// Token Storage (localStorage)
// ---------------------------------------------------------------------------

function getStorageKey(config: AuthConfig, suffix: string): string {
  return `${config.storagePrefix}:${suffix}`;
}

function storeRefreshToken(config: AuthConfig, token: string): void {
  try {
    localStorage.setItem(getStorageKey(config, "refresh_token"), token);
  } catch {
    console.warn("[auth] Failed to store refresh token in localStorage");
  }
}

function loadRefreshToken(config: AuthConfig): string | null {
  try {
    return localStorage.getItem(getStorageKey(config, "refresh_token"));
  } catch {
    return null;
  }
}

function clearRefreshToken(config: AuthConfig): void {
  try {
    localStorage.removeItem(getStorageKey(config, "refresh_token"));
  } catch {
    // ignore
  }
}

// ---------------------------------------------------------------------------
// Token Refresh (silent, no popup)
// ---------------------------------------------------------------------------

async function refreshAccessToken(
  config: AuthConfig,
  refreshToken: string,
): Promise<TokenSet | null> {
  try {
    const body = new URLSearchParams({
      grant_type: "refresh_token",
      client_id: config.clientId,
      refresh_token: refreshToken,
    });

    const resp = await fetch(config.tokenUrl, {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: body.toString(),
    });

    if (!resp.ok) {
      console.warn("[auth] Refresh token exchange failed:", resp.status);
      return null;
    }

    const data = await resp.json();
    const tokenSet: TokenSet = {
      accessToken: data.access_token,
      refreshToken: data.refresh_token ?? refreshToken,
      expiresAt: Date.now() + (data.expires_in ?? 3600) * 1000,
    };

    if (data.refresh_token) {
      storeRefreshToken(config, data.refresh_token);
    }

    return tokenSet;
  } catch (err) {
    console.error("[auth] Refresh token exchange error:", err);
    return null;
  }
}

// ---------------------------------------------------------------------------
// Server-side relay polling
// ---------------------------------------------------------------------------

let activeAbortController: AbortController | null = null;

/** Cancel an in-progress login flow. */
export function cancelLogin(): void {
  if (activeAbortController) {
    activeAbortController.abort();
    activeAbortController = null;
  }
}

/**
 * Poll the relay endpoint until the callback page deposits the tokens,
 * or until cancelled / timed out.
 */
function pollForAuthResult(
  config: AuthConfig,
  state: string,
  signal: AbortSignal,
): Promise<TokenSet> {
  return new Promise<TokenSet>((resolve, reject) => {
    const pollUrl = `${config.authResultUrl}?state=${encodeURIComponent(state)}`;
    const POLL_INTERVAL_MS = 1000;
    const MAX_POLL_TIME_MS = 5 * 60 * 1000;
    const deadline = Date.now() + MAX_POLL_TIME_MS;
    let settled = false;

    const onAbort = () => {
      if (!settled) {
        settled = true;
        reject(new Error("Login was cancelled."));
      }
    };
    signal.addEventListener("abort", onAbort, { once: true });

    const poll = async () => {
      while (!settled && Date.now() < deadline) {
        if (signal.aborted) return;

        try {
          const resp = await fetch(pollUrl);
          if (resp.ok) {
            const data = await resp.json();
            if (data.status === "complete" && !settled) {
              if (data.error) {
                settled = true;
                signal.removeEventListener("abort", onAbort);
                reject(new Error(`OAuth error: ${data.error}`));
                return;
              }

              const tokenSet: TokenSet = {
                accessToken: data.access_token,
                refreshToken: data.refresh_token ?? null,
                expiresAt: Date.now() + (data.expires_in ?? 3600) * 1000,
              };

              if (tokenSet.refreshToken) {
                storeRefreshToken(config, tokenSet.refreshToken);
              }

              settled = true;
              signal.removeEventListener("abort", onAbort);
              resolve(tokenSet);
              return;
            }
          }
        } catch {
          if (signal.aborted) return;
        }

        await new Promise<void>((r) => setTimeout(r, POLL_INTERVAL_MS));
      }

      if (!settled) {
        settled = true;
        signal.removeEventListener("abort", onAbort);
        reject(new Error("Authentication timed out — no response from callback page."));
      }
    };

    poll();
  });
}

// ---------------------------------------------------------------------------
// Popup detection
// ---------------------------------------------------------------------------

/**
 * Try to open a real browser popup. Returns the Window if successful,
 * or null if the popup was blocked or is not a real browser window
 * (e.g., Foxglove desktop creates an internal tab instead).
 *
 * Detection: window.open("about:blank") returns a Window in both web
 * browsers and Foxglove desktop. But in a real browser, about:blank
 * inherits the opener's origin, so popup.document is accessible.
 * In Foxglove desktop, the "popup" is an internal tab with its own
 * origin, so accessing popup.document throws a cross-origin error.
 *
 * MUST be called synchronously within a user-gesture (click) handler
 * to avoid popup blockers.
 */
function tryOpenPopup(): Window | null {
  let popup: Window | null = null;
  try {
    popup = window.open(
      "about:blank",
      "oauth_popup",
      "width=500,height=700,left=200,top=100",
    );
    if (popup) {
      // Test if this is a real browser popup by accessing its document.
      // In a real browser, about:blank inherits the opener's origin.
      // In Foxglove desktop (Electron), this throws a cross-origin error
      // because the "popup" is actually an internal Foxglove tab.
      popup.document.title = "Logging in…";
    }
  } catch {
    // Cross-origin error → not a real popup (Foxglove desktop)
    console.log("[auth] window.open produced a non-browser window, will use system browser");
    if (popup) {
      try { popup.close(); } catch { /* ignore */ }
    }
    popup = null;
  }
  return popup;
}

// ---------------------------------------------------------------------------
// Login (unified — works on both web and desktop)
// ---------------------------------------------------------------------------

/**
 * Start the PKCE login flow. Automatically detects the environment:
 *
 * - **Web app**: Opens a browser popup to the IdP. The callback page
 *   exchanges the auth code and POSTs tokens to the relay. The extension
 *   polls the relay until the tokens arrive.
 *
 * - **Desktop app (Electron)**: window.open() creates a Foxglove tab
 *   instead of a real popup. The function detects this and falls back
 *   to calling the openBrowserUrl endpoint, which opens the system
 *   browser. Token delivery still works through the relay.
 *
 * MUST be called from a user-gesture (click) handler so that
 * window.open() is not blocked by the browser.
 */
export async function login(config: AuthConfig): Promise<TokenSet> {
  // Step 1: Try to open a real browser popup (synchronous, in click context)
  const popup = tryOpenPopup();

  // Step 2: Generate PKCE parameters
  const codeVerifier = generateCodeVerifier();
  const codeChallenge = await generateCodeChallenge(codeVerifier);
  const state = generateState();

  const abortController = new AbortController();
  activeAbortController = abortController;

  const params = new URLSearchParams({
    response_type: "code",
    client_id: config.clientId,
    redirect_uri: config.redirectUri,
    scope: config.scopes.join(" "),
    state,
    code_challenge: codeChallenge,
    code_challenge_method: "S256",
    code_verifier: codeVerifier,
  });

  const authUrl = `${config.authorizationUrl}?${params.toString()}`;

  // Step 3: Navigate to the IdP
  if (popup) {
    // Web path — navigate the popup we already opened
    console.log("[auth] Using browser popup for login");
    popup.location.href = authUrl;
  } else {
    // Desktop path — ask the server to open the system browser
    console.log("[auth] Using system browser for login");
    const openUrl = `${config.openBrowserUrl}?url=${encodeURIComponent(authUrl)}`;
    try {
      const resp = await fetch(openUrl);
      if (!resp.ok) {
        activeAbortController = null;
        throw new Error("Failed to open system browser for login");
      }
    } catch (err) {
      activeAbortController = null;
      throw err;
    }
  }

  // Step 4: Poll the relay for tokens (works the same for both paths)
  try {
    const tokenSet = await pollForAuthResult(config, state, abortController.signal);
    activeAbortController = null;
    return tokenSet;
  } catch (err) {
    activeAbortController = null;
    throw err;
  }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

export type AuthState =
  | { status: "idle" }
  | { status: "loading" }
  | { status: "authenticated"; tokenSet: TokenSet }
  | { status: "error"; message: string };

/**
 * Force a fresh access token using the stored refresh token.
 */
export async function silentRefresh(config: AuthConfig): Promise<TokenSet | null> {
  const rt = loadRefreshToken(config);
  if (!rt) return null;
  const result = await refreshAccessToken(config, rt);
  if (!result) {
    clearRefreshToken(config);
  }
  return result;
}

/** Clear all stored auth state (logout). */
export function logout(config: AuthConfig): void {
  clearRefreshToken(config);
}
