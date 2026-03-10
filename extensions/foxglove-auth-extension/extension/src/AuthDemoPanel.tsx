import { PanelExtensionContext } from "@foxglove/extension";
import { useCallback, useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import {
  AuthConfig,
  AuthState,
  TokenSet,
  cancelLogin,
  login,
  logout,
  silentRefresh,
} from "./auth";

// ---------------------------------------------------------------------------
// Configuration — points at the local test infrastructure by default
// ---------------------------------------------------------------------------
const AUTH_CONFIG: AuthConfig = {
  clientId: "foxglove-extension-demo",
  authorizationUrl: "http://localhost:4000/authorize",
  tokenUrl: "http://localhost:4000/token",
  redirectUri: "http://localhost:4001/callback.html",
  scopes: ["read", "write"],
  storagePrefix: "foxglove_ext_auth_demo",
  authResultUrl: "http://localhost:4000/auth-result",
  openBrowserUrl: "http://localhost:4000/open-browser",
};

const BACKEND_API_URL = "http://localhost:4002";

// ---------------------------------------------------------------------------
// Panel Component
// ---------------------------------------------------------------------------

function AuthDemoPanel(): JSX.Element {
  const [authState, setAuthState] = useState<AuthState>({ status: "idle" });
  const [apiResponse, setApiResponse] = useState<string | null>(null);
  const tokenSetRef = useRef<TokenSet | null>(null);

  // Try silent refresh on mount
  useEffect(() => {
    let cancelled = false;
    (async () => {
      setAuthState({ status: "loading" });
      const tokenSet = await silentRefresh(AUTH_CONFIG);
      if (cancelled) return;
      if (tokenSet) {
        tokenSetRef.current = tokenSet;
        setAuthState({ status: "authenticated", tokenSet });
      } else {
        setAuthState({ status: "idle" });
      }
    })();
    return () => {
      cancelled = true;
    };
  }, []);

  const handleLogin = useCallback(async () => {
    setApiResponse(null);
    setAuthState({ status: "loading" });

    try {
      // Try silent refresh first — no popup/browser needed
      const refreshed = await silentRefresh(AUTH_CONFIG);
      if (refreshed) {
        tokenSetRef.current = refreshed;
        setAuthState({ status: "authenticated", tokenSet: refreshed });
        return;
      }

      // login() auto-detects the environment:
      // - Web: opens a browser popup
      // - Desktop (Electron): falls back to system browser
      // Token delivery uses the server-side relay in both cases.
      const tokenSet = await login(AUTH_CONFIG);
      tokenSetRef.current = tokenSet;
      setAuthState({ status: "authenticated", tokenSet });
    } catch (err: unknown) {
      const message = err instanceof Error ? err.message : String(err);
      setAuthState({ status: "error", message });
    }
  }, []);

  const handleCancel = useCallback(() => {
    cancelLogin();
    setAuthState({ status: "idle" });
  }, []);

  const handleLogout = useCallback(() => {
    logout(AUTH_CONFIG);
    tokenSetRef.current = null;
    setAuthState({ status: "idle" });
    setApiResponse(null);
  }, []);

  const handleCallApi = useCallback(async () => {
    const ts = tokenSetRef.current;
    if (!ts) return;

    // Auto-refresh if token is about to expire
    if (ts.expiresAt - Date.now() < 30_000) {
      const refreshed = await silentRefresh(AUTH_CONFIG);
      if (refreshed) {
        tokenSetRef.current = refreshed;
        setAuthState({ status: "authenticated", tokenSet: refreshed });
      } else {
        setAuthState({ status: "idle" });
        setApiResponse("Token expired. Please log in again.");
        return;
      }
    }

    try {
      const resp = await fetch(`${BACKEND_API_URL}/api/data`, {
        headers: { Authorization: `Bearer ${tokenSetRef.current!.accessToken}` },
      });
      if (!resp.ok) {
        setApiResponse(`API error: ${resp.status} ${resp.statusText}`);
        return;
      }
      const data = await resp.json();
      setApiResponse(JSON.stringify(data, null, 2));
    } catch (err: unknown) {
      const message = err instanceof Error ? err.message : String(err);
      setApiResponse(`Fetch error: ${message}`);
    }
  }, []);

  return (
    <div style={{ padding: 16, fontFamily: "sans-serif", color: "#e0e0e0" }}>
      <h2 style={{ marginTop: 0 }}>Auth Demo Panel <span style={{ fontSize: 12, opacity: 0.5 }}>v0.0.10</span></h2>
      <p style={{ fontSize: 13, opacity: 0.7 }}>
        OAuth 2.0 PKCE with popup login &amp; refresh token persistence
      </p>

      {/* Auth status */}
      <div
        style={{
          padding: 12,
          marginBottom: 16,
          borderRadius: 6,
          background: authState.status === "authenticated" ? "#1b3a1b" : "#2a2a2a",
          border: `1px solid ${authState.status === "authenticated" ? "#2d7a2d" : "#444"}`,
        }}
      >
        <strong>Status: </strong>
        {authState.status === "idle" && <span>Not authenticated</span>}
        {authState.status === "loading" && (
          <span>Waiting for login… (complete login in the popup window)</span>
        )}
        {authState.status === "authenticated" && (
          <span style={{ color: "#4caf50" }}>Authenticated</span>
        )}
        {authState.status === "error" && (
          <span style={{ color: "#f44336" }}>Error: {authState.message}</span>
        )}

        {authState.status === "authenticated" && (
          <div style={{ marginTop: 8, fontSize: 12, opacity: 0.8 }}>
            <div>
              Access token: <code>{authState.tokenSet.accessToken.slice(0, 20)}…</code>
            </div>
            <div>
              Expires:{" "}
              {new Date(authState.tokenSet.expiresAt).toLocaleTimeString()}
            </div>
          </div>
        )}
      </div>

      {/* Actions */}
      <div style={{ display: "flex", gap: 8, flexWrap: "wrap" }}>
        {authState.status === "idle" || authState.status === "error" ? (
          <button onClick={handleLogin} style={buttonStyle("#1976d2")}>
            Log In (PKCE Popup)
          </button>
        ) : null}

        {authState.status === "loading" && (
          <button onClick={handleCancel} style={buttonStyle("#b71c1c")}>
            Cancel Login
          </button>
        )}

        {authState.status === "authenticated" && (
          <>
            <button onClick={handleCallApi} style={buttonStyle("#388e3c")}>
              Call Backend API
            </button>
            <button onClick={handleLogout} style={buttonStyle("#666")}>
              Log Out
            </button>
          </>
        )}
      </div>

      {/* API Response */}
      {apiResponse != null && (
        <div style={{ marginTop: 16 }}>
          <strong>API Response:</strong>
          <pre
            style={{
              background: "#1e1e1e",
              padding: 12,
              borderRadius: 4,
              overflow: "auto",
              maxHeight: 300,
              fontSize: 12,
            }}
          >
            {apiResponse}
          </pre>
        </div>
      )}
    </div>
  );
}

function buttonStyle(bg: string): React.CSSProperties {
  return {
    padding: "8px 16px",
    border: "none",
    borderRadius: 4,
    background: bg,
    color: "#fff",
    cursor: "pointer",
    fontSize: 14,
  };
}

// ---------------------------------------------------------------------------
// Foxglove Panel Registration
// ---------------------------------------------------------------------------

export function initAuthDemoPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<AuthDemoPanel />);

  return () => {
    root.unmount();
  };
}
