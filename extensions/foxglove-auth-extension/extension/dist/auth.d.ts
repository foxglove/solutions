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
    expiresAt: number;
}
/** Cancel an in-progress login flow. */
export declare function cancelLogin(): void;
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
export declare function login(config: AuthConfig): Promise<TokenSet>;
export type AuthState = {
    status: "idle";
} | {
    status: "loading";
} | {
    status: "authenticated";
    tokenSet: TokenSet;
} | {
    status: "error";
    message: string;
};
/**
 * Force a fresh access token using the stored refresh token.
 */
export declare function silentRefresh(config: AuthConfig): Promise<TokenSet | null>;
/** Clear all stored auth state (logout). */
export declare function logout(config: AuthConfig): void;
