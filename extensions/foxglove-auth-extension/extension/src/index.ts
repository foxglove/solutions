import { ExtensionContext } from "@foxglove/extension";
import { initAuthDemoPanel } from "./AuthDemoPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "auth-demo-panel",
    initPanel: initAuthDemoPanel,
  });
}
