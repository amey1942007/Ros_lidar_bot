# AMR website patch (control Realtime + theme)

This Cloud Agent cannot push to `amey1942007/AMR` (token is scoped to `Ros_lidar_bot` only).

## What the patch fixes
1. **CSP:** `connect-src` now includes `wss://<supabase-host>` so the browser can open Supabase Realtime. Without it, `/control` shows Online / waiting for data and `Command rejected: no-channel` even when the bridge/demo is publishing.
2. **Theme:** Login + control panel use the site violet/magenta tokens, Unbounded / Inter Tight, and an AMR brand wordmark (no robot emoji).

## Apply on AMR
```bash
git clone https://github.com/amey1942007/AMR.git
cd AMR
git checkout -b cursor/control-theme-and-telemetry-4b29
git apply /path/to/Ros_lidar_bot/patches/amr-control-theme-csp.patch
git commit -am "Fix control Realtime CSP and align login/control with site theme"
git push -u origin HEAD
# merge to main → Pages workflow publishes amrslam.tech
```

Or grant this agent write access to `AMR` and re-ask it to push.
