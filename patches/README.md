# AMR website patches (amrslam.tech)

This Cloud Agent cannot push to `amey1942007/AMR`. Apply these on that repo.

## Latest: liquid glass + Xbox controller + bot persist

**File:** `amr-control-glass-xbox.patch`

Includes:
- Liquid-glass control/login styling, wave background, dense vertical scroll (status on top)
- Bot selection persisted in `localStorage` + `?bot=` (refresh keeps Hardware bot / AMR4)
- **Controller** button → on-screen Xbox pad that streams the same `joy` `{a[8], b}` layout as `web_gamepad_node` `/api/joy`
- Physical Xbox pads still work; on-screen pad overrides while you touch it

### Apply (PowerShell on Windows)

```powershell
cd D:\AMR
git checkout main
git pull
git checkout -b cursor/control-glass-xbox-4b29

Invoke-WebRequest -Uri "https://raw.githubusercontent.com/amey1942007/Ros_lidar_bot/cursor/tailscale-bridge-relay-4b29/patches/amr-control-glass-xbox.patch" -OutFile "$env:USERPROFILE\Downloads\amr-control-glass-xbox.patch"

git apply "$env:USERPROFILE\Downloads\amr-control-glass-xbox.patch"
git add -A
git commit -m "Control panel: liquid glass, Xbox joy to web_gamepad, persist bot"
git push -u origin cursor/control-glass-xbox-4b29
```

Then open: https://github.com/amey1942007/AMR/pull/new/cursor/control-glass-xbox-4b29 → merge to `main`.

### After deploy — how to drive

1. Hard-refresh https://amrslam.tech/control/
2. Select **Hardware bot** (sticky after refresh)
3. **Request / take control**
4. Click **Controller** → drag left stick (or plug in a real Xbox pad)
5. On the Pi: `AMR_GAMEPAD_URL=http://127.0.0.1:8765` + `web_gamepad_node` running for full `/joy`; otherwise bridge falls back to nudge from left stick

## Older: CSP wss + basic theme (already merged as AMR PR #14)

`amr-control-theme-csp.patch` — only needed if that merge is missing.
