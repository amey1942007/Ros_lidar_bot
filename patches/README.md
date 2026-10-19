# AMR website patches (amrslam.tech)

This Cloud Agent cannot push to `amey1942007/AMR`. Apply these on that repo.

## Latest: wireframe control layout + controller window

**File:** `amr-control-wireframe-layout.patch`

Matches the hand sketch:
- **Live view** (left) with **Camera feed** picture-in-picture when vision is on
- **System status + System** (right rail)
- **Tools** (full width)
- **Waypoint sequence** | **Console** (bottom row)
- **Controller** opens a **new window** (`/control/pad/`) for mobile complete control

```powershell
cd D:\AMR
git checkout main
git pull
git checkout -b cursor/control-wireframe-layout-4b29

Invoke-WebRequest -Uri "https://raw.githubusercontent.com/amey1942007/Ros_lidar_bot/cursor/tailscale-bridge-relay-4b29/patches/amr-control-wireframe-layout.patch" -OutFile "$env:USERPROFILE\Downloads\amr-control-wireframe-layout.patch"

git apply "$env:USERPROFILE\Downloads\amr-control-wireframe-layout.patch"
git add -A
git commit -m "Control panel wireframe layout + controller window"
git push -u origin cursor/control-wireframe-layout-4b29
```

Open: https://github.com/amey1942007/AMR/pull/new/cursor/control-wireframe-layout-4b29 → merge. Wait for green **Deploy to GitHub Pages**, then hard-refresh `/control/`.

## Already merged

- `amr-no-password-min.patch` — no 10-char password minimum
- `amr-fix-deploy-lint.patch` — Pages deploy lint (PR #17)
- `amr-control-login-home.patch` — name login + Home
- `amr-control-glass-xbox.patch` — liquid glass + Xbox
- `amr-control-theme-csp.patch` — CSP wss
