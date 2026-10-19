# AMR website patches (amrslam.tech)

This Cloud Agent cannot push to `amey1942007/AMR`. Apply these on that repo.

## Latest: status dot only

**File:** `amr-control-status-dot.patch`

- Removes the secondary status strip (CONNECTED NO DATA / NOT CONNECTED / ERROR / x·y·yaw·v)
- Replaces Offline / Connecting / Connected text with a **single colored dot**:
  - **Green** = live
  - **Yellow** = connecting / idle
  - **Red** = offline  
  (full phrase still in tooltip + screen-reader label)

```powershell
cd D:\AMR
git checkout main
git pull
git checkout -b cursor/control-status-dot-4b29

Invoke-WebRequest -Uri "https://raw.githubusercontent.com/amey1942007/Ros_lidar_bot/cursor/tailscale-bridge-relay-4b29/patches/amr-control-status-dot.patch" -OutFile "$env:USERPROFILE\Downloads\amr-control-status-dot.patch"

git apply "$env:USERPROFILE\Downloads\amr-control-status-dot.patch"
git add -A
git commit -m "Simplify control link status to a color dot"
git push -u origin cursor/control-status-dot-4b29
```

Open: https://github.com/amey1942007/AMR/pull/new/cursor/control-status-dot-4b29 → merge.

## Also pending / previous

- `amr-center-hero-bot.patch` — center landing robot (`git apply --binary`)
- `amr-control-wireframe-layout.patch` — already merged (PR #19)
