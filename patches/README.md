# AMR website patches (amrslam.tech)

Prefer pushing the AMR branch directly when credentials allow. Patches here are a
fallback apply path on a Windows checkout of `amey1942007/AMR`.

## Latest: control pipeline + signal hysteresis

**File:** `amr-control-pipeline-signal.patch`

Pairs with the Ros_lidar_bot bridge fix on `cursor/control-pipeline-signal-4b29`:

1. **`NEXT_PUBLIC_PRIVATE_CONTROL=1`** in Pages build — Pi keeps `AMR_PUBLIC_CHANNEL=0`, so nav goal / joy / tools only work on private `bot:<id>:cmd`.
2. **`STALE_VEIL_MS = 6000`** — canvas “no data from robot…” waits ~6s of missing telemetry instead of flashing on brief lag.

```powershell
cd D:\AMR
git checkout main
git pull
git checkout -b cursor/control-pipeline-signal-4b29

Invoke-WebRequest -Uri "https://raw.githubusercontent.com/amey1942007/Ros_lidar_bot/cursor/control-pipeline-signal-4b29/patches/amr-control-pipeline-signal.patch" -OutFile "$env:USERPROFILE\Downloads\amr-control-pipeline-signal.patch"

git apply "$env:USERPROFILE\Downloads\amr-control-pipeline-signal.patch"
git add -A
git commit -m "Enable private control and 6s stale veil for bot lag"
git push -u origin cursor/control-pipeline-signal-4b29
```

Open: https://github.com/amey1942007/AMR/pull/new/cursor/control-pipeline-signal-4b29 → merge (redeploys Pages).

**Pi:** pull/rebuild the Ros bridge branch so joy POSTs to web_gamepad when `AMR_SOURCE=ros`.

## Also pending / previous

- `amr-control-status-dot.patch` — status color-dot (branch `cursor/tailscale-bridge-relay-4b29`)
- `amr-fix-media-check-deploy.patch` — Pages media check (merged / deployed as needed)
- `amr-center-hero-bot.patch` — center landing robot (`git apply --binary`)
- `amr-control-wireframe-layout.patch` — already merged (PR #19)
