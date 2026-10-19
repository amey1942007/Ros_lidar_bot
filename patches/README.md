# AMR website patches (amrslam.tech)

This Cloud Agent cannot push to `amey1942007/AMR`. Apply these on that repo.

## Latest: frontend priority fixes (1, 3, 5–9)

**File:** `amr-frontend-priority-fixes.patch`

Implements critique priorities **1, 3, 5, 6, 7, 8, 9** (skips 2 coming-soon video, 4 mobile sticky E-STOP):

1. **Scroll jank** — lower mobile DPR/clouds, defer HDRI, pause frameloop when tab/canvas hidden or scrolled past
3. **Offline empty state** — one centered offline panel; no duplicate “Waiting for robot…”; no “X is driving” toast while offline
5. **Pad status** — color `LinkDot` only (no “Connecting” text)
6. **Login speed** — `operator-auth` status returns email; client caches; skips second round-trip
7. **Accent cleanup** — stop/cancel buttons use danger red (no mustard)
8. **Hero contrast** — rim light, darker blob/chassis materials
9. **Font fallbacks** — disable broken next/font size-adjust for Inter Tight / JetBrains

**Also redeploy the edge function** after merging (status now includes `email`):

```bash
supabase functions deploy operator-auth
```

```powershell
cd D:\AMR
git checkout main
git pull
git checkout -b cursor/frontend-priority-fixes-4b29

Invoke-WebRequest -Uri "https://raw.githubusercontent.com/amey1942007/Ros_lidar_bot/cursor/frontend-priority-fixes-4b29/patches/amr-frontend-priority-fixes.patch" -OutFile "$env:USERPROFILE\Downloads\amr-frontend-priority-fixes.patch"

git apply "$env:USERPROFILE\Downloads\amr-frontend-priority-fixes.patch"
git add -A
git commit -m "Frontend priority fixes: scroll, offline, login, accents"
git push -u origin cursor/frontend-priority-fixes-4b29
```

Open: https://github.com/amey1942007/AMR/pull/new/cursor/frontend-priority-fixes-4b29 → merge → deploy Pages + `operator-auth`.

## Also pending / previous

- `amr-control-status-dot.patch` — already merged (PR #20) if live shows the color dot
- `amr-center-hero-bot.patch` — center landing robot (`git apply --binary`)
- `amr-control-wireframe-layout.patch` — already merged (PR #19)
