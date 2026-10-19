# AMR website patches (amrslam.tech)

This Cloud Agent cannot push to `amey1942007/AMR`. Apply these on that repo.

## Latest: center hero bot

**File:** `amr-center-hero-bot.patch`

Centers the 3D robot on the landing hero (camera aimed at path start −2.6, 2.45; rebaked `camera.bin`).

```powershell
cd D:\AMR
git checkout main
git pull
git checkout -b cursor/center-hero-bot-4b29

Invoke-WebRequest -Uri "https://raw.githubusercontent.com/amey1942007/Ros_lidar_bot/cursor/tailscale-bridge-relay-4b29/patches/amr-center-hero-bot.patch" -OutFile "$env:USERPROFILE\Downloads\amr-center-hero-bot.patch"

git apply --binary "$env:USERPROFILE\Downloads\amr-center-hero-bot.patch"
git add -A
git commit -m "Center the hero robot in the landing viewport"
git push -u origin cursor/center-hero-bot-4b29
```

Open: https://github.com/amey1942007/AMR/pull/new/cursor/center-hero-bot-4b29 → merge. Wait for green **Deploy to GitHub Pages**, then hard-refresh the homepage.

## Other patches

- `amr-control-wireframe-layout.patch` — control panel sketch layout + controller window
- `amr-no-password-min.patch` — no 10-char password minimum
- `amr-fix-deploy-lint.patch` / login / glass / CSP — already merged earlier
