# AMR website patches (amrslam.tech)

This Cloud Agent cannot push to `amey1942007/AMR`. Apply these on that repo.

## Latest: remove 10-character password minimum

**File:** `amr-no-password-min.patch`

- Site + `operator-auth` only require a non-empty password (max 128)
- **Already live on Supabase:** edge function redeployed

```powershell
cd D:\AMR
git checkout main
git pull
git checkout -b cursor/no-password-min-4b29

Invoke-WebRequest -Uri "https://raw.githubusercontent.com/amey1942007/Ros_lidar_bot/cursor/tailscale-bridge-relay-4b29/patches/amr-no-password-min.patch" -OutFile "$env:USERPROFILE\Downloads\amr-no-password-min.patch"

git apply "$env:USERPROFILE\Downloads\amr-no-password-min.patch"
git add -A
git commit -m "Remove 10-character password minimum"
git push -u origin cursor/no-password-min-4b29
```

Open: https://github.com/amey1942007/AMR/pull/new/cursor/no-password-min-4b29 → merge. Wait for green **Deploy to GitHub Pages**, then hard-refresh sign-in.

## Already merged

- `amr-fix-deploy-lint.patch` — fixed Pages deploy (PR #17)
- `amr-control-login-home.patch` — name login + Home (PR #16)
- `amr-control-glass-xbox.patch` — liquid glass + Xbox (PR #15)
- `amr-control-theme-csp.patch` — CSP wss (PR #14)
