# AMR website patches (amrslam.tech)

This Cloud Agent cannot push to `amey1942007/AMR`. Apply these on that repo.

## Latest: name-only login + Home button + password reset

**File:** `amr-control-login-home.patch`

**Already done on Supabase (no action needed):**
- `operator-auth` edge function redeployed (claim / status / email)
- All active operators set to `password_set = false` — next login asks them to choose a new password

Includes:
- Sign-in: **Name only** (allowlist: amey, vanshika, gautam, lakshya, sanidhya, himanshu, ammar)
- If password was reset / never set → “Set your password” then continue
- No Forgot password — footer says email `amey@amrslam.tech` or `admin@amrslam.tech`
- Control panel header: **← Home** back to the marketing site

### Apply (PowerShell) — after glass/Xbox patch is on main

```powershell
cd D:\AMR
git checkout main
git pull
git checkout -b cursor/control-login-home-4b29

Invoke-WebRequest -Uri "https://raw.githubusercontent.com/amey1942007/Ros_lidar_bot/cursor/tailscale-bridge-relay-4b29/patches/amr-control-login-home.patch" -OutFile "$env:USERPROFILE\Downloads\amr-control-login-home.patch"

git apply "$env:USERPROFILE\Downloads\amr-control-login-home.patch"
git add -A
git commit -m "Control login: name-only allowlist, password reset, Home link"
git push -u origin cursor/control-login-home-4b29
```

Then open: https://github.com/amey1942007/AMR/pull/new/cursor/control-login-home-4b29 → merge to `main`.

### After deploy — login flow

1. Hard-refresh https://amrslam.tech/sign-in/
2. Enter your operator **name** (not email) → Continue
3. Set a new password (min 10 chars) → Save & continue
4. On `/control/`, use **← Home** to return to the site

## Previous: liquid glass + Xbox controller + bot persist

**File:** `amr-control-glass-xbox.patch` — apply first if not already on `main`.

## Older: CSP wss + basic theme (already merged as AMR PR #14)

`amr-control-theme-csp.patch` — only needed if that merge is missing.
