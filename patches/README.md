# AMR website patches (amrslam.tech)

This Cloud Agent cannot push to `amey1942007/AMR`. Apply these on that repo.

## Why the site still looks old

Source for name-login + Home **is on `main`**, but **Deploy to GitHub Pages failed** on ESLint:

- `Dashboard.tsx` / `XboxController.tsx` — `react-hooks/set-state-in-effect`
- So the live static site never rebuilt; you still see Email + Forgot password.

## Fix deploy (apply this now)

**File:** `amr-fix-deploy-lint.patch`

```powershell
cd D:\AMR
git checkout main
git pull
git checkout -b cursor/fix-deploy-lint-4b29

Invoke-WebRequest -Uri "https://raw.githubusercontent.com/amey1942007/Ros_lidar_bot/cursor/tailscale-bridge-relay-4b29/patches/amr-fix-deploy-lint.patch" -OutFile "$env:USERPROFILE\Downloads\amr-fix-deploy-lint.patch"

git apply "$env:USERPROFILE\Downloads\amr-fix-deploy-lint.patch"
git add -A
git commit -m "Fix control lint so GitHub Pages deploy can publish"
git push -u origin cursor/fix-deploy-lint-4b29
```

Open: https://github.com/amey1942007/AMR/pull/new/cursor/fix-deploy-lint-4b29 → merge.

Watch: https://github.com/amey1942007/AMR/actions — **Deploy to GitHub Pages** must be **green**.

Then hard-refresh https://amrslam.tech/sign-in/ (Ctrl+Shift+R). You should see **Name** (not Email), no Forgot password, and **← Home** on `/control/`.

## Other patches (already merged into AMR main)

- `amr-control-login-home.patch` — name login + Home (merged; blocked by lint above)
- `amr-control-glass-xbox.patch` — liquid glass + Xbox (merged; same deploy fail)
- `amr-control-theme-csp.patch` — CSP wss (AMR PR #14, live)
