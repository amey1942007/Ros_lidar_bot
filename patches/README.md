# AMR website patches (amrslam.tech)

Prefer pushing the AMR branch directly when credentials allow. Patches here are a
fallback apply path on a Windows checkout of `amey1942007/AMR`.

## Latest on main (bridge joy fix already merged)

Joy/`AMR_SOURCE=ros` gamepad fix and private-control patch landed via PRs #21 + #16.

## Still optional for AMR: landscape pad one-window fit

**File:** `amr-pad-landscape-fit.patch`

```powershell
cd D:\AMR
git checkout main
git pull
git checkout -b cursor/pad-landscape-fit-4b29

Invoke-WebRequest -Uri "https://raw.githubusercontent.com/amey1942007/Ros_lidar_bot/main/patches/amr-pad-landscape-fit.patch" -OutFile "$env:USERPROFILE\Downloads\amr-pad-landscape-fit.patch"

git apply "$env:USERPROFILE\Downloads\amr-pad-landscape-fit.patch"
git add -A
git commit -m "Fit landscape pad to one-window wireframe layout"
git push -u origin cursor/pad-landscape-fit-4b29
```

## Also available / already applied on AMR

- `amr-control-pipeline-signal.patch` — private control + 6s stale veil (AMR PR #24)
- `amr-fix-media-check-deploy.patch` — Pages media check (AMR PR #23)
- Earlier control/UI patches — already merged on AMR
