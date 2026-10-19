# AMR website patches (amrslam.tech)

This Cloud Agent cannot push to `amey1942007/AMR`. Apply these on that repo.

## Latest: pad landscape one-window fit

**File:** `amr-pad-landscape-fit.patch`

Complete controller (`/control/pad/`) fits in **one phone landscape window** with no scroll — compact header + shoulders row + single-row playfield (dpad · LS · mid · face · RS).

```powershell
cd D:\AMR
git checkout main
git pull
git checkout -b cursor/pad-landscape-fit-4b29

Invoke-WebRequest -Uri "https://raw.githubusercontent.com/amey1942007/Ros_lidar_bot/cursor/pad-landscape-fit-4b29/patches/amr-pad-landscape-fit.patch" -OutFile "$env:USERPROFILE\Downloads\amr-pad-landscape-fit.patch"

git apply "$env:USERPROFILE\Downloads\amr-pad-landscape-fit.patch"
git add -A
git commit -m "Fit complete controller in one phone landscape window"
git push -u origin cursor/pad-landscape-fit-4b29
```

Open: https://github.com/amey1942007/AMR/pull/new/cursor/pad-landscape-fit-4b29 → merge.

## Previous: frontend priority fixes (1, 3, 5–9)

**File:** `amr-frontend-priority-fixes.patch` — merged as AMR PR #21 (Pages deploy may still be queued).


## Also pending / previous

- `amr-center-hero-bot.patch` — center landing robot (`git apply --binary`)
- `amr-control-wireframe-layout.patch` — already merged (PR #19)
