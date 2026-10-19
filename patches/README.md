# AMR website patches (amrslam.tech)

This Cloud Agent cannot push to `amey1942007/AMR`. Apply these on that repo.

## Latest: simulation carousel + cloud-scroll startup info

**File:** `amr-sim-carousel-cloud-info.patch`

1. **Simulation section** — replaces “COMING SOON” with a side-by-side coverflow carousel of the two Drive demos (autonomous exploration + manual teleop). Arrows, dots, and drag/swipe rotate to the next clip. Add more later in `src/data/media.ts` → `simulationSlides`.
2. **Cloud scroll** — fills the long empty cloud stretch with four sequential startup beats (SCALE / BUILD / STACK / DRIVE).

```powershell
cd D:\AMR
git checkout main
git pull
git checkout -b cursor/sim-carousel-cloud-info-4b29

Invoke-WebRequest -Uri "https://raw.githubusercontent.com/amey1942007/Ros_lidar_bot/cursor/sim-carousel-cloud-info-4b29/patches/amr-sim-carousel-cloud-info.patch" -OutFile "$env:USERPROFILE\Downloads\amr-sim-carousel-cloud-info.patch"

git apply --binary "$env:USERPROFILE\Downloads\amr-sim-carousel-cloud-info.patch"
git add -A
git commit -m "Add sim video carousel and cloud-scroll startup beats"
git push -u origin cursor/sim-carousel-cloud-info-4b29
```

Open: https://github.com/amey1942007/AMR/pull/new/cursor/sim-carousel-cloud-info-4b29 → merge.

Drive source folder: https://drive.google.com/drive/folders/1ZrrKsbYMGt1IxCyFVXAASHwaje1rcNoL

## Previous: pad landscape one-window fit

**File:** `amr-pad-landscape-fit.patch` (on branch `cursor/pad-landscape-fit-4b29`)

Complete controller (`/control/pad/`) fits in one phone landscape window — wireframe layout.

## Previous: frontend priority fixes (1, 3, 5–9)

**File:** `amr-frontend-priority-fixes.patch` — merged as AMR PR #21 (live on Pages ~16:10 UTC).

## Also pending / previous

- `amr-center-hero-bot.patch` — center landing robot (`git apply --binary`)
- `amr-control-wireframe-layout.patch` — already merged (PR #19)
