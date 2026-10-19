# Source and history traceability

This index supports the two manuscripts in `iteration1_concise/` and `iteration2_comprehensive/`. Line numbers refer to `Ros_lidar_bot/frontier_explorer_node.py` on `main` at `de8209a`, which matches the explorer last edited in `eccbb97` (1448 lines). Claims about Nav2, SLAM, and the safety stop cite the other files named below.

Nothing in this index is a measured exploration result.

## Current behaviour

| Claim | Where |
|---|---|
| Subscriptions are map, safety Boolean, manual blacklist; publication is the marker array; actuation is `NavigateToPose` | `frontier_explorer_node.py` 151–162 |
| Timer period is `goal_interval_sec` | 162 |
| First grid calibrates once: radius only raised, patience `max(config, floor(2L))`, queue threshold `max(3*s_min, 15, floor(0.0015*W*H))` | 178–212 |
| Relaxation every 3 empty ticks; range to cap at tick 6; minimum distance halved at tick 9; sticky | 214–250 |
| In-progress order: vanish, safety, 90 s timeout, 25 s stall, then queue refresh | 252–375 |
| Vanish radius hard-coded 0.8 m; clean cancel does not blacklist | 434–461, 261–288 |
| Idle path pops the queue before a full scan | 396–405 |
| Hysteresis only when idle and a goal position is still stored; margin from YAML | 423–427 |
| Pop score is `-distance / max_goal_distance`, not the four-term utility; distance ceiling on pop is the 12 m cap | 463–508 |
| Generation counter drops stale callbacks | 539–579, 664–666; introduced in `badd582` |
| Phantom threshold 2, timeout constant 600 s | 113–114, 581–610 |
| First abort is an ordinary blacklist; second forces hits to 10 and timeout to 600 s | 621–644 |
| Cancel callback does not blacklist | 645–656; fixed in `47a164d` |
| Progress clock updates only on a drop in `distance_remaining` | 664–674 |
| Four-term score and 0.15 openness floor | 778–808 |
| Far bonus constants 0.35, 0.40, 0.45 | 816–845 |
| Queue skips the best cluster, keeps up to 5, separation `2.5 * blacklist radius`, large-only with a small-cluster fallback | 854–888 |
| Border flood is 4-connected; frontiers are 8-connected; map border of the frontier mask is cleared | 899–930, 1022–1052 |
| Permanent zones: components of at least 3 cells, centroid merge at `2 * radius`, cell keys `round(world/resolution)` plus a 1-cell buffer | 980–1015, 1231–1238 |
| Marker disk for permanent zones is still `2 * radius` | 1325–1342 |
| Blacklist merge `1.5 * radius`; timeout `min(T0 * 2^(h-1), 2400)`; test radius doubles when hits > 1 | 1140–1239 |
| Manual JSON zones | 1181–1220; dashboard conversion `robot_dashboard_node.py` 581–609 |
| Completion is a log; the timer is not stopped | 407–419 |
| `goal_handle.cancel_goal_async` is unguarded | 267, 311, 324, 342 |

## Effective parameters

Launched values are `config/frontier_explorer.yaml`, loaded by `launch/autonomous_robot.launch.py` lines 52–58. Node defaults that differ are the `declare_parameter` calls at `frontier_explorer_node.py` 28–50. The manuscripts list both. The YAML file was last changed in `d81dd84` (24 May 2026).

Safety values that the Boolean depends on are set in `launch/launch_robot.launch.py` 222–236 and implemented in `Ros_lidar_bot/safety_stop_node.py`. Nav2 interactions are `config/nav2_params.yaml`. SLAM resolution and laser limits are `config/mapper_params_online_async.yaml`. Scan filter minimum range is `launch/launch_robot.launch.py` (0.30 m), which does not match the SLAM file's 0.45 m floor.

Startup: SLAM at 5 s and Nav2 at 8 s in `launch/launch_robot.launch.py`; explorer at 22 s in `launch/autonomous_robot.launch.py`.

## History that changed decisions

| Date | Commit | What it supports |
|---|---|---|
| 2026-05-12 | `f493c96` | Baseline utility, 0.25 openness floor, 0.60 m border, blacklist 0.50 m / 90 s |
| 2026-05-13 | `82dbf87` | Floor removed; message says it rejected tight indoor frontiers |
| 2026-05-13 | `48d893c` | Calibration and relaxation added |
| 2026-05-24 | `76eae0d` | Border 0.60 m to 0.15 m; comment says 0.60 m rejected wall frontiers |
| 2026-05-24 | `d81dd84` | Current YAML, including blacklist 0.80 m / 300 s and patience 20 |
| 2026-06-02 | `827026d` | Queue and safety preemption |
| 2026-06-02 | `14edb83` | Repeat radius and minimum distance from the stuck point |
| 2026-06-03 | `6b7e6c2` | Phantom counter at threshold 3; openness floor returns at 0.15 |
| 2026-06-03 | `f1a9bd3` | Enclosed fill and diverse queue |
| 2026-06-03 | `b8676df` | Large-only queue and far bonus |
| 2026-06-03 | `276f5cb` | First abort is ordinary; second is the long exclusion |
| 2026-06-03 | `29348bf` | Legacy report v2, which still says threshold 3 and a circular permanent zone |
| 2026-06-10 | `b79ad0d` | Threshold 2; map-relative queue size; vanish and timeout pop the queue |
| 2026-06-10 | `47a164d` | No second blacklist on cancel |
| 2026-06-10 | `badd582` | Generation guard |
| 2026-06-10 | `5bbaf0f` | Exact-cell permanent zones replace the 1.6 m circle |
| 2026-07-19 | `eccbb97` | Manual blacklist; last explorer edit |
| 2026-07-26 | `91d464f`, `8547161` | 50° safety cone and PolygonSlow; explorer logic unchanged |

`home_updated` has a home-return goal (`home_recovery_ticks`, default 6) that is not on `main`. No commit on `main` explains the omission.

## Legacy report disagreements

`docs/frontier_explorer_report_v2.md` at `29348bf` disagrees with the current source on: phantom threshold (3 vs 2), permanent exclusion shape (disk vs cells), queue size rule (no `0.0015*WH` term), vanish and goal-timeout handling, termination (it says the node stops), safety geometry (simulation-era 0.25 m and 180°), and the border margin (it says 3 cells; `int(0.15/0.05)` is 2).

## Checked arithmetic, not trials

These were evaluated in Python against the source formulas, at the launched weights. They are the worked examples in the manuscripts.

- Ordinary score at size 40/80, distance 2.2 m, heading π/2, openness 0.60, D = 5.5 m: **0.67**.
- Pop of that point: **−0.40**.
- `int(0.15/0.05)` = **2**.
- Half-to-even `round(i+0.5)` aliases cell columns 1 and 2.
- Timeout from a 300 s base: 300, 600, 1200, 2400 s.

## Not in the repository

No rosbag, CSV, coverage log, or repeated-trial table. Simulation worlds named in May commits are not on `main`; they remain in the `Simulation` branch history. This study did not launch the robot or Gazebo.
