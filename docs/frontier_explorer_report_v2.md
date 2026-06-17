# Frontier Explorer — Autonomous Navigation Reference
**Ros_lidar_bot · Branch: device-amey · 2026-06-03**

---

## 1. What It Does

Every 3 seconds the frontier explorer scores all unexplored boundaries (frontiers) on the SLAM map, sends the best one to Nav2, and keeps a standby queue for instant switching when blocked. A safety stop node runs in parallel as a hardware-critical velocity filter. Together they give the robot fully autonomous coverage with progressive recovery from any stuck state.

---

## 2. Parameters (frontier_explorer.yaml)

### Frontier Filtering

| Parameter | Value | Purpose |
|---|---|---|
| min_frontier_size | 3 cells | Smallest cluster worth a dedicated trip |
| min_goal_distance | 0.35 m | Ignore frontiers the robot is already near |
| max_goal_distance | 5.5 m | Normal search radius (auto-scales to 50% of room on startup) |
| max_goal_distance_cap | 12.0 m | Hard ceiling even after relaxation |
| max_range_expansion_factor | 2.0× | Single fallback range if nothing in normal range |
| wall_safe_distance | 0.15 m | Reject targets within 3 cells of map boundary (prevents worldToMap errors) |

### Blacklist

| Parameter | Value | Purpose |
|---|---|---|
| goal_blacklist_radius | 0.80 m | Exclusion zone around each failed position |
| blacklist_timeout_sec | 300 s | How long a standard failure stays excluded (5 min) |

### Scoring Weights

| Parameter | Value | Effect |
|---|---|---|
| size_weight | 1.8 | Favours larger unexplored clusters |
| distance_weight | 1.0 | Penalises distance to frontier |
| heading_weight | 0.5 | Penalises frontiers requiring a big turn |
| openness_weight | 0.7 | Favours open, unobstructed approach paths |
| improvement_margin | 0.20 | New goal must beat active score by this to switch |

### Goal Lifecycle

| Parameter | Value | Purpose |
|---|---|---|
| goal_interval_sec | 3.0 s | Tick rate — how often the explorer evaluates frontiers |
| goal_cooldown_sec | 3.0 s | Pause between goal end and next selection |
| goal_reached_radius | 0.45 m | Distance threshold to consider goal reached |
| goal_timeout_sec | 90.0 s | Hard cancel if nav takes too long |
| progress_timeout_sec | 25.0 s | Cancel if robot doesn't move ≥ 12 cm in this window |
| min_progress_delta | 0.12 m | Movement needed to reset the progress timer |
| no_frontier_done_ticks | 20 | Consecutive empty ticks before declaring exploration done (auto-scales with room size) |

### Safety Stop (launch_sim.launch.py)

| Parameter | Value | Purpose |
|---|---|---|
| min_safe_distance | 0.25 m | Block motion if obstacle closer than this |
| front_opening_deg | 180° (±90°) | Forward arc — covers full front hemisphere including sides |
| rear_opening_deg | 60° (±30°) | Rear arc — blocks reverse into obstacles |

---

## 3. Frontier Detection Pipeline

```
map tick
  │
  ├─ 1. Flood-fill enclosed regions → mark as occupied (permanent dead zones)
  ├─ 2. Extract frontier cells (8-connectivity numpy, O(W×H))
  ├─ 3. BFS cluster grouping (8-connectivity)
  ├─ 4. Per-cluster filters (size / target / wall / blacklist / distance / clearance)
  ├─ 5. Score remaining clusters (size + distance + heading + openness)
  ├─ 6. Far-frontier bonus if all frontiers are beyond 35% of max range
  └─ 7. Best → Nav2 goal; rest → standby queue (top 5, geographically spread)
```

### 3.1 Flood-Fill — Permanent Dead Zones

Every tick before extraction, a BFS seeds from all four map boundary edges through free/unknown cells. Any unknown cell it cannot reach is enclosed by solid obstacles (sofa interior, cupboard void, etc.) — the lidar can never see through them. These cells are:

- Marked occupied in a local grid copy → no frontier forms inside them
- Their pocket centroids stored in `_permanent_dead_zones` (no expiry, 1.60 m exclusion radius)
- Shown as **black "PERM" cylinders** in RViz2 `/frontier_debug`

```
traversable = free | unknown
reachable   = BFS from all map edges through traversable
enclosed    = unknown & ~reachable  →  mark occupied (100)
```

Without this: robot loops to furniture edges forever (phantom-success counter fires after 3 arrivals — 10 min penalty — then moves to the next pocket). With this: enclosed pockets are eliminated before any frontier is scored.

### 3.2 Per-Cluster Filters

| Filter | Condition | Effect |
|---|---|---|
| Size | len(cluster) < min_frontier_size | Skip |
| Target exists | No free cell adjacent to cluster | Skip |
| Wall guard | Target within 3 cells of map edge | Skip (prevents worldToMap errors) |
| Blacklist | Target within any blacklisted zone | Skip |
| Distance | dist < 0.35 m or dist > max_goal_distance | Skip |
| Clearance floor | Openness < 0.15 at target cell | Skip (robot would be wedged against obstacle) |

---

## 4. Scoring

```
score = 1.8 × size_term
      − 1.0 × distance_term
      − 0.5 × heading_term
      + 0.7 × openness_term
```

| Term | Formula | Range |
|---|---|---|
| size_term | cluster_size / max_cluster_size | 0–1, higher = better |
| distance_term | dist / max_goal_distance | 0–1, higher = worse |
| heading_term | abs(angle_to_frontier − robot_yaw) / π | 0–1, higher = worse |
| openness_term | free_cells / total_cells in 3-cell radius around target | 0–1, higher = better |

### Far-Frontier Bonus

Activates when the closest available frontier is beyond 35% of max_goal_distance (i.e. all frontiers are far). Distance penalty is reduced proportionally to cluster size so large far zones beat small far clusters:

```
bonus         = min(0.40, size_ratio × 0.45)
distance_term = max(0.0,  dist/max_dist − bonus)
```

Large frontier (size_ratio = 1.0) earns 0.40 bonus — distance term nearly halved. Prevents exploration stalling when only distant zones remain.

### Hysteresis Guard

A new candidate must score at least `active_score + 0.20` to trigger a goal switch. Prevents flip-flopping between marginally different frontiers.

---

## 5. Blacklist System

### Five Ways to Get Blacklisted

| Trigger | Radius | Duration |
|---|---|---|
| STATUS_CANCELLED (our timeout or safety switch) | 0.80 m | 300 s |
| STATUS_ABORTED — 1st time at this position | 0.80 m | 300 s |
| STATUS_ABORTED — 2nd+ time same area (within 1.2 m) | 1.60 m | 600 s (dead zone) |
| Phantom success — reached 2× but frontier never cleared | 1.60 m | 600 s (dead zone) |
| Enclosed region (flood-fill) | 1.60 m | **Never expires** |

**Why 1st abort is not immediate dead zone:** SLAM costmap expansion (map boundary growing) can cause a temporary Nav2 abort on navigable space. One standard blacklist gives it a chance to recover; only repeat failure confirms the area is genuinely unreachable.

### Adaptive Radius

Any entry hit more than once doubles its exclusion radius:

```
hits = 1  →  radius = 0.80 m (standard)
hits > 1  →  radius = 1.60 m (double)
```

### Exponential Timeout

Repeat failures at the same area stay excluded progressively longer:

```
timeout = min(300 × 2^(hits−1), 2400) seconds
hit 1 → 300 s    hit 2 → 600 s    hit 3 → 1200 s    hit 4+ → 2400 s
```

### Minimum Distance from Last Stuck Point

After any cancellation, the next queued frontier must be ≥ 1.60 m from where the robot just got stuck. Prevents immediately re-selecting a frontier just outside the blacklist radius.

### Clearing the Blacklist

| How | What happens |
|---|---|
| Timeout expires | Entry pruned, hit counter deleted, area available again |
| STATUS_SUCCEEDED + frontier actually cleared | Phantom counter reset to 0, no blacklist |
| SUCCESS on any goal | `_last_cancel_xy` cleared — minimum-distance restriction lifted |
| Permanent dead zone | Never cleared — geometry guarantees unreachability |

---

## 6. Phantom Success Detection

Nav2 reports STATUS_SUCCEEDED when the robot is within 0.45 m of the target. But if the unknown cells near the target haven't cleared, the area is permanently occluded (solid furniture the lidar can't see through).

```
ON STATUS_SUCCEEDED:
    if frontier still present within 0.80 m of target:
        phantom_count += 1
        if phantom_count >= 2:
            blacklist for 600 s, radius 1.60 m (dead zone)
    else:
        phantom_count = 0   # genuine success
```

---

## 7. Frontier Queue

### What it is

Every time a goal is sent, the top-5 runners-up are stored as a pre-validated standby list. The queue is checked **first every tick** — avoids a full map rescan when switching goals quickly.

### Queue Rules

| Rule | Detail |
|---|---|
| Large frontiers only | Cluster must have ≥ max(min_frontier_size × 3, 15) cells |
| Geographic diversity | Each entry must be ≥ 2.0 m from every other queue entry |
| Fallback | If < 5 large entries exist, any valid frontier fills remaining slots |
| Stale after | 20 s — if goal active: queue silently rebuilt in background same tick; if no goal: full rescan next tick |

### Queue Validation (lazy, at time of use)

```
for each (xy, score) in queue:
    skip if blacklisted
    skip if frontier already mapped
    skip if distance < min_goal_distance
    skip if distance > max_goal_distance_cap
    skip if within 1.60 m of last stuck point
    re-score from current robot position → return
```

### When the Queue Fires

| Trigger | Action |
|---|---|
| Safety blocked ≥ 3 s | Pop queue, blacklist current goal, send next immediately |
| Progress timeout (25 s no movement) | Pop queue, blacklist current goal, send next same tick |
| Frontier mapped mid-trip | Cancel without blacklisting, revert to full rescan |
| Queue stale or empty | Full fresh rescan next tick |

---

## 8. Progressive Relaxation

When no frontier is found, constraints loosen every 3 ticks so the robot doesn't give up prematurely:

| Tick | Change | Effect |
|---|---|---|
| 3 | min_frontier_size − 1 | Accepts smaller clusters |
| 6 | min_frontier_size − 1 + max range → 12 m cap | Full room search range |
| 9 | min_frontier_size − 1 + min_goal_distance halved | Accepts near frontiers |
| 12+ | min_frontier_size − 1 every 3 ticks until floor (1) | Any unknown cell counts |
| ≥ done_ticks | Exploration declared complete | Node stops sending goals |

Relaxation is **sticky** — parameters stay loose for the rest of the session once loosened.

### Auto-Calibration (one-time, on first map)

```
max_goal_distance      = min(room_larger_dim × 0.50,  12.0 m)
no_frontier_done_ticks = max(config_value,  room_larger_dim × 2 ticks/m)

10 m room → 5.0 m range, 20 ticks
15 m room → 7.5 m range, 30 ticks
25 m hall → 12.0 m range, 50 ticks
```

---

## 9. Safety Stop Node

Sits between Nav2 and the robot driver. Intercepts `/cmd_vel`, applies safety rules, publishes `/cmd_vel_safe`.

```
Nav2  →  /cmd_vel  →  safety_stop  →  /cmd_vel_safe  →  robot
```

### Rules

- Forward motion (`linear.x > 0`) blocked if any obstacle within **0.25 m** in the **180° front arc (±90°)**
- Reverse motion (`linear.x < 0`) blocked if obstacle within **0.25 m** in the **60° rear arc (±30°)**
- `angular.z` **always passes through** — Nav2 spin recovery can still rotate even when blocked

### Signal to Explorer

Publishes `/safety_blocked` (Bool) at 20 Hz. Explorer tracks continuous blocking duration:

| Duration | Action |
|---|---|
| 0 – 3 s | No action — Nav2 recovery behaviours try to clear the path |
| ≥ 3 s continuous | Explorer blacklists current goal, pops queue, sends next frontier immediately |
| Clears | Timer resets; next block starts a fresh 3 s countdown |

### Nav2 — RPP Collision Detection

`use_collision_detection: false` in nav2_params.yaml. Safety stop provides real-time laser-based protection with no path-planning delay, making the RPP lookahead redundant and a source of false stops.

---

## 10. Goal Lifecycle Summary

| Event | Blacklisted? | Next goal |
|---|---|---|
| STATUS_SUCCEEDED + frontier cleared | No | Queue → rescan if empty |
| STATUS_SUCCEEDED + frontier NOT cleared (1st) | No | Queue → rescan if empty |
| STATUS_SUCCEEDED + frontier NOT cleared (2nd) | YES — 600 s, 1.60 m | Pop queue |
| STATUS_ABORTED — 1st time | YES — 300 s, 0.80 m | Pop queue |
| STATUS_ABORTED — 2nd+ same area | YES — 600 s, 1.60 m | Pop queue |
| STATUS_CANCELLED (our timeout / safety) | Already done before cancel | Pop queue |
| Safety blocked 3 s | YES — 300 s, 0.80 m | Pop queue immediately |
| Progress timeout (25 s) | YES — 300 s, 0.80 m | Pop queue same tick |
| Goal timeout (90 s) | YES — 300 s, 0.80 m | Pop queue same tick → rescan if empty |
| Frontier mapped mid-trip | **No** (clean cancel — area is fine) | Pop queue same tick → rescan if empty |

---

## 11. Debug Visualisation (`/frontier_debug`)

Published every tick, even when idle. Add a MarkerArray display in RViz2.

| Marker | Colour | Shows |
|---|---|---|
| Active blacklist zones | Red cylinder | Standard failures — radius 0.80 m, label "BL×{hits} {remaining}s" |
| Repeat-failure dead zones | Dark-red cylinder | Radius 1.60 m |
| Permanent dead zones | Black cylinder | Enclosed map pockets — label "PERM", never expires |
| Queued frontiers | Green spheres (#1 bright → #5 yellow) | Up to 5 standby candidates — label "Q{rank} {score:.2f}" |

---

## 12. Branch Overview (autonomous nav features)

| Feature | main | home_updated | jazzy | device-amey |
|---|---|---|---|---|
| Frontier explorer (basic) | Yes | Yes | Yes | Yes |
| Auto-calibration from map size | No | Yes | Yes | Yes |
| Progressive relaxation | No | Yes | Yes | Yes |
| Home recovery (return to start when stuck) | No | **Yes** | No | No |
| Safety stop node | No | No | No | Yes |
| Frontier standby queue | No | No | No | Yes |
| Queue used on every tick (not just failover) | No | No | No | Yes |
| Flood-fill permanent dead zones | No | No | No | Yes |
| Phantom success detection | No | No | No | Yes |
| Phantom threshold | — | — | — | 2 arrivals |
| Exponential blacklist timeout | No | No | No | Yes |
| Adaptive abort (1st vs 2nd+) | No | No | No | Yes |
| Far-frontier bonus | No | No | No | Yes |
| Map-relative large frontier threshold | No | No | No | Yes |
| Debug markers (/frontier_debug) | No | No | No | Yes |
| Gazebo Harmonic support | No | No | **Yes** | No |
| Explorer lines | 599 | 649 | 599 | 1255 |
