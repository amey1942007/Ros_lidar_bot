# Semantic Object Positioning — Full Mathematical Derivation

This document derives, step by step, how to compute the position of a
detected object in the SLAM map frame using only three inputs that are
already available on the robot:

1. A YOLO bounding box `(x_min, y_min, x_max, y_max)` in pixel coordinates
2. The live LiDAR scan `/scan`
3. The current robot pose from the TF tree (`map → base_footprint`)

No depth camera is needed. The LiDAR provides the depth, YOLO provides
the semantic class and horizontal direction, and TF provides the robot's
location in the world.

---

## 1. Coordinate Frames — The Foundation

Before any calculation, you must know which coordinate system each
measurement lives in. The robot has several frames:

```
map
 └─ odom                     ← accumulated wheel movement since start
     └─ base_footprint        ← robot contact point with ground
         └─ base_link          ← robot body centre
             ├─ laser_frame    ← LiDAR sensor origin
             └─ camera_link    ← camera body
                 └─ camera_link_optical   ← camera optical axis (Z forward)
```

### Frame definitions

**`map` frame**
- Origin: fixed in the world, set when SLAM is first initialised
- X: east (or arbitrary forward direction of the building)
- Y: north (90° CCW from X when viewed from above)
- Z: up
- Published by: SLAM Toolbox (`map → odom` transform)

**`base_footprint` frame**
- Origin: the point directly below the robot centre on the floor plane
- X: forward (direction robot faces)
- Y: left
- Z: up
- Published by: EKF node (`odom → base_footprint` transform)

**`base_link` frame**
- Coincides with `base_footprint` for this robot (zero-offset fixed joint)
- In general, `base_link` is the rigid body frame and `base_footprint` is
  its projection onto the ground

**`laser_frame`** (from `lidar.xacro`)
```
joint: laser_joint
  parent: base_link
  origin xyz: -0.001839  0.002589  0.104922   (metres)
  origin rpy:  0          0         0
```
- The LiDAR is located 0.105 m ABOVE `base_link`, essentially at the same
  x,y position (< 2 mm offset)
- The scan plane is horizontal (z = constant in `laser_frame`)
- Angle 0 rad = forward (+X), increases counter-clockwise when viewed from above

**`camera_link`** (from `camera.xacro`)
```
joint: camera_joint
  parent: base_link
  origin xyz: 0.082  0  0.250   (metres)
  origin rpy: 0      0  0
```
- Camera body is 82 mm AHEAD of `base_link` centre and 250 mm ABOVE it
- Faces in the +X direction (same forward direction as the robot)

**`camera_link_optical`**
```
joint: camera_optical_joint
  parent: camera_link
  origin xyz: 0.03  0  0
  origin rpy: -π/2  0  -π/2
```
- This frame applies the "robotics → optics" rotation convention
- In optics: Z = forward (into the scene), X = right, Y = down
- The rpy `(-π/2, 0, -π/2)` rotates so that the camera's Z axis aligns
  with `camera_link`'s +X axis

---

## 2. The Pinhole Camera Model

A pinhole camera maps a 3D point in the camera frame to a 2D pixel using
the projection equations:

```
Given a 3D point P = (X_c, Y_c, Z_c) in camera_link_optical frame:

  u = f_x * (X_c / Z_c) + c_x       (pixel column, horizontal)
  v = f_y * (Y_c / Z_c) + c_y       (pixel row, vertical)
```

where:
- `(u, v)` is the pixel coordinate
- `(c_x, c_y)` is the principal point (usually image centre = (320, 240))
- `f_x`, `f_y` are focal lengths in pixels

### Computing focal length from field of view

The camera is configured with:
```
horizontal FOV = 62° = 1.089 radians
image width    = 640 pixels
```

The relationship between FOV and focal length for a pinhole camera:

```
FOV_H = 2 * arctan(image_width / (2 * f_x))

Solving for f_x:
  f_x = image_width / (2 * tan(FOV_H / 2))
      = 640 / (2 * tan(1.089 / 2))
      = 640 / (2 * tan(0.5445))
      = 640 / (2 * 0.5936)
      = 640 / 1.1872
      ≈ 539  pixels
```

Similarly for vertical (vertical FOV ≈ 48° for a 640×480 4:3 sensor with 62° HFOV):
```
FOV_V ≈ 2 * arctan((480/640) * tan(FOV_H/2))
      ≈ 48°

f_y = 480 / (2 * tan(48° / 2))
    ≈ 480 / (2 * tan(24°))
    ≈ 480 / (2 * 0.4452)
    ≈ 539  pixels
```

Both focal lengths are equal because the pixel aspect ratio is 1:1.

---

## 3. Inverting the Projection — Pixel to Direction

We want the INVERSE operation: given pixel `(u, v)`, find the direction in
the camera frame that the pixel corresponds to.

From the projection equations:
```
X_c / Z_c = (u - c_x) / f_x
Y_c / Z_c = (v - c_y) / f_y
```

This defines a RAY from the camera origin through the pixel. Without depth
information, we only know the direction, not the distance. The ray
direction (unit vector) in the optical frame is:

```
d_x = (u - c_x) / f_x
d_y = (v - c_y) / f_y
d_z = 1.0                    (by convention, pointing into the scene)

Normalised:
  magnitude = sqrt(d_x² + d_y² + d_z²)
  ray_direction = (d_x/magnitude, d_y/magnitude, d_z/magnitude)
```

For the BOUNDING BOX, use its horizontal centre pixel:
```
u_center = (x_min + x_max) / 2
v_center = (y_min + y_max) / 2

d_x = (u_center - 320) / 539
d_y = (v_center - 240) / 539
d_z = 1.0
```

---

## 4. Converting Camera Ray to LiDAR Angle

The camera and LiDAR are NOT at the same position or orientation.
We need to bring the camera ray direction into the `laser_frame`.

### Step 4a — optical frame → base_link

The `camera_link_optical` to `camera_link` rotation is `rpy = (-π/2, 0, -π/2)`.

This rotation matrix (R_optical_to_camera) converts a vector from the
optical frame (Z-forward, X-right, Y-down) to the camera body frame
(X-forward, Y-left, Z-up):

```
R_opt_to_cam = Rz(-π/2) * Rx(-π/2)

Rz(-π/2) = [  0   1   0 ]     (rotation about Z by -90°)
            [ -1   0   0 ]
            [  0   0   1 ]

Rx(-π/2) = [  1   0   0 ]     (rotation about X by -90°)
            [  0   0   1 ]
            [  0  -1   0 ]

R_opt_to_cam = Rz(-π/2) * Rx(-π/2)
             = [  0   1   0 ] * [  1   0   0 ]
               [ -1   0   0 ]   [  0   0   1 ]
               [  0   0   1 ]   [  0  -1   0 ]

             = [  0   0   1 ]
               [ -1   0   0 ]
               [  0  -1   0 ]
```

Applying to our ray direction `d = (d_x, d_y, d_z)`:

```
d_cam = R_opt_to_cam * d_opt

d_cam_x =  d_z      (what was Z-forward becomes X-forward)
d_cam_y = -d_x      (what was X-right becomes -Y, i.e. right in camera = right in robot)
d_cam_z = -d_y      (what was Y-down becomes -Z, i.e. down in image = down in robot)
```

Since `camera_joint` has zero rotation (rpy = 0,0,0) from `base_link`,
and `camera_link` is just offset (no rotation) from `base_link`:
```
d_base = d_cam      (directions are the same, only position offsets differ)
```

### Step 4b — Horizontal angle in base_link frame

The LiDAR scans horizontally (Z=0 plane in `laser_frame`). We only care
about the HORIZONTAL component of the direction vector:

```
horizontal_angle = atan2(d_base_y, d_base_x)
                 = atan2(-d_x, d_z)           (substituting from Step 4a)
                 = atan2(-(u_center - 320)/539, 1.0)
```

For small angles (object not at extreme edge of frame):
```
horizontal_angle ≈ -(u_center - 320) / 539    (in radians)
```

The negative sign: when the object is to the RIGHT in the image
(u > 320), the angle is negative (clockwise = right = negative in
ROS2 convention where CCW is positive).

---

## 5. LiDAR Depth Lookup

The LiDAR publishes `sensor_msgs/LaserScan` on `/scan`:

```
LaserScan fields:
  angle_min       ← angle of the first beam (radians)
  angle_max       ← angle of the last beam (radians)
  angle_increment ← angular step between beams (radians)
  ranges[]        ← array of distance readings (metres)
  range_min       ← minimum valid reading (0.3 m for this robot)
  range_max       ← maximum valid reading (12.0 m)
```

For this robot: 360 beams covering -π to +π, so `angle_increment = 2π/360 ≈ 0.01745 rad`.

To find the depth at our `horizontal_angle`:

```python
idx = round((horizontal_angle - scan.angle_min) / scan.angle_increment)
idx = clamp(idx, 0, len(scan.ranges) - 1)

depth = scan.ranges[idx]

# Validate reading
if depth < scan.range_min or depth > scan.range_max or isnan(depth) or isinf(depth):
    depth = None   # invalid reading, skip this detection
```

**Why the depth might be invalid:**
- `nan` or `inf`: the laser ray hit nothing within max range (looking out a window, or the object is transparent)
- `< range_min`: the object is too close (< 0.3 m) for the sensor to measure

---

## 6. Object Position in base_link Frame

Now we have:
- `depth` (metres) — distance from the LiDAR origin to the object
- `horizontal_angle` (radians) — direction from the LiDAR to the object

The LiDAR origin is at `laser_frame`, which is offset from `base_link` by
`(−0.002, 0.003, 0.105)` metres. For a ground-navigating robot, the x,y
offset is negligible (< 3 mm). The z offset (height of the LiDAR) doesn't
affect the 2D floor position.

Object position in `base_link` frame:

```
obj_x_base = depth * cos(horizontal_angle)
obj_y_base = depth * sin(horizontal_angle)
obj_z_base = 0.0          (floor-level approximation)
```

Numerically, if `horizontal_angle = 0.15 rad` and `depth = 2.3 m`:
```
obj_x_base = 2.3 * cos(0.15) = 2.3 * 0.9888 = 2.274 m  (forward)
obj_y_base = 2.3 * sin(0.15) = 2.3 * 0.1494 = 0.344 m  (left)
```

---

## 7. Transforms — How to Get to Map Frame

This is the most important section. All robot navigation happens in the
`map` frame. To place the detected object on the map, we must apply the
chain of transforms from `base_link` to `map`.

### What is a Transform?

A transform describes "how do I go from coordinate system A to coordinate
system B?" It has two components:

```
Transform T_A_to_B = {
    translation: (tx, ty, tz)   ← where is A's origin in B's coordinates?
    rotation:    quaternion      ← how is A's orientation relative to B?
}
```

Given a point P expressed in frame A, its coordinates in frame B are:
```
P_B = R * P_A + t

where R is the 3×3 rotation matrix from the quaternion,
      t is the translation vector (tx, ty, tz)
```

### The TF Tree for this robot

```
map → odom → base_footprint → base_link → laser_frame
                                        → camera_link → camera_link_optical
```

To transform from `base_link` to `map`, we chain the transforms:

```
T_map = T(map→odom) * T(odom→base_footprint) * T(base_footprint→base_link)
```

In practice, TF2 does this chaining automatically. You simply ask:
"give me the transform from `map` to `base_link`" and TF2 looks up the
tree and composes all intermediate transforms.

### The 2D case (flat floor robot)

Because this is a ground robot operating on a flat floor, we only need
the 2D version of the transform:

```
Robot state in map frame: (robot_x, robot_y, robot_yaw)
  robot_x    ← east position in map (metres)
  robot_y    ← north position in map (metres)
  robot_yaw  ← heading angle (radians, CCW from east is positive)
```

Given an object at `(obj_x_base, obj_y_base)` in the robot's frame,
its position in the map frame is:

```
obj_x_map = robot_x + obj_x_base * cos(robot_yaw)
                     - obj_y_base * sin(robot_yaw)

obj_y_map = robot_y + obj_x_base * sin(robot_yaw)
                     + obj_y_base * cos(robot_yaw)
```

This is a 2D rotation (by `robot_yaw`) followed by a translation
(by `robot_x, robot_y`). The rotation aligns the robot's forward
direction with its actual heading in the map.

### Where robot_yaw comes from

The robot's orientation from TF is stored as a QUATERNION. The quaternion
`(qx, qy, qz, qw)` represents 3D rotation. For a flat-floor robot, only
`qz` and `qw` are non-zero (rotation about Z axis only):

```
robot_yaw = atan2(2 * (qw * qz + qx * qy),
                  1 - 2 * (qy * qy + qz * qz))
```

This formula extracts the yaw (rotation about Z) from the quaternion.
It is the standard formula for "quaternion to Euler angles (Z rotation)".

### Numerical example

Suppose:
- Robot is at map position `(3.5, 2.0)` facing `45°` (northeast), so `robot_yaw = π/4 ≈ 0.785 rad`
- Object detected at `obj_x_base = 2.274 m`, `obj_y_base = 0.344 m`

Rotation:
```
cos(0.785) = 0.707,  sin(0.785) = 0.707

obj_x_map = 3.5 + 2.274 * 0.707 - 0.344 * 0.707
          = 3.5 + 1.608 - 0.243
          = 4.865 m

obj_y_map = 2.0 + 2.274 * 0.707 + 0.344 * 0.707
          = 2.0 + 1.608 + 0.243
          = 3.851 m
```

The chair that was 2.27 m ahead and 0.34 m to the left of the robot is
located at map coordinates `(4.865, 3.851)`.

---

## 8. Getting the Object's Footprint Size

The bounding box gives us the horizontal extent of the object in the image.
Using the same projection math we can estimate the object's physical width.

The bounding box spans from pixel `x_min` to `x_max`. The corresponding
angular span in the horizontal direction:

```
angle_left  = atan2(x_min - 320, 539)
angle_right = atan2(x_max - 320, 539)
angular_width = angle_right - angle_left   (radians)
```

At distance `depth`, the physical width of the object is:

```
physical_width = 2 * depth * tan(angular_width / 2)
               ≈ depth * angular_width    (for small angles)
```

For example, an object at 2.3 m spanning 80 pixels wide:
```
angular_width = atan2(40, 539) - atan2(-40, 539)
              = 2 * atan2(40, 539)
              = 2 * 0.0740
              = 0.148 rad

physical_width = 2.3 * tan(0.148 / 2) * 2
               ≈ 2.3 * 0.148
               = 0.340 m   (34 cm wide object)
```

A MORE ACCURATE method: project ALL LiDAR points whose angles fall within
`[angle_left, angle_right]` into the base_link frame, then take the
range extent of those points:

```python
object_points = []
for i, r in enumerate(scan.ranges):
    beam_angle = scan.angle_min + i * scan.angle_increment
    if angle_left <= beam_angle <= angle_right and scan.range_min < r < scan.range_max:
        px = r * cos(beam_angle)
        py = r * sin(beam_angle)
        object_points.append((px, py))

if object_points:
    xs = [p[0] for p in object_points]
    ys = [p[1] for p in object_points]
    footprint_width = max(ys) - min(ys)   # width (lateral extent)
    footprint_depth = max(xs) - min(xs)   # depth (along robot forward)
    centroid_x = sum(xs) / len(xs)
    centroid_y = sum(ys) / len(ys)
```

This is more accurate because it uses actual LiDAR geometry rather than
image-plane approximations.

---

## 9. Complete Algorithm Summary

```
INPUT:
  bbox       = YOLO bounding box (x_min, y_min, x_max, y_max) in pixels
  scan       = sensor_msgs/LaserScan from /scan
  robot_pose = (robot_x, robot_y, robot_yaw) from TF map→base_footprint

CONSTANTS:
  image_width  = 640
  image_height = 480
  cx = 320, cy = 240         (principal point)
  fx = fy = 539              (focal length in pixels)
  fov_h = 1.089 rad (62°)

STEP 1 — Bounding box centre pixel
  u = (x_min + x_max) / 2
  v = (y_min + y_max) / 2

STEP 2 — Camera ray direction (in optical frame)
  d_x = (u - cx) / fx
  d_y = (v - cy) / fy
  d_z = 1.0

STEP 3 — Rotate to base_link frame (using R_opt_to_base)
  d_forward = d_z         (Z optical = X base)
  d_left    = -d_x        (X optical = -Y base, so right=negative)

STEP 4 — Horizontal angle in base_link / laser_frame
  horizontal_angle = atan2(d_left, d_forward)
                   = atan2(-d_x, d_z)
                   = atan2(-(u - 320)/539, 1.0)

STEP 5 — LiDAR depth at that angle
  idx   = round((horizontal_angle - scan.angle_min) / scan.angle_increment)
  depth = scan.ranges[idx]
  VALIDATE: depth must be in [scan.range_min, scan.range_max], not NaN/Inf

STEP 6 — Object position in base_link frame
  obj_x_base = depth * cos(horizontal_angle)
  obj_y_base = depth * sin(horizontal_angle)

STEP 7 — Transform to map frame
  obj_x_map = robot_x + obj_x_base * cos(robot_yaw) - obj_y_base * sin(robot_yaw)
  obj_y_map = robot_y + obj_x_base * sin(robot_yaw) + obj_y_base * cos(robot_yaw)

STEP 8 — (Optional) estimate footprint
  Collect all LiDAR points in [angle_left, angle_right]
  Compute centroid and min/max extents in base_link frame
  Transform centroid to map frame using same rotation/translation as Step 7

OUTPUT:
  map position: (obj_x_map, obj_y_map)
  class label:  e.g. "chair"
  confidence:   YOLO confidence score
  footprint:    width × depth in metres
```

---

## 10. Sources of Error and How to Mitigate Them

### Error 1 — Camera-LiDAR axis offset

The camera is 82 mm ahead of `base_link`. The LiDAR is essentially AT
`base_link` (< 2 mm offset). For an object at distance `D`:

```
Angular error ≈ arctan(0.082 / D)
At D = 1.0 m: error ≈ 4.7°  (significant for close objects)
At D = 2.0 m: error ≈ 2.3°
At D = 5.0 m: error ≈ 0.9°
```

**Mitigation**: Only use detections where depth > 1.5 m, OR properly
transform the camera optical centre to `laser_frame` before computing
the angle. The latter requires applying the full TF chain:
```
p_in_laser = T(laser_frame → base_link)^{-1} * T(base_link → camera_link) * p_camera_origin
```

### Error 2 — LiDAR angular quantisation

The LiDAR has 360 beams over 360°, so `angle_increment = 1°`. At 3 m:
```
1° of angle error = 3.0 * tan(1° * π/180) = 3.0 * 0.01745 = 0.052 m
```

So position error is ≈ 5 cm at 3 m due to beam spacing alone.

**Mitigation**: Average over multiple detections (deduplication step).

### Error 3 — LiDAR and camera not time-synchronised

The camera captures at 30 Hz, LiDAR at 20 Hz. A 50 ms time difference
while moving at 0.22 m/s produces:
```
position error = 0.22 * 0.05 = 0.011 m  (1 cm)
```

This is negligible and can be ignored in practice.

### Error 4 — Robot pose uncertainty

The TF pose `(robot_x, robot_y, robot_yaw)` comes from SLAM + EKF. The
localisation error is typically < 5–10 cm in position and < 2° in heading.
At object distance of 3 m, a 2° heading error produces:
```
lateral error = 3.0 * tan(2° * π/180) = 3.0 * 0.0349 = 0.105 m
```

**Mitigation**: The deduplication averaging (weighted average of multiple
observations from different robot positions) naturally reduces this error.

---

## 11. What TF2 Does Internally (the full chain)

When you call `tf_buffer.lookup_transform('map', 'base_link', ...)`:

1. TF2 looks up the tree: `map → odom → base_footprint → base_link`
2. It retrieves each individual transform at the requested timestamp
3. It composes them by matrix multiplication:

```
T_map_to_base = T(map→odom) ∘ T(odom→base_footprint) ∘ T(base_footprint→base_link)
```

Each transform is stored as `(translation, quaternion)`. Composition means:

```
T_AB ∘ T_BC = T_AC

Translation:  t_AC = t_AB + R_AB * t_BC
Rotation:     q_AC = q_AB ⊗ q_BC    (quaternion product)
```

The quaternion product `⊗` is:
```
q_1 ⊗ q_2 = (
    w1*w2 - x1*x2 - y1*y2 - z1*z2,   ← w
    w1*x2 + x1*w2 + y1*z2 - z1*y2,   ← x
    w1*y2 - x1*z2 + y1*w2 + z1*x2,   ← y
    w1*z2 + x1*y2 - y1*x2 + z1*w2    ← z
)
```

This is why you should ALWAYS use TF2 for transforms rather than doing
them manually — the chaining, interpolation, and timestamp handling are
all handled correctly.

### Why quaternions and not Euler angles?

Euler angles (roll, pitch, yaw) suffer from "gimbal lock" — when two
rotation axes align, one degree of freedom is lost. Quaternions avoid
this. They represent any 3D rotation as a unit vector in 4D space:
```
q = (x, y, z, w) where x² + y² + z² + w² = 1
```

For a flat-floor robot rotating about Z only:
```
q = (0, 0, sin(yaw/2), cos(yaw/2))
```

To extract yaw from a general quaternion (handles all cases):
```
yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
```

---

## 12. Sensor Frame Locations (summary table)

| Frame | x (m) | y (m) | z (m) | Roll | Pitch | Yaw |
|---|---|---|---|---|---|---|
| base_footprint | 0 | 0 | 0 | 0 | 0 | 0 |
| base_link | 0 | 0 | 0 | 0 | 0 | 0 |
| laser_frame | -0.002 | +0.003 | +0.105 | 0 | 0 | 0 |
| camera_link | +0.082 | 0 | +0.250 | 0 | 0 | 0 |
| camera_link_optical | +0.030 rel | 0 | 0 | -π/2 | 0 | -π/2 |

All positions relative to `base_link`. The laser_frame and camera_link
are both forward-facing (same yaw as base_link = 0).
