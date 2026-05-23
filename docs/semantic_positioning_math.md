# How We Find Where a Detected Object Is on the Map

---

## What we have as inputs

1. **YOLO bounding box** — tells us where in the camera IMAGE the object is (in pixels)
2. **LiDAR scan** — tells us how far away things are in each direction (in metres)
3. **Robot position on the map** — tells us where the robot itself is right now

Goal: use all three to figure out where the object is on the SLAM map.

---

## Step 1 — Which direction is the object? (pixels → angle)

Your camera image is 640 pixels wide and covers 62 degrees of view.

```
62 degrees spread across 640 pixels
→ each pixel = 62 / 640 = 0.097 degrees per pixel
```

The centre of the image (pixel 320) is straight ahead — 0 degrees.
Pixel 0 is far left, pixel 640 is far right.

So for any pixel, the angle from straight ahead is:

```
angle = (pixel_x - 320) × (62 / 640)    ← in degrees
      = (pixel_x - 320) × 0.00170       ← in radians (just multiply by π/180)
```

**Example:** YOLO finds a chair. Bounding box horizontal centre is at pixel 400.

```
angle = (400 - 320) × 0.00170
      = 80 × 0.00170
      = 0.136 radians = 7.8 degrees to the right
```

That's it. This angle tells you which direction to look in the LiDAR scan.

---

## Step 2 — How far away is the object? (LiDAR depth)

The LiDAR fires 360 laser beams in a full circle. Each beam returns one number: how many metres until it hit something.

```
scan.ranges[i]  =  distance at angle i
```

The angle 0 radians = straight ahead. Going counter-clockwise is positive.
(So right of the robot = negative angle, left = positive angle.)

To get the distance to our chair:

```python
angle_radians = (400 - 320) * (1.089 / 640)   # 0.136 rad = slightly right

# Find which beam index corresponds to this angle
index = round((angle_radians - scan.angle_min) / scan.angle_increment)

depth = scan.ranges[index]   # e.g. 2.3 metres
```

---

## Step 3 — x,y position relative to the robot (cos and sin)

You know the direction (0.136 rad) and the distance (2.3 m).
This is just basic trigonometry — exactly like finding a point on a circle:

```
x = depth × cos(angle)   ← how far FORWARD the object is from the robot
y = depth × sin(angle)   ← how far to the LEFT the object is from the robot
```

For our chair:
```
x = 2.3 × cos(0.136) = 2.3 × 0.991 = 2.28 m  forward
y = 2.3 × sin(0.136) = 2.3 × 0.136 = 0.31 m  to the right (negative because sin of a negative angle)
```

At this point you know the chair is 2.28 m in front and 0.31 m to the right
**relative to the robot**. But the robot is somewhere on the map, not at the origin.

---

## Step 4 — Convert to map position (the only tricky part)

The robot knows:
- Its position on the map: `robot_x`, `robot_y` (in metres from map origin)
- Which direction it is facing: `robot_yaw` (the angle it has rotated from east)

**What is yaw?** Imagine looking at the robot from above. If the robot faces east, yaw = 0. If it turns left 90 degrees to face north, yaw = 90 degrees. Yaw is just the robot's compass heading.

**Roll and pitch** (which you asked about) — these are the OTHER two rotation axes. Roll = the robot tilting sideways (left wheel going into a dip). Pitch = the robot tilting nose-up or nose-down (going up a ramp). For a flat-floor robot, roll and pitch are always zero. We only ever care about yaw.

Now: the x,y we computed in Step 3 assumes the robot is facing east (yaw = 0). But the robot might be facing 45 degrees northeast. So we need to ROTATE our x,y by the robot's actual heading before adding it to the robot's map position.

Think of it like this: you measure something 3 metres in front of you. But "in front" means different things depending on which way you're facing. You have to rotate that "3 metres forward" by your own heading to get the actual map direction.

```
object_map_x = robot_x  +  (x × cos(yaw))  −  (y × sin(yaw))
object_map_y = robot_y  +  (x × sin(yaw))  +  (y × cos(yaw))
```

**Example:** Robot is at map position (3.0, 2.0) facing 45 degrees (yaw = 0.785 radians).
Object is at x=2.28 m forward, y=−0.31 m (slightly right).

```
object_map_x = 3.0  +  (2.28 × cos(0.785))  −  (−0.31 × sin(0.785))
             = 3.0  +  (2.28 × 0.707)        −  (−0.31 × 0.707)
             = 3.0  +  1.61                   +  0.22
             = 4.83 m

object_map_y = 2.0  +  (2.28 × sin(0.785))  +  (−0.31 × cos(0.785))
             = 2.0  +  (2.28 × 0.707)        +  (−0.31 × 0.707)
             = 2.0  +  1.61                   −  0.22
             = 3.39 m
```

The chair is at map position (4.83, 3.39). Done.

---

## Where does robot_x, robot_y, robot_yaw come from?

The robot always knows its position and heading from the TF system (the same system that RViz uses to show the robot on the map). You ask:

```python
transform = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

robot_x   = transform.transform.translation.x
robot_y   = transform.transform.translation.y
# heading comes from the quaternion (explained below)
```

The position (x, y) is straightforward. The heading is stored as a **quaternion** — a 4-number format for rotations. You don't need to understand why, just know how to extract the yaw from it:

```python
qz = transform.transform.rotation.z
qw = transform.transform.rotation.w
robot_yaw = 2 × atan2(qz, qw)   # extracts the yaw angle in radians
```

This works because for a flat-floor robot that only rotates in the horizontal plane, the quaternion simplifies to just two numbers (qz and qw), and the formula above recovers the yaw angle from them.

---

## What you get right and what's the one thing missing

**What you described — 100% correct:**
- 640 pixels = 62 degrees → divide to get radians per pixel ✓
- Pixel offset from centre (320) × radians per pixel = angle ✓
- Look up that angle in the LiDAR scan → depth ✓
- depth × cos(angle) = forward distance, depth × sin(angle) = left distance ✓

**The one thing your summary missed:**
Step 4 — rotating the robot-relative position by the robot's heading before adding it to the robot's map position. If the robot is always facing east this step is trivial (cos(0)=1, sin(0)=0 so the formula just adds). But when the robot is facing a different direction, skipping this step puts the object in the wrong place on the map.

---

## One practical concern — the camera is not at the same spot as the LiDAR

The camera sits 82 mm AHEAD of the robot centre. The LiDAR is at the robot centre. For an object 2 metres away, this 82 mm offset causes a 2.4 degree pointing error — meaning the angle you computed from the camera might point to a slightly wrong beam in the LiDAR scan.

For objects farther than about 1.5 metres this error is small enough to ignore (< 5 cm position error). For very close objects it matters.

The fix: account for the camera-to-LiDAR offset using the TF system, but this adds complexity. For most use cases (detecting desks, chairs, people at > 1.5 m), ignoring it is fine.

---

## Complete summary in 6 lines

```
1. angle  = (bbox_center_x - 320) × (1.089 / 640)
2. index  = round((angle - scan.angle_min) / scan.angle_increment)
3. depth  = scan.ranges[index]
4. x_robot = depth × cos(angle)    ← forward from robot
5. y_robot = depth × sin(angle)    ← left from robot
6. x_map = robot_x + x_robot×cos(yaw) - y_robot×sin(yaw)
   y_map = robot_y + x_robot×sin(yaw) + y_robot×cos(yaw)
```
