## Project Overview

This project is a half-scale stair-climbing wheelchair model developed by NishCorp Technology. The system includes a real-time balancing algorithm for maintaining stability on uneven terrain and slopes in wheeled mode, as well as body balancing during stair climbing in track mode. The robot also performs stair detection using depth camera data, enabling it to identify individual steps and transition smoothly between wheel and track locomotion modes.

---

## Objectives

- Develop a real-time balancing and body-levelling algorithm for stable motion on uneven terrain, slopes, and stairs
- Implement a depth camera point cloud processing pipeline for staircase and individual step detection
- Develop a closed-form 2-DOF inverse kinematics (IK) solver for dynamic leg positioning and posture adjustment
- Implement IMU-based automatic balancing and body levelling using PID control
- Design a smooth autonomous transition between wheel mode and track mode
- Build a finite state machine for autonomous stair-climbing operation
- Provide keyboard teleoperation for manual control, testing, and override
- Develop a modular ROS 2 architecture with runtime-tunable parameters

---

## Outcomes and Deliverables

**Four ROS 2 nodes** forming a complete perception-to-actuation pipeline:

- `voxel_filter_node` — reduces raw depth camera data from ~307,000 to ~15,000–30,000 points per frame using a 3 cm voxel grid, making downstream processing feasible in real time
- `stair_detector_node` — detects and classifies staircases using iterative RANSAC plane segmentation, EMA temporal tracking, and a regularity check; publishes step geometry and approach state
- `controller_node` — controller integrating IK-based leg positioning, dual-mode PID body levelling and smooth mode transition blending
- `teleop_node` — keyboard teleoperation with full body posture control and mode override

**robot description:**
- URDF/Xacro model with 2 hip joints, 2 knee joints, 2 front wheels and 2 rear free wheels
- Gazebo Jazzy physics with `gz_ros2_control` effort and velocity interfaces
- Integrated IMU and OAK-D Lite stereo depth camera (mounted rear-facing, rotated 180°)

---

## Features

- Autonomous stair detection using depth camera point clouds and iterative RANSAC plane fitting
- EMA temporal tracking of detected steps — filters noise and transient false positives
- Smooth transition between wheeled driving (flat ground) and track mode (stair climbing)
- Real-time body levelling using IMU feedback and cascaded PID control
- 2-DOF closed-form inverse kinematics for dynamic leg height and posture adjustment
- Independent left/right leg control for roll compensation
- 3-phase stair-climbing finite state machine with transitioning in between mode
- Full physics simulation in Gazebo Jazzy with effort-controlled joints
- Keyboard teleoperation with body height, pitch, and roll adjustment
- All parameters tunable at runtime via `ros2 param set`
  
---

## Requirements

- ROS 2 Jazzy
- Gazebo Harmonic
- `gz_ros2_control`
- `ros2_control` and `ros2_controllers`
- PCL (Point Cloud Library)
- `tf2`
- `pcl_conversions`
- `visualization_msgs`
- Colcon build system
- RViz2

---

## Installation

**1. Clone the repository into your ROS 2 workspace:**

```bash
cd ~/ros2_ws/src
git clone https://github.com/Siddhant-nishcorptech/Hetvi-internship-work.git
```

**2. Install dependencies:**

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

**3. Build the package:**

```bash
cd ~/ros2_ws
colcon build --packages-select my_bot_urdf_description
```

**4. Source your workspace:**

```bash
source install/setup.bash
```
---

## Usage

All nodes should be started in separate terminals, each with the workspace sourced.

**Terminal 1 — Launch the Gazebo simulation:**

```bash
ros2 launch my_bot_urdf_description gazebo.launch.py
```

**Terminal 2 — Start the perception pipeline:**

```bash
ros2 run my_bot_urdf_description stair_detector_node
```

**Terminal 3 — Start teleoperation:**

```bash
ros2 run my_bot_urdf_description teleop_node
```

**Terminal 4 — Visualize in RViz:**

```bash
ros2 launch my_bot_urdf_description display.launch.py
```
**Verifying everything is running:**

```bash
ros2 node list
# /voxel_filter_node
# /stair_detector_node
# /controller_node
# /teleop_node

ros2 topic echo /step_state     # prints 0 when no stairs visible
ros2 topic echo /stair_info     # prints array of 7 zeros
```

---

## Teleop Controls

```
Moving around:
   ↑  : Forward
   ↓  : Backward
   ←  : Turn left
   →  : Turn right

Body Control:
   w / s  : Body height up / down   (adjusts H by ±0.01 m per keypress)
   t / g  : Pitch forward / backward
   a / d  : Roll left / right

Mode Switching:
   p  : Toggle track mode on / off  (also auto-triggered by stair detection)
   l  : Manually trigger stair climb sequence

Speed Scaling:
   q / z  : Increase / decrease all speeds by 10%
   e / c  : Increase / decrease turn speed only by 10%

CTRL-C to quit
```

---

## Nodes

### Voxel Filter Node

**File:** `src/voxel_filter_node.cpp`

The OAK-D Lite depth camera at VGA resolution produces approximately 307,200 points per frame. Feeding this volume directly into RANSAC plane fitting would be too slow for real-time use. This node applies a VoxelGrid filter — 3D space is divided into uniform cubic cells of side length `leaf_size`. All points inside the same voxel are replaced by their centroid. At the default 3 cm leaf size, the output contains roughly 15,000–30,000 points while preserving the large-scale geometry of stair edges and flat surfaces. The header (frame ID and timestamp) is forwarded unchanged so downstream nodes retain correct TF context.

### Stair Detector Node

**File:** `src/stair_detector_node.cpp`

**Published step states on `/step_state`:**

| Value | Meaning | Triggered when |
|-------|---------|----------------|
| `0` | No staircase | No confident steps, or distance out of range |
| `1` | First step ahead | 1+ steps detected, nearest step 0.90–1.50 m away |
| `2` | Mid-climb | 2+ steps detected, nearest step 0.25–0.45 m away |
| `3` | Platform reached | Final tread ≥ 0.35 m deep, nearest step 0.50–0.89 m away |

RViz markers are published on `/stair_edges` — green LINE_STRIP for the front (convex) edge of each step, red for the back (concave) edge. 

### Controller Node

**File:** `src/controller_node.cpp`

- The controller node continuously processes IMU feedback, balancing control, inverse kinematics, and locomotion commands. The controller computes the control timestep (dt) while limiting it to avoid timer jitter, updates the smooth transition parameter between wheel and track modes, and applies low-pass filtering to the IMU roll and pitch measurements for stable balancing.
- Using PID-based body levelling, the controller generates pitch and roll correction values that are applied to the inverse kinematics of both legs. Separate IK targets are computed for wheel mode and track mode, and these targets are smoothly blended using the transition parameter for seamless mode switching. During autonomous stair climbing, the controller activates the finite state machine (FSM), which overrides normal joint commands to execute the climbing sequence.
- The node also publishes wheel velocity commands and track speed commands based on the current locomotion mode, while final joint torques are generated using JointPD controllers and sent to the robot actuators.

---

## Stair Detection Logic

The stair detector processes each incoming point cloud through four sequential stages:

**Stage 1 — ROI Crop**

Two PassThrough filters narrow the cloud to the region where stairs can actually appear:
- X axis (depth): 0.05–3.00 m forward. The 0.05 m lower bound prevents the robot's own chassis from appearing; 3.00 m avoids processing distant irrelevant geometry.
- Z axis (height): −0.60 to +0.50 m. Captures the floor level and approximately one floor of stairs while discarding the ceiling.

If fewer than `min_plane_inliers` points remain after cropping, all tracked steps are decayed and the frame is skipped.

**Stage 2 — Iterative RANSAC Plane Fitting**

`pcl::SACSegmentation` is configured with model `SACMODEL_PERPENDICULAR_PLANE` — it only fits planes whose normal is within 0.30 radians of vertical, which automatically rejects walls and ramps.  Each iteration finds the dominant plane, extracts its inlier points, removes them from the cloud, and checks the plane against three acceptance filters:

Accepted planes are stored with centroid position, bounding box, inlier count, and tread dimensions, then sorted by height ascending before tracking.

**Stage 3 — EMA Temporal Tracker**

Raw per-frame detections are noisy. The tracker maintains a list of `TrackedStep` objects that persist across frames:

- **Matching:** each new detection is matched to the nearest existing tracked step by height, within `height_match_tol` = 0.025 m. This tolerance is deliberately smaller than half the minimum riser height (~0.020 m) so a detection never migrates to the wrong step.
- **Update (matched):** all geometry fields are blended with EMA α = 0.25 — new measurements contribute 25%, history contributes 75%. Confidence increments by 2, capped at 12.
- **Insert (unmatched):** a new tracked step is created with confidence = 1.
- **Decay (not seen):** confidence decrements by 1. Steps reaching 0 are pruned.

Only steps with `confidence ≥ min_confidence` (default 4) are published, meaning a step must be consistently detected across at least 2 frames before influencing the robot's behaviour.

**Stage 4 — Regularity Check**

To distinguish a true staircase from random horizontal surfaces (shelves, table edges), the height differences between consecutive confident steps are measured. The **coefficient of variation** (CV = standard deviation ÷ mean) of those riser heights is computed. If CV ≤ `regularity_max_cv` (default 0.40), the set is flagged as a staircase. A CV of 0.40 tolerates approximately ±40% variation around the mean riser, handling imperfect detection while rejecting clearly non-uniform surfaces.

**Published stair geometry on `/stair_info`:**

| Index | Field | Description |
|-------|-------|-------------|
| `[0]` | `n` | Number of confident steps visible |
| `[1]` | `mean_rise` | Average riser height (m) |
| `[2]` | `mean_run` | Average tread depth (m) |
| `[3]` | `mean_width` | Average tread width (m) |
| `[4]` | `dist_first` | Distance to the nearest step (m) |
| `[5]` | `height_last` | Height of the highest step (m) |
| `[6]` | `dist_last` | Distance to the highest step (m) |

---

## Inverse Kinematics

Each leg has two revolute joints — a hip and a knee. Given a desired foot position relative to the hip joint, the IK solver computes the two joint angles that place the foot exactly at that position.

**Coordinate convention:**
- Origin: hip joint
- `x`: forward (positive = in front of robot)
- `z`: vertical (negative = downward toward ground)

**Closed-form solution for a 2-link planar arm:**

```
r  = √(x² + z²)
       ← Euclidean distance from hip to foot target

cos_θ = (L1² + L2² − r²) / (2 · L1 · L2)
       ← cosine rule; clamped to [−1, 1] to handle numerical boundary cases

q2 = −( π − arccos(cos_θ) )
       ← knee angle; negative = knee bends rearward

q1 = atan2(x, −z) − arccos( (L1² + r² − L2²) / (2 · L1 · r) )
       ← hip angle
```

**Link lengths:** L1 = 0.2442 m (upper leg), L2 = 0.175 m (lower leg)

The `cos_theta` clamp guards against floating-point errors when the target is exactly at maximum reach (r = L1 + L2), which would otherwise produce NaN via `arccos` of a value slightly above 1.0.

**Left / right leg differential for roll compensation:**

```
delta_z     = (track_width / 2) · tan(imu_roll) + roll_pid_output

z_right = −H − delta_z      ← right leg extends when rolling right
z_left  = −H + delta_z      ← left leg retracts when rolling right
```

This allows the robot to keep the seat level on a lateral slope by differentially extending one leg and retracting the other.

**Pitch compensation on the foot's forward offset:**

```
x_final = x0 + H · sin(pitch_correction) + lean_factor · (−sin(imu_roll))
```

The `H · sin(pitch_correction)` term shifts the foot forward when the body pitches forward, maintaining stable ground contact.

---

## Locomotion Modes and Mode Blending

The robot has two drive modes controlled by a continuous blend parameter `transition_alpha_` ∈ [0, 1]:

| α | Mode | Drive method |
|---|------|-------------|
| 1.0 | **Wheel** | Diff-drive via `/wheel_controller/cmd_vel` |
| 0.0 | **Track** | Speed commands via `/track_right` and `/track_left` | 

---


## Package Structure

```
my_bot_urdf_description/
│
├── urdf/
│   ├── my_bot_urdf.urdf.xacro        # Top-level robot description
│   ├── my_bot_urdf.gazebo             # Gazebo plugin configuration
│   ├── my_bot_urdf.trans              # Transmission definitions for ros2_control
│   ├── materials.xacro                # Visual material definitions
│   └── sensor.xacro                   # IMU and OAK-D Lite camera macros
│
├── meshes/                             # STL mesh files for all robot links
│
├── config/
│   └── controllers.yaml               # ros2_control joint controller configuration
│
├── launch/
│   ├── gazebo.launch.py               # Main simulation launch file
│   └── display.launch.py              # RViz visualization launch file
│
└── src/
    ├── controller_node.cpp            # Motion controller (IK, PID)
    ├── stair_detector_node.cpp        # Point cloud stair detection pipeline
    ├── teleop_node.cpp                # Keyboard teleoperation
    └── voxel_filter_node.cpp          # Point cloud voxel downsampling
```


