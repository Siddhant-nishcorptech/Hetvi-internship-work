# Wheelchair Isaac Sim — Four-Bar Linkage Simulation

Simulation of a stair-climbing wheelchair robot with four-bar track linkages, implemented in NVIDIA Isaac Sim 5.1 with PhysX TGS solver and ROS 2 Jazzy.

---

## Overview

This project simulates the track mechanism of a stair-climbing wheelchair using a four-bar linkage on each side. The mechanical design connects the base frame to rubber track assemblies through a crank, coupler, and follower link.

Because URDF cannot represent kinematic loops, the closed-loop constraint is re-established at runtime using PhysX ball joints after import.

**Key features:**

- Closed-loop kinematic constraints via PhysX D6/Ball joints (replicating MuJoCo `<connect>`)
- Analytical Inverse Kinematics solver for the four-bar mechanism
- IMU-driven PID balance controller with stair detection state machine
- Tkinter GUI for manual joint and conveyor speed control
- ROS 2 keyboard teleoperation node

---

## Repository Structure

```
wheelchair_isaac/
├── urdf/
│   ├── robot.xacro               # Xacro source 
│   ├── robot_flat.urdf           # Flattened URDF for Isaac Sim import
│   └── isaac_output/             # Auto-generated USD output (do not edit)
├── meshes/                       # STL mesh files for all links
├── scripts/
│   ├── fourbar_isaac_gui.py      # GUI control for joints
│   ├── balance_controller.py     # ROS 2 IMU-PID balance node
│   └── teleop_keyboard.py        # ROS 2 keyboard teleoperation node
├── package.xml
└── setup.cfg
```

| File | Purpose |
|---|---|
| `fourbar_isaac_gui.py` | Loads URDF, configures physics, adds closure joints, runs GUI and sim loop |
| `balance_controller.py` | Subscribes `/imu/data`, publishes `/fourbar/crank_cmd` via PID |
| `teleop_keyboard.py` | Reads keyboard input, publishes `/cmd_vel` Twist messages |
| `robot.xacro` | Source robot description with joint limits and mesh references |
| `robot_flat.urdf` | Flattened URDF consumed by `URDFParseAndImportFile` |

---

## Four-Bar Linkage Mechanism

### Topology

One four-bar mechanism exists on each side (left and right). Each has four rigid links in a closed loop:

- **Ground link (d)** — the fixed base frame (`base_link`). Connects follower pivot A to crank pivot B.
- **Crank (a)** — the driven input link, pivoting at B. Actuated by a position drive.
- **Coupler (b)** — the floating track assembly (`track_left_1` / `track_right_1`). Not directly actuated; follows from the constraint.
- **Follower (c)** — passive output link, pivoting at A. Angle fully determined by the crank through the closed-loop constraint.

The loop closure removes one DOF, leaving exactly one: the crank angle. Controlling the crank fully determines the posture of the entire mechanism.

> **Why is the loop broken in the URDF?**
> URDF is tree-structured and cannot represent kinematic loops. The joint `link1_to_track` is removed from the URDF tree and re-closed in Isaac Sim at runtime by adding a PhysX ball joint (`ClosureJoint_left` / `ClosureJoint_right`) at the exact pin-hole coordinates from CAD geometry. This replicates MuJoCo `<connect>` behaviour.

### Link Lengths and Pivot Coordinates 

| Parameter | Value |
|---|---|
| A — follower pivot (YZ) | `[0.0798, -0.054998]` |
| B — crank pivot (YZ) | `[0.185, -0.179243]` |
| C_local — crank tip offset | `[-0.096704, 0.011871]` |
| D_local — follower tip offset | `[-0.112968, -0.099623]` |
| Dp_local — coupler attach offset | `[-0.122098, 0.012436]` |
| Ground link `d = \|B − A\|` | `0.1763 m` |
| Crank `a = \|C_local\|` | `0.0969 m` |
| Coupler `b = \|Dp_local\|` | `0.1228 m` |
| Follower `c = \|D_local\|` | `0.1512 m` |

---

## Inverse Kinematics

The IK solver computes all joint angles analytically from a single input angle using circle-circle intersection. 

All geometry is computed in a 2D YZ plane. Angles are measured from the positive Y axis, increasing counter-clockwise. Each USD joint angle `q` maps to an absolute geometric angle `theta` via a fixed offset:

```
theta_abs = offset - q_joint
```

Offsets are computed in `__init__` from local frame vectors:

```
crank_offset    = atan2(C_local[1],  C_local[0])
follower_offset = atan2(D_local[1],  D_local[0])
coupler_offset  = atan2(Dp_local[1], Dp_local[0])
```

### Forward Problem: Crank → Follower

**Given:** `q_crank` — **Find:** `q_foll`, `q_track`

1. Convert to absolute angle: `theta_crank = crank_offset - q_crank`
2. Locate crank tip C: `C = B + a * [cos(theta_crank), sin(theta_crank)]`
3. Find follower tip D via circle-circle intersection (radius `c` from A, radius `b` from C)
4. Recover follower angle: `q_foll = follower_offset - atan2(D-A)`
5. Recover track angle: `q_track = coupler_offset - theta_coup - q_crank`

### Inverse Problem: Follower → Crank

**Given:** `q_foll` — **Find:** `q_crank`, `q_track`

Mirror of the forward problem. Locate follower tip D from `q_foll`, then find crank tip C via circle-circle intersection (radius `a` from B, radius `b` from D), then recover `q_crank` and `q_track`.

### Branch Calibration

Circle-circle intersection always yields two solutions (±h). At startup, `calibrate_branch(q_crank_init=0.0, q_foll_init=0.0)` tries both branches at the zero pose and stores the one that reproduces `q_foll_init` within 0.05 rad. This branch is used for all subsequent calls, preventing branch-switching during operation.

**Numerical safeguards:**
- All angles are wrapped with `atan2(sin(q), cos(q))` to prevent drift.
- `_circle_intersect` returns `None` when the linkage is overextended or degenerate. The GUI interprets `None` as a motion limit and does not update joint targets.
- `h = sqrt(max(0, r1² - a_cc²))` clamps to zero at boundary cases to prevent NaN.

---

## Isaac Sim Implementation

### URDF Import Settings

| Parameter | Value | Reason |
|---|---|---|
| `fix_base` | `True` | `base_link` is fixed to world |
| `import_inertia_tensor` | `True` | Uses URDF inertia tensors, not auto-computed |
| `default_drive_type` | `1` (Position) | Joints respond to angle targets in degrees |
| `default_drive_strength` | `1000.0` | Initial stiffness; overridden per-joint |
| `default_position_drive_damping` | `100.0` | Initial damping; overridden per-joint |
| `create_physics_scene` | `False` | Physics scene configured manually for TGS |

### Physics Solver

| Setting | Value | Reason |
|---|---|---|
| Solver type | TGS | Better constraint coupling than PGS for closed loops |
| Position iterations | 255 (max) | Required to resolve simultaneous ball-joint constraints |
| Velocity iterations | 64 | Sufficient for velocity-level constraint satisfaction |
| Stabilization | enabled | Prevents slow drift in constrained bodies |

### GUI and Shared State

The Tkinter GUI runs in a daemon thread. A `State` object with a `dirty` flag mediates communication with the Isaac Sim main thread: the GUI writes new joint angles and sets `dirty = True`; the sim loop reads and applies them, then clears `dirty`. An `_updating` flag prevents recursive slider callbacks when one slider programmatically updates another.

---

## Balance Controller

**File:** `balance_controller.py`

A ROS 2 node that reads IMU orientation, runs pitch and roll PID controllers at 50 Hz, detects stair-climbing conditions, and publishes differential crank angle commands.

---

## Keyboard Teleoperation

**File:** `teleop_keyboard.py`

Reads single characters from stdin in raw terminal mode and publishes `geometry_msgs/Twist` on `/cmd_vel`. Isaac Sim maps `/cmd_vel` to `PhysxSurfaceVelocityAPI` on the track bodies.

| Key | Action |
|---|---|
| `i` | Forward (linear.x = +2.0 m/s) |
| `,` | Backward (linear.x = −2.0 m/s) |
| `j` | Turn left (angular.z = +1.0 rad/s) |
| `l` | Turn right (angular.z = −1.0 rad/s) |
| `k` | Stop |
| `Ctrl-C` | Exit (publishes zero Twist before shutdown) |

---

## Installation & Running

### Prerequisites

- NVIDIA Isaac Sim 5.1
- ROS 2 Jazzy sourced in shell
- Workspace built with `colcon build`
- URDF at `~/ros2_ws/src/wheelchair_isaac/urdf/robot_flat.urdf`

### Clone and Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/Siddhant-nishcorptech/Hetvi-internship-work.git
cd ~/ros2_ws
colcon build --packages-select wheelchair_isaac
source install/setup.bash
```

### Run

**Terminal 1 — Isaac Sim:**
```bash
~/.local/share/ov/pkg/isaac-sim-5.1.0/python.sh \
    ~/ros2_ws/src/wheelchair_isaac/scripts/fourbar_isaac_gui.py
```

Startup takes ~500 simulation frames (~30 seconds). Expected terminal output:
```
[PHASE 1] Settling at IK-valid pose...
[PHASE 2] Adding closure joints to settled simulation...
[PHASE 3] Settling closure joints (200 frames, soft drives)...
[PHASE 3] Done. Robot ready.
```

**Terminal 2 — Balance Controller:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run wheelchair_isaac balance_controller
```

**Terminal 3 — Keyboard Teleop:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run wheelchair_isaac teleop_keyboard
```

### Monitor and Tune

```bash
# Monitor topics
ros2 topic echo /balancing/state
ros2 topic echo /fourbar/crank_cmd
ros2 topic hz   /imu/data

# Tune PID at runtime
ros2 param set /balance_controller pitch_pid.kp 1.5
ros2 param set /balance_controller stair_detection.pitch_threshold_deg 10.0
ros2 param set /balance_controller stair_mode.clearance_offset_rad -0.30
```
