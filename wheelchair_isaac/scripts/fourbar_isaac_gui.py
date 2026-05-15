#!/usr/bin/env python3
"""
fourbar_isaac_gui.py — Isaac Sim 5.1 Four-Bar Linkage Simulation & GUI

Architecture Overview:
    1. FourBarGeometry   — Pure-math IK solver for a planar four-bar linkage.
    2. URDF Import       — Loads the robot model and configures joint drives.
    3. Physics Config    — TGS solver, collision filtering, conveyor surfaces.
    4. Closure Joints    — D6 constraints that close the kinematic loop the
                           URDF tree cannot represent (URDF is tree-only).
    5. Three-Phase Init  — Settle → add closure → re-settle startup sequence.
    6. Tkinter GUI       — Slider controls for crank/follower/seat/conveyor.
    7. Simulation Loop   — Reads GUI state, sets joint targets, steps physics.

Four-Bar Topology (one side):
    Ground link (d): connects pivot A (follower) to pivot B (crank) on base_link
    Crank   (a): driven link at pivot B  (base_to_link2_left/right)
    Coupler (b): connects crank tip C to follower tip D  (the track assembly)
    Follower(c): passive link at pivot A (base_to_link1_left/right)

    The URDF tree has: base→crank→track and base→follower, but NOT
    follower→track (that would create a loop). The closure joint bridges
    follower-tip D to track-attach-point D' at runtime in PhysX.
"""

from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": False, "width": 1280, "height": 720})

import numpy as np
import math, os, shutil, threading
import tkinter as tk
from tkinter import ttk

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf
import omni.usd
import omni.kit.commands
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage

class FourBarGeometry:
    """Planar four-bar linkage inverse/forward kinematics solver.

    Models a four-bar mechanism in the YZ plane with four links:
        d (ground)  : fixed distance from follower pivot A to crank pivot B
        a (crank)   : driven input link, pivots at B
        b (coupler) : connects crank tip C to follower tip D (the track)
        c (follower): passive link, pivots at A

    Args:
        A_yz:        [y, z] position of follower pivot A in base frame.
        B_yz:        [y, z] position of crank pivot B in base frame.
        C_local_yz:  [y, z] offset from B to crank tip C in crank-joint frame.
        D_local_yz:  [y, z] offset from A to follower tip D in follower-joint frame.
        Dp_local_yz: [y, z] offset from C to coupler attach D' in track-joint frame.
    """

    def __init__(self, A_yz, B_yz, C_local_yz, D_local_yz, Dp_local_yz):
        self.A = np.array(A_yz, dtype=float)   # Follower pivot (base_to_link1)
        self.B = np.array(B_yz, dtype=float)   # Crank pivot    (base_to_link2)

        self.d = np.linalg.norm(self.B - self.A)      # Ground link: distance A→B
        self.a = np.linalg.norm(C_local_yz)            # Crank link:  |C_local| = distance B→C
        self.b = np.linalg.norm(Dp_local_yz)           # Coupler:     |Dp_local| = distance C→D'
        self.c = np.linalg.norm(D_local_yz)            # Follower:    |D_local| = distance A→D

        self.crank_offset    = math.atan2(C_local_yz[1],  C_local_yz[0])
        self.follower_offset = math.atan2(D_local_yz[1],  D_local_yz[0])
        self.coupler_offset  = math.atan2(Dp_local_yz[1], Dp_local_yz[0])

    def _pos_from_angle(self, origin, length, abs_angle):
        return origin + length * np.array([math.cos(abs_angle), math.sin(abs_angle)])

    def _circle_intersect(self, P1, r1, P2, r2, branch):
        dx, dy = P2[0] - P1[0], P2[1] - P1[1]
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > r1 + r2 or dist < abs(r1 - r2) or dist < 1e-12:
            return None
        a_cc = (r1*r1 - r2*r2 + dist*dist) / (2*dist)
        h = math.sqrt(max(0, r1*r1 - a_cc*a_cc))     
        mx = P1[0] + a_cc * dx / dist                
        my = P1[1] + a_cc * dy / dist                
        return np.array([mx + branch * h * dy / dist,
                         my - branch * h * dx / dist])

    def crank_to_follower(self, q_crank, branch=None):
        """Forward IK: given crank joint angle, compute follower and track angles.

        Steps:
            1. Convert q_crank to absolute angle: θ_crank = crank_offset - q_crank
            2. Locate crank tip C = B + a*[cos θ, sin θ]
            3. Find follower tip D via circle-circle intersection:
               circle(A, c) ∩ circle(C, b)  — D lies on both circles.
            4. Recover q_foll = follower_offset - atan2(D-A)
            5. Recover q_track = coupler_offset - atan2(D-C) - q_crank

        Args:
            q_crank: Crank joint angle in radians (URDF convention).
            branch:  ±1 to select linkage configuration, or None to try both.

        Returns:
            (q_foll, q_track) tuple in radians, or None if no valid solution.
        """
        theta_crank = self.crank_offset - q_crank
        C = self._pos_from_angle(self.B, self.a, theta_crank)
        best = None
        for br in ([branch] if branch is not None else [+1, -1]):
            #   - follower circle: center=A, radius=c (follower length)
            #   - coupler circle:  center=C, radius=b (coupler length)
            D = self._circle_intersect(self.A, self.c, C, self.b, br)
            if D is None:
                continue
            theta_foll = math.atan2(D[1] - self.A[1], D[0] - self.A[0])
            q_foll = self.follower_offset - theta_foll
            q_foll = math.atan2(math.sin(q_foll), math.cos(q_foll))  
            theta_coup = math.atan2(D[1] - C[1], D[0] - C[0])
            q_track = self.coupler_offset - theta_coup - q_crank
            q_track = math.atan2(math.sin(q_track), math.cos(q_track))
            if best is None:
                best = (q_foll, q_track, br)
            else:
                return (q_foll, q_track)
        if best is not None:
            return (best[0], best[1])
        return None

    def follower_to_crank(self, q_foll, branch=None):
        """Inverse IK: given follower joint angle, compute crank and track angles.

        Mirror of crank_to_follower — starts from the follower end instead.

        Steps:
            1. Convert q_foll to absolute angle: θ_foll = follower_offset - q_foll
            2. Locate follower tip D = A + c*[cos θ, sin θ]
            3. Find crank tip C via circle-circle intersection:
               circle(B, a) ∩ circle(D, b)
            4. Recover q_crank = crank_offset - atan2(C-B)
            5. Recover q_track = coupler_offset - atan2(D-C) - q_crank

        Args:
            q_foll: Follower joint angle in radians.
            branch: ±1 to select configuration, or None to try both.

        Returns:
            (q_crank, q_track) tuple in radians, or None if no valid solution.
        """
        theta_foll = self.follower_offset - q_foll
        D = self._pos_from_angle(self.A, self.c, theta_foll)
        best = None
        for br in ([branch] if branch is not None else [+1, -1]):
            C = self._circle_intersect(self.B, self.a, D, self.b, br)
            if C is None:
                continue
            theta_crank = math.atan2(C[1] - self.B[1], C[0] - self.B[0])
            q_crank = self.crank_offset - theta_crank
            q_crank = math.atan2(math.sin(q_crank), math.cos(q_crank))  # Wrap to [-π, π]
            theta_coup = math.atan2(D[1] - C[1], D[0] - C[0])
            q_track = self.coupler_offset - theta_coup - q_crank
            q_track = math.atan2(math.sin(q_track), math.cos(q_track))  # Wrap to [-π, π]
            if best is None:
                best = (q_crank, q_track, br)
            else:
                return (q_crank, q_track)
        if best is not None:
            return (best[0], best[1])
        return None

    def calibrate_branch(self, q_crank_init, q_foll_init):
        theta_crank = self.crank_offset - q_crank_init
        C = self._pos_from_angle(self.B, self.a, theta_crank)
        for br in [+1, -1]:
            D = self._circle_intersect(self.A, self.c, C, self.b, br)
            if D is None:
                continue
            theta_foll = math.atan2(D[1] - self.A[1], D[0] - self.A[0])
            q_foll = self.follower_offset - theta_foll
            q_foll = math.atan2(math.sin(q_foll), math.cos(q_foll))
            if abs(q_foll - q_foll_init) < 0.05:  # ~3° tolerance
                self._branch = br
                print(f"  Calibrated branch = {br:+d}")
                return br
        self._branch = +1
        return +1

    def crank_to_follower_cal(self, q_crank):
        """Forward IK using the calibrated branch. See crank_to_follower()."""
        return self.crank_to_follower(q_crank, self._branch)

    def follower_to_crank_cal(self, q_foll):
        """Inverse IK using the calibrated branch. See follower_to_crank()."""
        return self.follower_to_crank(q_foll, self._branch)

# Left side: YZ coordinates from URDF joint origins (X is lateral, ignored for planar IK)
# A = base_to_link1_left  origin YZ = [0.0798, -0.054998]
# B = base_to_link2_left  origin YZ = [0.185,  -0.179243]
# C_local = Revolute_38   origin YZ = [-0.096704, 0.011871]  (crank tip in crank frame)
# D_local = closure pos0  YZ = [-0.112968, -0.099623]        (follower tip in follower frame)
# Dp_local = closure pos1 YZ = [-0.122098, 0.012436]         (coupler attach in track frame)
fb_left = FourBarGeometry(
    A_yz=[0.0798, -0.054998],
    B_yz=[0.185,  -0.179243],
    C_local_yz=[-0.096704, 0.011871],
    D_local_yz=[-0.112968, -0.099623],
    Dp_local_yz=[-0.122098, 0.012436],
)
fb_right = FourBarGeometry(
    A_yz=[0.0798, -0.054998],
    B_yz=[0.185,  -0.179243],
    C_local_yz=[-0.096704, 0.011871],
    D_local_yz=[-0.112858, -0.099467],
    Dp_local_yz=[-0.122232, 0.012300],
)

fb_left.calibrate_branch(0.0, 0.0)
fb_right.calibrate_branch(0.0, 0.0)

print(f"Left:  ground={fb_left.d:.4f} crank={fb_left.a:.4f} "
      f"coupler={fb_left.b:.4f} follower={fb_left.c:.4f}")
print(f"Right: ground={fb_right.d:.4f} crank={fb_right.a:.4f} "
      f"coupler={fb_right.b:.4f} follower={fb_right.c:.4f}")

URDF_PATH = os.path.expanduser(
    "~/ros2_ws/src/wheelchair_isaac/urdf/robot_flat.urdf"
)
USD_DIR = os.path.expanduser(
    "~/ros2_ws/src/wheelchair_isaac/urdf/isaac_output"
)
USD_PATH = os.path.join(USD_DIR, "robot.usd")

if os.path.exists(USD_DIR):
    shutil.rmtree(USD_DIR)
os.makedirs(USD_DIR)

_, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.set_import_inertia_tensor(True)          
cfg.set_fix_base(True)                       
cfg.set_default_drive_type(1)                
cfg.set_default_drive_strength(1000.0)      
cfg.set_default_position_drive_damping(100.0)
cfg.set_make_default_prim(True)            
cfg.set_create_physics_scene(False)          

status, _ = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=URDF_PATH, dest_path=USD_PATH, import_config=cfg,
)
print(f"\n[URDF] Import status={status}")

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane(z_position=-1.0) 
add_reference_to_stage(usd_path=USD_PATH, prim_path="/World/Robot")
stage = omni.usd.get_context().get_stage()

def find_rigid_body(root_path, name):
    for p in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        if p.GetName() == name and p.HasAPI(UsdPhysics.RigidBodyAPI):
            return p.GetPath().pathString
    for p in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        if p.GetName() == name:
            print(f"  [WARN] {name} found but no RigidBodyAPI at {p.GetPath()}")
            return p.GetPath().pathString
    return None

print("\n[DRIVES & LIMITS] Configuring joint parameters...")
joint_prims = {}

joint_params = {
    "base_to_link2_left":  {"stiffness": 1e4, "damping": 1e3, "low": -90.0, "high": 18.0},
    "base_to_link2_right": {"stiffness": 1e4, "damping": 1e3, "low": -90.0, "high": 18.0},
    
    "base_to_link1_left":  {"stiffness": 0.0, "damping": 10.0, "low": -90.0, "high": 90.0},
    "base_to_link1_right": {"stiffness": 0.0, "damping": 10.0, "low": -90.0, "high": 90.0},
    
    "Revolute_38": {"stiffness": 0.0, "damping": 10.0, "low": None, "high": None},
    "Revolute_39": {"stiffness": 0.0, "damping": 10.0, "low": None, "high": None},
    
    "base_to_backSeat": {"stiffness": 500.0, "damping": 100.0, "low": -6.88, "high": 69.93},
    "wheel_left":  {"stiffness": 0.0, "damping": 10.0, "low": None, "high": None},
    "wheel_right": {"stiffness": 0.0, "damping": 10.0, "low": None, "high": None},
}

for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Robot")):
    if "RevoluteJoint" not in prim.GetTypeName():
        continue
    name = prim.GetName()
    
    lookup_name = name.replace(" ", "_")
    
    params = joint_params.get(lookup_name)
    if not params:
        params = {"stiffness": 0.0, "damping": 10.0, "low": None, "high": None}
        
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if not drive:
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        
    drive.GetStiffnessAttr().Set(params["stiffness"])
    drive.GetDampingAttr().Set(params["damping"])
    
    rev_joint = UsdPhysics.RevoluteJoint(prim)
    if params["low"] is not None and params["high"] is not None:
        rev_joint.GetLowerLimitAttr().Set(params["low"])
        rev_joint.GetUpperLimitAttr().Set(params["high"])
    
    joint_prims[lookup_name] = prim
    lo = rev_joint.GetLowerLimitAttr().Get()
    hi = rev_joint.GetUpperLimitAttr().Get()
    print(f"  {name}: stiffness={params['stiffness']}, damping={params['damping']}, "
          f"limits=[{lo}, {hi}]")

print(f"\n[JOINTS] Found {len(joint_prims)}: {list(joint_prims.keys())}")


def set_joint_target_rad(joint_name, angle_rad):
    """Set the position-drive target for a revolute joint.

    Converts radians to degrees (USD convention) and writes to the
    angular drive's target position attribute.

    Args:
        joint_name: URDF joint name (spaces are auto-replaced with underscores).
        angle_rad:  Target angle in radians.
    """
    prim = joint_prims.get(joint_name.replace(" ", "_"))
    if not prim:
        return
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if drive:
        attr = drive.GetTargetPositionAttr()
        if not attr:
            attr = drive.CreateTargetPositionAttr()
        attr.Set(math.degrees(angle_rad)) 

print("\n[PHYSICS] Configuring solver...")
for prim in Usd.PrimRange(stage.GetPseudoRoot()):
    if prim.IsA(UsdPhysics.Scene):
        px = PhysxSchema.PhysxSceneAPI.Apply(prim)
        px.GetMaxPositionIterationCountAttr().Set(255)
        px.GetMaxVelocityIterationCountAttr().Set(64)
        px.CreateEnableStabilizationAttr().Set(True)
        solver_attr = prim.GetAttribute("physxScene:solverType")
        if solver_attr and solver_attr.IsValid():
            solver_attr.Set("TGS")
        else:
            try:
                px.CreateSolverTypeAttr().Set("TGS")
            except Exception as e:
                print(f"  [WARN] TGS solver attr failed: {e}")
        print(f"  Scene {prim.GetPath()}: TGS, pos=255, vel=64")
        break

print("\n[COLLISION] Applying collision filters...")
physics_scene_prim = None
for p in Usd.PrimRange(stage.GetPseudoRoot()):
    if p.IsA(UsdPhysics.Scene):
        physics_scene_prim = p
        break

if physics_scene_prim:
    filt = UsdPhysics.FilteredPairsAPI.Apply(physics_scene_prim)
    
    collision_pairs = [
        # Left Side
        ("link_1_1",       "track_left_1"),
        ("link_2_1",       "link_1_1"),
        ("link_2_1",       "track_left_1"),
        ("base_link",      "track_left_1"),
        ("base_link",      "link_1_1"),
        ("base_link",      "link_2_1"),
        ("back_seat_1",    "track_left_1"),
        ("back_seat_1",    "link_1_1"),
        ("back_seat_1",    "link_2_1"),
        
        # Right Side
        ("link_1_right_1", "track_right_1"),
        ("link_2_right_1", "link_1_right_1"),
        ("link_2_right_1", "track_right_1"), 
        ("base_link",      "track_right_1"),
        ("base_link",      "link_1_right_1"),
        ("base_link",      "link_2_right_1"),
        ("back_seat_1",    "track_right_1"),
        ("back_seat_1",    "link_1_right_1"),
        ("back_seat_1",    "link_2_right_1"),
        
        ("base_link",      "back_seat_1"),
    ]
    
    for body_a, body_b in collision_pairs:
        pa = find_rigid_body("/World/Robot", body_a)
        pb = find_rigid_body("/World/Robot", body_b)
        if pa and pb:
            filt.GetFilteredPairsRel().AddTarget(Sdf.Path(pa))
            filt.GetFilteredPairsRel().AddTarget(Sdf.Path(pb))
            print(f"  [OK] Excluded: {body_a} <-> {body_b}")

def setup_conveyor(track_name, velocity_ms=0.0):
    """Apply PhysxSurfaceVelocityAPI to a track rigid body.

    This makes the track surface act like a conveyor belt, imparting
    tangential velocity to any contacting body (the ground).

    Args:
        track_name:   Name of the track rigid body in the URDF.
        velocity_ms:  Initial surface velocity in m/s (Y-axis = longitudinal).

    Returns:
        PhysxSurfaceVelocityAPI handle
    """
    track_path = find_rigid_body("/World/Robot", track_name)
    if not track_path:
        print(f"  [FAIL] Could not find {track_name} for conveyor setup.")
        return None
    
    prim = stage.GetPrimAtPath(track_path)
    if not prim.HasAPI(PhysxSchema.PhysxSurfaceVelocityAPI):
        conveyor_api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
    else:
        conveyor_api = PhysxSchema.PhysxSurfaceVelocityAPI(prim)
    
    conveyor_api.GetSurfaceVelocityEnabledAttr().Set(True)
    conveyor_api.GetSurfaceVelocityAttr().Set(Gf.Vec3f(0.0, velocity_ms, 0.0))
    
    return conveyor_api

conveyor_l = setup_conveyor("track_left_1")
conveyor_r = setup_conveyor("track_right_1")

print("\n[PHASE 1] Settling without closure joints (300 frames)...")

for name, prim in joint_prims.items():
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if drive:
        drive.GetStiffnessAttr().Set(100000.0) 
        drive.GetDampingAttr().Set(10000.0)    

init_l = fb_left.crank_to_follower_cal(0.0)
init_r = fb_right.crank_to_follower_cal(0.0)

print(f"[PHASE 1] Settling at IK-valid pose...")
world.reset()  

set_joint_target_rad("base_to_link2_left", 0.0)       
set_joint_target_rad("base_to_link1_left", init_l[0])  
set_joint_target_rad("Revolute_38",        init_l[1])  

set_joint_target_rad("base_to_link2_right", 0.0)
set_joint_target_rad("base_to_link1_right", init_r[0])
set_joint_target_rad("Revolute_39",         init_r[1])

for i in range(300):
    world.step(render=True)
    if i % 50 == 0:
        print(f"  settling... frame {i}/300")

print("[PHASE 1] Done — bodies held perfectly at rest pose.")

print("\n[PHASE 2] Adding closure joints to settled simulation...")

CLOSURE_PARAMS = {
    "left": {
        "link1": "link_1_1",
        "track": "track_left_1",
        "pos0":  Gf.Vec3f(0.0,     -0.112968, -0.099623), 
        "pos1":  Gf.Vec3f(0.0,     -0.122098,  0.012436), 
    },
    "right": {
        "link1": "link_1_right_1",
        "track": "track_right_1",
        "pos0":  Gf.Vec3f(0.0,     -0.112858, -0.099467),
        "pos1":  Gf.Vec3f(0.0,     -0.122232,  0.012300),
    },
}

closure_joints = {}  

for side, params in CLOSURE_PARAMS.items():
    link1_path = find_rigid_body("/World/Robot", params["link1"])
    track_path = find_rigid_body("/World/Robot", params["track"])

    print(f"\n  [{side.upper()}] body0={link1_path}  body1={track_path}")
    if not link1_path or not track_path:
        print(f"  [FAIL] Body not found for {side}!")
        continue

    
    jpath = f"/World/ClosureJoint_{side}"
    if stage.GetPrimAtPath(jpath).IsValid():
        omni.kit.commands.execute("DeletePrimsCommand", paths=[jpath])

    joint_prim = UsdPhysics.Joint.Define(stage, jpath)
    jprim = stage.GetPrimAtPath(jpath)

    physx_joint = PhysxSchema.PhysxJointAPI.Apply(jprim)
    physx_joint.CreateJointFrictionAttr().Set(0.0)
    physx_joint.CreateMaxJointVelocityAttr().Set(1e6)

    jprim.CreateAttribute(
        "physxJoint:enableProjection", Sdf.ValueTypeNames.Bool
    ).Set(True)

    jprim.CreateAttribute(
        "physics:excludeFromArticulation", Sdf.ValueTypeNames.Bool
    ).Set(True)

    joint_prim.CreateBody0Rel().SetTargets([Sdf.Path(link1_path)])
    joint_prim.CreateBody1Rel().SetTargets([Sdf.Path(track_path)])

    # Set local anchor positions (where the pin-hole is in each body's frame)
    joint_prim.CreateLocalPos0Attr().Set(params["pos0"])
    joint_prim.CreateLocalPos1Attr().Set(params["pos1"])
    joint_prim.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))  
    joint_prim.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

    for ax in ["transX", "transY", "transZ"]:
        lim = UsdPhysics.LimitAPI.Apply(jprim, ax)
        lim.CreateLowAttr().Set(0.0)
        lim.CreateHighAttr().Set(0.0)

    print(f"  [OK] {jpath}: transXYZ LOCKED, ALL ROTATIONS FREE (Ball Joint)")
    
    closure_joints[side] = {
        "body0_path": link1_path,
        "body1_path": track_path,
        "pos0": params["pos0"],
        "pos1": params["pos1"]
    }

print("\n[PHASE 3] Settling closure joints (200 frames, soft drives)...")

for name, prim in joint_prims.items():
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if drive:
        if "link2" in name:  
            drive.GetStiffnessAttr().Set(200.0)
        else:  
            drive.GetStiffnessAttr().Set(10.0)

for i in range(200):
    world.step(render=True)
    if i % 50 == 0:
        print(f"  settling... frame {i}/200")

for name, prim in joint_prims.items():
    params = joint_params.get(name, {"stiffness": 0.0, "damping": 10.0})
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if drive:
        drive.GetStiffnessAttr().Set(params["stiffness"])
        drive.GetDampingAttr().Set(params["damping"])

print("[PHASE 3] Done. Robot ready.")

r = fb_left.crank_to_follower_cal(0.0)
if r:
    print(f"\n[IK TEST] crank=0 → follower={math.degrees(r[0]):+.1f}°  "
          f"track={math.degrees(r[1]):+.1f}°")

class State:
    """Shared mutable state between GUI thread and simulation loop.

    Attributes:
        lock_tracks:     bool — if True, left/right sliders move together.
        q_crank_left/right:  float — crank joint target (radians).
        q_foll_left/right:   float — follower joint target (radians).
        q_track_left/right:  float — track/coupler joint target (radians).
        seat:            float — seat joint target (radians).
        conveyor_speed:  float — track surface velocity (m/s).
        dirty:           bool — set True when any value changes; sim loop
                                clears it after applying the new targets.
    """

    def __init__(self):
        self.lock_tracks   = True  
        
        # Initialise follower/track angles from IK at crank=0
        r_left = fb_left.crank_to_follower_cal(0.0)
        r_right = fb_right.crank_to_follower_cal(0.0)
        
        self.q_crank_left  = 0.0
        self.q_foll_left   = r_left[0] if r_left else 0.0
        self.q_track_left  = r_left[1] if r_left else 0.0
        
        self.q_crank_right = 0.0
        self.q_foll_right  = r_right[0] if r_right else 0.0
        self.q_track_right = r_right[1] if r_right else 0.0
        
        self.seat  = 0.0
        self.conveyor_speed = 0.0  # m/s
        self.dirty = True          # Force initial target write

state = State()

# TKINTER GUI — Runs in a separate daemon thread so it doesn't
# block the Isaac Sim rendering loop. Communicates with the sim
# loop exclusively through the shared `state` object.
gui_ready = threading.Event()  # Signals main thread that GUI is initialised

def gui_thread():
    root = tk.Tk()
    root.title("Isaac Sim — Four-Bar Joint Controller")
    root.geometry("560x600")
    root.configure(bg="#1e1e2e")

    _updating = [False]

    ttk.Style().theme_use("clam")
    for sn, bg, fg, ft in [
        ("TLabel",   "#1e1e2e", "#cdd6f4", ("Monospace", 10)),
        ("H.TLabel", "#1e1e2e", "#89b4fa", ("Monospace", 11, "bold")),
    ]:
        ttk.Style().configure(sn, background=bg, foreground=fg, font=ft)

    ttk.Label(root, text="═══ FOUR-BAR JOINT CONTROLS ═══",
              style="H.TLabel").pack(pady=(10, 5))

    lock_var = tk.BooleanVar(value=True)
    tk.Checkbutton(root, text="Lock Left/Right together",
                   variable=lock_var,
                   command=lambda: setattr(state, 'lock_tracks', lock_var.get()),
                   bg="#1e1e2e", fg="#cdd6f4", selectcolor="#313244").pack()

    def safe_set(scale, value):
        cb = scale.cget("command")
        scale.config(command="")
        scale.set(value)
        scale.config(command=cb)

    def mkslider(parent, label, lo, hi, cb):
        ttk.Label(parent, text=label).pack(pady=(4, 0))
        s = tk.Scale(parent, from_=lo, to=hi, resolution=0.001,
                     orient=tk.HORIZONTAL, command=cb, length=500,
                     bg="#313244", fg="#cdd6f4", troughcolor="#45475a",
                     highlightthickness=0)
        s.pack()
        return s

    c_lo, c_hi = -1.57, 0.315   # Crank: -90° to +18°
    f_lo, f_hi = -1.57, 0.315   # Follower: same range (IK-linked)

    def on_crank_left(val):
        """Left crank slider moved → forward IK → update follower + state."""
        if _updating[0]: return
        q = float(val)
        r = fb_left.crank_to_follower_cal(q)  # Forward IK: crank → follower
        if not r: return
        state.q_crank_left, state.q_foll_left, state.q_track_left = q, r[0], r[1]
        _updating[0] = True  
        safe_set(sl_foll_l, r[0]) 
        if state.lock_tracks:  
            rr = fb_right.crank_to_follower_cal(q)
            if rr:
                state.q_crank_right, state.q_foll_right, state.q_track_right = q, rr[0], rr[1]
                safe_set(sl_crank_r, q)
                safe_set(sl_foll_r, rr[0])
        _updating[0] = False
        state.dirty = True

    def on_crank_right(val):
        if _updating[0]: return
        q = float(val)
        r = fb_right.crank_to_follower_cal(q)
        if not r: return
        state.q_crank_right, state.q_foll_right, state.q_track_right = q, r[0], r[1]
        _updating[0] = True
        safe_set(sl_foll_r, r[0])
        if state.lock_tracks:
            rl = fb_left.crank_to_follower_cal(q)
            if rl:
                state.q_crank_left, state.q_foll_left, state.q_track_left = q, rl[0], rl[1]
                safe_set(sl_crank_l, q)
                safe_set(sl_foll_l, rl[0])
        _updating[0] = False
        state.dirty = True

    def on_foll_left(val):
        if _updating[0]: return
        q = float(val)
        r = fb_left.follower_to_crank_cal(q)
        if not r: return
        state.q_foll_left, state.q_crank_left, state.q_track_left = q, r[0], r[1]
        _updating[0] = True
        safe_set(sl_crank_l, r[0])
        if state.lock_tracks:
            rr = fb_right.follower_to_crank_cal(q)
            if rr:
                state.q_foll_right, state.q_crank_right, state.q_track_right = q, rr[0], rr[1]
                safe_set(sl_foll_r, q)
                safe_set(sl_crank_r, rr[0])
        _updating[0] = False
        state.dirty = True

    def on_foll_right(val):
        if _updating[0]: return
        q = float(val)
        r = fb_right.follower_to_crank_cal(q)
        if not r: return
        state.q_foll_right, state.q_crank_right, state.q_track_right = q, r[0], r[1]
        _updating[0] = True
        safe_set(sl_crank_r, r[0])
        if state.lock_tracks:
            rl = fb_left.follower_to_crank_cal(q)
            if rl:
                state.q_foll_left, state.q_crank_left, state.q_track_left = q, rl[0], rl[1]
                safe_set(sl_foll_l, q)
                safe_set(sl_crank_l, rl[0])
        _updating[0] = False
        state.dirty = True

    def on_seat(val):
        state.seat = float(val)
        state.dirty = True

    def on_conveyor(val):
        state.conveyor_speed = float(val)
        state.dirty = True

    _updating[0] = True
    sl_crank_l = mkslider(root, "Left CRANK  (base_to_link2_left)",     c_lo, c_hi, on_crank_left)
    sl_crank_r = mkslider(root, "Right CRANK (base_to_link2_right)",    c_lo, c_hi, on_crank_right)
    sl_foll_l  = mkslider(root, "Left FOLLOWER  (base_to_link1_left)",  f_lo, f_hi, on_foll_left)
    sl_foll_r  = mkslider(root, "Right FOLLOWER (base_to_link1_right)", f_lo, f_hi, on_foll_right)
    sl_seat    = mkslider(root, "Seat (base_to_backSeat)", -0.5, 3.14, on_seat)
    sl_conveyor = mkslider(root, "Track Conveyor Speed (m/s)", -2.0, 2.0, on_conveyor)
    
    r_left = fb_left.crank_to_follower_cal(0.0)
    r_right = fb_right.crank_to_follower_cal(0.0)
    safe_set(sl_crank_l, 0.0)
    safe_set(sl_crank_r, 0.0)
    safe_set(sl_foll_l, r_left[0] if r_left else 0.0)
    safe_set(sl_foll_r, r_right[0] if r_right else 0.0)
    safe_set(sl_seat, 0.0)
    safe_set(sl_conveyor, 0.0)
    _updating[0] = False

    def reset_all():
        _updating[0] = True
        r_left = fb_left.crank_to_follower_cal(0.0)
        r_right = fb_right.crank_to_follower_cal(0.0)
        safe_set(sl_crank_l, 0.0)
        safe_set(sl_crank_r, 0.0)
        safe_set(sl_foll_l, r_left[0] if r_left else 0.0)
        safe_set(sl_foll_r, r_right[0] if r_right else 0.0)
        safe_set(sl_seat, 0.0)
        safe_set(sl_conveyor, 0.0)
        _updating[0] = False
        
        state.q_crank_left = 0.0
        state.q_foll_left = r_left[0] if r_left else 0.0
        state.q_track_left = r_left[1] if r_left else 0.0
        
        state.q_crank_right = 0.0
        state.q_foll_right = r_right[0] if r_right else 0.0
        state.q_track_right = r_right[1] if r_right else 0.0
        
        state.seat  = 0.0
        state.conveyor_speed = 0.0
        state.dirty = True

    tk.Button(root, text="↺ Reset All", command=reset_all,
              bg="#45475a", fg="#cdd6f4", padx=20, pady=5).pack(pady=10)

    gui_ready.set()
    root.mainloop()

t = threading.Thread(target=gui_thread, daemon=True)
t.start()
gui_ready.wait()

print("\n" + "=" * 50)
print("  FOUR-BAR JOINT CONTROLLER (Isaac Sim 5.1)")
print("  Move sliders — IK drives all joints together")
print("=" * 50 + "\n")

def check_constraints():
    for side, info in closure_joints.items():
        prim0 = stage.GetPrimAtPath(info["body0_path"])
        prim1 = stage.GetPrimAtPath(info["body1_path"])
        if prim0.IsValid() and prim1.IsValid():
            xf0 = UsdGeom.Xformable(prim0)
            xf1 = UsdGeom.Xformable(prim1)
            time = Usd.TimeCode.Default()
            
            w_t_0 = xf0.ComputeLocalToWorldTransform(time)
            w_t_1 = xf1.ComputeLocalToWorldTransform(time)
            
            p0_world = w_t_0.Transform(info["pos0"])  # Follower pin in world
            p1_world = w_t_1.Transform(info["pos1"])  # Track pin in world
            
            dist = np.linalg.norm(np.array(p0_world) - np.array(p1_world))
            status = "OK" if dist < 0.001 else "WARN"  # 1mm threshold
            print(f"  {side.upper()} Error: {dist*1000:.2f} mm [{status}]")

frame = 0
try:
    while sim_app.is_running():
        if state.dirty:
            state.dirty = False
            set_joint_target_rad("base_to_link1_left",  state.q_foll_left)
            set_joint_target_rad("base_to_link2_left",  state.q_crank_left)
            set_joint_target_rad("Revolute_38",         state.q_track_left)
            set_joint_target_rad("base_to_link1_right", state.q_foll_right)
            set_joint_target_rad("base_to_link2_right", state.q_crank_right)
            set_joint_target_rad("Revolute_39",         state.q_track_right)
            set_joint_target_rad("base_to_backSeat",    state.seat)

            # Update conveyor belt velocity on both tracks
            vel_vector = Gf.Vec3f(0.0, state.conveyor_speed, 0.0)
            if conveyor_l:
                conveyor_l.GetSurfaceVelocityAttr().Set(vel_vector)
            if conveyor_r:
                conveyor_r.GetSurfaceVelocityAttr().Set(vel_vector)

            if frame % 60 == 0:
                _drv = UsdPhysics.DriveAPI.Get(joint_prims["base_to_link2_left"], "angular")
                _tgt = _drv.GetTargetPositionAttr().Get() if _drv else None
                _stf = _drv.GetStiffnessAttr().Get() if _drv else None
                print(
                    f"  crank_L={math.degrees(state.q_crank_left):+6.1f}°  "
                    f"foll_L={math.degrees(state.q_foll_left):+6.1f}°  "
                    f"track_L={math.degrees(state.q_track_left):+6.1f}°  "
                    f"USD_tgt={_tgt:.1f}° stiff={_stf}"
                )

        if frame > 0 and frame % 300 == 0:
            check_constraints()

        world.step(render=True)
        frame += 1

except KeyboardInterrupt:
    print("\n[SIM] Stopped.")

sim_app.close()