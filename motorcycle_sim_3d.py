#!/usr/bin/env python3
"""
Motorcycle hard-braking scenario — 3D standalone simulation.

No Isaac Sim or GPU required. Uses pyrender + EGL for headless 3D rendering.
Produces an MP4 video from a follow camera behind the motorcycle.

Scenario:
  - Both vehicles start at cruise speed (realistic — no acceleration from rest)
  - At brake_trigger_s: lead vehicle brakes hard
  - After reaction_time_s: motorcycle detects and brakes
  - Motorcycle stops safely with gap remaining

Install deps (one-time):
  pip install --user --break-system-packages numpy matplotlib pyrender trimesh imageio imageio-ffmpeg
"""

import argparse
import math
import sys
import os
import numpy as np

# ── pyrender needs EGL for headless rendering ─────────────────────────────────
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")


# ══════════════════════════════════════════════════════════════════════════════
# CLI Arguments
# ══════════════════════════════════════════════════════════════════════════════

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="3D motorcycle hard-braking scenario (no GPU required)."
    )
    # Scenario (more realistic defaults for a city/suburban road)
    p.add_argument("--cruise-speed-mps",    type=float, default=12.0,
                   help="Both vehicles cruise speed at start (m/s ≈ 43 km/h)")
    p.add_argument("--lead-start-x",        type=float, default=25.0,
                   help="Lead vehicle initial x position (m)")
    p.add_argument("--ego-start-x",         type=float, default=0.0,
                   help="Motorcycle start x position (m)")
    p.add_argument("--brake-trigger-s",     type=float, default=3.0,
                   help="Sim time when lead vehicle brakes (s)")
    p.add_argument("--reaction-time-s",     type=float, default=1.0,
                   help="Rider reaction time after car brakes (s)")
    p.add_argument("--lead-decel-mps2",     type=float, default=7.0,
                   help="Lead vehicle braking deceleration (m/s²)")
    p.add_argument("--ego-decel-mps2",      type=float, default=6.5,
                   help="Motorcycle emergency braking deceleration (m/s²)")
    p.add_argument("--sim-seconds",         type=float, default=12.0,
                   help="Maximum simulation duration (s)")
    # Lean dynamics (same model as motorcycle_standalone_sim.py)
    p.add_argument("--lean-kp",             type=float, default=3500.0)
    p.add_argument("--lean-kd",             type=float, default=500.0)
    p.add_argument("--moto-mass-kg",        type=float, default=200.0)
    p.add_argument("--com-height-m",        type=float, default=0.6)
    p.add_argument("--roll-inertia",        type=float, default=20.0)
    # Physics
    p.add_argument("--dt",                  type=float, default=1/60,
                   help="Physics timestep (s)")
    # Rendering
    p.add_argument("--width",               type=int,   default=1280)
    p.add_argument("--height",              type=int,   default=720)
    p.add_argument("--fps",                 type=int,   default=30)
    p.add_argument("--output",              type=str,   default="motorcycle_sim_3d.mp4")
    p.add_argument("--no-video",            action="store_true",
                   help="Skip rendering — print stats only")
    return p


# ══════════════════════════════════════════════════════════════════════════════
# Physics Simulation
# ══════════════════════════════════════════════════════════════════════════════

def simulate(args) -> dict:
    """
    Discrete-time Euler integration.

    Scenario:
      Phase 1 (0 → brake_trigger_s):         Both cruise at constant speed
      Phase 2 (brake_trigger_s → +reaction):  Car brakes, motorcycle still cruising
      Phase 3 (brake_trigger_s+reaction → …): Both braking, motorcycle decelerates to stop

    Lean dynamics: 2nd-order inverted pendulum with PD stabiliser.
    """
    dt  = args.dt
    g   = 9.81
    # Pre-compute lean dynamics constants
    net_stiffness = (args.moto_mass_kg * g * args.com_height_m) - args.lean_kp
    kd  = args.lean_kd
    I   = args.roll_inertia

    # Mutable state — both vehicles start at cruise speed
    ego_x      = float(args.ego_start_x)
    lead_x     = float(args.lead_start_x)
    ego_speed  = float(args.cruise_speed_mps)   # starts at cruise (not from rest)
    lead_speed = float(args.cruise_speed_mps)   # same initial speed
    roll       = 0.0
    roll_rate  = 0.0
    sim_time   = 0.0

    ego_braking_started = False  # track when motorcycle starts braking

    max_frames = int(args.sim_seconds / dt) + 20
    t_arr          = np.empty(max_frames)
    ego_x_arr      = np.empty(max_frames)
    lead_x_arr     = np.empty(max_frames)
    ego_speed_arr  = np.empty(max_frames)
    lead_speed_arr = np.empty(max_frames)
    gap_arr        = np.empty(max_frames)
    roll_arr       = np.empty(max_frames)
    braking_arr    = np.empty(max_frames, dtype=bool)  # True when lead vehicle is braking

    frame = 0
    while sim_time <= args.sim_seconds:
        raw_gap = lead_x - ego_x
        t_arr[frame]          = sim_time
        ego_x_arr[frame]      = ego_x
        lead_x_arr[frame]     = lead_x
        ego_speed_arr[frame]  = ego_speed
        lead_speed_arr[frame] = lead_speed
        gap_arr[frame]        = raw_gap
        roll_arr[frame]       = math.degrees(roll)
        braking_arr[frame]    = sim_time >= args.brake_trigger_s

        # ── Lead vehicle: brakes at trigger time ─────────────────────────────
        if sim_time >= args.brake_trigger_s:
            lead_speed = max(0.0, lead_speed - args.lead_decel_mps2 * dt)
        lead_x += lead_speed * dt

        # ── Ego motorcycle: brakes after reaction time ────────────────────────
        moto_brake_start = args.brake_trigger_s + args.reaction_time_s
        if sim_time >= moto_brake_start:
            if not ego_braking_started:
                ego_braking_started = True
            ego_speed = max(0.0, ego_speed - args.ego_decel_mps2 * dt)
        ego_x += ego_speed * dt

        # ── Lean dynamics ────────────────────────────────────────────────────
        roll_ddot  = (net_stiffness * roll - kd * roll_rate) / I
        roll_rate += roll_ddot * dt
        roll      += roll_rate * dt

        sim_time += dt
        frame    += 1

        # Terminate early when both stopped
        if ego_speed < 0.01 and lead_speed < 0.01 and sim_time > 2.0:
            break

    n = frame
    print(f"  {n} physics frames | {t_arr[n-1]:.2f}s simulated | "
          f"final gap = {gap_arr[n-1]:.2f}m | max lean = {max(abs(roll_arr[:n].min()), abs(roll_arr[:n].max())):.2f}°")

    return {
        "t":          t_arr[:n],
        "ego_x":      ego_x_arr[:n],
        "lead_x":     lead_x_arr[:n],
        "ego_speed":  ego_speed_arr[:n],
        "lead_speed": lead_speed_arr[:n],
        "gap":        gap_arr[:n],
        "roll_deg":   roll_arr[:n],
        "braking":    braking_arr[:n],  # True when lead vehicle is braking
    }


# ══════════════════════════════════════════════════════════════════════════════
# Vehicle Mesh Builders
# ══════════════════════════════════════════════════════════════════════════════
# Coordinate system (Y-up, matches pyrender/OpenGL):
#   +X  = forward (direction of travel)
#   +Y  = up
#   +Z  = left
# All vehicle meshes are built at origin, facing +X.

def _wheel_rot():
    """Rotation matrix: cylinder axis Z → axis Y (so wheel disc is in XZ plane = vertical)."""
    import trimesh.transformations as T
    return T.rotation_matrix(math.pi / 2, [1, 0, 0])


def build_motorcycle_mesh():
    """
    Sport bike composed of trimesh primitives.
    Returns a single concatenated trimesh (all vertex colors preserved).

    Physical layout (Y-up, origin at ground under rear axle centre):
      rear wheel  at x=0,    y=wr
      front wheel at x=1.3,  y=wr
      rider       above chassis, leaning slightly forward
    """
    import trimesh
    import trimesh.transformations as T

    parts = []
    wr  = 0.32   # wheel radius (m)
    wh  = 0.14   # wheel thickness

    rot_wheel = _wheel_rot()

    def colored(mesh, rgba):
        mesh = mesh.copy()
        mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, vertex_colors=np.tile(rgba, (len(mesh.vertices), 1)))
        return mesh

    # ── Wheels ───────────────────────────────────────────────────────────────
    rear_w = colored(trimesh.creation.cylinder(radius=wr, height=wh, sections=28), [22, 22, 22, 255])
    rear_w.apply_transform(rot_wheel)
    rear_w.apply_translation([0.0, wr, 0.0])

    rear_hub = colored(trimesh.creation.cylinder(radius=0.10, height=wh + 0.01, sections=16), [120, 120, 130, 255])
    rear_hub.apply_transform(rot_wheel)
    rear_hub.apply_translation([0.0, wr, 0.0])

    front_w = colored(trimesh.creation.cylinder(radius=wr, height=wh, sections=28), [22, 22, 22, 255])
    front_w.apply_transform(rot_wheel)
    front_w.apply_translation([1.3, wr, 0.0])

    front_hub = colored(trimesh.creation.cylinder(radius=0.10, height=wh + 0.01, sections=16), [120, 120, 130, 255])
    front_hub.apply_transform(rot_wheel)
    front_hub.apply_translation([1.3, wr, 0.0])

    # ── Swingarm + main chassis (dark steel) ─────────────────────────────────
    chassis = colored(trimesh.creation.box(extents=[1.2, 0.22, 0.28]), [38, 38, 48, 255])
    chassis.apply_translation([0.55, wr + 0.11, 0.0])

    # ── Fairing / body cowl (sport bike blue) ────────────────────────────────
    fairing = colored(trimesh.creation.box(extents=[0.85, 0.42, 0.32]), [25, 75, 200, 255])
    fairing.apply_translation([0.55, wr + 0.42, 0.0])

    # ── Fuel tank ────────────────────────────────────────────────────────────
    tank = colored(trimesh.creation.box(extents=[0.55, 0.20, 0.26]), [18, 55, 165, 255])
    tank.apply_translation([0.42, wr + 0.70, 0.0])

    # ── Seat / tail unit ─────────────────────────────────────────────────────
    tail = colored(trimesh.creation.box(extents=[0.55, 0.14, 0.24]), [15, 15, 20, 255])
    tail.apply_translation([-0.1, wr + 0.58, 0.0])

    # ── Front fork (silver tubes) ─────────────────────────────────────────────
    fork = colored(trimesh.creation.box(extents=[0.07, 0.58, 0.10]), [160, 160, 175, 255])
    fork.apply_translation([1.15, wr + 0.29, 0.0])

    # ── Handlebars ────────────────────────────────────────────────────────────
    hbar = colored(trimesh.creation.cylinder(radius=0.022, height=0.52, sections=8), [90, 90, 100, 255])
    # axis is already Z — perfect for handlebars running laterally
    hbar.apply_translation([0.85, wr + 0.76, 0.0])

    # ── Rider torso (orange race jacket, leaning forward over tank) ───────────
    torso = colored(trimesh.creation.box(extents=[0.38, 0.48, 0.28]), [210, 85, 15, 255])
    torso.apply_translation([0.35, wr + 0.98, 0.0])

    # ── Rider head / helmet (red + visor) ─────────────────────────────────────
    helmet = colored(trimesh.creation.icosphere(subdivisions=2, radius=0.18), [200, 30, 30, 255])
    helmet.apply_translation([0.30, wr + 1.42, 0.0])

    visor = colored(trimesh.creation.box(extents=[0.10, 0.09, 0.30]), [50, 80, 120, 200])
    visor.apply_translation([0.38, wr + 1.42, 0.0])

    # ── Headlight ─────────────────────────────────────────────────────────────
    headlight = colored(trimesh.creation.icosphere(subdivisions=1, radius=0.085), [255, 255, 200, 255])
    headlight.apply_translation([1.33, wr + 0.40, 0.0])

    # ── Exhaust (right side, gold/chrome) ─────────────────────────────────────
    exhaust = colored(trimesh.creation.cylinder(radius=0.038, height=0.72, sections=8), [185, 165, 95, 255])
    exhaust.apply_transform(T.rotation_matrix(math.pi / 2, [0, 1, 0]))   # axis along X
    exhaust.apply_translation([0.36, wr + 0.04, -0.16])

    parts = [rear_w, rear_hub, front_w, front_hub, chassis, fairing, tank, tail,
             fork, hbar, torso, helmet, visor, headlight, exhaust]
    return trimesh.util.concatenate(parts)


def build_car_mesh(braking: bool = False):
    """
    Generic SUV composed of trimesh primitives.
    Origin at ground-level centre. Faces +X.
    braking=True makes rear brake lights bright red.
    """
    import trimesh
    import trimesh.transformations as T

    def colored(mesh, rgba):
        mesh = mesh.copy()
        mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, vertex_colors=np.tile(rgba, (len(mesh.vertices), 1)))
        return mesh

    rot_wheel = _wheel_rot()
    wr = 0.37
    wh = 0.22

    # ── Lower body ────────────────────────────────────────────────────────────
    lower = colored(trimesh.creation.box(extents=[4.10, 0.88, 1.80]), [95, 105, 115, 255])
    lower.apply_translation([0.0, 0.44, 0.0])

    # ── Cabin / greenhouse ────────────────────────────────────────────────────
    cabin = colored(trimesh.creation.box(extents=[2.35, 0.72, 1.68]), [115, 125, 138, 255])
    cabin.apply_translation([-0.12, 0.88 + 0.36, 0.0])

    # ── Windshield ────────────────────────────────────────────────────────────
    wshield = colored(trimesh.creation.box(extents=[0.08, 0.62, 1.42]), [55, 80, 110, 220])
    wshield.apply_translation([0.94, 0.88 + 0.31, 0.0])

    # ── Rear window ───────────────────────────────────────────────────────────
    rwnd = colored(trimesh.creation.box(extents=[0.08, 0.58, 1.38]), [55, 80, 110, 220])
    rwnd.apply_translation([-1.18, 0.88 + 0.29, 0.0])

    # ── Front bumper ──────────────────────────────────────────────────────────
    fbump = colored(trimesh.creation.box(extents=[0.18, 0.38, 1.75]), [75, 85, 95, 255])
    fbump.apply_translation([2.14, 0.19, 0.0])

    # ── Rear bumper ───────────────────────────────────────────────────────────
    rbump = colored(trimesh.creation.box(extents=[0.18, 0.38, 1.75]), [75, 85, 95, 255])
    rbump.apply_translation([-2.14, 0.19, 0.0])

    # ── Brake lights ──────────────────────────────────────────────────────────
    bl_color = [255, 25, 25, 255] if braking else [110, 10, 10, 255]
    bl_L = colored(trimesh.creation.box(extents=[0.07, 0.14, 0.42]), bl_color)
    bl_L.apply_translation([-2.05, 0.65, 0.68])
    bl_R = colored(trimesh.creation.box(extents=[0.07, 0.14, 0.42]), bl_color)
    bl_R.apply_translation([-2.05, 0.65, -0.68])

    # ── Headlights ────────────────────────────────────────────────────────────
    hl_L = colored(trimesh.creation.box(extents=[0.07, 0.13, 0.38]), [255, 255, 200, 255])
    hl_L.apply_translation([2.05, 0.70, 0.68])
    hl_R = colored(trimesh.creation.box(extents=[0.07, 0.13, 0.38]), [255, 255, 200, 255])
    hl_R.apply_translation([2.05, 0.70, -0.68])

    # ── Roof rack (detail) ────────────────────────────────────────────────────
    rack = colored(trimesh.creation.box(extents=[1.80, 0.06, 1.60]), [80, 85, 90, 255])
    rack.apply_translation([-0.12, 0.88 + 0.72 + 0.03, 0.0])

    # ── Wheels + hubs (all 4) ─────────────────────────────────────────────────
    wheel_parts = []
    for xpos, zpos in [(1.35, 0.92), (1.35, -0.92), (-1.35, 0.92), (-1.35, -0.92)]:
        tire = colored(trimesh.creation.cylinder(radius=wr, height=wh, sections=24), [22, 22, 22, 255])
        tire.apply_transform(rot_wheel)
        tire.apply_translation([xpos, wr, zpos])

        hub = colored(trimesh.creation.cylinder(radius=0.17, height=wh + 0.01, sections=10), [145, 145, 155, 255])
        hub.apply_transform(rot_wheel)
        hub.apply_translation([xpos, wr, zpos])
        wheel_parts.extend([tire, hub])

    all_parts = [lower, cabin, wshield, rwnd, fbump, rbump,
                 bl_L, bl_R, hl_L, hl_R, rack] + wheel_parts
    return trimesh.util.concatenate(all_parts)


# ══════════════════════════════════════════════════════════════════════════════
# Camera Utilities
# ══════════════════════════════════════════════════════════════════════════════

def lookat_matrix(eye: np.ndarray, center: np.ndarray, up: np.ndarray) -> np.ndarray:
    """
    Build a 4×4 camera-to-world pose matrix (pyrender convention: camera looks along -Z).
    """
    fwd = center - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, up)
    right /= np.linalg.norm(right)
    true_up = np.cross(right, fwd)

    pose = np.eye(4)
    pose[:3, 0] = right
    pose[:3, 1] = true_up
    pose[:3, 2] = -fwd       # camera looks along -Z in OpenGL convention
    pose[:3, 3] = eye
    return pose


# ══════════════════════════════════════════════════════════════════════════════
# Road / Environment
# ══════════════════════════════════════════════════════════════════════════════

def build_road_mesh(length: float = 350.0, width: float = 7.5) -> "trimesh.Trimesh":
    """Asphalt road strip. Positioned along X axis, Y=0 (ground level)."""
    import trimesh

    def colored(mesh, rgba):
        mesh = mesh.copy()
        mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, vertex_colors=np.tile(rgba, (len(mesh.vertices), 1)))
        return mesh

    parts = []

    # Asphalt surface
    road = colored(trimesh.creation.box(extents=[length, 0.04, width]), [45, 45, 50, 255])
    road.apply_translation([length / 2 - 20, -0.02, 0.0])   # start before ego
    parts.append(road)

    # Outer kerb lines (white)
    for z_sign in (+1, -1):
        kerb = colored(trimesh.creation.box(extents=[length, 0.05, 0.15]), [230, 230, 230, 255])
        kerb.apply_translation([length / 2 - 20, 0.025, z_sign * (width / 2 - 0.075)])
        parts.append(kerb)

    # Dashed centre line (white) — one dash every 8m, 4m long
    for i in range(int(length / 8)):
        dash = colored(trimesh.creation.box(extents=[4.0, 0.05, 0.12]), [200, 200, 200, 255])
        dash.apply_translation([-20 + i * 8 + 2.0, 0.025, 0.0])
        parts.append(dash)

    import trimesh as tm
    return tm.util.concatenate(parts)


# ══════════════════════════════════════════════════════════════════════════════
# Video Renderer
# ══════════════════════════════════════════════════════════════════════════════

def render_video(data: dict, args) -> None:
    import trimesh
    import pyrender
    import imageio
    import imageio_ffmpeg  # noqa: F401  ensures bundled ffmpeg is on PATH

    t          = data["t"]
    ego_x      = data["ego_x"]
    lead_x     = data["lead_x"]
    braking    = data["braking"]
    n          = len(t)

    # Subsample to target fps
    step       = max(1, int(round((1.0 / args.dt) / args.fps)))
    frame_idx  = list(range(0, n, step))
    n_frames   = len(frame_idx)

    # ── Build static meshes ───────────────────────────────────────────────────
    print("  Building vehicle meshes …")
    moto_tmesh          = build_motorcycle_mesh()
    car_normal_tmesh    = build_car_mesh(braking=False)
    car_braking_tmesh   = build_car_mesh(braking=True)
    road_tmesh          = build_road_mesh(length=400.0)

    moto_pmesh          = pyrender.Mesh.from_trimesh(moto_tmesh,    smooth=True)
    car_normal_pmesh    = pyrender.Mesh.from_trimesh(car_normal_tmesh,  smooth=True)
    car_braking_pmesh   = pyrender.Mesh.from_trimesh(car_braking_tmesh, smooth=True)
    road_pmesh          = pyrender.Mesh.from_trimesh(road_tmesh,    smooth=False)

    # ── Build scene ───────────────────────────────────────────────────────────
    # Sky blue background
    scene = pyrender.Scene(
        bg_color=np.array([0.53, 0.81, 0.92, 1.0]),
        ambient_light=np.array([0.35, 0.35, 0.38])
    )

    # Road (static)
    scene.add(road_pmesh, pose=np.eye(4))

    # Motorcycle node (pose updated each frame)
    moto_node = scene.add(moto_pmesh, pose=np.eye(4))

    # Car node (swapped when braking starts)
    car_node = scene.add(car_normal_pmesh, pose=np.eye(4))
    car_is_braking = False

    # Lighting — sun from upper-front-right
    sun = pyrender.DirectionalLight(color=np.ones(3), intensity=4.5)
    sun_pose = lookat_matrix(
        eye=np.array([10.0, 20.0, -15.0]),
        center=np.array([0.0, 0.0, 0.0]),
        up=np.array([0.0, 1.0, 0.0])
    )
    scene.add(sun, pose=sun_pose)

    # Fill light from left (softer, cooler)
    fill = pyrender.DirectionalLight(color=np.array([0.7, 0.8, 1.0]), intensity=1.8)
    fill_pose = lookat_matrix(
        eye=np.array([-5.0, 10.0, 20.0]),
        center=np.array([0.0, 0.0, 0.0]),
        up=np.array([0.0, 1.0, 0.0])
    )
    scene.add(fill, pose=fill_pose)

    # Camera
    cam = pyrender.PerspectiveCamera(yfov=math.radians(58), aspectRatio=args.width / args.height)
    cam_node = scene.add(cam, pose=np.eye(4))

    # ── Renderer ──────────────────────────────────────────────────────────────
    renderer = pyrender.OffscreenRenderer(args.width, args.height)

    # Pre-compute motorcycle height above ground (rear axle at y=wheel_radius)
    moto_ground_y = 0.0   # mesh origin is at ground, no offset needed
    car_ground_y  = 0.0   # same

    # ── Render loop ───────────────────────────────────────────────────────────
    print(f"  Rendering {n_frames} frames → {args.output} …")
    writer = imageio.get_writer(
        args.output,
        fps=args.fps,
        codec="libx264",
        quality=9,
        ffmpeg_params=["-pix_fmt", "yuv420p"],
        ffmpeg_log_level="quiet",
    )

    def make_pose(tx, ty, tz, heading_rad=0.0):
        """4×4 pose: translation + Y-axis rotation (heading)."""
        c, s = math.cos(heading_rad), math.sin(heading_rad)
        return np.array([
            [ c, 0, s, tx],
            [ 0, 1, 0, ty],
            [-s, 0, c, tz],
            [ 0, 0, 0,  1],
        ], dtype=np.float64)

    for fn, i in enumerate(frame_idx):
        ex = float(ego_x[i])
        lx = float(lead_x[i])

        # ── Update motorcycle pose ─────────────────────────────────────────
        scene.set_pose(moto_node, pose=make_pose(ex, moto_ground_y, 0.0))

        # ── Swap car mesh when braking begins ─────────────────────────────
        if braking[i] and not car_is_braking:
            scene.remove_node(car_node)
            car_node = scene.add(car_braking_pmesh, pose=np.eye(4))
            car_is_braking = True
        scene.set_pose(car_node, pose=make_pose(lx - 2.05, car_ground_y, 0.0))
        # Note: subtract 2.05 so the rear bumper of the car is at lead_x
        # (the car mesh front is at +2.05m from origin, so origin → rear of car at lead_x)

        # ── Follow camera: behind and above motorcycle, looking forward ────
        # Camera slightly offset to right (+Z) for a cinematic 3/4 rear view
        cam_eye = np.array([ex - 7.0,  2.8,  3.2])
        cam_tgt = np.array([ex + 18.0, 0.6,  0.0])
        cam_up  = np.array([0.0, 1.0, 0.0])
        scene.set_pose(cam_node, pose=lookat_matrix(cam_eye, cam_tgt, cam_up))

        # ── Render frame ──────────────────────────────────────────────────
        color, _ = renderer.render(scene)
        writer.append_data(color)

        if fn % 30 == 0:
            pct = fn / n_frames * 100
            print(f"    {pct:3.0f}%  t={t[i]:.1f}s  gap={float(data['gap'][i]):.1f}m", end="\r", flush=True)

    renderer.delete()
    writer.close()

    from pathlib import Path
    print(f"\n  Saved: {Path(args.output).resolve()}")


# ══════════════════════════════════════════════════════════════════════════════
# Entry Point
# ══════════════════════════════════════════════════════════════════════════════

def main():
    args = build_parser().parse_args()

    print("Simulating physics …")
    data = simulate(args)

    if args.no_video:
        return

    print("Building scene and rendering …")
    render_video(data, args)


if __name__ == "__main__":
    main()
