#!/usr/bin/env python3
"""
motorcycle_sim_blender.py — Motorcycle ADAS braking scenario: physics + Blender render.

Scenario
--------
  0–3 s  : Both vehicles cruise at 12 m/s.
            Motorcycle follows a raised-cosine lateral weave (±0.8 m) —
            shows the active lean-stabilisation controller working.
  3 s    : Lead SUV brakes hard at −7 m/s² (brake lights switch on).
  4 s    : Motorcycle ADAS detects the event and brakes at −6.5 m/s².
  ~6 s   : Both stopped.  Final gap ≈ 7 m.  No collision.

Lean dynamics use a linearised inverted-pendulum + PD controller.
The raised-cosine weave has zero velocity at both endpoints so there
is no abrupt snap when the weave ends at t = 3 s.

Output
------
  Rendered by Blender Cycles CPU + OIDN denoising (48 spp default).
  Default output: C:\\Users\\<you>\\motorcycle_blender.mp4  (override with --output)

Requirements
------------
  blender_scene.py   in the same directory as this file
  Blender 4.x        — see README for download and setup
  No Python packages needed (standard library only)

Usage
-----
  python3 motorcycle_sim_blender.py                         # 48 spp, ~30-90 min
  python3 motorcycle_sim_blender.py --samples 16            # quick preview, ~10-30 min
  python3 motorcycle_sim_blender.py --samples 128           # high quality, ~2-6 h
  python3 motorcycle_sim_blender.py --no-render             # physics only, writes JSON
  python3 motorcycle_sim_blender.py --output C:\\path\\out.mp4
"""

import argparse
import json
import math
import os
import subprocess
import sys

# ── Scenario parameters ────────────────────────────────────────────────────────

DT              = 0.002   # physics timestep (s)
V_CRUISE        = 12.0    # cruise speed (m/s ≈ 43 km/h)
GRAVITY         = 9.81    # m/s²

# Vehicle geometry (metres)
MOTO_WHEELBASE  = 1.28    # rear axle to front axle
CAR_LENGTH      = 4.16    # front bumper to rear bumper

# Initial gap: car rear bumper to motorcycle front axle
INITIAL_VIS_GAP = 20.0    # m

# Lead vehicle
CAR_BRAKE_TIME  = 3.0     # s  — car starts braking
CAR_BRAKE_ACCEL = -7.0    # m/s²

# Motorcycle ADAS response
MOTO_REACT_S    = 1.0     # s  — reaction delay after car brakes
MOTO_BRAKE_ACCEL = -6.5   # m/s²

SIM_DURATION    = 12.0    # s  — total simulation length

# ── Lean dynamics (linearised inverted pendulum + PD controller) ───────────────
# θ̈ = (M·g·h·θ + τ) / I_xx     (gravity destabilises; τ stabilises)
# τ  = −Kp·(θ − θ_eq) − Kd·θ̇    (PD, drives lean toward equilibrium)
# θ_eq = atan(−lateral_accel / g) (equilibrium lean for a sustained curve)

MOTO_MASS  = 200.0   # kg  (bike + rider)
LEAN_H     = 0.60    # m   (CoM height above ground)
LEAN_I_XX  = 20.0    # kg·m²
KP         = 3500.0
KD         = 500.0

# ── Lane weave (raised cosine: zero velocity at t=0 and t=WEAVE_DUR) ──────────
# z(t) = (A/2)·(1 − cos(2π·t/T))   for 0 ≤ t < T
# Peak lateral displacement = A = 0.8 m (well within lane width of ±5.5 m)
# Peak lateral acceleration  ≈ 1.76 m/s²  →  lean_eq_max ≈ 10°

WEAVE_AMP   = 0.8    # m  (peak lateral from centre)
WEAVE_DUR   = 3.0    # s  (same as CAR_BRAKE_TIME: weave ends exactly when car brakes)

# ── Video parameters ───────────────────────────────────────────────────────────
FPS    = 30
WIDTH  = 1920
HEIGHT = 1080

# ── Blender executable path ────────────────────────────────────────────────────
# WSL2 (ARM64 Windows):  /mnt/c/blender/blender-4.x-windows-arm64/blender.exe
# WSL2 (x86_64 Windows): /mnt/c/blender/blender-4.x-windows-x64/blender.exe
# Native Linux:          /path/to/blender/blender
# UPDATE THIS to match where you installed Blender.
BLENDER_EXE = "/mnt/c/blender/blender-4.3.2-windows-arm64/blender.exe"


# ══════════════════════════════════════════════════════════════════════════════
# Physics simulation
# ══════════════════════════════════════════════════════════════════════════════

def simulate():
    """
    Euler integration of the two-vehicle scenario.

    Coordinate system
    -----------------
    ego_x, lead_x : longitudinal (forward) positions, metres
    ego_z, lead_z  : lateral positions (positive = left of centre), metres
    ego_heading    : yaw angle, radians (0 = straight ahead)
    lean_deg       : roll angle, degrees (positive = lean right)
    braking        : True once the car starts braking

    Motorcycle origin = rear axle at (ego_x, ego_z).
    Car lead_x tracks the front bumper of the car.
    """
    ego_x,  ego_vx  = 0.0, V_CRUISE
    lead_x, lead_vx = MOTO_WHEELBASE + INITIAL_VIS_GAP + CAR_LENGTH, V_CRUISE

    lean_rad  = 0.0
    lean_rate = 0.0

    braking       = False
    moto_t_brake  = None

    out = {k: [] for k in [
        "t", "ego_x", "ego_z", "ego_heading",
        "lean_deg", "lead_x", "lead_z", "lead_heading", "braking",
    ]}

    t = 0.0
    while t <= SIM_DURATION + DT / 2:

        # Lead vehicle
        if t >= CAR_BRAKE_TIME and lead_vx > 0.0:
            braking  = True
            lead_vx  = max(0.0, lead_vx + CAR_BRAKE_ACCEL * DT)
        lead_x += lead_vx * DT

        # Motorcycle longitudinal
        if braking and moto_t_brake is None:
            moto_t_brake = t + MOTO_REACT_S
        if moto_t_brake and t >= moto_t_brake and ego_vx > 0.0:
            ego_vx = max(0.0, ego_vx + MOTO_BRAKE_ACCEL * DT)
        ego_x += ego_vx * DT

        # Lateral weave (raised cosine)
        if t < WEAVE_DUR:
            w      = 2.0 * math.pi / WEAVE_DUR
            ego_z  = (WEAVE_AMP / 2.0) * (1.0 - math.cos(w * t))
            ego_vz = (WEAVE_AMP / 2.0) * w       * math.sin(w * t)
            lat_a  = (WEAVE_AMP / 2.0) * w * w   * math.cos(w * t)
        else:
            ego_z = ego_vz = lat_a = 0.0

        # Lean dynamics (inverted pendulum + PD controller)
        lean_eq    = math.atan2(-lat_a, GRAVITY)
        torque      = -KP * (lean_rad - lean_eq) - KD * lean_rate
        lean_accel  = (MOTO_MASS * GRAVITY * LEAN_H * lean_rad + torque) / LEAN_I_XX
        lean_rate  += lean_accel * DT
        lean_rad   += lean_rate  * DT

        # Heading from velocity
        ego_heading = math.atan2(ego_vz, ego_vx) if ego_vx > 0.01 else 0.0

        out["t"].append(round(t, 6))
        out["ego_x"].append(ego_x)
        out["ego_z"].append(ego_z)
        out["ego_heading"].append(ego_heading)
        out["lean_deg"].append(math.degrees(lean_rad))
        out["lead_x"].append(lead_x)
        out["lead_z"].append(0.0)
        out["lead_heading"].append(0.0)
        out["braking"].append(braking)

        t += DT

    return out


# ══════════════════════════════════════════════════════════════════════════════
# Path helpers (WSL ↔ Windows)
# ══════════════════════════════════════════════════════════════════════════════

def to_win_path(linux_path: str) -> str:
    """Convert /mnt/c/some/path → C:\\some\\path for Windows processes."""
    if linux_path.startswith("/mnt/c/"):
        return "C:\\" + linux_path[7:].replace("/", "\\")
    return linux_path


def to_wsl_unc(linux_path: str) -> str:
    """
    Convert a WSL filesystem path (/tmp/foo) to a Windows UNC path
    (\\\\wsl.localhost\\Ubuntu\\tmp\\foo) so a Windows process can read it.
    """
    distro = "Ubuntu"
    try:
        with open("/etc/os-release") as fh:
            for line in fh:
                if line.startswith("NAME="):
                    distro = line.split("=", 1)[1].strip().strip('"').split()[0]
                    break
    except Exception:
        pass
    return "\\\\" + "wsl.localhost\\" + distro + linux_path.replace("/", "\\")


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description="Run motorcycle ADAS physics sim and render with Blender."
    )
    ap.add_argument("--samples",   type=int, default=48,
                    help="Cycles samples per pixel (16=preview, 48=default, 128=HQ)")
    ap.add_argument("--output",    default=None,
                    help="Output MP4 path (Windows path). Defaults to C:\\Users\\<user>\\motorcycle_blender.mp4")
    ap.add_argument("--width",     type=int, default=WIDTH)
    ap.add_argument("--height",    type=int, default=HEIGHT)
    ap.add_argument("--no-render", action="store_true",
                    help="Run physics only, write JSON, skip Blender")
    args = ap.parse_args()

    if args.output is None:
        username = os.environ.get("USER", os.environ.get("USERNAME", "user"))
        args.output = f"C:\\Users\\{username}\\motorcycle_blender.mp4"

    print("[Sim] Running physics simulation ...")
    data = simulate()

    n          = len(data["t"])
    max_lean   = max(abs(l) for l in data["lean_deg"])
    final_gap  = (data["lead_x"][-1] - CAR_LENGTH) - (data["ego_x"][-1] + MOTO_WHEELBASE)

    print(f"      Steps    : {n}  ({data['t'][-1]:.1f} s simulated)")
    print(f"      Peak lean: {max_lean:.1f}°")
    print(f"      Final gap: {final_gap:.1f} m  {'← collision! ⚠' if final_gap < 0 else '← safe ✓'}")

    data.update(fps=FPS, width=args.width, height=args.height)

    json_path = "/tmp/moto_physics.json"
    with open(json_path, "w") as fh:
        json.dump(data, fh)
    print(f"[Sim] Physics JSON → {json_path}")

    if args.no_render:
        print("[Sim] --no-render: done.")
        return

    script_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "blender_scene.py"
    )
    win_script = to_win_path(script_path)
    win_json   = to_wsl_unc(json_path)

    n_frames   = int(round(data["t"][-1] * FPS))
    mins_lo    = max(1, n_frames * 5  // 60)
    mins_hi    = max(2, n_frames * 20 // 60)

    print(f"\n[Blender] Launching render ...")
    print(f"  Script  : {win_script}")
    print(f"  JSON    : {win_json}")
    print(f"  Output  : {args.output}")
    print(f"  Res     : {args.width}×{args.height} @ {FPS} fps")
    print(f"  Quality : {args.samples} spp + OIDN denoising")
    print(f"  Frames  : ~{n_frames}")
    print(f"  Est.    : {mins_lo}–{mins_hi} min on CPU")
    print(f"  (Run with --samples 16 for a quick preview)\n")

    cmd = [
        BLENDER_EXE,
        "--background",
        "--python", win_script,
        "--",
        win_json,
        args.output,
        str(args.samples),
    ]

    result = subprocess.run(cmd)

    if result.returncode == 0:
        print(f"\n[Done] Video saved → {args.output}")
    else:
        print(f"\n[Error] Blender exited with code {result.returncode}")
        sys.exit(1)


if __name__ == "__main__":
    main()
