#!/usr/bin/env python3
"""
motorcycle_sim_blender.py — Motorcycle ADAS braking scenario: physics + Blender render.

Scenario
--------
  0–3 s  : Both vehicles cruise at 12 m/s.  Motorcycle drives straight, upright.
  3 s    : Lead car brakes hard at −9 m/s² (brake lights switch on).
  4 s    : Motorcycle ADAS detects the event and brakes at −4.6 m/s².
  ~5.6 s : Both stopped.  Final gap ≈ 0.35 m (one foot).  No collision.

Output
------
  C:\\Users\\shaur\\motorcycle_blender.mp4  (or --output PATH)
  Rendered by Blender Cycles CPU + OIDN denoising (48 spp default).

Requirements
------------
  blender_scene.py   in the same directory as this file
  Blender 4.3.2      at /mnt/c/blender/blender-4.3.2-windows-arm64/blender.exe

Usage
-----
  python3 motorcycle_sim_blender.py                 # 48 spp, ~30-90 min
  python3 motorcycle_sim_blender.py --samples 16    # quick preview, ~10-30 min
  python3 motorcycle_sim_blender.py --samples 128   # high quality, ~2-6 h
  python3 motorcycle_sim_blender.py --no-render     # physics only, writes JSON
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
CAR_BRAKE_ACCEL = -9.0    # m/s²  (hard ABS stop)

# Motorcycle ADAS response
MOTO_REACT_S     = 1.0    # s  — reaction delay after car brakes
MOTO_BRAKE_ACCEL = -4.6   # m/s²  (tuned so gap ≈ 0.35 m at stop)

SIM_DURATION    = 12.0    # s  — total simulation length

# ── Video parameters ───────────────────────────────────────────────────────────
FPS    = 30
WIDTH  = 1920
HEIGHT = 1080

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
    ego_z          : lateral position — always 0 (straight lane, no weave)
    lean_deg       : roll angle — always 0 (motorcycle stays upright)
    braking        : True once the car starts braking

    Motorcycle origin = rear axle at ego_x.
    Car lead_x tracks the front bumper of the car.
    """
    ego_x,  ego_vx  = 0.0, V_CRUISE
    lead_x, lead_vx = MOTO_WHEELBASE + INITIAL_VIS_GAP + CAR_LENGTH, V_CRUISE

    braking       = False
    moto_t_brake  = None

    out = {k: [] for k in [
        "t", "ego_x", "ego_z", "ego_heading",
        "lean_deg", "lead_x", "lead_z", "lead_heading", "braking",
    ]}

    t = 0.0
    while t <= SIM_DURATION + DT / 2:

        # Lead vehicle longitudinal
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

        # Record — lateral and lean always zero (straight, upright)
        out["t"].append(round(t, 6))
        out["ego_x"].append(ego_x)
        out["ego_z"].append(0.0)
        out["ego_heading"].append(0.0)
        out["lean_deg"].append(0.0)
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
                    # NAME="Ubuntu" or NAME="Debian GNU/Linux" → first word
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
    ap.add_argument("--output",    default="C:\\Users\\shaur\\motorcycle_blender.mp4",
                    help="Output MP4 path (Windows path for Blender)")
    ap.add_argument("--width",     type=int, default=WIDTH)
    ap.add_argument("--height",    type=int, default=HEIGHT)
    ap.add_argument("--no-render", action="store_true",
                    help="Run physics only, write JSON, skip Blender")
    args = ap.parse_args()

    # ── Physics ────────────────────────────────────────────────────────────────
    print("[Sim] Running physics simulation ...")
    data = simulate()

    n          = len(data["t"])
    # final gap: car rear bumper − moto front axle
    final_gap  = (data["lead_x"][-1] - CAR_LENGTH) - (data["ego_x"][-1] + MOTO_WHEELBASE)

    print(f"      Steps    : {n}  ({data['t'][-1]:.1f} s simulated)")
    print(f"      Final gap: {final_gap:.2f} m  {'← collision! ⚠' if final_gap < 0 else '← safe ✓'}")

    data.update(fps=FPS, width=args.width, height=args.height)

    json_path = "/tmp/moto_physics.json"
    with open(json_path, "w") as fh:
        json.dump(data, fh)
    print(f"[Sim] Physics JSON → {json_path}")

    if args.no_render:
        print("[Sim] --no-render: done.")
        return

    # ── Blender render ─────────────────────────────────────────────────────────
    # blender_scene.py lives in the same directory as this script
    script_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "blender_scene.py"
    )

    # Blender is a Windows process: it needs Windows-style paths
    win_script = to_win_path(script_path)       # /mnt/c/isaacsim/... → C:\isaacsim\...
    win_json   = to_wsl_unc(json_path)          # /tmp/... → \\wsl.localhost\Ubuntu\tmp\...

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

    result = subprocess.run(cmd)   # streams Blender stdout to terminal

    if result.returncode == 0:
        print(f"\n[Done] Video saved → {args.output}")
    else:
        print(f"\n[Error] Blender exited with code {result.returncode}")
        sys.exit(1)


if __name__ == "__main__":
    main()
