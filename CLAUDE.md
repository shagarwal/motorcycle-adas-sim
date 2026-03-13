# Motorcycle ADAS Sim — Claude Instructions

This repo contains a motorcycle ADAS braking scenario rendered with Blender.
If asked to continue working on this project, here is everything you need to know.

## What this does

`motorcycle_sim_blender.py` runs a Python physics simulation (no external deps)
then calls Blender to render a 3D video. `blender_scene.py` is the Blender
Python scene script called internally — do not run it directly.

## Setup on a new machine (WSL2 on Windows)

### 1. Install Blender
```bash
mkdir -p /mnt/c/blender
curl -L "https://www.blender.org/download/release/Blender4.3/blender-4.3.2-windows-arm64.zip" \
     -o /tmp/blender.zip
unzip /tmp/blender.zip -d /mnt/c/blender/
# Verify:
/mnt/c/blender/blender-4.3.2-windows-arm64/blender.exe --version
```
For x86_64 Windows, replace `arm64` with `x64` in the URL and path.

### 2. Clone and run
```bash
git clone https://github.com/shagarwal/motorcycle-adas-sim.git
cd motorcycle-adas-sim
python3 motorcycle_sim_blender.py --samples 16   # preview
```

### 3. Update BLENDER_EXE if needed
The `BLENDER_EXE` constant near the top of `motorcycle_sim_blender.py`
must point to the actual blender executable.

## How it works

- `motorcycle_sim_blender.py` runs physics (Euler integration, inverted-pendulum
  lean dynamics, PD controller) and exports `/tmp/moto_physics.json`.
- It then calls `blender.exe --background --python blender_scene.py -- json output.mp4 samples`.
- `blender_scene.py` reads the JSON and creates the 3D scene + animation + render.
- Blender is a Windows process running via WSL2 interop. It reads the physics JSON
  via a UNC path (`\\wsl.localhost\Ubuntu\tmp\...`).
- No Python packages are needed — only standard library.

## Key physics parameters

| Constant | Value | What it controls |
|---|---|---|
| `V_CRUISE` | 12 m/s | Cruise speed |
| `CAR_BRAKE_TIME` | 3.0 s | When lead car brakes |
| `CAR_BRAKE_ACCEL` | -7.0 m/s² | Car braking deceleration |
| `MOTO_REACT_S` | 1.0 s | Motorcycle ADAS reaction delay |
| `MOTO_BRAKE_ACCEL` | -6.5 m/s² | Motorcycle braking deceleration |
| `WEAVE_AMP` | 0.8 m | Lane weave amplitude (drives visible lean) |
| `KP` / `KD` | 3500 / 500 | Lean PD controller gains |

## Render quality guide

```bash
python3 motorcycle_sim_blender.py --samples 16    # ~10-30 min preview
python3 motorcycle_sim_blender.py --samples 48    # ~30-90 min default
python3 motorcycle_sim_blender.py --samples 128   # ~2-6 hr high quality
python3 motorcycle_sim_blender.py --no-render     # physics check only
```

Output goes to `C:\Users\<username>\motorcycle_blender.mp4` by default.
Override with `--output C:\path\to\out.mp4`.

## Blender scene notes

- Coordinate system: Z-up, vehicles face +X, Y = lateral
- Motorcycle: blue sport bike with orange-suit rider; origin = rear axle ground level
- Car: silver SUV; `car.location.x = lead_x - 2.08` (centres car so front bumper = lead_x)
- Lean: rotation_euler X-axis in degrees, positive = lean right
- Camera: 8 m behind / 2.5 m right / 2.8 m up in motorcycle local frame, Track-To constraint
- Road: straight, 11 m wide, runs +X from x=-30 to x=500
- Rendering: Cycles CPU, OIDN denoising, Filmic colour, FFMPEG H264 MP4 output

## Common issues

**Blender can't find physics JSON:**
Check the UNC path printed in `[Blender] Launching render...` output.
The distro name in `\\wsl.localhost\<distro>\tmp\...` must match your WSL distro.
The `to_wsl_unc()` function reads this from `/etc/os-release NAME=`.

**Wrong Blender path:**
Update `BLENDER_EXE` in `motorcycle_sim_blender.py`.

**No output file produced:**
Run `--no-render` first and check physics prints `← safe ✓`.
Then try `--samples 8` for the fastest possible test render.
