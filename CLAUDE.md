# Motorcycle ADAS Sim — Claude Instructions

Standalone Blender-based motorcycle ADAS braking scenario. **No Isaac Sim required.**
Working directory: `/mnt/c/Users/shaur/Documents/Code_Repository/3d_sim_creator/`
GitHub: https://github.com/shagarwal/motorcycle-adas-sim

## Files

| File | Purpose |
|---|---|
| `motorcycle_sim_blender.py` | Orchestrator: runs physics, calls Blender |
| `blender_scene.py` | Blender Python scene builder — called internally, do not run directly |
| `motorcycle_sim_3d.py` | Alternative pyrender renderer (no Blender required) |

## Setup (WSL2 on Windows)

```bash
# ARM64 Windows:
winget install -e --id BlenderFoundation.Blender
# OR manual install at /mnt/c/blender/blender-4.3.2-windows-arm64/blender.exe

# x86_64 Windows: download from blender.org, update BLENDER_EXE in motorcycle_sim_blender.py
```

## Running

```bash
cd /mnt/c/Users/shaur/Documents/Code_Repository/3d_sim_creator/

python3 motorcycle_sim_blender.py --samples 8    # fast sanity check ~10s
python3 motorcycle_sim_blender.py --samples 16   # quick preview ~10-30 min
python3 motorcycle_sim_blender.py --samples 48   # default quality ~30-90 min
python3 motorcycle_sim_blender.py --samples 128  # high quality ~2-6 hr
python3 motorcycle_sim_blender.py --no-render    # physics check only (writes JSON)
```

Output: `C:\Users\shaur\motorcycle_blender.mp4` (override with `--output`)

### Quick single-frame preview (for iterating on visuals)
```bash
# First generate physics JSON (once per session):
python3 motorcycle_sim_blender.py --no-render

# Then render a single frame (fast, ~10s):
TS=$(date +%H%M%S) && WIN_OUT="C:\\Users\\shaur\\moto_preview_${TS}.png" && \
/mnt/c/blender/blender-4.3.2-windows-arm64/blender.exe --background \
  --python "C:\\Users\\shaur\\Documents\\Code_Repository\\3d_sim_creator\\blender_scene.py" \
  -- "\\\\wsl.localhost\\Ubuntu\\tmp\\moto_physics.json" "$WIN_OUT" 8 120
```

## Current Physics Parameters

| Constant | Value | What it controls |
|---|---|---|
| `V_CRUISE` | 12.0 m/s | Cruise speed (~43 km/h) |
| `CAR_BRAKE_TIME` | 3.0 s | When lead car starts braking |
| `CAR_BRAKE_ACCEL` | -9.0 m/s² | Car hard-ABS braking |
| `MOTO_REACT_S` | 1.0 s | ADAS reaction delay |
| `MOTO_BRAKE_ACCEL` | -4.6 m/s² | Motorcycle braking |
| `SIM_DURATION` | 12.0 s | Total sim length |

Final gap at stop: ~0.32 m — safe ✓. Motorcycle drives **straight** (no lateral weave).

## Scene Description

- **Motorcycle**: Black angular sport bike (box geometry), humanoid rider in dark suit + light-grey helmet
- **Car**: Camry-inspired silver sedan with corner-mounted tail lights (L-shaped, on quarter panels)
- **Road**: Straight along +X, 11 m wide, dashed centre line, grass shoulders. Both vehicles in right lane (Y offset +2.5 m)
- **Camera**: 8 m behind / 1.5 m right / 2.8 m up in motorcycle local frame, follows motorcycle

## Blender Coordinate System

- Z-up, vehicles face **+X**, Y = lateral
- Car origin under rear axle; `car.location.x = lead_x - 2.08` (front bumper = lead_x)
- Motorcycle origin: rear axle at ground level
- Tail lights named `BrakeLt_*` — brake-light switcher keys on this prefix

## Rendering

- **Engine**: Cycles CPU + OIDN denoising
- **Color**: AgX, look='None', exposure=-1.2
- **Sun energy**: 0.7, sky strength 0.18
- **Output**: PNG for preview, FFMPEG H264 MP4 for full render
- **Blender path**: `/mnt/c/blender/blender-4.3.2-windows-arm64/blender.exe`
- **Physics JSON**: `/tmp/moto_physics.json` (WSL), read via `\\wsl.localhost\Ubuntu\tmp\moto_physics.json`

## WSL Path Helpers (in motorcycle_sim_blender.py)

- `to_win_path("/mnt/c/foo")` → `"C:\\foo"` (for script path passed to blender.exe)
- `to_wsl_unc("/tmp/foo")` → `"\\\\wsl.localhost\\Ubuntu\\tmp\\foo"` (for JSON read by Windows Blender)

## Common Issues

**Blender can't find physics JSON**: Check that `\\wsl.localhost\Ubuntu` uses your actual distro name (read from `/etc/os-release NAME=`).

**Wrong Blender path**: Update `BLENDER_EXE` constant in `motorcycle_sim_blender.py`.

**No output / black render**: Run `--no-render` first to verify physics, then try `--samples 8` for a quick test.
