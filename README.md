# Motorcycle ADAS Braking Simulation

Physics simulation + Blender 3D rendering of a motorcycle automatic emergency braking (ADAS) scenario. No GPU required — runs entirely on CPU.

## What it produces

A 1920×1080 MP4 video showing:
- Both vehicles cruising at 43 km/h
- Motorcycle doing a gentle lateral weave (~14° lean) — demonstrating active lean stabilisation
- Lead SUV brakes hard (brake lights glow red)
- Motorcycle ADAS detects the event 1 second later and brakes
- Both stop safely with a 7 m gap — no collision

## Requirements

- **Python 3.x** (standard system install)
- **Blender 4.x** (free download, see below)
- **WSL2 on Windows** (or native Linux — see notes)
- No NVIDIA GPU required
- No Python packages needed — only standard library (`math`, `json`, `subprocess`, `argparse`)

## Setup

### 1. Install Blender

**Windows ARM64** (Surface, Snapdragon):
```bash
mkdir -p /mnt/c/blender
curl -L "https://www.blender.org/download/release/Blender4.3/blender-4.3.2-windows-arm64.zip" \
     -o /tmp/blender.zip
unzip /tmp/blender.zip -d /mnt/c/blender/
```

**Windows x86_64** (Intel/AMD):
```bash
mkdir -p /mnt/c/blender
curl -L "https://www.blender.org/download/release/Blender4.3/blender-4.3.2-windows-x64.zip" \
     -o /tmp/blender.zip
unzip /tmp/blender.zip -d /mnt/c/blender/
```

**Native Linux x86_64**:
```bash
mkdir -p ~/blender
curl -L "https://www.blender.org/download/release/Blender4.3/blender-4.3.2-linux-x64.tar.xz" \
     -o /tmp/blender.tar.xz
tar -xf /tmp/blender.tar.xz -C ~/blender/
```

### 2. Clone this repo

```bash
git clone https://github.com/shagarwal/motorcycle-adas-sim.git
cd motorcycle-adas-sim
```

### 3. Set the Blender path

Open `motorcycle_sim_blender.py` and update `BLENDER_EXE` near the top:

```python
# WSL2 ARM64 Windows (Surface/Snapdragon — default):
BLENDER_EXE = "/mnt/c/blender/blender-4.3.2-windows-arm64/blender.exe"

# WSL2 x86_64 Windows (Intel/AMD):
# BLENDER_EXE = "/mnt/c/blender/blender-4.3.2-windows-x64/blender.exe"

# Native Linux:
# BLENDER_EXE = "/home/you/blender/blender-4.3.2-linux-x64/blender"
```

## Usage

```bash
# Quick preview (16 spp, ~10-30 min on laptop CPU)
python3 motorcycle_sim_blender.py --samples 16

# Default quality (48 spp, ~30-90 min)
python3 motorcycle_sim_blender.py

# High quality (128 spp, ~2-6 hours)
python3 motorcycle_sim_blender.py --samples 128

# Custom output path
python3 motorcycle_sim_blender.py --output C:\\Users\\you\\my_video.mp4

# Physics only — no render, writes /tmp/moto_physics.json
python3 motorcycle_sim_blender.py --no-render
```

Output is saved to `C:\Users\<username>\motorcycle_blender.mp4` by default (username auto-detected from `$USER`).

## How it works

```
motorcycle_sim_blender.py          blender_scene.py
        │                                  │
  Physics sim (pure Python)         Blender Python API
  • Euler integration               • Sport motorcycle model
  • Inverted-pendulum lean          • SUV model
  • PD controller                   • Road + grass + sky
  • Raised-cosine lane weave        • Nishita sky + sun light
        │                           • Cycles CPU rendering
        │ writes JSON                • OIDN denoising → MP4
        ▼                                  ▲
  /tmp/moto_physics.json ─────────────────┘
        │
        └─ calls blender.exe --background --python blender_scene.py
```

### Physics parameters (editable in `motorcycle_sim_blender.py`)

| Parameter | Value | Description |
|---|---|---|
| `V_CRUISE` | 12 m/s | Cruise speed (~43 km/h) |
| `CAR_BRAKE_TIME` | 3.0 s | When lead car brakes |
| `CAR_BRAKE_ACCEL` | −7.0 m/s² | Car braking deceleration |
| `MOTO_REACT_S` | 1.0 s | ADAS reaction delay |
| `MOTO_BRAKE_ACCEL` | −6.5 m/s² | Motorcycle braking deceleration |
| `WEAVE_AMP` | 0.8 m | Lane weave amplitude (shows lean) |
| `KP` / `KD` | 3500 / 500 | Lean PD controller gains |

## Render time estimates (CPU only)

| Samples | Quality | Laptop | Desktop |
|---|---|---|---|
| 16 | Preview | 10–30 min | 5–10 min |
| 48 | Good | 30–90 min | 15–30 min |
| 128 | High | 2–6 hours | 45–90 min |

For much faster rendering, use a cloud GPU (RunPod, Vast.ai, Google Colab Pro) and set `cycles.device = 'GPU'` in `blender_scene.py`.

## Files

| File | Purpose |
|---|---|
| `motorcycle_sim_blender.py` | Physics simulation + Blender launcher |
| `blender_scene.py` | Blender scene, materials, animation, render config |
| `CLAUDE.md` | Instructions for Claude Code to resume work on a new machine |
