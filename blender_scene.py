"""
blender_scene.py  —  Motorcycle hard-braking scenario: Blender Python scene script.

Do NOT run this directly. It is called by motorcycle_sim_blender.py via:
    blender.exe --background --python blender_scene.py -- physics.json output.mp4 [samples]

Uses Blender 4.x Python API (bpy). Renders with Cycles CPU + OIDN denoising.
"""

import bpy
import math
import json
import sys
import os
from mathutils import Vector, Euler


def parse_args():
    argv = sys.argv
    try:
        idx  = argv.index("--")
        args = argv[idx + 1:]
    except ValueError:
        args = []
    physics_json = args[0] if len(args) > 0 else "/tmp/moto_physics.json"
    output_path  = args[1] if len(args) > 1 else "/tmp/motorcycle_blender.mp4"
    samples      = int(args[2]) if len(args) > 2 else 48
    return physics_json, output_path, samples


# ══ Material helpers ════════════════════════════════════════════════════════════════════════════

_mat_cache = {}

def make_material(name, base_color, roughness=0.5, metallic=0.0,
                  specular=0.5, clearcoat=0.0, emission=None,
                  emission_strength=0.0, transmission=0.0, ior=1.45):
    """Create and cache a Principled BSDF material."""
    if name in _mat_cache:
        return _mat_cache[name]
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value      = (*base_color, 1.0)
    bsdf.inputs["Roughness"].default_value       = roughness
    bsdf.inputs["Metallic"].default_value        = metallic
    bsdf.inputs["Specular IOR Level"].default_value = specular
    bsdf.inputs["Coat Weight"].default_value     = clearcoat
    bsdf.inputs["Coat Roughness"].default_value  = 0.05
    bsdf.inputs["Transmission Weight"].default_value = transmission
    bsdf.inputs["IOR"].default_value             = ior
    if emission:
        bsdf.inputs["Emission Color"].default_value  = (*emission, 1.0)
        bsdf.inputs["Emission Strength"].default_value = emission_strength
    _mat_cache[name] = mat
    return mat


def assign_mat(obj, mat):
    if obj.data.materials:
        obj.data.materials[0] = mat
    else:
        obj.data.materials.append(mat)


# ══ Scene setup ═════════════════════════════════════════════════════════════════════════════

def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    for col in bpy.data.collections:
        bpy.data.collections.remove(col)


def setup_world():
    world = bpy.data.worlds["World"]
    world.use_nodes = True
    nt = world.node_tree
    for node in nt.nodes:
        nt.nodes.remove(node)
    sky = nt.nodes.new("ShaderNodeTexSky")
    sky.sky_type      = "NISHITA"
    sky.sun_elevation = math.radians(38)
    sky.sun_rotation  = math.radians(215)
    sky.altitude      = 50.0
    sky.air_density   = 1.0
    sky.dust_density  = 0.8
    bg  = nt.nodes.new("ShaderNodeBackground")
    bg.inputs["Strength"].default_value = 1.0
    out = nt.nodes.new("ShaderNodeOutputWorld")
    nt.links.new(sky.outputs["Color"], bg.inputs["Color"])
    nt.links.new(bg.outputs["Background"], out.inputs["Surface"])


def add_sun():
    bpy.ops.object.light_add(type='SUN', location=(50, 30, 80))
    sun = bpy.context.active_object
    sun.name = "Sun"
    sun.data.energy    = 3.5
    sun.data.color     = (1.0, 0.97, 0.88)
    sun.data.angle     = math.radians(0.5)
    sun.rotation_euler = Euler((math.radians(-48), math.radians(12), math.radians(215)))
    return sun


def create_road(length=500.0, half_width=5.5):
    """Asphalt road along +X with lane markings and grass shoulders."""
    x0, x1 = -30.0, length
    y0, y1 = -half_width, half_width
    verts = [(x0,y0,0),(x1,y0,0),(x1,y1,0),(x0,y1,0)]
    mesh = bpy.data.meshes.new("RoadMesh")
    mesh.from_pydata(verts, [], [(0,1,2,3)])
    road = bpy.data.objects.new("Road", mesh)
    bpy.context.collection.objects.link(road)
    assign_mat(road, make_material("Asphalt", (0.10,0.10,0.11), roughness=0.88, specular=0.15))

    # Dashed centre line
    for i in range(int(length / 12)):
        xs = -20 + i * 12
        xe = xs + 5.5
        v  = [(xs,-0.07,0.003),(xe,-0.07,0.003),(xe,0.07,0.003),(xs,0.07,0.003)]
        m  = bpy.data.meshes.new(f"Dash{i}")
        m.from_pydata(v, [], [(0,1,2,3)])
        o  = bpy.data.objects.new(f"Dash{i}", m)
        bpy.context.collection.objects.link(o)
        assign_mat(o, make_material("CentreLine", (0.90,0.90,0.90), roughness=0.85))

    # Edge lines
    for ye in (-half_width+0.2, half_width-0.2):
        v = [(-30,ye-0.10,0.003),(length,ye-0.10,0.003),(length,ye+0.10,0.003),(-30,ye+0.10,0.003)]
        m = bpy.data.meshes.new("EdgeLine")
        m.from_pydata(v, [], [(0,1,2,3)])
        o = bpy.data.objects.new("EdgeLine", m)
        bpy.context.collection.objects.link(o)
        assign_mat(o, make_material("EdgeLine", (0.85,0.85,0.85), roughness=0.85))

    # Grass shoulders
    for ys in (-1, 1):
        yi, yo = ys*half_width, ys*(half_width+40)
        v = [(-30,yi,-0.02),(length,yi,-0.02),(length,yo,-0.02),(-30,yo,-0.02)]
        m = bpy.data.meshes.new("Grass")
        m.from_pydata(v, [], [(0,1,2,3)])
        o = bpy.data.objects.new("Grass", m)
        bpy.context.collection.objects.link(o)
        assign_mat(o, make_material("Grass", (0.14,0.28,0.08), roughness=0.92, specular=0.1))

    return road


# ══ Primitive helpers ═════════════════════════════════════════════════════════════════════════

def _add_sphere(name, location, scale, mat):
    bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0, segments=32, ring_count=20, location=location)
    o = bpy.context.active_object
    o.name = name; o.scale = scale
    mod = o.modifiers.new("SubSurf","SUBSURF"); mod.levels=1; mod.render_levels=2
    assign_mat(o, mat); return o

def _add_cylinder(name, location, rotation_euler, radius, depth, mat, vertices=24):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius, depth=depth, location=location)
    o = bpy.context.active_object
    o.name = name; o.rotation_euler = rotation_euler
    assign_mat(o, mat); return o

def _add_torus(name, location, rotation_euler, major_r, minor_r, mat, major_seg=48, minor_seg=16):
    bpy.ops.mesh.primitive_torus_add(
        major_radius=major_r, minor_radius=minor_r,
        major_segments=major_seg, minor_segments=minor_seg, location=location)
    o = bpy.context.active_object
    o.name = name; o.rotation_euler = rotation_euler
    assign_mat(o, mat); return o

def _add_box(name, location, scale, mat):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=location)
    o = bpy.context.active_object
    o.name = name; o.scale = scale
    b = o.modifiers.new("Bevel","BEVEL"); b.width=0.06; b.segments=3
    s = o.modifiers.new("SubSurf","SUBSURF"); s.levels=1; s.render_levels=2
    assign_mat(o, mat); return o

def parent_to(obj, parent_obj):
    obj.parent = parent_obj
    obj.matrix_parent_inverse = parent_obj.matrix_world.inverted()


# ══ Motorcycle model ═════════════════════════════════════════════════════════════════════════

def create_sport_motorcycle():
    """Sport bike. Origin = ground level under rear axle. Returns parent Empty."""
    m_paint  = make_material("BikeBlue",   (0.02,0.08,0.88), roughness=0.04, metallic=0.0, specular=0.85, clearcoat=1.0)
    m_carbon = make_material("Carbon",     (0.04,0.04,0.05), roughness=0.12, metallic=0.3, specular=0.6)
    m_rubber = make_material("Rubber",     (0.03,0.03,0.03), roughness=0.88)
    m_chrome = make_material("Chrome",     (0.88,0.90,0.95), roughness=0.04, metallic=1.0, specular=1.0)
    m_exhaust= make_material("Exhaust",    (0.72,0.60,0.32), roughness=0.22, metallic=0.95)
    m_suit   = make_material("RacingSuit", (0.90,0.42,0.04), roughness=0.72)
    m_helmet = make_material("Helmet",     (0.88,0.08,0.08), roughness=0.06, specular=0.9, clearcoat=1.0)
    m_visor  = make_material("Visor",      (0.12,0.22,0.38), roughness=0.02, transmission=0.45, ior=1.5, specular=1.0)
    m_headlt = make_material("Headlight",  (1.00,0.97,0.88), roughness=0.0,  emission=(1.0,0.97,0.88), emission_strength=8.0)
    m_taillt = make_material("TailLight",  (1.00,0.06,0.06), roughness=0.04, emission=(1.0,0.05,0.05), emission_strength=2.0)
    m_seat   = make_material("Seat",       (0.06,0.05,0.05), roughness=0.75)
    m_glass  = make_material("Windscreen", (0.65,0.75,0.85), roughness=0.0,  transmission=0.90, ior=1.52, specular=1.0)

    bpy.ops.object.empty_add(type='PLAIN_AXES', location=(0,0,0))
    parent = bpy.context.active_object; parent.name = "Motorcycle"
    parts = []

    wr, wmr = 0.30, 0.074
    parts += [_add_torus("RearWheel",  (0,0,wr),     (math.pi/2,0,0), wr, wmr, m_rubber),
               _add_cylinder("RearRim",(0,0,wr),     (math.pi/2,0,0), 0.205,0.13,m_chrome,32),
               _add_torus("FrontWheel",(1.28,0,wr),  (math.pi/2,0,0), wr, wmr, m_rubber),
               _add_cylinder("FrontRim",(1.28,0,wr), (math.pi/2,0,0), 0.205,0.13,m_chrome,32)]

    parts += [_add_sphere("Frame",        (0.50,0,wr+0.26), (1.0, 0.24,0.24), m_carbon),
               _add_sphere("UpperFairing",(0.55,0,wr+0.44), (0.72,0.295,0.34),m_paint),
               _add_sphere("LowerFairing",(0.48,0,wr+0.22), (0.68,0.28,0.22), m_paint),
               _add_sphere("Tail",        (-0.20,0,wr+0.50),(0.52,0.22,0.20), m_paint),
               _add_sphere("FuelTank",    (0.22,0,wr+0.76), (0.44,0.22,0.20), m_paint),
               _add_sphere("Seat",        (-0.04,0,wr+0.78),(0.50,0.20,0.12), m_seat)]

    for y in (0.055,-0.055):
        parts.append(_add_cylinder(f"Fork_{y}",(1.04,y,wr+0.30),(math.radians(10),0,0),0.022,0.56,m_chrome,12))

    parts += [_add_cylinder("Handlebars",(0.80,0,wr+0.84),(0,math.pi/2,0),0.018,0.46,m_chrome,10),
               _add_sphere("Windscreen", (0.86,0,wr+0.78),(0.18,0.26,0.22),m_glass),
               _add_cylinder("Exhaust",  (0.10,-0.16,wr+0.04),(0,math.pi/2,0),0.038,0.72,m_exhaust,16),
               _add_cylinder("ExhaustTip",(-0.34,-0.16,wr+0.04),(0,math.pi/2,0),0.052,0.10,m_chrome,16),
               _add_sphere("Headlight",  (1.36,0,wr+0.44),(0.10,0.18,0.16),m_headlt),
               _add_sphere("TailLight",  (-0.62,0,wr+0.58),(0.10,0.14,0.09),m_taillt),
               _add_sphere("Hips",  (0.12,0,wr+0.92),(0.32,0.26,0.22),m_suit),
               _add_sphere("Torso", (0.42,0,wr+1.08),(0.40,0.26,0.26),m_suit),
               _add_sphere("RArm",  (0.72,-0.20,wr+0.92),(0.32,0.16,0.16),m_suit),
               _add_sphere("LArm",  (0.72, 0.20,wr+0.92),(0.32,0.16,0.16),m_suit),
               _add_sphere("Helmet",(0.40,0,wr+1.44),(0.19,0.185,0.215),m_helmet),
               _add_sphere("Visor", (0.56,0,wr+1.42),(0.12,0.18,0.165),m_visor)]

    for p in parts:
        parent_to(p, parent)
    return parent


# ══ SUV model ══════════════════════════════════════════════════════════════════════════════

def create_suv(braking: bool = False):
    """Full-size SUV. Origin = ground level under centre. Returns parent Empty."""
    m_body  = make_material("SuvSilver", (0.72,0.74,0.76), roughness=0.08, metallic=0.05, specular=0.9, clearcoat=0.9)
    m_rubber= make_material("Rubber",    (0.03,0.03,0.03), roughness=0.88)
    m_rim   = make_material("SuvRim",    (0.80,0.82,0.88), roughness=0.10, metallic=0.9,  specular=0.9)
    m_glass = make_material("CarGlass",  (0.55,0.65,0.72), roughness=0.02, transmission=0.82, ior=1.52, specular=1.0)
    m_bump  = make_material("Bumper",    (0.35,0.35,0.36), roughness=0.55)
    bl_c    = (1.0,0.06,0.06) if braking else (0.45,0.02,0.02)
    bl_e    = (1.0,0.05,0.05) if braking else (0.40,0.01,0.01)
    m_brake = make_material("BrakeLights_"+("on" if braking else "off"), bl_c, roughness=0.05,
                             emission=bl_e, emission_strength=6.0 if braking else 0.5)
    m_flt   = make_material("FrontLights",(1.0,0.97,0.88), roughness=0.0, emission=(1.0,0.97,0.88), emission_strength=4.0)
    m_under = make_material("SuvUnder",  (0.08,0.08,0.09), roughness=0.85)

    bpy.ops.object.empty_add(type='PLAIN_AXES', location=(0,0,0))
    parent = bpy.context.active_object; parent.name = "Car"
    parts = []
    wr, wmr = 0.365, 0.085

    for xp,yp in [(1.35,0.92),(1.35,-0.92),(-1.35,0.92),(-1.35,-0.92)]:
        parts += [_add_torus(f"Tire_{xp}_{yp}",(xp,yp,wr),(math.pi/2,0,0),wr,wmr,m_rubber,48,16),
                   _add_cylinder(f"Rim_{xp}_{yp}",(xp,yp,wr),(math.pi/2,0,0),0.265,0.19,m_rim,32)]

    parts += [_add_box("LowerBody",(0,0,wr+0.44),(2.05,0.92,0.46),m_body),
               _add_box("Underside",(0,0,wr+0.04),(1.90,0.82,0.08),m_under),
               _add_box("Cabin",(-0.15,0,wr+1.08),(1.28,0.88,0.42),m_body),
               _add_box("Hood", (0.95,0,wr+0.86),(0.76,0.88,0.10),m_body),
               _add_box("Roof", (-0.15,0,wr+1.38),(1.18,0.86,0.08),m_body),
               _add_box("Windshield",(0.50,0,wr+1.10),(0.08,0.82,0.42),m_glass),
               _add_box("RearWindow",(-0.78,0,wr+1.08),(0.08,0.80,0.38),m_glass)]

    for xp in (0.25,-0.40):
        for ys in (1,-1):
            parts.append(_add_box(f"SideWin_{xp}_{ys}",(xp,ys*0.91,wr+1.10),(0.38,0.04,0.35),m_glass))

    parts += [_add_box("FrontBumper",(2.08,0,wr+0.32),(0.10,0.88,0.32),m_bump),
               _add_box("RearBumper",(-2.08,0,wr+0.32),(0.10,0.88,0.32),m_bump)]

    for ys in (1,-1):
        parts += [_add_box(f"HeadLt_{ys}", (2.06, ys*0.68,wr+0.72),(0.06,0.28,0.14),m_flt),
                   _add_box(f"BrakeLt_{ys}",(-2.06,ys*0.68,wr+0.72),(0.06,0.30,0.18),m_brake),
                   _add_cylinder(f"Rail_{ys}",(-0.15,ys*0.80,wr+1.45),(0,math.pi/2,0),0.022,1.10,m_rim,8)]

    for p in parts:
        parent_to(p, parent)
    return parent


# ══ Animation ═════════════════════════════════════════════════════════════════════════════

def animate_scene(data, moto, car, camera, fps):
    scene = bpy.context.scene
    n = len(data["t"])
    step = max(1, int(round((1.0 / (data["t"][1] - data["t"][0])) / fps)))
    frame_num = 1

    for i in range(0, n, step):
        scene.frame_set(frame_num)
        ex = float(data["ego_x"][i]);  ez = float(data["ego_z"][i])
        eh = float(data["ego_heading"][i]);  el = float(data["lean_deg"][i])
        lx = float(data["lead_x"][i]);  lz = float(data["lead_z"][i])
        lh = float(data["lead_heading"][i])

        moto.location       = (ex, ez, 0.0)
        moto.rotation_euler = Euler((math.radians(el), 0.0, eh), 'XYZ')
        moto.keyframe_insert(data_path="location",       frame=frame_num)
        moto.keyframe_insert(data_path="rotation_euler", frame=frame_num)

        car.location        = (lx - 2.08, lz, 0.0)
        car.rotation_euler  = Euler((0.0, 0.0, lh), 'XYZ')
        car.keyframe_insert(data_path="location",       frame=frame_num)
        car.keyframe_insert(data_path="rotation_euler", frame=frame_num)

        cos_h, sin_h = math.cos(eh), math.sin(eh)
        camera.location = (ex + (-8.0)*cos_h - 2.5*sin_h,
                           ez + (-8.0)*sin_h + 2.5*cos_h, 2.8)
        camera.keyframe_insert(data_path="location", frame=frame_num)
        frame_num += 1

    scene.frame_start = 1; scene.frame_end = frame_num - 1

    for obj in (moto, car, camera):
        if obj.animation_data:
            for fc in obj.animation_data.action.fcurves:
                for kf in fc.keyframe_points:
                    kf.interpolation = 'LINEAR'


# ══ Camera ══════════════════════════════════════════════════════════════════════════════

def create_camera(moto_obj):
    bpy.ops.object.camera_add(location=(0, 0, 3))
    cam = bpy.context.active_object; cam.name = "FollowCam"
    cam.data.lens = 35
    cam.data.dof.use_dof = True
    cam.data.dof.focus_object = moto_obj
    cam.data.dof.aperture_fstop = 4.0
    track = cam.constraints.new(type='TRACK_TO')
    track.target = moto_obj; track.track_axis = 'TRACK_NEGATIVE_Z'; track.up_axis = 'UP_Y'
    bpy.context.scene.camera = cam
    return cam


# ══ Render config ═══════════════════════════════════════════════════════════════════════════

def configure_render(output_path, width, height, fps, samples):
    scene = bpy.context.scene; r = scene.render
    r.engine = 'CYCLES'; r.resolution_x = width; r.resolution_y = height; r.fps = fps
    r.image_settings.file_format = 'FFMPEG'
    r.ffmpeg.format = 'MPEG4'; r.ffmpeg.codec = 'H264'
    r.ffmpeg.constant_rate_factor = 'HIGH'; r.ffmpeg.ffmpeg_preset = 'GOOD'
    r.filepath = output_path
    c = scene.cycles
    c.device = 'CPU'; c.samples = samples
    c.use_denoising = True; c.denoiser = 'OPENIMAGEDENOISE'
    c.denoising_input_passes = 'RGB_ALBEDO_NORMAL'; c.use_fast_gi = True
    scene.view_settings.view_transform = 'Filmic'
    scene.view_settings.look = 'Medium High Contrast'


# ══ Main ══════════════════════════════════════════════════════════════════════════════

def main():
    physics_json, output_path, samples = parse_args()
    print(f"\n[Blender] Loading: {physics_json}")
    with open(physics_json) as f:
        data = json.load(f)
    fps    = data.get("fps",   30)
    width  = data.get("width", 1280)
    height = data.get("height", 720)
    print(f"[Blender] {len(data['t'])} frames @ {fps} fps")

    clear_scene(); setup_world(); add_sun(); create_road(500)
    print("[Blender] Building motorcycle ...")
    moto = create_sport_motorcycle()
    print("[Blender] Building car ...")
    car  = create_suv(braking=False)
    cam  = create_camera(moto)
    print("[Blender] Writing keyframes ...")
    animate_scene(data, moto, car, cam, fps)

    # Activate brake lights
    braking_mat = make_material("BrakeLights_on",(1.0,0.06,0.06),roughness=0.05,
                                 emission=(1.0,0.05,0.05),emission_strength=8.0)
    for obj in bpy.data.objects:
        if obj.name.startswith("BrakeLt_"):
            obj.data.materials[0] = braking_mat

    configure_render(output_path, width, height, fps, samples)
    total = bpy.context.scene.frame_end - bpy.context.scene.frame_start + 1
    print(f"[Blender] Rendering {total} frames → {output_path}")
    bpy.ops.render.render(animation=True)
    print(f"\n[Blender] Done → {output_path}")


main()
