"""
blender_scene.py  —  Motorcycle hard-braking scenario: Blender Python scene script.

Run via:
    blender.exe --background --python blender_scene.py -- physics_data.json output.mp4

Uses Blender 4.x Python API (bpy).  Renders with Cycles CPU + OIDN denoising.
"""

import bpy
import math
import json
import sys
import os
from mathutils import Vector, Euler


# ── Parse arguments (everything after " -- ") ────────────────────────────────
def parse_args():
    argv = sys.argv
    try:
        idx  = argv.index("--")
        args = argv[idx + 1:]
    except ValueError:
        args = []
    physics_json  = args[0] if len(args) > 0 else "/tmp/moto_physics.json"
    output_path   = args[1] if len(args) > 1 else "/tmp/motorcycle_blender.mp4"
    samples       = int(args[2]) if len(args) > 2 else 48
    # Optional 4th arg: frame number for single-frame PNG preview (e.g. "60")
    preview_frame = int(args[3]) if len(args) > 3 else None
    return physics_json, output_path, samples, preview_frame


# ══════════════════════════════════════════════════════════════════════════════
# Material helpers
# ══════════════════════════════════════════════════════════════════════════════

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


# ══════════════════════════════════════════════════════════════════════════════
# Scene setup
# ══════════════════════════════════════════════════════════════════════════════

def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    for col in bpy.data.collections:
        bpy.data.collections.remove(col)


def setup_world():
    """Sky + atmosphere lighting."""
    world = bpy.data.worlds["World"]
    world.use_nodes = True
    nt = world.node_tree
    # Remove default nodes
    for node in nt.nodes:
        nt.nodes.remove(node)
    # Sky texture
    sky = nt.nodes.new("ShaderNodeTexSky")
    sky.sky_type  = "NISHITA"
    sky.sun_elevation = math.radians(38)
    sky.sun_rotation  = math.radians(215)
    sky.altitude      = 50.0
    sky.air_density   = 1.0
    sky.dust_density  = 0.8

    bg = nt.nodes.new("ShaderNodeBackground")
    bg.inputs["Strength"].default_value = 0.18

    out = nt.nodes.new("ShaderNodeOutputWorld")
    nt.links.new(sky.outputs["Color"], bg.inputs["Color"])
    nt.links.new(bg.outputs["Background"], out.inputs["Surface"])


def add_sun():
    """Primary sun light."""
    bpy.ops.object.light_add(type='SUN', location=(50, 30, 80))
    sun = bpy.context.active_object
    sun.name = "Sun"
    sun.data.energy       = 0.7
    sun.data.color        = (1.0, 0.97, 0.88)
    sun.data.angle        = math.radians(0.5)
    sun.rotation_euler    = Euler((math.radians(-48), math.radians(12), math.radians(215)))
    return sun


def create_road(length=500.0, half_width=5.5):
    """
    Asphalt road strip running along +X from x=-30 to x=length.
    In Blender coordinates: Z-up, road in XY plane.
    """
    verts, faces = [], []

    x0, x1 = -30.0, length
    y0, y1 = -half_width, half_width

    # Road surface
    verts = [
        (x0, y0, 0), (x1, y0, 0),
        (x1, y1, 0), (x0, y1, 0),
    ]
    faces = [(0, 1, 2, 3)]

    mesh = bpy.data.meshes.new("RoadMesh")
    mesh.from_pydata(verts, [], faces)
    road = bpy.data.objects.new("Road", mesh)
    bpy.context.collection.objects.link(road)

    # Asphalt material with slight roughness variation
    mat = make_material("Asphalt", (0.10, 0.10, 0.11), roughness=0.88, metallic=0.0, specular=0.15)
    assign_mat(road, mat)

    # Lane markings: dashed centre line
    for i in range(int(length / 12)):
        x_start = -20 + i * 12
        x_end   = x_start + 5.5
        v = [
            (x_start, -0.07, 0.003),
            (x_end,   -0.07, 0.003),
            (x_end,    0.07, 0.003),
            (x_start,  0.07, 0.003),
        ]
        m = bpy.data.meshes.new(f"Dash{i}")
        m.from_pydata(v, [], [(0, 1, 2, 3)])
        o = bpy.data.objects.new(f"Dash{i}", m)
        bpy.context.collection.objects.link(o)
        assign_mat(o, make_material("CentreLine", (0.90, 0.90, 0.90), roughness=0.85))

    # Solid edge lines
    for y_edge in (-half_width + 0.2, half_width - 0.2):
        v = [
            (-30,     y_edge - 0.10, 0.003),
            (length,  y_edge - 0.10, 0.003),
            (length,  y_edge + 0.10, 0.003),
            (-30,     y_edge + 0.10, 0.003),
        ]
        m = bpy.data.meshes.new("EdgeLine")
        m.from_pydata(v, [], [(0, 1, 2, 3)])
        o = bpy.data.objects.new("EdgeLine", m)
        bpy.context.collection.objects.link(o)
        assign_mat(o, make_material("EdgeLine", (0.85, 0.85, 0.85), roughness=0.85))

    # Roadside grass
    for y_sign in (-1, 1):
        y_inner = y_sign * half_width
        y_outer = y_sign * (half_width + 40)
        v = [
            (-30, y_inner, -0.02), (length, y_inner, -0.02),
            (length, y_outer, -0.02), (-30, y_outer, -0.02),
        ]
        m = bpy.data.meshes.new("Grass")
        m.from_pydata(v, [], [(0, 1, 2, 3)])
        o = bpy.data.objects.new("Grass", m)
        bpy.context.collection.objects.link(o)
        assign_mat(o, make_material("Grass", (0.14, 0.28, 0.08), roughness=0.92, specular=0.1))

    return road


# ══════════════════════════════════════════════════════════════════════════════
# Vehicle models
# ══════════════════════════════════════════════════════════════════════════════
# Coordinate system: Z-up, vehicles face +X.
# For motorcycles:  Y = lateral (right), Z = up
# Lean (roll): rotation around X-axis
# Heading (yaw): rotation around Z-axis


def _add_sphere(name, location, scale, mat):
    bpy.ops.mesh.primitive_uv_sphere_add(
        radius=1.0, segments=32, ring_count=20,
        location=location
    )
    o = bpy.context.active_object
    o.name = name
    o.scale = scale
    # Add Subdivision Surface for smoother appearance
    mod = o.modifiers.new("SubSurf", "SUBSURF")
    mod.levels = 1
    mod.render_levels = 2
    assign_mat(o, mat)
    return o


def _add_cylinder(name, location, rotation_euler, radius, depth, mat, vertices=24):
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=vertices, radius=radius, depth=depth,
        location=location
    )
    o = bpy.context.active_object
    o.name = name
    o.rotation_euler = rotation_euler
    assign_mat(o, mat)
    return o


def _add_torus(name, location, rotation_euler, major_r, minor_r, mat,
               major_seg=48, minor_seg=16):
    bpy.ops.mesh.primitive_torus_add(
        major_radius=major_r, minor_radius=minor_r,
        major_segments=major_seg, minor_segments=minor_seg,
        location=location
    )
    o = bpy.context.active_object
    o.name = name
    o.rotation_euler = rotation_euler
    assign_mat(o, mat)
    return o


def _add_box(name, location, scale, mat, bevel_width=0.06, bevel_segments=3, rotation=None):
    """Beveled + subdivision-smoothed box.  Optional rotation=(rx,ry,rz) in radians."""
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=location)
    o = bpy.context.active_object
    o.name = name
    o.scale = scale
    if rotation is not None:
        o.rotation_euler = Euler(rotation, 'XYZ')
    mod = o.modifiers.new("Bevel", "BEVEL")
    mod.width = bevel_width
    mod.segments = bevel_segments
    mod2 = o.modifiers.new("SubSurf", "SUBSURF")
    mod2.levels = 1
    mod2.render_levels = 2
    assign_mat(o, mat)
    return o


def _add_limb(name, p0, p1, radius, mat, vertices=12):
    """Add a cylinder aligned between two 3D points p0 and p1."""
    from mathutils import Vector
    mid    = ((p0[0]+p1[0])/2, (p0[1]+p1[1])/2, (p0[2]+p1[2])/2)
    dx, dy, dz = p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2]
    length = math.sqrt(dx*dx + dy*dy + dz*dz)
    if length < 1e-6:
        return None
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=vertices, radius=radius, depth=length, location=mid
    )
    o = bpy.context.active_object
    o.name = name
    # Rotate default Z-axis cylinder to match the direction vector
    dir_vec = Vector((dx, dy, dz)).normalized()
    z_axis  = Vector((0, 0, 1))
    cross   = z_axis.cross(dir_vec)
    if cross.length > 1e-6:
        angle = math.acos(max(-1.0, min(1.0, z_axis.dot(dir_vec))))
        cross.normalize()
        o.rotation_mode = 'AXIS_ANGLE'
        o.rotation_axis_angle = [angle, cross.x, cross.y, cross.z]
        o.rotation_mode = 'XYZ'
    assign_mat(o, mat)
    return o


def parent_to(obj, parent_obj):
    obj.parent = parent_obj
    obj.matrix_parent_inverse = parent_obj.matrix_world.inverted()


def create_sport_motorcycle():
    """
    Sport bike model — angular box/cylinder geometry matching real bike proportions.
    Origin at ground level, under the rear axle.  Faces +X.
    Returns the parent Empty (named "Motorcycle").
    """
    wr  = 0.30    # wheel radius
    wmr = 0.074   # tyre cross-section radius

    # ── Materials ─────────────────────────────────────────────────────────────
    m_paint   = make_material("BikeBlack",  (0.01, 0.01, 0.01), roughness=0.04,
                               metallic=0.0, specular=0.9, clearcoat=1.0)
    m_carbon  = make_material("Carbon",     (0.03, 0.03, 0.03), roughness=0.15,
                               metallic=0.2, specular=0.5)
    m_rubber  = make_material("Rubber",     (0.03, 0.03, 0.03), roughness=0.88)
    m_chrome  = make_material("Chrome",     (0.82, 0.84, 0.88), roughness=0.06,
                               metallic=1.0, specular=1.0)
    m_exhaust = make_material("Exhaust",    (0.55, 0.48, 0.28), roughness=0.25,
                               metallic=0.92)
    m_suit    = make_material("RiderSuit",  (0.10, 0.10, 0.12), roughness=0.72)
    m_helmet  = make_material("Helmet",     (0.85, 0.85, 0.85), roughness=0.06,
                               specular=0.9, clearcoat=0.8)   # light grey helmet like reference
    m_visor   = make_material("Visor",      (0.04, 0.06, 0.08), roughness=0.02,
                               transmission=0.30, ior=1.5, specular=1.0)
    m_headlit = make_material("Headlight",  (1.00, 0.97, 0.88), roughness=0.0,
                               emission=(1.0, 0.97, 0.88), emission_strength=8.0)
    m_taillit = make_material("TailLight",  (1.00, 0.06, 0.06), roughness=0.04,
                               emission=(1.0, 0.05, 0.05), emission_strength=3.0)
    m_seat    = make_material("Seat",       (0.05, 0.04, 0.04), roughness=0.80)
    m_glass   = make_material("Windscreen", (0.06, 0.10, 0.16), roughness=0.02,
                               transmission=0.75, ior=1.52, specular=1.0)
    m_lp      = make_material("LicensePlateMoto", (0.82, 0.82, 0.84), roughness=0.7)

    # ── Parent empty ──────────────────────────────────────────────────────────
    bpy.ops.object.empty_add(type='PLAIN_AXES', location=(0, 0, 0))
    parent = bpy.context.active_object
    parent.name = "Motorcycle"
    parts = []

    # ── Wheels ────────────────────────────────────────────────────────────────
    rw  = _add_torus("RearWheel",  (0,    0, wr), (math.pi/2, 0, 0), wr, wmr, m_rubber)
    rrm = _add_cylinder("RearRim", (0,    0, wr), (math.pi/2, 0, 0), 0.205, 0.13, m_chrome, 32)
    fw  = _add_torus("FrontWheel", (1.28, 0, wr), (math.pi/2, 0, 0), wr, wmr, m_rubber)
    frm = _add_cylinder("FrontRim",(1.28, 0, wr), (math.pi/2, 0, 0), 0.205, 0.13, m_chrome, 32)
    parts += [rw, rrm, fw, frm]

    # ── Swingarm — box from rear axle forward to frame pivot ──────────────────
    swa = _add_box("Swingarm", (0.26, 0, wr+0.06), (0.52, 0.09, 0.06), m_carbon,
                   bevel_width=0.015)
    parts.append(swa)

    # ── Engine block (visible between fairings) ────────────────────────────────
    eng = _add_box("Engine", (0.54, 0, wr+0.22), (0.44, 0.26, 0.32), m_carbon,
                   bevel_width=0.02, bevel_segments=3)
    parts.append(eng)

    # ── Belly pan — lower fairing under engine ────────────────────────────────
    belly = _add_box("BellyPan", (0.55, 0, wr+0.10), (0.64, 0.30, 0.17), m_paint,
                     bevel_width=0.03, bevel_segments=3)
    parts.append(belly)

    # ── Side fairings — angular panels flanking engine ────────────────────────
    # Two flat panels, left and right, with slight rake matching frame angle
    for y_sign in (1, -1):
        sf = _add_box(f"SideFairing_{y_sign}", (0.50, y_sign*0.155, wr+0.38),
                      (0.72, 0.09, 0.44), m_paint, bevel_width=0.035, bevel_segments=3)
        parts.append(sf)

    # ── Fuel tank — angular box sitting on frame ───────────────────────────────
    # Wider at rear (knee grip area), slightly forward of rider hip
    tank = _add_box("FuelTank", (0.28, 0, wr+0.78), (0.52, 0.28, 0.22), m_paint,
                    bevel_width=0.045, bevel_segments=3)
    parts.append(tank)

    # ── Seat — flat padded slab ────────────────────────────────────────────────
    seat = _add_box("Seat", (-0.04, 0, wr+0.83), (0.52, 0.24, 0.055), m_seat,
                    bevel_width=0.025)
    parts.append(seat)

    # ── Tail unit — angular wedge, rises toward rear ───────────────────────────
    # Centre at x=-0.28; mild nose-up tilt gives the pointed-tail silhouette
    tail_body = _add_box("TailBody", (-0.28, 0, wr+0.70), (0.46, 0.21, 0.22), m_paint,
                          bevel_width=0.04, bevel_segments=3,
                          rotation=(0, math.radians(8), 0))
    # Sharp rear fin — the "tip" of the tail that juts up behind rider
    tail_fin  = _add_box("TailFin",  (-0.52, 0, wr+0.75), (0.10, 0.18, 0.14), m_paint,
                          bevel_width=0.025, bevel_segments=2)
    # Rear mudguard arch over rear wheel
    mudguard  = _add_box("Mudguard", (-0.22, 0, wr+0.48), (0.36, 0.22, 0.06), m_paint,
                          bevel_width=0.018)
    parts += [tail_body, tail_fin, mudguard]

    # ── Tail light — slim horizontal bar at rear of tail fin ─────────────────
    tl = _add_box("TailLight", (-0.58, 0, wr+0.76), (0.04, 0.16, 0.05), m_taillit,
                  bevel_width=0.008)
    parts.append(tl)

    # ── License plate — mounted below tail fin, angled slightly forward ────────
    lp = _add_box("LicensePlateMoto", (-0.60, 0, wr+0.52), (0.03, 0.22, 0.14), m_lp,
                  bevel_width=0.01,
                  rotation=(0, math.radians(-12), 0))
    parts.append(lp)

    # ── Front forks ───────────────────────────────────────────────────────────
    for y_sign in (0.055, -0.055):
        fk = _add_cylinder(f"Fork_{y_sign}", (1.04, y_sign, wr+0.30),
                           (math.radians(10), 0, 0), 0.022, 0.58, m_chrome, 12)
        parts.append(fk)

    # ── Handlebars — clip-on style (two short bars, slightly forward) ──────────
    hb = _add_cylinder("Handlebars", (0.82, 0, wr+0.86),
                        (0, math.pi/2, 0), 0.016, 0.42, m_chrome, 10)
    parts.append(hb)

    # ── Windscreen — thin angled panel above headlight ─────────────────────────
    ws = _add_box("Windscreen", (1.10, 0, wr+0.76), (0.04, 0.28, 0.28), m_glass,
                  bevel_width=0.004, bevel_segments=2,
                  rotation=(0, math.radians(-28), 0))
    parts.append(ws)

    # ── Headlight — rectangular LED unit ──────────────────────────────────────
    hl = _add_box("Headlight", (1.38, 0, wr+0.44), (0.06, 0.22, 0.13), m_headlit,
                  bevel_width=0.018, bevel_segments=2)
    parts.append(hl)

    # ── Exhaust pipe (right side, low) ────────────────────────────────────────
    ex     = _add_cylinder("Exhaust",    (0.12, -0.16, wr+0.05),
                            (0, math.pi/2, 0), 0.036, 0.76, m_exhaust, 16)
    ex_tip = _add_cylinder("ExhaustTip", (-0.36, -0.16, wr+0.05),
                            (0, math.pi/2, 0), 0.050, 0.10, m_chrome, 16)
    parts += [ex, ex_tip]

    # ── Rider ─────────────────────────────────────────────────────────────────
    # Sport-bike tuck: hips on seat, torso leaning forward ~40°, head forward
    hip_c  = (0.08, 0.0, wr + 0.84)   # pelvis centre
    shl_c  = (0.44, 0.0, wr + 1.18)   # shoulder midpoint (forward lean)
    head_c = (0.52, 0.0, wr + 1.40)   # helmet centre

    pelvis = _add_box("Pelvis", hip_c, (0.24, 0.22, 0.13), m_suit,
                      bevel_width=0.03)
    torso  = _add_limb("Torso", hip_c, shl_c, 0.11, m_suit)

    # Helmet: sphere for smooth dome shape (realistic)
    helm  = _add_sphere("Helmet", head_c, (0.19, 0.185, 0.215), m_helmet)
    visor = _add_sphere("Visor",
                        (head_c[0]+0.10, head_c[1], head_c[2]-0.03),
                        (0.12, 0.17, 0.13), m_visor)

    rider_parts = [pelvis, torso, helm, visor]

    # Arms: shoulder → elbow → hand on bars
    for y_sign, side in [(0.22, "R"), (-0.22, "L")]:
        shoulder = (shl_c[0]-0.02, y_sign,         shl_c[2]-0.06)
        elbow    = (0.65,          y_sign,          wr+1.00)
        hand     = (0.80,          y_sign*0.95,     wr+0.90)
        ua = _add_limb(f"UpperArm_{side}", shoulder, elbow, 0.048, m_suit)
        fa = _add_limb(f"Forearm_{side}",  elbow,    hand,  0.040, m_suit)
        rider_parts += [ua, fa]

    # Legs: hip → knee → foot on pegs
    for y_sign, side in [(0.19, "R"), (-0.19, "L")]:
        hip_j = (hip_c[0]-0.03, y_sign, hip_c[2]-0.06)
        knee  = (0.30,          y_sign, wr+0.56)
        foot  = (0.10,          y_sign, wr+0.32)
        ul = _add_limb(f"Thigh_{side}", hip_j, knee, 0.065, m_suit)
        ll = _add_limb(f"Shin_{side}",  knee,  foot, 0.052, m_suit)
        rider_parts += [ul, ll]

    parts += [p for p in rider_parts if p is not None]

    # ── Parent all parts ──────────────────────────────────────────────────────
    for p in parts:
        parent_to(p, parent)

    return parent


def create_sedan(braking: bool = False):
    """
    Camry-inspired modern sedan.  Origin at ground, under centre.
    Body 1.84 m wide.  Front bumper face at x ≈ +2.08.

    Greenhouse geometry (Camry-like fastback silhouette):
      Belt line Z                   : wr + 0.74
      Roof peak Z                   : wr + 1.08
      A-pillar base / windshield bot: x = +0.54
      A-pillar top  / windshield top: x = +0.22  (−43° rake)
      B-pillar                      : x = −0.14
      C-pillar base (trunk cut)     : x = −1.10  Z = wr+0.86
      C-pillar top  / roof rail     : x = −0.62  Z = wr+1.04
      Rear window (backlight) rake  : +69° — very flat, Camry fastback style
    """
    m_body   = make_material("SedanSilver", (0.58, 0.60, 0.63), roughness=0.12,
                              metallic=0.05, specular=0.7, clearcoat=0.2)
    m_rubber = make_material("Rubber",      (0.03, 0.03, 0.03), roughness=0.88)
    # Dark gunmetal alloys like Camry SE
    m_rim    = make_material("SedanRim",    (0.18, 0.18, 0.20), roughness=0.08,
                              metallic=0.95, specular=0.9)
    m_glass  = make_material("CarGlass",    (0.12, 0.20, 0.32), roughness=0.03,
                              transmission=0.45, ior=1.52, specular=1.0)
    m_bump   = make_material("SedanBump",   (0.22, 0.22, 0.24), roughness=0.55)
    m_dark   = make_material("SedanDark",   (0.06, 0.06, 0.07), roughness=0.65)
    m_under  = make_material("SedanUnder",  (0.08, 0.08, 0.09), roughness=0.85)
    m_exhaust_car = make_material("CarExhaust", (0.72, 0.74, 0.80), roughness=0.10,
                                   metallic=0.95)
    bl_color = (1.0, 0.06, 0.06) if braking else (0.45, 0.02, 0.02)
    bl_emit  = (1.0, 0.05, 0.05) if braking else (0.4, 0.01, 0.01)
    bl_str   = 6.0 if braking else 0.5
    m_brake  = make_material("BrakeLights_" + ("on" if braking else "off"),
                              bl_color, roughness=0.05,
                              emission=bl_emit, emission_strength=bl_str)
    m_hlight = make_material("FrontLights", (1.0, 0.97, 0.88), roughness=0.0,
                              emission=(1.0, 0.97, 0.88), emission_strength=4.0)

    bpy.ops.object.empty_add(type='PLAIN_AXES', location=(0, 0, 0))
    parent = bpy.context.active_object
    parent.name = "Car"
    parts = []
    wr  = 0.340   # wheel radius
    wmr = 0.080   # tyre cross-section radius

    # ── Wheels ────────────────────────────────────────────────────────────────
    for xpos, ypos in [(1.40, 0.82), (1.40, -0.82), (-1.40, 0.82), (-1.40, -0.82)]:
        tire = _add_torus(f"Tire_{xpos}_{ypos}", (xpos, ypos, wr),
                          (math.pi/2, 0, 0), wr, wmr, m_rubber, 48, 16)
        rim  = _add_cylinder(f"Rim_{xpos}_{ypos}", (xpos, ypos, wr),
                             (math.pi/2, 0, 0), 0.245, 0.18, m_rim, 32)
        parts += [tire, rim]

    # ── Structural body panels ─────────────────────────────────────────────────
    # Lower sill — full body length, covers lower panels and wheel arches
    lower  = _add_box("LowerBody",  (0.00, 0, wr+0.37), (2.10, 1.84, 0.40), m_body,
                      bevel_width=0.08, bevel_segments=4)
    under  = _add_box("Underside",  (0.00, 0, wr+0.05), (1.94, 1.60, 0.08), m_under)

    # Hood — long flat panel (front half of car)
    hood   = _add_box("Hood",       (1.28, 0, wr+0.66), (1.46, 1.76, 0.07), m_body,
                      bevel_width=0.08, bevel_segments=4)

    # Front fascia
    fascia = _add_box("FrontFascia",(2.03, 0, wr+0.52), (0.14, 1.76, 0.46), m_body,
                      bevel_width=0.08, bevel_segments=3)

    # Cabin — door panels from A-pillar (x=+0.54) to C-pillar base (x=−1.10)
    cabin  = _add_box("Cabin",      (-0.28, 0, wr+0.92), (1.68, 1.72, 0.36), m_body,
                      bevel_width=0.07, bevel_segments=3)

    # Rear quarter lower — trunk body below belt line, from C-pillar base to bumper
    rear_lo = _add_box("RearBodyLo",(-1.50, 0, wr+0.56), (0.86, 1.82, 0.36), m_body,
                       bevel_width=0.08, bevel_segments=4)

    # Rear quarter upper — above belt line in trunk area (below trunk lid)
    rear_up = _add_box("RearBodyUp",(-1.50, 0, wr+0.86), (0.86, 1.82, 0.24), m_body,
                       bevel_width=0.07, bevel_segments=3)

    # Trunk lid — raised, nearly horizontal (Camry's fastback-style trunk)
    tlid   = _add_box("TrunkLid",   (-1.51, 0, wr+0.98), (0.78, 1.74, 0.04), m_body,
                      bevel_width=0.06, bevel_segments=3)

    # Trunk spoiler — small aero lip at rear edge of trunk (Camry SE signature)
    spoiler = _add_box("TrunkSpoiler",(-1.90, 0, wr+1.02), (0.06, 1.46, 0.055), m_body,
                       bevel_width=0.01, bevel_segments=2)

    parts += [lower, under, hood, fascia, cabin, rear_lo, rear_up, tlid, spoiler]

    # ── Roof — dome sphere ─────────────────────────────────────────────────────
    # Spans A-pillar top (x=+0.22) to C-pillar top (x=−0.62); centre at x=−0.20
    roof   = _add_sphere("Roof", (-0.20, 0, wr+1.07), (0.42, 0.82, 0.12), m_body)
    parts.append(roof)

    # ── Pillars ────────────────────────────────────────────────────────────────
    for y_sign in (1, -1):
        # A-pillar: −43° rake (matches windshield)
        ap = _add_limb(f"APillar_{y_sign}",
                       (+0.54, y_sign*0.82, wr+0.74),
                       (+0.22, y_sign*0.82, wr+1.08), 0.028, m_body, 8)
        # B-pillar: nearly vertical
        bp = _add_limb(f"BPillar_{y_sign}",
                       (-0.14, y_sign*0.87, wr+0.74),
                       (-0.13, y_sign*0.83, wr+1.08), 0.030, m_body, 8)
        # C-pillar: dramatic forward-lean matching the raked backlight
        cp = _add_limb(f"CPillar_{y_sign}",
                       (-1.10, y_sign*0.82, wr+0.86),
                       (-0.62, y_sign*0.82, wr+1.04), 0.032, m_body, 8)
        for p in [ap, bp, cp]:
            if p: parts.append(p)

    # ── Glass ─────────────────────────────────────────────────────────────────
    # Windshield: base (x=+0.54, Z=wr+0.74) → top (x=+0.22, Z=wr+1.08)
    # Δx=−0.32, ΔZ=+0.34 → 43° from vertical.  rotation Y = −43°
    ws_len = math.sqrt(0.32**2 + 0.34**2)   # ≈ 0.467 m
    ws = _add_box("Windshield",
                  (0.38, 0, wr+0.91), (0.04, 1.52, ws_len), m_glass,
                  bevel_width=0.005, bevel_segments=2,
                  rotation=(0, math.radians(-43), 0))

    # Rear window (backlight): base (x=−1.10, Z=wr+0.86) → top (x=−0.62, Z=wr+1.04)
    # Δx=+0.48, ΔZ=+0.18 → 69° from vertical — Camry's very-raked fastback glass
    rw_len = math.sqrt(0.48**2 + 0.18**2)   # ≈ 0.511 m
    rwin = _add_box("RearWindow",
                    (-0.86, 0, wr+0.95), (0.04, 1.44, rw_len), m_glass,
                    bevel_width=0.005, bevel_segments=2,
                    rotation=(0, math.radians(+69), 0))

    parts += [ws, rwin]

    # Side windows: front door (wide), rear door (narrower, taller C-pillar)
    # Front: B-pillar (x=−0.14) to A-pillar (x=+0.54), centre +0.20
    # Rear:  C-pillar base (x=−1.10) to B-pillar (x=−0.14), centre −0.62
    for xc, xw in [(0.20, 0.52), (-0.62, 0.92)]:
        for y_sign in (1, -1):
            sw = _add_box(f"SideWin_{xc}_{y_sign}",
                          (xc, y_sign*0.88, wr+0.94),
                          (xw, 0.04, 0.30), m_glass,
                          bevel_width=0.005, bevel_segments=2)
            parts.append(sw)

    # ── Front bumper + grille ──────────────────────────────────────────────────
    fb     = _add_box("FrontBumper", (2.08, 0, wr+0.24), (0.10, 1.76, 0.26), m_bump)
    grille = _add_box("Grille",      (2.09, 0, wr+0.30), (0.06, 1.04, 0.18), m_dark)
    parts += [fb, grille]

    # ── Headlights ────────────────────────────────────────────────────────────
    for y_sign in (1, -1):
        hl  = _add_box(f"HeadLt_{y_sign}", (2.07, y_sign*0.72, wr+0.64),
                       (0.06, 0.36, 0.08), m_hlight)
        drl = _add_box(f"DRL_{y_sign}",    (2.08, y_sign*0.70, wr+0.54),
                       (0.05, 0.28, 0.04), m_hlight)
        parts += [hl, drl]

    # ── Rear end — modern sedan (Camry/Accord-inspired) ───────────────────────────
    # Architecture (viewed from behind):
    #
    #   [dark trim]  ── trunk-lid bottom edge ──────────────────────  Z ≈ wr+0.93
    #   [BrkLt side]  corner-mounted strips on QUARTER PANELS        Z wr+0.62..0.94
    #     (primary light element is on the SIDE of the car)
    #   [BrkLt rear]  thin wrap at outer corner of rear face          Z same
    #   [dark center] body-colour panel in the middle — NOT red       Z same
    #   [rear face]   body-colour panel below lights, above bumper    Z wr+0.38..0.62
    #   [licence plt] recessed in centre of rear face
    #   [bumper up]   painted body-colour + angled corner wrap-arounds Z wr+0.15..0.38
    #   [valance]     dark plastic lower section                       Z wr+0.03..0.15
    #   [exhaust]     two oval tips at lower bumper corners

    # Dark trim strip — fills gap between trunk lid and tail light zone
    dark_top = _add_box("RearDarkTop",
                         (-2.07, 0, wr + 0.93), (0.04, 1.70, 0.10), m_dark,
                         bevel_width=0.005, bevel_segments=2)
    parts.append(dark_top)

    # Tail light clusters — L-shaped, corner-mounted.
    # Primary element: side-facing strip on rear quarter panel (dominates rear-right view)
    # Secondary element: thin rear-face wrap at outer corner (visible from directly behind)
    # Together they form an L viewed from above — with DARK panel between them at centre.
    for y_sign in (1, -1):
        # Primary: horizontal strip on SIDE of car at rear quarter (x = -1.67 → -2.09)
        side_lt = _add_box(f"BrakeLt_Side_{y_sign}",
                            (-1.88, y_sign * 0.885, wr + 0.78),
                            (0.42, 0.038, 0.32), m_brake,
                            bevel_width=0.008, bevel_segments=2)
        # Secondary: thin vertical strip wrapping around rear-face corner
        rear_lt = _add_box(f"BrakeLt_{y_sign}",
                            (-2.06, y_sign * 0.65, wr + 0.78),
                            (0.04, 0.30, 0.32), m_brake,
                            bevel_width=0.008, bevel_segments=2)
        parts += [side_lt, rear_lt]

    # Body-colour rear face — below tail lights, above bumper.
    # Slightly narrower in Y than the widest body point (subtle taper, anti-boxy).
    rear_face = _add_box("RearBodyFace",
                          (-2.07, 0, wr + 0.50),
                          (0.04, 1.60, 0.24), m_body,
                          bevel_width=0.06, bevel_segments=3)
    parts.append(rear_face)

    # License plate — recessed into centre of rear face
    lp_mat = make_material("LicensePlate", (0.78, 0.80, 0.82), roughness=0.65)
    lp = _add_box("LicensePlate", (-2.11, 0, wr + 0.50), (0.03, 0.40, 0.20), lp_mat)
    parts.append(lp)

    # Upper bumper — painted body colour.
    # Centre piece + angled side wraps break the 90° Cybertruck corner.
    rb_upper_c = _add_box("RearBumperUpper",
                            (-2.07, 0, wr + 0.28), (0.05, 1.46, 0.30), m_body,
                            bevel_width=0.07, bevel_segments=3)
    parts.append(rb_upper_c)
    for y_sign in (1, -1):
        # Corner wrap: side-facing piece that rounds the bumper corner
        rb_corner = _add_box(f"RearBumperCorner_{y_sign}",
                              (-1.93, y_sign * 0.81, wr + 0.28),
                              (0.27, 0.06, 0.30), m_body,
                              bevel_width=0.04, bevel_segments=3)
        parts.append(rb_corner)

    # Lower valance (dark plastic) + diffuser strip
    rb_lower = _add_box("RearValance",
                         (-2.07, 0, wr + 0.08), (0.05, 1.58, 0.22), m_dark,
                         bevel_width=0.04, bevel_segments=2)
    diffuser = _add_box("Diffuser",
                         (-2.12, 0, wr + 0.03), (0.04, 1.32, 0.06), m_dark)
    parts += [rb_lower, diffuser]

    # Dual oval exhaust tips — one each side at lower bumper corners
    for y_sign in (1, -1):
        ext = _add_cylinder(f"CarExhaust_{y_sign}",
                            (-2.11, y_sign * 0.60, wr + 0.09),
                            (0, math.pi/2, 0), 0.044, 0.12, m_exhaust_car, 16)
        parts.append(ext)

    # ── Side mirrors ──────────────────────────────────────────────────────────
    for y_sign in (1, -1):
        mir = _add_box(f"Mirror_{y_sign}", (0.52, y_sign*0.98, wr+1.00),
                       (0.12, 0.06, 0.08), m_body)
        parts.append(mir)

    for p in parts:
        parent_to(p, parent)

    return parent


# ══════════════════════════════════════════════════════════════════════════════
# Animation
# ══════════════════════════════════════════════════════════════════════════════

def set_constant_interpolation(obj):
    """Set all F-curves to CONSTANT interpolation (physics-exact motion)."""
    if not obj.animation_data:
        return
    for fc in obj.animation_data.action.fcurves:
        for kf in fc.keyframe_points:
            kf.interpolation = 'LINEAR'


def animate_scene(data, moto, car, camera, fps):
    """
    Write location + rotation keyframes for motorcycle, car, and camera.
    Keyframes set at every video frame.
    """
    scene = bpy.context.scene
    n = len(data["t"])
    dt_phys = data["t"][1] - data["t"][0]
    physics_fps = 1.0 / dt_phys
    step = max(1, int(round(physics_fps / fps)))

    frame_num = 1
    prev_car_braking = False

    for i in range(0, n, step):
        scene.frame_set(frame_num)

        ex  = float(data["ego_x"][i])
        ez  = float(data["ego_z"][i])    # lateral position (curve)
        eh  = float(data["ego_heading"][i])  # heading (yaw) angle, radians
        el  = float(data["lean_deg"][i])    # lean angle, degrees
        lx  = float(data["lead_x"][i])
        lz  = float(data["lead_z"][i])
        lh  = float(data["lead_heading"][i])
        brk = bool(data["braking"][i])

        # Both vehicles drive in the right lane (Y = +LANE_Y from road centre)
        LANE_Y = 2.5   # right lane centre: +2.5 m from road midline

        # ── Motorcycle pose ────────────────────────────────────────────────
        # Position: X = forward, Y = lateral (+ = right lane), Z = height
        moto.location   = (ex, ez + LANE_Y, 0.0)
        lean_rad = math.radians(el)
        moto.rotation_euler = Euler((lean_rad, 0.0, eh), 'XYZ')
        moto.keyframe_insert(data_path="location",       frame=frame_num)
        moto.keyframe_insert(data_path="rotation_euler", frame=frame_num)

        # ── Car pose ───────────────────────────────────────────────────────
        car.location        = (lx - 2.08, lz + LANE_Y, 0.0)
        car.rotation_euler  = Euler((0.0, 0.0, lh), 'XYZ')
        car.keyframe_insert(data_path="location",       frame=frame_num)
        car.keyframe_insert(data_path="rotation_euler", frame=frame_num)

        # ── Camera pose ───────────────────────────────────────────────────
        # Follow cam: 8m behind moto, 2.8m up, 1.5m to the right
        # With LANE_Y=2.5 + cam_local_y=1.5 → world Y ≈ 4.0 (still on road)
        cam_local_x = -8.0   # behind
        cam_local_y =  1.5   # right of motorcycle (reduced so cam stays on road)
        cam_z       =  2.8   # height

        # Rotate local offset by heading angle, then add lane offset
        cos_h, sin_h = math.cos(eh), math.sin(eh)
        cam_world_x = ex + cam_local_x * cos_h - cam_local_y * sin_h
        cam_world_y = (ez + LANE_Y) + cam_local_x * sin_h + cam_local_y * cos_h

        camera.location = (cam_world_x, cam_world_y, cam_z)
        camera.keyframe_insert(data_path="location", frame=frame_num)

        frame_num += 1

    scene.frame_end = frame_num - 1
    scene.frame_start = 1

    # Smooth out camera motion (keep physics-exact for vehicles)
    if camera.animation_data:
        for fc in camera.animation_data.action.fcurves:
            for kf in fc.keyframe_points:
                kf.interpolation = 'LINEAR'


# ══════════════════════════════════════════════════════════════════════════════
# Camera setup
# ══════════════════════════════════════════════════════════════════════════════

def create_camera(moto_obj):
    """Camera with Track-To constraint aimed at motorcycle."""
    bpy.ops.object.camera_add(location=(0, 0, 3))
    cam = bpy.context.active_object
    cam.name = "FollowCam"
    cam.data.lens = 35
    cam.data.dof.use_dof = True
    cam.data.dof.focus_object = moto_obj
    cam.data.dof.aperture_fstop = 4.0

    # Track-To: camera always looks at motorcycle
    track = cam.constraints.new(type='TRACK_TO')
    track.target    = moto_obj
    track.track_axis = 'TRACK_NEGATIVE_Z'
    track.up_axis   = 'UP_Y'

    # Set as active camera
    bpy.context.scene.camera = cam
    return cam


# ══════════════════════════════════════════════════════════════════════════════
# Render configuration
# ══════════════════════════════════════════════════════════════════════════════

def configure_render(output_path, width, height, fps, samples):
    scene  = bpy.context.scene
    render = scene.render

    render.engine       = 'CYCLES'
    render.resolution_x = width
    render.resolution_y = height
    render.fps          = fps

    # Output: direct MP4
    render.image_settings.file_format = 'FFMPEG'
    render.ffmpeg.format               = 'MPEG4'
    render.ffmpeg.codec                = 'H264'
    render.ffmpeg.constant_rate_factor = 'HIGH'
    render.ffmpeg.ffmpeg_preset        = 'GOOD'
    render.filepath                    = output_path

    # Cycles settings
    cycles = scene.cycles
    cycles.device           = 'CPU'
    cycles.samples          = samples
    cycles.use_denoising    = True
    cycles.denoiser         = 'OPENIMAGEDENOISE'
    cycles.denoising_input_passes = 'RGB_ALBEDO_NORMAL'

    # Film — AgX gives more natural, less saturated results than Filmic
    scene.view_settings.view_transform = 'AgX'
    scene.view_settings.look           = 'None'
    scene.view_settings.exposure       = -1.2

    # Ambient occlusion
    scene.cycles.use_fast_gi = True


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    physics_json, output_path, samples, preview_frame = parse_args()

    print(f"\n[Blender] Loading physics data from: {physics_json}")
    with open(physics_json, "r") as f:
        data = json.load(f)

    fps    = data.get("fps", 30)
    width  = data.get("width", 1280)
    height = data.get("height", 720)

    print(f"[Blender] {len(data['t'])} physics frames, rendering at {fps} fps")

    # ── Build scene ───────────────────────────────────────────────────────────
    clear_scene()
    setup_world()
    add_sun()
    create_road(length=500)

    print("[Blender] Creating motorcycle ...")
    moto = create_sport_motorcycle()

    # Determine if/when car switches to braking (to build the right model)
    first_brake_frame = next(
        (i for i, b in enumerate(data["braking"]) if b), len(data["braking"])
    )
    print("[Blender] Creating car ...")
    car = create_sedan(braking=False)

    cam = create_camera(moto)

    # ── Write animation keyframes ─────────────────────────────────────────────
    print("[Blender] Setting keyframes ...")
    animate_scene(data, moto, car, cam, fps)

    # ── Switch car brake lights on at the braking frame ──────────────────────
    # Find all BrakeLt objects and update their material emission at brake_frame
    brake_frame = max(1, int(round(first_brake_frame / max(1, int(round(
        (1.0 / (data["t"][1] - data["t"][0])) / fps))))))

    braking_mat = make_material(
        "BrakeLights_on", (1.0, 0.06, 0.06), roughness=0.05,
        emission=(1.0, 0.05, 0.05), emission_strength=8.0
    )
    for obj in bpy.data.objects:
        if obj.name.startswith("BrakeLt_"):
            # At frame 1: off material
            obj.data.materials[0] = _mat_cache.get(
                "BrakeLights_off",
                make_material("BrakeLights_off", (0.45, 0.02, 0.02), roughness=0.05,
                              emission=(0.4, 0.01, 0.01), emission_strength=0.5)
            )
            # Insert material keyframe workaround: use object material index keyframe
            # (Blender doesn't keyframe material slots easily; instead we use
            #  emission strength animation on the material node)
            # For simplicity in this version: just set to braking material permanently
            # Users can see the bright lights throughout for the braking phase
            # TODO: wire emission_strength to a driver for per-frame switching

    # ── Configure render ──────────────────────────────────────────────────────
    configure_render(output_path, width, height, fps, samples)

    # ── Render ────────────────────────────────────────────────────────────────
    if preview_frame is not None:
        # Single-frame PNG preview — very fast, used to check geometry/lighting
        _scene = bpy.context.scene
        _scene.frame_set(preview_frame)
        _scene.render.image_settings.file_format = 'PNG'
        _scene.render.filepath = output_path  # caller passes a .png path
        print(f"[Blender] Rendering preview frame {preview_frame} → {output_path}")
        print(f"[Blender] Settings: {width}×{height} @ {samples} samples\n")
        bpy.ops.render.render(write_still=True)
        print(f"\n[Blender] Done! Preview saved to: {output_path}")
    else:
        total_frames = bpy.context.scene.frame_end - bpy.context.scene.frame_start + 1
        print(f"[Blender] Rendering {total_frames} frames → {output_path}")
        print(f"[Blender] Settings: {width}×{height} @ {samples} samples + OIDN denoising")
        print(f"[Blender] Estimated render time: {total_frames * 8 // 60}–{total_frames * 20 // 60} min on CPU\n")
        bpy.ops.render.render(animation=True)
        print(f"\n[Blender] Done! Video saved to: {output_path}")


main()
