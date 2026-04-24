#!/usr/bin/env python3
"""Flatten a COLLADA (.dae) scene graph by baking every per-node
transform into the vertex arrays it references.

Why this exists: our Bevy pipeline uses `mesh_loader` which reads the
raw `<float_array>` positions out of each `<geometry>` but does NOT
apply the per-node `<matrix>` transforms in `<visual_scene>`. DAEs
exported by SolidWorks / Rhino / Fusion use instance-graphs heavily —
a single `<geometry>` can be placed many times with different scene
transforms, and individual body panels are in their own local frames.
Without the transforms the parts all pile up around the local origin
of each geometry, which is why meshes like `ranger_base.dae` appeared
as a cluster of bolts rather than a chassis.

This script walks `<visual_scene>`, accumulates matrices node→leaf,
and for every `<instance_geometry>` it applies the accumulated matrix
to the target geometry's positions + normals. When the same geometry
is instanced multiple times (via different transforms) we duplicate
the `<geometry>` definition once per instance and rewrite the URLs so
each instance points at its own baked copy. Afterwards we set every
node matrix to identity so the file is idempotent.

Usage:
    python3 flatten_dae.py mesh.dae                 # in place
    python3 flatten_dae.py path/to/meshes/          # recursive
"""

import re
import sys
import uuid
import copy
from pathlib import Path
from xml.etree import ElementTree as ET


COLLADA_NS = "http://www.collada.org/2005/11/COLLADASchema"
NS = {"c": COLLADA_NS}
ET.register_namespace("", COLLADA_NS)

IDENTITY4 = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


def _local(tag):
    return tag.split("}", 1)[-1] if "{" in tag else tag


def _find(parent, local_name):
    for c in parent:
        if _local(c.tag) == local_name:
            return c
    return None


def _findall(parent, local_name):
    return [c for c in parent if _local(c.tag) == local_name]


# ═══════════════════════════════════════════════════════════════════
# Matrix math
# ═══════════════════════════════════════════════════════════════════
def parse_matrix(text):
    """COLLADA stores 4×4 matrices row-major as 16 space-separated floats."""
    vals = [float(x) for x in text.split()]
    if len(vals) != 16:
        return None
    return [vals[0:4], vals[4:8], vals[8:12], vals[12:16]]


def mat_mul(A, B):
    return [
        [sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
        for i in range(4)
    ]


def mat_mul_point(M, v):
    """Transform a 3-point (homogeneous w=1)."""
    x, y, z = v
    return (
        M[0][0] * x + M[0][1] * y + M[0][2] * z + M[0][3],
        M[1][0] * x + M[1][1] * y + M[1][2] * z + M[1][3],
        M[2][0] * x + M[2][1] * y + M[2][2] * z + M[2][3],
    )


def mat_mul_direction(M, v):
    """Transform a 3-direction (ignore translation)."""
    x, y, z = v
    return (
        M[0][0] * x + M[0][1] * y + M[0][2] * z,
        M[1][0] * x + M[1][1] * y + M[1][2] * z,
        M[2][0] * x + M[2][1] * y + M[2][2] * z,
    )


def is_identity(M, eps=1e-9):
    for i in range(4):
        for j in range(4):
            want = 1.0 if i == j else 0.0
            if abs(M[i][j] - want) > eps:
                return False
    return True


def from_translate(t):
    x, y, z = [float(v) for v in t.split()]
    M = [row[:] for row in IDENTITY4]
    M[0][3] = x
    M[1][3] = y
    M[2][3] = z
    return M


def from_scale(s):
    sx, sy, sz = [float(v) for v in s.split()]
    M = [row[:] for row in IDENTITY4]
    M[0][0] = sx
    M[1][1] = sy
    M[2][2] = sz
    return M


def from_rotate(r):
    # axis-angle "x y z angle-degrees"
    import math
    vals = [float(v) for v in r.split()]
    if len(vals) != 4:
        return [row[:] for row in IDENTITY4]
    ax, ay, az, deg = vals
    ang = math.radians(deg)
    length = (ax * ax + ay * ay + az * az) ** 0.5
    if length < 1e-9:
        return [row[:] for row in IDENTITY4]
    ax, ay, az = ax / length, ay / length, az / length
    c, s = math.cos(ang), math.sin(ang)
    C = 1.0 - c
    return [
        [ax * ax * C + c, ax * ay * C - az * s, ax * az * C + ay * s, 0.0],
        [ay * ax * C + az * s, ay * ay * C + c, ay * az * C - ax * s, 0.0],
        [az * ax * C - ay * s, az * ay * C + ax * s, az * az * C + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def node_transform(node):
    """Combine all <matrix>/<translate>/<rotate>/<scale> children of a
    single <node> into one 4×4. COLLADA applies them in document
    order, so we left-multiply in order: M = T1 * T2 * T3 …"""
    M = [row[:] for row in IDENTITY4]
    for child in node:
        tag = _local(child.tag)
        if tag == "matrix":
            m = parse_matrix(child.text or "")
            if m is not None:
                M = mat_mul(M, m)
        elif tag == "translate":
            M = mat_mul(M, from_translate(child.text or "0 0 0"))
        elif tag == "scale":
            M = mat_mul(M, from_scale(child.text or "1 1 1"))
        elif tag == "rotate":
            M = mat_mul(M, from_rotate(child.text or "0 0 1 0"))
    return M


def iter_nodes(scene, transform=None):
    """Depth-first yield (node_elem, accumulated_world_matrix). Every
    level's own transform is combined into the path so leaves see the
    fully composed world transform."""
    if transform is None:
        transform = [row[:] for row in IDENTITY4]
    for n in _findall(scene, "node"):
        local = node_transform(n)
        world = mat_mul(transform, local)
        yield n, world
        yield from iter_nodes(n, world)


# ═══════════════════════════════════════════════════════════════════
# Library walking
# ═══════════════════════════════════════════════════════════════════
def geometry_index(root):
    """Map id → <geometry> element."""
    out = {}
    for lib in root:
        if _local(lib.tag) == "library_geometries":
            for g in lib:
                if _local(g.tag) == "geometry":
                    gid = g.get("id")
                    if gid:
                        out[gid] = (lib, g)
    return out


def apply_matrix_to_positions(geometry, M):
    """Walk every `<source>` whose id contains 'position' and mutate
    its `<float_array>` text in place, applied as a point transform."""
    changed = 0
    for mesh in geometry.iter():
        if _local(mesh.tag) != "source":
            continue
        sid = mesh.get("id", "")
        if "position" in sid.lower():
            kind = "point"
        elif "normal" in sid.lower():
            kind = "direction"
        else:
            continue
        fa = None
        for c in mesh:
            if _local(c.tag) == "float_array":
                fa = c
                break
        if fa is None or not fa.text:
            continue
        vals = [float(x) for x in fa.text.split()]
        out = []
        for i in range(0, len(vals) - 2, 3):
            v = (vals[i], vals[i + 1], vals[i + 2])
            if kind == "point":
                nv = mat_mul_point(M, v)
            else:
                # Direction (normal). Rotation only; we just ignore scale-
                # skewing here because robot DAEs use uniform scale or
                # rigid transforms — good enough for visual meshes.
                nv = mat_mul_direction(M, v)
            out.append(f"{nv[0]:.6f}")
            out.append(f"{nv[1]:.6f}")
            out.append(f"{nv[2]:.6f}")
        fa.text = "\n" + " ".join(out) + "\n"
        changed += 1
    return changed


def clone_geometry(src_elem, new_id):
    """Deep-copy a <geometry> element and rewrite its id + source ids
    so each instance gets its own uniquely-addressed geometry."""
    clone = copy.deepcopy(src_elem)
    clone.set("id", new_id)
    # Rewrite internal `id`s that live inside this geometry. The
    # primitive `<triangles>` / `<polylist>` reference their sources
    # by `#<id>`, so we rename them together via a suffix.
    suffix = "-" + new_id[-8:]
    remap = {}
    for elem in clone.iter():
        sid = elem.get("id")
        if sid:
            new_sid = sid + suffix
            remap[sid] = new_sid
            elem.set("id", new_sid)
    # Rewrite every "#old" reference to "#new" within this clone.
    for elem in clone.iter():
        for attr in ("source", "offset", "url", "target"):
            v = elem.get(attr)
            if v and v.startswith("#"):
                ref = v[1:]
                if ref in remap:
                    elem.set(attr, "#" + remap[ref])
    return clone


# ═══════════════════════════════════════════════════════════════════
# Flatten
# ═══════════════════════════════════════════════════════════════════
def flatten(path: Path):
    try:
        tree = ET.parse(path)
    except ET.ParseError as e:
        return False, f"parse-error: {e}"
    root = tree.getroot()
    geom_map = geometry_index(root)
    if not geom_map:
        return False, "no-geometries"

    # Find <visual_scene> root(s).
    visual_scene = None
    for lib in root:
        if _local(lib.tag) == "library_visual_scenes":
            visual_scene = _find(lib, "visual_scene")
            break
    if visual_scene is None:
        return False, "no-visual-scene"

    # First pass: walk scene, for every <instance_geometry> record
    # the (node, world_matrix, url, instance_material_copy).
    instances = []  # list of (containing_node, world_M, instance_elem, geom_id)
    for node, world in iter_nodes(visual_scene):
        for child in list(node):
            tag = _local(child.tag)
            if tag == "instance_geometry":
                url = child.get("url", "")
                if url.startswith("#"):
                    gid = url[1:]
                    if gid in geom_map:
                        instances.append((node, world, child, gid))

    if not instances:
        return False, "no-instances"

    # Track which geometry ids have been used. First instance takes
    # the original geometry; subsequent instances get cloned copies.
    baked_count = 0
    used_first = set()
    for node, world, instance_elem, gid in instances:
        lib, geom = geom_map[gid]
        if is_identity(world):
            used_first.add(gid)
            continue
        if gid not in used_first:
            # First instance — bake directly into the existing geom.
            apply_matrix_to_positions(geom, world)
            used_first.add(gid)
        else:
            # Second+ instance — clone geom, bake into clone, repoint.
            new_id = f"{gid}-baked-{uuid.uuid4().hex[:8]}"
            clone = clone_geometry(geom, new_id)
            apply_matrix_to_positions(clone, world)
            lib.append(clone)
            instance_elem.set("url", "#" + new_id)
        baked_count += 1

    # Second pass: wipe all node transforms so re-running this doesn't
    # double-apply.
    for node, _ in iter_nodes(visual_scene):
        for child in list(node):
            tag = _local(child.tag)
            if tag == "matrix":
                child.text = " ".join(f"{IDENTITY4[i][j]}" for i in range(4) for j in range(4))
            elif tag == "translate":
                child.text = "0 0 0"
            elif tag == "scale":
                child.text = "1 1 1"
            elif tag == "rotate":
                child.text = "0 0 1 0"

    tree.write(path, xml_declaration=True, encoding="utf-8")
    return True, f"baked {baked_count}/{len(instances)} instances"


def iter_dae(arg: Path):
    if arg.is_dir():
        yield from arg.rglob("*.dae")
    elif arg.suffix == ".dae":
        yield arg


def main():
    total = baked = 0
    for a in sys.argv[1:]:
        for dae in iter_dae(Path(a)):
            total += 1
            ok, info = flatten(dae)
            marker = "baked" if ok else "skip"
            print(f"{dae}  {marker}  {info}")
            if ok:
                baked += 1
    print(f"\n{baked}/{total} flattened")


if __name__ == "__main__":
    main()
