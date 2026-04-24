#!/usr/bin/env python3
"""Bake DAE scene-node transforms into the vertex data.

Collada files commonly include a top-level `<matrix>` inside a
`<visual_scene>/<node>` that contains both the scale needed to convert
the mesh's authoring units (often mm) to meters, AND a translation to
place the mesh at a sensible origin. The `mesh_loader` crate used by
our Bevy loader reads the raw `<float_array>` of positions but does
NOT honour that scene matrix — so SolidWorks-exported DAE files render
at the wrong scale / in the wrong place.

This script walks every `*.dae` under the given paths, multiplies every
position float_array through the scene matrix, replaces the matrix with
the 4×4 identity, and writes the file back. Normals are rotated by the
matrix's upper-left 3×3 (inverse-transpose of the diagonal isn't needed
here because we assume uniform scale inside a single node; if the scale
differs per axis we recompute normals from positions).

Usage:
    python3 bake_dae_matrix.py path/to/file.dae [more.dae...]
    python3 bake_dae_matrix.py path/to/meshes_dir    # recursive
"""
import re
import sys
from pathlib import Path
from xml.etree import ElementTree as ET


IDENTITY_MATRIX = '1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1'


def parse_matrix(text):
    """Parse a 16-value row-major column-major string into a 4x4 list."""
    vals = [float(x) for x in text.split()]
    if len(vals) != 16:
        return None
    return [vals[0:4], vals[4:8], vals[8:12], vals[12:16]]


def mat_mul_vec3(M, v):
    """4x4 matrix * 3-vec (treating v as homogeneous point)."""
    x, y, z = v
    return (
        M[0][0] * x + M[0][1] * y + M[0][2] * z + M[0][3],
        M[1][0] * x + M[1][1] * y + M[1][2] * z + M[1][3],
        M[2][0] * x + M[2][1] * y + M[2][2] * z + M[2][3],
    )


def mat_mul_normal(M, n):
    """Apply rotation/scale (no translation) to a normal."""
    x, y, z = n
    return (
        M[0][0] * x + M[0][1] * y + M[0][2] * z,
        M[1][0] * x + M[1][1] * y + M[1][2] * z,
        M[2][0] * x + M[2][1] * y + M[2][2] * z,
    )


def is_identity(M, eps=1e-6):
    target = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    for i in range(4):
        for j in range(4):
            if abs(M[i][j] - target[i][j]) > eps:
                return False
    return True


SOURCE_POS_RE = re.compile(
    r'(<source[^>]*id="[^"]*position[^"]*"[^>]*>.*?<float_array[^>]*>)'
    r'([^<]+)'
    r'(</float_array>)',
    re.DOTALL | re.IGNORECASE,
)
SOURCE_NORMAL_RE = re.compile(
    r'(<source[^>]*id="[^"]*normal[^"]*"[^>]*>.*?<float_array[^>]*>)'
    r'([^<]+)'
    r'(</float_array>)',
    re.DOTALL | re.IGNORECASE,
)
# Match a <node>…</node> block whose body contains `<instance_geometry>`
# or `<instance_node>` (i.e. the node that places our mesh, not a
# camera or light).
MESH_NODE_RE = re.compile(
    r'<node\b[^>]*>(.*?)</node>',
    re.DOTALL,
)
ANY_MATRIX_RE = re.compile(r'<matrix([^>]*)>([^<]+)</matrix>', re.DOTALL)
UNIT_RE = re.compile(r'<unit([^>]*)\s*meter="([^"]+)"([^/]*)/>', re.DOTALL)


def _local(tag):
    return tag.split('}', 1)[-1] if tag.startswith('{') else tag


def _find_mesh_node_matrix(node):
    """Depth-first walk: find the first <node> that (directly or via a
    descendant chain of <node>s) contains an <instance_geometry> or
    <instance_node>, and return its accumulated world matrix plus the
    raw matrix text of the nearest enclosing <matrix> element so the
    caller can null it out in the file.

    Accumulates the chain of matrices along the way — each nested node
    can carry its own transform and the world transform is their
    product top-down. When we hit a target node, we also find any
    per-node matrix along that chain so we can replace it with identity
    in the file (since we've baked it into the vertices)."""
    # Stack entries: (elem, accumulated_M_so_far, list_of_matrix_raw_texts_along_path)
    stack = [(node, IDENTITY_MATRIX_LIST, [])]
    while stack:
        current, parent_M, parent_matrices = stack.pop()
        local_M = parent_M
        local_matrices = list(parent_matrices)
        is_camera = False
        direct_instance = False
        for child in list(current):
            t = _local(child.tag)
            if t == 'matrix':
                M = parse_matrix(child.text or '')
                if M is not None:
                    local_M = mat_mul_4x4(local_M, M)
                    local_matrices.append((child.text or '').strip())
            elif t in ('instance_camera', 'instance_light'):
                is_camera = True
            elif t in ('instance_geometry', 'instance_node'):
                direct_instance = True
        if is_camera:
            continue
        if direct_instance:
            return local_matrices, local_M
        for child in list(current):
            if _local(child.tag) == 'node':
                stack.append((child, local_M, local_matrices))
    return None, None


IDENTITY_MATRIX_LIST = [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]


def mat_mul_4x4(A, B):
    return [
        [sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
        for i in range(4)
    ]


def find_mesh_matrix(data: str):
    """Parse `data` as COLLADA XML and locate the world-space transform
    for the first mesh node. Returns (list_of_raw_matrix_texts, 4x4)."""
    try:
        root = ET.fromstring(data)
    except ET.ParseError:
        return None, None
    visual_scene = None
    for n in root.iter():
        if _local(n.tag) == 'visual_scene':
            visual_scene = n
            break
    if visual_scene is None:
        return None, None
    return _find_mesh_node_matrix(visual_scene)


def bake(path: Path):
    data = path.read_text()
    # Combine scene-node matrix + asset unit scale into a single
    # transform. The COLLADA <unit meter="0.001"/> means "1 stored
    # vertex unit = 0.001 meters"; mesh_loader doesn't apply it, so
    # we bake the unit into the vertex values alongside the scene
    # matrix and then rewrite unit to meter="1".
    raw_matrices, M = find_mesh_matrix(data)
    has_matrix = raw_matrices is not None and len(raw_matrices) > 0
    if raw_matrices is not None and M is None:
        return False, 'bad-matrix'

    u = UNIT_RE.search(data)
    unit_scale = float(u.group(2)) if u else 1.0
    matrix_identity = M is None or is_identity(M)
    unit_identity = abs(unit_scale - 1.0) < 1e-9
    if matrix_identity and unit_identity:
        return False, 'identity'
    if M is None:
        M = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    # Compose: final_vertex = M * v_in * unit_scale  (unit first, then
    # scene transform). We fold the unit scale into M so the same mul
    # handles both.
    if not unit_identity:
        for i in range(3):
            for j in range(3):
                M[i][j] *= unit_scale
            M[i][3] *= 1.0  # translation already in meters in most files

    def xform_array(match, xform):
        """Apply `xform` (a (vals)->vals callable) to an array of floats."""
        header, body, footer = match.group(1), match.group(2), match.group(3)
        vals = body.split()
        out = []
        for i in range(0, len(vals) - 2, 3):
            x, y, z = float(vals[i]), float(vals[i + 1]), float(vals[i + 2])
            nx, ny, nz = xform((x, y, z))
            out.append(f'{nx:.6f}')
            out.append(f'{ny:.6f}')
            out.append(f'{nz:.6f}')
        new_body = '\n' + ' '.join(out) + '\n'
        return header + new_body + footer

    # Bake positions.
    new_data = SOURCE_POS_RE.sub(
        lambda mm: xform_array(mm, lambda v: mat_mul_vec3(M, v)),
        data,
    )
    # Bake normals (rotation/scale only).
    new_data = SOURCE_NORMAL_RE.sub(
        lambda mm: xform_array(mm, lambda v: mat_mul_normal(M, v)),
        new_data,
    )
    # Replace each <matrix> we consumed with the 4×4 identity so the
    # file is idempotent (re-running won't re-apply the transforms).
    if has_matrix:
        for raw in raw_matrices:
            needle = f'>{raw}<'
            if needle in new_data:
                new_data = new_data.replace(needle, f'>{IDENTITY_MATRIX}<', 1)
    # Rewrite the unit to meter="1" so re-running doesn't double-scale.
    if not unit_identity and u is not None:
        new_data = UNIT_RE.sub(
            lambda mm: f'<unit{mm.group(1)}meter="1"{mm.group(3)}/>',
            new_data,
            count=1,
        )
    path.write_text(new_data)
    return True, 'baked'


def iter_dae(arg: Path):
    if arg.is_dir():
        yield from arg.rglob('*.dae')
    elif arg.suffix == '.dae':
        yield arg


def main():
    total = baked = 0
    for a in sys.argv[1:]:
        for dae in iter_dae(Path(a)):
            total += 1
            ok, reason = bake(dae)
            if ok:
                baked += 1
                print(f'{dae}  baked')
            else:
                print(f'{dae}  skip ({reason})')
    print(f'\n{baked}/{total} baked')


if __name__ == '__main__':
    main()
