#!/usr/bin/env python3
"""Webots PROTO → URDF converter.

Handles (enough for Robotti + simpler Webots protos):
  * PROTO definitions with typed field list + default values.
  * `IS fieldname` bindings resolved against caller's args.
  * `%< ... >%` JS template blocks: let/const, if/else, ternaries,
    string equality, field-value references, Math.floor.
  * `%<= EXPR >%` inline expressions.
  * Sub-PROTO calls expanded from side-by-side .proto files.
  * Pose/Solid/Transform accumulated transforms.
  * HingeJoint → revolute joints with anchor/axis/limits.
  * Shape+geometry (Box/Cylinder/Sphere) with PBRAppearance.baseColor.
  * CadShape (DAE mesh reference).
  * Axis-angle rotations → RPY in URDF.

Hand-rolled. Not bulletproof. If something explodes, edit the proto
to simplify it first (strip physics, hidden sensors, etc.).

    python3 scripts/proto2urdf.py path/to/Robot.proto OUTDIR NAME
"""
import math
import re
import sys
import json
import os
import urllib.request
from pathlib import Path

# Local mirror of cyberbotics/webots PROTOs referenced via `webots://` or
# EXTERNPROTO. Populated lazily — we only fetch files that a converted
# robot actually needs.
WEBOTS_CACHE = Path(os.environ.get('WEBOTS_CACHE', Path.home() / '.cache' / 'webots_protos'))
WEBOTS_RAW = 'https://raw.githubusercontent.com/cyberbotics/webots/master/'


_FETCH_MISS = set()  # URLs we've already failed to fetch this run


def fetch_webots_path(rel_path):
    """Download `projects/...` file from cyberbotics/webots into the
    local cache. Returns local Path on success, None if fetch fails."""
    rel = rel_path.lstrip('/').removeprefix('webots://')
    local = WEBOTS_CACHE / rel
    if local.exists():
        return local
    if rel in _FETCH_MISS:
        return None
    url = WEBOTS_RAW + rel
    try:
        local.parent.mkdir(parents=True, exist_ok=True)
        req = urllib.request.Request(url, headers={'User-Agent': 'proto2urdf'})
        with urllib.request.urlopen(req, timeout=10) as r:
            local.write_bytes(r.read())
        return local
    except Exception:
        _FETCH_MISS.add(rel)
        return None


# ══════════════════════════════════════════════════════════════════
# 1. JS mini-evaluator
# ══════════════════════════════════════════════════════════════════
class JSEnv:
    """Evaluates the tiny subset of JS that Webots PROTO templates use."""
    def __init__(self, fields):
        # fields: dict[name] -> {"value": any}
        self.fields = fields
        self.locals = {}

    def eval(self, expr):
        expr = expr.strip().rstrip(';').strip()
        # Ternary: A ? B : C (simplest handling: split on top-level ?)
        tern = self._split_top(expr, '?')
        if len(tern) == 2:
            cond = tern[0]
            br = self._split_top(tern[1], ':')
            if len(br) == 2:
                return self.eval(br[0]) if self.eval(cond) else self.eval(br[1])
        # || and && (short-circuit)
        for op, fn in (('||', any), ('&&', all)):
            parts = self._split_top(expr, op)
            if len(parts) > 1:
                vals = [self._truthy(self.eval(p)) for p in parts]
                if op == '||':
                    return next((v for v in vals if v), False)
                return all(vals)
        # ==, !=
        for op in ('==', '!='):
            parts = self._split_top(expr, op)
            if len(parts) == 2:
                a = self.eval(parts[0]); b = self.eval(parts[1])
                return a == b if op == '==' else a != b
        # < > <= >=
        for op in ('<=', '>=', '<', '>'):
            parts = self._split_top(expr, op)
            if len(parts) == 2:
                a = float(self.eval(parts[0])); b = float(self.eval(parts[1]))
                return {'<=': a <= b, '>=': a >= b, '<': a < b, '>': a > b}[op]
        # Arithmetic — but only when the operator is genuinely infix
        # (both sides non-empty). `- x * 0.5` must NOT be parsed as
        # `'' - x` = 0; it's unary minus applied to `x * 0.5`.
        for op in ('+', '-', '*', '/'):
            parts = self._split_top(expr, op)
            if len(parts) >= 2 and all(p.strip() for p in parts):
                # Left-associative fold: ((a op b) op c) op d …
                try:
                    acc = self.eval(parts[0])
                    for p in parts[1:]:
                        b = self.eval(p)
                        acc = {'+': lambda x, y: x + y,
                               '-': lambda x, y: x - y,
                               '*': lambda x, y: x * y,
                               '/': lambda x, y: x / y}[op](acc, b)
                    return acc
                except (TypeError, ZeroDivisionError):
                    return 0
        # Unary minus
        if expr.startswith('-'):
            return -self.eval(expr[1:])
        # String literal
        if len(expr) >= 2 and expr[0] in ('"', "'") and expr[-1] == expr[0]:
            return expr[1:-1]
        # Number
        try:
            if '.' in expr or 'e' in expr:
                return float(expr)
            return int(expr)
        except ValueError:
            pass
        # true/false
        if expr == 'true': return True
        if expr == 'false': return False
        # Math.floor / Math.ceil / Math.round
        m = re.match(r'Math\.(floor|ceil|round|abs|sqrt|sin|cos|tan)\((.*)\)$', expr)
        if m:
            f = getattr(math, m.group(1), None) or {'floor': math.floor, 'ceil': math.ceil, 'round': round}.get(m.group(1))
            inner = self.eval(m.group(2))
            try:
                return f(float(inner))
            except (TypeError, ValueError):
                return 0
        # fields.NAME.value (optionally .x/.y/.z)
        m = re.match(r'fields\.(\w+)\.value(?:\.(\w+))?$', expr)
        if m:
            name, attr = m.group(1), m.group(2)
            if name not in self.fields:
                return None
            val = self.fields[name]['value']
            if attr is None:
                return val
            if isinstance(val, (list, tuple)):
                idx = {'x': 0, 'y': 1, 'z': 2, 'w': 3}.get(attr, 0)
                return val[idx] if idx < len(val) else 0
            if attr == 'defaultValue':
                # Fallback used by Webots sanity-check blocks when the
                # runtime value is invalid. We don't track default-vs-
                # provided separately; just reuse the stored value.
                return val
            return val
        # Local var
        if expr in self.locals:
            return self.locals[expr]
        # Parenthesized expr
        if expr.startswith('(') and expr.endswith(')'):
            return self.eval(expr[1:-1])
        # Fallback: emit as-is
        return expr

    def exec_block(self, block):
        """Execute a JS statement block.

        We don't actually parse JS — we scan for top-level `let NAME =
        EXPR;` / `const NAME = EXPR;` / `var NAME = EXPR;` patterns and
        evaluate each EXPR with our mini-evaluator. Control structures
        (if/else), sanity-check reassignments, console.error calls, and
        other side-effects are ignored. Webots PROTOs use these blocks
        for argument-validation wrappers — missing the side effects is
        fine because we trust the caller-supplied values.
        """
        # Strip `// line comments` and `/* block comments */`.
        block = re.sub(r'//[^\n]*', '', block)
        block = re.sub(r'/\*.*?\*/', '', block, flags=re.DOTALL)
        # Find every `let / const / var NAME = ...;` at any indent. The
        # value extends until the next top-level `;`.
        i = 0
        while i < len(block):
            m = re.compile(r'(?:let|const|var)\s+(\w+)\s*=\s*',
                           re.MULTILINE).search(block, i)
            if not m:
                break
            name = m.group(1)
            j = m.end()
            # Scan forward to the next `;` that's outside brackets/quotes.
            depth = 0
            quote = None
            k = j
            while k < len(block):
                c = block[k]
                if quote:
                    if c == quote and block[k - 1] != '\\':
                        quote = None
                elif c in ('"', "'"):
                    quote = c
                elif c in '([{':
                    depth += 1
                elif c in ')]}':
                    depth -= 1
                elif c == ';' and depth == 0:
                    break
                k += 1
            expr = block[j:k].strip()
            try:
                self.locals[name] = self.eval(expr)
            except Exception:
                pass
            i = k + 1

    def _truthy(self, v):
        if v is None or v is False or v == 0 or v == '':
            return False
        return True

    def _split_top(self, s, op):
        """Split on an operator that's not inside any bracket / quote."""
        depth = 0
        quote = None
        parts = []
        last = 0
        i = 0
        while i < len(s):
            c = s[i]
            if quote:
                if c == quote and s[i - 1] != '\\':
                    quote = None
            elif c in ('"', "'"):
                quote = c
            elif c in '([{':
                depth += 1
            elif c in ')]}':
                depth -= 1
            elif depth == 0 and s.startswith(op, i):
                parts.append(s[last:i])
                i += len(op)
                last = i
                continue
            i += 1
        parts.append(s[last:])
        return parts

    def _split_stmts(self, block):
        return [x for x in block.split(';') if x.strip()]


# ══════════════════════════════════════════════════════════════════
# 2. JS template preprocessor: %< ... >% + %<= ... >%
# ══════════════════════════════════════════════════════════════════
def render_template(text, env):
    """Turn the PROTO template into flat VRML by resolving JS blocks.

    Processes `%< ... >%` statement blocks and `%<= EXPR >%` inline
    expressions in a single forward pass so that `let` / `const`
    declarations inside a preceding block are visible when the next
    inline expression evaluates.
    """
    out = []
    i = 0
    stack = []  # include?-flags for nested if/else
    pattern = re.compile(r'%<(=?)(.*?)>%', re.DOTALL)
    while i < len(text):
        m = pattern.search(text, i)
        if not m:
            if all(stack):
                out.append(text[i:])
            break
        if all(stack):
            out.append(text[i:m.start()])
        i = m.end()
        is_expr = m.group(1) == '='
        inner = m.group(2).strip()
        if is_expr:
            if all(stack):
                v = env.eval(inner)
                if isinstance(v, float):
                    out.append(f'{v:.6g}')
                else:
                    out.append(str(v))
            continue
        if not inner:
            continue
        # if (cond) {
        mm = re.match(r'if\s*\((.+)\)\s*\{$', inner)
        if mm:
            cond = env._truthy(env.eval(mm.group(1)))
            stack.append(cond and all(stack))
            continue
        # } else if (cond) {
        mm = re.match(r'\}\s*else\s+if\s*\((.+)\)\s*\{$', inner)
        if mm:
            if stack:
                prev = stack.pop()
                cond = (not prev) and env._truthy(env.eval(mm.group(1)))
                stack.append(cond and all(stack))
            continue
        # } else {
        if re.match(r'\}\s*else\s*\{$', inner):
            if stack:
                stack.append(not stack.pop() and all(stack))
            continue
        # }
        if inner == '}':
            if stack:
                stack.pop()
            continue
        # Statement block — only run if we are in an included branch.
        if all(stack):
            env.exec_block(inner)
    return ''.join(out)


# ══════════════════════════════════════════════════════════════════
# 3. PROTO loader: extract fields + body
# ══════════════════════════════════════════════════════════════════
PROTO_HEADER_RE = re.compile(
    r'PROTO\s+(\w+)\s*\[\s*(.*?)\s*\]\s*\{',
    re.DOTALL,
)
FIELD_LINE_RE = re.compile(
    r'(?:field|hiddenField)\s+(\w+(?:\{[^}]*\})?)\s+(\w+)\s+(.*?)\s*(?=field\s|\]|#|$)',
    re.DOTALL,
)

def parse_field_default(field_type, default_str):
    """Parse a default value according to its type."""
    s = default_str.strip()
    # Quoted string
    if s.startswith('"'):
        m = re.match(r'"([^"]*)"', s)
        if m: return m.group(1)
    # TRUE/FALSE
    if s == 'TRUE': return True
    if s == 'FALSE': return False
    # Numeric sequence?
    nums = re.findall(r'-?[0-9]+\.?[0-9]*(?:e[-+]?[0-9]+)?', s)
    if nums and 'Vec' in field_type or 'Rotation' in field_type or field_type.startswith('SFColor'):
        return [float(x) for x in nums]
    if nums and 'SFFloat' in field_type:
        return float(nums[0])
    if nums and 'SFInt' in field_type:
        return int(nums[0])
    # [] or list
    if s.startswith('['):
        return []
    return s

EXTERNPROTO_RE = re.compile(r'^\s*(?:IMPORTABLE\s+)?EXTERNPROTO\s+"([^"]+)"', re.MULTILINE)


def scan_externprotos(text):
    """Return the list of EXTERNPROTO paths declared at the top of a
    PROTO file. Paths can be:
        "webots://projects/..."            (Webots repo)
        "https://raw.githubusercontent.com/..." (also Webots repo)
        "Relative.proto"                   (sibling file)
    """
    return EXTERNPROTO_RE.findall(text)


def load_proto(path):
    text = Path(path).read_text()
    # Strip comments first
    body_text = re.sub(r'#[^\n]*', '', text)
    m = PROTO_HEADER_RE.search(body_text)
    if not m:
        raise ValueError(f'No PROTO header in {path}')
    name = m.group(1)
    field_block = m.group(2)
    # Parse the fields
    fields = {}
    # Each field on its own line roughly: `field TYPE name default_value`
    for line in field_block.split('\n'):
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        mm = re.match(r'(?:field|hiddenField|unconnectedField)\s+([A-Za-z][\w]*(?:\{[^}]*\})?)\s+(\w+)\s+(.*)$', line)
        if mm:
            ftype, fname, fdefault = mm.group(1), mm.group(2), mm.group(3)
            fields[fname] = {'type': ftype, 'value': parse_field_default(ftype, fdefault)}
    # Body: from the `{` after the `]` to the matching `}` at the end.
    # Simplest: take everything after the first `{` following the `]`.
    body_start = m.end()
    # Find the matching `}` by counting (ignoring those inside strings)
    depth = 1
    i = body_start
    in_str = False
    while i < len(body_text) and depth > 0:
        c = body_text[i]
        if in_str:
            if c == '"' and body_text[i - 1] != '\\':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
        i += 1
    body = body_text[body_start:i - 1]
    externs = scan_externprotos(text)
    return {'name': name, 'fields': fields, 'body': body,
            'path': Path(path), 'externs': externs}


def _deep_copy_node(val):
    """Deep-copy a parsed VRML node/value so DEF/USE references don't
    share mutable state. Only (node_type, field_dict) tuples get
    descended into — plain ('IS', 'fieldname') pointer-tuples are
    treated as opaque scalars."""
    if (isinstance(val, tuple) and len(val) == 2
            and isinstance(val[0], str) and isinstance(val[1], dict)):
        return (val[0], {k: _deep_copy_node(v) for k, v in val[1].items()})
    if isinstance(val, list):
        return [_deep_copy_node(x) for x in val]
    if isinstance(val, dict):
        return {k: _deep_copy_node(v) for k, v in val.items()}
    return val


# ══════════════════════════════════════════════════════════════════
# 4. VRML tokenizer / parser — runs on *rendered* template text.
# ══════════════════════════════════════════════════════════════════
VRML_TOKEN_RE = re.compile(
    r'"([^"]*)"|(\{|\}|\[|\])|(-?[0-9]+\.?[0-9]*(?:e[-+]?[0-9]+)?)|(\w+(?:\.\w+)*)'
)

def tokenize_vrml(text):
    out = []
    for m in VRML_TOKEN_RE.finditer(text):
        if m.group(1) is not None:
            out.append(('STR', m.group(1)))
        elif m.group(2):
            out.append((m.group(2), m.group(2)))
        elif m.group(3):
            out.append(('NUM', float(m.group(3))))
        elif m.group(4):
            out.append(('NAME', m.group(4)))
    return out

class VRMLParser:
    def __init__(self, tokens):
        self.tokens = tokens
        self.i = 0
        self.defs = {}  # DEF-name → deep-copied node tuple

    def peek(self, n=0):
        if self.i + n < len(self.tokens):
            return self.tokens[self.i + n]
        return (None, None)

    def eat(self):
        t = self.tokens[self.i]
        self.i += 1
        return t

    def parse_node(self):
        """Assume self.peek() is NAME (node type). Returns (type, fields_dict) or None."""
        if self.peek()[0] != 'NAME':
            return None
        type_name = self.eat()[1]
        if type_name == 'USE':
            # USE Identifier — look up the previously-DEFined node and
            # return a deep copy so later mutations don't ripple.
            if self.peek()[0] == 'NAME':
                ref = self.eat()[1]
                node = self.defs.get(ref)
                if node is not None:
                    return _deep_copy_node(node)
            return ('USE', {})
        if type_name == 'DEF':
            # DEF name NodeType { ... } — remember the node by name so
            # subsequent USEs can resolve to its contents.
            def_name = self.eat()[1] if self.peek()[0] == 'NAME' else None
            node = self.parse_node()
            if def_name and node is not None:
                self.defs[def_name] = node
            return node
        fields = {}
        if self.peek()[0] != '{':
            return (type_name, fields)
        self.eat()  # consume {
        while self.i < len(self.tokens) and self.peek()[0] != '}':
            tt, tv = self.peek()
            if tt != 'NAME':
                self.eat()
                continue
            field_name = self.eat()[1]
            val = self.parse_value()
            fields[field_name] = val
        if self.peek()[0] == '}':
            self.eat()
        return (type_name, fields)

    def parse_value(self):
        tt, tv = self.peek()
        if tt == 'NUM':
            nums = [self.eat()[1]]
            while self.peek()[0] == 'NUM':
                nums.append(self.eat()[1])
            return nums if len(nums) > 1 else nums[0]
        if tt == 'STR':
            return self.eat()[1]
        if tt == '[':
            self.eat()
            items = []
            while self.i < len(self.tokens) and self.peek()[0] != ']':
                ptt = self.peek()[0]
                if ptt == 'NAME':
                    nd = self.parse_node()
                    if nd: items.append(nd)
                elif ptt == 'STR':
                    items.append(self.eat()[1])
                elif ptt == 'NUM':
                    items.append(self.eat()[1])
                else:
                    self.eat()
            if self.peek()[0] == ']':
                self.eat()
            return items
        if tt == 'NAME':
            if tv == 'TRUE':
                self.eat(); return True
            if tv == 'FALSE':
                self.eat(); return False
            if tv == 'NULL':
                self.eat(); return None
            if tv == 'IS':
                self.eat()  # IS
                if self.peek()[0] == 'NAME':
                    return ('IS', self.eat()[1])
                return None
            # Nested node
            return self.parse_node()
        self.eat()
        return None


# ══════════════════════════════════════════════════════════════════
# 5. Transform math
# ══════════════════════════════════════════════════════════════════
def aa_to_quat(axis_angle):
    if not isinstance(axis_angle, list) or len(axis_angle) < 4:
        return (0, 0, 0, 1)
    ax, ay, az, ang = axis_angle[:4]
    l = math.sqrt(ax * ax + ay * ay + az * az)
    if l < 1e-9:
        return (0, 0, 0, 1)
    s = math.sin(ang / 2) / l
    return (ax * s, ay * s, az * s, math.cos(ang / 2))

def quat_mul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )

def quat_rot_vec(q, v):
    x, y, z, w = q
    vq = (v[0], v[1], v[2], 0.0)
    qinv = (-x, -y, -z, w)
    r = quat_mul(quat_mul(q, vq), qinv)
    return (r[0], r[1], r[2])

def quat_to_rpy(q):
    x, y, z, w = q
    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sp = 2 * (w * y - z * x)
    p = math.copysign(math.pi / 2, sp) if abs(sp) >= 1 else math.asin(sp)
    yv = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return (r, p, yv)


# ══════════════════════════════════════════════════════════════════
# 6. Walker
# ══════════════════════════════════════════════════════════════════
class Converter:
    def __init__(self, proto_dirs, robot_name, mesh_prefix):
        self.proto_dirs = [Path(d) for d in proto_dirs]
        self.name = robot_name
        self.mesh_prefix = mesh_prefix  # e.g. "package://robotti/meshes/"
        self.protos = {}  # name → loaded proto
        self.links = []   # list of (name, xml)
        self.joints = []  # list of (name, xml)
        self.link_counter = 0
        self.joint_counter = 0
        self.meshes_used = set()

    def find_proto(self, name):
        if name in self.protos:
            return self.protos[name]
        # 1. Search explicit proto_dirs.
        for d in self.proto_dirs:
            p = d / f'{name}.proto'
            if p.exists():
                return self._register_proto(p)
        # 2. Search recorded EXTERNPROTO declarations from already-loaded
        #    PROTOs. Both webots:// URIs and relative paths.
        for parent in list(self.protos.values()):
            for ext in parent.get('externs', []):
                if Path(ext).stem != name:
                    continue
                local = self._resolve_extern(ext, parent['path'])
                if local:
                    return self._register_proto(local)
        # 3. Fallback: try the Webots repo under common locations.
        for guess in (
            f'projects/vehicles/protos/abstract/{name}.proto',
            f'projects/vehicles/protos/generic/{name}.proto',
            f'projects/appearances/protos/{name}.proto',
            f'projects/objects/factory/{name}/protos/{name}.proto',
            f'projects/robots/{name}/{name}.proto',
        ):
            local = fetch_webots_path(guess)
            if local:
                return self._register_proto(local)
        return None

    def _register_proto(self, path):
        proto = load_proto(path)
        self.protos[proto['name']] = proto
        return proto

    def _resolve_extern(self, extern_ref, referrer_path):
        """Resolve an EXTERNPROTO path declared in a PROTO file."""
        if extern_ref.startswith('webots://'):
            return fetch_webots_path(extern_ref.removeprefix('webots://'))
        if extern_ref.startswith('https://raw.githubusercontent.com/cyberbotics/webots/'):
            # Strip leading URL and branch.
            m = re.match(r'https://raw\.githubusercontent\.com/cyberbotics/webots/[^/]+/(.+)', extern_ref)
            if m:
                return fetch_webots_path(m.group(1))
            return None
        if extern_ref.startswith('http://') or extern_ref.startswith('https://'):
            return None
        # Relative path — resolve next to the referring PROTO.
        p = referrer_path.parent / extern_ref
        return p if p.exists() else None

    def resolve_is(self, val, args):
        """Replace ('IS', 'fieldname') with args[fieldname] recursively."""
        if isinstance(val, tuple) and len(val) == 2 and val[0] == 'IS':
            name = val[1]
            return args.get(name, {'value': None})['value']
        if isinstance(val, list):
            return [self.resolve_is(x, args) for x in val]
        if isinstance(val, tuple) and len(val) == 2 and isinstance(val[0], str) and not val[0] == 'IS':
            # (node_type, fields_dict)
            return (val[0], {k: self.resolve_is(v, args) for k, v in val[1].items()})
        if isinstance(val, dict):
            return {k: self.resolve_is(v, args) for k, v in val.items()}
        return val

    def instantiate(self, proto_name, call_args_fields):
        """proto_name: PROTO type. call_args_fields: dict[name]->{value} with
        caller-supplied fields. Fill in defaults for missing. Render
        JS blocks. Parse resulting VRML. Return root node."""
        proto = self.find_proto(proto_name)
        if not proto:
            return None
        merged = {}
        for name, info in proto['fields'].items():
            merged[name] = {'value': info['value']}
        for name, info in call_args_fields.items():
            merged[name] = info
        env = JSEnv(merged)
        rendered = render_template(proto['body'], env)
        tokens = tokenize_vrml(rendered)
        parser = VRMLParser(tokens)
        # First node in body is the one we want
        while parser.i < len(tokens):
            if parser.peek()[0] == 'NAME':
                node = parser.parse_node()
                if node:
                    # Resolve any IS references using the call args +
                    # defaults (already merged).
                    return self.resolve_is(node, merged)
            else:
                parser.eat()
        return None

    def fmt_mesh_url(self, url):
        if isinstance(url, list):
            url = url[0] if url else ''
        # Strip any directory prefix; we stage all meshes flat.
        return Path(url).name

    def emit_link(self, name=None, visuals=None):
        lname = name or f'link_{self.link_counter}'
        self.link_counter += 1
        lines = [f'  <link name="{lname}">']
        for v in visuals or []:
            lines.append(v)
        lines.append('  </link>')
        self.links.append((lname, '\n'.join(lines)))
        return lname

    def emit_fixed_joint(self, parent, child, xyz, rpy):
        jn = f'joint_{self.joint_counter}'
        self.joint_counter += 1
        xml = (
            f'  <joint name="{jn}" type="fixed">\n'
            f'    <origin xyz="{xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f}" rpy="{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}"/>\n'
            f'    <parent link="{parent}"/>\n'
            f'    <child link="{child}"/>\n'
            f'  </joint>'
        )
        self.joints.append((jn, xml))

    def emit_hinge_joint(self, parent, child, xyz, rpy, axis):
        jn = f'joint_{self.joint_counter}'
        self.joint_counter += 1
        ax = axis or [0, 0, 1]
        xml = (
            f'  <joint name="{jn}" type="continuous">\n'
            f'    <origin xyz="{xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f}" rpy="{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}"/>\n'
            f'    <parent link="{parent}"/>\n'
            f'    <child link="{child}"/>\n'
            f'    <axis xyz="{ax[0]:.4f} {ax[1]:.4f} {ax[2]:.4f}"/>\n'
            f'  </joint>'
        )
        self.joints.append((jn, xml))

    def color_from_appearance(self, app):
        """Extract baseColor (PBR) / diffuseColor (classic) from an
        appearance node. Also looks inside PROTO wrappers like
        `MattePaint { baseColor ... }` by falling through to the
        embedded PBR fields when a common field name matches."""
        if not isinstance(app, tuple):
            return None
        atype, afields = app
        if atype == 'PBRAppearance':
            bc = afields.get('baseColor')
            if isinstance(bc, list) and len(bc) >= 3:
                return bc[:3]
        elif atype == 'Appearance':
            mat = afields.get('material')
            if isinstance(mat, tuple):
                mtype, mfields = mat
                dc = mfields.get('diffuseColor')
                if isinstance(dc, list) and len(dc) >= 3:
                    return dc[:3]
        # PROTO-wrapped appearance (MattePaint, BrushedAluminium, …).
        # Most of them expose a `baseColor` field at the top level.
        bc = afields.get('baseColor')
        if isinstance(bc, list) and len(bc) >= 3:
            return bc[:3]
        return None

    def _emit_indexed_face_set(self, gfields, scale):
        """Turn an IndexedFaceSet node into an on-disk .obj file and
        return its filename. VRML IFS: `coord Coordinate { point [...] }`
        plus `coordIndex [v0 v1 v2 -1 v3 v4 v5 v6 -1 …]`."""
        coord = gfields.get('coord')
        if not isinstance(coord, tuple) or coord[0] != 'Coordinate':
            return None
        pts = coord[1].get('point')
        if not isinstance(pts, list) or not pts:
            return None
        idx = gfields.get('coordIndex')
        if not isinstance(idx, list) or not idx:
            return None
        # Flatten `point` into (x,y,z) tuples.
        if isinstance(pts[0], (int, float)):
            verts = [(pts[i] * scale[0], pts[i + 1] * scale[1],
                      pts[i + 2] * scale[2])
                     for i in range(0, len(pts) - 2, 3)]
        else:
            verts = [(p[0] * scale[0], p[1] * scale[1], p[2] * scale[2])
                     for p in pts if isinstance(p, (list, tuple)) and len(p) >= 3]
        # Split coordIndex on -1 sentinels, triangulate each face (fan).
        faces = []
        cur = []
        for v in idx:
            iv = int(v)
            if iv < 0:
                if len(cur) >= 3:
                    for k in range(1, len(cur) - 1):
                        faces.append((cur[0], cur[k], cur[k + 1]))
                cur = []
            else:
                cur.append(iv)
        if len(cur) >= 3:
            for k in range(1, len(cur) - 1):
                faces.append((cur[0], cur[k], cur[k + 1]))
        if not faces:
            return None
        self._ifs_counter = getattr(self, '_ifs_counter', 0) + 1
        obj_name = f'ifs_{self._ifs_counter}.obj'
        lines = [f'v {x:.6f} {y:.6f} {z:.6f}' for (x, y, z) in verts]
        lines += [f'f {a+1} {b+1} {c+1}' for (a, b, c) in faces]
        self._pending_objs = getattr(self, '_pending_objs', {})
        self._pending_objs[obj_name] = '\n'.join(lines) + '\n'
        return obj_name

    def _visual_mesh_xml(self, filename, xyz, rpy, color):
        mesh_attr = f'filename="{self.mesh_prefix}{filename}"'
        origin = f'<origin xyz="{xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f}" rpy="{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}"/>'
        mat_xml = ''
        if color:
            mat_xml = f'<material name=""><color rgba="{color[0]} {color[1]} {color[2]} 1.0"/></material>'
        return f'    <visual>\n      {origin}\n      <geometry><mesh {mesh_attr}/></geometry>\n      {mat_xml}\n    </visual>'

    def visual_from_shape(self, shape_fields, xyz, parent_q, scale=(1, 1, 1)):
        """Turn a Shape { geometry ... appearance ... } into a <visual> XML fragment.
        `scale` is the accumulated Transform.scale so primitives shrink
        with their wrapping Transforms (wheels, etc.)."""
        geom = shape_fields.get('geometry')
        if not isinstance(geom, tuple):
            return None
        gtype, gfields = geom
        color = self.color_from_appearance(shape_fields.get('appearance'))
        q = parent_q
        inner = None
        # Webots Cylinder / Capsule height runs along local +Y; URDF
        # primitives run along local +Z. Add an Rx(-π/2) to convert.
        # (Some Webots PROTOs put an explicit Rx(π/2) on the wrapping
        # Solid to "stand the cylinder up" — with our fix included, the
        # net rotation matches Webots' rendering.)
        if gtype == 'Cylinder':
            r = gfields.get('radius', 0.05); h = gfields.get('height', 0.1)
            r *= (scale[0] + scale[2]) / 2
            h *= scale[1] if abs(scale[1]) > 1e-6 else 1.0
            q = quat_mul(parent_q, (math.sin(-math.pi / 4), 0.0, 0.0,
                                    math.cos(-math.pi / 4)))
            inner = f'<cylinder radius="{r}" length="{h}"/>'
        elif gtype == 'Capsule':
            r = gfields.get('radius', 0.05); h = gfields.get('height', 0.1)
            r *= (scale[0] + scale[2]) / 2
            h *= scale[1] if abs(scale[1]) > 1e-6 else 1.0
            q = quat_mul(parent_q, (math.sin(-math.pi / 4), 0.0, 0.0,
                                    math.cos(-math.pi / 4)))
            inner = f'<capsule radius="{r}" length="{h}"/>'
        elif gtype == 'Box':
            sz = gfields.get('size', [0.1, 0.1, 0.1])
            inner = f'<box size="{sz[0] * scale[0]} {sz[1] * scale[1]} {sz[2] * scale[2]}"/>'
        elif gtype == 'Sphere':
            r = gfields.get('radius', 0.05)
            r *= (scale[0] + scale[1] + scale[2]) / 3
            inner = f'<sphere radius="{r}"/>'
        elif gtype == 'IndexedFaceSet':
            # Emit as an external OBJ — no native polygon-mesh type in
            # URDF. The mesh gets written to the output `meshes/` dir
            # and referenced like a CadShape.
            obj_name = self._emit_indexed_face_set(gfields, scale)
            if obj_name:
                self.meshes_used.add(obj_name)
                return self._visual_mesh_xml(obj_name, xyz,
                                             quat_to_rpy(q), color)
            return None
        if not inner:
            return None
        r_, p_, y_ = quat_to_rpy(q)
        origin = f'<origin xyz="{xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f}" rpy="{r_:.4f} {p_:.4f} {y_:.4f}"/>'
        mat_xml = ''
        if color:
            mat_xml = f'<material name=""><color rgba="{color[0]} {color[1]} {color[2]} 1.0"/></material>'
        return f'    <visual>\n      {origin}\n      <geometry>{inner}</geometry>\n      {mat_xml}\n    </visual>'

    def visual_from_cadshape(self, url, xyz, rpy, scale=(1, 1, 1)):
        mesh = self.fmt_mesh_url(url)
        if not mesh:
            return None
        self.meshes_used.add(mesh)
        origin = f'<origin xyz="{xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f}" rpy="{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}"/>'
        mesh_attr = f'filename="{self.mesh_prefix}{mesh}"'
        if scale != (1, 1, 1):
            mesh_attr += f' scale="{scale[0]} {scale[1]} {scale[2]}"'
        return f'    <visual>\n      {origin}\n      <geometry><mesh {mesh_attr}/></geometry>\n    </visual>'

    def walk(self, node, parent_link, t, q, depth=0, scale=(1.0, 1.0, 1.0)):
        """Walk a (type, fields) node. `t`/`q` are the caller's cumulative
        world transform. Emits visual links at world poses + fixed joints
        under `parent_link`. HingeJoints are treated as pass-throughs:
        we descend into their endPoint as if it were a plain child, so
        the physics-hinge becomes a static-pose render.

        (A previous version tried to emit real URDF revolute joints and
        made the transforms double-count: the HingeJoint's `anchor` was
        added in addition to the endPoint's own `translation`, which in
        Webots are typically the same point.)"""
        if depth > 40 or not isinstance(node, tuple):
            return
        ntype, nfields = node
        lt = nfields.get('translation', [0, 0, 0])
        if not isinstance(lt, list): lt = [0, 0, 0]
        lr = nfields.get('rotation', [0, 0, 1, 0])
        if not isinstance(lr, list): lr = [0, 0, 1, 0]
        lq = aa_to_quat(lr)
        rot_lt = quat_rot_vec(q, lt[:3])
        new_t = (t[0] + rot_lt[0], t[1] + rot_lt[1], t[2] + rot_lt[2])
        new_q = quat_mul(q, lq)
        # Multiply scale (Transform nodes carry one — Pose/Solid don't).
        ls = nfields.get('scale', [1, 1, 1])
        if not isinstance(ls, list): ls = [1, 1, 1]
        new_scale = (scale[0] * ls[0], scale[1] * ls[1], scale[2] * ls[2])

        if ntype in ('HingeJoint', 'Hinge2Joint', 'SliderJoint',
                     'BallJoint', 'Joint', 'Slot'):
            ep = nfields.get('endPoint')
            if isinstance(ep, tuple):
                self.walk(ep, parent_link, t, q, depth + 1, scale)
            elif isinstance(ep, list):
                for e in ep:
                    if isinstance(e, tuple):
                        self.walk(e, parent_link, t, q, depth + 1, scale)
            return

        if ntype == 'CadShape':
            url = nfields.get('url')
            r, p, y = quat_to_rpy(new_q)
            xml = self.visual_from_cadshape(url, new_t, (r, p, y), scale)
            if xml:
                self.emit_link(visuals=[xml])
                child_name = self.links[-1][0]
                self.emit_fixed_joint(parent_link, child_name, (0, 0, 0), (0, 0, 0))
            return

        if ntype == 'Shape':
            xml = self.visual_from_shape(nfields, new_t, new_q, scale)
            if xml:
                self.emit_link(visuals=[xml])
                child_name = self.links[-1][0]
                self.emit_fixed_joint(parent_link, child_name, (0, 0, 0), (0, 0, 0))
            return

        # Sub-PROTO: expand and walk. Caller's transform already included
        # via `new_t`/`new_q`; inside the expansion, the top-level Solid
        # carries the sub-proto's own translation IS binding, which will
        # apply again on top. But the outer node here has already locked
        # its `translation` value into the expansion's own fields (via
        # IS), so we must NOT pre-accumulate into new_t/new_q — pass
        # parent's t/q untouched.
        if ntype not in ('Pose', 'Solid', 'Transform', 'Robot', 'Group'):
            sub = self.find_proto(ntype)
            if sub:
                # Pass ALL fields through (including MFNode like children /
                # extensionSlot / wheelFrontRight / ...). The PROTO body's
                # `IS fieldName` will pick them up.
                call_args = {k: {'value': v} for k, v in nfields.items()}
                expanded = self.instantiate(ntype, call_args)
                if expanded:
                    self.walk(expanded, parent_link, t, q, depth + 1, scale)
                return

        # Pose / Solid / Transform / Robot / Group / AckermannVehicle:
        # walk `children` and `extensionSlot` as plain child lists.
        for field_name in ('children', 'extensionSlot'):
            val = nfields.get(field_name)
            if isinstance(val, list):
                for c in val:
                    if isinstance(c, tuple):
                        self.walk(c, parent_link, new_t, new_q, depth + 1, new_scale)

    def walk_solid(self, solid_node, parent_link, t, q, hinge_axis=None, depth=0):
        """Walk a Solid that's the endPoint of a HingeJoint — emit a link with its
        accumulated geometry under a hinge joint."""
        if not isinstance(solid_node, tuple):
            return
        _, fields = solid_node
        lt = fields.get('translation', [0, 0, 0])
        lr = fields.get('rotation', [0, 0, 1, 0])
        if not isinstance(lt, list): lt = [0, 0, 0]
        if not isinstance(lr, list): lr = [0, 0, 1, 0]
        lq = aa_to_quat(lr)
        rot_lt = quat_rot_vec(q, lt[:3])
        child_t = (t[0] + rot_lt[0], t[1] + rot_lt[1], t[2] + rot_lt[2])
        child_q = quat_mul(q, lq)

        # Emit this link + a hinge joint from parent.
        link_name = self.emit_link(visuals=[])
        r, p, y = quat_to_rpy(child_q)
        # Joint origin = solid translation/rotation in parent frame
        r0, p0, y0 = quat_to_rpy(lq)
        self.emit_hinge_joint(parent_link, link_name, tuple(lt[:3]), (r0, p0, y0), hinge_axis)
        # Walk this solid's children to attach geometry under link_name.
        children = fields.get('children')
        if isinstance(children, list):
            for c in children:
                if isinstance(c, tuple):
                    self.walk(c, link_name, child_t, child_q, depth + 1)

    def emit_urdf(self):
        # Root base link
        base_xml = '  <link name="base_link"/>'
        out = [f'<?xml version="1.0"?>\n<!-- Auto-converted from Webots PROTO -->\n<robot name="{self.name}">']
        out.append(base_xml)
        for _, xml in self.links:
            out.append(xml)
        for _, xml in self.joints:
            out.append(xml)
        out.append('</robot>\n')
        return '\n'.join(out)


# ══════════════════════════════════════════════════════════════════
# 7. Main
# ══════════════════════════════════════════════════════════════════
def detect_coord_system(proto_path):
    """Return 'NUE' (Y-up, older Webots) or 'ENU' (Z-up, default since
    R2023a). Based on the PROTO's `#VRML_SIM <version>` header — there
    is no explicit coordinateSystem in PROTOs, so we infer from the
    release and the default-translation Y-vs-Z convention."""
    head = Path(proto_path).read_text(errors='ignore').splitlines()[:5]
    for line in head:
        m = re.search(r'#VRML_SIM\s+(R?)(\d+)([a-zA-Z]?)', line)
        if m:
            prefix, year, suffix = m.group(1), int(m.group(2)), m.group(3)
            # Webots 2020/2021/2022 defaulted to NUE. R2023a+ is ENU.
            if year >= 2023:
                return 'ENU'
            return 'NUE'
    return 'ENU'


def main():
    if len(sys.argv) < 4:
        print(__doc__, file=sys.stderr)
        sys.exit(2)
    proto_path = Path(sys.argv[1])
    outdir = Path(sys.argv[2])
    name = sys.argv[3]
    coord_override = sys.argv[4] if len(sys.argv) > 4 else None
    proto_dir = proto_path.parent
    conv = Converter([proto_dir], name, f'package://{name}/meshes/')
    root = conv.instantiate(proto_path.stem, {})
    if not root:
        print(f'failed to load {proto_path}', file=sys.stderr)
        sys.exit(1)
    coord = coord_override or detect_coord_system(proto_path)
    if coord == 'NUE':
        # Y-up → Z-up. Rx(+π/2) sends the PROTO's +Y (up) to URDF +Z.
        # NOTE: only useful when the ENTIRE tree (body + wheels + sub-
        # protos) was authored against NUE. Our script pulls Webots
        # abstract PROTOs (Car / AckermannVehicle) from master, which
        # are ENU, so this wrap breaks the wheel positions for NUE-era
        # PROTOs unless we also fetch the matching NUE versions. Skip
        # it for now — use the R2023a+ Webots PROTO where available.
        print(f'[coord] detected NUE (Y-up) — NOT wrapping (see proto2urdf.py); '
              f'prefer R2023a+ PROTOs for conversion', file=sys.stderr)
    q0 = (0.0, 0.0, 0.0, 1.0)
    conv.walk(root, 'base_link', (0, 0, 0), q0)
    urdf_text = conv.emit_urdf()
    outdir.mkdir(parents=True, exist_ok=True)
    (outdir / 'urdf').mkdir(parents=True, exist_ok=True)
    (outdir / 'meshes').mkdir(parents=True, exist_ok=True)
    (outdir / 'urdf' / f'{name}.urdf').write_text(urdf_text)
    # Copy referenced meshes from the source PROTO directory.
    for mesh in conv.meshes_used:
        src = proto_dir / mesh
        if src.exists():
            (outdir / 'meshes' / mesh).write_bytes(src.read_bytes())
    # Flush any IndexedFaceSet → OBJ meshes we generated inline.
    for obj_name, content in getattr(conv, '_pending_objs', {}).items():
        (outdir / 'meshes' / obj_name).write_text(content)
    print(f'wrote {outdir / "urdf" / f"{name}.urdf"} ({len(conv.links)} links, {len(conv.joints)} joints, {len(conv.meshes_used)} meshes)')

if __name__ == '__main__':
    main()
