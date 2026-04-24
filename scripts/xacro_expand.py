#!/usr/bin/env python3
"""Minimal xacro expander.

Handles enough of the xacro spec to flatten simple URDF xacro files that
define a robot from properties, macros, and conditionals — no ROS install
required. Feature coverage:

* xacro:property name=X value=Y
* ${expr} substitution in attributes and text (basic python-like math,
  reads known properties and macro params as names)
* xacro:arg name=X default=Y — treated as its default
* $(arg X), $(optenv X Y), $(find X) — kept as literals (unresolved
  include paths are skipped)
* xacro:include filename=…  — inlined if the file exists locally,
  silently skipped otherwise (external ROS packages, etc.)
* xacro:macro name=X params=…  — expanded on xacro:X invocation, with
  parameter substitution (`*name` block params inlined verbatim)
* xacro:if/xacro:unless — truthy checks against the value
* Nested xacro expansion until a fixed point

Not supported: xacro:element-attribute, xacro:insert_block inside macros,
full recursive macro re-invocation from within macros (handled in a
single pass).

Usage:
    python3 xacro_expand.py INPUT.urdf.xacro > OUT.urdf
"""
import re
import sys
import copy
import math
from pathlib import Path
from xml.etree import ElementTree as ET

XACRO_NS = 'http://ros.org/wiki/xacro'
# Xacro has used both `wiki.ros.org/xacro` and `ros.org/wiki/xacro` over
# the years — accept either so we don't mis-skip elements depending on
# which spelling the PROTO author picked.
XACRO_NS_VARIANTS = (
    'http://ros.org/wiki/xacro',
    'http://wiki.ros.org/xacro',
    'http://www.ros.org/wiki/xacro',
)
ET.register_namespace('xacro', XACRO_NS)

NS = f'{{{XACRO_NS}}}'


def _strip_ns(tag):
    """Return (namespace, local_name) for an ElementTree tag string."""
    if tag.startswith('{'):
        i = tag.find('}')
        return tag[1:i], tag[i + 1:]
    return '', tag


def is_xacro_ns(tag):
    """True if the tag is in any of the xacro namespaces."""
    ns, _ = _strip_ns(tag)
    return ns in XACRO_NS_VARIANTS


def is_xacro(elem, tag):
    ns, local = _strip_ns(elem.tag)
    return ns in XACRO_NS_VARIANTS and local == tag


def eval_expr(expr, scope):
    """Evaluate a ${...} expression. `scope` is a dict of name -> value
    (strings or numbers). We try to interpret the expression as Python
    math after substituting scope names."""
    s = expr.strip()
    # Substitute names (longest first to avoid prefix clashes).
    names = sorted(scope.keys(), key=len, reverse=True)
    for n in names:
        s = re.sub(rf'\b{re.escape(n)}\b', str(scope[n]), s)
    # $(arg X) / $(optenv X ...) / $(find X) fall through to the scope
    # lookup above when `X` is known; otherwise keep as-is so a downstream
    # user can see the missing value. Strip the $(optenv ...) wrapper.
    s = re.sub(r'\$\(optenv\s+\w+\s+([^\)]*)\)', r'\1', s)
    s = re.sub(r'\$\(arg\s+(\w+)\)', r'\1', s)
    s = re.sub(r'\$\(find\s+[^\)]+\)', '', s)
    try:
        return eval(s, {'__builtins__': {}}, {
            'pi': math.pi, 'sin': math.sin, 'cos': math.cos,
            'sqrt': math.sqrt, 'tan': math.tan, 'abs': abs,
        })
    except Exception:
        return s


def substitute(text, scope):
    """Replace every `${...}`, `$(arg X)`, `$(optenv ENV DEFAULT)` token
    in `text` with its evaluated value. Multiple passes so nested
    references collapse."""
    if text is None:
        return text

    def repl_brace(m):
        v = eval_expr(m.group(1), scope)
        if isinstance(v, float):
            return f'{v:.6g}'
        return str(v)

    def repl_arg(m):
        name = m.group(1)
        v = scope.get(name)
        return str(v) if v is not None else ''

    def repl_optenv(m):
        # $(optenv ENVVAR default words) — no env access, use the default.
        rest = m.group(1).strip()
        parts = rest.split(None, 1)
        return parts[1] if len(parts) > 1 else ''

    for _ in range(8):
        before = text
        text = re.sub(r'\$\(optenv\s+(.+?)\)', repl_optenv, text)
        text = re.sub(r'\$\(arg\s+(\w+)\)', repl_arg, text)
        text = re.sub(r'\$\(find\s+[^\)]+\)', '', text)
        text = re.sub(r'\$\{([^{}]+)\}', repl_brace, text)
        if text == before:
            break
    return text


def substitute_attrs(elem, scope):
    for k, v in list(elem.attrib.items()):
        elem.attrib[k] = substitute(v, scope)
    if elem.text:
        elem.text = substitute(elem.text, scope)
    if elem.tail:
        elem.tail = substitute(elem.tail, scope)


def parse_args_and_props(root, scope):
    """First pass: collect xacro:property and xacro:arg defaults into
    `scope`. Returns the list of (name, macro-element) for later."""
    for e in list(root.iter()):
        if is_xacro(e, 'property'):
            name = e.get('name')
            val = e.get('value', '')
            scope[name] = eval_expr(val, scope) if '$' not in val else val
        elif is_xacro(e, 'arg'):
            name = e.get('name')
            scope.setdefault(name, e.get('default', ''))


def load_macros(root):
    macros = {}
    for e in list(root.iter()):
        if is_xacro(e, 'macro'):
            macros[e.get('name')] = e
    return macros


def strip_nodes(root, predicate):
    """Remove all elements for which `predicate(elem)` is true. Works
    recursively."""
    for parent in list(root.iter()):
        for child in list(parent):
            if predicate(child):
                parent.remove(child)
            else:
                strip_nodes(child, predicate)


def walk_remove_by_tag(root, tags):
    """Delete all children whose tag matches one of `tags`."""
    def pred(e):
        return e.tag in tags
    for parent in list(root.iter()):
        for child in list(parent):
            if pred(child):
                parent.remove(child)


def resolve_includes(root, base_dir):
    """Inline xacro:include filename=… children, following nested
    includes until the tree stops changing. Skips nonexistent files so
    external ROS packages (velodyne_description etc.) drop silently."""
    pkg_path_pattern = re.compile(r'\$\(find\s+([^\)]+)\)')

    def candidates(raw_path, base_dir):
        # Strip all $(find ...) prefixes and leading slashes.
        p = pkg_path_pattern.sub('', raw_path).lstrip('/')
        # Also strip a leading "urdf/" suffix mismatch.
        p = substitute(p, {})
        out = [base_dir / p, Path(p)]
        # A common pattern: base_dir already ends with `urdf/` but the
        # include path also starts with `urdf/...`. Strip once.
        if p.startswith('urdf/'):
            out.append(base_dir / p[len('urdf/'):])
            out.append(base_dir.parent / p)
        out.append(base_dir.parent / p)
        # The filename alone (last path component).
        out.append(base_dir / Path(p).name)
        return out

    def do_pass():
        expanded_any = False
        for parent in list(root.iter()):
            new_children = []
            local_changed = False
            for child in list(parent):
                if is_xacro(child, 'include'):
                    local_changed = True
                    expanded_any = True
                    raw_path = child.get('filename', '')
                    picked = None
                    for cand in candidates(raw_path, base_dir):
                        if cand.is_file():
                            picked = cand
                            break
                    if picked is not None:
                        sub_tree = ET.parse(picked).getroot()
                        for sc in list(sub_tree):
                            new_children.append(sc)
                else:
                    new_children.append(child)
            if local_changed:
                for c in list(parent):
                    parent.remove(c)
                for c in new_children:
                    parent.append(c)
        return expanded_any

    # Keep expanding until no more includes are found (handles nested
    # `meshes.xacro` that itself includes another file).
    for _ in range(8):
        if not do_pass():
            break


def resolve_if_unless(root, scope):
    """Expand xacro:if/unless nodes. True: keep children inline; False:
    drop the whole node."""
    for parent in list(root.iter()):
        new_children = []
        changed = False
        for child in list(parent):
            if is_xacro(child, 'if') or is_xacro(child, 'unless'):
                changed = True
                val = child.get('value', '')
                val = eval_expr(val, scope) if '$' in val or '{' in val else val
                truthy = str(val).strip().lower() in ('true', '1', 'yes')
                keep = truthy if is_xacro(child, 'if') else not truthy
                if keep:
                    for sc in list(child):
                        new_children.append(sc)
            else:
                new_children.append(child)
        if changed:
            for c in list(parent):
                parent.remove(c)
            for c in new_children:
                parent.append(c)


def expand_macros(root, macros, scope):
    """Expand xacro:<macro> invocations. `params="a b *block"` gives
    scalar params (read from invocation attributes) and block params
    (child elements of the invocation, matched by order)."""

    def substitute_tree(elem, local_scope):
        """Recursively substitute ${...} in attrs/text for this element
        and all descendants. Block-param references (xacro:insert_block)
        expand in place."""
        substitute_attrs(elem, local_scope)
        new = []
        for child in list(elem):
            if is_xacro(child, 'insert_block'):
                name = child.get('name')
                blk = local_scope.get(f'_block_{name}')
                if blk is not None:
                    # Insert the block element itself (e.g. <origin .../>
                    # or <joint .../>) AND any children it carries.
                    clone = copy.deepcopy(blk)
                    substitute_tree(clone, local_scope)
                    new.append(clone)
                continue
            substitute_tree(child, local_scope)
            new.append(child)
        for c in list(elem):
            elem.remove(c)
        for c in new:
            elem.append(c)

    def expand_one(invocation, macro_name):
        macro_elem = macros[macro_name]
        param_spec = macro_elem.get('params', '').split()
        local_scope = dict(scope)
        block_params = []
        scalar_params = []
        for p in param_spec:
            if p.startswith('*'):
                block_params.append(p[1:])
            else:
                scalar_params.append(p)
        # Scalar params: attribute on invocation, or default. Substitute
        # the RAW value against the caller's scope first so nested
        # `${...}` references from outer macros collapse to a number
        # (e.g. `x="${base_x}"` passed down into a deeper macro).
        for p in scalar_params:
            if ':=' in p:
                name, default = p.split(':=', 1)
            elif '=' in p:
                name, default = p.split('=', 1)
            else:
                name, default = p, ''
            raw = invocation.get(name, default.strip('"\''))
            local_scope[name] = substitute(raw, scope)
        # Block params: child elements of the invocation that are
        # themselves not xacro:arg or xacro:insert_block.
        block_children = [c for c in list(invocation)
                          if not c.tag.startswith(NS)]
        for name, child in zip(block_params, block_children):
            local_scope[f'_block_{name}'] = child
        # Clone macro body and substitute.
        body = [copy.deepcopy(c) for c in list(macro_elem)]
        for b in body:
            substitute_tree(b, local_scope)
        return body

    def pass_once(tree_root):
        changed = False
        for parent in list(tree_root.iter()):
            new_children = []
            dirty = False
            for child in list(parent):
                if is_xacro_ns(child.tag):
                    _, local = _strip_ns(child.tag)
                    if local in macros:
                        new_children.extend(expand_one(child, local))
                        dirty = True
                        changed = True
                        continue
                new_children.append(child)
            if dirty:
                for c in list(parent):
                    parent.remove(c)
                for c in new_children:
                    parent.append(c)
        return changed

    for _ in range(8):
        if not pass_once(root):
            break


def strip_xacro_declarations(root):
    """After expansion, remove every remaining xacro:* element (both
    the housekeeping declarations and any unresolved macro invocations
    for which we never loaded a definition, e.g. includes from
    external packages)."""
    for parent in list(root.iter()):
        for child in list(parent):
            if is_xacro_ns(child.tag):
                parent.remove(child)


def strip_gazebo_only(root):
    """Drop Gazebo-specific elements that urdf-rs rejects: `<gazebo>`,
    `<transmission>` with unresolved content, `<plugin>`."""
    for parent in list(root.iter()):
        for child in list(parent):
            if child.tag in ('gazebo', 'plugin'):
                parent.remove(child)


def substitute_all(root, scope):
    for e in root.iter():
        substitute_attrs(e, scope)


def strip_xacro_attr_namespace(root):
    """Remove xmlns:xacro from the root attrs for tidy output."""
    root.attrib.pop('xmlns:xacro', None)


def main():
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr)
        sys.exit(2)
    src = Path(sys.argv[1])
    tree = ET.parse(src)
    root = tree.getroot()
    scope = {}
    resolve_includes(root, src.parent)
    parse_args_and_props(root, scope)
    macros = load_macros(root)
    resolve_if_unless(root, scope)
    expand_macros(root, macros, scope)
    substitute_all(root, scope)
    strip_xacro_declarations(root)
    strip_gazebo_only(root)
    # Final substitution once more — includes may have introduced props.
    parse_args_and_props(root, scope)
    substitute_all(root, scope)
    # Emit cleaned XML. ElementTree writes namespace on the root; strip
    # the xmlns:xacro attribute so the output is a plain URDF.
    xml = ET.tostring(root, encoding='unicode')
    xml = re.sub(r'\sxmlns:xacro="[^"]*"', '', xml)
    xml = re.sub(r'<xacro:[^/>]*/>', '', xml)
    xml = re.sub(r'<xacro:[^>]*>.*?</xacro:[^>]*>', '', xml, flags=re.DOTALL)
    print('<?xml version="1.0"?>')
    print(xml)


if __name__ == '__main__':
    main()
