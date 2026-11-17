#!/usr/bin/env python3
"""Import the official Galbot S1 description into this package.

Upstream: https://github.com/GalaxyGeneralRobotics/galbot_s1_description (Apache-2.0)

What it does
  * urdf/meshes/visual/*.glb  ->  meshes/visual/*.dae (+ base color textures as .jpg/.png)
    Gazebo Classic cannot read glTF, so every visual is rewritten as COLLADA with
    its materials (base color factor / base color texture) preserved.
  * urdf/meshes/collision/*.stl  ->  meshes/collision/*.stl (copied as is)
  * urdf/galbot_s1.urdf  ->  urdf/galbot_s1_body.xacro (a xacro macro)
      - mesh paths become ${mesh_uri}/visual/<name>.dae, ${mesh_uri}/collision/<name>.stl
      - a base_footprint root link is added on the floor (base_link sits on the wheels)
      - mimic tags are kept behind the `use_mimic` macro parameter
      - wheel collisions are replaced by spheres (stable contact in ODE / DART)
      - zero-mass inertial blocks are dropped (those links are fixed-joint children)
      - a few limits that only matter in simulation are overridden (see LIMIT_OVERRIDES)

Usage
  python3 tools/import_upstream.py /path/to/galbot_s1_description

Only the Python standard library and numpy are required.
"""
import argparse
import datetime
import json
import os
import shutil
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Limits that are overridden for simulation. The upstream values are kept in a
# comment of the generated xacro so it is obvious what changed.
LIMIT_OVERRIDES = {
    # The drive wheels are only animated in simulation (the base is moved as a
    # planar body), but 1.5 rad/s (= 0.12 m/s) would make the animation lag.
    'wheel_driving_joint1': {'velocity': 30.0},
    'wheel_driving_joint2': {'velocity': 30.0},
    'wheel_driving_joint3': {'velocity': 30.0},
    'wheel_driving_joint4': {'velocity': 30.0},
    'wheel_steering_joint1': {'velocity': 6.0},
    'wheel_steering_joint2': {'velocity': 6.0},
    'wheel_steering_joint3': {'velocity': 6.0},
    'wheel_steering_joint4': {'velocity': 6.0},
    # The lift carries torso + both arms + head (~90 kg). 1000 N is not enough
    # for a velocity servo in gz-sim; the MJCF model uses 4200 N.
    'torso_lift_joint1': {'effort': 5000.0},
}

WHEEL_LINKS = ['wheel_driving_link%d' % i for i in range(1, 5)]


# --------------------------------------------------------------------------- glTF
_CT = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16,
       5125: np.uint32, 5126: np.float32}
_NC = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}


def read_glb(path):
    with open(path, 'rb') as f:
        data = f.read()
    magic, _version, length = struct.unpack('<4sII', data[:12])
    if magic != b'glTF':
        raise ValueError('%s is not a GLB file' % path)
    off = 12
    chunks = []
    while off < length:
        clen, ctype = struct.unpack('<I4s', data[off:off + 8])
        off += 8
        chunks.append((ctype, data[off:off + clen]))
        off += clen
    js = json.loads(chunks[0][1])
    binary = chunks[1][1] if len(chunks) > 1 else b''
    return js, binary


def accessor(js, binary, index):
    acc = js['accessors'][index]
    view = js['bufferViews'][acc['bufferView']]
    dtype = np.dtype(_CT[acc['componentType']])
    ncomp = _NC[acc['type']]
    offset = view.get('byteOffset', 0) + acc.get('byteOffset', 0)
    stride = view.get('byteStride', 0)
    count = acc['count']
    item = dtype.itemsize * ncomp
    if stride and stride != item:
        rows = [np.frombuffer(binary, dtype=dtype, count=ncomp, offset=offset + i * stride)
                for i in range(count)]
        arr = np.array(rows)
    else:
        arr = np.frombuffer(binary, dtype=dtype, count=count * ncomp, offset=offset).reshape(count, ncomp)
    return arr if ncomp > 1 else arr.reshape(-1)


def buffer_view_bytes(js, binary, index):
    view = js['bufferViews'][index]
    off = view.get('byteOffset', 0)
    return binary[off:off + view['byteLength']]


MAX_TEXTURE_SIZE = 2048


def shrink_texture(path):
    """Upstream textures are 4096^2. 2048^2 is plenty for simulation and keeps
    GPU memory / software rendering (Docker without GPU) reasonable."""
    try:
        from PIL import Image
    except ImportError:
        print('  (Pillow not installed, keeping %s at full resolution)' % os.path.basename(path))
        return
    im = Image.open(path)
    if max(im.size) <= MAX_TEXTURE_SIZE:
        return
    scale = MAX_TEXTURE_SIZE / float(max(im.size))
    im = im.convert('RGB').resize((int(im.size[0] * scale), int(im.size[1] * scale)), Image.LANCZOS)
    if path.endswith('.png'):
        im.save(path)
    else:
        im.save(path, quality=90)


def linear_to_srgb(c):
    c = max(0.0, min(1.0, float(c)))
    return 12.92 * c if c <= 0.0031308 else 1.055 * (c ** (1.0 / 2.4)) - 0.055


def _fmt(values, digits):
    fmt = '%.' + str(digits) + 'g'
    return ' '.join(fmt % v for v in values)


def glb_to_dae(glb_path, dae_path, texture_prefix):
    """Convert one GLB file to COLLADA 1.4.1. Returns list of written texture files."""
    js, binary = read_glb(glb_path)
    out_dir = os.path.dirname(dae_path)
    written = []

    # Node transforms are not supported (upstream files have none) - fail loudly.
    for node in js.get('nodes', []):
        for key in ('matrix', 'translation', 'rotation', 'scale'):
            if key in node:
                raise ValueError('%s: node transforms (%s) are not supported' % (glb_path, key))

    # ---- textures -----------------------------------------------------------
    image_files = {}

    def texture_file(tex_index):
        tex = js['textures'][tex_index]
        img_index = tex['source']
        if img_index in image_files:
            return image_files[img_index]
        img = js['images'][img_index]
        ext = '.png' if img.get('mimeType') == 'image/png' else '.jpg'
        name = '%s_tex%d%s' % (texture_prefix, img_index, ext)
        path = os.path.join(out_dir, name)
        with open(path, 'wb') as f:
            f.write(buffer_view_bytes(js, binary, img['bufferView']))
        shrink_texture(path)
        written.append(name)
        image_files[img_index] = name
        return name

    # ---- materials ----------------------------------------------------------
    materials = []
    for i, mat in enumerate(js.get('materials', [{}]) or [{}]):
        pbr = mat.get('pbrMetallicRoughness', {})
        factor = pbr.get('baseColorFactor', [1.0, 1.0, 1.0, 1.0])
        rgba = [linear_to_srgb(factor[0]), linear_to_srgb(factor[1]),
                linear_to_srgb(factor[2]), float(factor[3])]
        tex = None
        if 'baseColorTexture' in pbr:
            tex = texture_file(pbr['baseColorTexture']['index'])
        materials.append({'id': 'mat%d' % i, 'name': mat.get('name', 'mat%d' % i),
                          'rgba': rgba, 'texture': tex})

    E = ET.Element
    sub = ET.SubElement
    root = E('COLLADA', {'xmlns': 'http://www.collada.org/2005/11/COLLADASchema', 'version': '1.4.1'})
    asset = sub(root, 'asset')
    contrib = sub(asset, 'contributor')
    sub(contrib, 'authoring_tool').text = 'galbot_s1_description/tools/import_upstream.py'
    now = datetime.datetime(2026, 1, 1).isoformat()
    sub(asset, 'created').text = now
    sub(asset, 'modified').text = now
    sub(asset, 'unit', {'name': 'meter', 'meter': '1'})
    sub(asset, 'up_axis').text = 'Z_UP'

    lib_images = sub(root, 'library_images')
    image_ids = {}
    for m in materials:
        if m['texture'] and m['texture'] not in image_ids:
            iid = 'img%d' % len(image_ids)
            image_ids[m['texture']] = iid
            sub(sub(lib_images, 'image', {'id': iid, 'name': iid}), 'init_from').text = m['texture']

    lib_effects = sub(root, 'library_effects')
    lib_materials = sub(root, 'library_materials')
    for m in materials:
        eff = sub(lib_effects, 'effect', {'id': m['id'] + '-effect'})
        prof = sub(eff, 'profile_COMMON')
        if m['texture']:
            iid = image_ids[m['texture']]
            surf = sub(sub(prof, 'newparam', {'sid': iid + '-surface'}), 'surface', {'type': '2D'})
            sub(surf, 'init_from').text = iid
            samp = sub(sub(prof, 'newparam', {'sid': iid + '-sampler'}), 'sampler2D')
            sub(samp, 'source').text = iid + '-surface'
        tech = sub(prof, 'technique', {'sid': 'common'})
        phong = sub(tech, 'phong')
        sub(sub(phong, 'emission'), 'color', {'sid': 'emission'}).text = '0 0 0 1'
        r, g, b, a = m['rgba']
        if m['texture']:
            sub(sub(phong, 'ambient'), 'color', {'sid': 'ambient'}).text = '0.8 0.8 0.8 1'
            sub(sub(phong, 'diffuse'), 'texture',
                {'texture': image_ids[m['texture']] + '-sampler', 'texcoord': 'UVMap'})
        else:
            sub(sub(phong, 'ambient'), 'color', {'sid': 'ambient'}).text = _fmt([r, g, b, 1.0], 4)
            sub(sub(phong, 'diffuse'), 'color', {'sid': 'diffuse'}).text = _fmt([r, g, b, a], 4)
        sub(sub(phong, 'specular'), 'color', {'sid': 'specular'}).text = '0.25 0.25 0.25 1'
        sub(sub(phong, 'shininess'), 'float', {'sid': 'shininess'}).text = '30'
        mat_el = sub(lib_materials, 'material', {'id': m['id'], 'name': m['name']})
        sub(mat_el, 'instance_effect', {'url': '#' + m['id'] + '-effect'})

    lib_geoms = sub(root, 'library_geometries')
    scene_nodes = []
    gi = 0
    for mesh in js['meshes']:
        for prim in mesh['primitives']:
            if prim.get('mode', 4) != 4:
                continue  # only triangles
            attrs = prim['attributes']
            pos = accessor(js, binary, attrs['POSITION']).astype(np.float64)
            if 'NORMAL' in attrs:
                nrm = accessor(js, binary, attrs['NORMAL']).astype(np.float64)
            else:
                nrm = None
            uv = None
            mat_index = prim.get('material', 0)
            if 'TEXCOORD_0' in attrs and materials[mat_index]['texture']:
                uv = accessor(js, binary, attrs['TEXCOORD_0']).astype(np.float64).copy()
                uv[:, 1] = 1.0 - uv[:, 1]  # glTF (top-left) -> COLLADA (bottom-left)
            if 'indices' in prim:
                idx = accessor(js, binary, prim['indices']).astype(np.int64)
            else:
                idx = np.arange(len(pos), dtype=np.int64)
            gid = 'geom%d' % gi
            geom = sub(lib_geoms, 'geometry', {'id': gid, 'name': '%s_%d' % (mesh.get('name', 'mesh'), gi)})
            m_el = sub(geom, 'mesh')

            def add_source(name, arr, params, digits):
                sid = '%s-%s' % (gid, name)
                src = sub(m_el, 'source', {'id': sid})
                fa = sub(src, 'float_array', {'id': sid + '-array', 'count': str(arr.size)})
                fa.text = _fmt(arr.reshape(-1), digits)
                acc = sub(sub(src, 'technique_common'), 'accessor',
                          {'source': '#' + sid + '-array', 'count': str(arr.shape[0]),
                           'stride': str(arr.shape[1])})
                for p in params:
                    sub(acc, 'param', {'name': p, 'type': 'float'})
                return sid

            pos_id = add_source('positions', pos, ['X', 'Y', 'Z'], 7)
            nrm_id = add_source('normals', nrm, ['X', 'Y', 'Z'], 4) if nrm is not None else None
            uv_id = add_source('texcoords', uv, ['S', 'T'], 6) if uv is not None else None
            verts = sub(m_el, 'vertices', {'id': gid + '-vertices'})
            sub(verts, 'input', {'semantic': 'POSITION', 'source': '#' + pos_id})
            tri = sub(m_el, 'triangles', {'material': materials[mat_index]['id'],
                                          'count': str(len(idx) // 3)})
            # One offset per input (like Blender exports). The Gazebo Classic /
            # gz-common loaders assume distinct offsets: sharing one index for
            # position, normal and uv makes them read the index list with the
            # wrong stride (and crash).
            n_inputs = 1
            sub(tri, 'input', {'semantic': 'VERTEX', 'source': '#' + gid + '-vertices', 'offset': '0'})
            if nrm_id:
                sub(tri, 'input', {'semantic': 'NORMAL', 'source': '#' + nrm_id, 'offset': str(n_inputs)})
                n_inputs += 1
            if uv_id:
                sub(tri, 'input', {'semantic': 'TEXCOORD', 'source': '#' + uv_id, 'offset': str(n_inputs),
                                   'set': '0'})
                n_inputs += 1
            sub(tri, 'p').text = ' '.join(' '.join([str(int(v))] * n_inputs) for v in idx)
            scene_nodes.append((gid, materials[mat_index]))
            gi += 1

    lib_scenes = sub(root, 'library_visual_scenes')
    vscene = sub(lib_scenes, 'visual_scene', {'id': 'Scene', 'name': 'Scene'})
    for gid, mat in scene_nodes:
        node = sub(vscene, 'node', {'id': gid + '-node', 'name': gid, 'type': 'NODE'})
        ig = sub(node, 'instance_geometry', {'url': '#' + gid})
        im = sub(sub(sub(ig, 'bind_material'), 'technique_common'), 'instance_material',
                 {'symbol': mat['id'], 'target': '#' + mat['id']})
        if mat['texture']:
            sub(im, 'bind_vertex_input', {'semantic': 'UVMap', 'input_semantic': 'TEXCOORD',
                                          'input_set': '0'})
    sub(sub(root, 'scene'), 'instance_visual_scene', {'url': '#Scene'})

    tree = ET.ElementTree(root)
    ET.indent(tree, space=' ')
    tree.write(dae_path, encoding='utf-8', xml_declaration=True)
    return written


# --------------------------------------------------------------------------- STL
def read_stl_vertices(path):
    with open(path, 'rb') as f:
        data = f.read()
    if data[:5] == b'solid' and b'facet' in data[:512]:
        verts = []
        for line in data.decode(errors='ignore').splitlines():
            tok = line.strip().split()
            if tok and tok[0] == 'vertex':
                verts.append([float(v) for v in tok[1:4]])
        return np.array(verts)
    n = struct.unpack('<I', data[80:84])[0]
    rec = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]),
                        count=n, offset=84)
    return rec['v'].reshape(-1, 3).astype(np.float64)


# --------------------------------------------------------------------------- URDF -> xacro
def _num(v):
    return ('%.10g' % float(v))


def repair_inertia(ixx, ixy, ixz, iyy, iyz, izz):
    """Make an inertia tensor physically valid (positive definite and the
    principal moments satisfy the triangle inequality). Returns (values, changed)."""
    M = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    w, V = np.linalg.eigh(M)
    w = np.maximum(w, 1e-9)
    changed = False
    for _ in range(3):
        order = np.argsort(w)
        a, b, c = w[order]
        if a + b < c * (1.0 + 1e-6):
            w[order[0]] = c * (1.0 + 1e-4) - b
            changed = True
    if not changed:
        return (ixx, ixy, ixz, iyy, iyz, izz), False
    R = V @ np.diag(w) @ V.T
    return (R[0, 0], R[0, 1], R[0, 2], R[1, 1], R[1, 2], R[2, 2]), True


def generate_body_xacro(urdf_path, wheel_radius, ground_offset):
    src = ET.parse(urdf_path).getroot()
    robot_name = src.get('name')
    link_names = {l.get('name') for l in src.findall('link')}
    # Upstream names one joint like its child link (right_fixed_contact_link);
    # SDF requires unique frame names, so such joints get a "_joint" name.
    joint_rename = {}
    for j in src.findall('joint'):
        if j.get('name') in link_names:
            new = j.get('name')
            new = new[:-5] + '_joint' if new.endswith('_link') else new + '_joint'
            joint_rename[j.get('name')] = new
    repaired = []
    for l in src.findall('link'):
        ine = l.find('inertial')
        if ine is None or float(ine.find('mass').get('value')) <= 0.0:
            continue
        I = ine.find('inertia')
        vals = [float(I.get(k)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')]
        fixed, changed = repair_inertia(*vals)
        if changed:
            for k, v in zip(('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'), fixed):
                I.set(k, '%.10g' % v)
            repaired.append(l.get('name'))
    lines = []
    w = lines.append
    w('<?xml version="1.0"?>')
    w('<!--')
    w('  AUTO-GENERATED by galbot_s1_description/tools/import_upstream.py - do not edit by hand.')
    w('  Source: GalaxyGeneralRobotics/galbot_s1_description urdf/galbot_s1.urdf')
    w('          (robot "%s", Apache License 2.0)' % robot_name)
    w('  Changes from upstream:')
    w('    * mesh paths use the ${mesh_uri} macro parameter, visuals are COLLADA (.dae)')
    w('    * base_footprint root link added on the floor (base_link is %.4f m above it)' % ground_offset)
    w('    * wheel collisions are spheres (r = %.4f m)' % wheel_radius)
    w('    * mimic tags are emitted only when use_mimic is true')
    w('    * zero-mass inertial blocks removed (all of them are fixed-joint children)')
    if joint_rename:
        w('    * joints renamed (name clashed with a link, invalid in SDF): %s' % ', '.join(
            '%s -> %s' % kv for kv in joint_rename.items()))
    if repaired:
        w('    * inertia made physically valid (triangle inequality) for: %s' % ', '.join(repaired))
    w('    * simulation limit overrides: %s' % ', '.join(
        '%s %s' % (j, ' '.join('%s=%g' % kv for kv in v.items())) for j, v in LIMIT_OVERRIDES.items()))
    w('-->')
    w('<robot xmlns:xacro="http://www.ros.org/wiki/xacro">')
    w('  <xacro:property name="galbot_s1_upstream_name" value="%s"/>' % robot_name)
    w('  <xacro:property name="galbot_s1_wheel_radius" value="%s"/>' % _num(wheel_radius))
    w('  <xacro:property name="galbot_s1_ground_offset" value="%s"/>' % _num(ground_offset))
    w('  <xacro:macro name="galbot_s1_body" params="mesh_uri use_mimic:=true">')
    w('    <link name="base_footprint"/>')
    w('    <joint name="base_footprint_joint" type="fixed">')
    w('      <origin xyz="0 0 ${galbot_s1_ground_offset}" rpy="0 0 0"/>')
    w('      <parent link="base_footprint"/>')
    w('      <child link="base_link"/>')
    w('    </joint>')

    def origin_str(el):
        o = el.find('origin')
        xyz = o.get('xyz', '0 0 0') if o is not None else '0 0 0'
        rpy = o.get('rpy', '0 0 0') if o is not None else '0 0 0'
        xyz = ' '.join(_num(v) for v in xyz.split())
        rpy = ' '.join(_num(v) for v in rpy.split())
        return '<origin xyz="%s" rpy="%s"/>' % (xyz, rpy)

    for el in src:
        if el.tag == 'link':
            name = el.get('name')
            vis = el.findall('visual')
            col = el.findall('collision')
            ine = el.find('inertial')
            has_mass = ine is not None and float(ine.find('mass').get('value')) > 0.0
            if not vis and not col and not has_mass:
                w('    <link name="%s"/>' % name)
                continue
            w('    <link name="%s">' % name)
            for v in vis:
                mesh = v.find('geometry/mesh')
                base = os.path.splitext(os.path.basename(mesh.get('filename')))[0]
                w('      <visual>')
                w('        ' + origin_str(v))
                w('        <geometry><mesh filename="${mesh_uri}/visual/%s.dae"/></geometry>' % base)
                w('      </visual>')
            if has_mass:
                m = ine.find('mass').get('value')
                I = ine.find('inertia')
                w('      <inertial>')
                w('        ' + origin_str(ine))
                w('        <mass value="%s"/>' % _num(m))
                w('        <inertia ixx="%s" ixy="%s" ixz="%s" iyy="%s" iyz="%s" izz="%s"/>' % tuple(
                    _num(I.get(k)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')))
                w('      </inertial>')
            for c in col:
                mesh = c.find('geometry/mesh')
                base = os.path.splitext(os.path.basename(mesh.get('filename')))[0]
                w('      <collision>')
                w('        ' + origin_str(c))
                if name in WHEEL_LINKS:
                    w('        <geometry><sphere radius="${galbot_s1_wheel_radius}"/></geometry>')
                else:
                    w('        <geometry><mesh filename="${mesh_uri}/collision/%s.stl"/></geometry>' % base)
                w('      </collision>')
            w('    </link>')
        elif el.tag == 'joint':
            name = joint_rename.get(el.get('name'), el.get('name'))
            jtype = el.get('type')
            w('    <joint name="%s" type="%s">' % (name, jtype))
            w('      ' + origin_str(el))
            w('      <parent link="%s"/>' % el.find('parent').get('link'))
            w('      <child link="%s"/>' % el.find('child').get('link'))
            ax = el.find('axis')
            if ax is not None:
                w('      <axis xyz="%s"/>' % ' '.join(_num(v) for v in ax.get('xyz').split()))
            lim = el.find('limit')
            if lim is not None:
                attrs = dict(lim.attrib)
                over = LIMIT_OVERRIDES.get(name, {})
                note = ''
                if over:
                    note = '  <!-- upstream: %s -->' % ' '.join(
                        '%s=%s' % (k, attrs.get(k, '-')) for k in over)
                    for k, v in over.items():
                        attrs[k] = str(v)
                order = [k for k in ('effort', 'lower', 'upper', 'velocity') if k in attrs]
                w('      <limit %s/>%s' % (' '.join('%s="%s"' % (k, _num(attrs[k])) for k in order), note))
            mim = el.find('mimic')
            if mim is not None:
                w('      <xacro:if value="${use_mimic}">')
                w('        <mimic joint="%s" multiplier="%s" offset="%s"/>' % (
                    mim.get('joint'), _num(mim.get('multiplier', '1')), _num(mim.get('offset', '0'))))
                w('      </xacro:if>')
            w('    </joint>')
    w('  </xacro:macro>')
    w('</robot>')
    return '\n'.join(lines) + '\n'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('upstream', help='path to a clone of GalaxyGeneralRobotics/galbot_s1_description')
    args = ap.parse_args()

    up_urdf_dir = os.path.join(args.upstream, 'urdf')
    up_urdf = os.path.join(up_urdf_dir, 'galbot_s1.urdf')
    if not os.path.isfile(up_urdf):
        sys.exit('not found: %s' % up_urdf)

    vis_out = os.path.join(PKG_DIR, 'meshes', 'visual')
    col_out = os.path.join(PKG_DIR, 'meshes', 'collision')
    for d in (vis_out, col_out):
        if os.path.isdir(d):
            shutil.rmtree(d)
        os.makedirs(d)

    vis_in = os.path.join(up_urdf_dir, 'meshes', 'visual')
    for fn in sorted(os.listdir(vis_in)):
        if not fn.endswith('.glb'):
            continue
        base = fn[:-4]
        textures = glb_to_dae(os.path.join(vis_in, fn), os.path.join(vis_out, base + '.dae'), base)
        print('visual    %-30s -> %s.dae %s' % (fn, base, ' '.join(textures)))

    col_in = os.path.join(up_urdf_dir, 'meshes', 'collision')
    for fn in sorted(os.listdir(col_in)):
        if fn.endswith('.stl'):
            shutil.copy2(os.path.join(col_in, fn), os.path.join(col_out, fn))
    print('collision copied: %d files' % len([f for f in os.listdir(col_out) if f.endswith('.stl')]))

    # Wheel radius from the drive wheel collision mesh (wheel axis = local z).
    wv = read_stl_vertices(os.path.join(col_in, 'wheel_driving_link1.stl'))
    wheel_radius = float(np.max(np.hypot(wv[:, 0], wv[:, 1])))
    # Height of the wheel axle above base_link (steering joint origin z).
    src = ET.parse(up_urdf).getroot()
    axle_z = None
    for j in src.findall('joint'):
        if j.get('name') == 'wheel_steering_joint1':
            axle_z = float(j.find('origin').get('xyz').split()[2])
    ground_offset = wheel_radius - axle_z
    print('wheel radius %.4f m, axle %.4f m above base_link -> base_link is %.4f m above floor'
          % (wheel_radius, axle_z, ground_offset))

    with open(os.path.join(PKG_DIR, 'urdf', 'galbot_s1_body.xacro'), 'w') as f:
        f.write(generate_body_xacro(up_urdf, wheel_radius, ground_offset))
    print('wrote urdf/galbot_s1_body.xacro')

    lic = os.path.join(args.upstream, 'LICENSE')
    if os.path.isfile(lic):
        shutil.copy2(lic, os.path.join(PKG_DIR, 'LICENSE.upstream'))


if __name__ == '__main__':
    main()
