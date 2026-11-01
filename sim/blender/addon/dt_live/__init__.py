"""DT Live: shows the digital twin being built, live, inside Blender.

Connects to the ROS 2 node `dt_blender_bridge` (TCP, see protocol.py) and updates:
  DT_Estimated  estimated device pose (instances the DT-1 device collection if present)
  DT_GroundTruth  ground-truth device pose (sim only)
  DT_Map        accumulated map as a point cloud, coloured by height
  DT_EstPath / DT_GTPath  trajectories
Sidebar: View3D > N panel > "DT Live".
"""
import socket
import time

import bpy
import numpy as np

from . import protocol

bl_info = {
    'name': 'DT Live Bridge',
    'author': 'Flo Mobility digital twin',
    'version': (0, 1, 0),
    'blender': (4, 2, 0),
    'location': 'View3D > Sidebar > DT Live',
    'description': 'Live view of the ROS 2 digital-twin stack (pose, map, ground truth)',
    'category': '3D View',
}

COLLECTION = 'DT_Live'
EST_COLOR = (1.0, 0.45, 0.05, 1.0)
GT_COLOR = (0.1, 0.85, 0.25, 1.0)


class _State:
    sock = None
    decoder = None
    est_path = []
    gt_path = []
    last_path_update = 0.0
    frames = 0
    points = 0
    status = 'disconnected'


def _collection():
    coll = bpy.data.collections.get(COLLECTION)
    if coll is None:
        coll = bpy.data.collections.new(COLLECTION)
        bpy.context.scene.collection.children.link(coll)
    return coll


def _device_collection():
    for coll in bpy.data.collections:
        if coll.name.upper().startswith('DT1') or coll.name.lower().startswith('device'):
            return coll
    return None


def _pose_object(name, color):
    obj = bpy.data.objects.get(name)
    if obj is None:
        obj = bpy.data.objects.new(name, None)
        obj.empty_display_type = 'ARROWS'
        obj.empty_display_size = 0.4
        obj.color = color
        obj.rotation_mode = 'QUATERNION'
        device = _device_collection()
        if device is not None and name == 'DT_Estimated':
            obj.instance_type = 'COLLECTION'
            obj.instance_collection = device
        _collection().objects.link(obj)
    return obj


def apply_pose(name, pose7, color):
    obj = _pose_object(name, color)
    x, y, z, qx, qy, qz, qw = (float(v) for v in pose7)
    obj.location = (x, y, z)
    obj.rotation_quaternion = (qw, qx, qy, qz)
    return obj


def _emission_material(name, attribute=None, color=(1, 1, 1, 1), strength=1.5):
    mat = bpy.data.materials.get(name)
    if mat is not None:
        return mat
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes, links = mat.node_tree.nodes, mat.node_tree.links
    nodes.clear()
    out = nodes.new('ShaderNodeOutputMaterial')
    emit = nodes.new('ShaderNodeEmission')
    emit.inputs['Strength'].default_value = strength
    if attribute:
        attr = nodes.new('ShaderNodeAttribute')
        attr.attribute_type = 'GEOMETRY'
        attr.attribute_name = attribute
        links.new(attr.outputs['Color'], emit.inputs['Color'])
    else:
        emit.inputs['Color'].default_value = color
    links.new(emit.outputs['Emission'], out.inputs['Surface'])
    mat.diffuse_color = color
    return mat


def _points_node_group(radius):
    ng = bpy.data.node_groups.get('DT_MeshToPoints')
    if ng is None:
        ng = bpy.data.node_groups.new('DT_MeshToPoints', 'GeometryNodeTree')
        ng.interface.new_socket('Geometry', in_out='INPUT', socket_type='NodeSocketGeometry')
        ng.interface.new_socket('Geometry', in_out='OUTPUT', socket_type='NodeSocketGeometry')
        gin = ng.nodes.new('NodeGroupInput')
        gout = ng.nodes.new('NodeGroupOutput')
        m2p = ng.nodes.new('GeometryNodeMeshToPoints')
        setmat = ng.nodes.new('GeometryNodeSetMaterial')
        setmat.inputs['Material'].default_value = _emission_material('DT_MapMaterial', attribute='Col')
        ng.links.new(gin.outputs[0], m2p.inputs['Mesh'])
        ng.links.new(m2p.outputs['Points'], setmat.inputs['Geometry'])
        ng.links.new(setmat.outputs['Geometry'], gout.inputs[0])
    ng.nodes['Mesh to Points'].inputs['Radius'].default_value = radius
    return ng


def height_colors(z):
    """Blue -> cyan -> green -> yellow -> red over the z range (RGBA float32)."""
    stops = np.array([[0.15, 0.25, 0.9], [0.0, 0.8, 0.9], [0.2, 0.85, 0.2], [0.95, 0.85, 0.1], [0.9, 0.2, 0.1]])
    lo, hi = np.percentile(z, 2), np.percentile(z, 98)
    s = np.clip((z - lo) / max(hi - lo, 1e-6), 0, 1) * (len(stops) - 1)
    i = np.minimum(s.astype(int), len(stops) - 2)
    f = (s - i)[:, None]
    rgb = stops[i] * (1 - f) + stops[i + 1] * f
    return np.hstack([rgb, np.ones((len(z), 1))]).astype(np.float32)


def apply_cloud(points, radius=0.025):
    obj = bpy.data.objects.get('DT_Map')
    if obj is None:
        mesh = bpy.data.meshes.new('DT_Map')
        obj = bpy.data.objects.new('DT_Map', mesh)
        _collection().objects.link(obj)
        mod = obj.modifiers.new('DT_Points', 'NODES')
        mod.node_group = _points_node_group(radius)
    mesh = obj.data
    mesh.clear_geometry()
    n = len(points)
    mesh.vertices.add(n)
    mesh.vertices.foreach_set('co', np.ascontiguousarray(points[:, :3], np.float32).ravel())
    col = mesh.color_attributes.get('Col') or mesh.color_attributes.new('Col', 'FLOAT_COLOR', 'POINT')
    if n:
        col.data.foreach_set('color', height_colors(points[:, 2]).ravel())
    mesh.update()
    _State.points = n
    return obj


def apply_path(name, positions, color):
    curve = bpy.data.curves.get(name)
    if curve is None:
        curve = bpy.data.curves.new(name, 'CURVE')
        curve.dimensions = '3D'
        curve.bevel_depth = 0.02
        obj = bpy.data.objects.new(name, curve)
        obj.data.materials.append(_emission_material(name + '_mat', color=color, strength=3.0))
        _collection().objects.link(obj)
    curve.splines.clear()
    if len(positions) < 2:
        return
    spline = curve.splines.new('POLY')
    spline.points.add(len(positions) - 1)
    pts = np.hstack([np.asarray(positions, np.float32), np.ones((len(positions), 1), np.float32)])
    spline.points.foreach_set('co', pts.ravel())


def handle_frames(frames, radius=0.025):
    """Applies decoded frames (only the latest pose/cloud of each kind matters)."""
    latest = {}
    for header, array in frames:
        kind = header['type']
        if kind in ('pose', 'gt_pose') and array is not None:
            (_State.est_path if kind == 'pose' else _State.gt_path).append(array[:3].copy())
        latest[kind] = array
    if latest.get('pose') is not None:
        apply_pose('DT_Estimated', latest['pose'], EST_COLOR)
    if latest.get('gt_pose') is not None:
        apply_pose('DT_GroundTruth', latest['gt_pose'], GT_COLOR)
    if 'cloud' in latest and latest['cloud'] is not None:
        apply_cloud(latest['cloud'], radius)
    now = time.monotonic()
    if now - _State.last_path_update > 1.0:
        _State.last_path_update = now
        apply_path('DT_EstPath', _State.est_path, EST_COLOR)
        apply_path('DT_GTPath', _State.gt_path, GT_COLOR)
    _State.frames += len(frames)


def poll_once():
    """Reads whatever is available without blocking. Returns frames handled, or None if disconnected."""
    if _State.sock is None:
        return None
    chunks = []
    try:
        while True:
            data = _State.sock.recv(1 << 20)
            if not data:
                disconnect('server closed')
                return None
            chunks.append(data)
            if sum(len(c) for c in chunks) > (64 << 20):
                break
    except BlockingIOError:
        pass
    except OSError as exc:
        disconnect(f'error: {exc}')
        return None
    frames = _State.decoder.feed(b''.join(chunks)) if chunks else []
    if frames:
        scene = bpy.context.scene
        handle_frames(frames, getattr(scene, 'dt_point_radius', 0.025))
    return len(frames)


def _timer():
    if poll_once() is None:
        return None
    return 0.05


def connect(host='127.0.0.1', port=5577):
    disconnect()
    sock = socket.create_connection((host, port), timeout=3)
    sock.setblocking(False)
    _State.sock, _State.decoder = sock, protocol.Decoder()
    _State.est_path, _State.gt_path, _State.frames = [], [], 0
    _State.status = f'connected to {host}:{port}'
    if not bpy.app.background and not bpy.app.timers.is_registered(_timer):
        bpy.app.timers.register(_timer)


def disconnect(reason='disconnected'):
    if _State.sock is not None:
        try:
            _State.sock.close()
        except OSError:
            pass
    _State.sock = None
    _State.status = reason
    if bpy.app.timers.is_registered(_timer):
        bpy.app.timers.unregister(_timer)


class DT_OT_connect(bpy.types.Operator):
    bl_idname = 'dt_live.connect'
    bl_label = 'Connect'
    bl_description = 'Connect to the dt_blender_bridge ROS 2 node'

    def execute(self, context):
        try:
            connect(context.scene.dt_host, context.scene.dt_port)
        except OSError as exc:
            self.report({'ERROR'}, f'cannot connect: {exc} (is dt_blender_bridge running?)')
            return {'CANCELLED'}
        return {'FINISHED'}


class DT_OT_disconnect(bpy.types.Operator):
    bl_idname = 'dt_live.disconnect'
    bl_label = 'Disconnect'

    def execute(self, context):
        disconnect()
        return {'FINISHED'}


class DT_OT_clear(bpy.types.Operator):
    bl_idname = 'dt_live.clear'
    bl_label = 'Clear'
    bl_description = 'Remove live objects (map, paths, poses)'

    def execute(self, context):
        coll = bpy.data.collections.get(COLLECTION)
        if coll:
            for obj in list(coll.objects):
                bpy.data.objects.remove(obj, do_unlink=True)
        _State.est_path, _State.gt_path = [], []
        return {'FINISHED'}


class DT_PT_live(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'DT Live'
    bl_label = 'Digital Twin Live'

    def draw(self, context):
        layout, scene = self.layout, context.scene
        layout.prop(scene, 'dt_host')
        layout.prop(scene, 'dt_port')
        row = layout.row()
        if _State.sock is None:
            row.operator(DT_OT_connect.bl_idname, icon='LINKED')
        else:
            row.operator(DT_OT_disconnect.bl_idname, icon='UNLINKED')
        row.operator(DT_OT_clear.bl_idname, icon='TRASH')
        layout.prop(scene, 'dt_point_radius')
        layout.label(text=_State.status)
        layout.label(text=f'frames {_State.frames}  map points {_State.points}')


_CLASSES = (DT_OT_connect, DT_OT_disconnect, DT_OT_clear, DT_PT_live)


def register():
    bpy.types.Scene.dt_host = bpy.props.StringProperty(name='Host', default='127.0.0.1')
    bpy.types.Scene.dt_port = bpy.props.IntProperty(name='Port', default=5577, min=1, max=65535)
    bpy.types.Scene.dt_point_radius = bpy.props.FloatProperty(name='Point radius', default=0.025,
                                                             min=0.002, max=0.2)
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister():
    disconnect()
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.dt_host
    del bpy.types.Scene.dt_port
    del bpy.types.Scene.dt_point_radius
