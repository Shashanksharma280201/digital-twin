"""Headless EEVEE render check (run via sim/blender/bl.sh). Writes a small PNG and prints timing."""
import sys
import time

import bpy

out = sys.argv[sys.argv.index('--') + 1] if '--' in sys.argv else '/tmp/dt_render_check.png'
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.engine = 'BLENDER_EEVEE'
scene.render.resolution_x, scene.render.resolution_y = 640, 360
scene.render.filepath = out
bpy.ops.mesh.primitive_monkey_add(location=(0, 0, 0))
bpy.ops.object.light_add(type='SUN', location=(2, -2, 4))
bpy.ops.object.camera_add(location=(0, -4, 1), rotation=(1.4, 0, 0))
scene.camera = bpy.context.object
t = time.time()
bpy.ops.render.render(write_still=True)
print(f'RENDER_OK {time.time() - t:.2f}s (first frame includes shader compile) -> {out}')
