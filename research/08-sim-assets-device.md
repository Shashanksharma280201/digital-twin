# Sim World Assets and Handheld Device Spec (Blender 5.2)

*Verified 2026-10-18. Asset APIs were checked with curl; CAD links returned HTTP 200 without login.*

## Procedural site: write our own bpy/bmesh generators
Building Tools, Archimesh, Buildify and SceneCity are all unreliable for headless use or not freely licensed. We generate everything from seeded parameters:
- **Structure:** column grid (6 m bays, 400 mm columns), 200–250 mm slabs, partial upper floors driven by a progress parameter.
- **Rebar:** curves with 6–16 mm bevel; starter bars at column tops.
- **Formwork:** plywood panels 1220×2440×18 mm, Ø48 mm shores.
- **Scaffolding:** Ø48.3 mm tubes, 2.0 m lifts, 2.5 m bays, 0.73 m wide.
- **Walls and stairs:** blockwork 390×190×190; stairs with 175 mm rise, 280 mm going.
- **Terrain:** displaced plane, excavation pit, spoil piles.
- **Props:** cones, Jersey barriers, pallets (1200×800×144), fencing, and a parametric excavator.

## Blender 4.x/5.x API changes we rely on
- **Principled BSDF socket names:** `Base Color`, `Metallic`, `Roughness`, `Specular IOR Level`, `Emission Color`, `Emission Strength`, `Coat Weight`, `Transmission Weight`.
- **5.0 render and compositor:**
  - EEVEE engine id is `BLENDER_EEVEE`; `BLENDER_EEVEE_NEXT` was removed.
  - Compositor uses `scene.compositing_node_group`; `use_nodes`/`node_tree` are deprecated.
  - Render passes renamed: `Z` → `Depth`.
  - `use_gtao` was removed.
- **5.0 data API:**
  - `action.fcurves` was removed (use channelbags / `bpy_extras.anim_utils`).
  - `mathutils` buffers are float32.
  - Boolean solver `FAST` is now `FLOAT`.
- **Python** is 3.13. There is no STEP import. The snap build is confined, so keep files under `$HOME`.
- **Headless GPU render:** `prefs.addons['cycles'].preferences.compute_device_type='OPTIX'`, then `get_devices()`.

## CC0 assets
- **Poly Haven** (requires a unique User-Agent):
  - API: `https://api.polyhaven.com/assets?type=…`, `/files/{id}`
  - Direct downloads: `https://dl.polyhaven.org/file/ph-assets/Textures/jpg/1k/<id>/<id>_diff_1k.jpg`, `…/HDRIs/exr/2k/<id>_2k.exr`
  - HDRI: `abandoned_construction`
  - Textures: `concrete_floor`, `rebar_reinforced_concrete`, `plywood`, `rusty_metal_02`, `gravel`, `raked_dirt`, `asphalt_02`, `brick_wall_001`
  - Models: `concrete_road_barrier`, `wooden_crate_01`, `Barrel_01`, `wooden_ladder`, `cement_bag`, `portable_generator`, `modular_chainlink_fence`, `old_tyre`
  - Not on Poly Haven: cone, pallet, scaffolding, excavator.
- **ambientCG:** `https://ambientcg.com/get?file={id}_2K-JPG.zip`
  - Concrete034, Wood089, Gravel023, Asphalt031, Bricks085, Metal021, CorrugatedSteel005.

## Handheld references
| Device | Form | Size / mass |
|---|---|---|
| Leica BLK2GO | Cylinder wand, LiDAR dome on top | 279 mm × Ø80, 775 g |
| XGRIDS Lixel K1 | Handle + tilted 360° head (Mid-360-like) + 2 panoramic cameras | ~1 kg |
| FARO Orbis | Head on handle + separate datalogger | head 2.1 kg |
| GeoSLAM ZEB Horizon RT | VLP-16 tilted on handle, tethered logger | head 1.45 kg |
| Feima SLAM100 | Rotating LiDAR on top, side cameras | 372×163×106 mm, 1.6 kg |

**Common pattern:** LiDAR on top, tilted 20–45°; cameras forward, just below it; vertical handle; compute in a hip logger.

## Sensor geometry
- **Mid-360:** 65×65×60 mm, 265 g, FoV 360° × (−7°…+52°).
  - Point-cloud origin is about 47 mm above the base on the axis **[verify]**.
  - IMU at (11.0, 23.29, −44.12) mm.
  - STEP: `https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/mid-360-asm.stp`
- **Airy:** Ø60 × 63 mm, under 240 g, FoV 360°×90°. No public STEP.
- **ZED 2i:** 175.25 × 30.25 × 43.10 mm, ~230 g, 120 mm baseline.
  - STEP: `https://cdn.sanity.io/files/s18ewfw4/staging/1eae249387d9bde53972abbe470483024c31c6fb.step/ZED2i.step`
- **CAD files are vendor-copyrighted.** Use them for local reference only; the procedural model is the default.
- **AGX Orin devkit:** 110×110×71.65 mm, ~1 kg+ (estimate), so it goes in a hip box.

## Proposed device ("DT-1 handheld")
- **Handle:** Ø38 × 135 mm, tilted 12°.
- **Head:** 190×75×85 mm. ZED 2i recessed in the front face, pitched down 10°.
- **LiDAR:** on a wedge on top, pitched forward 30° (parameter range 0–45°). Mid-360 or Airy, swappable.
- **Rear:** phone clamp.
- **Hip box:** 200×150×95 mm, holding the Orin + a 98 Wh V-mount; tethered to the head.
- **Mass and power:** head ~0.95 kg; hip box ~2 kg; ~45 W total, ~2 h runtime.
- **Frames:** sensor optical frames are modelled as empties (`lidar_link`, `zed_left`, `zed_right`, `imu_link`).
