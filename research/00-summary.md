# Digital Twin: Research Summary

*2026-10-18. Detailed reports: [01-landscape](01-landscape.md) · [02-hardware](02-hardware.md) · [03-algorithms-software](03-algorithms-software.md)*

## What a "complete" digital twin is made of

A digital twin has five layers. Most products on the market cover only one or two.

| Layer | What it is | Update rate | Main tech |
|---|---|---|---|
| **1. Geometry** | Metric 3D model of the space (point cloud, mesh) | Weekly or per capture | LiDAR SLAM, photogrammetry, TLS |
| **2. Appearance** | Photoreal, walkable view | Weekly | 360° imagery, Gaussian splatting |
| **3. Semantics** | Labelled objects (walls, ducts, rebar, machines), linked to BIM elements | Per capture | Point-cloud segmentation, SAM 3 / Grounding DINO lifted to 3D, IFC |
| **4. Time (4D)** | History of changes compared with the plan: progress, deviation, schedule | Per capture plus history | Scan-to-scan (M3C2), scan-to-BIM, P6/MSP linking |
| **5. Live state** | Where workers and equipment are, sensor readings, alerts | Seconds to minutes | Fixed cameras, UWB/BLE, telematics, IoT over MQTT |

## What others are doing (summary)
- **Most money is in progress tracking against BIM and the schedule**, captured by a person walking with a 360° helmet camera each week. Players: OpenSpace, Buildots (~$1B valuation), Doxel, Track3D and Cupix. Data centers are the leading vertical.
- **Capture is now a commodity.** Track3D gives documentation away free and charges $0.15/sq ft for progress analytics.
- **Autonomous capture is the new frontier, but still early.** DroneDeploy's Capture Agent (Spot/Unitree, navigates from drawings) is in early access. Spot costs $150k+.
- **Live data sits in silos.** Crane cameras (Versatile), safety CV (Intenseye), telematics (Propeller DirtMate) and fixed cameras (OxBlue/Evercam) are not fused into one spatial twin.
- **Incumbents keep buying startups:** Hexagon, Trimble, Bentley+Cesium, Autodesk, AMETEK/FARO, CoStar/Matterport.

## Opportunities that fit Flo Mobility (robotics)
1. **Autonomous ground-robot capture sold as a service.** A wheeled AMR (later legged) repeats a capture route nightly without a human. This makes capture cheap and frequent, and turns a weekly twin into a daily one.
2. **A fused live twin.** The robot's periodic scans become the geometry backbone, and cameras, UWB tags and telematics become the live layer on top.
3. **Trustworthy analytics.** Element-level progress and deviation with confidence scores, and occluded elements reported as unknown.
4. **A twin built for robots.** The same map serves navigation and simulation (Isaac Sim / USD), not just human viewing.

## Bill of materials: what we need

### Sensors (MVP to mid tier on our robot)
| Item | Pick | Approx. cost |
|---|---|---|
| 3D LiDAR | Hesai JT128 (hemispherical) or Livox Mid-360 (cheap, FAST-LIO standard) | $1-1.5k |
| Cameras | ZED X stereo (GMSL) + Insta360 X5 360° camera | ~$1.1k |
| Positioning | Unicore UM980 / u-blox F9P RTK on the robot + Emlid RS3/RS4 for ground control | $0.3k + $3-4k |
| Edge compute | Jetson AGX Orin 64 GB (Orin NX for MVP) | $1-3.5k |
| Storage | 2-4 TB NVMe on the robot | ~$0.3k |
| Exterior | DJI Matrice 4E drone (or Skydio, for NDAA compliance) | ~$5k |
| Reconstruction | Workstation with RTX 5090/6000-class GPU and 128 GB+ RAM, or cloud GPUs | $6-15k |
| Accuracy check | Rented TLS (RTC360) for ground truth | ~$500-1k/day |

Expected accuracy: **1-2 cm relative, 2-3 cm absolute**. That is enough for progress tracking (LOA10-20). It is not enough for QA against BIM, which needs 5-15 mm (LOA30) and requires TLS or a partnership.

### Live layer (phase 2)
Fixed or crane cameras, UWB badges (~$25-99 per tag, $500-800 per anchor), ISO 15143-3 (AEMP) telematics APIs, concrete maturity and environmental sensors over LoRaWAN, and Starlink or 5G backhaul.

### Algorithms and software (open-source first)
| Stage | Pick |
|---|---|
| Robot middleware | ROS 2 Jazzy, MCAP, Foxglove/Rerun |
| Odometry / SLAM | FAST-LIO2 / FAST-LIVO2 → GLIM (global, multi-session, GNSS) |
| Drone | WebODM |
| Point clouds | PDAL + COPC, Open3D, CloudCompare, small_gicp, TEASER++ |
| Visual twin | gsplat, seeded with LiDAR points (Apache; avoid Inria's non-commercial code) |
| Dynamic-object removal | MapMOS / DUFOMap (keep removed objects as the safety layer) |
| Semantics | Pointcept (PTv3/Sonata), SAM 3 + Grounding DINO / RF-DETR |
| Change and progress | py4dgeo M3C2, IfcOpenShell C2M per IFC GUID, MPXJ for P6/MSP schedules |
| Twin state and IoT | Eclipse Ditto, MQTT → Kafka/Redpanda → TimescaleDB + PostGIS |
| Viewer | CesiumJS (3D Tiles + splats), Potree, That Open Engine (IFC) |
| Formats | E57, LAZ/COPC, IFC 4.3, glTF + KHR_gaussian_splatting/SPZ, 3D Tiles, OpenUSD |

**Licensing to watch:** ORB-SLAM3 is GPL. OpenMVS and Ultralytics YOLO are AGPL. Inria 3DGS is non-commercial. FAST-LIO is GPL-2.0, which matters if we ship it on a product.

## Reference pipeline
```
Robot/drone/360/TLS capture ─► Ingest (MCAP/E57/LAZ → S3) ─► Align (LIO → GLIM merge → RTK/GCP georef → BIM registration)
   ─► Reconstruct (COPC, splats, mesh → 3D Tiles) ─► Semanticize (segment + remove dynamics + link to IFC)
   ─► Analyze (scan-to-scan, scan-to-BIM, progress → schedule, safety zones) ─► Serve (web viewer, API) ─► Alert
Edge: drivers, real-time LIO, PPE/equipment detection, blurring, compression.   Cloud: everything heavy.
```

## Proposed phased plan
| Phase | Goal | Outcome |
|---|---|---|
| **0. Rig and data (2-4 wks)** | Mid-360/JT128 + cameras + RTK on a Flo robot (or handheld pole); record MCAP | Repeated captures of one test site |
| **1. Geometry twin (4-6 wks)** | FAST-LIO2 → GLIM, georeferenced COPC, Potree/Cesium viewer | A measurable 3D site you can view in a browser |
| **2. Visual + 4D (4-6 wks)** | gsplat from the same captures; scan-to-scan change maps over time | A photoreal twin with a timeline slider and change heat maps |
| **3. Semantics + BIM (6-8 wks)** | IFC registration, per-element progress and deviation, schedule link | Progress reports |
| **4. Autonomy + live layer** | Nightly autonomous capture routes; cameras, UWB, telematics; alerts | A near-live twin; robotics-as-a-service pitch |

## Decisions needed from the team
1. Main user and buyer: GC project managers (progress), safety teams, owners, or our own robots.
2. Update rate: nightly autonomous capture vs. live (cameras + tracking) first.
3. Target site types: building interiors (BIM-heavy) vs. earthworks, solar and civil (underserved).
4. Reuse of existing Flo Mobility robot hardware.
