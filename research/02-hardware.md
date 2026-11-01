# Hardware for a Construction-Site Digital Twin

*Research date: 2026-10-18. Price legend: **(~)** = reseller, third-party or estimate; **(?)** = sources conflict or unverified. Figures without a URL are background knowledge, not verified in this session.*

## 1. Capture sensors

### 1a. LiDAR sensors you can mount on a robot or vehicle
| Model | Key specs | Price (2025-26) | Notes |
|---|---|---|---|
| **Unitree L2** | 360°×96° FoV, 64k pts/s effective, 30 m range, ≤2 cm accuracy, built-in IMU, 230 g | **$419** ([Unitree](https://shop.unitree.com/products/unitree-4d-lidar-l2)) | Cheapest usable 3D LiDAR. Good for SLAM; too noisy for QA. |
| **Livox Mid-360** | 360°×59°, ~200k pts/s, 40 m at 10% reflectivity, ≤2 cm at 10 m, built-in IMU | $770-1,250 (China sellers); $3,800 at US resellers ([RoboStore](https://robostore.com/products/livox-mid-360-lidar)) | Standard sensor for FAST-LIO. An ISPRS 2025 test of 35 units found all within spec ([paper](https://isprs-archives.copernicus.org/articles/XLVIII-1-W6-2025/147/2025/isprs-archives-XLVIII-1-W6-2025-147-2025.pdf)). |
| **Livox Avia** | 70°×77° FoV, 450 m range | $2,079 ([DJI](https://store.dji.com/product/livox-avia)) | Suits drones and long range. |
| **Hesai JT128** | 128 channels, 360°×189° hemispherical | $1,129-1,500 ([SensorPartners](https://sensorpartners.com/en/product/jt128/)) | Excellent value. Sees floor and ceiling in one pass. |
| **Hesai XT32 / XT32M** | 32 channels, 120 m range, ±1 cm | $3.7-4.2k (XT32M ~$5.4-6.2k) ([LidarSeek](https://lidarseek.com/product/hesai-xt32)) | Survey-grade mobile LiDAR core. |
| **Ouster OS0 / OS1** | OS0: 90° vertical FoV, ~50 m. OS1: 45° vertical FoV, ~120 m. 32-128 channels | OS1-32 ~$6.4k; OS1-128 ~$12k **(~)**, sold by quote | Best ROS drivers. Also outputs near-IR and ambient images. |
| Velodyne VLP-16 | Legacy | used $1-3k **(~)** | End-of-life. |

### 1b. Terrestrial laser scanners (tripod, survey-grade)
| Model | Accuracy | Price |
|---|---|---|
| Leica BLK360 G2 | ~4 mm at 10 m | ~$28.7k **(~)** |
| Leica RTC360 | 1.9 mm at 10 m | $65-100k new **(~)** |
| FARO Focus Premium | ±1 mm ranging | ~€38k EU; $66-84k US **(?)** |
| Trimble X7 | 2.4 mm at 10 m | ~$35-50k **(~)** |

### 1c. Mobile and handheld SLAM scanners
| Model | Accuracy | Price |
|---|---|---|
| Leica BLK2GO | 6-10 mm relative | $37.5-61k ([Leica](https://shop.leica-geosystems.com/reality-capture/blk2go/buy)) |
| FARO Orbis | 5 mm SLAM, 2 mm flash mode | ~$20-35k **(~)** |
| GeoSLAM/FARO ZEB Horizon RT | 1-3 cm | ~$17-26k **(~)** |
| NavVis VLX 2/3 (wearable) | ~5 mm (claimed) | €60k to $130-170k **(?)** |
| Emesent Hovermap ST-X | 1-2 cm; works on drone, Spot, vehicle or backpack | $80-120k **(?)** |

Software adds $5-15k per year (Cyclone, IVION and similar).

### 1d. Cameras
| Type | Model | Price |
|---|---|---|
| 360° | Insta360 X5 (72 MP stills, 8K30 video) | ~$465 |
| 360° | Ricoh Theta Z1 (standard camera for OpenSpace-style walkthroughs) | ~$800-1,100 **(~)** |
| Stereo | ZED X (GMSL2, IP67, global shutter, built for Jetson) | $599 |
| Stereo | ZED 2i (USB) | $499 |
| RGB-D | RealSense D455 (noisy outdoors) | ~$300-420 |
| Stereo + AI | OAK-D Pro W (runs inference on-device) | ~$350-400 **(~)** |
| Thermal | FLIR Lepton/Boson | $250 to $3k+ **(~)** |

### 1e. Drones
| Model | Use | Price |
|---|---|---|
| DJI Matrice 4E (replaces the Mavic 3E) | RTK photogrammetry, 1-3 cm with ground control | ~$5.1k |
| DJI M400 + Zenmuse L3 | LiDAR, 3-4 cm RMSE | ~$28k |
| Skydio X10 | Autonomous, NDAA-compliant | $15-25k by quote |
| DJI Dock 3 + M4D | Drone-in-a-box for daily automated flights | from $15.9k |

US restrictions on DJI (NDAA and FCC rules) are a procurement risk. Skydio is the compliant alternative.

### 1f. GNSS RTK and IMUs
- **Emlid Reach RS3/RS4:** $3-4.6k each; base-plus-rover kit $6-8.9k. Accuracy ~7 mm + 1 ppm.
- **Robot-mounted RTK:** u-blox ZED-F9P (~$200-300) or Unicore UM980 (~$150-250).
- **IMUs:** the built-in Livox/Ouster IMUs are enough for LIO. Xsens MTi-680G costs ~$2-5k.

## 2. Persistent site sensors for a "live" twin
| Category | Examples | Price | Notes |
|---|---|---|---|
| Fixed / PTZ cameras (managed) | OxBlue, EarthCam, TrueLook | $2-20k hardware + $200-900/mo | Time-lapse and progress. |
| DIY IP cameras | Axis, Reolink with 4G and solar | $150-2,000 per camera; solar trailer $5-15k | Hikvision and Dahua are NDAA-restricted. |
| Crane cameras | Versatile, EarthCam | Quote only | Near-daily top-down views without a drone. |
| UWB / BLE tags | UbiTrack, Pozyx, Sewio, Redpoint | Tags $25-99; anchors $499-799 | UWB 10-30 cm; BLE 1-5 m. |
| Equipment telematics | ISO 15143-3 (AEMP 2.0): Cat, Komatsu, Volvo, Deere | Included with OEM subscription; retrofit $100-300 + $15-30/mo | Location, hours, fuel, idle time, fault codes. Updates every 10 minutes to 24 hours. |
| Concrete maturity | Giatec SmartRock | $135-250 per sensor | |
| Dust, noise, vibration | Aeroqual, Kunak; Svantek, Sigicom | $250 to $10k | Permit compliance. |
| Structural | Tiltmeters, crack meters, automated total stations | $300-1,500 per node; total station $60k+ | Excavations and neighbouring buildings. |
| Connectivity | LoRaWAN gateway $300-1,500; private 5G $20-100k+ **(?)**; Starlink $1,999 + ~$250-290/mo | | Scan uploads need Starlink, 5G or carrying an SSD off site. |

## 3. Capture platforms
| Platform | Cost | Strengths and weaknesses |
|---|---|---|
| Handheld / pole | $1-60k | Cheap and flexible, but labor-heavy. |
| Backpack / helmet | $5-170k | Fast coverage of floors. |
| **Wheeled AMR (the opening for Flo Mobility)** | $10-40k custom | Repeatable routes, cheap per capture. Struggles with stairs, rubble and mud. |
| Legged (Spot $150-375k; Unitree B2 ~$77-106k; Go2 EDU ~$10-15k) | | Stairs and rough ground. Spot + BLK ARC is the reference integration. |
| Drone | $5-30k | Exteriors and earthworks. No interiors once the building is enclosed. |
| Vehicle mobile mapping | $30k-500k | Roads and large sites. |

## 4. Compute and storage
### Edge
| Module | Price | What it can run |
|---|---|---|
| Jetson Orin Nano Super | $249 | FAST-LIO2 + light YOLO + logging |
| Jetson Orin NX 16 GB | ~$999 module | LIO + stereo + 2-4 camera detection + nvblox |
| Jetson AGX Orin 64 GB | $3,499 | Several LiDARs + 4-6 cameras + on-board change detection |
| Jetson AGX Thor | $5,499 | VLMs on-device **(?)** |

NVIDIA raised Jetson prices in 2026.

Data rates: a rosbag with LiDAR + IMU + 2 compressed cameras is ~0.5-2 GB per minute, so plan 2-4 TB of NVMe on the robot.

### Reconstruction
- **Photogrammetry:** 1-3k images need 64-128 GB RAM and a 24 GB GPU, taking 2-8 hours.
- **Gaussian splatting:** a medium scene needs ~24 GB VRAM; a large scene 40-80 GB. An RTX 4090 trains 30k iterations in ~30-40 minutes. [CLM-GS](https://github.com/nyu-systems/CLM-GS) fits 100M+ Gaussians on a 24 GB GPU.
- **Recommended:** a workstation with an RTX 5090 or RTX 6000-class GPU and 128-256 GB RAM ($6-15k), or cloud A100/H100 at ~$1.5-4 per GPU-hour.

### Storage
A weekly capture of a medium site is ~50-200 GB raw, or about **3-10 TB per project-year**. Serve it through Potree or 3D Tiles.

## 5. Recommended bundles by tier
USIBD accuracy bands: **LOA10** worse than 5 cm, **LOA20** 15-50 mm, **LOA30** 5-15 mm, **LOA40** 1-5 mm. Progress tracking needs about LOA10-20 (2-5 cm). QA against BIM needs about LOA30 (1-2 cm or better).

| Tier | Bundle | Cost | Accuracy | Good for |
|---|---|---|---|---|
| (a) MVP | Mid-360 or Unitree L2 + Insta360 X5 + F9P/UM980 RTK + Jetson Orin NX/Nano + SSD, on a pole **or a Flo robot** | $2.5-5k | 1.5-3 cm relative; 2-5 cm absolute with RTK and control points | Progress tracking, visual twin, navigation maps. Not QA. |
| (b) Mid | Hesai JT128 or Ouster OS0 + ZED X + 360° camera + Emlid RS4 + AGX Orin; plus an M4E drone; GPU workstation | $15-30k | 1-2 cm relative, 2-3 cm absolute | Weekly progress, approximate deviation checks, earthwork volumes |
| (c) Survey | RTC360 or X7 + BLK2GO/VLX/Hovermap + M400 + L3 + Spot + BLK ARC | $80-400k+ | 2-6 mm | QA/QC against BIM, contractual as-builts |

### Suggested path for Flo Mobility
1. Build tier (b) on your own AMR, for repeatable automated captures.
2. Calibrate against a rented TLS (~$500-1,000/day) so you can report LOA bands credibly.
3. Add the live layer: cameras, UWB, telematics, maturity sensors, LoRa.
4. Reach QA through partnerships or importing data from survey scanners rather than building it.
