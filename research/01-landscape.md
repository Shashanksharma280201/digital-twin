# Construction and Physical-Space Digital Twins: Landscape

*Research date: 2026-10-18. Vendor figures are self-reported. **[uncertain]** = conflicting or unverified.*

## 1. Executive summary
- **The market is consolidating fast.**
  - Hexagon bought Avvir and owns OxBlue.
  - DroneDeploy absorbed StructionSite (brand shut down Nov 2024).
  - OpenSpace acquired Disperse (Oct 2025).
  - AMETEK bought FARO/HoloBuilder (~$920M, Jul 2025).
  - CoStar bought Matterport ($1.6B, Feb 2025).
  - Bentley bought Cesium (Sep 2024).
- **The commercial core is progress tracking against BIM and the schedule.**
  - Capture is mostly a weekly 360° helmet walk, plus drones for exteriors and laser scanners for QA.
  - Buildots raised a $130M Series E (Sep 2026) at a valuation of nearly $1B.
- **Data centers are the leading vertical:** MEP-heavy work, tight schedules, and pay applications tied to installed work.
- **The frontier is autonomous capture plus AI agents.** Examples: DroneDeploy docked drones, Spot/Unitree robots and Capture Agent robotics-as-a-service (Jul 2026); OpenSpace agents; Bentley Copilot; NVIDIA Omniverse blueprints.
- **Twins are still periodic snapshots.** Live signals (cameras, crane hooks, wearables, telematics) sit in separate silos and are rarely fused into one spatial model.

## 2. Construction reality capture and progress tracking

### AI progress-tracking platforms
- **[OpenSpace](https://www.openspace.ai/press-releases/)**
  - Capture: 360° helmet camera with SLAM-like auto-placement on floor plans, plus Air (drones), scans and BIM+.
  - Analytics: Track (from the [Disperse acquisition](https://www.openspace.ai/news/openspace-acquires-construction-progress-tracking-firm-disperse/)) uses computer vision plus human verification and claims 95% accuracy. It connects to P6, Asta and MS Project. Waypoint 2026 added Autolocation 2.0, Site Mode, agents and a Track API.
  - Funding and pricing: ~$199M raised ($902M valuation in 2022); pricing starts around a $10k/yr minimum, scaled by volume.
- **Buildots**
  - Capture: helmet 360° walk 1-2x per week.
  - Analytics: element-level comparison against BIM and schedule across 80+ stages, plus delay forecasting. Needs clean BIM.
  - Funding and customers: $297M total and a valuation of nearly $1B ([Construction Dive](https://www.constructiondive.com/news/buildots-raises-130m-data-center-projects-surge/830222/)). Customers include Intel, Digital Realty, JE Dunn, Mortenson and HOCHTIEF.
- **[Doxel](https://doxel.ai/)**
  - Capture: started with a LiDAR robot/cart; now a 360° helmet camera ([uncertain] whether LiDAR is still offered).
  - Analytics: work-in-place per trade and system, 4D views. Focus on data centers.
  - ~$56.5M raised.
- **Track3D**
  - Capture: any 360° camera, plus scans.
  - Analytics: planned vs actual progress.
  - **Pricing: $0.15/sq ft for progress tracking, documentation free** ([PR](https://track3d.ai/press-releases/track3d-launches-new-commercial-model-for-progress-tracking/)). This is the clearest sign that capture is a commodity and analytics is the paid product.
  - $14.3M raised, 400+ projects.
- **[Cupix](https://www.cupix.com/)**
  - Capture: 360° video turned into a 3D map. VideoDrive captures from a vehicle at up to 40 km/h.
  - Analytics: SiteInsights for percent complete and BIM deviation.
- **Reconstruct**
  - Turns drone, 360° and phone footage into an as-built plan and 3D twin aligned with P6.
  - Example: a 600k sq ft project twinned in 2 days, versus 2+ months by scanning.
- **Avvir:** scan-vs-BIM checks; now part of Hexagon.

### Aerial and robotic capture
- **DroneDeploy**
  - More than 20 trillion sq ft of imagery captured.
  - 13k docked-drone missions and 16k ground-robot missions year-to-date by Apr 2026, on Spot and Unitree Go2-W.
  - **Capture Agent** (Jul 2026, early access): the robot navigates from drawings only, with no markers, and scans after hours. Sold as robotics-as-a-service.
  - Progress, Safety and Inspection AI agents.
  - ~$349-649/mo, then enterprise pricing ([Jul 2026 release](https://www.dronedeploy.com/blog/whats-new-in-dronedeploy-july-2026-quarterly-release)).
- **Propeller Aero:** drone surveying with AeroPoints smart GCPs; volumes and cut/fill. **DirtMate** links equipment telemetry to the terrain model. ~$12k per drone per year.
- **Skydio:** X10 + Dock for scheduled autonomous missions; 3D Scan for inspection-grade models.
- **Boston Dynamics Spot:** Orbit 5.0 fleet software; Leica BLK ARC for autonomous scanning.
- **Dusty Robotics:** layout printing from the model onto the slab (1/16"), 100M+ sq ft. **Civ Robotics:** layout robots.
- **Versatile CraneView:** a crane-hook sensor and camera tracking lifts and utilization (+28% claimed); $100M+ raised.

### Incumbent platforms
- **Hexagon/Leica:** BLK2GO, BLK ARC, RTC360, Reality Cloud, OxBlue cameras, Avvir.
- **Trimble:** X9 scanner, Reality Capture in Connect (accepts iPhone LiDAR), SiteVision AR.
- **Bentley:** iTwin (Capture now produces splats), SYNCHRO+ 4D with Copilot agents, Cesium / 3D Tiles.
- **Autodesk:** Construction Cloud, possibly renamed **Forma** [uncertain]. Tandem is the operations twin (BIM handover + IoT).
- **NVIDIA Omniverse:** Mega and DSX blueprints (AI-factory twins built before construction). A horizontal enabler, not a site product.
- **Unity/Unreal:** visualization runtimes used by integrators.

### Live-site video and safety
Intenseye (~$90M raised), Everguard (cameras + wearables + RTLS), Evercam (cameras + drone + BIM timeline), viAct. All are detection-only and not registered to a twin.

## 3. General-place twins
- **Matterport (CoStar):** 14M+ spaces; FY2024 revenue $169.7M.
- **NavVis:** VLX 4, CLX and the IVION platform; $85M Series D (Aug 2026) pitched as a "data foundation for physical AI"; 2B+ m² captured.
- **Niantic Spatial:** $250M spinout; Large Geospatial Model; Scaniverse produces splats and VPS maps; VPS 2.0 gives near-cm 6DoF localization.
- **Google and Apple:** Google Photorealistic 3D Tiles as a base layer; Apple RoomPlan and Object Capture.
- **Splat tools:** Polycam (room-scale); Luma, which has pivoted to world models ($900M raised). Splat standards are settling: KHR_gaussian_splatting and a native OpenUSD 26.03 schema.
- **Smart-city twins:** Virtual Singapore, Helsinki, Rotterdam. Slow to refresh and focused on planning.

## 4. Academic and open-source references
- **Scan-to-BIM:** BIMNet (IFC scan-to-BIM benchmark), CV4AEC Scan-to-BIM.
- **SLAM and scene benchmarks:** Hilti SLAM Challenge, ConSLAM, S3DIS.
- **Industrial:** Industrial3D (MEP point clouds).
- **Robot-ready site twins:** BIM2RDT.
- **Splat twins for simulation:** ArtiTwinSplat (Isaac Sim-ready twins).
- **Key gap:** [LiBiDT, Sci. Reports 2026](https://www.nature.com/articles/s41598-026-58063-3) notes that **no public time-stamped, multi-capture dataset of a site under active construction exists**.

## 5. Comparison table
| Company | Capture | Cadence | Analytics | Pricing | Funding |
|---|---|---|---|---|---|
| OpenSpace | 360° helmet, drone, scans | Weekly | Auto-placement, Track, agents | ~$10k/yr minimum | ~$199M |
| Buildots | 360° helmet, drone | 1-2x/week | Element-level BIM progress, delay forecasting | Enterprise | $297M, ~$1B valuation |
| Doxel | 360° (formerly LiDAR robot) | Weekly | Work in place, 4D | Premium | ~$56.5M |
| Track3D | Any 360° camera | ~6 h turnaround | Planned vs actual | $0.15/sq ft | $14.3M |
| Cupix | 360° video, scans, drone | Per capture | Percent complete, deviation | Quote | Private |
| Reconstruct | Drone, 360°, phone | Per capture | As-built + P6 | Quote | Strategic round |
| DroneDeploy | Drone, dock, Spot/Unitree | Autonomous, up to nightly | AI agents | $349-649/mo and up; RaaS | Private |
| Propeller | Drone + AeroPoints + machine sensors | Per flight | Volumes, equipment paths | ~$12k/drone/yr | ~$32M |
| Hexagon/Leica | BLK2GO/ARC, RTC360, OxBlue | Per scan | Registration, BIM checks | Hardware + cloud | Public |
| Bentley | iTwin Capture, Cesium | Per capture | 4D, agents | Licensing | Public |
| Autodesk Tandem | BIM + IoT | Live (operations) | Operations twin | Subscription | Public |
| NavVis | Mobile scanners | Per scan | IVION | Hardware + SaaS | $85M Series D |
| Niantic Spatial | Phone, 360°, drone | Crowd-sourced | VPS, LGM | API | $250M |
| Versatile | Crane hook | Live | Lifts, utilization | Subscription | $100M+ |

## 6. Business patterns
1. **Capture is a commodity; analytics is the product.** Verified progress tied to pay applications and the schedule is what customers pay for.
2. **Pricing is per square foot or by project volume**, sold to general contractors and increasingly to owners.
3. **Human-in-the-loop verification** makes up for weak computer vision.
4. **Integrations are table stakes** (Procore, ACC/Forma, P6, Aconex), and incumbents buy point solutions.
5. **"Physical AI data" pitch:** spatial datasets sold as training data for robots and world models.
6. **Hardware-agnostic vendors vs hardware incumbents** who lock customers into their cloud.

## 7. Gaps and opportunities
1. **Periodic, not live.** Opportunity: one continuously updated 4D twin that fuses fixed cameras, robots, telematics and worker location, with change detection between full captures.
2. **Autonomous capture is early and fragile.** Sites change daily and break robot routes. Opportunity: robots that navigate from drawings/BIM and update their own maps; cheap wheeled or quadruped platforms; robotics-as-a-service.
3. **MEP and hidden work gets covered by drywall before it is captured.** Opportunity: triggered capture before close-in.
4. **Trust and accuracy are the number one adoption barrier**, and vendor claims are unaudited. Opportunity: element-level confidence scores, provenance and an open benchmark.
5. **Dependence on clean BIM.** Opportunity: progress tracking driven by drawings or the schedule; automatic scan-to-BIM.
6. **Civil, solar, earthworks and mining are underserved.**
7. **Safety, equipment and worker data are not registered to the twin.**
8. **Construction-to-operations handover is disconnected.**
9. **Splats look real but carry no meaning or measurement guarantees.** Opportunity: semantic, measurable splats linked to BIM elements.

**Where a newcomer can differentiate:**
- (a) Near-real-time updates from fixed cameras and robots.
- (b) Autonomous robotic capture sold as a service.
- (c) Equipment and worker tracking fused into the same spatial model.
- (d) Verifiable BIM-vs-as-built deviation with confidence scores.
- (e) Underserved verticals and projects without BIM.
