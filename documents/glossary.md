# Glossary

Every technical word used in this project, in one plain sentence each.

| Term | Meaning |
|---|---|
| **AGX Orin** | NVIDIA's small, powerful computer (Jetson AGX Orin 64 GB) that runs the stack on the rig. |
| **Airy** | The RoboSense LiDAR we use now: 96 laser beams, sees a dome from its horizon up. |
| **ATE (absolute trajectory error)** | How far the estimated path is from the true path, on average (only possible in simulation). |
| **Bag (rosbag2)** | A recording file of ROS messages, played back later as if live. |
| **Bridge (`dt_lidar_bridge`)** | Our node that converts either LiDAR's output into one common format. |
| **Bundle adjustment (BA, BALM, HBA)** | Adjusting many poses at once so that every view of the same surface agrees. |
| **camera_info** | Message with the camera's focal length, image centre and size. |
| **Canonical contract** | Our agreed format for points (`/dt/lidar/points`) and IMU (`/dt/lidar/imu`). |
| **Colcon** | The tool that builds the ROS 2 workspace. |
| **COLMAP format** | Folder of images + camera poses + points, used to train splats. |
| **CycloneDDS** | The message-transport layer ROS 2 uses here; set with large buffers for big LiDAR clouds. |
| **Deskew** | Correcting each point for the rig's motion during one LiDAR spin. |
| **DIFOP** | The Airy's status packet: device info, beam calibration and factory IMU calibration. |
| **Digital twin** | A measured 3D copy of a real place, at true scale. |
| **Drift** | Slowly growing position or heading error of the odometry. |
| **DUFOMap** | Method that removes moving objects (people) by checking which space is later seen through. |
| **Extrinsic** | The fixed position and angle of one sensor relative to another (e.g. camera vs LiDAR). |
| **FAST-LIO (FAST-LIO2)** | Fast LiDAR-inertial odometry: the live "where am I" engine. |
| **Frame (coordinate frame)** | A set of x, y, z axes attached to something (world, LiDAR, camera). |
| **Gaussian splat** | A photo-real 3D model made of millions of soft coloured blobs, trained from photos. |
| **GICP** | Generalized ICP: precise alignment of two point clouds using local surface shapes. |
| **GLIM** | A LiDAR-IMU mapping system with global optimisation; an optional back-end. |
| **Ground truth** | The exact true answer, known only in simulation. |
| **IMU** | Inertial measurement unit: measures rotation speed and acceleration, 200 times per second. |
| **Intensity** | How strongly a laser echo came back; depends on surface material. |
| **Keyframe** | A selected scan used as a node in the pose graph. |
| **LAS / LAZ** | Standard survey file format for point clouds; LAZ is compressed. |
| **Launch file** | A script that starts several ROS nodes with their settings at once. |
| **LiDAR** | Laser scanner that measures distance by timing laser echoes. |
| **LIO** | LiDAR-inertial odometry: odometry using both LiDAR and IMU. |
| **Loop closure** | Recognising a place visited before and using it to correct drift. |
| **Marching cubes** | Method that turns a TSDF grid into a triangle mesh. |
| **MCAP** | The file format of our bags (fast, indexed), compressed with zstd. |
| **Mesh** | A surface made of connected triangles. |
| **Mid-360** | The Livox LiDAR, also supported by the stack. |
| **Mixed pixel** | A false LiDAR point at an object edge, between the front and back surface. |
| **Node** | One running ROS 2 program. |
| **Odometry** | Estimating your own motion step by step from sensors. |
| **Operator zone (self-zone)** | Points within 0.8 m of the rig, removed because they are the person holding it. |
| **Optical frame** | Camera frame with z pointing forward, x right, y down (OpenCV convention). |
| **Photometric consistency** | The idea that a surface point should look the same colour from every view. |
| **Plane thickness** | Our sharpness measure: spread of points across a flat surface. |
| **Point cloud / point map** | Millions of measured 3D points. |
| **Pose** | Position plus orientation of something at one moment. |
| **Pose graph** | Poses (nodes) linked by measured relative motions (edges), solved together. |
| **PSNR** | Image similarity score in dB; higher means the rendered image matches the photo better. |
| **PTP** | Precision Time Protocol: keeps the LiDAR's clock synchronised to the computer, to microseconds. |
| **Quaternion** | Four numbers that describe a rotation without the problems of angles. |
| **Replay** | Feeding a recorded bag back through the stack to test other settings. |
| **Ring** | One of the LiDAR's laser beams; each ring scans one cone. |
| **ROS 2 (Humble)** | Robot Operating System: framework of nodes and topics; Humble is the version. |
| **RViz** | ROS's live 3D viewer. |
| **Scan** | One full LiDAR spin, 0.1 s, about 80,000 points. |
| **Session** | One recording folder: bag, SVO2, rig.yaml, meta.yaml, products. |
| **SLAM** | Simultaneous localisation and mapping: building a map while finding yourself in it. |
| **SVO2** | ZED's own video file: every camera frame with time stamps. |
| **TAI / UTC** | Atomic time vs normal civil time; TAI is 37 s ahead. |
| **TF / TF tree** | ROS's system that tracks how all frames sit relative to each other. |
| **Topic** | A named ROS message channel, e.g. `/dt/lidar/points`. |
| **Trajectory** | The path the rig took: pose over time (`traj_lidar.tum`). |
| **TSDF** | Truncated signed distance field: a grid where each cell stores its distance to the nearest surface; averaging many scans gives a smooth surface. |
| **TUM format** | Text format for trajectories: time, x, y, z, qx, qy, qz, qw. |
| **VDBFusion** | The library we use to build the TSDF. |
| **Voxel** | A small 3D cube; maps are stored as voxels (2 cm). |
| **ZED 2i** | Stereolabs stereo camera used for colour. |
| **zstd** | Fast compression used inside the bags. |
