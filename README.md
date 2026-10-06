# Robust Camera–LiDAR Fusion for Real-Time 3D Multi-Object Tracking

A ROS 2-based multi-sensor perception system for **3D object detection, camera–LiDAR fusion, multi-object tracking, ego-motion compensation, and dynamic obstacle mapping** for autonomous robots and ADAS applications.

The project combines camera-based semantic perception with LiDAR-based geometric perception to estimate the **3D position, velocity, identity, and confidence** of dynamic objects in real time.

---

## 1. Project Overview

Autonomous systems need to answer three fundamental questions:

1. **What objects are around me?**
2. **Where are they in 3D space?**
3. **How are they moving over time?**

A camera provides rich semantic information:

- Object class
- Appearance
- Color
- Texture
- High-resolution 2D localization

LiDAR provides geometric information:

- 3D position
- Depth
- Spatial structure
- Object geometry
- Robust distance estimation

Neither sensor is sufficient by itself for reliable 3D tracking in all conditions.

This project therefore implements:

```text
                  Camera
                    |
                    v
              2D Detection
                    |
                    |
                    v
             +-------------+
             |             |
             |   Sensor    |
             |   Fusion    |
             |             |
             +-------------+
                    ^
                    |
                    |
             3D LiDAR Objects
                    ^
                    |
                 LiDAR
```

The fused detections are passed to a multi-object tracker that maintains persistent object identities and estimates object motion.

---

# 2. Project Goals

The primary objective is to develop a **real-time camera–LiDAR 3D multi-object tracking pipeline**.

### Core goals

- Detect objects using a camera
- Extract 3D objects from LiDAR point clouds
- Calibrate camera and LiDAR coordinate systems
- Project LiDAR points into the camera image
- Associate camera and LiDAR detections
- Fuse semantic and geometric information
- Track multiple objects simultaneously
- Estimate object position and velocity
- Maintain persistent object identities
- Handle missed detections and temporary occlusions
- Compensate for ego-vehicle motion
- Publish tracked objects through ROS 2
- Generate a dynamic obstacle representation
- Deploy the perception pipeline on NVIDIA Jetson Orin
- Measure accuracy, latency, FPS, and resource utilization

---

# 3. Target Applications

The system is designed around applications such as:

### ADAS

- Vehicle tracking
- Pedestrian tracking
- Cyclist tracking
- Collision-risk estimation
- Dynamic obstacle detection
- Surround-view perception

### Autonomous Mobile Robots

- Dynamic obstacle tracking
- Human tracking
- Navigation around moving objects
- Dynamic costmap generation

### Autonomous Vehicles

- 3D object perception
- Multi-object tracking
- Motion prediction
- Sensor fusion

---

# 4. System Architecture

```text
                           CAMERA
                             |
                             v
                    +----------------+
                    | Image Decoder  |
                    +-------+--------+
                            |
                            v
                    +----------------+
                    | Preprocessing  |
                    +-------+--------+
                            |
                            v
                    +----------------+
                    | YOLO Detector  |
                    +-------+--------+
                            |
                            v
                    2D Bounding Boxes
                            |
                            |
                            v
                    +----------------+
                    | Camera-LiDAR   |
                    | Association    |
                    +-------+--------+
                            ^
                            |
                            |
                    +-------+--------+
                    | LiDAR Pipeline |
                    +-------+--------+
                            ^
                            |
                         PointCloud
                            |
                            v
                    +----------------+
                    | Ground Removal |
                    +-------+--------+
                            |
                            v
                    +----------------+
                    | Point Clustering|
                    +-------+--------+
                            |
                            v
                      3D Detections
                            |
                            |
                            v
                  +---------------------+
                  | Data Association     |
                  | Hungarian Algorithm   |
                  +----------+----------+
                             |
                             v
                  +---------------------+
                  | Kalman Filter        |
                  | State Estimation      |
                  +----------+----------+
                             |
                             v
                  +---------------------+
                  | Track Management     |
                  | Tentative/Confirmed  |
                  | Lost/Deleted         |
                  +----------+----------+
                             |
                             v
                  +---------------------+
                  | Ego Motion           |
                  | Compensation         |
                  +----------+----------+
                             |
                             v
                  +---------------------+
                  | 3D Object Tracks     |
                  +----------+----------+
                             |
                +------------+-------------+
                |                          |
                v                          v
        RViz Visualization         Dynamic Obstacle Map
                                           |
                                           v
                                         Nav2
```

---

# 5. Sensor Inputs

## 5.1 Camera

The camera provides:

```text
RGB Image
CameraInfo
Timestamp
```

Example ROS 2 topics:

```text
/camera/image_raw
/camera/camera_info
```

or:

```text
/camera/image_rect_color
/camera/camera_info
```

The camera is responsible primarily for:

- Semantic classification
- 2D object localization
- Appearance information

---

# 6. LiDAR Input

The LiDAR provides:

```text
sensor_msgs/msg/PointCloud2
```

Example:

```text
/lidar/points
```

Each point contains:

```text
x
y
z
intensity
```

depending on the LiDAR sensor.

LiDAR is responsible primarily for:

- 3D geometry
- Depth
- Spatial localization
- Object shape estimation

---

# 7. Camera Detection

The initial detector will use a YOLO-based object detector.

Possible models:

```text
YOLOv8
YOLO11
YOLO11-Seg
```

The detector produces:

```text
class_id
confidence
x_min
y_min
x_max
y_max
```

Example:

```text
Car
Confidence: 0.92
Bounding box:
(640, 280)
(850, 490)
```

The detection can be represented as:

```text
D_camera = {
    class,
    confidence,
    bbox,
    timestamp
}
```

---

# 8. LiDAR Processing Pipeline

Raw point clouds contain:

- Ground
- Road
- Vegetation
- Buildings
- Dynamic objects
- Sensor noise

Therefore, the LiDAR pipeline is:

```text
Raw PointCloud
      |
      v
ROI Filtering
      |
      v
Ground Removal
      |
      v
Noise Filtering
      |
      v
Clustering
      |
      v
3D Object Candidates
```

---

# 9. Region of Interest Filtering

Before clustering, remove irrelevant points.

Example:

```text
x: 0 m → 50 m
y: -20 m → 20 m
z: -2 m → 5 m
```

This significantly reduces processing cost.

Generic filter:

```cpp
if (x < X_MIN || x > X_MAX)
    reject;

if (y < Y_MIN || y > Y_MAX)
    reject;

if (z < Z_MIN || z > Z_MAX)
    reject;
```

---

# 10. Ground Removal

Ground points should be separated from object points.

Possible methods:

### Method 1 — Height threshold

Simple environments:

```text
z > ground_height + threshold
```

### Method 2 — RANSAC plane fitting

Fit:

\[
ax + by + cz + d = 0
\]

For each point:

\[
distance =
\frac{|ax+by+cz+d|}
{\sqrt{a^2+b^2+c^2}}
\]

Points below the threshold are classified as ground.

### Future improvement

Use:

- Progressive Morphological Filter
- Elevation map
- GridMap-based ground estimation

---

# 11. Point Cloud Clustering

After removing the ground, cluster remaining points.

Possible approaches:

- Euclidean clustering
- DBSCAN
- HDBSCAN
- Voxel-based clustering

Initial implementation:

```text
PointCloud
     |
     v
Voxel Downsampling
     |
     v
Euclidean / DBSCAN
     |
     v
Clusters
```

Each cluster becomes a potential 3D object.

---

# 12. 3D Object Representation

For each LiDAR cluster:

```text
Object:
    centroid
    bounding_box
    dimensions
    point_count
    confidence
```

Example:

```text
Object ID: Candidate 4

Centroid:
x = 18.4 m
y = -2.1 m
z = 0.8 m

Dimensions:
length = 4.3 m
width  = 1.9 m
height = 1.6 m

Points:
247
```

---

# 13. Camera–LiDAR Calibration

Camera and LiDAR operate in different coordinate systems.

Example:

```text
LiDAR frame
    |
    | Extrinsic Transform
    v
Camera frame
```

The transformation is:

\[
P_c = RP_l+t
\]

where:

- \(P_l\) = LiDAR point
- \(R\) = rotation matrix
- \(t\) = translation vector
- \(P_c\) = camera-frame point

The homogeneous representation is:

\[
\begin{bmatrix}
X_c\\
Y_c\\
Z_c\\
1
\end{bmatrix}
=
\begin{bmatrix}
R&t\\
0&1
\end{bmatrix}
\begin{bmatrix}
X_l\\
Y_l\\
Z_l\\
1
\end{bmatrix}
\]

---

# 14. Camera Projection

After transforming a LiDAR point into the camera frame:

\[
u=f_x\frac{X_c}{Z_c}+c_x
\]

\[
v=f_y\frac{Y_c}{Z_c}+c_y
\]

where:

- \(f_x,f_y\) = focal lengths
- \(c_x,c_y\) = principal point
- \(u,v\) = image coordinates

The complete projection can be expressed as:

\[
p = K[R|t]P_l
\]

where:

\[
K =
\begin{bmatrix}
f_x & 0 & c_x\\
0 & f_y & c_y\\
0 & 0 & 1
\end{bmatrix}
\]

---

# 15. LiDAR-to-Camera Association

Projected LiDAR points are tested against camera detections.

```text
LiDAR Point
     |
     v
Transform to Camera Frame
     |
     v
Project to Image
     |
     v
Inside Bounding Box?
     |
   +---+---+
   |       |
  YES      NO
   |       |
   v       X
Associate
```

For each camera bounding box:

```text
x_min < u < x_max
y_min < v < y_max
```

If enough LiDAR points fall inside the bounding box, the corresponding LiDAR cluster becomes a candidate match.

---

# 16. Robust Association

Simple bounding-box inclusion is only the first version.

The final association cost will combine several terms:

\[
C_{ij}
=
w_b C_{bbox}
+
w_d C_{distance}
+
w_z C_{depth}
+
w_c C_{class}
\]

where:

### Bounding-box cost

\[
C_{bbox}=1-IoU
\]

### Distance cost

\[
C_{distance}
=
\|p_{pred}-p_{lidar}\|
\]

### Depth cost

\[
C_{depth}
=
|Z_{camera}-Z_{lidar}|
\]

### Class cost

Example:

```text
Camera: car
LiDAR: vehicle
```

Class-compatible objects receive a lower cost.

---

# 17. Hungarian Algorithm

The association problem is represented as a cost matrix:

```text
                LiDAR
             L1   L2   L3
Camera C1    .2   .8   .7
Camera C2    .9   .1   .6
Camera C3    .7   .6   .2
```

The Hungarian algorithm finds the globally optimal assignment.

Example:

```text
C1 → L1
C2 → L2
C3 → L3
```

This prevents greedy matching errors.

---

# 18. Multi-Object Tracking

Each fused object is assigned a persistent track ID.

Example:

```text
Frame 100:
Car → ID 7

Frame 101:
Car → ID 7

Frame 102:
Car → ID 7

Frame 103:
Car → ID 7
```

Even though the detector produces a new bounding box every frame, the tracker maintains identity.

---

# 19. Kalman Filter

The initial tracker will use a Kalman Filter.

A possible state vector is:

\[
x =
\begin{bmatrix}
x & y & z & v_x & v_y & v_z
\end{bmatrix}^{T}
\]

The motion model is:

\[
x_k = Fx_{k-1}+w_k
\]

with:

\[
F=
\begin{bmatrix}
1&0&0&\Delta t&0&0\\
0&1&0&0&\Delta t&0\\
0&0&1&0&0&\Delta t\\
0&0&0&1&0&0\\
0&0&0&0&1&0\\
0&0&0&0&0&1
\end{bmatrix}
\]

---

# 20. Kalman Prediction

Prediction:

\[
\hat{x}_{k|k-1}
=
F\hat{x}_{k-1|k-1}
\]

Covariance:

\[
P_{k|k-1}
=
FP_{k-1|k-1}F^T+Q
\]

---

# 21. Kalman Update

Measurement:

\[
z_k=Hx_k+v_k
\]

Innovation:

\[
y_k=z_k-H\hat{x}_{k|k-1}
\]

Innovation covariance:

\[
S_k=HPH^T+R
\]

Kalman gain:

\[
K_k=PH^TS^{-1}
\]

State update:

\[
x_k=x_{k|k-1}+K_ky_k
\]

Covariance update:

\[
P_k=(I-KH)P
\]

---

# 22. Track Management

Each track has a lifecycle.

```text
               Detection
                  |
                  v
             TENTATIVE
                  |
            N consecutive
              matches
                  |
                  v
             CONFIRMED
                  |
            Missed frames
                  |
                  v
                LOST
                  |
          Recovery window
          /            \
       Match          No Match
         |                |
         v                v
     CONFIRMED         DELETED
```

Example parameters:

```yaml
min_hits: 3
max_age: 5
max_distance: 3.0
```

These parameters will be experimentally tuned.

---

# 23. Ego-Motion Compensation

A moving robot or vehicle causes apparent object motion even when an object is stationary.

Therefore:

```text
Observed motion
      =
Object motion
      +
Ego motion
```

The system will use:

- IMU
- Wheel odometry
- LiDAR odometry
- Robot localization

to estimate ego motion.

---

# 24. Coordinate Frames

The ROS TF tree will follow:

```text
map
 |
 +-- odom
      |
      +-- base_link
             |
             +-- camera_link
             |
             +-- lidar_link
```

Sensor measurements will be transformed using TF2.

Example:

```bash
ros2 run tf2_ros tf2_echo base_link lidar_link
```

---

# 25. Ego-Motion Compensation Concept

Suppose an object was detected at:

```text
t0:
P0 = [20, -3, 0]
```

At:

```text
t1:
P1 = [19, -3, 0]
```

A naive tracker might conclude:

```text
object velocity = -1 m/s
```

But if the vehicle itself moved forward by 1 m:

```text
ego motion = +1 m
```

the object may actually be stationary.

Therefore, transform historical tracks into the current reference frame before calculating object velocity.

---

# 26. Object Track Representation

Each track will contain:

```cpp
struct Track
{
    int id;

    ObjectClass class_id;

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;

    Eigen::Vector3d dimensions;

    double confidence;

    int age;
    int hits;
    int misses;

    TrackState state;

    rclcpp::Time last_update;
};
```

---

# 27. Track Output

Each tracked object should provide:

```text
Track ID
Class
Position
Velocity
Dimensions
Confidence
Age
Track State
Timestamp
```

Example:

```text
Track 17

Class: CAR
Position: [18.2, -2.4, 0.9] m
Velocity: [4.1, 0.2, 0.0] m/s
Dimensions: [4.3, 1.9, 1.6] m

Confidence: 0.94
Age: 82 frames
Misses: 0
State: CONFIRMED
```

---

# 28. ROS 2 Architecture

The project will be implemented as modular ROS 2 nodes.

```text
camera_detector_node
        |
        v
camera_detections
        |
        +----------------+
                         |
                         v
lidar_processing_node → fusion_node
                         |
                         v
                    tracker_node
                         |
                         v
                 tracked_objects
                         |
              +----------+----------+
              |                     |
              v                     v
        visualization_node    dynamic_map_node
                                    |
                                    v
                                   Nav2
```

---

# 29. Proposed ROS 2 Nodes

## `camera_detector_node`

Responsibilities:

- Subscribe to image
- Run YOLO
- Publish detections

Input:

```text
/camera/image_raw
```

Output:

```text
/camera/detections
```

---

## `lidar_processor_node`

Responsibilities:

- Point cloud filtering
- Ground removal
- Clustering
- 3D object generation

Input:

```text
/lidar/points
```

Output:

```text
/lidar/objects
```

---

## `fusion_node`

Responsibilities:

- Camera–LiDAR calibration
- Projection
- Association
- Sensor fusion

Input:

```text
/camera/detections
/lidar/objects
/tf
```

Output:

```text
/fused/objects
```

---

## `tracker_node`

Responsibilities:

- Track prediction
- Data association
- Kalman update
- Track lifecycle
- Velocity estimation

Input:

```text
/fused/objects
```

Output:

```text
/tracked_objects
```

---

## `dynamic_map_node`

Responsibilities:

- Convert tracked objects into dynamic obstacle representation
- Publish obstacle information to navigation

Output:

```text
/dynamic_obstacles
```

Potential future integration:

```text
Nav2 costmap
```

---

# 30. Example ROS 2 Topic Architecture

```text
/camera/image_raw
/camera/camera_info

/lidar/points

/camera/detections
/lidar/objects

/fused/objects

/tracked_objects

/dynamic_obstacles

/tf
/tf_static

/diagnostics
```

---

# 31. Visualization

RViz2 will be used to visualize:

### Camera

- Bounding boxes
- Object labels
- Confidence

### LiDAR

- Raw point cloud
- Ground points
- Object clusters

### Fusion

- Projected LiDAR points
- Camera detections
- 3D object positions

### Tracking

- Track IDs
- Velocity vectors
- 3D bounding boxes
- Track trajectories

Example:

```text
       Camera Image

     +-------------+
     |             |
     |   CAR #17   |
     |  +-------+  |
     |  |       |  |
     |  +-------+  |
     |             |
     +-------------+
          |
          | projection
          v

       LiDAR View

        #17
       /----\
      /      \
     |  CAR   | ----> velocity
      \      /
       \----/
```

---

# 32. Dynamic Obstacle Representation

Tracked objects can be converted into a dynamic obstacle layer.

For each object:

```text
position
velocity
dimensions
prediction horizon
```

The future position can be estimated:

\[
p(t+\Delta t)
=
p(t)+v(t)\Delta t
\]

For a constant velocity model.

This can be used to estimate where a moving obstacle will be.

---

# 33. Motion Prediction

Initial model:

### Constant Velocity Model

\[
x_{t+\Delta t}
=
x_t+v_x\Delta t
\]

\[
y_{t+\Delta t}
=
y_t+v_y\Delta t
\]

Future versions may use:

- Constant acceleration
- IMM filter
- LSTM
- Transformer-based trajectory prediction

Deep learning prediction is intentionally kept as a future extension rather than the starting point.

---

# 34. Robustness to Sensor Failures

A major goal is to make the system robust when one sensor becomes unreliable.

### Camera failure

```text
Camera unavailable
       |
       v
LiDAR tracking continues
       |
       v
Geometry-only tracking
```

### LiDAR failure

```text
LiDAR unavailable
       |
       v
Camera tracking continues
       |
       v
2D tracking
```

### Temporary detection failure

```text
Detection missing
       |
       v
Kalman prediction
       |
       v
Track maintained
```

This will allow quantitative testing of sensor degradation.

---

# 35. Weather Robustness

The project can later be tested under:

```text
Clear
Night
Rain
Fog
Low-light
Partial occlusion
```

Camera and LiDAR respond differently to environmental conditions.

The goal is to evaluate whether fusion improves tracking robustness.

---

# 36. Evaluation Metrics

A major objective is to make the project measurable.

## Detection Metrics

```text
Precision
Recall
mAP@50
mAP@50:95
```

---

# 37. Tracking Metrics

### MOTA

\[
MOTA =
1-
\frac{FN+FP+IDS}{GT}
\]

where:

- FN = false negatives
- FP = false positives
- IDS = identity switches
- GT = ground truth objects

---

### MOTP

Measures localization precision.

---

### IDF1

Measures identity preservation.

---

### ID Switches

Number of times a tracked object changes identity.

Target:

```text
Lower ID switches = better
```

---

### HOTA

HOTA evaluates both:

- Detection
- Association

This is useful for evaluating modern multi-object tracking systems.

---

# 38. System Performance Metrics

In addition to accuracy:

```text
End-to-end latency
Camera FPS
LiDAR processing FPS
Tracking FPS
GPU utilization
CPU utilization
Memory usage
Power consumption
```

Example performance table:

| Module | Target |
|---|---:|
| Camera preprocessing | < 5 ms |
| YOLO inference | < 20 ms |
| LiDAR processing | < 15 ms |
| Fusion | < 5 ms |
| Tracking | < 2 ms |
| Total | < 50 ms |
| Target FPS | > 20 FPS |

These are initial engineering targets and will be replaced by measured values.

---

# 39. Embedded Deployment

The final system should be deployed on:

## NVIDIA Jetson Orin

Deployment stack:

```text
ROS 2
   |
CUDA
   |
TensorRT
   |
YOLO
   |
Camera-LiDAR Fusion
   |
Tracker
```

The detector should be exported:

```text
PyTorch
   ↓
ONNX
   ↓
TensorRT
   ↓
FP16 Engine
```

Example:

```bash
trtexec \
    --onnx=model.onnx \
    --saveEngine=model_fp16.engine \
    --fp16
```

Actual TensorRT parameters will depend on the target model and Jetson environment.

---

# 40. Software Stack

## Operating System

```text
Ubuntu 22.04
```

## Robotics

```text
ROS 2 Humble
```

## Programming

```text
C++
Python
```

## Computer Vision

```text
OpenCV
YOLO
```

## Point Cloud

Possible libraries:

```text
PCL
Open3D
```

## Mathematics

```text
Eigen
```

## GPU

```text
CUDA
TensorRT
```

## Visualization

```text
RViz2
```

---

# 41. Repository Structure

Proposed repository:

```text
camera_lidar_tracking/
│
├── README.md
│
├── docs/
│   ├── architecture.md
│   ├── calibration.md
│   ├── tracking.md
│   ├── sensor_fusion.md
│   └── evaluation.md
│
├── ros2_ws/
│   └── src/
│       │
│       ├── camera_detector/
│       │
│       ├── lidar_processor/
│       │
│       ├── sensor_fusion/
│       │
│       ├── multi_object_tracker/
│       │
│       ├── dynamic_obstacle_mapper/
│       │
│       └── tracking_msgs/
│
├── models/
│   ├── yolo.onnx
│   └── yolo_fp16.engine
│
├── config/
│   ├── camera.yaml
│   ├── lidar.yaml
│   ├── fusion.yaml
│   └── tracker.yaml
│
├── launch/
│   ├── perception.launch.py
│   └── tracking.launch.py
│
├── scripts/
│   ├── evaluate_tracking.py
│   ├── calculate_metrics.py
│   └── benchmark.py
│
├── datasets/
│   └── README.md
│
├── docker/
│   └── Dockerfile
│
├── results/
│   ├── metrics/
│   ├── plots/
│   └── videos/
│
└── LICENSE
```

---

# 42. Custom ROS 2 Message

A custom message can represent a tracked object.

Example:

```text
TrackedObject.msg
```

```text
int32 id
string class_name

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 dimensions

float32 confidence

uint8 state

builtin_interfaces/Time timestamp
```

A collection message:

```text
TrackedObjectArray.msg
```

```text
std_msgs/Header header
TrackedObject[] objects
```

---

# 43. Configuration

Example:

```yaml
tracker:
  max_match_distance: 3.0
  max_age: 5
  min_hits: 3

  process_noise:
    position: 0.1
    velocity: 0.5

  measurement_noise:
    position: 0.2

fusion:
  max_depth_difference: 2.0
  min_lidar_points: 5
  association_threshold: 0.6
```

All important parameters should be configurable rather than hard-coded.

---

# 44. Dataset Strategy

The project should be evaluated using datasets containing synchronized camera and LiDAR data.

Possible datasets:

- KITTI
- nuScenes
- Waymo Open Dataset
- Argoverse
- PandaSet

The initial implementation should use a dataset that provides:

```text
Camera
+
LiDAR
+
Calibration
+
Timestamps
+
Object annotations
```

KITTI can be used for the initial proof of concept, while nuScenes or another larger dataset can be used for more comprehensive evaluation.

---

# 45. Dataset Synchronization

Sensor timestamps are critical.

Example:

```text
Camera:
t = 10.024 s

LiDAR:
t = 10.019 s
```

The system must either:

- synchronize using approximate time synchronization, or
- compensate for timestamp differences.

For ROS 2:

```text
message_filters
```

can be used.

---

# 46. Synchronization Pipeline

```text
Camera Frame
     |
     | timestamp
     |
     +------------------+
                        |
                        v
                 Approximate Time
                    Synchronizer
                        ^
                        |
     +------------------+
     |
LiDAR PointCloud
```

---

# 47. Initial Development Roadmap

## Phase 1 — Environment

```text
[ ] Ubuntu 22.04
[ ] ROS 2 Humble
[ ] OpenCV
[ ] PCL
[ ] Eigen
[ ] YOLO
[ ] RViz2
```

---

## Phase 2 — LiDAR

```text
[ ] Read PointCloud2
[ ] ROI filtering
[ ] Ground removal
[ ] Voxel filtering
[ ] Clustering
[ ] 3D object extraction
[ ] RViz visualization
```

---

## Phase 3 — Camera

```text
[ ] Read image
[ ] Run YOLO
[ ] Publish detections
[ ] Visualize bounding boxes
[ ] Measure inference latency
```

---

## Phase 4 — Calibration

```text
[ ] Camera intrinsics
[ ] LiDAR-camera extrinsics
[ ] TF tree
[ ] LiDAR projection
[ ] Projection visualization
```

---

## Phase 5 — Fusion

```text
[ ] Associate LiDAR points with camera boxes
[ ] Estimate object depth
[ ] Build fused detections
[ ] Implement association cost
[ ] Implement Hungarian algorithm
```

---

## Phase 6 — Tracking

```text
[ ] Kalman Filter
[ ] Track creation
[ ] Track confirmation
[ ] Track prediction
[ ] Track deletion
[ ] ID management
[ ] Velocity estimation
```

---

## Phase 7 — Ego Motion

```text
[ ] Integrate TF
[ ] Integrate odometry
[ ] Transform historical tracks
[ ] Compensate ego motion
[ ] Evaluate stationary objects
```

---

## Phase 8 — Robustness

```text
[ ] Occlusion handling
[ ] Missed detections
[ ] Camera-only fallback
[ ] LiDAR-only fallback
[ ] Sensor degradation tests
```

---

## Phase 9 — Embedded Deployment

```text
[ ] ONNX export
[ ] TensorRT conversion
[ ] FP16 optimization
[ ] Jetson deployment
[ ] GPU profiling
[ ] End-to-end benchmarking
```

---

## Phase 10 — Dynamic Navigation

```text
[ ] Dynamic obstacle representation
[ ] Publish tracked objects
[ ] Generate dynamic obstacle layer
[ ] Nav2 integration
[ ] Test moving obstacle avoidance
```

---

# 48. Experimental Plan

The project should not be evaluated only on one sequence.

Run controlled experiments.

### Experiment 1

Camera only:

```text
YOLO + 2D tracking
```

### Experiment 2

LiDAR only:

```text
LiDAR detection + tracking
```

### Experiment 3

Camera + LiDAR:

```text
Fusion + tracking
```

### Experiment 4

Fusion + ego-motion compensation

### Experiment 5

Sensor degradation

```text
Camera dropout
LiDAR dropout
```

### Experiment 6

Environmental degradation

```text
Night
Rain
Fog
```

---

# 49. Comparison Table

Final results should contain a table similar to:

| Method | MOTA | IDF1 | HOTA | ID Switches | FPS |
|---|---:|---:|---:|---:|---:|
| Camera only | -- | -- | -- | -- | -- |
| LiDAR only | -- | -- | -- | -- | -- |
| Camera + LiDAR | -- | -- | -- | -- | -- |
| Fusion + Ego Motion | -- | -- | -- | -- | -- |

The actual values will be measured experimentally.

---

# 50. Ablation Study

An ablation study will determine which components actually improve the system.

Example:

```text
Baseline
    ↓
+ LiDAR
    ↓
+ Calibration
    ↓
+ Sensor Fusion
    ↓
+ Hungarian Association
    ↓
+ Kalman Filter
    ↓
+ Ego Motion
    ↓
+ Robustness
```

Measure:

```text
MOTA
IDF1
HOTA
ID switches
Latency
FPS
```

This provides evidence that each engineering component contributes to performance.

---

# 51. Failure Cases

The project should explicitly document failure cases.

Examples:

### Sparse LiDAR

```text
Far object
    ↓
Very few points
    ↓
Poor 3D localization
```

### Heavy occlusion

```text
Object partially hidden
    ↓
Detection confidence drops
```

### Camera false detection

```text
Camera detects object
    ↓
No corresponding LiDAR geometry
    ↓
Association rejected
```

### LiDAR false cluster

```text
LiDAR cluster
    ↓
No camera detection
    ↓
Candidate rejected or maintained as geometry-only track
```

---

# 52. Confidence Estimation

The system should maintain multiple confidence values.

Example:

```text
Camera confidence       = 0.91
LiDAR confidence        = 0.84
Association confidence  = 0.88
Track confidence        = 0.94
```

A fused confidence can be represented as:

\[
C_{fused}
=
w_cC_c+w_lC_l+w_aC_a
\]

where:

\[
w_c+w_l+w_a=1
\]

The exact formulation should be experimentally validated.

---

# 53. Sensor Failure Strategy

The tracker should degrade gracefully.

### Normal operation

```text
Camera + LiDAR
       ↓
Full 3D tracking
```

### Camera unavailable

```text
LiDAR
  ↓
3D tracking
```

### LiDAR unavailable

```text
Camera
  ↓
2D tracking
```

### Both temporarily unavailable

```text
Prediction
  ↓
Short-term track maintenance
```

This is important for production-style perception systems.

---

# 54. Performance Optimization

Optimization will be performed progressively.

### CPU

- Point-cloud filtering
- Memory allocation reduction
- Multi-threading
- Efficient clustering

### GPU

- TensorRT inference
- CUDA preprocessing
- CUDA postprocessing where beneficial

### ROS 2

- QoS tuning
- Intra-process communication
- Component composition
- Zero-copy approaches where practical

### Memory

Avoid unnecessary:

```text
CPU → GPU → CPU
```

copies.

---

# 55. Target Performance

Initial targets:

```text
Camera:
30 FPS

Detection:
20–30 FPS

Tracking:
30+ FPS

End-to-end:
20+ FPS

Latency:
< 50 ms
```

Final values will depend on:

- Model
- Resolution
- LiDAR density
- Hardware
- Number of tracked objects

---

# 56. Engineering KPIs

The final project should report at least:

### Perception

```text
mAP
Precision
Recall
```

### Tracking

```text
MOTA
MOTP
IDF1
HOTA
ID switches
Fragmentation
```

### Localization

```text
3D position error
Velocity error
Depth error
```

### Runtime

```text
FPS
Latency
CPU %
GPU %
Memory
Power
```

---

# 57. Skills Demonstrated

This project is intentionally designed to demonstrate a broad perception engineering skill set.

## Computer Vision

- Object detection
- Image processing
- Camera calibration
- Projection geometry
- 2D tracking

## LiDAR Perception

- PointCloud2
- Point-cloud filtering
- Ground segmentation
- Clustering
- 3D localization

## Sensor Fusion

- Coordinate transforms
- Camera–LiDAR calibration
- Data association
- Multi-sensor confidence
- Sensor fallback

## Tracking

- Kalman Filter
- Motion models
- Hungarian algorithm
- Track lifecycle
- Identity management

## Robotics

- ROS 2
- TF2
- RViz2
- Nav2
- Dynamic obstacles

## Embedded AI

- ONNX
- TensorRT
- CUDA
- FP16
- Jetson Orin

## Engineering

- C++
- Python
- ROS 2 architecture
- Profiling
- Benchmarking
- Quantitative evaluation

---

# 58. Future Extensions

Possible future improvements:

### Advanced Tracking

- Extended Kalman Filter
- Unscented Kalman Filter
- IMM
- JPDA
- MHT

### Deep Learning Tracking

- DeepSORT
- OC-SORT
- ByteTrack
- CenterTrack
- BEV-based tracking

### Advanced Fusion

- Feature-level fusion
- BEV fusion
- Transformer-based fusion
- Learned association

### Prediction

- Constant acceleration model
- LSTM trajectory prediction
- Transformer trajectory prediction

### Perception

- 3D object detection
- PointPillars
- CenterPoint
- BEVFusion

---

# 59. Recommended Development Philosophy

The project will deliberately progress from classical methods to advanced methods.

```text
Classical Geometry
        ↓
Point Cloud Processing
        ↓
Camera Detection
        ↓
Sensor Calibration
        ↓
Data Association
        ↓
Kalman Tracking
        ↓
Ego Motion
        ↓
Robust Fusion
        ↓
TensorRT
        ↓
Jetson
        ↓
Advanced Tracking
```

The purpose is to understand **why each component exists**, rather than simply integrating pre-built frameworks.

---

# 60. Definition of Done

The project will be considered complete when the system can:

```text
[✓] Receive camera data
[✓] Receive LiDAR data
[✓] Detect objects
[✓] Generate 3D LiDAR objects
[✓] Transform between coordinate frames
[✓] Project LiDAR into camera
[✓] Associate camera and LiDAR objects
[✓] Generate fused detections
[✓] Track multiple objects
[✓] Maintain persistent IDs
[✓] Estimate velocity
[✓] Handle missed detections
[✓] Compensate ego motion
[✓] Visualize tracks in RViz
[✓] Publish ROS 2 tracked objects
[✓] Generate dynamic obstacles
[✓] Benchmark accuracy
[✓] Benchmark latency
[✓] Benchmark FPS
[✓] Deploy inference using TensorRT
[✓] Run on Jetson Orin
```

---

# 61. Final Project Outcome

The final system should demonstrate:

```text
                  SENSOR INPUT
                       |
          +------------+------------+
          |                         |
       CAMERA                     LiDAR
          |                         |
     YOLO Detection          Point Processing
          |                         |
          +------------+------------+
                       |
                 Sensor Fusion
                       |
                Data Association
                       |
                Kalman Tracking
                       |
              Ego Motion Compensation
                       |
                       v
             3D Multi-Object Tracks
                       |
          +------------+------------+
          |                         |
          v                         v
    Visualization            Dynamic Map
                                    |
                                    v
                                  Nav2
```

The final deliverable is not simply an object tracker.

It is a **real-time robotic perception pipeline capable of converting heterogeneous camera and LiDAR measurements into persistent, motion-aware 3D object tracks suitable for autonomous navigation and ADAS applications.**

---

# 62. Portfolio / Resume Positioning

Recommended project title:

> **Robust Camera–LiDAR Fusion for Real-Time 3D Multi-Object Tracking**

Short description:

> Developed a ROS 2-based camera–LiDAR perception pipeline combining YOLO-based 2D detection, LiDAR 3D clustering, geometric sensor calibration, Hungarian data association, Kalman-filter tracking, and ego-motion compensation for real-time multi-object tracking.

The final resume bullets should be based on measured results rather than predetermined claims.

Example structure:

```text
• Developed a ROS 2-based camera–LiDAR fusion pipeline for real-time
  3D multi-object tracking using YOLO detection, LiDAR clustering,
  geometric projection, and Hungarian data association.

• Implemented Kalman-filter-based 3D state estimation, track lifecycle
  management, and ego-motion compensation, achieving XX MOTA, XX IDF1,
  and XX HOTA on XX benchmark sequences.

• Optimized inference using ONNX/TensorRT FP16 and deployed the
  perception pipeline on NVIDIA Jetson Orin, achieving XX FPS at
  XX ms end-to-end latency.
```

---

# 63. Long-Term Goal

The long-term goal is to evolve this project from:

```text
Camera + LiDAR
      ↓
Object Tracking
```

into:

```text
Camera
   +
LiDAR
   +
IMU
   +
Odometry
   +
Semantic Perception
        ↓
Multi-Sensor 3D Perception
        ↓
Object Tracking
        ↓
Motion Prediction
        ↓
Dynamic Environment Model
        ↓
Navigation / Planning
```

This creates a direct bridge between **computer vision, LiDAR perception, sensor fusion, state estimation, autonomous navigation, and embedded AI deployment**.
