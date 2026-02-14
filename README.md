# ACC UNIT

This repository contains two ROS 2 nodes developed for autonomous vehicle perception and longitudinal control:

1. **FrontObjectDetector** – Detects vehicles in front of the ego vehicle using radar tracks.  
2. **LongitudinalController** – Controls the vehicle's velocity along a recorded trajectory, allowing smooth acceleration and deceleration.  

---

## 1. FrontObjectDetector

### Overview
The **FrontObjectDetector** node subscribes to radar tracks from a front-facing radar sensor and identifies objects directly in front of the ego vehicle. It publishes both visualization markers and positional data for the detected objects.

### Subscriptions
- **`/radar_1/radar_tracks`** (`radar_msgs/msg/RadarTracks`)  
  Receives radar track information, including positions of detected objects.

### Publications
- **`/radar_1/front_track_marker`** (`visualization_msgs/msg/Marker`)  
  Publishes visual markers (spheres and distance labels) for objects detected in front of the vehicle.

- **`/radar_1/front_track_data`** (`geometry_msgs/msg/PoseArray`)  
  Publishes the position of the detected front objects in a PoseArray message for downstream processing.

### Parameters
- **`max_angle_deg`** (default: `5`)  
  Maximum lateral angle (in degrees) from the ego vehicle’s forward axis to consider an object as “in front.”
  
- 
- **`vehicle_width_half`** (default: `0.9`)  
  Half of ego vehicle width .”

### Key Features
- Filters radar tracks to detect only objects directly in front of the vehicle within a configurable angular range and vehicle width.  
- Publishes markers for RViz visualization, including distance labels.  
- Provides real-time front object positions for use in longitudinal control or collision avoidance modules.

---

## 2. LongitudinalController

### Overview
The **LongitudinalController** node manages the longitudinal (forward/backward) velocity of the vehicle along a pre-recorded trajectory. It reads a CSV file containing a trajectory and continuously updates the vehicle's velocity based on the current target velocity and vehicle state.

### Subscriptions
- **`/localization/kinematic_state`** (`nav_msgs/msg/Odometry`)  
  Provides the current vehicle state, including position and velocity.

- **`/acc/target_vel`** (`std_msgs/msg/Float64`)  
  Provides the target velocity for the vehicle.

### Publications
- **`/planning/scenario_planning/trajectory`** (`autoware_auto_planning_msgs/msg/Trajectory`)  
  Publishes an updated trajectory with interpolated velocities according to the target velocity and current vehicle state.

### Parameters
- **`look_ahead_distance`** (default: `5.0`)  
  Distance along the trajectory over which velocities are interpolated.

- **`timer_duration_msec`** (default: `100.0`)  
  Interval in milliseconds for periodically publishing updated trajectories.

- **`csv_file_path`** (default: `/data/parking_lot_trajectory_acc.csv`)  
  Path to the CSV file containing the recorded trajectory.

### Key Features
- Reads a pre-recorded trajectory from a CSV file.  
- Finds the nearest trajectory point to the vehicle in real-time.  
- Interpolates velocities along a look-ahead distance to smoothly achieve the target velocity.  
- Safely handles cases where vehicle state or target velocity is unavailable.  
- Publishes updated trajectories for downstream planning or control modules.

---
## Usage

This section explains how to launch the two ROS 2 nodes included in this package: the **FrontObjectDetector** and the **LongitudinalController**.

```bash
ros2 launch radar_front_object_detector front_object_detector.launch.xml
```

```bash
ros2 launch longitudinal_control longitudinal_control.launch.xml
```

