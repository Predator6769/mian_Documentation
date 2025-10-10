# Sensor Kit Setup in Autoware Universe

This guide outlines how to configure and integrate your custom **sensor kit** in **Autoware Universe**, including calibration, URDF integration, and launch setup for LiDAR, IMU, and GNSS sensors.

---

## 1. Sensor Calibration

### File 1: `sensor_kit_calibration.yaml`
**Path:**  
`autoware/src/param/autoware_individual_params/individual_params/config/default/sample_sensor_kit/sensor_kit_calibration.yaml`

Update this file with:
- All **sensor frame names**
- Their corresponding **static transforms**
- Remove unused sensors

#### Example:
```yaml
sensor_kit_base_link:
  velodyne_top_base_link:
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  gnss_link:
    x: -1.02
    y: 0.0
    z: -1.23
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  imu_link:
    x: -1.02
    y: 0.0
    z: -1.23
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  radar_1_link:
    x: 2.98
    y: 0.0
    z: -0.66
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
```

---

### File 2: `sensors_calibration.yaml`
**Path:**  
`autoware/src/param/autoware_individual_params/individual_params/config/default/sample_sensor_kit/sensors_calibration.yaml`

#### Example:
```yaml
base_link:
  sensor_kit_base_link:
    x: 1.02
    y: 0.0
    z: 1.23
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
```
> Change frame names here if necessary.

---

## 2. URDF Integration

**Path:**  
`autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_description/urdf/sensor_kit.xacro`

Update this file to include your sensor URDFs:

### Key Points:
- Calibration values come from `sensor_kit_calibration.yaml`
- Sensor frame names are defined via the `name` argument in xacro macros
- All required sensor xacros are already included in Autoware
- **Camera integration** is not yet supported → keep those lines commented
- Import and use macros for sensors like LiDAR, radar, and IMU

#### Example
```yaml
    <!-- lidar -->
    <xacro:VLP-16 parent="sensor_kit_base_link" name="velodyne_top" topic="/velodyne_packets" hz="10" samples="220" gpu="$(arg gpu)">
      <origin
        xyz="${calibration['sensor_kit_base_link']['velodyne_top_base_link']['x']}
             ${calibration['sensor_kit_base_link']['velodyne_top_base_link']['y']}
             ${calibration['sensor_kit_base_link']['velodyne_top_base_link']['z']}"
        rpy="${calibration['sensor_kit_base_link']['velodyne_top_base_link']['roll']}
             ${calibration['sensor_kit_base_link']['velodyne_top_base_link']['pitch']}
             ${calibration['sensor_kit_base_link']['velodyne_top_base_link']['yaw']}"
      />
    </xacro:VLP-16>
```

---

## 3. Launch Configuration

### File: `sensing.launch.xml`
**Path:**  
`autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/sensing.launch.xml`

Include launch files for all required sensors (LiDAR, camera, radar, etc.).  
> Currently, camera launching is commented.

---

## LiDAR Setup

### File: `lidar.launch.xml`
**Path:**  
`autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/lidar.launch.xml`

Launch only the required LiDAR (single LiDAR setup).
Comment out unnecessary LiDAR launches.  
Keep the **pre-processing launch** enabled.
### Keep this code for lidar
```yaml
    <group>
      <push-ros-namespace namespace="top"/>
      <include file="$(find-pkg-share common_sensor_launch)/launch/velodyne_VLP16.launch.xml">
        <arg name="max_range" value="5.0"/>
        <arg name="sensor_frame" value="velodyne_top"/>
        <arg name="sensor_ip" value="192.168.1.202"/>
        <arg name="host_ip" value="$(var host_ip)"/>
        <arg name="data_port" value="2369"/>
        <arg name="scan_phase" value="180.0"/>
        <arg name="cloud_min_angle" value="300"/>
        <arg name="cloud_max_angle" value="60"/>
        <arg name="launch_driver" value="$(var launch_driver)"/>
        <arg name="vehicle_mirror_param_file" value="$(var vehicle_mirror_param_file)"/>
        <arg name="container_name" value="pointcloud_container"/>
      </include>
    </group>
```
### Nebula Preprocessing Node
Launching `lidar.launch.xml` starts `nebula_node_container.launch.py`, which handles LiDAR preprocessing.

**Path:**  
`autoware/src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/nebula_node_container.launch.py`

#### Steps:
- Comment out the **driver launch section** (since you use a custom driver).
```yaml
    # nodes.append(
    #     ComposableNode(
    #         package="nebula_ros",
    #         plugin=sensor_make + "DriverRosWrapper",
    #         name=sensor_make.lower() + "_driver_ros_wrapper_node",
    #         parameters=[
    #             {
    #                 "calibration_file": sensor_calib_fp,
    #                 "sensor_model": sensor_model,
    #                 **create_parameter_dict(
    #                     "host_ip",
    #                     "sensor_ip",
    #                     "data_port",
    #                     "return_mode",
    #                     "min_range",
    #                     "max_range",
    #                     "frame_id",
    #                     "scan_phase",
    #                     "cloud_min_angle",
    #                     "cloud_max_angle",
    #                     "dual_return_distance_threshold",
    #                 ),
    #             },
    #         ],
    #         remappings=[
    #             ("aw_points", "pointcloud_raw"),
    #             ("aw_points_ex", "pointcloud_raw_ex"),
    #         ],
    #         extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    #     )
    # )
```
- Remap the point cloud topic from your driver to the **input** of the next node (e.g., `crop_box_filter`).
```yaml
    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            remappings=[
                ("input", "velodyne_points"),
                ("output", "self_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )
```
- Replace filters if necessary. Example:  
  Changed `RingOutlierFilterComponent` → `RadiusSearch2DOutlierFilterComponent`.

#### Expected PointCloud Format
Autoware Universe expects:
```
PointXYZIRADRT
```
If your driver outputs a different format, convert it using a custom C++ node.  
(Currently handled by `autoware_independent_utility` package.)

---

## Sensor Frame IDs
Ensure all sensor frame IDs match those in `sensor_kit_calibration.yaml`.

---

## IMU Setup

### File: `imu.launch.xml`
**Path:**  
`autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/imu.launch.xml`

Set the `imu_raw_name` argument to your IMU topic published by the custom driver.
```yaml
<arg name="imu_raw_name" default="/novatel/oem7/imu/data_raw_frame"/>
```

---

## GNSS Setup

- Ensure the map used for GNSS localization is in **Traverse Mercator projection**, not a local frame.
- You can set the projection using **Vector Map Builder**.
- Provide map origin coordinates (latitude, longitude, altitude).
```yaml
projector_type: TransverseMercator
vertical_datum: WGS84
map_origin:
   latitude: 39.141570894944486
   longitude: -85.92405042212876
   altitude: 187.7943782536313
```

### File: `map_projector_info.yaml`
Add this file with map origin info to your map directory.

### File: `gnss.launch.xml`
**Path:**  
`autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/gnss.launch.xml`

Set:
```xml
<let name="navsatfix_topic_name" value="/novatel/oem7/fix_frame" if="$(eval &quot;'$(var gnss_receiver)'=='ublox'&quot;)"/>
```

---
