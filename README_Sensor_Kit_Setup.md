# 🚗 Sensor Kit Setup in Autoware Universe

This guide outlines how to configure and integrate your custom **sensor kit** in **Autoware Universe**, including calibration, URDF integration, and launch setup for LiDAR, IMU, and GNSS sensors.

---

## 📦 1. Sensor Calibration

### 🔧 File 1: `sensor_kit_calibration.yaml`
**Path:**  
`autoware/src/param/autoware_individual_params/individual_params/config/default/sample_sensor_kit/sensor_kit_calibration.yaml`

Update this file with:
- All **sensor frame names**
- Their corresponding **static transforms**
- Remove unused sensors

#### ✅ Example:
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

### 🔧 File 2: `sensors_calibration.yaml`
**Path:**  
`autoware/src/param/autoware_individual_params/individual_params/config/default/sample_sensor_kit/sensors_calibration.yaml`

#### ✅ Example:
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
> 📝 Change frame names here if necessary.

---

## 🧩 2. URDF Integration

**Path:**  
`autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_description/urdf/sensor_kit.xacro`

Update this file to include your sensor URDFs:

### ✅ Key Points:
- Calibration values come from `sensor_kit_calibration.yaml`
- Sensor frame names are defined via the `name` argument in xacro macros
- All required sensor xacros are already included in Autoware
- **Camera integration** is not yet supported → keep those lines commented
- Import and use macros for sensors like LiDAR, radar, and IMU

---

## 🚀 3. Launch Configuration

### File: `sensing.launch.xml`
**Path:**  
`autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/sensing.launch.xml`

Include launch files for all required sensors (LiDAR, camera, radar, etc.).  
> ⚙️ Currently, camera launching is commented.

---

## 🌐 LiDAR Setup

### File: `lidar.launch.xml`
**Path:**  
`autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/lidar.launch.xml`

Launch only the required LiDAR (single LiDAR setup).  
Comment out unnecessary LiDAR launches.  
Keep the **pre-processing launch** enabled.

### 🧠 Nebula Preprocessing Node
Launching `lidar.launch.xml` starts `nebula_node_container.launch.py`, which handles LiDAR preprocessing.

**Path:**  
`autoware/src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/nebula_node_container.launch.py`

#### Steps:
- Comment out the **driver launch section** (since you use a custom driver).
- Remap the point cloud topic from your driver to the **input** of the next node (e.g., `crop_box_filter`).
- Replace filters if necessary. Example:  
  Changed `RingOutlierFilterComponent` → `RadiusSearch2DOutlierFilterComponent`.

#### 📊 Expected PointCloud Format
Autoware Universe expects:
```
PointXYZIRADRT
```
If your driver outputs a different format, convert it using a custom C++ node.  
(Currently handled by `autoware_independent_utility` package.)

---

## 🧭 Sensor Frame IDs
Ensure all sensor frame IDs match those in `sensor_kit_calibration.yaml`.

---

## ⚙️ IMU Setup

### File: `imu.launch.xml`
**Path:**  
`autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/imu.launch.xml`

Set the `imu_raw_name` argument to your IMU topic published by the custom driver.

---

## 📡 GNSS Setup

- Ensure the map used for GNSS localization is in **Traverse Mercator projection**, not a local frame.
- You can set the projection using **Vector Map Builder**.
- Provide map origin coordinates (latitude, longitude, altitude).

### 🗺️ File: `map_projector_info.yaml`
Add this file with map origin info to your map directory.

### File: `gnss.launch.xml`
**Path:**  
`autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/gnss.launch.xml`

Set:
```xml
<navsatfix_topic_name>GNSS fix topic from your driver</navsatfix_topic_name>
```

---

## ✅ Summary
| Step | Component | File Path | Key Action |
|------|------------|------------|-------------|
| 1 | Sensor Calibration | `sensor_kit_calibration.yaml` | Define transforms |
| 2 | Sensor Calibration | `sensors_calibration.yaml` | Define base to kit transform |
| 3 | URDF Integration | `sensor_kit.xacro` | Add sensors to URDF |
| 4 | Launch Config | `sensing.launch.xml` | Include sensor launch files |
| 5 | LiDAR | `lidar.launch.xml` | Enable preprocessing |
| 6 | IMU | `imu.launch.xml` | Set topic name |
| 7 | GNSS | `gnss.launch.xml` | Configure projection + topic |

---

📘 **Author:** Prashanth  
🕹️ **Project:** Autoware Universe Sensor Kit Setup  
🏁 **Purpose:** For integrating LiDAR, IMU, and GNSS with a custom sensor kit
