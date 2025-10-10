## NovAtel OEM7 Messages

Since the GNSS system here is based on NovAtel, you need to clone and build the **novatel_oem7_msgs** package (if not availale already) to make these custom messages available.

Repository: [novatel_oem7_driver](https://github.com/novatel/novatel_oem7_driver/tree/master/src)

## Update vehicle launch
Change `vehicle.launch.xml` in:  
```
autoware/src/universe/autoware.universe/launch/tier4_vehicle_launch/launch/vehicle.launch.xml
```
with the provided file in this repo.

Make sure the `vehicle_launch_pkg` argument in this file points to the correct `pacmod_interface` path on your system. Then add the pacmod_interface package and raw_vehicle_converter.launch.xml

```yaml
<include file="$(var vehicle_launch_pkg)/launch/pacmod_interface.launch.xml">
   <arg name="vehicle_id" value="$(var vehicle_id)"/>
   <arg name="raw_vehicle_cmd_converter_param_path" value="$(var raw_vehicle_cmd_converter_param_path)"/>
   <arg name="initial_engage_state" value="$(var initial_engage_state)"/>
</include>
```
```yaml
<group>
   <include file="$(find-pkg-share raw_vehicle_cmd_converter)/launch/raw_vehicle_converter.launch.xml"/>
</group>
```

---

## Update pacmod interface
Replace `pacmod_interface.cpp` in:  
```
autoware/src/vehicle/external/pacmod_interface/pacmod_interface/src/pacmod_interface/
```  
with the provided file in this repo.
Then rebuild using:
```
cd ~/autoware

colcon build --symlink-install --packages-select pacmod_interface
```
---

## Update calibration files
Replace `sensor_kit_calibration.yaml` and `sensors_calibration.yaml` in:  
```
autoware/src/param/autoware_individual_params/individual_params/config/default/sample_sensor_kit/
```  
with the provided versions in this repo.

---

## Replace sensor kit folders
Replace the following folders in:  
```
autoware/src/sensor_kit/sample_sensor_kit_launch/
```  
- `common_sensor_launch`  
- `sample_sensor_kit_description`  
- `sample_sensor_kit_launch`  

with the provided ones in this repo.  

Then rebuild these packages using:
```
cd ~/autoware

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sample_sensor_kit_description sample_sensor_kit_launch
```

---

## Localization Setup Instructions

### Update PointCloud Input Topic
**File:** `autoware/src/launcher/autoware_launch/autoware_launch/launch/components/tier4_localization_component.launch_original.xml`

Replace the `input_pointcloud` argument with the topic that publishes the processed point cloud data.

**Current topic:**
```
/sensing/lidar/top/pointcloud_before_sync
```

### Enable Localization
**File:** `autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml`

Make sure the **localization module** and **API module** are enabled.


---

## Perception Setup Instructions

### Configure PointCloud Input
**File:** `autoware/src/universe/autoware.universe/launch/tier4_perception_launch/launch/perception.launch.xml`

Update the `input_pointcloud` argument to use the correct point cloud topic.

### Enable Perception Module
Ensure that the **perception module** is enabled in `autoware.launch.xml`.


---

## System Module Setup

### Modify System Launch File
**File:** `autoware/src/universe/autoware.universe/launch/tier4_system_launch/launch/system.launch.xml`
- Comment out the **emergency_handler** module.
- Set `launch_system_monitor` to `false`.

### Modify Component State Monitor
**File:** `autoware_launch/config/system/component_state_monitor/topics.yaml`
- Comment out all topics related to **perception** and **control** modules.

Then, enable the **System** module in `autoware.launch.xml`.


---

## Planning Module Setup

- **No changes required.**
- If the **System** and **Vehicle Interface** are correctly set up and topics are publishing:
  → Simply enable the **Planning** module in `autoware.launch.xml`.


---

## Control Module Setup

**File:** `autoware/src/launcher/autoware_launch/autoware_launch/launch/components/tier4_control_component.launch.xml`
- Set `enable_autonomous_emergency_braking` to `false`.

Then, enable the **Control** module in `autoware.launch.xml`.


---

## Final Launch Steps

### 1️ Launch the Vehicle Platform
```bash
ros2 launch vehicle_platform platform.launch.xml
```

### 2️ Build and source utility package
Build and source the **autoware_independent_utility** ROS 2 package in this repo.  
Then run:
```bash
cd ~/autoware_independent_utility
source install/setup.bash
ros2 launch velodyne_quick_converter velodyne_quick_converter.launch.py
ros2 launch gnss_imu_quick_convert gnss_imu_quick_convert.launch.xml
```

### 3️ Launch Autoware with Map and Vehicle Configurations
```bash
ros2 launch autoware_launch autoware.launch.xml map_path:=$HOME/parking_lot_big_loop_lanelet vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```
> Change `map_path` based on your map directory.

### 4️ Wait for Localization
Wait for the vehicle to successfully **localize** before proceeding.

### 5️ Set a Goal in RViz
In **RViz**, set a goal point (ensure it is centered within a lane).

### 6️ Engage Vehicle Control
Once the path is planned:
- Toggle **Autoware Control** to engage **PACMod** vehicle control.

### 7️ Switch to Auto Mode
After successful engagement:
- Switch to **AUTO** mode in RViz.
- The vehicle should now begin following the planned trajectory

---
