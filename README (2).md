## Drive Link for Data
[Download Data (Google Drive)](https://drive.google.com/file/d/1O_A29byAwOdklaBuVPmSqvipU-cOxMCh/view?usp=sharing)

After downloading, extract the ZIP file. It contains:
- ROS bag  
- Map data  
- A demonstration video showing the issue  

# How to Reproduce the issue

## NovAtel OEM7 Messages

Since the GNSS system here is based on NovAtel, you need to clone and build the **novatel_oem7_msgs** package (if not availale already) to make these custom messages available.

Repository: [novatel_oem7_driver](https://github.com/novatel/novatel_oem7_driver/tree/master/src)

## 1. Update vehicle launch
Replace `vehicle.launch.xml` in:  
```
autoware/src/universe/autoware.universe/launch/tier4_vehicle_launch/launch/vehicle.launch.xml
```
with the provided file in this repo.

Make sure the `vehicle_launch_pkg` argument in this file points to the correct `pacmod_interface` path on your system.

---

## 2. Update pacmod interface
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

## 3. Update calibration files
Replace `sensor_kit_calibration.yaml` and `sensors_calibration.yaml` in:  
```
autoware/src/param/autoware_individual_params/individual_params/config/default/sample_sensor_kit/
```  
with the provided versions in this repo.

---

## 4. Replace sensor kit folders
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

## 5. Update Eagleye config
Replace the contents of:  
```
autoware/src/launcher/autoware_launch/autoware_launch/config/localization/eagleye_config.param.yaml
```  
with the provided config file.  

Then update the following arguments in:  
```
autoware/src/launcher/autoware_launch/autoware_launch/launch/components/tier4_localization_component.launch.xml
```  
- `pose_source` → `"eagleye"`  
- `twist_source` → `"eagleye"`

---

## 6. Build and source utility package
Build and source the **autoware_independent_utility** ROS 2 package in this repo.  
Then run:
```bash
ros2 launch velodyne_quick_converter velodyne_quick_converter.launch.py
ros2 launch gnss_imu_quick_convert gnss_imu_quick_convert.launch.xml
```

---

## 7. Play bag file and launch Autoware
Play the provided bag file with remapping:
```bash
ros2 bag play test_field_eagleye_bag --remap /tf:=/b1 /tf_static:=/b2
```

Run Autoware using autoware.launch.xml with the provided map data using (change map path accordingly):
```bash
ros2 launch autoware_launch autoware.launch.xml map_path:=$HOME/map_location/test_field_map_latest/ vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```
Disable **system**, **planning**, **perception**, and **control** modules in `autoware.launch.xml` since only localization is being tested.

---

## 8. Topics for verification
To check the point cloud output, listen on:  
```
/sensing/lidar/top/pointcoud_before_sync
```

---

If you’re testing with the provided **map and bag data**, follow **steps 1–8**.  
If you’re using your **own map and sensor data**, just complete **steps 5** (skip others).
