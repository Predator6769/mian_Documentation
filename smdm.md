# SMDM

## Step 1: Build and Source Utility Package
```bash
cd ~/autoware_independent_utility
source install/setup.bash
ros2 launch velodyne_quick_converter velodyne_quick_converter.launch.py
ros2 launch gnss_imu_quick_convert gnss_imu_quick_convert.launch.xml
```

## Step 2: Launch Autoware with Map and Vehicle Configurations
```bash
ros2 launch autoware_launch autoware.launch.xml map_path:=$HOME/parking_lot_big_loop_lanelet vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

## Step 3: Run Goal and Pose Tracking Scripts
In another terminal:
```bash
conda deactivate
cd ~/Documents
python3 goal_check.py
python3 record_vehicle_pose.py
```

## Step 4: Launch SMDM Demo
In another terminal:
```bash
conda activate smdm
cd ~/Documents/SMDM/SMDM/
python3 demo.py
```
