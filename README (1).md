# Waypoint Following and Mapping Guide (Autoware + LIO-SAM)

This guide provides a complete procedure to **record, publish, and follow waypoints** in Autoware, and to **generate maps** using LIO-SAM with or without GPS.

---

## Step 1: Disable Planning and Perception

Before starting, open:
```
autoware\src\launcher\autoware_launch\autoware_launch\launch\autoware.launch.xml
```
and **disable** the *Planning* and *Perception* modules.

---

## Step 2: Recording Waypoints

### 1. Launch Autoware
Deactivate Conda and start Autoware with your desired map and vehicle configuration:

```bash
conda deactivate
ros2 launch autoware_launch autoware.launch.xml   map_path:=$HOME/parking_lot_big_loop_lanelet   vehicle_model:=sample_vehicle   sensor_model:=sample_sensor_kit
```

### 2. Start Waypoint Recording
In a new terminal:
```bash
cd ~/Documents
python3 record_waypoints.py
```
Then **drive the vehicle manually** to trace the desired path.

### 3. Stop Recording
Once done, stop the recording with:
```
Ctrl + C
```
A `.csv` file containing all waypoints will be created in your **home directory**.

### 4. Post-Processing the Waypoints
Open the generated `.csv` file and verify that the **last entry** has:
```
velocity = 0.0
acceleration = 0.0
```
Edit manually if necessary.

---

## Step 3: Publishing the Recorded Waypoints

### 1. Update CSV File Path
Open:
```
autoware_independent_utility/src/publish_waypoints/src/publish_waypoints.cpp
```
and update the CSV file name to point to your recorded file. Do this using the variable **csv_file_path_**

### 2. Build and Run
```bash
cd ~/autoware_independent_utility
source install/setup.bash
ros2 run publish_waypoints publish_waypoints
```

### 3. Vehicle Alignment and Control
- Ensure the vehicle in **RViz** is aligned with the **start point** of your path.
- Toggle **Autoware Control** to engage **PACMod** vehicle control.
- After engagement, switch to **AUTO mode** in RViz.

---

## Step 4: Mapping Using LIO-SAM

You can generate maps either **with GPS** or **without GPS**.

### Without GPS

1. Navigate to your LIO-SAM workspace:
   ```bash
   cd ~/lio-sam-ws
   source install/setup.bash
   ```

2. Open the config file:
   ```
   lio-sam-ws/src/LIO-SAM/config/params.yaml
   ```

3. Modify:
   ```yaml
   gpsTopic: "/gps_odom"
   ```
   to a **dummy topic**, e.g.:
   ```yaml
   gpsTopic: "/gps_o"
   ```

4. Launch LIO-SAM:
   ```bash
   ros2 launch lio_sam run.launch.py
   ```

5. Drive manually to record the map data.

6. After recording, open another terminal and save the map:
   ```bash
   cd ~/lio-sam-ws
   source install/setup.bash
   ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: <YOUR-MAP-DIRECTORY>}"
   ```
   This command will save the generated **.pcd map files**.

---

### With GPS

1. Ensure the vehicle is **facing east** before starting.  
   - If it’s not, adjust the **angle calibration** using the parameters:
     ```yaml
     extrinsicRot
     extrinsicRPY
     ```
     in:
     ```
     lio-sam-ws/src/LIO-SAM/config/params.yaml
     ```

2. In the same file, set:
   ```yaml
   gpsTopic: "/gps_odom"
   ```

3. Launch LIO-SAM:
   ```bash
   cd ~/lio-sam-ws
   source install/setup.bash
   ros2 launch lio_sam run.launch.py
   ```

4. Drive manually to record.

5. After finishing, save the map in another terminal:
   ```bash
   cd ~/lio-sam-ws
   source install/setup.bash
   ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: <YOUR-MAP-DIRECTORY>}"
   ```
   This will save your **.pcd map files** in the specified directory.

---
