# Body-Frame Trajectory CSV Guidelines

This document specifies the constraints and formatting requirements for CSV files representing a vehicle trajectory in the **body frame**. These files are intended for use with the ROS 2 node that converts body-frame trajectories into world-frame `autoware_auto_planning_msgs/Trajectory`.

## CSV Format

Each row in the CSV file must include the following columns **in order**:

```
x, y, z, roll, pitch, yaw, velocity
```

- **x**: Longitudinal position in meters (forward direction relative to vehicle).  
- **y**: Lateral position in meters (sideways relative to vehicle).  
- **z**: Vertical position in meters.  
- **roll, pitch, yaw**: Orientation in **radians** with respect to body frame.  
- **velocity**: Longitudinal velocity at that point in **m/s**.

### Example:

```csv
x,y,z,roll,pitch,yaw,velocity
0.0,0.0,0.0,0.0,0.0,0.0,2.5
3.0,0.0,0.0,0.0,0.0,0.0,3.0
6.0,0.0,0.0,0.0,0.0,0.0,0.0
```

## Constraints

1. **First Point Must Be at Origin**  
   - `x = 0.0`, `y = 0.0`, `z = 0.0`.  
   - `velocity` **must be non-zero**.

2. **Last Point Velocity Must Be Zero**  

3. **All Points Must Include All Columns**  
   - Each row must include values for `x, y, z, roll, pitch, yaw, velocity`.  
   - Missing values are not allowed.

4. **Orientation Values**  
   - `roll`, `pitch`, `yaw` must be in **radians**.  

5. **Velocity Consistency**  
   - Intermediate points may have varying velocity, but the first point must start with a **non-zero velocity**, and the last point must end with **0 m/s**.

---

This ensures that the ROS 2 trajectory converter can:

- Correctly transform points from **body frame → world frame** using odometry.  
- Convert **roll/pitch/yaw → quaternion**.  
- Compute **acceleration** from velocity and distance between points.  
- Generate valid `autoware_auto_planning_msgs/Trajectory` messages for simulation or real vehicle execution.

