# Eco Drive Stack

## Overview

The `eco_drive` stack contains a set of ROS 2 packages used for testing and evaluating eco-driving strategies. The system currently consists of:

- `pure_idm`
- `eco_drive_nmpc`
- `eco_drive_fsm`

All packages are launched together using:

```bash
ros2 launch acc_unit_launch eco_drive_launch.launch.xml
```

---

# Package Structure

```text
eco_drive/
├── pure_idm/
├── eco_drive_nmpc/
├── eco_drive_fsm/
```

---

# Packages

## 1. pure_idm

The `pure_idm` package implements a simple proportional longitudinal controller.

This package is equivalent to the `simple_proportional_controller` package available under the `acc_unit_branch`, with the proportional gain configured as:

```text
kp = 0
```

### Purpose

The package serves as the baseline IDM-style velocity source for comparison and arbitration inside the FSM.

### Output

Publishes:

- Desired velocity
- Desired acceleration

---

## 2. eco_drive_nmpc (Will change package name later)

At present, it functions as a dummy velocity publisher.

### Current Functionality

- Publishes a constant velocity configured through a YAML file
- Computes the corresponding acceleration
- Publishes both velocity and acceleration

### Configuration

File:

```text
eco_drive_nmpc.yaml
```

Example:

```yaml
eco_drive_nmpc:
  ros__parameters:
    publish_rate: 10.0
    velocity: 5.0
```

### Parameters

| Parameter      | Description                     | Default |
|----------------|---------------------------------|---------|
| `publish_rate` | Velocity publishing frequency   | `10.0`  |
| `velocity`     | Constant target velocity (m/s)  | `5.0`   |

---

## 3. eco_drive_fsm

The `eco_drive_fsm` package implements a simple finite state machine (FSM) responsible for selecting the final vehicle command.

### Responsibilities

- Monitors emergency conditions
- Chooses between IDM and dummy velocity outputs
- Publishes final velocity and acceleration commands

### FSM Logic

#### Emergency Brake State

If:

```text
gap < 5.0
```

Then:

```text
velocity     = 0.0
acceleration = -3.0
```

#### Velocity Selection Logic

If:

```text
dummy_velocity > idm_velocity
```

Then:

```text
final_velocity = idm_velocity
```

Else:

```text
final_velocity = dummy_velocity
```

The corresponding acceleration is also published.

---

# Launching the System

Launch the complete eco-drive stack using:

```bash
ros2 launch acc_unit_launch eco_drive_launch.launch.xml
```

---