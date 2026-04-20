# ROS 2 Robot Navigator

## What it does
When you launch the system, a new dedicated terminal window will open, presenting an interactive UI menu with two main options:

1. **Set a new Target:** You will be prompted to enter the desired `(X, Y, Theta)` coordinates. Once set, the robot will immediately start moving towards the target. Its movement speed is proportional to the remaining distance (capped at a safe maximum limit to ensure smooth acceleration and deceleration). Once the robot reaches the exact `(X, Y)` position, it will perform a precise in-place rotation to match your requested `Theta` orientation.
2. **Cancel Target:** Instantly stops the robot and aborts the mission.

**Seamless Goal Overwriting:** The system is fully asynchronous. If you enter a new goal while the robot is already executing a previous one, the system will perfectly overwrite the target, aborting the old trajectory and immediately heading to the new destination.

## Architecture
The system is composed of two main nodes running inside a `ComposableNodeContainer`:

1. **`NavigationServer`**: Listens to `/odom` to update the `base_footprint` frame with respect to `base_world` frame. When a goal is accepted, it spawns a worker thread to broadcast the `goal_frame` and calculates the required `cmd_vel` using a Proportional Controller.
2. **`NavigationClient`**: A dedicated UI thread that prompts the user for `(X, Y, Theta)` coordinates and interacts asynchronously with the Action Server.

## Instructions to Use

Follow these steps using **two separate terminal windows**:

### Terminal 1: Build & Simulation
In this terminal, we will build the workspace and launch the Gazebo simulation environment.

```bash
colcon build

source install/setup.bash

ros2 launch bme_gazebo_sensors spawn_robot_ex.launch.py
```

### Terminal 2: Navigation System
Once the simulation is running and the /odom topic is actively publishing data, open a second terminal to start the control nodes.

```bash
source install/setup.bash

ros2 launch robot_navigator navigation_container.launch.py
```

