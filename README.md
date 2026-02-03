# Dynamic Window Approach Local Planner for ROS2 Humble

## Overview

This project implements a custom **Dynamic Window Approach (DWA) local planner** for a **TurtleBot3** in **ROS2 Humble**. The planner is written from scratch without using `nav2_dwb_controller` and generates velocity commands (`/cmd_vel`) for safe obstacle avoidance and goal navigation.

## Algorithm Description

The Dynamic Window Approach (DWA) is a local path planning algorithm that:

1. **Samples velocity commands** within dynamic constraints (max linear and angular velocities)
2. **Predicts trajectories** by simulating robot motion forward in time for each velocity sample
3. **Evaluates trajectories** using a multi-objective cost function that considers:
   - **Distance to goal**: Prefers trajectories that bring the robot closer to the target
   - **Obstacle avoidance**: Heavily penalizes trajectories that would cause collisions or get too close to obstacles
   - **Path smoothness**: Prefers smoother trajectories with less angular velocity changes
4. **Selects the best trajectory** based on the cost function and publishes the corresponding velocity command

The implementation uses a gradient-based obstacle cost function that provides smooth penalties based on proximity to obstacles, ensuring safe navigation even in cluttered environments.

## Features

- ✅ Samples velocity commands within dynamic constraints
- ✅ Predicts trajectories based on sampled velocities
- ✅ Evaluates trajectories using a comprehensive cost function (goal distance, obstacle avoidance, and smoothness)
- ✅ Selects the best trajectory and publishes velocity commands (`/cmd_vel`)
- ✅ Subscribes to **Odometry (`/odom`)** and **LaserScan (`/scan`)**
- ✅ Uses **RViz Markers** to visualize sampled trajectories
- ✅ Works in **Gazebo** with obstacles for real-world testing
- ✅ Continuous goal input with 120-second timeout per goal
- ✅ Comprehensive debugging messages for trajectory evaluation and decision-making

## Installation & Setup

### Prerequisites

- ROS2 Humble installed
- Gazebo simulator
- Python 3.10+ with numpy and transforms3d

### 1. Install ROS2 Humble and TurtleBot3 Simulation

**Install ROS2 Humble:**

Follow the official installation guide: [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)


**Install TurtleBot3:**

Follow the official TurtleBot3 setup guide: [TurtleBot3 Installation](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)


### 2. Install Python Dependencies

```sh
pip3 install numpy transforms3d
```

### 3. Build This Package

```sh
cd ~/ros2_ws  # Navigate to your ROS2 workspace
source /opt/ros/humble/setup.bash  # or setup.zsh if using zsh like me
colcon build --packages-select dwa_planner
source install/setup.bash  # or setup.zsh if using zsh
```

### 4. Launch the Simulation

**Terminal 1 - Launch Gazebo:**

```sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 - Run DWA Planner(after gazebo has launched):**

```sh
cd ~/ros2_ws  # or your workspace path
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dwa_planner dwa_planner
```

When prompted, enter goal coordinates (e.g., 1.5 for X, 0.5 for Y).

**Terminal 3 (Optional) - RViz Visualization:**

```sh
cd ~/ros2_ws  # or your workspace path
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2
```

In RViz:

- Click **Add → By Topic** and select **visual_paths** to view the planned trajectories
- Add **LaserScan** (`/scan`) to see obstacle detection
- Add **TF** to visualize coordinate frames
- Add **RobotModel** to see the robot

## Usage

### Goal Input

- The planner prompts for goal coordinates when it starts
- After reaching a goal (or 120-second timeout), it automatically prompts for the next goal
- The robot will navigate to each goal while avoiding obstacles

### Debugging Messages

**Terminal Output:**
The terminal displays only essential information:
- Current linear and angular velocities
- Time remaining to reach the goal

Example terminal output:
```
Velocities - Linear: 0.100 m/s, Angular: 0.250 rad/s | Time remaining: 45.3s
```

**Detailed Log Files:**
All detailed debugging information is saved to log files in `~/.ros/dwa_planner_logs/`. Each session creates a timestamped log file (e.g., `dwa_planner_20260203_143022.log`).

The log files contain:
- **Trajectory sampling**: Number of paths evaluated per iteration
- **Cost function breakdown**: Goal distance, obstacle avoidance, and smoothness costs
- **Navigation status**: Distance to goal, obstacle proximity, and goal completion
- **Initialization messages**: Planner configuration and setup information
- **Goal status**: Goal reached notifications and timeout warnings

Example log file content:
```
2026-02-03 14:30:22 - INFO - DWA Planner initialized. Starting navigation loop.
2026-02-03 14:30:25 - INFO - Trajectory Evaluation (Iteration 20)
2026-02-03 14:30:25 - INFO -   Sampled 15000 trajectories | Best score: -1253.60
2026-02-03 14:30:25 - INFO -   Cost breakdown - Goal: -3.45, Obstacle: -1250.00, Smoothness: -0.15
2026-02-03 14:30:25 - INFO -   Navigation status - Distance to goal: 1.23 m | Min obstacle distance: 0.85 m
2026-02-03 14:30:27 - INFO - Goal reached at (2.0, 1.0)! Time taken: 12.45 seconds.
```

## Implementation Details

### Cost Function Weights

- **Goal weight**: 2.0 (reduced to prioritize safety)
- **Obstacle weight**: 2.0 (high priority for collision avoidance)
- **Heading weight**: 1.0 (alignment with goal direction)
- **Smoothness weight**: 0.3 (preference for smooth paths)

### Safety Parameters

- **Robot radius**: 0.2 m
- **Safety margin**: 0.5 m
- **Minimum safe distance**: 0.7 m
- **Max linear velocity**: 0.10 m/s
- **Max angular velocity**: 1.8 rad/s

### Trajectory Sampling

- **Samples per iteration**: 15,000 trajectories
- **Prediction horizon**: 100 steps × 0.1s = 10 seconds
- **Step time**: 0.1 seconds

## Known Limitations & Future Improvements

- The planner uses a simplified collision checking algorithm that could be enhanced with more sophisticated footprint checking
- Cost function weights may need tuning for different environments or robot types
- The current implementation uses random sampling; grid-based or adaptive sampling could improve efficiency
- Integration with a global planner for long-range navigation would enhance the system

## License

MIT License
