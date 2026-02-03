# Dynamic Window Approach Local Planner for ROS2 Humble

## Overview

This project presents a custom implementation of a **Dynamic Window Approach (DWA) local planner** for **TurtleBot3** robots operating in **ROS2 Humble**. Developed from scratch. this planner computes velocity commands (`/cmd_vel`) to enable autonomous navigation with effective obstacle avoidance and goal-directed movement.

## Algorithm & Features

The Dynamic Window Approach (DWA) planner operates through a four-stage process that runs iteratively:

1. **Velocity Sampling**: The planner generates candidate velocity pairs (linear and angular) by randomly sampling within predefined dynamic constraints. Each iteration evaluates 15,000 potential velocity commands, ensuring comprehensive coverage of the robot's motion capabilities.
2. **Trajectory Prediction**: For each sampled velocity command, the algorithm simulates the robot's motion forward in time (10-second prediction horizon) by integrating kinematic equations. This produces a predicted path that the robot would follow if it executed that particular velocity command.
3. **Trajectory Evaluation**: Each predicted trajectory is scored using a multi-objective cost function that balances three key factors:

   - **Goal Proximity**: Rewards trajectories that minimize the distance to the target goal position
   - **Obstacle Avoidance**: Applies gradient-based penalties that increase exponentially as trajectories approach obstacles detected by the laser scanner, with collision paths receiving severe penalties
   - **Motion Smoothness**: Favors trajectories with lower angular velocities to promote stable, predictable motion
4. **Optimal Command Selection**: The trajectory with the highest combined score is selected, and its corresponding velocity command is published to `/cmd_vel` for execution.

**ROS2 Integration**:

- Subscribes to **Odometry (`/odom`)** for real-time robot pose estimation
- Subscribes to **LaserScan (`/scan`)** for obstacle detection and proximity sensing
- Publishes velocity commands to **`/cmd_vel`** for robot control
- Publishes **RViz Markers** to `/visual_paths` for trajectory visualization in RViz

**Additional Capabilities**:

- Continuous goal input system with automatic prompting after goal completion or timeout (120 seconds)
- Comprehensive file-based logging system that records detailed trajectory evaluation metrics
- Terminal output displays real-time velocities and remaining time for goal completion
- Tested and validated in Gazebo with multiple goal states. Some are trickier than others and the planner doesn't have a perfect success rate

## Demo Videos

### Gazebo Simulation

The following video demonstrates the DWA planner navigating a TurtleBot3 robot in Gazebo while avoiding obstacles:

<video width="800" controls>
  <source src="screengrabs/gazebo.mp4" type="video/mp4">
  Your browser does not support the video tag. [Download video](screengrabs/gazebo.mp4)
</video>

### Terminal Output

This video shows the terminal output displaying real-time velocities and navigation status:

<video width="800" controls>
  <source src="screengrabs/terminal.mp4" type="video/mp4">
  Your browser does not support the video tag. [Download video](screengrabs/terminal.mp4)
</video>

**Note**: GitHub README files support video playback through HTML5 video tags. Click the links above or use the video players to view the demonstrations.

## Installation & Setup

### Prerequisites

- ROS2 Humble installed (includes `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `visualization_msgs`)
- Gazebo simulator
- Python 3.10+
- TurtleBot3 simulation packages

### 1. Install ROS2 Humble and TurtleBot3 Simulation

**Install ROS2 Humble:**

Follow the official installation guide: [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)

**Install TurtleBot3:**

Follow the official TurtleBot3 setup guide: [TurtleBot3 Installation](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

### 2. Install Python Dependencies

The planner requires the following Python packages:

```sh
pip3 install numpy transforms3d
```

**Note**: ROS2 message packages (`rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `visualization_msgs`) are included with ROS2 Humble installation and do not need separate installation.

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

- Click **Add → By Topic** and select **visual_paths**(only available while planner is running) to view the planned trajectories
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
All detailed debugging information is saved to log files in `~/.ros/dwa_planner_logs/`. Each session creates a timestamped log file.

## Implementation Details

### Cost Function Weights

- **Goal weight**: 2.0 (reduced to prioritize safety)
- **Obstacle weight**: 2.0 (high priority for collision avoidance)
- **Heading weight**: 1.0 (alignment with goal direction)
- **Smoothness weight**: 0.3 (preference for smooth paths)

### Key Parameters

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
- Cost function weights need tuning for better obstacle avoidance or for adapting to new environments
- The current implementation uses random sampling; grid-based or adaptive sampling could improve efficiency
- Integration with a global planner for long-range navigation would enhance the system

## License

MIT License
