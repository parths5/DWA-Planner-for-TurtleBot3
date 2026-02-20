#!/usr/bin/env python3
"""
Dynamic Window Approach (DWA) Local Planner for TurtleBot3 in ROS2 Humble with Gazebo Simulation

Author: Parth Singh
Institution: Carnegie Mellon University
Program: MS Robotics
Date: 2026-02-03

This implementation is a custom DWA local planner written from scratch
for TurtleBot3 navigation in ROS2 Humble with Gazebo Simulation. The planner samples velocity
commands, predicts trajectories, evaluates them using a multi-objective
cost function, and selects the best trajectory for safe obstacle avoidance
and goal navigation.

License: MIT
"""

import rclpy
import math
import numpy as np
import random
import threading
import time
import logging
import os
from datetime import datetime
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from transforms3d.euler import quat2euler

# Setup file logger for detailed logs
log_dir = os.path.expanduser("~/.ros/dwa_planner_logs")
os.makedirs(log_dir, exist_ok=True)
log_filename = os.path.join(log_dir, f"dwa_planner_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")

file_logger = logging.getLogger('dwa_file_logger')
file_logger.setLevel(logging.INFO)
file_handler = logging.FileHandler(log_filename)
file_handler.setLevel(logging.INFO)
file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
file_handler.setFormatter(file_formatter)
file_logger.addHandler(file_handler)
file_logger.propagate = False  # Don't propagate to root logger

# Global state variables for DWA planner
target_goal_x = None
target_goal_y = None
goal_angle = None
robot_odom_data = None
laser_scan_data = None
goal_reached_flag = False
goal_start_timestamp = None
goal_timeout_duration = 120.0  # 120 seconds timeout
input_lock = threading.Lock()
waiting_for_input = False
debug_counter = 0  # Counter for periodic debugging messages
last_debug_time = None  # Track time for periodic debugging


def ask_for_goal():
    global target_goal_x, target_goal_y, goal_reached_flag, goal_start_timestamp, waiting_for_input
    
    with input_lock:
        waiting_for_input = True
    
    try:
        target_goal_x = float(input("Enter goal X coordinate : "))
        target_goal_y = float(input("Enter goal Y coordinate : "))
        goal_reached_flag = False
        goal_start_timestamp = time.time()
        print(f"New goal set: ({target_goal_x}, {target_goal_y}). Timeout: {goal_timeout_duration} seconds.")
    except (ValueError, EOFError, KeyboardInterrupt):
        print("Invalid input or interrupted. Using default goal (0, 0)")
        target_goal_x = 0.0
        target_goal_y = 0.0
        goal_reached_flag = False
        goal_start_timestamp = time.time()
    finally:
        with input_lock:
            waiting_for_input = False


def odom_callback(msg):
    global robot_odom_data
    robot_odom_data = msg


def scan_callback(msg):
    global laser_scan_data
    laser_scan_data = msg


def predict_motion(speed, turn_rate, step_time):
    if robot_odom_data is None:
        return []

    x = robot_odom_data.pose.pose.position.x
    y = robot_odom_data.pose.pose.position.y
    orient = robot_odom_data.pose.pose.orientation
    roll, pitch, yaw = quat2euler([orient.w, orient.x, orient.y, orient.z], axes='sxyz')

    path = []
    for i in range(100):
        yaw += turn_rate * step_time
        x += speed * math.cos(yaw) * step_time
        y += speed * math.sin(yaw) * step_time
        path.append((x, y))

    return path


def check_for_collisions(path):
    #Check for collisions along the path and return a cost based on proximity to obstacles.
    #Returns a gradient cost: more negative = closer to obstacles, 0 = safe.
    #If there is no scan data or odom data, return -infinity
    if laser_scan_data is None or robot_odom_data is None:
        return -float('inf')

    # Robot's current position and orientation
    robot_x = robot_odom_data.pose.pose.position.x
    robot_y = robot_odom_data.pose.pose.position.y
    orient = robot_odom_data.pose.pose.orientation
    roll, pitch, robot_yaw = quat2euler([orient.w, orient.x, orient.y, orient.z], axes='sxyz')
    
    # Safety parameters tuned a bit, can be tuned more for better obstacle avoidance
    robot_radius = 0.2
    safety_margin = 0.5
    min_safe_distance = robot_radius + safety_margin
    
    # Cost parameters - tuned to be aggressive, robot was colliding often while testing
    collision_penalty = -10000000
    proximity_weight = -20000
    
    min_distance_to_obstacle = float('inf')
    
    # Checking each point along the path (sampling every point for thoroughness)
    # Also checking intermediate points for better collision detection
    check_every_n = 1
    for idx in range(0, len(path), check_every_n):
        world_x, world_y = path[idx]
        # Transform path point to robot frame (relative to robot)
        dx = world_x - robot_x
        dy = world_y - robot_y
        
        # Rotate to robot frame
        rel_x = dx * math.cos(-robot_yaw) - dy * math.sin(-robot_yaw)
        rel_y = dx * math.sin(-robot_yaw) + dy * math.cos(-robot_yaw)
        
        # Distance from robot to this path point
        path_distance = math.sqrt(rel_x**2 + rel_y**2)
        
        # Angle from robot to this path point
        path_angle = math.atan2(rel_y, rel_x)
        
        # Convert angle to scan index
        # Laser scan typically ranges from angle_min to angle_max
        angle_min = laser_scan_data.angle_min
        angle_max = laser_scan_data.angle_max
        angle_increment = laser_scan_data.angle_increment
        
        # Normalize path_angle to scan range
        if path_angle < angle_min:
            path_angle += 2 * math.pi
        elif path_angle > angle_max:
            path_angle -= 2 * math.pi
        
        # Calculate scan index
        scan_index = int((path_angle - angle_min) / angle_increment)
        scan_index = max(0, min(len(laser_scan_data.ranges) - 1, scan_index))
        
        # Get obstacle distance at this angle and nearby angles (check a window)
        min_obstacle_distance = float('inf')
        check_window = 3  # Checking 3 scan indices around the target angle
        
        for offset in range(-check_window, check_window + 1):
            check_index = scan_index + offset
            if 0 <= check_index < len(laser_scan_data.ranges):
                if (laser_scan_data.ranges[check_index] > laser_scan_data.range_min and 
                    laser_scan_data.ranges[check_index] < laser_scan_data.range_max):
                    if laser_scan_data.ranges[check_index] < min_obstacle_distance:
                        min_obstacle_distance = laser_scan_data.ranges[check_index]
        
        if min_obstacle_distance < float('inf'):
            # Checking if path point is too close to obstacle
            if path_distance < min_obstacle_distance - min_safe_distance:
                # Collision detected! BAD!
                return collision_penalty
            
            # Track minimum distance to obstacle along path
            distance_to_obstacle = min_obstacle_distance - path_distance
            if distance_to_obstacle < min_distance_to_obstacle:
                min_distance_to_obstacle = distance_to_obstacle
    
    # Return gradient cost based on proximity (closer = more negative)
    # More aggressive gradient for better obstacle avoidance
    if min_distance_to_obstacle < min_safe_distance:
        # Very close to obstacle, extremely high penalty, LESS BAD BUT STILL BAD!
        penalty_factor = (min_safe_distance - min_distance_to_obstacle) / min_safe_distance
        return proximity_weight * penalty_factor * penalty_factor
    elif min_distance_to_obstacle < min_safe_distance * 1.5:
        # Moderately close, high penalty
        penalty_factor = (min_safe_distance * 1.5 - min_distance_to_obstacle) / (min_safe_distance * 0.5)
        return proximity_weight * 0.5 * penalty_factor
    elif min_distance_to_obstacle < min_safe_distance * 2.5:
        # Somewhat close, moderate penalty
        penalty_factor = (min_safe_distance * 2.5 - min_distance_to_obstacle) / (min_safe_distance * 1.0)
        return proximity_weight * 0.2 * penalty_factor
    
    # Safe distance, no penalty
    return 0


def choose_best_path(node, possible_paths):
    global target_goal_x, target_goal_y, goal_reached_flag, goal_start_timestamp

    if robot_odom_data is None or target_goal_x is None or target_goal_y is None:
        return 0.0, 0.0, 0.0, 0.0, 0.0, float('-inf')

    current_x = robot_odom_data.pose.pose.position.x
    current_y = robot_odom_data.pose.pose.position.y
    orient = robot_odom_data.pose.pose.orientation
    roll, pitch, yaw = quat2euler([orient.w, orient.x, orient.y, orient.z], axes='sxyz')

    distance_to_goal = math.hypot(target_goal_x - current_x, target_goal_y - current_y)
    
    # Check if goal is reached
    if distance_to_goal < 0.05:
        if not goal_reached_flag:
            goal_reached_flag = True
            elapsed_time = time.time() - goal_start_timestamp if goal_start_timestamp else 0
            file_logger.info(f"Goal reached at ({target_goal_x}, {target_goal_y})! Time taken: {elapsed_time:.2f} seconds")
            print(f"Goal reached! Time taken: {elapsed_time:.2f} seconds")
            # Request new goal in a separate thread
            threading.Thread(target=ask_for_goal, daemon=True).start()
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    
    # Check for timeout
    if goal_start_timestamp is not None:
        elapsed_time = time.time() - goal_start_timestamp
        if elapsed_time > goal_timeout_duration:
            if not goal_reached_flag:
                file_logger.warning(f"Goal timeout! Could not reach ({target_goal_x}, {target_goal_y}) in {goal_timeout_duration} seconds.")
                print(f"Goal timeout! Could not reach goal in {goal_timeout_duration} seconds.")
                goal_reached_flag = True  # Set to True to trigger new goal input
                # Request new goal in a separate thread
                threading.Thread(target=ask_for_goal, daemon=True).start()
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    best_score = float('-inf')
    best_speed, best_turn = 0.05, 0.0
    best_goal_cost = 0.0
    best_obstacle_cost = 0.0
    best_smoothness_cost = 0.0

    # Cost function weights (optimized for obstacle avoidance - more aggressive, can be tuned better that this)
    goal_weight = 2.0
    heading_weight = 1.0
    obstacle_weight = 2.0 
    smoothness_weight = 0.3
    speed_weight = 0.2

    for speed, turn, path in possible_paths:
        # Goal distance cost (negative because we want to minimize distance)
        goal_distance = math.hypot(path[-1][0] - target_goal_x, path[-1][1] - target_goal_y)
        goal_distance_score = -goal_distance * goal_weight
        
        # Heading alignment cost (negative because we want to minimize angle difference)
        goal_angle = math.atan2(target_goal_y - current_y, target_goal_x - current_x)
        angle_diff = abs(goal_angle - yaw)
        # Normalize angle difference to [0, pi]
        if angle_diff > math.pi:
            angle_diff = 2 * math.pi - angle_diff
        heading_score = -angle_diff * heading_weight
        
        # Obstacle avoidance cost (most important - already weighted heavily in function)
        collision_risk = check_for_collisions(path) * obstacle_weight
        
        # Smoothness cost (prefer straight paths, oculd have implemented in place rotation to avoid this so that's an area of improvement)
        smoothness_score = -abs(turn) * smoothness_weight
        
        # Speed bonus (small preference for forward motion)
        speed_bonus = speed * speed_weight
        
        total_score = goal_distance_score + heading_score + collision_risk + smoothness_score + speed_bonus
        
        if total_score > best_score:
            best_score = total_score
            best_speed, best_turn = speed, turn
            best_goal_cost = goal_distance_score
            best_obstacle_cost = collision_risk
            best_smoothness_cost = smoothness_score

    # Return best path info along with velocities for debugging
    return best_speed, best_turn, best_goal_cost, best_obstacle_cost, best_smoothness_cost, best_score



def generate_infinite_paths(max_speed, max_turn, step_time):
    while True:
        speed = random.uniform(0, max_speed)
        turn = random.uniform(-max_turn, max_turn)


        path = predict_motion(speed, turn, step_time)

        yield (speed, turn, path)


def movement_loop(node, cmd_publisher, path_publisher, max_speed, max_turn, step_time):
    global goal_reached_flag, waiting_for_input, debug_counter, last_debug_time

    # Don't move if waiting for input
    with input_lock:
        if waiting_for_input:
            return
    
    if robot_odom_data is None or laser_scan_data is None or goal_reached_flag or target_goal_x is None or target_goal_y is None:
        return

    # Initialize debug timing
    current_time = time.time()
    if last_debug_time is None:
        last_debug_time = current_time

    path_generator = generate_infinite_paths(max_speed, max_turn, step_time)
    # Increased samples for better path selection
    num_samples = 15000
    possible_paths = [next(path_generator) for i in range(num_samples)]

    result = choose_best_path(node, possible_paths)
    speed, turn, best_goal_cost, best_obstacle_cost, best_smoothness_cost, best_score = result

    move_cmd = Twist()
    move_cmd.linear.x = speed
    move_cmd.angular.z = turn
    
    # Calculate distance to goal and obstacle proximity for debugging
    current_x = robot_odom_data.pose.pose.position.x
    current_y = robot_odom_data.pose.pose.position.y
    distance_to_goal = math.hypot(target_goal_x - current_x, target_goal_y - current_y) if target_goal_x is not None else 0.0
    
    # Find minimum obstacle distance from scan
    min_obstacle_dist = float('inf')
    if laser_scan_data is not None:
        for r in laser_scan_data.ranges:
            if r > laser_scan_data.range_min and r < laser_scan_data.range_max:
                if r < min_obstacle_dist:
                    min_obstacle_dist = r
    
    # Periodic debugging (every 1 second) - write to file
    debug_counter += 1
    time_since_last_debug = current_time - last_debug_time
    if time_since_last_debug >= 1.0:  # Log detailed info every 1 second to file
        elapsed_time = time.time() - goal_start_timestamp if goal_start_timestamp else 0
        remaining_time = max(0, goal_timeout_duration - elapsed_time) if goal_start_timestamp else 0
        
        # Write detailed logs to file
        file_logger.info(f"Trajectory Evaluation (Iteration {debug_counter})")
        file_logger.info(f"  Sampled {num_samples} trajectories | Best score: {best_score:.2f}")
        file_logger.info(f"  Cost breakdown - Goal: {best_goal_cost:.2f}, Obstacle: {best_obstacle_cost:.2f}, Smoothness: {best_smoothness_cost:.2f}")
        file_logger.info(f"  Navigation status - Distance to goal: {distance_to_goal:.3f} m | Min obstacle distance: {min_obstacle_dist:.3f} m")
        file_logger.info(f"  Selected velocities - Linear: {speed:.3f} m/s, Angular: {turn:.3f} rad/s | Time remaining: {remaining_time:.1f}s")
        
        last_debug_time = current_time
    
    # Terminal output: Only velocities and time (every iteration)
    if goal_start_timestamp is not None:
        elapsed_time = time.time() - goal_start_timestamp
        remaining_time = max(0, goal_timeout_duration - elapsed_time)
        print(f"Velocities - Linear: {speed:.3f} m/s, Angular: {turn:.3f} rad/s | Time remaining: {remaining_time:.1f}s")
    else:
        print(f"Velocities - Linear: {speed:.3f} m/s, Angular: {turn:.3f} rad/s")
    
    cmd_publisher.publish(move_cmd)

    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.01
    marker.color.r = 1.0
    marker.color.a = 0.5

    for _, _, path in possible_paths:
        for x, y in path:
            point = Point()
            point.x = x
            point.y = y
            marker.points.append(point)

    path_publisher.publish(marker)


def main():
    rclpy.init()
    node = Node('dwa_planner')

    # Log initialization to file
    file_logger.info("=" * 60)
    file_logger.info("Custom DWA Local Planner for ROS2 Humble")
    file_logger.info("Author: Parth Singh, MS Robotics Systems Development, Carnegie Mellon University")
    file_logger.info("=" * 60)
    file_logger.info("Initializing DWA planner...")
    file_logger.info("Subscribing to /odom and /scan")
    file_logger.info("Publishing to /cmd_vel and /visual_paths")
    file_logger.info("Goal timeout: 120 seconds")
    file_logger.info(f"Detailed logs saved to: {log_filename}")
    file_logger.info("=" * 60)

    # Terminal output minimal
    print("DWA Planner initialized. Detailed logs saved to file.")
    print(f"Log file: {log_filename}")
    print("Terminal will show only velocities and time remaining.")
    print("-" * 60)

    ask_for_goal()

    node.create_subscription(Odometry, '/odom', odom_callback, 10)
    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    cmd_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    path_publisher = node.create_publisher(Marker, '/visual_paths', 10)

    # Further reduced max speed for better obstacle avoidance
    max_speed = 0.10
    max_turn = 1.8
    step_time = 0.1

    file_logger.info("DWA Planner initialized. Starting navigation loop.")
    file_logger.info("Trajectory evaluation logging every 1 second.")
    file_logger.info("-" * 60)

    node.create_timer(step_time, lambda: movement_loop(node, cmd_publisher, path_publisher, max_speed, max_turn, step_time))

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
