
# Leader-Follower Control for Pioneer 3-DX in ROS + Gazebo

This repository contains ROS nodes for implementing **leader-follower control** of Pioneer 3-DX robots in a Gazebo simulation.  
- The **leader node** (`l.cpp`) moves the leader robot in a circular trajectory.  
- The **follower node** (`f.cpp`) enables the follower robot to track the leader robot using a proportional control law based on odometry feedback.  

Both nodes are written in C++ and use ROS messages (`geometry_msgs/Twist`, `nav_msgs/Odometry`) for communication.

---

## Features
- Simulates **leader-follower behavior** between two Pioneer 3-DX robots.
- Leader robot moves in a **circular trajectory**.
- Follower robot adjusts its linear and angular velocity to minimize error relative to the leader.
- Logs and plots:
  - Desired vs. Actual Trajectory  
  - Linear velocity tracking  
  - Angular velocity tracking  
  - Orientation (Theta) tracking  
- Uses **Gnuplot** for visualization.

---

## Requirements
- **Ubuntu** (tested on 20.04 / 22.04)  
- **ROS** (tested on ROS Noetic, should work with other versions with minor adjustments)  
- **Gazebo** simulator  
- **Gnuplot** (for trajectory and velocity plots)

# Usage

# Step 1: Launch Gazebo with Pioneer 3-DX robots
Make sure your Gazebo world contains two Pioneer 3-DX robots named:

/leader (publishes to /leader/cmd_vel and /leader/odom)

/follower (publishes to /follower/cmd_vel and /follower/odom)

You can use an existing Pioneer 3-DX model or adapt your Gazebo world accordingly.

# Step 2: Run the Leader Node

The leader moves in a circular trajectory:

rosrun leader_follower l

# Step 3: Run the Follower Node

The follower will track the leader:
rosrun leader_follower f

# Output
The follower robot tracks the leader in Gazebo.

After execution (~34s), plots will be generated using Gnuplot:

Desired vs. Actual Trajectory

Linear Velocity Tracking

Angular Velocity Tracking

Orientation Tracking
