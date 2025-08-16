# Kinematic Controller for Pioneer 3-DX (ROS + Gazebo)

📌 **Overview**

This project implements a **Kinematic Controller** for the Pioneer 3-DX robot in ROS.  
The controller generates velocity commands based on pose and orientation errors to ensure accurate **circular trajectory tracking**.

Unlike the fuzzy-ADRC approach, this controller uses **simple proportional gains** on position and orientation errors.

✨ **Features**

✅ Kinematic control law for linear & angular velocity  
✅ Circular trajectory generation  
✅ Real-time odometry feedback (`/RosAria/pose` or `/odom`)  
✅ Velocity command publishing (`/RosAria/cmd_vel` or `/cmd_vel`)  
✅ Gnuplot visualization for:
- Desired vs. actual trajectory
- Linear & angular velocities
- Desired vs. actual orientation

---

📂 **File Structure**
kc_controller/
├── src/
│ └── kc.cpp # Main kinematic controller
├── CMakeLists.txt # Example ROS build file
├── package.xml # ROS package definition
└── README.md # Documentation

⚙️ **Requirements**

- ROS 1 (Noetic/Melodic recommended)  
- Gazebo with Pioneer 3-DX model  
- Gnuplot (for plotting results)

Install Gnuplot if missing:
sudo apt-get install gnuplot

# ▶️ How to Run

1. Clone the repo into your ROS workspace:

cd ~/catkin_ws/src
git clone https://github.com/<your-username>/<repo-name>.git
cd ..
catkin_make
source devel/setup.bash

2. For Gazebo Simulation:

roslaunch p3dx_gazebo p3dx_gazebo.launch
rosrun <your-package> kc

3. For Real Robot (RosAria):

rosrun <your-package> kc
Plots of trajectory, velocities, and orientation will open in Gnuplot automatically.

# 📊 Example Output

Trajectory Tracking → Blue = desired path, Red = actual path

Linear/Angular Velocity Tracking → Desired vs. Actual

Orientation Tracking → Desired vs. Actual heading angle
