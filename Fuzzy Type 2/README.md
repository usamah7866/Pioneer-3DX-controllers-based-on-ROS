
# Fuzzy Logic Type 2 + ADRC Controller for Pioneer 3-DX (ROS + Gazebo)

# 📌 Overview

This project implements a **Fuzzy Logic-based Active Disturbance Rejection Controller (ADRC)** for the Pioneer 3-DX differential drive robot in ROS.  
The controller ensures **robust trajectory tracking** while compensating for disturbances and uncertainties using fuzzy inference on both linear and angular velocity errors.

The robot is commanded to follow a **circular trajectory**, and the fuzzy-ADRC dynamically adjusts control signals to minimize tracking error.

✨ **Features**

✅ Fuzzy Logic with error & error-derivative fuzzification  
✅ ADRC compensation for linear & angular velocities  
✅ Circular trajectory tracking  
✅ Real-time odometry feedback (`/pioneer/odom`)  
✅ Velocity command publishing (`/pioneer/cmd_vel`)  
✅ Automatic disturbance rejection  
✅ Gnuplot visualization for:
- Desired vs. actual trajectory
- Linear & angular velocities
- Desired vs. actual orientation
- Controller response over time

---

📂 **File Structure**
fuzzy2_controller/
├── src/
│ └── fuzzy2.cpp # Main controller with fuzzy + ADRC
├── CMakeLists.txt # Example ROS build file
├── package.xml # ROS package definition
└── README.md # Documentation

⚙️ **Requirements**

- ROS 1 (Noetic/Melodic recommended)  
- Gazebo with Pioneer 3-DX model  
- Gnuplot (for plotting results)

Install Gnuplot if missing:
```bash
sudo apt-get install gnuplot

▶️ **How to Run**

1. Clone the repo into your ROS workspace:

cd ~/catkin_ws/src
git clone https://github.com/<your-username>/<repo-name>.git
cd ..
catkin_make
source devel/setup.bash

2. Launch Gazebo with Pioneer 3-DX:

roslaunch p3dx_gazebo p3dx_gazebo.launch

3.Run the fuzzy-ADRC controller node:

rosrun <your-package> fuzzy2
Gnuplot windows will display trajectory and control results.

# 📊 Example Output

Trajectory Tracking → Blue = desired path, Red = actual path

Velocity Tracking → Desired vs. Actual (linear & angular)

Orientation Tracking → Desired vs. Actual heading angle

Fuzzy-ADRC control response

