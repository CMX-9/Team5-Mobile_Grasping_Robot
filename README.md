# Manchester Uni Robotic Systems Design Project Team5 (AERO62520)
## Introduction
Welcome to the GitHub repository of Manchester Robotics Team5!
Our team consists of four passionate and responsible members: Mingxiang Chen、Zhen Yang、Yunxue Pan、Yu-chuan Liao.
The goal of this project is to **"Develop a robot that can autonomously retrieve colored objects from the environment and place them in matching storage bins located at the starting point."**

### Hardware Components
The main hardware of the robot includes:
- the Trossen Robotics Pincher X150 mobile arm  
- Slamtec RpLiDAR A2M12  
- PincherX150 manipulator  
- Intel D435 RealSense depth camera  
- RGB camera  

### Software Modules
The software mainly implements six functional modules:
- Perception Module  
- Navigation Module  
- Grasping Module  
- User Interface Module  
- Communication Module  
- System Management Module
  
**Please note that the project is still ongoing, and the code is currently incomplete.**

---

## Table of Contents
- [1. Environment Setup](#environment-setup)
- [2. Core Packages](#core-packages)
- [3. Module Analysis](#module-analysis)
- [4. Future Prospects](#future-prospects)
- [5. Contact](#contact)
- [6. Acknowledgments](#acknowledgments)

## 1. Environment Setup
- Refer to the documentation provided on the official [leo rover](https://docs.fictionlab.pl/leo-rover) website to understand the assembly process and relevant information about Leo Rover.  
- Install **ROS 2 Humble** and **[Gazebo](https://gazebosim.org/api/gazebo/6.1/install.html)** on Ubuntu.  
- Clone the [leo_simulator-ros2](https://github.com/LeoRover/leo_simulator-ros2) from the official ROS website.  
- Create a Python package and source its setup file:  
   ```bash
   ros2 pkg create --build-type ament_python <package_name>
   source install/local_setup.bash```
- Clone our repository, place it in your workspace, then compile the Python package and source its setup file.

## 2. Core Packages
RPLIDAR ROS - [https://github.com/Slamtec/rplidar_ros](https://github.com/Slamtec/rplidar_ros)  
SLAM-Toolbox - [https://github.com/SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)  
Nav2 - [https://index.ros.org/p/nav2_map_server](https://index.ros.org/p/nav2_map_server)  
REALSENSE2 - [https://github.com/IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros)  
interbotix_ros_manipulators - [https://github.com/Interbotix/interbotix_ros_manipulators](https://github.com/Interbotix/interbotix_ros_manipulators)  
Wavefront Exploration - [https://github.com/gjcliff/SLAM-Frontier-Exploration](https://github.com/gjcliff/SLAM-Frontier-Exploration)

## 3. Core Modules

### Navigation Module
Based on the **leo_navigation tutorials**, our preliminary design is as follows (detailed code is not yet completed):  
We plan to use **slam_toolbox** to build real-time maps of dynamic environments for exploring unknown areas while performing local localization. Additionally, we will combine **EKF (Extended Kalman Filter)** to fuse odometry, IMU, and LiDAR data, improving the smoothness and accuracy of localization.  
The target detection algorithm has not yet been determined. For **Global Path Planning**, we plan to use the **A*** algorithm, which is simple and efficient in finding the shortest path. For **Local Path Planning**, we plan to use **TEB**.  

**Core nodes include:**
- `/planner_server`, `/controller_server`, and `/bt_navigator` for path planning and navigation.  
- `/slam_toolbox` for real-time SLAM and localization.  
- `/velocity_smoother` for smooth velocity control.  
- `/explore_node` for target detection and obstacle avoidance during navigation.  

**Communication Topics**:
- Topics such as `/odom`, `/speed_limit`, `/map`, and `/joint_states` will be used to update `/cmd_vel`, `/pose`, etc., in real-time.  
- This will enable autonomous navigation, obstacle avoidance, dynamic path adjustment, and target detection during navigation.  

#### Navigation-Related Packages and Documentation:
- **Odometry**: [ekf](https://wiki.ros.org/robot_localization), [imu](https://wiki.ros.org/imu_filter_madgwick)  
- **Mapping**:  [mapping](https://wiki.ros.org/gmapping)
- **Twist Multiplexer**:  [twist_mux](https://wiki.ros.org/twist_mux)
- **AMCL**:  [map_server](https://wiki.ros.org/twist_mux),[amcl](https://wiki.ros.org/amcl)
- **Move_Base**:  [Move_Base](https://wiki.ros.org/move_base)

By writing relevant launch files and combining them, basic navigation functionality can be achieved.

---

### Grasping Module
We use the **[Manipulator PincherX150](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html)**. For necessary software libraries related to this manipulator, please download them from the official documentation:  
The package contains the necessary configuration and launch files to integrate many Interbotix X-Series robotic arms with [the perception pipeline](https://industrial-training-master.readthedocs.io/en/melodic/_source/session5/Building-a-Perception-Pipeline.html).  
The manipulator can pick up any small, non-reflective object from a flat surface within the field of view of a **RealSense color/depth camera**, making it ideal for achieving the grasping objectives in our robot project.  

For detailed information, refer to the [interbotix_perception_modules](https://github.com/Interbotix/interbotix_ros_toolboxes/tree/humble/interbotix_perception_toolbox/interbotix_perception_modules) ROS package.

---

## 4. Future Prospects

### Navigation
To achieve full autonomy for the Leo Rover, we can introduce more advanced navigation technologies, such as semantic navigation, multi-sensor fusion, and reinforcement learning navigation. 
- **Semantic Navigation**:  
  Combining semantic segmentation techniques to enable the rover to understand different areas of the environment (e.g. roads, obstacles, storage areas) and make smarter path-planning decisions.  
- **Multi-Sensor Fusion**:  
  Using the deep fusion of LiDAR, depth cameras, and IMU data to enhance localization accuracy and adapt to complex dynamic environments.  
- **Reinforcement Learning Navigation**:  
  Applying reinforcement learning to enable the rover to continuously optimize its path planning and obstacle avoidance strategies in unknown or dynamic environments, gradually forming autonomous learning and adaptability.

### Grasping and Placement
We can use **[Gazebo MoveIt](https://github.com/bjsowa/interbotix_ros_arms/tree/master)** to implement robotic arm movement (for detailed content, refer to the official Gazebo MoveIt documentation).  
In the future, we can explore combining **deep learning** and **real-time vision systems** to optimize grasping paths and reduce the failure rate. Additionally, we can investigate flexible designs and force feedback technologies, enabling the rover to safely and efficiently grasp objects of various shapes and materials.

### Target Detection
We can use **OpenCV** to process images, achieving target detection and assisting in grasping functionality.  
By combining **depth camera images** and **depth information**, the understanding of a target's position, size, and shape can be improved, achieving precise detection and classification.  
In the future, we may could employ advanced target detection algorithms (e.g., **YOLOv8**, **Mask R-CNN**) for multi-target real-time detection.  
Semantic analysis of the environment can help distinguish dynamic and static objects, ensuring detection reliability in complex scenarios.

Additionally, the following aspects need to be considered during the design phase and will be continuously improved during implementation:
- Design metrics to evaluate the efficiency and robustness of the software system.  
- Analyze potential risks and corresponding mitigation strategies.  
- Analyze potential failures (e.g., recognition failures, navigation path interruptions, grasping deviations).  
- Design corresponding error detection and recovery mechanisms:  
  - Timeout retry mechanisms.  
  - Error logging and analysis.  

---

## 5. Contact
For any queries or collaboration opportunities, feel free to reach out to us at the following emails:  
- [mingxiang.chen@student.manchester.ac.uk](mailto:mingxiang.chen@student.manchester.ac.uk)  
- [yu-chuan.liao@postgrad.manchester.ac.uk](mailto:yu-chuan.liao@postgrad.manchester.ac.uk)  
- [yunxue.pan@postgrad.manchester.ac.uk](mailto:yunxue.pan@postgrad.manchester.ac.uk)  
- [zhen.yang@student.manchester.ac.uk](mailto:zhen.yang@student.manchester.ac.uk)  

---

## 6. Acknowledgments
Special thanks to the University of Manchester and the Department of Robotics for providing support and resources for this project.
