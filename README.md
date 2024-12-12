# Manchester Uni Robotic Systems Design Project Team5 (AERO62520)
## Introduction
Welcome to the GitHub repository of Manchester Robotics Team5!
Our team consists of four passionate and responsible members: xxxx.
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
1. Environment Setup  
2. Core Packages  
3. Module Analysis  
4. Future Prospects  
5. Contact  
6. Acknowledgments  

## 1. Environment Setup
- Refer to the documentation provided on the official [leo rover](https://docs.fictionlab.pl/leo-rover) website to understand the assembly process and relevant information about Leo Rover.  
- Install **ROS 2 Humble** and **[Gazebo](https://gazebosim.org/api/gazebo/6.1/install.html)** on Ubuntu.  
- Clone the `[leo_simulator-ros2](https://github.com/LeoRover/leo_simulator-ros2)` from the official ROS website.  
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

Core nodes include:
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

### 1. Navigation
To achieve full autonomy for the Leo Rover, we can introduce more advanced navigation technologies, such as semantic navigation, multi-sensor fusion, and reinforcement learning navigation. 
- **Semantic Navigation**:  
  Combining semantic segmentation techniques to enable the rover to understand different areas of the environment (e.g. roads, obstacles, storage areas) and make smarter path-planning decisions.  
- **Multi-Sensor Fusion**:  
  Using the deep fusion of LiDAR, depth cameras, and IMU data to enhance localization accuracy and adapt to complex dynamic environments.  
- **Reinforcement Learning Navigation**:  
  Applying reinforcement learning to enable the rover to continuously optimize its path planning and obstacle avoidance strategies in unknown or dynamic environments, gradually forming autonomous learning and adaptability.

### 2. Grasping and Placement
We can use **[Gazebo MoveIt](https://github.com/bjsowa/interbotix_ros_arms/tree/master)** to implement robotic arm movement (for detailed content, refer to the official Gazebo MoveIt documentation).  
In the future, we can explore combining **deep learning** and **real-time vision systems** to optimize grasping paths and reduce the failure rate. Additionally, we can investigate flexible designs and force feedback technologies, enabling the rover to safely and efficiently grasp objects of various shapes and materials.

### 3. Target Detection
We can use **OpenCV** to process images, achieving target detection and assisting in grasping functionality.  
By combining **depth camera images** and **depth information**, the understanding of a target's position, size, and shape can be improved, achieving precise detection and classification.  
In the future, we may could employ advanced target detection algorithms (e.g., **YOLOv8**, **Mask R-CNN**) for multi-target real-time detection.  
Semantic analysis of the environment can help distinguish dynamic and static objects, ensuring detection reliability in complex scenarios.

---

## 5. Contact
For any queries or collaboration opportunities, feel free to reach out to us at the following emails:  
- alexander.morley@student.manchester.ac.uk  
- michalis.iakovides@student.manchester.ac.uk  
- geetik.mamillapalli@postgrad.manchester.ac.uk  
- ziyi.cheng@student.manchester.ac.uk  

---

## 6. Acknowledgments
Special thanks to the University of Manchester and the Department of Robotics for providing support and resources for this project.

# 3、核心模块
## Navigation Module
Based on leo_navigation tutorials，我们初步设计如下，详细代码还没有完成：
我们计划利用slam_toolbox构建动态环境的实时地图，以便探索未知领域，同时进行局部定位。并且结合EKF（扩展卡尔曼滤波）融合里程计、IMU和LiDAR数据，提升定位的平滑性与精确性。目标检测算法暂时还没有定，Global Path Planning倾向于使用A*算法，可简单且能高效找到最短路径，Local Path Planning倾向于TEB。核心节点包括/planner_server、/controller_server、/bt_navigator实现路径规划及导航，/slam_toolbox负责实时地图构建（SLAM）和定位，同时还有/velocity_smother速度平滑节点，/explore_node节点用于导航过程中的目标检测及避障等相关功能。通过/odm,/speed_limit,/map，/joint_states等话题通信，实时更新/cmd_vel，/pose等，实现机器人自主导航、避障、动态调整路径、导航中的目标检测等功能。
计划用到的Navigation相关package及文档：
odometry相关：[ekf](https://wiki.ros.org/robot_localization),[imu](https://wiki.ros.org/imu_filter_madgwick)  
mapping相关：
twist_mux相关：
amcl相关：
move_base相关：
通过编写相关launch，并将其组合起来，可实现简单的Navigation功能。

## Grasping Module
我们使用Manipulator PincherX150，关于此机械臂相关的necessary software libraries，请按照这个链接下载：
其中软件包包含了必要的配置和启动文件，可让众多 Interbotix X 系列机械臂与感知管道配合使用。机械臂可以从平整的表面拾取 RealSense 彩色/深度摄像头视场内的任何小型非反射物体，故而使用此机械臂用于实现我们机器人项目中的抓取目标任务。
详细内容请参考：interbotix_perception_modules ROS package

# 4、未来展望
1）导航
为了实现 Leo Rover 的完全自主功能，我们可以通过引入更高级的导航技术，例如语义导航、多传感器融合和强化学习导航等。
—语义导航：结合语义分割技术，Rover 能够理解环境中的不同区域（如道路、障碍物、存储区域等），从而制定更加智能的路径规划。
-多传感器融合：利用 LiDAR、深度摄像头和 IMU 数据的深度融合，提升定位精度和对复杂动态环境的适应能力。
-强化学习导航：通过强化学习，Rover 可以在未知或动态环境中不断优化其路径规划和避障策略，逐渐形成自主学习和适应能力。
2）机械臂抓取及放置
我们可以利用 Gazebo MoveIt 实现机械臂的运动，详细内容请参考Gazebo MoveIt官方文档内容。
未来，可以探索结合深度学习和实时视觉系统，优化抓取路径，减少抓取失败的概率。探索机械臂的柔性设计和力反馈技术，使 Rover 能够安全高效地抓取多种形状和材质的物体。
3）目标检测
我们可以使用 OpenCV 处理图像，以实现目标检测和抓取功能。通过结合 depth camera 图像和深度信息，提升对目标位置、大小和形状的理解，实现精准检测与分类。未来，还可结合使用先进的目标检测算法（如 YOLOv8、Mask R-CNN），实现多目标实时识别，通过环境语义分析还可以区分动态和静态物体，确保在复杂场景中的检测可靠性。

# 5、Contact
For any queries or collaborations, feel free to contact us at emails below.
alexander.morley@student.manchester.ac.uk
michalis.iakovides@student.manchester.ac.uk
geetik.mamillapalli@postgrad.manchester.ac.uk
ziyi.cheng@student.manchester.ac.uk
6、Acknowledgments
特别感谢曼彻斯特大学和机器人系为本项目提供的支持和资源。

