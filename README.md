# Manchester Uni Robotic Systems Design Project Team5 (AERO62520)
# Introduction
Welcome to the GitHub repository of the Manchester Robotics Team5! 我们团队主要由四位充满热情且做事认真负责的成员构成：xxxx
我们这个project的目的是:Develop a robot which can autonomously retrieve coloured objects from the  environment and place them in matching storage bins located at the starting point.
机器人硬件方面主要组成：the Trossen Robotics Pincher X150 mobile arm, Slamtec RpLiDAR A2M12, pincherX150 manipulator, and Intel D435 RealSense depth camera，RGB camera等。
软件方面主要实现六大功能模块：Perception Module、Navigation Module、Grasping Module、User Interface Module、Communication Module、System Management Module。
Please note that the project is still in progress, so the code is incomplete.

# Table of Contents
1、环境设置
2、核心package
3、模块分析
4、未来展望
5、Contact
6、Acknowledgments

1、环境设置
1）可参阅 leo rover的官方网站上提供的文档，了解组装过程和有关 leo rover的相关信息。
2）Ubuntu系统上，下载安装ROS2 Humble and Gazebo (preferably ignition version)
3）official ROS website上clone the leo simulator-ros2
4）Create a python package and sourcing its setup file
ros2 pkg create --build-type ament_python <package_name>
source install/local_setup.bash
5）克隆我们的版本库，并将其放在你的软件包中，然后编译 Python 软件包并获取其设置文件。

2、核心package
RPLIDAR ROS - https://github.com/Slamtec/rplidar_ros
SLAM-Toolbox - https://github.com/SteveMacenski/slam_toolbox
Nav2 - https://index.ros.org/p/nav2_map_server
REALSENSE2 - https://github.com/IntelRealSense/realsense-ros
interbotix_ros_manipulators - https://github.com/Interbotix/interbotix_ros_manipulators
Wavefront Exploration - https://github.com/gjcliff/SLAM-Frontier-Exploration

3、核心模块
## Navigation Module
Based on leo_navigation tutorials，我们初步设计如下，详细代码还没有完成：
我们计划利用slam_toolbox构建动态环境的实时地图，以便探索未知领域，同时进行局部定位。并且结合EKF（扩展卡尔曼滤波）融合里程计、IMU和LiDAR数据，提升定位的平滑性与精确性。目标检测算法暂时还没有定，Global Path Planning倾向于使用A*算法，可简单且能高效找到最短路径，Local Path Planning倾向于TEB。核心节点包括/planner_server、/controller_server、/bt_navigator实现路径规划及导航，/slam_toolbox负责实时地图构建（SLAM）和定位，同时还有/velocity_smother速度平滑节点，/explore_node节点用于导航过程中的目标检测及避障等相关功能。通过/odm,/speed_limit,/map，/joint_states等话题通信，实时更新/cmd_vel，/pose等，实现机器人自主导航、避障、动态调整路径、导航中的目标检测等功能。
计划用到的Navigation相关package及文档：
odometry相关：ekf<https://wiki.ros.org/robot_localization>,imu<https://wiki.ros.org/imu_filter_madgwick>
mapping相关：
twist_mux相关：
amcl相关：
move_base相关：
通过编写相关launch，并将其组合起来，可实现简单的Navigation功能。

## Grasping Module
我们使用Manipulator PincherX150，关于此机械臂相关的necessary software libraries，请按照这个链接下载：
其中软件包包含了必要的配置和启动文件，可让众多 Interbotix X 系列机械臂与感知管道配合使用。机械臂可以从平整的表面拾取 RealSense 彩色/深度摄像头视场内的任何小型非反射物体，故而使用此机械臂用于实现我们机器人项目中的抓取目标任务。
详细内容请参考：interbotix_perception_modules ROS package

4、未来展望
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

5、Contact
For any queries or collaborations, feel free to contact us at emails below.
alexander.morley@student.manchester.ac.uk
michalis.iakovides@student.manchester.ac.uk
geetik.mamillapalli@postgrad.manchester.ac.uk
ziyi.cheng@student.manchester.ac.uk
6、Acknowledgments
特别感谢曼彻斯特大学和机器人系为本项目提供的支持和资源。

