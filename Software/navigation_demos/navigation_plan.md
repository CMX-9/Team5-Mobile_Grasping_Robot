
We plan to use `slam_toolbox` to build real-time maps of dynamic environments, enabling exploration of unknown areas while performing local localization. Additionally, we will integrate **EKF (Extended Kalman Filter)** to fuse odometry, IMU, and LiDAR data, improving the smoothness and accuracy of localization. The target detection algorithm has not yet been decided. For **Global Path Planning**, we tend to use the **A*** algorithm, which is simple and efficient in finding the shortest path, while for **Local Path Planning**, we plan to use **TEB**.

### Core Nodes:
- `/planner_server`, `/controller_server`, and `/bt_navigator` for path planning and navigation.
- `/slam_toolbox` for real-time SLAM and localization.
- `/velocity_smoother` for velocity smoothing.
- `/explore_node` for target detection and obstacle avoidance during navigation.

### Communication Topics:
- Use topics such as `/odom`, `/speed_limit`, `/map`, and `/joint_states` to update `/cmd_vel` and `/pose` in real time.
- These updates enable:
  - Autonomous navigation
  - Obstacle avoidance
  - Dynamic path adjustment
  - Target detection during navigation


We will refer to the following resources（Only a portion is listed here; others will be added later）：
Wavefront Exploration - https://github.com/gjcliff/SLAM-Frontier-Exploration  
SLAM-Toolbox - https://github.com/SteveMacenski/slam_toolbox  
Nav2 - https://index.ros.org/p/nav2_map_server

![GitHub Logo](https://github.githubassets.com/images/modules/logos_page/GitHub-Mark.png "GitHub Logo")
![GitHub Logo](https://github.githubassets.com/images/modules/logos_page/GitHub-Mark.png "GitHub Logo")


