# Obstacle Avoidance & Mapping Robot using ROS 2

This project demonstrates SLAM, obstacle avoidance, and GPIO-based motor control on a Raspberry Pi using ROS 2 Humble, RPLIDAR, and Python-based hardware interfacing.

## Project Overview

| Feature                  | Description                                     |
|--------------------------|-------------------------------------------------|
| Platform                 | Raspberry Pi + ROS 2 Humble (Docker-based)     |
| Sensor                   | RPLIDAR C1                                      |
| Motor Control            | GPIO + Python (lgpio or pigpio)                |
| SLAM                     | slam_toolbox                                    |
| Localization             | AMCL (nav2)                                     |
| Obstacle Avoidance       | Custom node reacting to /scan messages         |
| LoRa Goals (optional)    | Listen to /lora_rx and convert to /goal_pose    |

## Folder Structure

ros2_wrksp/
├── ros2_ws/
│   └── src/
│       ├── rplidar_ros/
│       ├── slam_map_launch/
│       ├── motor_interface/
│       ├── obstacle_avoidance/
│       └── ...
└── ros2_nav_ws/
    └── src/
        ├── nav2_bringup_custom/
        └── path_follow_with_avoidance/

## Setup Instructions

### 1. Clone and Build
```bash
cd ~/ros2_wrksp/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Start LIDAR
```bash
ros2 run rplidar_ros rplidar_composition   --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=460800   -p frame_id:=laser -p scan_mode:=Standard &
```

### 3. Run Static Transforms
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link laser &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint &
```

### 4. Run SLAM Toolbox
```bash
ros2 run slam_toolbox async_slam_toolbox_node   --ros-args -p use_sim_time:=false -p provide_odom_frame:=false   -p base_frame:=base_link -p map_frame:=map -p scan_topic:=scan &
```

### 5. Save Map
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap   "{name: {data: '/ros2_wrksp/ros2_ws/maps/full_room_map'}}"
```

## Localization & Navigation

### 1. Launch Nav2 Localization
```bash
cd ~/ros2_wrksp/ros2_nav_ws
ros2 launch nav2_bringup_custom nav2_localization.launch.py
```

### 2. Send Goal Pose (Manual or from LoRa)
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "..."  # or use lora_listener node
```

## Motor Control (GPIO-based)

### 1. Wiring

| Function          | RPi Pin | Motor Wire   |
|------------------|---------|--------------|
| Left DIR         | 16 (GPIO23) | DIR Left     |
| Left PWM         | 12 (GPIO18) | EN Left      |
| Right DIR        | 18 (GPIO24) | DIR Right    |
| Right PWM        | 35 (GPIO19) | EN Right     |

Ensure you have pigpio or lgpio running:
```bash
sudo pigpiod
```

### 2. Run Motor Driver Node
```bash
ros2 run motor_interface cmd_vel_to_gpio
```

## Obstacle Avoidance

### Run custom avoidance node
```bash
ros2 run obstacle_avoidance obstacle_avoider
```

## Test Path Following (optional)
```bash
ros2 run path_follow_with_avoidance path_follower
```

## RViz Visualization

On your host PC:
```bash
rviz2
```

Add the following displays:
- LaserScan: /scan
- TF
- Map: /map
- Pose: /amcl_pose

## Troubleshooting

- No `/map` topic? Ensure SLAM Toolbox is running and map is being updated
- Transform errors? Check all static transform publishers are running
- GPIO error? Ensure pigpiod is running and correct pins used

## Hardware Used

- Raspberry Pi 4
- RPLIDAR C1
- TB6612FNG motor driver
- BO motors with encoders
- LoRa SX1278 module (optional)

## Authors

- Your Name
- Guided by: Prof. XYZ (if applicable)

## License

MIT License or your preferred license