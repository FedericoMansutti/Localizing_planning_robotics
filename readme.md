# ROS Navigation and Mapping Project

A ROS package for autonomous robot navigation and mapping using the Scout Mini robot configured with Mecanum wheels for omnidirectional movement.

## Authors
- Federico Mansutti (10683462)
- Niccolò Guariglia (10668501)

## Overview

This project implements two main functionalities:
1. **Mapping**: Real-time map generation using gmapping with LiDAR data
2. **Navigation**: Autonomous waypoint navigation using move_base

## Features

- **3D to 2D Point Cloud Conversion**: Converts RSLiDAR 3D point cloud data to 2D laser scan for mapping
- **SLAM Mapping**: Uses gmapping for Simultaneous Localization and Mapping
- **Autonomous Navigation**: Waypoint-based navigation with obstacle avoidance
- **Dynamic Path Planning**: Utilizes DWA (Dynamic Window Approach) local planner
- **Localization**: AMCL (Adaptive Monte Carlo Localization) for robot pose estimation

## Prerequisites

- ROS (tested on ROS Noetic/Melodic)
- Required ROS packages:
  - `gmapping`
  - `move_base`
  - `amcl`
  - `pointcloud_to_laserscan`
  - `map_server`
  - `stage_ros` (for simulation)
  - `turtlebot3` packages (for robot model)

### Installing Dependencies

```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-gmapping \
                     ros-$ROS_DISTRO-move-base \
                     ros-$ROS_DISTRO-amcl \
                     ros-$ROS_DISTRO-pointcloud-to-laserscan \
                     ros-$ROS_DISTRO-map-server \
                     ros-$ROS_DISTRO-stage-ros \
                     ros-$ROS_DISTRO-turtlebot3*
```

## Installation

1. Clone this repository into your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone <repository-url> second_project
```

2. Build the package:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Project Structure

```
second_project/
├── cfg/                    # Configuration files
│   ├── costmap_common_params.yaml
│   ├── dwa_local_planner_params.yaml
│   ├── global_costmap_params.yaml
│   ├── local_costmap_params.yaml
│   ├── move_base_params.yaml
│   └── slam_config_3D.yaml
├── launch/                 # Launch files
│   ├── mapping.launch      # For mapping task
│   ├── navigation.launch   # For navigation task
│   ├── turtlebot_amcl.launch
│   └── amcl.launch.xml
├── maps/                   # Map files
├── rviz/                   # RViz configuration files
│   ├── mapping.rviz
│   └── robot_navigation.rviz
├── src/                    # Source code
│   ├── tf_publisher.cpp    # Publishes TF transforms from odometry
│   ├── base_to_lidar.cpp   # Publishes base_link to rslidar transform
│   └── navigation.cpp      # Waypoint navigation node
└── waypoints.csv          # Navigation waypoints

```

## Usage

### Task 1: Mapping

To create a map of the environment:

```bash
roslaunch second_project mapping.launch
```

This will:
- Start the pointcloud to laserscan conversion
- Launch gmapping for SLAM
- Publish necessary TF transforms
- Open RViz for visualization

Save the generated map using:
```bash
rosrun map_server map_saver -f <map_name>
```

### Task 2: Navigation

To navigate through predefined waypoints:

```bash
roslaunch second_project navigation.launch
```

This will:
- Load the pre-built map
- Start the Stage simulator with the robot model
- Launch move_base for path planning
- Start AMCL for localization
- Execute waypoint navigation from `waypoints.csv`

## Configuration

### Robot Configuration
- Robot: Scout Mini with Mecanum wheels (omnidirectional)
- Footprint: [[-0.0306, -0.029], [0.0306, -0.029], [0.0306, 0.029], [-0.0306, 0.029]]

### Navigation Parameters
- Max velocity: 0.5 m/s
- Max rotational velocity: 3.0 rad/s
- Obstacle range: 2.5 m
- Raytrace range: 3.0 m

### Waypoints
Waypoints are defined in `waypoints.csv` with format:
```
x_position, y_position, orientation_z
```

Example:
```
26.3,16.0,-0.7
26.5,13.0,-1.0
30.0,12.0,0.0
```

## Key Components

### TF Publisher (`tf_publisher.cpp`)
Converts odometry messages to TF transforms for the robot's base frame.

### Base to LiDAR Transform (`base_to_lidar.cpp`)
Publishes the static transform between the robot base and the RSLiDAR sensor.

### Navigation Node (`navigation.cpp`)
Reads waypoints from CSV file and sends sequential navigation goals to move_base.

## Sensors and Hardware

### LiDAR Configuration
- **Sensor**: RSLiDAR (3D LiDAR)
- **Point Cloud Topic**: `/ugv/rslidar_points`
- **Converted Laser Scan Topic**: `/ugv/rslidar_points_tolaser`
- **Height Filter**: 0.2m - 0.6m (for ground robot navigation)

### Odometry
- **Odometry Topic**: `/ugv/odom`
- **TF Frames**: `odom` → `base_link` → `rslidar`

## Customization

### Modifying Navigation Waypoints
Edit `waypoints.csv` to define your custom navigation path.

### Adjusting Robot Parameters
Modify the configuration files in the `cfg/` directory:
- `dwa_local_planner_params.yaml`: Local planner settings
- `costmap_common_params.yaml`: Obstacle detection parameters
- `move_base_params.yaml`: Navigation behavior settings

### Changing Initial Position
In `navigation.launch`, modify:
```xml
<arg name="initial_pose_x" default="26.4"/>
<arg name="initial_pose_y" default="16.05"/>
<arg name="initial_pose_a" default="0.0"/>
```

## Simulation Environment

The project uses Stage simulator for testing navigation:
- **World File**: Located in `maps/stage/map.world`
- **Robot Model**: TurtleBot3 Burger (adapted for Scout Mini kinematics)
- **Simulation Time**: Enabled (`use_sim_time = true`)

## Performance Considerations

- **Map Update Rate**: 3.0 Hz (configurable in `slam_config_3D.yaml`)
- **Controller Frequency**: 5.0 Hz
- **Transform Publish Rate**: 50 Hz (0.02s period)
- **Local Costmap**: 6x6 meters, 0.05m resolution
- **Stack Size**: Increased to 40MB for large map serialization

## Troubleshooting

1. **Robot not moving**: Check that all TF frames are being published correctly using:
   ```bash
   rosrun tf view_frames
   ```

2. **Poor localization**: Increase AMCL particle count in `amcl.launch.xml`

3. **Navigation failures**: Adjust costmap parameters or increase `controller_patience` in `move_base_params.yaml`

## License

TODO

## Acknowledgments

This project was developed as part of a robotics course assignment focusing on autonomous navigation and mapping capabilities.