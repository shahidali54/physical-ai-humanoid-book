# Quickstart Guide: Module 3 - AI-Robot Brain (NVIDIA Isaac™)

## Overview
This guide provides a quick introduction to the NVIDIA Isaac ecosystem components: Isaac Sim for simulation, Isaac ROS for perception/navigation, and Nav2 for path planning.

## Prerequisites
- NVIDIA GPU with CUDA support (RTX series recommended)
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- NVIDIA Isaac Sim installation
- Isaac ROS packages
- Navigation2 (Nav2)

## Setting up Isaac Sim Environment

### 1. Create a Basic Scene
```bash
# Navigate to Isaac Sim directory
cd ~/isaac-sim
./isaac-sim.bat  # On Windows
# or
./isaac-sim.sh   # On Linux

# Launch Isaac Sim and create a new USD scene
# Add a robot model (e.g., URDF import)
# Configure sensors (camera, lidar)
```

### 2. Configure Synthetic Data Generation
```python
# Example Python script for Isaac Sim
import omni
from omni.isaac.synthetic_data.scripts import capture

# Configure synthetic data sensors
# Set up depth, segmentation, bounding box generation
# Run data capture sequence
```

### 3. Export Synthetic Dataset
```bash
# Export dataset in standard formats
# Images, point clouds, annotations
# Format: COCO, KITTI, or custom
```

## Using Isaac ROS for VSLAM

### 1. Launch Isaac ROS VSLAM Pipeline
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Launch VSLAM nodes
ros2 launch isaac_ros_visual_slam visual_slam.launch.py
```

### 2. Process Sensor Data
```bash
# Subscribe to sensor topics
ros2 topic echo /camera/color/image_raw
ros2 topic echo /imu/data

# Monitor VSLAM output
ros2 topic echo /visual_slam/tracking/pose_graph
```

### 3. Create Maps
```bash
# Combine VSLAM pose estimates with sensor data
# Generate 2D/3D maps
# Save for Nav2 navigation
```

## Configuring Nav2 for Humanoid Navigation

### 1. Set Up Humanoid-Specific Parameters
```yaml
# humanoid_nav2_config.yaml
local_costmap:
  local_costmap:
    plugins: ["obstacle_layer", "inflation_layer"]
    # Humanoid-specific obstacle inflation
    inflation_layer:
      inflation_radius: 0.8  # Larger for humanoid footprint

global_costmap:
  global_costmap:
    plugins: ["obstacle_layer", "inflation_layer"]
    # Humanoid-specific parameters
```

### 2. Launch Navigation
```bash
# Launch Nav2 with humanoid configuration
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=~/humanoid_nav2_config.yaml
```

### 3. Send Navigation Goals
```bash
# Send navigation goal
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}, header: {frame_id: 'map'}}}"
```

## Complete Pipeline: Perception → Mapping → Planning

### 1. End-to-End Workflow
```bash
# Terminal 1: Launch Isaac Sim with scene
./isaac-sim.sh

# Terminal 2: Run Isaac ROS VSLAM
ros2 launch isaac_ros_visual_slam visual_slam.launch.py

# Terminal 3: Run Nav2 navigation
ros2 launch nav2_bringup navigation_launch.py

# Terminal 4: Send navigation commands
ros2 action send_goal /navigate_to_pose ...
```

### 2. Validation
- Verify synthetic data generation from Isaac Sim
- Confirm VSLAM creates accurate maps
- Test navigation execution with humanoid constraints

## Troubleshooting

### Common Issues:
- **GPU Memory**: Ensure sufficient VRAM for Isaac Sim rendering
- **ROS 2 Connection**: Verify all nodes are on the same ROS domain
- **Sensor Calibration**: Check Isaac ROS sensor configurations match Isaac Sim

### Performance Tips:
- Use RTX 3080 or better for real-time performance
- Limit simulation complexity for real-time VSLAM
- Optimize Nav2 parameters for humanoid dynamics