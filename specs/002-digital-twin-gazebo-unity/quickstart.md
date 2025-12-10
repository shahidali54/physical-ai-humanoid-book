# Quickstart Guide: Digital Twin Development with Gazebo & Unity

## Prerequisites
- ROS 2 Humble Hawksbill installed
- Gazebo Garden or compatible version
- Unity Hub with Unity 2022.3 LTS or newer
- Git for version control
- Python 3.10+ for ROS 2 packages

## Setting Up the Environment

### Gazebo Setup
1. Install ROS 2 Humble following official documentation
2. Install Gazebo Garden: `sudo apt install ros-humble-gazebo-*`
3. Verify installation: `gazebo --version`
4. Set up ROS 2 workspace for simulation packages

### Unity Setup
1. Download and install Unity Hub
2. Install Unity 2022.3 LTS through Unity Hub
3. Install required packages via Package Manager:
   - Physics package
   - XR Interaction Toolkit (if needed)
4. Set up project template for robotics simulation

## Running the Examples

### Gazebo Physics Example
1. Navigate to the example directory
2. Source ROS 2 environment: `source /opt/ros/humble/setup.bash`
3. Launch the simulation: `ros2 launch gazebo_examples physics_demo.launch.py`
4. Observe gravity and collision behavior

### Unity Visualization Example
1. Open the Unity project in Unity Hub
2. Navigate to the humanoid scene
3. Press Play to start the simulation
4. Use input controls to interact with the humanoid model

### Sensor Simulation Example
1. Run both Gazebo and Unity instances
2. Launch the sensor bridge: `ros2 run sensor_examples sensor_bridge_node`
3. Observe synchronized sensor data from both environments