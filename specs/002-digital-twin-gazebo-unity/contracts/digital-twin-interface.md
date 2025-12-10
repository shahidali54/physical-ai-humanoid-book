# Digital Twin Interface Contract

## Purpose
This contract defines the interface between the educational content and the simulation environments (Gazebo and Unity) for the digital twin module.

## Endpoints

### GET /simulation/gazebo/{example}
- **Description**: Retrieve Gazebo simulation example configuration
- **Parameters**:
  - example: String (name of the example to retrieve)
- **Response**:
  - 200: Simulation configuration file (URDF, world file, or launch file)
  - 404: Example not found

### GET /simulation/unity/{scene}
- **Description**: Retrieve Unity scene configuration
- **Parameters**:
  - scene: String (name of the scene to retrieve)
- **Response**:
  - 200: Scene configuration and assets
  - 404: Scene not found

### POST /simulation/run
- **Description**: Execute a simulation with specified parameters
- **Request Body**:
  - environment: String (gazebo | unity)
  - example: String (name of example to run)
  - parameters: Object (physics, visual, or sensor parameters)
- **Response**:
  - 200: Simulation started successfully
  - 400: Invalid parameters
  - 500: Simulation environment error

### GET /sensors/{type}/data
- **Description**: Retrieve simulated sensor data
- **Parameters**:
  - type: String (camera | lidar | imu)
- **Response**:
  - 200: Sensor data in appropriate format
  - 404: Sensor type not supported