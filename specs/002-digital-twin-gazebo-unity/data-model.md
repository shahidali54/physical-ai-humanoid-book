# Data Model: Digital Twin (Gazebo & Unity)

## Digital Twin Entity
- **Name**: String (unique identifier for the digital twin)
- **Description**: Text (explanation of the twin's purpose)
- **Physics Properties**: Object (gravity, mass, friction, collision parameters)
- **Visual Properties**: Object (materials, textures, 3D model reference)
- **Sensor Configuration**: Array (list of simulated sensors with parameters)

## Simulation Environment Entity
- **Type**: Enum (Gazebo | Unity)
- **Configuration**: Object (environment settings, physics parameters)
- **Assets**: Array (models, textures, world files)
- **Supported Sensors**: Array (list of sensor types supported by this environment)

## Sensor Simulation Entity
- **Type**: Enum (Camera | Lidar | IMU | [other sensor types])
- **Parameters**: Object (specific to each sensor type)
- **Output Format**: String (format of the simulated data)
- **Environment Compatibility**: Array (list of compatible simulation environments)

## Humanoid Model Entity
- **Name**: String (identifier for the humanoid model)
- **Joint Configuration**: Array (list of joints with properties)
- **Visual Mesh**: String (reference to 3D model)
- **Physical Properties**: Object (mass, collision shapes)
- **Control Interface**: Object (input/output mappings for interaction)