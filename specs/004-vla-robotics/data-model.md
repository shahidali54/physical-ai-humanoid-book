# Data Model: Module 4 - Vision-Language-Action (VLA) Robotics

## Overview
This document defines the key entities and data structures relevant to the Vision-Language-Action (VLA) robotics system for educational content in Module 4.

## Entity: Voice Command
**Description**: Natural language instruction provided through speech that initiates the VLA pipeline
**Fields**:
- `id`: Unique identifier for the voice command
- `audio_data`: Raw audio data or reference to audio file
- `transcript`: Text transcription of the spoken command
- `confidence_score`: Confidence level of speech recognition (0.0-1.0)
- `timestamp`: Time when command was received
- `language`: Language of the spoken command
- `command_type`: Classification of command type (navigation, manipulation, etc.)
- `processed_status`: Whether the command has been processed (pending, processing, completed, failed)
**Relationships**: Transforms to Cognitive Plan via LLM processing

## Entity: Cognitive Plan
**Description**: LLM-generated sequence of steps that decomposes high-level goals into executable actions
**Fields**:
- `id`: Unique identifier for the cognitive plan
- `source_command`: Reference to the original voice command
- `task_decomposition`: List of subtasks that decompose the high-level goal
- `action_sequence`: Ordered sequence of robot actions to execute
- `execution_priority`: Priority level for action execution
- `estimated_duration`: Estimated time to complete the plan
- `resource_requirements`: Robot resources needed (manipulation, navigation, etc.)
- `error_recovery_plan`: Steps to take if an action fails
**Relationships**: Generated from Voice Command, executed as Action Graph

## Entity: Action Graph
**Description**: ROS 2 structure that coordinates multiple robot capabilities to execute complex tasks
**Fields**:
- `id`: Unique identifier for the action graph
- `root_node`: Starting node of the action graph
- `nodes`: Collection of action nodes in the graph
- `edges`: Dependencies and connections between nodes
- `execution_status`: Current status of graph execution (pending, running, completed, failed)
- `start_time`: Time when execution started
- `end_time`: Time when execution completed
- `execution_log`: Log of actions executed and their outcomes
- `feedback_channels`: Communication channels for real-time feedback
**Relationships**: Represents execution of Cognitive Plan, connects to ROS 2 action servers

## Entity: VLA Pipeline
**Description**: Integrated system connecting vision (perception), language (understanding), and action (execution) components
**Fields**:
- `id`: Unique identifier for the pipeline instance
- `vision_component`: Reference to perception system
- `language_component`: Reference to speech and text processing system
- `action_component`: Reference to robot control system
- `integration_status`: Status of component integration (active, paused, error)
- `performance_metrics`: Latency, accuracy, and success rate metrics
- `user_session`: Associated user interaction session
- `configuration`: Pipeline configuration parameters
**Relationships**: Coordinates all other entities in the complete VLA workflow

## Entity: Action Node
**Description**: Individual action within an action graph that can be executed by the robot
**Fields**:
- `id`: Unique identifier for the action node
- `action_type`: Type of action (navigation, manipulation, perception, etc.)
- `parameters`: Parameters required for action execution
- `dependencies`: Other nodes that must complete before this node can start
- `status`: Current status (pending, executing, completed, failed)
- `result`: Result of action execution
- `timeout`: Maximum time allowed for action completion
- `retries_allowed`: Number of retry attempts if action fails
**Relationships**: Part of Action Graph, connects to ROS 2 action servers

## Entity: Perception Data
**Description**: Environmental data collected by vision systems to support VLA decision making
**Fields**:
- `id`: Unique identifier for the perception data
- `sensor_data`: Raw data from cameras, lidars, or other sensors
- `object_detections`: List of detected objects with positions and properties
- `spatial_map`: Spatial understanding of the environment
- `timestamp`: Time when data was collected
- `confidence_scores`: Confidence levels for each detection
- `semantic_labels`: Semantic understanding of the scene
- `tracking_ids`: Tracking identifiers for moving objects
**Relationships**: Used by Cognitive Plan for context-aware task decomposition