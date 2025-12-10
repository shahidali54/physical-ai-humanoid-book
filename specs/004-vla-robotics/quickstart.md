# Quickstart Guide: Module 4 - Vision-Language-Action (VLA) Robotics

## Overview
This guide provides a quick introduction to Vision-Language-Action (VLA) robotics concepts, including the integration of speech recognition (Whisper), cognitive planning with LLMs, and ROS 2 action graph execution for humanoid robots.

## Prerequisites
- Python 3.8+ with pip
- ROS 2 Humble Hawksbill installed
- OpenAI Whisper (or similar ASR system)
- Access to a large language model (GPT, Claude, or open-source alternative)
- Humanoid robot simulator or real robot with ROS 2 interface

## Setting up the VLA Pipeline

### 1. Install Dependencies
```bash
# Install Whisper for speech recognition
pip install openai-whisper

# Install ROS 2 Python libraries
pip install rclpy

# Install LLM interface (example with OpenAI)
pip install openai

# Install additional dependencies
pip install numpy pyaudio speech_recognition
```

### 2. Configure Voice-to-Action Pipeline
```python
# Example Python script for basic VLA setup
import whisper
import openai
import rclpy
from rclpy.action import ActionClient
from std_msgs.msg import String

# Initialize Whisper model
model = whisper.load_model("base")

# Configure LLM interface
client = openai.OpenAI()  # or your preferred LLM client

# Initialize ROS 2 node
rclpy.init()
node = rclpy.create_node('vla_pipeline')
```

### 3. Process Voice Input
```bash
# Record and process audio
python -c "
import pyaudio
import wave
import whisper

# Record audio
# Process with Whisper
result = whisper.transcribe('audio_file.wav', model='base')
print('Transcript:', result['text'])
"
```

## Using LLM for Task Decomposition

### 1. Set up LLM Interface
```bash
# Example using OpenAI API
export OPENAI_API_KEY='your-api-key'
```

### 2. Decompose Complex Commands
```python
def decompose_task(command_text):
    prompt = f"""
    Decompose the following high-level command into specific, executable robot actions:
    Command: '{command_text}'

    Provide the result as a sequence of simple actions that a humanoid robot can execute.
    Each action should be specific and actionable.

    Example format:
    1. Navigate to [location]
    2. Detect [object]
    3. Grasp [object]
    4. Transport [object] to [location]
    5. Release [object]
    """

    response = client.chat.completions.create(
        model="gpt-3.5-turbo",  # or your preferred model
        messages=[{"role": "user", "content": prompt}]
    )

    return response.choices[0].message.content
```

## Configuring ROS 2 Action Graphs

### 1. Define Action Types
```python
# Example action definitions for humanoid robot
from rclpy.action import ActionClient
from example_interfaces.action import NavigateToPose
from example_interfaces.action import ManipulateObject

# Action clients for different robot capabilities
navigation_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
manipulation_client = ActionClient(node, ManipulateObject, 'manipulate_object')
```

### 2. Execute Action Sequences
```python
def execute_action_sequence(action_list):
    """Execute a sequence of actions from LLM decomposition"""
    for action in action_list:
        if action.startswith("Navigate to"):
            # Execute navigation action
            pass
        elif action.startswith("Detect"):
            # Execute perception action
            pass
        elif action.startswith("Grasp"):
            # Execute manipulation action
            pass
        # Add more action types as needed
```

## Complete Pipeline: Voice → Language → Action

### 1. End-to-End Workflow
```bash
# Terminal 1: Start ROS 2 environment
source /opt/ros/humble/setup.bash
ros2 run your_robot_bringup robot_server

# Terminal 2: Run VLA pipeline
python vla_pipeline.py

# Terminal 3: Send voice command (or use text for testing)
echo "Clean the room" | python send_command.py
```

### 2. Example Voice Command Processing
```
Voice Input: "Pick up the red cup and put it on the table"
↓
Whisper ASR: "Pick up the red cup and put it on the table"
↓
LLM Processing:
- Navigate to cup location
- Detect red cup
- Grasp cup
- Navigate to table
- Place cup on table
↓
ROS 2 Action Graph:
- Navigation action → Perception action → Manipulation action → Navigation action → Placement action
↓
Robot Execution: Physical actions performed by humanoid robot
```

## Troubleshooting

### Common Issues:
- **High latency**: Check network connection for LLM API calls
- **Voice recognition errors**: Ensure quiet environment and clear speech
- **Action failures**: Verify robot capabilities match planned actions
- **Synchronization issues**: Ensure ROS 2 nodes are properly connected

### Performance Tips:
- Use smaller Whisper models for faster processing
- Cache LLM responses for common commands
- Implement timeout mechanisms for long-running actions
- Use simulated environments for initial testing