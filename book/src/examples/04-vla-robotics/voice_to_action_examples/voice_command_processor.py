#!/usr/bin/env python3
"""
Voice Command Processor for VLA Robotics

This module processes voice commands using Whisper ASR and converts them
into actionable robot commands for the VLA pipeline.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import AudioData
from builtin_interfaces.msg import Duration
import numpy as np
import whisper
import torch
import threading
import queue
import time
from typing import Dict, Any, Optional


class VoiceCommandProcessorNode(Node):
    """
    ROS 2 node for processing voice commands using Whisper ASR
    """

    def __init__(self):
        super().__init__('voice_command_processor')

        # Declare parameters
        self.declare_parameter('whisper_model_size', 'base')
        self.declare_parameter('recording_duration', 5.0)
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('sample_rate', 16000)

        # Get parameters
        self.whisper_model_size = self.get_parameter('whisper_model_size').value
        self.recording_duration = self.get_parameter('recording_duration').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.sample_rate = self.get_parameter('sample_rate').value

        # Initialize Whisper model
        self.get_logger().info(f"Loading Whisper model: {self.whisper_model_size}")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = whisper.load_model(self.whisper_model_size).to(self.device)
        self.get_logger().info("Whisper model loaded successfully")

        # Initialize command interpreter
        self.interpreter = VoiceCommandInterpreter()

        # Initialize queues for thread communication
        self.audio_queue = queue.Queue()
        self.processing_thread = None
        self.is_processing = False

        # Publishers
        self.voice_command_pub = self.create_publisher(String, 'voice_commands', 10)
        self.command_confidence_pub = self.create_publisher(Float32, 'command_confidence', 10)
        self.robot_command_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10
        )

        # Timer for processing audio queue
        self.process_timer = self.create_timer(1.0, self.process_audio_queue)

        # Statistics
        self.command_count = 0
        self.successful_transcriptions = 0

        self.get_logger().info("Voice Command Processor node initialized")

    def audio_callback(self, msg: AudioData):
        """
        Callback for incoming audio data
        """
        # Add audio data to processing queue
        self.audio_queue.put(msg.data)
        self.get_logger().debug(f"Audio data received, queue size: {self.audio_queue.qsize()}")

    def process_audio_queue(self):
        """
        Process audio data from the queue
        """
        if not self.audio_queue.empty():
            # Get audio data from queue
            audio_data = self.audio_queue.get()

            # Process the audio data in a separate thread to avoid blocking
            if not self.is_processing:
                self.is_processing = True
                self.processing_thread = threading.Thread(
                    target=self.process_audio_data,
                    args=(audio_data,)
                )
                self.processing_thread.start()

    def process_audio_data(self, audio_data: bytes):
        """
        Process raw audio data through Whisper ASR
        """
        try:
            # Convert audio bytes to numpy array
            audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

            # Pad or trim audio to appropriate length
            audio_tensor = torch.from_numpy(audio_array).to(self.device)

            # Convert to log-Mel spectrogram
            mel = whisper.log_mel_spectrogram(audio_tensor).to(self.model.device)

            # Decode the audio
            options = whisper.DecodingOptions(fp16=False)
            result = whisper.decode(self.model, mel, options)

            # Get transcription result
            transcription = result.text.strip()
            confidence = getattr(result, 'avg_logprob', 0.0)

            self.get_logger().info(f"Transcription: '{transcription}' (confidence: {confidence:.2f})")

            # Publish transcription
            cmd_msg = String()
            cmd_msg.data = transcription
            self.voice_command_pub.publish(cmd_msg)

            # Publish confidence
            conf_msg = Float32()
            conf_msg.data = confidence
            self.command_confidence_pub.publish(conf_msg)

            # Process if confidence is above threshold
            if confidence > self.confidence_threshold:
                self.handle_interpreted_command(transcription, confidence)
                self.successful_transcriptions += 1
            else:
                self.get_logger().warn(f"Low confidence transcription ignored: {confidence:.2f}")

            self.command_count += 1

        except Exception as e:
            self.get_logger().error(f"Error processing audio data: {str(e)}")

        finally:
            self.is_processing = False

    def handle_interpreted_command(self, transcription: str, confidence: float):
        """
        Handle interpreted command and send to robot
        """
        # Interpret the command
        action = self.interpreter.interpret_command(transcription)

        self.get_logger().info(f"Interpreted action: {action['action_type']}")

        # Send appropriate command to robot based on action type
        if action['action_type'] == 'MOVE_FORWARD':
            self.send_move_command(linear_x=0.5, angular_z=0.0)
        elif action['action_type'] == 'MOVE_BACKWARD':
            self.send_move_command(linear_x=-0.5, angular_z=0.0)
        elif action['action_type'] == 'TURN_LEFT':
            self.send_move_command(linear_x=0.0, angular_z=0.5)
        elif action['action_type'] == 'TURN_RIGHT':
            self.send_move_command(linear_x=0.0, angular_z=-0.5)
        elif action['action_type'] == 'STOP':
            self.send_move_command(linear_x=0.0, angular_z=0.0)
        elif action['action_type'] == 'NAVIGATE_TO_LOCATION':
            # In a real system, this would trigger navigation
            location = action.get('parameters', {}).get('target', 'unknown')
            self.get_logger().info(f"Navigation command to: {location}")
            # This would call a navigation action server in a real implementation
        else:
            self.get_logger().info(f"Unknown command type: {action['action_type']}")

    def send_move_command(self, linear_x: float = 0.0, angular_z: float = 0.0):
        """
        Send movement command to robot
        """
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.robot_command_pub.publish(twist_msg)
        self.get_logger().info(f"Sent move command: linear_x={linear_x}, angular_z={angular_z}")

    def get_statistics(self) -> Dict[str, Any]:
        """
        Get processing statistics
        """
        return {
            'total_commands': self.command_count,
            'successful_transcriptions': self.successful_transcriptions,
            'success_rate': self.successful_transcriptions / self.command_count if self.command_count > 0 else 0.0,
            'queue_size': self.audio_queue.qsize(),
            'is_processing': self.is_processing
        }


class VoiceCommandInterpreter:
    """
    Interprets voice commands and converts them to robot actions
    """

    def __init__(self):
        # Define command patterns and their corresponding robot actions
        self.command_patterns = {
            # Navigation commands
            r"go to the (.+)": "NAVIGATE_TO_LOCATION",
            r"move to the (.+)": "NAVIGATE_TO_LOCATION",
            r"go to (.+)": "NAVIGATE_TO_LOCATION",
            r"move to (.+)": "NAVIGATE_TO_LOCATION",
            r"move forward": "MOVE_FORWARD",
            r"move backward": "MOVE_BACKWARD",
            r"go forward": "MOVE_FORWARD",
            r"go backward": "MOVE_BACKWARD",
            r"turn left": "TURN_LEFT",
            r"turn right": "TURN_RIGHT",
            r"rotate left": "ROTATE_LEFT",
            r"rotate right": "ROTATE_RIGHT",

            # Movement commands
            r"stop": "STOP",
            r"halt": "STOP",
            r"pause": "PAUSE",
            r"continue": "CONTINUE"
        }

        # Map action types to parameter extraction patterns
        self.parameter_patterns = {
            "NAVIGATE_TO_LOCATION": r"(?:go to the |move to the |go to |move to )(.+)"
        }

    def interpret_command(self, transcription: str) -> Dict[str, Any]:
        """
        Interpret a transcribed command and convert to robot action

        Args:
            transcription: The transcribed text from speech

        Returns:
            Dictionary containing action type and parameters
        """
        import re

        transcription_lower = transcription.lower().strip()

        # Try to match each pattern
        for pattern, action_type in self.command_patterns.items():
            match = re.search(pattern, transcription_lower)
            if match:
                # Extract parameters if needed
                params = {}

                # Extract the matched object/location
                if action_type in self.parameter_patterns:
                    param_match = re.search(self.parameter_patterns[action_type], transcription_lower)
                    if param_match:
                        params['target'] = param_match.group(1).strip()

                return {
                    "action_type": action_type,
                    "raw_command": transcription,
                    "parameters": params,
                    "confidence": 0.8  # High confidence for pattern-matched commands
                }

        # If no pattern matches, return unknown command
        return {
            "action_type": "UNKNOWN_COMMAND",
            "raw_command": transcription,
            "parameters": {},
            "confidence": 0.0
        }


def main(args=None):
    """
    Main function to run the voice command processor node
    """
    rclpy.init(args=args)

    node = VoiceCommandProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Voice Command Processor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()