#!/usr/bin/env python3
"""
Whisper Integration for Voice-to-Action Pipeline

This script demonstrates how to integrate OpenAI's Whisper ASR system
into a robotics pipeline for voice command processing.
"""

import os
import sys
import subprocess
import tempfile
import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write
import whisper
import torch
from typing import Optional, Tuple, Dict, Any


class WhisperVoiceProcessor:
    """
    A class to handle voice processing using OpenAI's Whisper model
    for robotics applications.
    """

    def __init__(self, model_size: str = "base", device: str = None):
        """
        Initialize the Whisper voice processor

        Args:
            model_size: Size of the Whisper model ('tiny', 'base', 'small', 'medium', 'large')
            device: Device to run the model on ('cpu', 'cuda', or None for auto)
        """
        self.model_size = model_size
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")

        print(f"Loading Whisper model '{model_size}' on {self.device}...")
        self.model = whisper.load_model(model_size).to(self.device)
        print("Model loaded successfully!")

    def record_audio(self, duration: float = 5.0, sample_rate: int = 16000) -> np.ndarray:
        """
        Record audio from the microphone

        Args:
            duration: Duration to record in seconds
            sample_rate: Sample rate for recording

        Returns:
            Audio data as numpy array
        """
        print(f"Recording audio for {duration} seconds...")
        audio_data = sd.rec(
            int(duration * sample_rate),
            samplerate=sample_rate,
            channels=1,
            dtype='float32'
        )
        sd.wait()  # Wait until recording is finished
        print("Recording completed.")

        # Convert to mono if needed and return
        if len(audio_data.shape) > 1:
            audio_data = audio_data.mean(axis=1)

        return audio_data

    def save_audio_to_temp_file(self, audio_data: np.ndarray, sample_rate: int = 16000) -> str:
        """
        Save audio data to a temporary WAV file

        Args:
            audio_data: Audio data as numpy array
            sample_rate: Sample rate for the audio

        Returns:
            Path to the temporary file
        """
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
        write(temp_file.name, sample_rate, (audio_data * 32767).astype(np.int16))
        return temp_file.name

    def transcribe_audio(self, audio_path: str, language: str = "en") -> Dict[str, Any]:
        """
        Transcribe audio using Whisper

        Args:
            audio_path: Path to the audio file
            language: Language code for transcription

        Returns:
            Dictionary containing transcription results
        """
        print(f"Transcribing audio from {audio_path}...")

        # Load audio file
        audio = whisper.load_audio(audio_path)
        audio = whisper.pad_or_trim(audio)

        # Make log-Mel spectrogram and move to the same device as the model
        mel = whisper.log_mel_spectrogram(audio).to(self.model.device)

        # Decode the audio
        options = whisper.DecodingOptions(language=language, fp16=False)
        result = whisper.decode(self.model, mel, options)

        # Return transcription result
        transcription_result = {
            "text": result.text.strip(),
            "language": result.language,
            "segments": [{"start": 0, "end": len(result.text), "text": result.text}],
            "confidence": getattr(result, 'avg_logprob', 0.0)  # Average log probability as confidence
        }

        print(f"Transcription: '{result.text.strip()}'")
        return transcription_result

    def process_voice_command(self, duration: float = 5.0) -> Dict[str, Any]:
        """
        Complete process: record, transcribe, and return command

        Args:
            duration: Duration to record voice command

        Returns:
            Dictionary containing transcription and confidence
        """
        try:
            # Record audio
            audio_data = self.record_audio(duration)

            # Save to temporary file
            temp_path = self.save_audio_to_temp_file(audio_data)

            # Transcribe
            result = self.transcribe_audio(temp_path)

            # Clean up temporary file
            os.unlink(temp_path)

            return result

        except Exception as e:
            print(f"Error processing voice command: {str(e)}")
            return {
                "text": "",
                "language": "",
                "segments": [],
                "confidence": 0.0,
                "error": str(e)
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

            # Manipulation commands
            r"pick up the (.+)": "PICK_UP_OBJECT",
            r"grab the (.+)": "PICK_UP_OBJECT",
            r"take the (.+)": "PICK_UP_OBJECT",
            r"grasp the (.+)": "PICK_UP_OBJECT",
            r"place the (.+)": "PLACE_OBJECT",
            r"put down the (.+)": "PUT_DOWN_OBJECT",
            r"release the (.+)": "RELEASE_OBJECT",

            # Object interaction
            r"detect (.+)": "DETECT_OBJECT",
            r"find (.+)": "FIND_OBJECT",
            r"show me the (.+)": "SHOW_OBJECT",

            # Movement commands
            r"stop": "STOP",
            r"halt": "STOP",
            r"pause": "PAUSE",
            r"continue": "CONTINUE"
        }

        # Map action types to parameter extraction patterns
        self.parameter_patterns = {
            "NAVIGATE_TO_LOCATION": r"(?:go to the |move to the |go to |move to )(.+)",
            "PICK_UP_OBJECT": r"(?:pick up the |grab the |take the |grasp the )(.+)",
            "PLACE_OBJECT": r"(?:place the |put the )(.+)",
            "DETECT_OBJECT": r"(?:detect |find |show me the )(.+)"
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


def main():
    """
    Main function to demonstrate Whisper integration
    """
    print("Whisper Voice-to-Action Pipeline Demo")
    print("=" * 40)

    # Initialize the voice processor
    try:
        processor = WhisperVoiceProcessor(model_size="base")
    except Exception as e:
        print(f"Failed to initialize Whisper processor: {e}")
        print("Make sure you have installed the required dependencies:")
        print("- pip install openai-whisper")
        print("- pip install sounddevice")
        print("- pip install scipy")
        print("- torch (with appropriate CUDA support if using GPU)")
        return

    interpreter = VoiceCommandInterpreter()

    print("\nReady to process voice commands!")
    print("Say something like 'Go to the kitchen' or 'Pick up the red cup'")
    print("Press Ctrl+C to exit\n")

    try:
        while True:
            print("\nListening... (Speak now)")
            result = processor.process_voice_command(duration=5.0)

            if result.get("error"):
                print(f"Error: {result['error']}")
                continue

            transcription = result["text"]
            confidence = result["confidence"]

            print(f"\nTranscription: '{transcription}' (confidence: {confidence:.2f})")

            if confidence > 0.3:  # Only process if confidence is reasonable
                action = interpreter.interpret_command(transcription)
                print(f"Interpreted action: {action['action_type']}")

                if action['parameters']:
                    print(f"Parameters: {action['parameters']}")

                # In a real robot system, you would send this action to the robot
                # For this demo, we just print it
                print(f"Action would be sent to robot: {action}")
            else:
                print("Low confidence transcription - ignoring")

            # Ask if user wants to continue
            user_input = input("\nPress Enter to listen again, or 'q' to quit: ")
            if user_input.lower() == 'q':
                break

    except KeyboardInterrupt:
        print("\n\nExiting voice processing demo...")

    print("\nDemo completed!")


if __name__ == "__main__":
    main()