#!/usr/bin/env python3
"""
LLM Interface for VLA Cognitive Planning

This module provides a standardized interface for interacting with Large Language Models
for cognitive planning in VLA robotics applications.
"""

import os
import json
import requests
from typing import Dict, Any, List, Optional, Union
from abc import ABC, abstractmethod
from dataclasses import dataclass
import time
import logging


# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class LLMResponse:
    """
    Represents a response from an LLM
    """
    content: str
    tokens_used: int
    model: str
    timestamp: float
    success: bool
    error_message: Optional[str] = None


class LLMInterface(ABC):
    """
    Abstract base class for LLM interfaces
    """

    @abstractmethod
    def generate(self, prompt: str, **kwargs) -> LLMResponse:
        """
        Generate a response from the LLM

        Args:
            prompt: The input prompt
            **kwargs: Additional parameters for the LLM

        Returns:
            LLMResponse object containing the result
        """
        pass

    @abstractmethod
    def validate_connection(self) -> bool:
        """
        Validate that the LLM interface is properly configured

        Returns:
            True if connection is valid, False otherwise
        """
        pass


class OpenAILLM(LLMInterface):
    """
    Interface for OpenAI models (GPT-3.5-turbo, GPT-4, etc.)
    """

    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo", temperature: float = 0.1):
        """
        Initialize the OpenAI LLM interface

        Args:
            api_key: OpenAI API key
            model: Model to use (default: gpt-3.5-turbo)
            temperature: Sampling temperature (lower = more deterministic)
        """
        self.api_key = api_key
        self.model = model
        self.temperature = temperature
        self.base_url = "https://api.openai.com/v1/chat/completions"
        self.headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        }

    def generate(self, prompt: str, **kwargs) -> LLMResponse:
        """
        Generate a response from the OpenAI API

        Args:
            prompt: The input prompt
            **kwargs: Additional parameters (max_tokens, top_p, etc.)

        Returns:
            LLMResponse object containing the result
        """
        start_time = time.time()

        # Prepare the request
        messages = [
            {"role": "system", "content": "You are an expert in robotics and AI planning. Provide precise, actionable responses."},
            {"role": "user", "content": prompt}
        ]

        data = {
            "model": self.model,
            "messages": messages,
            "temperature": self.temperature,
            "max_tokens": kwargs.get("max_tokens", 1000),
            "top_p": kwargs.get("top_p", 1.0),
            "frequency_penalty": kwargs.get("frequency_penalty", 0.0),
            "presence_penalty": kwargs.get("presence_penalty", 0.0)
        }

        try:
            response = requests.post(
                self.base_url,
                headers=self.headers,
                json=data,
                timeout=30
            )

            if response.status_code == 200:
                result = response.json()
                content = result['choices'][0]['message']['content']
                tokens_used = result['usage']['total_tokens']

                logger.info(f"OpenAI API call took {time.time() - start_time:.2f}s")
                return LLMResponse(
                    content=content,
                    tokens_used=tokens_used,
                    model=self.model,
                    timestamp=time.time(),
                    success=True
                )
            else:
                error_msg = f"API Error: {response.status_code} - {response.text}"
                logger.error(error_msg)
                return LLMResponse(
                    content="",
                    tokens_used=0,
                    model=self.model,
                    timestamp=time.time(),
                    success=False,
                    error_message=error_msg
                )

        except requests.exceptions.RequestException as e:
            error_msg = f"Request failed: {str(e)}"
            logger.error(error_msg)
            return LLMResponse(
                content="",
                tokens_used=0,
                model=self.model,
                timestamp=time.time(),
                success=False,
                error_message=error_msg
            )

    def validate_connection(self) -> bool:
        """
        Validate that the OpenAI API connection is working

        Returns:
            True if connection is valid, False otherwise
        """
        try:
            response = self.generate("Say 'connection test' in one word", max_tokens=5)
            return response.success and "connection" in response.content.lower()
        except Exception:
            return False


class AnthropicLLM(LLMInterface):
    """
    Interface for Anthropic models (Claude)
    """

    def __init__(self, api_key: str, model: str = "claude-3-sonnet-20240229", temperature: float = 0.1):
        """
        Initialize the Anthropic LLM interface

        Args:
            api_key: Anthropic API key
            model: Model to use (default: claude-3-sonnet-20240229)
            temperature: Sampling temperature (lower = more deterministic)
        """
        self.api_key = api_key
        self.model = model
        self.temperature = temperature
        self.base_url = "https://api.anthropic.com/v1/messages"
        self.headers = {
            "x-api-key": api_key,
            "Content-Type": "application/json",
            "anthropic-version": "2023-06-01"
        }

    def generate(self, prompt: str, **kwargs) -> LLMResponse:
        """
        Generate a response from the Anthropic API

        Args:
            prompt: The input prompt
            **kwargs: Additional parameters (max_tokens, etc.)

        Returns:
            LLMResponse object containing the result
        """
        start_time = time.time()

        data = {
            "model": self.model,
            "messages": [
                {"role": "user", "content": prompt}
            ],
            "temperature": self.temperature,
            "max_tokens": kwargs.get("max_tokens", 1000)
        }

        try:
            response = requests.post(
                self.base_url,
                headers=self.headers,
                json=data,
                timeout=30
            )

            if response.status_code == 200:
                result = response.json()
                content = result['content'][0]['text']
                tokens_used = result['usage']['input_tokens'] + result['usage']['output_tokens']

                logger.info(f"Anthropic API call took {time.time() - start_time:.2f}s")
                return LLMResponse(
                    content=content,
                    tokens_used=tokens_used,
                    model=self.model,
                    timestamp=time.time(),
                    success=True
                )
            else:
                error_msg = f"API Error: {response.status_code} - {response.text}"
                logger.error(error_msg)
                return LLMResponse(
                    content="",
                    tokens_used=0,
                    model=self.model,
                    timestamp=time.time(),
                    success=False,
                    error_message=error_msg
                )

        except requests.exceptions.RequestException as e:
            error_msg = f"Request failed: {str(e)}"
            logger.error(error_msg)
            return LLMResponse(
                content="",
                tokens_used=0,
                model=self.model,
                timestamp=time.time(),
                success=False,
                error_message=error_msg
            )

    def validate_connection(self) -> bool:
        """
        Validate that the Anthropic API connection is working

        Returns:
            True if connection is valid, False otherwise
        """
        try:
            response = self.generate("Say 'connection test' in one word", max_tokens=5)
            return response.success and "connection" in response.content.lower()
        except Exception:
            return False


class HuggingFaceLLM(LLMInterface):
    """
    Interface for Hugging Face hosted models
    """

    def __init__(self, api_key: str, model: str, temperature: float = 0.1):
        """
        Initialize the Hugging Face LLM interface

        Args:
            api_key: Hugging Face API key
            model: Model identifier (e.g., "microsoft/DialoGPT-medium")
            temperature: Sampling temperature (lower = more deterministic)
        """
        self.api_key = api_key
        self.model = model
        self.temperature = temperature
        self.base_url = f"https://api-inference.huggingface.co/models/{model}"
        self.headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        }

    def generate(self, prompt: str, **kwargs) -> LLMResponse:
        """
        Generate a response from the Hugging Face API

        Args:
            prompt: The input prompt
            **kwargs: Additional parameters

        Returns:
            LLMResponse object containing the result
        """
        start_time = time.time()

        # Prepare the request
        data = {
            "inputs": prompt,
            "parameters": {
                "temperature": self.temperature,
                "max_new_tokens": kwargs.get("max_tokens", 100),
                "return_full_text": False
            }
        }

        try:
            response = requests.post(
                self.base_url,
                headers=self.headers,
                json=data,
                timeout=30
            )

            if response.status_code == 200:
                result = response.json()
                content = result[0]['generated_text']

                # Estimate token count (rough approximation)
                tokens_used = len(content.split())

                logger.info(f"Hugging Face API call took {time.time() - start_time:.2f}s")
                return LLMResponse(
                    content=content,
                    tokens_used=tokens_used,
                    model=self.model,
                    timestamp=time.time(),
                    success=True
                )
            else:
                error_msg = f"API Error: {response.status_code} - {response.text}"
                logger.error(error_msg)
                return LLMResponse(
                    content="",
                    tokens_used=0,
                    model=self.model,
                    timestamp=time.time(),
                    success=False,
                    error_message=error_msg
                )

        except requests.exceptions.RequestException as e:
            error_msg = f"Request failed: {str(e)}"
            logger.error(error_msg)
            return LLMResponse(
                content="",
                tokens_used=0,
                model=self.model,
                timestamp=time.time(),
                success=False,
                error_message=error_msg
            )

    def validate_connection(self) -> bool:
        """
        Validate that the Hugging Face API connection is working

        Returns:
            True if connection is valid, False otherwise
        """
        try:
            response = self.generate("Hello", max_tokens=5)
            return response.success and len(response.content) > 0
        except Exception:
            return False


class VLAPlanningInterface:
    """
    High-level interface for VLA cognitive planning using LLMs
    """

    def __init__(self, llm_interface: LLMInterface):
        """
        Initialize the VLA planning interface

        Args:
            llm_interface: Instance of an LLMInterface implementation
        """
        self.llm = llm_interface
        if not self.llm.validate_connection():
            raise ValueError("LLM connection validation failed")

    def decompose_task(self, task_description: str, environment_state: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Decompose a high-level task into executable robot actions

        Args:
            task_description: Natural language description of the task
            environment_state: Current state of the environment (optional)

        Returns:
            Dictionary containing the decomposed task
        """
        environment_info = json.dumps(environment_state or {}, indent=2)

        prompt = f"""
        Decompose the following high-level task into specific, executable robot actions:

        TASK: {task_description}

        ENVIRONMENT STATE:
        {environment_info}

        Provide the result as a JSON object with the following structure:
        {{
            "task_breakdown": [
                {{
                    "step_number": 1,
                    "action_type": "NAVIGATE_TO_LOCATION|DETECT_OBJECT|GRASP_OBJECT|etc.",
                    "description": "Human-readable description of the action",
                    "parameters": {{"key": "value"}},
                    "preconditions": ["list of conditions that must be met"],
                    "effects": ["list of effects of the action"],
                    "estimated_duration": 5.0,
                    "confidence": 0.8
                }}
            ],
            "overall_strategy": "Brief description of the overall approach",
            "potential_issues": ["list of potential problems and mitigations"],
            "success_criteria": ["list of conditions that indicate task completion"]
        }}

        Each action should be:
        - Specific and executable
        - Include all necessary parameters
        - Have clear preconditions and expected effects
        - Be compatible with typical humanoid robot capabilities
        """

        response = self.llm.generate(prompt, max_tokens=1500)
        if not response.success:
            raise RuntimeError(f"LLM request failed: {response.error_message}")

        # Extract and parse the JSON response
        content = response.content.strip()

        # Find JSON in the response (might have text before/after)
        json_start = content.find('{')
        json_end = content.rfind('}') + 1

        if json_start != -1 and json_end != 0:
            json_str = content[json_start:json_end]
            try:
                return json.loads(json_str)
            except json.JSONDecodeError:
                logger.error(f"Failed to parse JSON from LLM response: {json_str}")
                # Fallback: return a basic structure
                return {
                    "task_breakdown": [],
                    "overall_strategy": "Unable to parse LLM response",
                    "potential_issues": [],
                    "success_criteria": []
                }
        else:
            logger.error(f"Could not find JSON in LLM response: {content}")
            return {
                "task_breakdown": [],
                "overall_strategy": "LLM response did not contain valid JSON",
                "potential_issues": [],
                "success_criteria": []
            }

    def plan_multi_step_task(self, goal: str, constraints: List[str] = None) -> Dict[str, Any]:
        """
        Plan a multi-step task with constraints

        Args:
            goal: The ultimate goal to achieve
            constraints: List of constraints to consider

        Returns:
            Dictionary containing the multi-step plan
        """
        constraints_str = "\n".join(constraints) if constraints else "None specified"

        prompt = f"""
        Plan a multi-step task to achieve the following goal:

        GOAL: {goal}

        CONSTRAINTS:
        {constraints_str}

        Provide a detailed plan as a JSON object with the following structure:
        {{
            "plan_overview": "Brief overview of the plan",
            "steps": [
                {{
                    "id": "unique_step_id",
                    "name": "Step name",
                    "description": "Detailed description of what to do",
                    "required_resources": ["list of resources needed"],
                    "estimated_time": 10.0,
                    "dependencies": ["list of step IDs this step depends on"],
                    "subtasks": [
                        {{
                            "name": "Subtask name",
                            "description": "What to do",
                            "completion_criteria": "How to know it's done"
                        }}
                    ],
                    "risk_assessment": "Potential risks and mitigation strategies",
                    "success_indicators": ["list of success indicators"]
                }}
            ],
            "resource_requirements": ["summary of all resources needed"],
            "estimated_total_time": 120.0,
            "critical_dependencies": ["list of critical dependency chains"]
        }}

        The plan should be:
        - Comprehensive and actionable
        - Consider the specified constraints
        - Include risk assessment and mitigation strategies
        - Have clear success criteria for each step
        """

        response = self.llm.generate(prompt, max_tokens=2000)
        if not response.success:
            raise RuntimeError(f"LLM request failed: {response.error_message}")

        # Extract and parse the JSON response
        content = response.content.strip()

        json_start = content.find('{')
        json_end = content.rfind('}') + 1

        if json_start != -1 and json_end != 0:
            json_str = content[json_start:json_end]
            try:
                return json.loads(json_str)
            except json.JSONDecodeError:
                logger.error(f"Failed to parse JSON from LLM response: {json_str}")
                return {
                    "plan_overview": "Unable to parse LLM response",
                    "steps": [],
                    "resource_requirements": [],
                    "estimated_total_time": 0.0,
                    "critical_dependencies": []
                }
        else:
            logger.error(f"Could not find JSON in LLM response: {content}")
            return {
                "plan_overview": "LLM response did not contain valid JSON",
                "steps": [],
                "resource_requirements": [],
                "estimated_total_time": 0.0,
                "critical_dependencies": []
            }

    def refine_plan(self, current_plan: Dict[str, Any], feedback: str) -> Dict[str, Any]:
        """
        Refine an existing plan based on feedback

        Args:
            current_plan: The current plan to refine
            feedback: Feedback about the current plan

        Returns:
            Dictionary containing the refined plan
        """
        current_plan_json = json.dumps(current_plan, indent=2)

        prompt = f"""
        Refine the following plan based on the provided feedback:

        CURRENT PLAN:
        {current_plan_json}

        FEEDBACK:
        {feedback}

        Provide the refined plan in the same JSON format as the original plan.
        Only make changes that address the feedback.
        If the feedback indicates no changes are needed, return the original plan unchanged.
        """

        response = self.llm.generate(prompt, max_tokens=2000)
        if not response.success:
            raise RuntimeError(f"LLM request failed: {response.error_message}")

        # Extract and parse the JSON response
        content = response.content.strip()

        json_start = content.find('{')
        json_end = content.rfind('}') + 1

        if json_start != -1 and json_end != 0:
            json_str = content[json_start:json_end]
            try:
                return json.loads(json_str)
            except json.JSONDecodeError:
                logger.error(f"Failed to parse JSON from refinement response: {json_str}")
                return current_plan  # Return original if refinement fails
        else:
            logger.warning(f"Could not find JSON in refinement response: {content}")
            return current_plan  # Return original if refinement fails


def main():
    """
    Demonstrate the LLM interface for VLA cognitive planning
    """
    print("VLA Cognitive Planning Interface Demo")
    print("=" * 50)

    # Note: In a real implementation, you would use one of these:
    # 1. OpenAI interface
    # api_key = os.environ.get("OPENAI_API_KEY")
    # if api_key:
    #     llm_interface = OpenAILLM(api_key, model="gpt-3.5-turbo")
    # 2. Anthropic interface
    # api_key = os.environ.get("ANTHROPIC_API_KEY")
    # if api_key:
    #     llm_interface = AnthropicLLM(api_key, model="claude-3-sonnet-20240229")
    # 3. Hugging Face interface
    # api_key = os.environ.get("HUGGING_FACE_API_KEY")
    # if api_key:
    #     llm_interface = HuggingFaceLLM(api_key, model="microsoft/DialoGPT-medium")

    # For this demo, we'll create a mock interface
    class MockLLMInterface(LLMInterface):
        def generate(self, prompt: str, **kwargs) -> LLMResponse:
            # Mock implementation for demonstration
            import random

            # Simulate different responses based on prompt content
            if "decompose" in prompt.lower() or "breakdown" in prompt.lower():
                mock_response = {
                    "task_breakdown": [
                        {
                            "step_number": 1,
                            "action_type": "NAVIGATE_TO_LOCATION",
                            "description": "Navigate to the kitchen counter",
                            "parameters": {"x": 2.0, "y": 1.5, "theta": 0.0},
                            "preconditions": ["robot_is_at_home_position", "no_obstacles_detected"],
                            "effects": ["robot_position_updated", "navigation_completed"],
                            "estimated_duration": 5.0,
                            "confidence": 0.9
                        },
                        {
                            "step_number": 2,
                            "action_type": "DETECT_OBJECT",
                            "description": "Detect the red cup on the counter",
                            "parameters": {"target_object": "red_cup", "search_area": "counter_surface"},
                            "preconditions": ["robot_at_kitchen_counter"],
                            "effects": ["object_detected", "object_location_known"],
                            "estimated_duration": 2.0,
                            "confidence": 0.85
                        },
                        {
                            "step_number": 3,
                            "action_type": "GRASP_OBJECT",
                            "description": "Grasp the red cup",
                            "parameters": {"object_id": "red_cup_123", "grasp_type": "top_grasp"},
                            "preconditions": ["object_detected", "object_within_reach"],
                            "effects": ["object_grasped", "robot_gripper_occupied"],
                            "estimated_duration": 3.0,
                            "confidence": 0.8
                        }
                    ],
                    "overall_strategy": "Go to kitchen, find cup, pick it up",
                    "potential_issues": [
                        "Cup might not be where expected",
                        "Robot arm might not reach the cup",
                        "Gripper might fail to grasp"
                    ],
                    "success_criteria": [
                        "Red cup is grasped by robot",
                        "Cup is lifted 10cm from surface"
                    ]
                }
                response_content = json.dumps(mock_response, indent=2)
            else:
                response_content = '{"plan_overview": "Mock plan for demonstration", "steps": [], "resource_requirements": [], "estimated_total_time": 60.0, "critical_dependencies": []}'

            return LLMResponse(
                content=response_content,
                tokens_used=random.randint(50, 150),
                model="mock-model",
                timestamp=time.time(),
                success=True
            )

        def validate_connection(self) -> bool:
            return True

    # Initialize the interface
    llm_interface = MockLLMInterface()
    planner = VLAPlanningInterface(llm_interface)

    # Test task decomposition
    print("\n1. Task Decomposition Test:")
    print("-" * 30)

    test_task = "Go to the kitchen, find the red cup on the counter, and pick it up"
    environment_state = {
        "robot_position": [0, 0, 0],
        "kitchen_location": [2, 1.5, 0],
        "known_objects": [
            {"name": "red_cup", "type": "drinkware", "location": [2.2, 1.6, 0.8]}
        ],
        "obstacles": []
    }

    try:
        decomposition = planner.decompose_task(test_task, environment_state)
        print(f"Task decomposed into {len(decomposition.get('task_breakdown', []))} steps:")
        for step in decomposition.get('task_breakdown', []):
            print(f"  - Step {step['step_number']}: {step['action_type']} - {step['description']}")
    except Exception as e:
        print(f"Error in task decomposition: {e}")

    # Test multi-step planning
    print("\n2. Multi-Step Planning Test:")
    print("-" * 30)

    goal = "Clean the living room by collecting all books and placing them on the bookshelf"
    constraints = [
        "Avoid fragile items",
        "Do not block doorways",
        "Return to charging station when battery low"
    ]

    try:
        plan = planner.plan_multi_step_task(goal, constraints)
        print(f"Plan created with {len(plan.get('steps', []))} main steps")
        print(f"Estimated total time: {plan.get('estimated_total_time', 0)} seconds")
    except Exception as e:
        print(f"Error in multi-step planning: {e}")

    print(f"\nDemo completed! In a real implementation, you would:")
    print("1. Set your LLM API key in environment variables")
    print("2. Choose the appropriate LLM provider (OpenAI, Anthropic, HuggingFace, etc.)")
    print("3. Connect to a real robot system for execution")


if __name__ == "__main__":
    main()