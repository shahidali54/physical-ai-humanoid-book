#!/usr/bin/env python3
"""
LLM-Based Task Decomposition for VLA Robotics

This module demonstrates how to use Large Language Models to decompose
complex natural language tasks into executable sequences for humanoid robots.
"""

import openai
import json
import re
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum


class ActionType(Enum):
    """
    Enumeration of robot action types
    """
    NAVIGATE_TO_LOCATION = "NAVIGATE_TO_LOCATION"
    DETECT_OBJECT = "DETECT_OBJECT"
    GRASP_OBJECT = "GRASP_OBJECT"
    PLACE_OBJECT = "PLACE_OBJECT"
    RELEASE_OBJECT = "RELEASE_OBJECT"
    MOVE_ARM = "MOVE_ARM"
    MOVE_BASE = "MOVE_BASE"
    WAIT = "WAIT"
    QUERY_ENVIRONMENT = "QUERY_ENVIRONMENT"
    FOLLOW_PATH = "FOLLOW_PATH"
    AVOID_OBSTACLE = "AVOID_OBSTACLE"


@dataclass
class ActionStep:
    """
    Represents a single action step in a robot task
    """
    action_type: ActionType
    parameters: Dict[str, Any]
    description: str
    priority: int = 1  # Lower number means higher priority
    estimated_duration: float = 1.0  # Estimated time in seconds


class TaskDecomposer:
    """
    Decomposes complex natural language tasks into sequences of robot actions
    using LLMs for cognitive planning.
    """

    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        """
        Initialize the task decomposer

        Args:
            api_key: OpenAI API key
            model: LLM model to use for decomposition
        """
        openai.api_key = api_key
        self.model = model
        self.action_types = [action.value for action in ActionType]

    def decompose_task(
        self,
        task_description: str,
        environment_state: Optional[Dict[str, Any]] = None,
        robot_capabilities: Optional[List[str]] = None
    ) -> List[ActionStep]:
        """
        Decompose a high-level task into a sequence of robot actions

        Args:
            task_description: Natural language description of the task
            environment_state: Current state of the environment
            robot_capabilities: List of robot capabilities

        Returns:
            List of ActionStep objects representing the decomposed task
        """
        environment_info = json.dumps(environment_state or {}, indent=2)
        capabilities_info = ", ".join(robot_capabilities or [])

        prompt = f"""
        Decompose the following high-level task into specific, executable robot actions:

        TASK: {task_description}

        ENVIRONMENT STATE:
        {environment_info}

        ROBOT CAPABILITIES:
        {capabilities_info}

        AVAILABLE ACTION TYPES:
        {', '.join(self.action_types)}

        Provide the result as a JSON array of action steps. Each action step should have:
        - action_type: The type of action (must be one of the available action types)
        - parameters: Dictionary of parameters needed for the action
        - description: Human-readable description of the action
        - priority: Priority level (1-5, where 1 is highest priority)
        - estimated_duration: Estimated time to complete the action in seconds

        Example format:
        [
            {{
                "action_type": "NAVIGATE_TO_LOCATION",
                "parameters": {{"x": 1.0, "y": 2.0, "theta": 0.0}},
                "description": "Navigate to the kitchen counter",
                "priority": 1,
                "estimated_duration": 5.0
            }},
            {{
                "action_type": "DETECT_OBJECT",
                "parameters": {{"target_object": "red cup"}},
                "description": "Detect the red cup on the counter",
                "priority": 2,
                "estimated_duration": 2.0
            }}
        ]

        Ensure that:
        1. Actions are logically ordered (consider dependencies)
        2. Parameters are specific and actionable
        3. Actions are compatible with the robot's capabilities
        4. The sequence achieves the overall task goal
        5. Include error handling steps if appropriate
        """

        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are an expert in robotics task planning. "
                            "Decompose high-level tasks into specific, executable robot actions. "
                            "Ensure the action sequence is logical and achievable with the given robot capabilities."
                        )
                    },
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1  # Low temperature for consistent results
            )

            # Extract the JSON response
            response_text = response.choices[0].message.content.strip()

            # Clean up the response to extract JSON
            json_start = response_text.find('[')
            json_end = response_text.rfind(']') + 1

            if json_start != -1 and json_end != 0:
                json_str = response_text[json_start:json_end]
                action_steps_data = json.loads(json_str)

                # Convert to ActionStep objects
                action_steps = []
                for step_data in action_steps_data:
                    action_type = ActionType(step_data['action_type'])
                    step = ActionStep(
                        action_type=action_type,
                        parameters=step_data.get('parameters', {}),
                        description=step_data.get('description', ''),
                        priority=step_data.get('priority', 1),
                        estimated_duration=step_data.get('estimated_duration', 1.0)
                    )
                    action_steps.append(step)

                return action_steps

        except json.JSONDecodeError as e:
            print(f"Error parsing LLM response: {e}")
            print(f"Response text: {response_text}")
            return self._fallback_decomposition(task_description)

        except Exception as e:
            print(f"Error during task decomposition: {e}")
            return self._fallback_decomposition(task_description)

    def _fallback_decomposition(self, task_description: str) -> List[ActionStep]:
        """
        Fallback decomposition method when LLM fails
        """
        # Simple rule-based decomposition as fallback
        actions = []

        task_lower = task_description.lower()

        if "clean" in task_lower or "pickup" in task_lower or "collect" in task_lower:
            # Add navigation to find objects
            actions.append(ActionStep(
                action_type=ActionType.NAVIGATE_TO_LOCATION,
                parameters={"location": "starting_area"},
                description="Navigate to starting area to scan for objects",
                priority=1,
                estimated_duration=3.0
            ))

            # Add object detection
            actions.append(ActionStep(
                action_type=ActionType.DETECT_OBJECT,
                parameters={"object_category": "debris"},
                description="Detect objects that need to be cleaned up",
                priority=2,
                estimated_duration=2.0
            ))

        elif "go to" in task_lower or "move to" in task_lower:
            # Extract location
            location_match = re.search(r'(?:go to|move to) the? (.+?)(?:\s|$)', task_lower)
            if location_match:
                location = location_match.group(1).strip()
                actions.append(ActionStep(
                    action_type=ActionType.NAVIGATE_TO_LOCATION,
                    parameters={"location": location},
                    description=f"Navigate to {location}",
                    priority=1,
                    estimated_duration=5.0
                ))

        return actions


class TaskExecutionPlanner:
    """
    Plans the execution of decomposed tasks considering robot constraints
    """

    def __init__(self):
        self.constraints = {
            "max_payload": 5.0,  # kg
            "max_reach": 1.5,    # meters
            "max_navigation_distance": 50.0,  # meters
            "battery_life": 3600.0  # seconds
        }

    def plan_execution(self, action_steps: List[ActionStep]) -> List[ActionStep]:
        """
        Plan the execution of action steps considering robot constraints

        Args:
            action_steps: List of decomposed action steps

        Returns:
            List of action steps with execution plan
        """
        # Validate actions against constraints
        validated_steps = []
        for step in action_steps:
            if self._validate_action(step):
                validated_steps.append(step)
            else:
                print(f"Warning: Action '{step.description}' violates constraints and was skipped")

        # Optimize execution order
        optimized_steps = self._optimize_execution_order(validated_steps)

        return optimized_steps

    def _validate_action(self, action_step: ActionStep) -> bool:
        """
        Validate an action against robot constraints
        """
        # Check payload constraints for grasping actions
        if action_step.action_type == ActionType.GRASP_OBJECT:
            weight = action_step.parameters.get('weight', 0.0)
            if weight > self.constraints['max_payload']:
                return False

        # Check reach constraints for manipulation actions
        if action_step.action_type in [ActionType.GRASP_OBJECT, ActionType.PLACE_OBJECT]:
            distance = action_step.parameters.get('distance', 0.0)
            if distance > self.constraints['max_reach']:
                return False

        return True

    def _optimize_execution_order(self, action_steps: List[ActionStep]) -> List[ActionStep]:
        """
        Optimize the order of action steps for efficient execution
        """
        # Sort by priority first
        sorted_steps = sorted(action_steps, key=lambda x: x.priority)

        # Group related actions (e.g., multiple navigations can be combined)
        optimized = []
        i = 0
        while i < len(sorted_steps):
            current = sorted_steps[i]

            # Look for opportunities to combine actions
            if (current.action_type == ActionType.NAVIGATE_TO_LOCATION and
                i + 1 < len(sorted_steps) and
                sorted_steps[i + 1].action_type == ActionType.NAVIGATE_TO_LOCATION):
                # Combine consecutive navigation actions if possible
                combined = self._combine_navigation_actions(current, sorted_steps[i + 1])
                if combined:
                    optimized.append(combined)
                    i += 2  # Skip next action since it's combined
                    continue

            optimized.append(current)
            i += 1

        return optimized

    def _combine_navigation_actions(self, action1: ActionStep, action2: ActionStep) -> Optional[ActionStep]:
        """
        Attempt to combine two navigation actions if beneficial
        """
        # For simplicity, we won't combine navigation actions in this example
        # In a real implementation, this would check if destinations are close
        return None


def main():
    """
    Demonstrate the task decomposition system
    """
    print("LLM-Based Task Decomposition Demo")
    print("=" * 40)

    # Note: You'll need to set your OpenAI API key
    # api_key = os.environ.get("OPENAI_API_KEY")
    # if not api_key:
    #     print("Error: OPENAI_API_KEY environment variable not set")
    #     return

    # For demonstration, we'll use a mock implementation
    print("Using mock implementation for demonstration...")

    # Create a mock task decomposer
    class MockTaskDecomposer:
        def decompose_task(self, task_description, environment_state=None, robot_capabilities=None):
            # Mock implementation for demonstration
            if "clean the room" in task_description.lower():
                return [
                    ActionStep(
                        action_type=ActionType.NAVIGATE_TO_LOCATION,
                        parameters={"x": 1.0, "y": 2.0, "theta": 0.0},
                        description="Navigate to starting position",
                        priority=1,
                        estimated_duration=3.0
                    ),
                    ActionStep(
                        action_type=ActionType.QUERY_ENVIRONMENT,
                        parameters={"scan_area": "360_degrees"},
                        description="Scan environment for objects to clean",
                        priority=2,
                        estimated_duration=5.0
                    ),
                    ActionStep(
                        action_type=ActionType.DETECT_OBJECT,
                        parameters={"target_object": "trash"},
                        description="Detect trash objects in the room",
                        priority=3,
                        estimated_duration=2.0
                    ),
                    ActionStep(
                        action_type=ActionType.GRASP_OBJECT,
                        parameters={"object_id": "trash_item_1", "position": [1.2, 2.1]},
                        description="Grasp the first detected trash item",
                        priority=4,
                        estimated_duration=3.0
                    ),
                    ActionStep(
                        action_type=ActionType.NAVIGATE_TO_LOCATION,
                        parameters={"x": 0.0, "y": 0.0, "theta": 0.0},
                        description="Navigate to trash disposal location",
                        priority=5,
                        estimated_duration=4.0
                    ),
                    ActionStep(
                        action_type=ActionType.RELEASE_OBJECT,
                        parameters={"object_id": "trash_item_1"},
                        description="Release the trash item in disposal area",
                        priority=6,
                        estimated_duration=1.5
                    )
                ]
            else:
                return [
                    ActionStep(
                        action_type=ActionType.NAVIGATE_TO_LOCATION,
                        parameters={"x": 2.0, "y": 1.5, "theta": 0.0},
                        description="Navigate to destination",
                        priority=1,
                        estimated_duration=4.0
                    )
                ]

    # Initialize components
    decomposer = MockTaskDecomposer()  # Use real one when API key is available
    planner = TaskExecutionPlanner()

    # Define test tasks
    test_tasks = [
        "Clean the room by picking up all books and placing them on the shelf",
        "Go to the kitchen and bring me a cup of water",
        "Move to the living room and turn on the TV"
    ]

    for i, task in enumerate(test_tasks, 1):
        print(f"\nTest Task {i}: {task}")
        print("-" * 30)

        # Decompose the task
        action_steps = decomposer.decompose_task(task)

        # Plan execution
        planned_steps = planner.plan_execution(action_steps)

        # Display results
        print(f"Decomposed into {len(planned_steps)} action steps:")
        for j, step in enumerate(planned_steps, 1):
            print(f"  {j}. {step.action_type.value}: {step.description}")
            print(f"     Parameters: {step.parameters}")
            print(f"     Priority: {step.priority}, Duration: ~{step.estimated_duration}s")

    print(f"\nDemo completed! In a real implementation, you would need to:")
    print("1. Set your OPENAI_API_KEY environment variable")
    print("2. Replace the mock implementation with the real one")
    print("3. Connect to a real robot system")


if __name__ == "__main__":
    main()