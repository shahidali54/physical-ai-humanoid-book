#!/usr/bin/env python3
"""
Action Graph Generator for VLA Robotics

This module generates action graphs for humanoid robots based on natural language commands
processed through LLM cognitive planning. The action graphs represent sequences of ROS 2
actions that coordinate humanoid robot behaviors.

The action graph is a directed graph where nodes represent actions and edges represent
dependencies between actions. This allows for complex multi-step tasks to be executed
in a coordinated manner.
"""

import json
import yaml
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, asdict
from enum import Enum


class ActionType(Enum):
    """Enumeration of available action types for humanoid robots"""
    NAVIGATE_TO_LOCATION = "NAVIGATE_TO_LOCATION"
    DETECT_OBJECT = "DETECT_OBJECT"
    GRASP_OBJECT = "GRASP_OBJECT"
    RELEASE_OBJECT = "RELEASE_OBJECT"
    TRANSPORT_OBJECT = "TRANSPORT_OBJECT"
    QUERY_ENVIRONMENT = "QUERY_ENVIRONMENT"
    WAIT_FOR_CONDITION = "WAIT_FOR_CONDITION"
    SPEAK = "SPEAK"
    LISTEN = "LISTEN"


@dataclass
class ActionNode:
    """Represents a single action in the action graph"""
    id: str
    action_type: ActionType
    description: str
    parameters: Dict[str, Any]
    dependencies: List[str]  # List of action IDs this action depends on
    priority: int = 1  # Priority level (1-5, where 1 is highest priority)
    estimated_duration: float = 0.0  # Estimated time to complete in seconds


@dataclass
class ActionGraph:
    """Represents an action graph for coordinating robot behaviors"""
    name: str
    description: str
    nodes: List[ActionNode]
    start_nodes: List[str]  # Actions that can be started immediately
    end_nodes: List[str]    # Actions that complete the graph


class ActionGraphGenerator:
    """
    Generates action graphs from LLM-processed natural language commands
    for humanoid robot execution.
    """

    def __init__(self):
        """Initialize the action graph generator"""
        self.available_actions = [
            ActionType.NAVIGATE_TO_LOCATION,
            ActionType.DETECT_OBJECT,
            ActionType.GRASP_OBJECT,
            ActionType.RELEASE_OBJECT,
            ActionType.TRANSPORT_OBJECT,
            ActionType.QUERY_ENVIRONMENT,
            ActionType.WAIT_FOR_CONDITION,
            ActionType.SPEAK,
            ActionType.LISTEN
        ]

    def generate_from_llm_plan(self, llm_plan: List[Dict[str, Any]]) -> ActionGraph:
        """
        Generate an action graph from a plan produced by LLM cognitive planning.

        Args:
            llm_plan: List of action dictionaries from LLM planning

        Returns:
            ActionGraph: Generated action graph for robot execution
        """
        nodes = []
        node_ids = set()

        for i, action_data in enumerate(llm_plan):
            # Validate action type
            try:
                action_type = ActionType(action_data['action_type'])
            except ValueError:
                raise ValueError(f"Invalid action type: {action_data['action_type']}")

            # Ensure unique node ID
            node_id = f"action_{i:03d}"
            while node_id in node_ids:
                i += 1
                node_id = f"action_{i:03d}"
            node_ids.add(node_id)

            # Create action node
            node = ActionNode(
                id=node_id,
                action_type=action_type,
                description=action_data.get('description', ''),
                parameters=action_data.get('parameters', {}),
                dependencies=action_data.get('dependencies', []),
                priority=action_data.get('priority', 1),
                estimated_duration=action_data.get('estimated_duration', 0.0)
            )
            nodes.append(node)

        # Calculate dependencies and determine start/end nodes
        all_node_ids = {node.id for node in nodes}
        dependency_graph = {node.id: set(node.dependencies) for node in nodes}

        # Find nodes with no dependencies (start nodes)
        start_nodes = [
            node.id for node in nodes
            if not dependency_graph[node.id] or not dependency_graph[node.id].intersection(all_node_ids)
        ]

        # Find nodes that are not dependencies of other nodes (end nodes)
        all_dependencies = set()
        for deps in dependency_graph.values():
            all_dependencies.update(deps)

        end_nodes = [
            node.id for node in nodes
            if node.id not in all_dependencies
        ]

        return ActionGraph(
            name="Generated Action Graph",
            description="Action graph generated from LLM cognitive planning",
            nodes=nodes,
            start_nodes=start_nodes,
            end_nodes=end_nodes
        )

    def generate_from_task_description(self, task_description: str, environment_state: str) -> ActionGraph:
        """
        Generate an action graph directly from a natural language task description.

        This method would typically call an LLM to decompose the task first, then
        generate the action graph from the LLM's plan. For this example, we'll
        simulate the process with a basic mapping.

        Args:
            task_description: Natural language task description
            environment_state: Current state of the environment

        Returns:
            ActionGraph: Generated action graph for robot execution
        """
        # In a real implementation, this would call an LLM to decompose the task
        # For this example, we'll create a basic plan based on common task patterns
        if "clean" in task_description.lower():
            # Example plan for cleaning tasks
            llm_plan = [
                {
                    "action_type": "QUERY_ENVIRONMENT",
                    "description": "Find objects to clean",
                    "parameters": {"category": "debris"},
                    "dependencies": [],
                    "priority": 1,
                    "estimated_duration": 2.0
                },
                {
                    "action_type": "NAVIGATE_TO_LOCATION",
                    "description": "Move to object location",
                    "parameters": {"x": 1.2, "y": 0.8},
                    "dependencies": ["action_000"],
                    "priority": 2,
                    "estimated_duration": 5.0
                },
                {
                    "action_type": "DETECT_OBJECT",
                    "description": "Identify specific object",
                    "parameters": {"target_object": "red_cup"},
                    "dependencies": ["action_001"],
                    "priority": 3,
                    "estimated_duration": 2.0
                },
                {
                    "action_type": "GRASP_OBJECT",
                    "description": "Pick up object",
                    "parameters": {"object_id": "red_cup"},
                    "dependencies": ["action_002"],
                    "priority": 4,
                    "estimated_duration": 3.0
                },
                {
                    "action_type": "NAVIGATE_TO_LOCATION",
                    "description": "Move to disposal area",
                    "parameters": {"x": 0.0, "y": 0.0},
                    "dependencies": ["action_003"],
                    "priority": 5,
                    "estimated_duration": 5.0
                },
                {
                    "action_type": "RELEASE_OBJECT",
                    "description": "Place object in disposal",
                    "parameters": {},
                    "dependencies": ["action_004"],
                    "priority": 6,
                    "estimated_duration": 2.0
                }
            ]
        elif "fetch" in task_description.lower() or "get" in task_description.lower():
            # Example plan for fetching tasks
            llm_plan = [
                {
                    "action_type": "QUERY_ENVIRONMENT",
                    "description": "Locate requested object",
                    "parameters": {"target": task_description.split()[-1]},
                    "dependencies": [],
                    "priority": 1,
                    "estimated_duration": 2.0
                },
                {
                    "action_type": "NAVIGATE_TO_LOCATION",
                    "description": "Move to object location",
                    "parameters": {"x": 2.0, "y": 1.5},
                    "dependencies": ["action_000"],
                    "priority": 2,
                    "estimated_duration": 5.0
                },
                {
                    "action_type": "DETECT_OBJECT",
                    "description": "Confirm object identity",
                    "parameters": {"target_object": task_description.split()[-1]},
                    "dependencies": ["action_001"],
                    "priority": 3,
                    "estimated_duration": 2.0
                },
                {
                    "action_type": "GRASP_OBJECT",
                    "description": "Pick up object",
                    "parameters": {"object_id": task_description.split()[-1]},
                    "dependencies": ["action_002"],
                    "priority": 4,
                    "estimated_duration": 3.0
                },
                {
                    "action_type": "NAVIGATE_TO_LOCATION",
                    "description": "Return to user location",
                    "parameters": {"x": 0.0, "y": 0.0},
                    "dependencies": ["action_003"],
                    "priority": 5,
                    "estimated_duration": 5.0
                },
                {
                    "action_type": "RELEASE_OBJECT",
                    "description": "Give object to user",
                    "parameters": {},
                    "dependencies": ["action_004"],
                    "priority": 6,
                    "estimated_duration": 2.0
                }
            ]
        else:
            # Default plan for unknown tasks
            llm_plan = [
                {
                    "action_type": "SPEAK",
                    "description": "Acknowledge task",
                    "parameters": {"text": f"Received task: {task_description}"},
                    "dependencies": [],
                    "priority": 1,
                    "estimated_duration": 1.0
                },
                {
                    "action_type": "QUERY_ENVIRONMENT",
                    "description": "Assess environment",
                    "parameters": {"context": environment_state},
                    "dependencies": ["action_000"],
                    "priority": 2,
                    "estimated_duration": 2.0
                }
            ]

        return self.generate_from_llm_plan(llm_plan)

    def validate_action_graph(self, graph: ActionGraph) -> List[str]:
        """
        Validate the action graph for potential issues.

        Args:
            graph: Action graph to validate

        Returns:
            List of validation issues found
        """
        issues = []

        # Check for circular dependencies
        if self._has_circular_dependencies(graph):
            issues.append("Circular dependencies detected in action graph")

        # Check for unreachable nodes
        unreachable = self._find_unreachable_nodes(graph)
        if unreachable:
            issues.extend([f"Unreachable node: {node_id}" for node_id in unreachable])

        # Validate action types
        for node in graph.nodes:
            if node.action_type not in self.available_actions:
                issues.append(f"Invalid action type in node {node.id}: {node.action_type}")

        return issues

    def _has_circular_dependencies(self, graph: ActionGraph) -> bool:
        """Check if the action graph has circular dependencies using DFS"""
        # Build adjacency list
        adj_list = {node.id: [] for node in graph.nodes}
        for node in graph.nodes:
            for dep in node.dependencies:
                if dep in adj_list:
                    adj_list[dep].append(node.id)

        # DFS to detect cycles
        visited = set()
        rec_stack = set()

        def dfs(node_id):
            visited.add(node_id)
            rec_stack.add(node_id)

            for neighbor in adj_list.get(node_id, []):
                if neighbor not in visited:
                    if dfs(neighbor):
                        return True
                elif neighbor in rec_stack:
                    return True

            rec_stack.remove(node_id)
            return False

        for node in graph.nodes:
            if node.id not in visited:
                if dfs(node.id):
                    return True

        return False

    def _find_unreachable_nodes(self, graph: ActionGraph) -> List[str]:
        """Find nodes that cannot be reached from start nodes"""
        if not graph.start_nodes:
            return [node.id for node in graph.nodes]

        # BFS from start nodes
        reachable = set()
        queue = list(graph.start_nodes)

        while queue:
            current = queue.pop(0)
            if current in reachable:
                continue

            reachable.add(current)

            # Add dependent nodes
            for node in graph.nodes:
                if current in [dep for dep in node.dependencies if dep in [n.id for n in graph.nodes]]:
                    if node.id not in reachable:
                        queue.append(node.id)

        all_node_ids = {node.id for node in graph.nodes}
        unreachable = all_node_ids - reachable

        return list(unreachable)

    def to_json(self, graph: ActionGraph) -> str:
        """Convert action graph to JSON string"""
        graph_dict = {
            "name": graph.name,
            "description": graph.description,
            "nodes": [
                {
                    "id": node.id,
                    "action_type": node.action_type.value,
                    "description": node.description,
                    "parameters": node.parameters,
                    "dependencies": node.dependencies,
                    "priority": node.priority,
                    "estimated_duration": node.estimated_duration
                }
                for node in graph.nodes
            ],
            "start_nodes": graph.start_nodes,
            "end_nodes": graph.end_nodes
        }

        return json.dumps(graph_dict, indent=2)

    def to_yaml(self, graph: ActionGraph) -> str:
        """Convert action graph to YAML string"""
        graph_dict = {
            "name": graph.name,
            "description": graph.description,
            "nodes": [
                {
                    "id": node.id,
                    "action_type": node.action_type.value,
                    "description": node.description,
                    "parameters": node.parameters,
                    "dependencies": node.dependencies,
                    "priority": node.priority,
                    "estimated_duration": node.estimated_duration
                }
                for node in graph.nodes
            ],
            "start_nodes": graph.start_nodes,
            "end_nodes": graph.end_nodes
        }

        return yaml.dump(graph_dict, default_flow_style=False)


def example_usage():
    """Example usage of the ActionGraphGenerator"""
    generator = ActionGraphGenerator()

    # Example 1: Generate from task description
    task = "Clean the living room by picking up all books and placing them on the bookshelf"
    env_state = "Living room with scattered books on floor, couch, and coffee table. Bookshelf located near window."

    print("Generating action graph for cleaning task...")
    graph = generator.generate_from_task_description(task, env_state)

    # Validate the graph
    issues = generator.validate_action_graph(graph)
    if issues:
        print("Validation issues found:")
        for issue in issues:
            print(f"  - {issue}")
    else:
        print("Action graph is valid!")

    print("\nAction Graph JSON:")
    print(generator.to_json(graph))

    # Example 2: Generate from LLM plan
    llm_plan = [
        {
            "action_type": "NAVIGATE_TO_LOCATION",
            "description": "Move to kitchen counter",
            "parameters": {"x": 1.0, "y": 2.0, "theta": 0.0},
            "dependencies": [],
            "priority": 1,
            "estimated_duration": 5.0
        },
        {
            "action_type": "DETECT_OBJECT",
            "description": "Detect the red cup on the counter",
            "parameters": {"target_object": "red cup"},
            "dependencies": ["action_000"],
            "priority": 2,
            "estimated_duration": 2.0
        }
    ]

    print("\n\nGenerating action graph from LLM plan...")
    graph2 = generator.generate_from_llm_plan(llm_plan)

    print("Action Graph 2 YAML:")
    print(generator.to_yaml(graph2))


if __name__ == "__main__":
    example_usage()