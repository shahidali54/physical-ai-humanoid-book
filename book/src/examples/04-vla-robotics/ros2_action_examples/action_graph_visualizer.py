#!/usr/bin/env python3
"""
Action Graph Visualizer for VLA Robotics

This module provides visualization capabilities for action graphs generated
by the LLM-based cognitive planning system. It can create visual representations
of action graphs in various formats including PNG, SVG, and interactive HTML.
"""

import json
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
import argparse
import os
from pathlib import Path

# Import our action graph components
from action_graph_generator import ActionGraph, ActionNode, ActionType


@dataclass
class VisualizationStyle:
    """Style configuration for action graph visualization"""
    node_color: str = '#e3f2fd'
    llm_color: str = '#fff3e0'
    process_color: str = '#f3e5f5'
    output_color: str = '#e8f5e9'
    edge_color: str = '#444444'
    node_size: int = 2000
    font_size: int = 10
    figsize: tuple = (12, 8)


class ActionGraphVisualizer:
    """
    Visualizes action graphs for humanoid robots, showing the relationships
    between different actions and their dependencies.
    """

    def __init__(self, style: Optional[VisualizationStyle] = None):
        """
        Initialize the visualizer with optional styling.

        Args:
            style: Visualization style configuration
        """
        self.style = style or VisualizationStyle()

    def visualize_to_matplotlib(self, graph: ActionGraph, title: str = "Action Graph") -> plt.Figure:
        """
        Create a matplotlib visualization of the action graph.

        Args:
            graph: The action graph to visualize
            title: Title for the visualization

        Returns:
            Matplotlib figure object
        """
        # Create a directed graph using NetworkX
        G = nx.DiGraph()

        # Add nodes to the graph
        for node in graph.nodes:
            G.add_node(node.id,
                      action_type=node.action_type.value,
                      description=node.description,
                      priority=node.priority)

        # Add edges based on dependencies
        for node in graph.nodes:
            for dep_id in node.dependencies:
                if dep_id in G.nodes:  # Only add edge if dependency exists
                    G.add_edge(dep_id, node.id)

        # Create the figure and axis
        fig, ax = plt.subplots(figsize=self.style.figsize)

        # Position nodes using a hierarchical layout
        pos = self._hierarchical_layout(G, graph.start_nodes)

        # Define colors based on action type
        node_colors = []
        for node_id in G.nodes:
            # Find the corresponding node in our action graph
            action_node = next((n for n in graph.nodes if n.id == node_id), None)
            if action_node:
                if action_node.action_type in [ActionType.SPEAK, ActionType.LISTEN]:
                    node_colors.append(self.style.llm_color)
                elif action_node.action_type in [ActionType.NAVIGATE_TO_LOCATION, ActionType.DETECT_OBJECT]:
                    node_colors.append(self.style.process_color)
                elif action_node.action_type in [ActionType.GRASP_OBJECT, ActionType.RELEASE_OBJECT]:
                    node_colors.append(self.style.output_color)
                else:
                    node_colors.append(self.style.node_color)
            else:
                node_colors.append(self.style.node_color)

        # Draw the graph
        nx.draw(G, pos, ax=ax,
                node_color=node_colors,
                node_size=self.style.node_size,
                font_size=self.style.font_size,
                font_weight='bold',
                arrows=True,
                arrowsize=20,
                edge_color=self.style.edge_color,
                with_labels=True,
                labels={node: f"{node}\n{G.nodes[node]['action_type']}" for node in G.nodes})

        # Set title
        ax.set_title(title, fontsize=14, fontweight='bold')

        # Add a legend
        legend_elements = [
            patches.Patch(color=self.style.llm_color, label='Communication (Speak/Listen)'),
            patches.Patch(color=self.style.process_color, label='Navigation/Detection'),
            patches.Patch(color=self.style.output_color, label='Manipulation (Grasp/Release)'),
            patches.Patch(color=self.style.node_color, label='Other Actions')
        ]
        ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(0, 1))

        # Adjust layout to prevent clipping
        plt.tight_layout()

        return fig

    def _hierarchical_layout(self, G: nx.DiGraph, start_nodes: List[str]) -> Dict[str, tuple]:
        """
        Create a hierarchical layout for the action graph based on dependencies.

        Args:
            G: NetworkX directed graph
            start_nodes: List of node IDs that can be executed immediately

        Returns:
            Dictionary mapping node IDs to (x, y) positions
        """
        # Calculate the level of each node based on dependencies
        node_levels = {}

        # Start nodes are at level 0
        for node in start_nodes:
            node_levels[node] = 0

        # Calculate levels for remaining nodes
        changed = True
        while changed:
            changed = False
            for node in G.nodes:
                if node not in node_levels:
                    # Check if all dependencies have levels assigned
                    dependencies = [pred for pred in G.predecessors(node)]
                    if all(dep in node_levels for dep in dependencies):
                        # Level is max dependency level + 1
                        max_dep_level = max(node_levels[dep] for dep in dependencies)
                        node_levels[node] = max_dep_level + 1
                        changed = True

        # Create positions based on levels
        pos = {}
        for node, level in node_levels.items():
            # Count nodes at this level to space them evenly
            nodes_at_level = [n for n, l in node_levels.items() if l == level]
            x_positions = [i for i in range(len(nodes_at_level))]

            # Find the index of current node in this level
            node_index = nodes_at_level.index(node)
            x = x_positions[node_index] - len(x_positions) / 2
            y = -level * 2  # Negative to have higher levels at the top

            pos[node] = (x, y)

        return pos

    def visualize_to_file(self, graph: ActionGraph, output_path: str, title: str = "Action Graph"):
        """
        Create a visualization and save it to a file.

        Args:
            graph: The action graph to visualize
            output_path: Path to save the visualization
            title: Title for the visualization
        """
        fig = self.visualize_to_matplotlib(graph, title)

        # Create directory if it doesn't exist
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)

        fig.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close(fig)  # Close the figure to free memory

    def visualize_to_interactive_html(self, graph: ActionGraph, output_path: str, title: str = "Action Graph"):
        """
        Create an interactive HTML visualization of the action graph.

        Args:
            graph: The action graph to visualize
            output_path: Path to save the HTML file
            title: Title for the visualization
        """
        # Create the HTML content
        html_content = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>{title}</title>
            <script src="https://d3js.org/d3.v7.min.js"></script>
            <style>
                body {{ font-family: Arial, sans-serif; margin: 0; padding: 20px; }}
                #graph-container {{ width: 100%; height: 600px; border: 1px solid #ccc; }}
                .node {{ stroke: #fff; stroke-width: 2px; }}
                .link {{ stroke: #999; stroke-opacity: 0.6; }}
                .node-text {{ font-size: 12px; text-anchor: middle; dominant-baseline: middle; }}
            </style>
        </head>
        <body>
            <h1>{title}</h1>
            <div id="graph-container"></div>

            <script>
                // Action graph data
                const graphData = {self._convert_to_d3_format(graph)};

                // Set up the SVG container
                const width = document.getElementById('graph-container').clientWidth;
                const height = document.getElementById('graph-container').clientHeight;

                const svg = d3.select("#graph-container")
                    .append("svg")
                    .attr("width", width)
                    .attr("height", height);

                // Create the simulation
                const simulation = d3.forceSimulation(graphData.nodes)
                    .force("link", d3.forceLink(graphData.links).id(d => d.id).distance(100))
                    .force("charge", d3.forceManyBody().strength(-300))
                    .force("center", d3.forceCenter(width / 2, height / 2));

                // Add links
                const link = svg.append("g")
                    .attr("class", "links")
                    .selectAll("line")
                    .data(graphData.links)
                    .enter().append("line")
                    .attr("class", "link")
                    .attr("stroke-width", 2);

                // Add nodes
                const node = svg.append("g")
                    .attr("class", "nodes")
                    .selectAll("circle")
                    .data(graphData.nodes)
                    .enter().append("circle")
                    .attr("class", "node")
                    .attr("r", 20)
                    .attr("fill", d => d.color)
                    .call(d3.drag()
                        .on("start", dragstarted)
                        .on("drag", dragged)
                        .on("end", dragended));

                // Add node labels
                const text = svg.append("g")
                    .attr("class", "node-texts")
                    .selectAll("text")
                    .data(graphData.nodes)
                    .enter().append("text")
                    .attr("class", "node-text")
                    .text(d => d.shortLabel)
                    .attr("dx", 0)
                    .attr("dy", 0);

                // Update positions on each tick
                simulation.on("tick", () => {
                    link
                        .attr("x1", d => d.source.x)
                        .attr("y1", d => d.source.y)
                        .attr("x2", d => d.target.x)
                        .attr("y2", d => d.target.y);

                    node
                        .attr("cx", d => d.x = Math.max(20, Math.min(width - 20, d.x)))
                        .attr("cy", d => d.y = Math.max(20, Math.min(height - 20, d.y)));

                    text
                        .attr("x", d => d.x)
                        .attr("y", d => d.y);
                });

                // Drag functions
                function dragstarted(event, d) {{
                    if (!event.active) simulation.alphaTarget(0.3).restart();
                    d.fx = d.x;
                    d.fy = d.y;
                }}

                function dragged(event, d) {{
                    d.fx = event.x;
                    d.fy = event.y;
                }}

                function dragended(event, d) {{
                    if (!event.active) simulation.alphaTarget(0);
                    d.fx = null;
                    d.fy = null;
                }}
            </script>
        </body>
        </html>
        """

        # Create directory if it doesn't exist
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)

        # Write the HTML file
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(html_content)

    def _convert_to_d3_format(self, graph: ActionGraph) -> Dict[str, Any]:
        """
        Convert the action graph to a format suitable for D3.js visualization.

        Args:
            graph: The action graph to convert

        Returns:
            Dictionary in D3.js force-directed graph format
        """
        # Create nodes with colors based on action type
        nodes = []
        for node in graph.nodes:
            # Determine color based on action type
            if node.action_type in [ActionType.SPEAK, ActionType.LISTEN]:
                color = "#FFA726"  # Orange for communication
            elif node.action_type in [ActionType.NAVIGATE_TO_LOCATION, ActionType.DETECT_OBJECT]:
                color = "#AB47BC"  # Purple for navigation/detection
            elif node.action_type in [ActionType.GRASP_OBJECT, ActionType.RELEASE_OBJECT]:
                color = "#66BB6A"  # Green for manipulation
            else:
                color = "#42A5F5"  # Blue for other actions

            # Create a short label for display
            short_label = node.action_type.value.split('_')[0]  # Use first part of action type

            nodes.append({
                'id': node.id,
                'action_type': node.action_type.value,
                'description': node.description,
                'color': color,
                'shortLabel': short_label
            })

        # Create links based on dependencies
        links = []
        for node in graph.nodes:
            for dep_id in node.dependencies:
                if dep_id in [n['id'] for n in nodes]:  # Only add link if dependency exists
                    links.append({
                        'source': dep_id,
                        'target': node.id
                    })

        return {
            'nodes': nodes,
            'links': links
        }


def load_action_graph_from_json(json_path: str) -> ActionGraph:
    """
    Load an action graph from a JSON file.

    Args:
        json_path: Path to the JSON file containing the action graph

    Returns:
        ActionGraph object
    """
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    nodes = []
    for node_data in data['nodes']:
        node = ActionNode(
            id=node_data['id'],
            action_type=ActionType(node_data['action_type']),
            description=node_data['description'],
            parameters=node_data['parameters'],
            dependencies=node_data['dependencies'],
            priority=node_data['priority'],
            estimated_duration=node_data['estimated_duration']
        )
        nodes.append(node)

    return ActionGraph(
        name=data['name'],
        description=data['description'],
        nodes=nodes,
        start_nodes=data['start_nodes'],
        end_nodes=data['end_nodes']
    )


def example_usage():
    """Example usage of the ActionGraphVisualizer"""
    from action_graph_generator import ActionGraphGenerator

    # Create a sample action graph
    generator = ActionGraphGenerator()
    sample_graph = generator.generate_from_task_description(
        "Clean the living room by picking up books and placing them on the shelf",
        "Living room with books scattered on floor and couch"
    )

    # Create visualizer
    visualizer = ActionGraphVisualizer()

    # Visualize to PNG
    print("Creating PNG visualization...")
    visualizer.visualize_to_file(
        sample_graph,
        "action_graph_visualization.png",
        "Sample Action Graph: Clean Living Room"
    )
    print("PNG visualization saved to action_graph_visualization.png")

    # Visualize to interactive HTML
    print("Creating interactive HTML visualization...")
    visualizer.visualize_to_interactive_html(
        sample_graph,
        "action_graph_visualization.html",
        "Interactive Action Graph: Clean Living Room"
    )
    print("HTML visualization saved to action_graph_visualization.html")

    # For this example, also show how to load and visualize from JSON
    # First, save the graph to JSON
    json_content = generator.to_json(sample_graph)
    with open("sample_action_graph.json", 'w', encoding='utf-8') as f:
        f.write(json_content)

    # Then load it back and visualize
    loaded_graph = load_action_graph_from_json("sample_action_graph.json")
    visualizer.visualize_to_file(
        loaded_graph,
        "loaded_action_graph_visualization.png",
        "Loaded Action Graph Visualization"
    )
    print("Visualization of loaded graph saved to loaded_action_graph_visualization.png")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Action Graph Visualizer for VLA Robotics')
    parser.add_argument('--input', '-i', type=str, help='Input JSON file containing action graph')
    parser.add_argument('--output', '-o', type=str, help='Output file path (PNG or HTML)')
    parser.add_argument('--title', '-t', type=str, default='Action Graph Visualization', help='Title for the visualization')
    parser.add_argument('--format', '-f', type=str, choices=['png', 'html'], default='png', help='Output format')

    args = parser.parse_args()

    visualizer = ActionGraphVisualizer()

    if args.input and args.output:
        # Load graph from file and visualize
        graph = load_action_graph_from_json(args.input)

        if args.format == 'png':
            visualizer.visualize_to_file(graph, args.output, args.title)
            print(f"Visualization saved to {args.output}")
        else:  # html
            visualizer.visualize_to_interactive_html(graph, args.output, args.title)
            print(f"Interactive visualization saved to {args.output}")
    else:
        # Run example usage
        print("Running example usage...")
        example_usage()