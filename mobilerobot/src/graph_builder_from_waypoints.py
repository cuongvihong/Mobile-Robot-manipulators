#!/usr/bin/env python3
import math
import yaml
from typing import Dict, List, Tuple, Any

import rclpy
from rclpy.node import Node


class GraphBuilderFromWaypoints(Node):
    """
    Đọc waypoint/lane từ YAML nguồn và sinh ra graph.yaml.

    Input YAML dạng:

    waypoints:
      - {id: A, x: 0.0, y: 0.0}
      - {id: B, x: 1.0, y: 0.0}

    lanes:
      - {from: A, to: B}
      - {from: B, to: C, weight: 2.5}

    Output YAML dạng:

    nodes:
      A: [0.0, 0.0]
      B: [1.0, 0.0]

    edges:
      - {from: A, to: B, weight: 1.0}
      - {from: B, to: C, weight: 2.5}
    """

    def __init__(self) -> None:
        super().__init__('graph_builder_from_waypoints')

        self.declare_parameter('input_file', '')
        self.declare_parameter('output_file', '')
        self.declare_parameter('overwrite_output', True)
        self.declare_parameter('use_euclid_if_missing', True)
        self.declare_parameter('verbose', True)

        self.input_file = str(self.get_parameter('input_file').value)
        self.output_file = str(self.get_parameter('output_file').value)
        self.overwrite_output = bool(self.get_parameter('overwrite_output').value)
        self.use_euclid_if_missing = bool(self.get_parameter('use_euclid_if_missing').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        if self.input_file == '':
            raise RuntimeError('input_file parameter is empty')
        if self.output_file == '':
            raise RuntimeError('output_file parameter is empty')

        self.build_graph_file()

        self.get_logger().info('Graph builder finished, shutting down')
        rclpy.shutdown()

    @staticmethod
    def euclid(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x2 - x1, y2 - y1)

    def load_input_yaml(self) -> Dict[str, Any]:
        with open(self.input_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)

    def validate_waypoints(self, waypoints: List[Dict[str, Any]]) -> Dict[str, Tuple[float, float]]:
        nodes: Dict[str, Tuple[float, float]] = {}

        for wp in waypoints:
            if 'id' not in wp or 'x' not in wp or 'y' not in wp:
                raise RuntimeError(f'Invalid waypoint entry: {wp}')

            node_id = str(wp['id'])
            x = float(wp['x'])
            y = float(wp['y'])

            if node_id in nodes:
                raise RuntimeError(f'Duplicate waypoint id: {node_id}')

            nodes[node_id] = (x, y)

        return nodes

    def build_edges(self, lanes: List[Dict[str, Any]], nodes: Dict[str, Tuple[float, float]]) -> List[Dict[str, Any]]:
        edges: List[Dict[str, Any]] = []
        seen = set()

        for lane in lanes:
            if 'from' not in lane or 'to' not in lane:
                raise RuntimeError(f'Invalid lane entry: {lane}')

            u = str(lane['from'])
            v = str(lane['to'])

            if u not in nodes or v not in nodes:
                raise RuntimeError(f'Lane references unknown node: {u} -> {v}')

            normalized = tuple(sorted((u, v)))
            if normalized in seen:
                self.get_logger().warn(f'Duplicate lane ignored: {u} <-> {v}')
                continue

            seen.add(normalized)

            if 'weight' in lane:
                weight = float(lane['weight'])
            else:
                if not self.use_euclid_if_missing:
                    raise RuntimeError(f'Missing weight for lane: {u} -> {v}')
                x1, y1 = nodes[u]
                x2, y2 = nodes[v]
                weight = self.euclid(x1, y1, x2, y2)

            edges.append({
                'from': u,
                'to': v,
                'weight': float(weight)
            })

        return edges

    def build_graph_file(self) -> None:
        data = self.load_input_yaml()

        waypoints = data.get('waypoints', [])
        lanes = data.get('lanes', [])

        if len(waypoints) == 0:
            raise RuntimeError('No waypoints found in input file')
        if len(lanes) == 0:
            raise RuntimeError('No lanes found in input file')

        nodes = self.validate_waypoints(waypoints)
        edges = self.build_edges(lanes, nodes)

        output = {
            'nodes': {node_id: [xy[0], xy[1]] for node_id, xy in nodes.items()},
            'edges': edges
        }

        mode = 'w' if self.overwrite_output else 'x'
        with open(self.output_file, mode, encoding='utf-8') as f:
            yaml.safe_dump(output, f, sort_keys=False, allow_unicode=True)

        if self.verbose:
            self.get_logger().info(f'Input file : {self.input_file}')
            self.get_logger().info(f'Output file: {self.output_file}')
            self.get_logger().info(f'Nodes      : {len(nodes)}')
            self.get_logger().info(f'Edges      : {len(edges)}')



def main(args=None) -> None:
    rclpy.init(args=args)
    GraphBuilderFromWaypoints()


if __name__ == '__main__':
    main()
