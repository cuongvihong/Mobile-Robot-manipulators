#!/usr/bin/env python3
import math
import yaml
from typing import Dict, List, Tuple, Any

import rclpy
from rclpy.node import Node


class GraphBuilderFromMap(Node):
    """
    Bản đầu của graph_builder_from_map:
    - đọc workspace zones / corridor spec từ YAML
    - sinh graph.yaml cho fleet_coordinator_v3

    Chưa phải IRIS thật. Đây là bước trung gian:
    map/corridor spec -> graph.yaml

    Input YAML ví dụ:

    waypoints:
      - {id: A, x: 0.0, y: 0.0}
      - {id: B, x: 1.0, y: 0.0}
      - {id: C, x: 2.0, y: 0.0}
      - {id: D, x: 2.0, y: 1.0}

    corridors:
      - {from: A, to: B, bidirectional: true}
      - {from: B, to: C, bidirectional: true}
      - {from: C, to: D, bidirectional: false}

    Output graph.yaml:

    nodes:
      A: [0.0, 0.0]
      B: [1.0, 0.0]
    edges:
      - {from: A, to: B, weight: 1.0}
      - {from: B, to: C, weight: 1.0}
    """

    def __init__(self) -> None:
        super().__init__('graph_builder_from_map')

        self.declare_parameter('input_file', '')
        self.declare_parameter('output_file', '')
        self.declare_parameter('overwrite_output', True)
        self.declare_parameter('verbose', True)
        self.declare_parameter('default_bidirectional', True)

        self.input_file = str(self.get_parameter('input_file').value)
        self.output_file = str(self.get_parameter('output_file').value)
        self.overwrite_output = bool(self.get_parameter('overwrite_output').value)
        self.verbose = bool(self.get_parameter('verbose').value)
        self.default_bidirectional = bool(self.get_parameter('default_bidirectional').value)

        if self.input_file == '':
            raise RuntimeError('input_file parameter is empty')
        if self.output_file == '':
            raise RuntimeError('output_file parameter is empty')

        self.build_graph_file()
        self.get_logger().info('Graph builder from map finished, shutting down')
        rclpy.shutdown()

    @staticmethod
    def euclid(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x2 - x1, y2 - y1)

    def load_input_yaml(self) -> Dict[str, Any]:
        with open(self.input_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)

    def parse_waypoints(self, waypoints: List[Dict[str, Any]]) -> Dict[str, Tuple[float, float]]:
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

    def parse_corridors(self, corridors: List[Dict[str, Any]], nodes: Dict[str, Tuple[float, float]]) -> List[Dict[str, Any]]:
        edges: List[Dict[str, Any]] = []
        seen = set()

        for c in corridors:
            if 'from' not in c or 'to' not in c:
                raise RuntimeError(f'Invalid corridor entry: {c}')

            u = str(c['from'])
            v = str(c['to'])

            if u not in nodes or v not in nodes:
                raise RuntimeError(f'Corridor references unknown node: {u} -> {v}')

            bidirectional = bool(c.get('bidirectional', self.default_bidirectional))

            if 'weight' in c:
                weight = float(c['weight'])
            else:
                x1, y1 = nodes[u]
                x2, y2 = nodes[v]
                weight = self.euclid(x1, y1, x2, y2)

            if bidirectional:
                normalized = tuple(sorted((u, v)))
                if normalized in seen:
                    self.get_logger().warn(f'Duplicate bidirectional corridor ignored: {u} <-> {v}')
                    continue
                seen.add(normalized)
                edges.append({'from': u, 'to': v, 'weight': weight})
            else:
                directed_key = (u, v)
                if directed_key in seen:
                    self.get_logger().warn(f'Duplicate directed corridor ignored: {u} -> {v}')
                    continue
                seen.add(directed_key)
                edges.append({'from': u, 'to': v, 'weight': weight})

        return edges

    def build_graph_file(self) -> None:
        data = self.load_input_yaml()

        waypoints = data.get('waypoints', [])
        corridors = data.get('corridors', [])

        if len(waypoints) == 0:
            raise RuntimeError('No waypoints found in input file')
        if len(corridors) == 0:
            raise RuntimeError('No corridors found in input file')

        nodes = self.parse_waypoints(waypoints)
        edges = self.parse_corridors(corridors, nodes)

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
    GraphBuilderFromMap()


if __name__ == '__main__':
    main()
