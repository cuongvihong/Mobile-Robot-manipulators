#!/usr/bin/env python3
import math
import yaml
from typing import Dict, List, Tuple, Any

import rclpy
from rclpy.node import Node


class GraphBuilderFromIris(Node):
    """
    Đọc IRIS/corridor decomposition output từ YAML và sinh graph.yaml.

    Input YAML dự kiến:

    regions:
      - id: R1
        center: [0.0, 0.0]
      - id: R2
        center: [1.0, 0.0]

    portals:
      - from: R1
        to: R2
        point: [0.5, 0.0]
        bidirectional: true
      - from: R2
        to: R3
        point: [1.5, 0.2]
        weight: 1.8
        bidirectional: false

    Output graph.yaml:

    nodes:
      R1: [0.0, 0.0]
      R2: [1.0, 0.0]

    edges:
      - {from: R1, to: R2, weight: 1.0}
    """

    def __init__(self) -> None:
        super().__init__('graph_builder_from_iris')

        self.declare_parameter('input_file', '')
        self.declare_parameter('output_file', '')
        self.declare_parameter('overwrite_output', True)
        self.declare_parameter('verbose', True)
        self.declare_parameter('default_bidirectional', True)
        self.declare_parameter('weight_mode', 'euclid_centers')
        # weight_mode:
        #   - euclid_centers
        #   - portal_detour
        #   - explicit_only

        self.input_file = str(self.get_parameter('input_file').value)
        self.output_file = str(self.get_parameter('output_file').value)
        self.overwrite_output = bool(self.get_parameter('overwrite_output').value)
        self.verbose = bool(self.get_parameter('verbose').value)
        self.default_bidirectional = bool(self.get_parameter('default_bidirectional').value)
        self.weight_mode = str(self.get_parameter('weight_mode').value)

        if self.input_file == '':
            raise RuntimeError('input_file parameter is empty')
        if self.output_file == '':
            raise RuntimeError('output_file parameter is empty')

        self.build_graph_file()
        self.get_logger().info('Graph builder from IRIS finished, shutting down')
        rclpy.shutdown()

    @staticmethod
    def euclid(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x2 - x1, y2 - y1)

    def load_input_yaml(self) -> Dict[str, Any]:
        with open(self.input_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)

    def parse_regions(self, regions: List[Dict[str, Any]]) -> Dict[str, Tuple[float, float]]:
        nodes: Dict[str, Tuple[float, float]] = {}

        for region in regions:
            if 'id' not in region:
                raise RuntimeError(f'Invalid region entry, missing id: {region}')

            region_id = str(region['id'])

            if 'center' in region:
                c = region['center']
                if not isinstance(c, list) or len(c) != 2:
                    raise RuntimeError(f'Invalid center for region {region_id}: {c}')
                x = float(c[0])
                y = float(c[1])

            elif 'centroid' in region:
                c = region['centroid']
                if not isinstance(c, list) or len(c) != 2:
                    raise RuntimeError(f'Invalid centroid for region {region_id}: {c}')
                x = float(c[0])
                y = float(c[1])

            else:
                raise RuntimeError(
                    f'Region {region_id} missing center/centroid. '
                    'Expected one of: center, centroid'
                )

            if region_id in nodes:
                raise RuntimeError(f'Duplicate region id: {region_id}')

            nodes[region_id] = (x, y)

        return nodes

    def compute_edge_weight(
        self,
        u: str,
        v: str,
        nodes: Dict[str, Tuple[float, float]],
        portal: Dict[str, Any]
    ) -> float:
        if 'weight' in portal:
            return float(portal['weight'])

        x1, y1 = nodes[u]
        x2, y2 = nodes[v]

        if self.weight_mode == 'explicit_only':
            raise RuntimeError(f'Missing explicit weight for portal {u} -> {v}')

        if self.weight_mode == 'euclid_centers':
            return self.euclid(x1, y1, x2, y2)

        if self.weight_mode == 'portal_detour':
            if 'point' not in portal:
                return self.euclid(x1, y1, x2, y2)

            p = portal['point']
            if not isinstance(p, list) or len(p) != 2:
                raise RuntimeError(f'Invalid portal point for {u}->{v}: {p}')

            px = float(p[0])
            py = float(p[1])

            return self.euclid(x1, y1, px, py) + self.euclid(px, py, x2, y2)

        raise RuntimeError(f'Unknown weight_mode: {self.weight_mode}')

    def parse_portals(
        self,
        portals: List[Dict[str, Any]],
        nodes: Dict[str, Tuple[float, float]]
    ) -> List[Dict[str, Any]]:
        edges: List[Dict[str, Any]] = []
        seen = set()

        for portal in portals:
            if 'from' not in portal or 'to' not in portal:
                raise RuntimeError(f'Invalid portal entry: {portal}')

            u = str(portal['from'])
            v = str(portal['to'])

            if u not in nodes or v not in nodes:
                raise RuntimeError(f'Portal references unknown region: {u} -> {v}')

            bidirectional = bool(portal.get('bidirectional', self.default_bidirectional))
            weight = self.compute_edge_weight(u, v, nodes, portal)

            if bidirectional:
                normalized = tuple(sorted((u, v)))
                if normalized in seen:
                    self.get_logger().warn(f'Duplicate bidirectional portal ignored: {u} <-> {v}')
                    continue
                seen.add(normalized)
                edges.append({
                    'from': u,
                    'to': v,
                    'weight': float(weight)
                })
            else:
                directed_key = (u, v)
                if directed_key in seen:
                    self.get_logger().warn(f'Duplicate directed portal ignored: {u} -> {v}')
                    continue
                seen.add(directed_key)
                edges.append({
                    'from': u,
                    'to': v,
                    'weight': float(weight)
                })

        return edges

    def build_graph_file(self) -> None:
        data = self.load_input_yaml()

        regions = data.get('regions', [])
        portals = data.get('portals', [])

        if len(regions) == 0:
            raise RuntimeError('No regions found in input file')
        if len(portals) == 0:
            raise RuntimeError('No portals found in input file')

        nodes = self.parse_regions(regions)
        edges = self.parse_portals(portals, nodes)

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
            self.get_logger().info(f'Regions    : {len(nodes)}')
            self.get_logger().info(f'Edges      : {len(edges)}')
            self.get_logger().info(f'Weight mode: {self.weight_mode}')


def main(args=None) -> None:
    rclpy.init(args=args)
    GraphBuilderFromIris()


if __name__ == '__main__':
    main()