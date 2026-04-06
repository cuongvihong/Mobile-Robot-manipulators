#!/usr/bin/env python3
import math
import heapq
from typing import Dict, List, Optional, Tuple

import yaml

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class PayloadPlannerV2(Node):
    """
    Planner cho payload.
    """

    def __init__(self) -> None:
        super().__init__('payload_planner_v2')

        self.declare_parameter('team_name', 'team_1')
        self.declare_parameter('graph_file', '')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('verbose', True)

        self.team_name = str(self.get_parameter('team_name').value)
        self.graph_file = str(self.get_parameter('graph_file').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        if self.graph_file == '':
            raise RuntimeError('graph_file parameter is empty')

        self.nodes, self.adj, self.edge_weights = self.load_graph_from_yaml(self.graph_file)

        self.payload_pose: Optional[PoseStamped] = None
        self.pickup_pose: Optional[PoseStamped] = None
        self.dropoff_pose: Optional[PoseStamped] = None
        self.current_target_mode: str = 'NONE'   # NONE | PICKUP | DROPOFF

        self.payload_pose_sub = self.create_subscription(
            PoseStamped,
            f'/{self.team_name}/payload_pose',
            self.payload_pose_callback,
            10
        )

        self.pickup_sub = self.create_subscription(
            PoseStamped,
            f'/{self.team_name}/pickup_pose',
            self.pickup_callback,
            10
        )

        self.dropoff_sub = self.create_subscription(
            PoseStamped,
            f'/{self.team_name}/dropoff_pose',
            self.dropoff_callback,
            10
        )

        self.payload_path_pub = self.create_publisher(
            Path,
            f'/{self.team_name}/payload_global_path',
            10
        )

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Payload planner v2 started')

    def load_graph_from_yaml(
        self,
        filepath: str
    ) -> Tuple[Dict[str, Tuple[float, float]], Dict[str, List[str]], Dict[Tuple[str, str], float]]:
        with open(filepath, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        nodes_raw = data.get('nodes', {})
        edges_raw = data.get('edges', [])

        nodes: Dict[str, Tuple[float, float]] = {}
        for node_id, xy in nodes_raw.items():
            nodes[str(node_id)] = (float(xy[0]), float(xy[1]))

        adj: Dict[str, List[str]] = {node_id: [] for node_id in nodes}
        edge_weights: Dict[Tuple[str, str], float] = {}

        for e in edges_raw:
            u = str(e['from'])
            v = str(e['to'])

            if u not in nodes or v not in nodes:
                raise RuntimeError(f'Edge references unknown node: {u} -> {v}')

            w = float(e['weight']) if 'weight' in e else self.euclid(nodes[u], nodes[v])

            adj[u].append(v)
            adj[v].append(u)
            edge_weights[self.normalize_edge(u, v)] = w

        return nodes, adj, edge_weights

    @staticmethod
    def euclid(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    @staticmethod
    def distance_xy(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x2 - x1, y2 - y1)

    @staticmethod
    def normalize_edge(u: str, v: str) -> Tuple[str, str]:
        return tuple(sorted((u, v)))

    def edge_cost(self, u: str, v: str) -> float:
        e = self.normalize_edge(u, v)
        return self.edge_weights.get(e, self.distance_xy(
            self.nodes[u][0], self.nodes[u][1],
            self.nodes[v][0], self.nodes[v][1]
        ))

    def nearest_node(self, x: float, y: float) -> str:
        best_id = ''
        best_dist = float('inf')

        for node_id, (nx, ny) in self.nodes.items():
            d = self.distance_xy(x, y, nx, ny)
            if d < best_dist:
                best_dist = d
                best_id = node_id

        return best_id

    def dijkstra(self, start: str, goal: str) -> List[str]:
        pq: List[Tuple[float, str]] = []
        heapq.heappush(pq, (0.0, start))

        dist: Dict[str, float] = {node: float('inf') for node in self.nodes}
        prev: Dict[str, Optional[str]] = {node: None for node in self.nodes}

        dist[start] = 0.0

        while pq:
            cur_dist, u = heapq.heappop(pq)

            if cur_dist > dist[u]:
                continue

            if u == goal:
                break

            for v in self.adj.get(u, []):
                alt = dist[u] + self.edge_cost(u, v)
                if alt < dist[v]:
                    dist[v] = alt
                    prev[v] = u
                    heapq.heappush(pq, (alt, v))

        if dist[goal] == float('inf'):
            return []

        route = []
        cur = goal
        while cur is not None:
            route.append(cur)
            cur = prev[cur]

        route.reverse()
        return route

    def payload_pose_callback(self, msg: PoseStamped) -> None:
        self.payload_pose = msg

    def pickup_callback(self, msg: PoseStamped) -> None:
        self.pickup_pose = msg
        self.current_target_mode = 'PICKUP'
        if self.verbose:
            self.get_logger().info(
                f'New pickup target: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
            )

    def dropoff_callback(self, msg: PoseStamped) -> None:
        self.dropoff_pose = msg
        self.current_target_mode = 'DROPOFF'
        if self.verbose:
            self.get_logger().info(
                f'New dropoff target: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
            )

    def build_path(self, start_pose: PoseStamped, goal_pose: PoseStamped) -> Optional[Path]:
        sx = start_pose.pose.position.x
        sy = start_pose.pose.position.y
        sq = start_pose.pose.orientation
        start_yaw = yaw_from_quaternion(sq.x, sq.y, sq.z, sq.w)

        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y
        gq = goal_pose.pose.orientation
        goal_yaw = yaw_from_quaternion(gq.x, gq.y, gq.z, gq.w)

        start_node = self.nearest_node(sx, sy)
        goal_node = self.nearest_node(gx, gy)

        route = self.dijkstra(start_node, goal_node)
        if len(route) == 0:
            self.get_logger().warn('No route found for payload')
            return None

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = goal_pose.header.frame_id if goal_pose.header.frame_id else 'odom'

        poses: List[PoseStamped] = []

        start_ps = PoseStamped()
        start_ps.header = path.header
        start_ps.pose.position.x = sx
        start_ps.pose.position.y = sy
        start_ps.pose.position.z = 0.0
        start_ps.pose.orientation = quaternion_from_yaw(start_yaw)
        poses.append(start_ps)

        for i, node_id in enumerate(route):
            x, y = self.nodes[node_id]

            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0

            if i < len(route) - 1:
                nx, ny = self.nodes[route[i + 1]]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = goal_yaw

            ps.pose.orientation = quaternion_from_yaw(yaw)
            poses.append(ps)

        final_ps = PoseStamped()
        final_ps.header = path.header
        final_ps.pose.position.x = gx
        final_ps.pose.position.y = gy
        final_ps.pose.position.z = 0.0
        final_ps.pose.orientation = quaternion_from_yaw(goal_yaw)
        poses.append(final_ps)

        path.poses = poses
        return path

    def timer_callback(self) -> None:
        if self.payload_pose is None:
            return

        if self.current_target_mode == 'PICKUP' and self.pickup_pose is not None:
            path = self.build_path(self.payload_pose, self.pickup_pose)
            if path is not None:
                self.payload_path_pub.publish(path)

        elif self.current_target_mode == 'DROPOFF' and self.dropoff_pose is not None:
            path = self.build_path(self.payload_pose, self.dropoff_pose)
            if path is not None:
                self.payload_path_pub.publish(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PayloadPlannerV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()