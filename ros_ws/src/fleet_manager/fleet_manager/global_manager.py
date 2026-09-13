#!/usr/bin/env python3
"""
Global Fleet Manager for Multi-Robot Coordination.
Implements the Hungarian Algorithm (Linear Sum Assignment) for optimal 
task allocation and tactical positioning during pursuit operations.
Integrates federated TF injection to maintain architectural independence.
"""

import time
import math
import numpy as np
from scipy.optimize import linear_sum_assignment
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer


class GlobalManager(Node):
    """
    Centralized coordinator for the robotic fleet.
    Manages global state transitions and optimal target assignment.
    """

    def __init__(self) -> None:
        super().__init__('global_manager')
        
        self.robots = ['robot1', 'robot2', 'robot3']
        self.state = 'patrol'  
        
        self.robot_poses = {robot: None for robot in self.robots}
        self.all_robots_ready = False
        
        self.intruder_seen = False
        self.last_intruder_pose = None
        self.last_intruder_time = 0.0
        self.search_start_time = 0.0
        
        # Static geographical points of interest
        self.tactical_choke_points = {
            'door': [7.97, -6.22],
            'stairs': [6.90, 1.89]
        }

        self.tactical_pubs = {}
        self.state_pubs = {}
        self.intruder_subs = []
        self.tf_subs = []
        
        # Core spatial transformation infrastructure (Standalone Buffer)
        self.tf_buffer = Buffer()
        
        self._setup_interfaces()

        # System control loop execution rate (1.0 Hz)
        self.timer = self.create_timer(1.0, self.global_control_loop)
        self.get_logger().info("Global Manager initialized. Awaiting federated TF telemetry.")

    def _setup_interfaces(self) -> None:
        """Initializes distributed publishers, subscribers, and federated TF injectors."""
        qos_static = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        for robot in self.robots:
            self.tactical_pubs[robot] = self.create_publisher(
                PointStamped, f'/{robot}/tactical_order', 10)
            
            self.state_pubs[robot] = self.create_publisher(
                String, f'/{robot}/state', 10)

            self.intruder_subs.append(self.create_subscription(
                PointStamped, f'/{robot}/global_intruder_position', 
                partial(self.intruder_callback, robot_id=robot), 10))

            # Independent TF injection architecture
            self.tf_subs.append(self.create_subscription(
                TFMessage, f'/{robot}/tf', 
                self.dynamic_tf_callback, 10))
                
            self.tf_subs.append(self.create_subscription(
                TFMessage, f'/{robot}/tf_static', 
                self.static_tf_callback, qos_static))

    def dynamic_tf_callback(self, msg: TFMessage) -> None:
        """Injects dynamic coordinate frames into the internal spatial buffer."""
        for transform in msg.transforms:
            self.tf_buffer.set_transform(transform, 'global_manager_internal')

    def static_tf_callback(self, msg: TFMessage) -> None:
        """Injects static coordinate frames into the internal spatial buffer."""
        for transform in msg.transforms:
            self.tf_buffer.set_transform_static(transform, 'global_manager_internal')

    def update_robot_poses(self) -> None:
        """Computes true global coordinates mapping AMCL corrections and odometry."""
        all_ready = True
        for robot in self.robots:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', 
                    f'{robot}/base_link', 
                    rclpy.time.Time()
                )
                self.robot_poses[robot] = [
                    transform.transform.translation.x,
                    transform.transform.translation.y
                ]
            except Exception:
                all_ready = False
                
        if not self.all_robots_ready and all_ready and all(p is not None for p in self.robot_poses.values()):
            self.all_robots_ready = True
            self.get_logger().info("Fleet initialization complete. AMCL-corrected TF synchronized.")

    def intruder_callback(self, msg: PointStamped, robot_id: str) -> None:
        """Processes target detection events and updates the global state."""
        self.intruder_seen = True
        self.last_intruder_pose = [msg.point.x, msg.point.y]
        self.last_intruder_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().warn(
            f"Target detected by {robot_id}. Estimated coordinates: ({msg.point.x:.2f}, {msg.point.y:.2f})."
        )

    def global_control_loop(self) -> None:
        """Executes the core control logic and state transitions for fleet behavior."""
        self.update_robot_poses()
        
        if not self.all_robots_ready:
            self.get_logger().info("Awaiting fleet telemetry (Federated TF)...", throttle_duration_sec=6.0)
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        next_state = self.state

        if self.state == 'patrol':
            if self.intruder_seen:
                next_state = 'tactical'
                
        elif self.state == 'tactical':
            if self.check_objective_reached():
                next_state = 'objective_reached'
            elif (current_time - self.last_intruder_time) > 10.0:
                next_state = 'search'
                self.search_start_time = current_time
                self.intruder_seen = False
                self.get_logger().info("Target lost. Transitioning to search protocol.")
                
        elif self.state == 'search':
            if self.intruder_seen:
                next_state = 'tactical'
            elif (current_time - self.search_start_time) > 15.0:
                next_state = 'patrol'
                self.get_logger().info("Search protocol timeout. Resuming standard patrol.")

        if next_state != self.state:
            self.get_logger().info(f"State transition: {self.state.upper()} -> {next_state.upper()}")
            self.state = next_state
            
            if self.state in ['search', 'patrol', 'objective_reached']:
                self.broadcast_state(self.state)

            if self.state == 'objective_reached':
                self.get_logger().warn(
                    "\n" +
                    "============================================================\n" +
                    "||                                                        ||\n" +
                    "||      TARGET APPREHENDED - SIMULATION COMPLETED         ||\n" +
                    "||      Tactical operations halted. Fleet standing by.    ||\n" +
                    "||                                                        ||\n" +
                    "============================================================"
                )
                # Node execution is intentionally preserved to anchor the final state.

        if self.state == 'tactical':
            self.calculate_and_send_tactical_positions()

    def broadcast_state(self, state_str: str) -> None:
        """Propagates global state constraints to all decentralized agents."""
        msg = String(data=state_str)
        for robot in self.robots:
            self.state_pubs[robot].publish(msg)

    def calculate_and_send_tactical_positions(self) -> None:
        """
        Computes optimal target assignments using the Hungarian Algorithm.
        Minimizes the global operational cost defined by the Manhattan distance 
        between current agent positions and assigned strategic points.
        """
        if not self.last_intruder_pose:
            return

        targets = [
            self.last_intruder_pose,
            self.tactical_choke_points['door'],
            self.last_intruder_pose
            #self.tactical_choke_points['stairs']
        ]
        
        # target_names = ['INTRUDER', 'DOOR', 'STAIRS']
        target_names = ['INTRUDER', 'DOOR', 'INTRUDER']
        
        cost_matrix = np.zeros((len(self.robots), len(targets)))
        
        for i, robot in enumerate(self.robots):
            rx, ry = self.robot_poses[robot]
            for j, (tx, ty) in enumerate(targets):
                cost_matrix[i, j] = abs(rx - tx) + abs(ry - ty)

        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        self.get_logger().warn(
            "\n" +
            "========================================\n" +
            "||  TACTICAL DEPLOYMENT ASSIGNMENTS   ||\n" +
            "========================================"
        )

        for idx in range(len(self.robots)):
            robot = self.robots[row_ind[idx]]
            assigned_target = targets[col_ind[idx]]
            assigned_name = target_names[col_ind[idx]]

            self.get_logger().warn(f" >>> {robot.upper()} -> {assigned_name}")

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.point.x = float(assigned_target[0])
            msg.point.y = float(assigned_target[1])
            msg.point.z = 0.0
            
            self.tactical_pubs[robot].publish(msg)
            
            # Rate limiting interval to prevent ROS timer callback overrun
            time.sleep(0.1)

    def check_objective_reached(self) -> bool:
        """Evaluates if interception criteria are met (agent within 1.5m of target)."""
        if not self.last_intruder_pose:
            return False
            
        tx, ty = self.last_intruder_pose
        
        for robot, pose in self.robot_poses.items():
            if pose is None:
                continue
            
            dist = math.hypot(pose[0] - tx, pose[1] - ty)
            
            if dist < 1.5:  
                self.get_logger().info(f"Target apprehended by {robot}. Final distance: {dist:.2f}m.")
                return True
                
        return False


def main(args=None) -> None:
    """Node execution entry point."""
    rclpy.init(args=args)
    node = GlobalManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()