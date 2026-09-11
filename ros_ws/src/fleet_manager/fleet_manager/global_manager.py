#!/usr/bin/env python3
"""
Global Fleet Manager for Multi-Robot Coordination.
Implements the Hungarian Algorithm (Linear Sum Assignment) for optimal 
task allocation and tactical positioning during pursuit operations.
"""

import time
import math
import sys  
from functools import partial

import numpy as np
from scipy.optimize import linear_sum_assignment

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


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
        
        self.tactical_choke_points = {
            'door': [7.97, -6.22],
            'stairs': [6.90, 1.89]
        }

        self.tactical_pubs = {}
        self.state_pubs = {}
        self.odom_subs = []
        self.intruder_subs = []

        self._setup_interfaces()

        self.timer = self.create_timer(3.0, self.global_control_loop)
        self.get_logger().info("Global Manager initialized. Awaiting fleet telemetry.")

    def _setup_interfaces(self) -> None:
        """Initializes distributed publishers and subscribers for the fleet."""
        for robot in self.robots:
            self.tactical_pubs[robot] = self.create_publisher(
                PointStamped, f'/{robot}/tactical_order', 10)
            
            self.state_pubs[robot] = self.create_publisher(
                String, f'/{robot}/state', 10)

            self.odom_subs.append(self.create_subscription(
                Odometry, f'/{robot}/odom', 
                partial(self.odom_callback, robot_id=robot), 10))
            
            self.intruder_subs.append(self.create_subscription(
                PointStamped, f'/{robot}/global_intruder_position', 
                partial(self.intruder_callback, robot_id=robot), 10))

    def odom_callback(self, msg: Odometry, robot_id: str) -> None:
        """Updates the internal representation of the fleet's spatial distribution."""
        self.robot_poses[robot_id] = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        
        if not self.all_robots_ready and all(p is not None for p in self.robot_poses.values()):
            self.all_robots_ready = True
            self.get_logger().info("Fleet initialization complete. Odometry synchronized.")

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
        if not self.all_robots_ready:
            self.get_logger().info("Awaiting fleet telemetry...", throttle_duration_sec=6.0)
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
                    "||      Initiating fleet shutdown sequence...             ||\n" +
                    "||                                                        ||\n" +
                    "============================================================"
                )
                time.sleep(1.5)  
                sys.exit(0)      

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
            self.tactical_choke_points['stairs']
        ]
        
        target_names = ['INTRUDER', 'DOOR', 'STAIRS']
        
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
            
            time.sleep(0.5)

    def check_objective_reached(self) -> bool:
        """Evaluates if the interception criteria are met (agent within 1.0m of target)."""
        if not self.last_intruder_pose:
            return False
            
        tx, ty = self.last_intruder_pose
        
        for robot, pose in self.robot_poses.items():
            if pose is None:
                continue
            
            dist = math.hypot(pose[0] - tx, pose[1] - ty)
            
            if dist < 1.0:  
                self.get_logger().info(f"Target apprehended by {robot}. Final distance: {dist:.2f}m.")
                return True
                
        return False


def main(args=None) -> None:
    """Execution entry point."""
    rclpy.init(args=args)
    node = GlobalManager()
    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info("Node gracefully terminated.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()