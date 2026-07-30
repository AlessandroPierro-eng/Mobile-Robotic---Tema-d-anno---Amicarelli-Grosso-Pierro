#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from functools import partial

# Math and Hungarian Algorithm dependencies
import numpy as np
from scipy.optimize import linear_sum_assignment
import math

class GlobalManager(Node):
    def __init__(self):
        super().__init__('global_manager')
        self.get_logger().warn("===================================================")
        self.get_logger().warn("   [START] Initializing Global Fleet Manager...    ")
        self.get_logger().warn("===================================================")

        # ==========================================
        # 1. GLOBAL STATE AND PARAMETERS
        # ==========================================
        self.robots = ['robot1', 'robot2', 'robot3']
        
        self.state = 'patrol'  # Allowed states: patrol, tactical, search, objective_reached
        
        # Fleet memory mapping
        self.robot_poses = {robot: None for robot in self.robots}
        self.all_robots_ready = False
        
        # Target memory and tactical coordinates
        self.intruder_seen = False
        self.last_intruder_pose = None
        self.last_intruder_time = 0.0
        self.search_start_time = 0.0
        
        # Strategic choke points (Nav2 logical coordinates)
        self.door = [7.97, -6.22]
        self.stairs = [6.9, 1.89]

        # ==========================================
        # 2. ROS 2 DYNAMIC INTERFACES
        # ==========================================
        self.tactical_pubs = {}
        self.state_pubs = {}
        self.odom_subs = []
        self.intruder_subs = []

        for robot in self.robots:
            # Publishers
            self.tactical_pubs[robot] = self.create_publisher(
                PointStamped, f'/{robot}/tactical_order', 10)
            
            self.state_pubs[robot] = self.create_publisher(
                String, f'/{robot}/state', 10)

            # Subscribers
            self.odom_subs.append(self.create_subscription(
                Odometry, f'/{robot}/odom', partial(self.odom_callback, robot_id=robot), 10))
            
            # Use partial to identify the alarm source
            self.intruder_subs.append(self.create_subscription(
                PointStamped, f'/{robot}/global_intruder_position', partial(self.intruder_callback, robot_id=robot), 10))

        # ==========================================
        # 3. DECISION LOOP (3-Second Interval)
        # ==========================================
        self.timer = self.create_timer(3.0, self.global_control_loop)
        self.get_logger().info("[INIT] Global Manager READY. Waiting for robots...")

    # -------------------------------------------------------------------------
    # CALLBACKS
    # -------------------------------------------------------------------------
    def odom_callback(self, msg, robot_id):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_poses[robot_id] = [x, y]
        
        # Log once when all robots are initialized
        if not self.all_robots_ready and all(pose is not None for pose in self.robot_poses.values()):
            self.all_robots_ready = True
            self.get_logger().warn("[ODOM] All robots online. Fleet is operational.")

    def intruder_callback(self, msg, robot_id):
        self.intruder_seen = True
        self.last_intruder_pose = [msg.point.x, msg.point.y]
        self.last_intruder_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().error(f"[ALARM] >>> TRIGGERED BY {robot_id.upper()} <<<")
        self.get_logger().error(f"[ALARM] Estimated global intruder position: X={msg.point.x:.2f}, Y={msg.point.y:.2f}")

    # -------------------------------------------------------------------------
    # GLOBAL STATE MACHINE
    # -------------------------------------------------------------------------
    def global_control_loop(self):
        if any(pose is None for pose in self.robot_poses.values()):
            self.get_logger().info("[STANDBY] Waiting for complete fleet odometry to initiate processing...", throttle_duration_sec=6.0)
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        next_state = self.state

        self.get_logger().info(f"[HEARTBEAT] Current State: {self.state.upper()} | Target Visible: {self.intruder_seen}")

        # --- A. STATE TRANSITION EVALUATION ---
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
                self.get_logger().warn("[TIMEOUT] Target lost for >10s. Transitioning to SEARCH.")
                
        elif self.state == 'search':
            if self.intruder_seen:
                next_state = 'tactical'
            elif (current_time - self.search_start_time) > 15.0:
                next_state = 'patrol'
                self.get_logger().warn("[TIMEOUT] Search failed for >15s. Transitioning to PATROL.")

        # --- B. STATE TRANSITION EXECUTION ---
        if next_state != self.state:
            self.get_logger().error(f"===================================================")
            self.get_logger().error(f"  [TRANSITION] {self.state.upper()} -> {next_state.upper()}  ")
            self.get_logger().error(f"===================================================")
            self.state = next_state
            
            if self.state == 'search':
                self.broadcast_state('search')
            elif self.state == 'patrol':
                self.broadcast_state('patrol')

        # --- C. CONTINUOUS ACTIONS ---
        if self.state == 'tactical':
            self.calculate_and_send_tactical_positions()
            
        elif self.state == 'objective_reached':
            self.get_logger().info("[SUCCESS] TARGET APPREHENDED! Simulation complete.", throttle_duration_sec=5.0)

    # -------------------------------------------------------------------------
    # OPERATIONAL FUNCTIONS AND MATHEMATICS
    # -------------------------------------------------------------------------
    def broadcast_state(self, state_str):
        msg = String()
        msg.data = state_str
        for robot in self.robots:
            self.state_pubs[robot].publish(msg)
        self.get_logger().info(f"[BROADCAST] Dispatched state order '{state_str.upper()}' to fleet.")

    def calculate_and_send_tactical_positions(self):
        """
        Calculates optimal robot-to-target assignment using the Hungarian algorithm.
        """
        if not self.last_intruder_pose:
            return

        self.get_logger().info("--- STARTING ASSIGNMENT CALCULATION (HUNGARIAN ALGORITHM) ---")

        # Strategic targets
        targets = [
            self.last_intruder_pose,
            self.door,
            self.stairs
        ]
        
        target_names = ["Target", "Door", "Stairs"]

        cost_matrix = np.zeros((len(self.robots), len(targets)))
        
        # Populate cost matrix using Manhattan distance
        for i, robot in enumerate(self.robots):
            rx, ry = self.robot_poses[robot]
            for j, target in enumerate(targets):
                tx, ty = target
                cost = abs(rx - tx) + abs(ry - ty)
                cost_matrix[i, j] = cost
                # Optional: log individual costs
                # self.get_logger().info(f"Cost {robot} -> {target_names[j]}: {cost:.2f}m")

        # Solve assignment problem
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # Dispatch optimal orders to robots
        for idx in range(len(self.robots)):
            robot = self.robots[row_ind[idx]]
            target_idx = col_ind[idx]
            assigned_target = targets[target_idx]
            target_name = target_names[target_idx]
            cost_dist = cost_matrix[row_ind[idx], col_ind[idx]]

            self.get_logger().warn(f"[ORDER] {robot.upper()} assigned to: {target_name.upper()} (X:{assigned_target[0]:.2f}, Y:{assigned_target[1]:.2f}) | Est. Dist: {cost_dist:.2f}m")

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.point.x = float(assigned_target[0])
            msg.point.y = float(assigned_target[1])
            msg.point.z = 0.0
            
            # Publish tactical order
            self.tactical_pubs[robot].publish(msg)
            
        self.get_logger().info("--------------------------------------------------------")

    def check_objective_reached(self):
        """
        Checks if at least one robot in the fleet is within 1.0 meter of the target.
        """
        if not self.last_intruder_pose:
            return False
            
        tx, ty = self.last_intruder_pose
        
        for robot, pose in self.robot_poses.items():
            if pose is None:
                continue
                
            rx, ry = pose
            
            # Euclidean distance for final capture condition
            dist = math.hypot(rx - tx, ry - ty)
            
            # Log real-time distances during pursuit
            self.get_logger().info(f"[DISTANCES] {robot.upper()} is {dist:.2f}m from target.")
            
            if dist < 1.0:  
                self.get_logger().error(f"!!! TARGET APPREHENDED BY {robot.upper()} !!! (Distance: {dist:.2f}m)")
                return True
                
        return False

def main(args=None):
    rclpy.init(args=args)
    node = GlobalManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()