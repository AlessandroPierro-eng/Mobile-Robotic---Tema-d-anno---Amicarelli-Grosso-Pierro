#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PointStamped, PoseWithCovarianceStamped
import os
import time
import math
from functools import partial

class MultiAgentDashboard(Node):
    def __init__(self):
        super().__init__('multi_agent_dashboard')
        
        self.robots = ['robot1', 'robot2', 'robot3']
        self.state = "STANDBY"
        
        self.velocities = {robot: {'v': 0.0, 'w': 0.0} for robot in self.robots}
        self.robot_poses = {robot: None for robot in self.robots}
        self.tactical_roles = {robot: "NESSUNO" for robot in self.robots}
        
        self.logs = ["-", "-", "-", "-", "-"] 
        self.last_alarm_time = 0.0
        self.last_intruder_pose = None
        
        self.choke_points = {
            'PORTA': [7.97, -6.22],
            'SCALE': [6.90, 1.89]
        }
        
        self.create_subscription(String, '/robot1/state', self.state_cb, 10)
        
        for robot in self.robots:
            # Velocità
            self.create_subscription(
                Twist, f'/{robot}/cmd_vel', 
                partial(self.vel_cb, robot_id=robot), 10
            )
            
            # Posizione Assoluta AMCL (Corretta per il frame MAP)
            self.create_subscription(
                PoseWithCovarianceStamped, f'/{robot}/amcl_pose', 
                partial(self.amcl_pose_cb, robot_id=robot), 10
            )
            
            # Ordini Tattici
            self.create_subscription(
                PointStamped, f'/{robot}/tactical_order', 
                partial(self.order_cb, robot_id=robot), 10
            )
            
            # Avvistamenti
            self.create_subscription(
                PointStamped, f'/{robot}/global_intruder_position', 
                partial(self.intruder_alert_cb, source_id=robot), 10
            )
        
        self.create_timer(0.2, self.draw_dashboard)

    def state_cb(self, msg):
        new_state = msg.data.upper()
        if new_state != self.state:
            self.add_log("SISTEMA", f"Transizione di stato: {new_state}")
            if new_state == "PATROL":
                for r in self.robots:
                    self.tactical_roles[r] = "PATTUGLIAMENTO"
        self.state = new_state

    def vel_cb(self, msg, robot_id):
        self.velocities[robot_id]['v'] = msg.linear.x
        self.velocities[robot_id]['w'] = msg.angular.z

    def amcl_pose_cb(self, msg, robot_id):
        # Estrae le coordinate globali corrette da AMCL
        self.robot_poses[robot_id] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def order_cb(self, msg, robot_id):
        tx, ty = msg.point.x, msg.point.y
        assigned_role = "INSEGUTORE"
        
        for name, coords in self.choke_points.items():
            if abs(tx - coords[0]) < 0.5 and abs(ty - coords[1]) < 0.5:
                assigned_role = name
                break
                
        self.tactical_roles[robot_id] = assigned_role

    def intruder_alert_cb(self, msg, source_id):
        self.last_intruder_pose = (msg.point.x, msg.point.y)
        
        current_time = time.time()
        if (current_time - self.last_alarm_time) > 3.0:
            self.add_log(source_id, f"INTRUSO AVVISTATO A: X={msg.point.x:.1f}, Y={msg.point.y:.1f}")
            self.last_alarm_time = current_time

    def add_log(self, source_id, message):
        formatted_log = f"[{source_id.upper()}] {message}"
        self.logs.append(formatted_log)
        if len(self.logs) > 5:
            self.logs.pop(0)

    def draw_dashboard(self):
        os.system('clear')
        
        C_RED = '\033[91m'
        C_GREEN = '\033[92m'
        C_YELLOW = '\033[93m'
        C_CYAN = '\033[96m'
        C_BOLD = '\033[1m'
        C_END = '\033[0m'
        
        state_color = C_GREEN
        if self.state in ["TACTICAL", "PURSUIT"]:
            state_color = C_RED
        elif self.state == "SEARCH":
            state_color = C_YELLOW

        dashboard = f"""
================================================================================
{C_CYAN}{C_BOLD}  MOBILE ROBOTICS - MULTI-AGENT TACTICAL DASHBOARD {C_END}
================================================================================

[ STATO GLOBALE SISTEMA ] 
> {state_color}{C_BOLD}{self.state}{C_END}

--------------------------------------------------------------------------------
[ TELEMETRIA FLOTTA ]
"""
        for robot in self.robots:
            v = self.velocities[robot]['v']
            w = self.velocities[robot]['w']
            role = self.tactical_roles[robot]
            
            dist_str = "N/D"
            if self.last_intruder_pose and self.robot_poses[robot]:
                rx, ry = self.robot_poses[robot]
                ix, iy = self.last_intruder_pose
                # Ora entrambe le coordinate sono nel frame 'map'
                dist = math.hypot(rx - ix, ry - iy)
                dist_str = f"{dist:.1f}m"
                
            role_str = f"{role:14}"
            if role == "INSEGUTORE":
                role_str = f"{C_RED}{role_str}{C_END}"
            elif role == "PORTA":
                role_str = f"{C_YELLOW}{role_str}{C_END}"

            dashboard += (
                f"  [{robot.upper()}] -> Ruolo: {role_str} | Dist. Ladro: {dist_str:5} | "
                f"v: {v: 5.2f} m/s  w: {w: 5.2f} rad/s\n"
            )

        dashboard += "\n--------------------------------------------------------------------------------\n[ ULTIMI EVENTI E LOG DI ALLERTA ]\n"
        
        for log in self.logs:
            if "INTRUSO" in log or "TACTICAL" in log:
                dashboard += f" * {C_RED}{log}{C_END}\n"
            else:
                dashboard += f" * {log}\n"
            
        dashboard += "================================================================================"
        
        print(dashboard)


def main(args=None):
    rclpy.init(args=args)
    node = MultiAgentDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        os.system('clear')
        print(f"\033[96mDashboard terminata correttamente.\033[0m")
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()