#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from functools import partial

# Import per la matematica e l'Algoritmo Ungherese
import numpy as np
from scipy.optimize import linear_sum_assignment
import math

class GlobalManager(Node):
    def __init__(self):
        super().__init__('global_manager')
        self.get_logger().warn("===================================================")
        self.get_logger().warn("   [START] Avvio Global Fleet Manager...           ")
        self.get_logger().warn("===================================================")

        # ==========================================
        # 1. PARAMETRI E STATO GLOBALE
        # ==========================================
        self.robots = ['robot1', 'robot2', 'robot3']
        
        self.state = 'patrol'  # patrol, tactical, search, objective_reached
        
        # Memoria della flotta
        self.robot_poses = {robot: None for robot in self.robots}
        self.all_robots_ready = False # Flag per stampare a schermo quando tutti i robot sono connessi
        
        # Memoria del target e Coordinate Tattiche (Nav2)
        self.intruder_seen = False
        self.last_intruder_pose = None
        self.last_intruder_time = 0.0
        self.search_start_time = 0.0
        
        # Punti di blocco strategici (Coordinate Nav2 logiche)
        self.door = [7.97, -6.22]
        self.stairs = [6.9, 1.89]

        # ==========================================
        # 2. CREAZIONE DINAMICA INTERFACCE ROS 2
        # ==========================================
        self.tactical_pubs = {}
        self.state_pubs = {}
        self.odom_subs = []
        self.intruder_subs = []

        for robot in self.robots:
            # Publisher
            self.tactical_pubs[robot] = self.create_publisher(
                PointStamped, f'/{robot}/tactical_order', 10)
            
            self.state_pubs[robot] = self.create_publisher(
                String, f'/{robot}/state', 10)

            # Subscriber
            self.odom_subs.append(self.create_subscription(
                Odometry, f'/{robot}/odom', partial(self.odom_callback, robot_id=robot), 10))
            
            # NOTA: Ho aggiunto il 'partial' anche qui per sapere CHI invia l'allarme!
            self.intruder_subs.append(self.create_subscription(
                PointStamped, f'/{robot}/global_intruder_position', partial(self.intruder_callback, robot_id=robot), 10))

        # ==========================================
        # 3. IL CICLO DECISIONALE (3 Secondi)
        # ==========================================
        self.timer = self.create_timer(3.0, self.global_control_loop)
        self.get_logger().info("[INIT] Global Manager PRONTO. In attesa dei robot...")

    # -------------------------------------------------------------------------
    # CALLBACKS
    # -------------------------------------------------------------------------
    def odom_callback(self, msg, robot_id):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_poses[robot_id] = [x, y]
        
        # Stampa una tantum quando tutti i robot sono online
        if not self.all_robots_ready and all(pose is not None for pose in self.robot_poses.values()):
            self.all_robots_ready = True
            self.get_logger().warn("[ODOM] Tutti e 3 i robot sono online! La flotta è operativa.")

    def intruder_callback(self, msg, robot_id):
        self.intruder_seen = True
        self.last_intruder_pose = [msg.point.x, msg.point.y]
        self.last_intruder_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().error(f"[ALLARME] >>> RICEVUTO DA {robot_id.upper()} <<<")
        self.get_logger().error(f"[ALLARME] Posizione ladro globale stimata: X={msg.point.x:.2f}, Y={msg.point.y:.2f}")

    # -------------------------------------------------------------------------
    # MACCHINA A STATI GLOBALE
    # -------------------------------------------------------------------------
    def global_control_loop(self):
        if any(pose is None for pose in self.robot_poses.values()):
            self.get_logger().info("[ATTESA] Aspetto l'odometria di tutti i robot per iniziare a calcolare...", throttle_duration_sec=6.0)
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        next_state = self.state

        self.get_logger().info(f"[HEARTBEAT] Stato Attuale: {self.state.upper()} | Ladro a vista: {self.intruder_seen}")

        # --- A. VALUTAZIONE TRANSIZIONI ---
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
                self.get_logger().warn("[TIMEOUT] Ladro perso di vista da oltre 10s. Passo in SEARCH.")
                
        elif self.state == 'search':
            if self.intruder_seen:
                next_state = 'tactical'
            elif (current_time - self.search_start_time) > 15.0:
                next_state = 'patrol'
                self.get_logger().warn("[TIMEOUT] Ricerca fallita da oltre 15s. Torno in PATROL.")

        # --- B. APPLICAZIONE TRANSIZIONI ---
        if next_state != self.state:
            self.get_logger().error(f"===================================================")
            self.get_logger().error(f"  [TRANSIZIONE GM] {self.state.upper()} -> {next_state.upper()}  ")
            self.get_logger().error(f"===================================================")
            self.state = next_state
            
            if self.state == 'search':
                self.broadcast_state('search')
            elif self.state == 'patrol':
                self.broadcast_state('patrol')

        # --- C. AZIONI CONTINUE ---
        if self.state == 'tactical':
            self.calculate_and_send_tactical_positions()
            
        elif self.state == 'objective_reached':
            self.get_logger().info("[VITTORIA] LADRO CATTURATO! Simulazione completata.", throttle_duration_sec=5.0)

    # -------------------------------------------------------------------------
    # FUNZIONI OPERATIVE E MATEMATICA
    # -------------------------------------------------------------------------
    def broadcast_state(self, state_str):
        msg = String()
        msg.data = state_str
        for robot in self.robots:
            self.state_pubs[robot].publish(msg)
        self.get_logger().info(f"[BROADCAST] Inviato ordine di stato '{state_str.upper()}' a tutta la flotta.")

    def calculate_and_send_tactical_positions(self):
        """
        Calcola l'assegnazione ottimale tra robot e obiettivi strategici.
        """
        if not self.last_intruder_pose:
            return

        self.get_logger().info("--- AVVIO CALCOLO ASSEGNAZIONI (ALGORITMO UNGHERESE) ---")

        # I 3 obiettivi da coprire
        targets = [
            self.last_intruder_pose,
            self.door,
            self.stairs
        ]
        
        target_names = ["Ladro", "Porta", "Scale"]

        cost_matrix = np.zeros((len(self.robots), len(targets)))
        
        # Riempimento matrice costi (Distanza di Manhattan)
        for i, robot in enumerate(self.robots):
            rx, ry = self.robot_poses[robot]
            for j, target in enumerate(targets):
                tx, ty = target
                cost = abs(rx - tx) + abs(ry - ty)
                cost_matrix[i, j] = cost
                # Stampa opzionale della matrice dei costi (commentata per non intasare, decommenta se serve)
                # self.get_logger().info(f"Costo {robot} -> {target_names[j]}: {cost:.2f}m")

        # Risoluzione con Algoritmo Ungherese
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # Invio degli ordini ottimali ai robot
        for idx in range(len(self.robots)):
            robot = self.robots[row_ind[idx]]
            target_idx = col_ind[idx]
            assigned_target = targets[target_idx]
            target_name = target_names[target_idx]
            cost_dist = cost_matrix[row_ind[idx], col_ind[idx]]

            self.get_logger().warn(f"[ORDINE] {robot.upper()} assegnato a: {target_name.upper()} (X:{assigned_target[0]:.2f}, Y:{assigned_target[1]:.2f}) | Dist stima: {cost_dist:.2f}m")

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.point.x = float(assigned_target[0])
            msg.point.y = float(assigned_target[1])
            msg.point.z = 0.0
            
            # Pubblicazione sul topic tactical_order
            self.tactical_pubs[robot].publish(msg)
            
        self.get_logger().info("--------------------------------------------------------")

    def check_objective_reached(self):
        """
        Controlla se ALMENO UN robot della flotta è a meno di 1.0 metri dal ladro.
        """
        if not self.last_intruder_pose:
            return False
            
        tx, ty = self.last_intruder_pose
        
        for robot, pose in self.robot_poses.items():
            if pose is None:
                continue
                
            rx, ry = pose
            
            # Usiamo la distanza Euclidea per il check finale di "cattura"
            dist = math.hypot(rx - tx, ry - ty)
            
            # Logghiamo le distanze in tempo reale durante il pursuit
            self.get_logger().info(f"[DISTANZE] {robot.upper()} dista {dist:.2f}m dal bersaglio.")
            
            if dist < 1.0:  
                self.get_logger().error(f"!!! CATTURA EFFETTUATA DA {robot.upper()} !!! (Distanza: {dist:.2f}m)")
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