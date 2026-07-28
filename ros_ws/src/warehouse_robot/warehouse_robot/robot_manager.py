#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math

# Messaggi per Nav2 e ROS
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from std_msgs.msg import String

# Librerie per le Trasformate (TF2)
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  


class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        
        self.get_logger().info("Inizializzazione Robot Manager in corso...")

        # ==========================================
        # 0. PARAMETRI ROS 2
        # ==========================================
        self.declare_parameter('start_wp_index', 0)
        self.current_wp_index = self.get_parameter('start_wp_index').get_parameter_value().integer_value

        # ==========================================
        # 1. MEMORIA DELLA MACCHINA A STATI (FSM)
        # ==========================================
        self.state = 'patrol'  # Stati: patrol, pursuit, tactical, search
        
        self.yolo_sees_intruder = False
        self.received_tactical_order = False
        self.received_search_cmd = False
        self.received_patrol_cmd = False  # <--- NUOVA FLAG PER IL GM
        
        self.last_intruder_pose = None
        self.tactical_target = None
        
        # Waypoints della ronda
        self.waypoints = [[7.7, 4.0], [-19.5, 6.1], [-20.5, -6.5], [6.8, -8.3]]
        self.current_wp_index = self.current_wp_index % len(self.waypoints)

        # ==========================================
        # 2. SISTEMA DI TRASFORMATE E NAVIGAZIONE
        # ==========================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_goal_handle = None    
        self.is_navigating = False      

        # ==========================================
        # 3. INTERFACCE DI COMUNICAZIONE
        # ==========================================
        self.yolo_sub = self.create_subscription(
            PointStamped, 'intruder_tracking', self.yolo_callback, 10)
            
        self.tactical_order_sub = self.create_subscription(
            PointStamped, 'tactical_order', self.tactical_order_callback, 10)
            
        self.state_sub = self.create_subscription(
            String, 'state', self.state_callback, 10)

        self.intruder_pos_pub = self.create_publisher(
            PointStamped, 'global_intruder_position', 10)    

        self.state_pub = self.create_publisher(String, 'state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # ==========================================
        # 4. IL BATTITO CARDIACO 
        # ==========================================
        timer_period = 0.2  # 5 Hz
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info(f"Robot Manager PRONTO. Stato: {self.state.upper()} | Start WP: {self.current_wp_index}")

    # -------------------------------------------------------------------------
    # CALLBACKS SENSORIALI
    # -------------------------------------------------------------------------
    def yolo_callback(self, msg):
        if msg.point.z > 0.0:
            self.yolo_sees_intruder = True
            global_coords = self.get_global_pose(msg)
            
            if global_coords:
                self.last_intruder_pose = global_coords
                
                pos_msg = PointStamped()
                pos_msg.header.stamp = self.get_clock().now().to_msg()
                pos_msg.header.frame_id = 'map' # È CRUCIALE CHE SIA 'map' E NON IL NOME DEL ROBOT!
                pos_msg.point.x = global_coords[0]
                pos_msg.point.y = global_coords[1]
                pos_msg.point.z = 0.0
                
                # LA STAMPA FONDAMENTALE MANCANTE:
                self.get_logger().warn(f"!!! [YOLO] AVVISTAMENTO! Invio allarme al Global Manager: X={global_coords[0]:.2f}, Y={global_coords[1]:.2f} !!!")
                
                self.intruder_pos_pub.publish(pos_msg)
            else:
                self.get_logger().error("[TF2] Errore critico: Yolo vede il bersaglio ma non riesco a tradurre le coordinate sulla mappa!")
        else:
            self.yolo_sees_intruder = False

    def tactical_order_callback(self, msg):
        self.tactical_target = [msg.point.x, msg.point.y]
        self.received_tactical_order = True
        self.get_logger().warn(f"[TATTICA] Ricevuto ordine dal Global Manager -> X={msg.point.x:.2f}, Y={msg.point.y:.2f}")


    def state_callback(self, msg):
        cmd = msg.data.lower()
        if cmd == 'search' and self.state != 'search':
            self.received_search_cmd = True
        elif cmd == 'patrol' and self.state != 'patrol':
            self.received_patrol_cmd = True

    def get_global_pose(self, local_point_msg):
        try:
            # Attendiamo che la trasformata sia disponibile. Aumentato il timeout a 0.2s per Docker
            transform = self.tf_buffer.lookup_transform(
                'map', local_point_msg.header.frame_id,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)
            )
            global_point_msg = tf2_geometry_msgs.do_transform_point(local_point_msg, transform)
            return [global_point_msg.point.x, global_point_msg.point.y]
        except TransformException as e:
            self.get_logger().error(f"[TF2 EXCEPTION] Impossibile trasformare {local_point_msg.header.frame_id} in 'map': {e}")
            return None

    # -------------------------------------------------------------------------
    # IL CERVELLO (Control Loop)
    # -------------------------------------------------------------------------
    def control_loop(self):
        current_time = time.time()
        next_state = self.state
        
        # --- A. VALUTAZIONE DELLE TRANSIZIONI (FSM) ---
        if self.state == 'patrol':
            if self.received_tactical_order:
                next_state = 'tactical'
            elif self.yolo_sees_intruder:
                next_state = 'pursuit'
                
        elif self.state == 'pursuit':
            if self.received_tactical_order:
                next_state = 'tactical'
            elif self.received_search_cmd:
                next_state = 'search'
                
        elif self.state == 'tactical':
            if self.received_search_cmd:
                next_state = 'search'
                
        elif self.state == 'search':
            if self.received_tactical_order:
                next_state = 'tactical'
            elif self.yolo_sees_intruder:
                next_state = 'pursuit'
            elif self.received_patrol_cmd:
                self.get_logger().info("Ordine di fine ricerca dal GM. Torno in pattuglia.")
                next_state = 'patrol'

        # Reset incondizionato delle flag
        self.received_tactical_order = False
        self.received_search_cmd = False
        self.received_patrol_cmd = False

        # Applicazione nuovo stato e Freno a Mano
        if next_state != self.state:
            self.get_logger().warn(f"TRANSIZIONE: {self.state.upper()} -> {next_state.upper()}")
            self.cancel_nav_goal()
            self.state = next_state
            
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        # --- B. ESECUZIONE DELLO STATO ATTUALE ---
        if self.state == 'patrol':
            if not self.is_navigating:
                wp = self.waypoints[self.current_wp_index]
                # Inviamo la meta. NON INCREMENTIAMO L'INDICE QUI!
                success = self.send_nav_goal(wp[0], wp[1])
                if success:
                    self.get_logger().info(f"Ronda: Invio ordine al WP {self.current_wp_index} -> {wp}")
                
        elif self.state == 'pursuit':
            if self.last_intruder_pose:
                tx, ty = self.last_intruder_pose
                if not hasattr(self, 'current_chase_target'):
                    self.current_chase_target = [0.0, 0.0]
                
                dist = math.hypot(tx - self.current_chase_target[0], ty - self.current_chase_target[1])
                
                if not self.is_navigating or dist > 0.5:
                    self.current_chase_target = [tx, ty]
                    self.send_nav_goal(tx, ty)
                    
        elif self.state == 'tactical':
            if self.tactical_target:
                tx, ty = self.tactical_target
                if not hasattr(self, 'current_tactical_target'):
                    self.current_tactical_target = [0.0, 0.0]
                
                dist = math.hypot(tx - self.current_tactical_target[0], ty - self.current_tactical_target[1])
                
                if not self.is_navigating or dist > 0.5:
                    self.current_tactical_target = [tx, ty]
                    self.send_nav_goal(tx, ty)
                    
        elif self.state == 'search':
            if not self.is_navigating:
                spin_msg = Twist()
                spin_msg.angular.z = 0.5 
                self.cmd_vel_pub.publish(spin_msg)

    # -------------------------------------------------------------------------
    # I MUSCOLI (Gestione asincrona Nav2)
    # -------------------------------------------------------------------------
    def send_nav_goal(self, x, y):
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0 

        self.is_navigating = True
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def cancel_nav_goal(self):
        """Tira il freno a mano e cancella il viaggio in corso."""
        if self.nav_goal_handle is not None and self.is_navigating:
            self.get_logger().info("Cancellazione del viaggio Nav2 in corso...")
            self.nav_goal_handle.cancel_goal_async()
            self.is_navigating = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 ha RIFIUTATO il target! (Riprovo...)")
            self.is_navigating = False
            return
            
        self.nav_goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        self.is_navigating = False

        # INCREMENTIAMO IL WAYPOINT SOLO SE SPOSTAMENTO COMPLETATO CON SUCCESSO
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.state == 'patrol':
                self.get_logger().info(f"WP {self.current_wp_index} RAGGIUNTO! Passo al prossimo.")
                self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
        else:
            self.get_logger().warn(f"Nav2 non è riuscito a raggiungere il WP {self.current_wp_index}. Stato: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()