#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
import math

class CameraTracker(Node):
    def __init__(self):
        super().__init__('camera_tracker')
        
        # --- PARAMETRI DI CONTROLLO INSEGUIMENTO (Priorità 1) ---
        self.kp = 4.0          # Molto più reattivo (prima era 1.0)
        self.max_vel = 3.0     # Limite alzato per permettere scatti veloci
        self.deadband = 0.05   # Zona morta: ignora errori sotto i 5 cm
        
        # --- PARAMETRI STATO E COMPORTAMENTO (Priorità 2 & 3) ---
        self.robot_state = 'patrol'             # Stato di default
        self.camera_joint_angle = 0.0           # Memoria dell'angolo attuale
        self.camera_joint_name = 'oakd_camera_bracket_joint'   
        
        self.patrol_angle = 1.57                # 90 gradi in radianti (+ o - a seconda del lato)
        self.kp_patrol = 2.0                    # Reattività per tornare a 90 gradi
        self.sweep_speed = 1.5                  # Velocità massima durante la spazzata
        
        # --- SOTTOSCRIZIONI (Tutte relative al namespace) ---
        
        # 1. Topic dello stato del robot
        self.state_sub = self.create_subscription(
            String, 'state', self.state_callback, 10)
            
        # 2. Topic per leggere l'angolazione attuale del giunto
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)
            
        # 3. Topic di puntamento YOLO
        self.subscription = self.create_subscription(
            PointStamped, 'intruder_tracking', self.tracking_callback, 10)
            
        # --- PUBBLICAZIONE ---
        self.joint_pub = self.create_publisher(Float64, 'bracket_vel', 10)
            
        self.get_logger().info("Camera Tracker 3D Ibrido avviato. Stato iniziale: PATROL")

    def state_callback(self, msg):
        """Aggiorna la variabile interna dello stato del robot"""
        self.robot_state = msg.data.lower()

    def joint_callback(self, msg):
        """Legge l'angolo attuale del giunto della telecamera"""
        try:
            # Cerca il giunto della telecamera nell'array dei giunti del robot
            idx = msg.name.index(self.camera_joint_name)
            self.camera_joint_angle = msg.position[idx]
        except ValueError:
            pass # Il messaggio non conteneva il giunto della telecamera

    def tracking_callback(self, msg):
        """Il "cervello" della telecamera: esegue la priorità corretta ad ogni frame"""
        cmd_msg = Float64()
        
        # ==========================================
        # PRIORITÀ 1: LADRO IN VISTA (Riflesso Incondizionato)
        # ==========================================
        if msg.point.z > 0.0:
            error_x = msg.point.x 
            
            if abs(error_x) < self.deadband:
                angular_velocity = 0.0
            else:
                angular_velocity = -self.kp * error_x 
            
            # Saturazione
            if angular_velocity > self.max_vel:
                angular_velocity = self.max_vel
            elif angular_velocity < -self.max_vel:
                angular_velocity = -self.max_vel
                
            cmd_msg.data = float(angular_velocity)
            
        # ==========================================
        # PRIORITÀ 2 & 3: BERSAGLIO PERSO / NESSUN LADRO
        # ==========================================
        else:
            if self.robot_state == 'patrol':
                # --- Priorità 2: Tieni i 90 Gradi (P-Controller) ---
                error_angle = self.patrol_angle - self.camera_joint_angle
                vel = self.kp_patrol * error_angle
                
                # Saturazione più morbida per non scattare violentemente verso i 90°
                cmd_msg.data = float(max(-1.5, min(1.5, vel)))
                
            elif self.robot_state in ['pursuit', 'tactical', 'search']:
                # --- Priorità 3: Sweep di Ricerca Attiva ---
                # Usiamo il tempo assoluto per creare un'onda coseno
                # Questo genera un movimento dolce che va a destra e a sinistra ciclicamente
                t = self.get_clock().now().nanoseconds / 1e9
                
                # cos(t) oscilla tra -1 e 1. Moltiplicato per sweep_speed dà la velocità desiderata.
                cmd_msg.data = float(math.cos(t) * self.sweep_speed)
                
            else:
                # Fallback di sicurezza (es. stato sconosciuto)
                cmd_msg.data = 0.0
                
        self.joint_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

