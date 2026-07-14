#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64

class CameraTracker(Node):
    def __init__(self):
        super().__init__('camera_tracker')
        
        # --- PARAMETRI DI CONTROLLO AGGIORNATI ---
        self.kp = 4.0          # Molto più reattivo (prima era 1.0)
        self.max_vel = 3.0     # Limite alzato per permettere scatti veloci
        self.deadband = 0.05   # Zona morta: ignora errori sotto i 5 cm
        
        self.subscription = self.create_subscription(
            PointStamped,
            'intruder_tracking',
            self.tracking_callback,
            10)
            
        self.joint_pub = self.create_publisher(
            Float64, 
            'bracket_vel', 
            10)
            
        self.get_logger().info("Camera Tracker 3D (Veloce) avviato. In attesa di target...")

    def tracking_callback(self, msg):
        cmd_msg = Float64()
        
        if msg.point.z > 0.0:
            error_x = msg.point.x 
            
            # --- APPLICAZIONE DELLA ZONA MORTA ---
            # Se il bersaglio è a meno di 5 cm dal centro, fermati (evita le vibrazioni)
            if abs(error_x) < self.deadband:
                angular_velocity = 0.0
            else:
                # Calcolo della velocità
                angular_velocity = -self.kp * error_x 
            
            # Saturazione
            if angular_velocity > self.max_vel:
                angular_velocity = self.max_vel
            elif angular_velocity < -self.max_vel:
                angular_velocity = -self.max_vel
                
            cmd_msg.data = float(angular_velocity)
            
        else:
            # Bersaglio perso, fermati
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