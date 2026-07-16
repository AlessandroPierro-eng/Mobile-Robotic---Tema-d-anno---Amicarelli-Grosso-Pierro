#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped  # Il nuovo pacchetto 3D!
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import math
import message_filters  # Magia per la sincronizzazione

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt') 
        
        # --- 1. SOTTOSCRIZIONI SINCRONIZZATE ---
        # Invece di leggere subito, mettiamo i messaggi in attesa...
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        # NOTA: Controlla che il topic depth su Gazebo sia questo (spesso è /camera/depth/image_raw)
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        
        # ...e li uniamo SOLO quando hanno lo stesso timestamp (tolleranza 0.1s)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)
            
        # Publisher aggiornato a PointStamped
        self.tracking_pub = self.create_publisher(
            PointStamped, 'intruder_tracking', 10)
            
        # Parametri ottici approssimati per trasformare i pixel in 3D 
        # (Su un robot reale li leggeremmo dal topic /camera_info)
        self.fx = 500.0  
        self.fy = 500.0  

        self.get_logger().info('YOLO 3D Detector avviato. In attesa di feed RGB-D...')

    def sync_callback(self, rgb_msg, depth_msg):
        # 1. Convertiamo entrambe le immagini per OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        
        # Le immagini di profondità su Gazebo sono matrici di numeri con la virgola (metri)
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        
        results = self.model(cv_image, verbose=False)
        
        # Prepariamo il pacchetto
        track_msg = PointStamped()
        track_msg.header.stamp = rgb_msg.header.stamp         # Copiamo il tempo esatto
        track_msg.header.frame_id = rgb_msg.header.frame_id   # Es: 'camera_link'
        
        person_found = False
        
        for box in results[0].boxes:
            if int(box.cls[0]) == 0 and float(box.conf[0]) >= 0.6:  # Classe Persona con confidenza >= 60%
                # Troviamo il pixel centrale del bersaglio (u, v)
                x1, y1, x2, y2 = box.xyxy[0]
                u = int((x1 + x2) / 2.0)
                v = int((y1 + y2) / 2.0)
                
                # 2. LETTURA PROFONDITÀ
                # Leggiamo il valore in metri esattamente a quel pixel
                z = float(depth_image[v, u])
                
                # Scartiamo letture non valide (sensore cieco o errore)
                if math.isnan(z) or z <= 0.0:
                    continue
                    
                # 3. CONVERSIONE PIXEL -> METRI (Pinhole Model)
                cx = cv_image.shape[1] / 2.0
                cy = cv_image.shape[0] / 2.0
                
                x_meters = (u - cx) * z / self.fx
                y_meters = (v - cy) * z / self.fy
                
                # Inseriamo i dati nel pacchetto
                track_msg.point.x = x_meters
                track_msg.point.y = y_meters
                track_msg.point.z = z
                
                person_found = True
                break  # Prendiamo la prima persona che troviamo
                
        # 4. IL TRUCCO DELLA Z
        if not person_found:
            track_msg.point.z = -1.0
            
        self.tracking_pub.publish(track_msg)
        
        # Disegno e Debug visivo
        annotated_frame = results[0].plot()
        cv2.imshow("Telecamera Robot - YOLOv8 3D", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()