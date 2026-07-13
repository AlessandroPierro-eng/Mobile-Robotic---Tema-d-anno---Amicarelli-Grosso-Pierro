#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point  # Importiamo il tipo di messaggio per le coordinate
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Sottoscrizione al video della telecamera
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.listener_callback, 10)
            
        # Publisher: Invia i dati al nodo camera_tracker
        self.tracking_pub = self.create_publisher(
            Point, 'intruder_tracking', 10)
            
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt') 
        self.get_logger().info('YOLO Detector avviato. In attesa di immagini...')

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image, verbose=False)
        
        person_found = False
        error_x = 0.0
        
        # Analizziamo i risultati per trovare una "persona" (classe 0 in COCO dataset)
        for box in results[0].boxes:
            if int(box.cls[0]) == 0:  # 0 = Person
                # Calcola il centro del rettangolo che inquadra la persona
                x1, y1, x2, y2 = box.xyxy[0]
                center_x = (x1 + x2) / 2.0
                
                # Calcola l'offset rispetto al centro dell'immagine
                img_width = cv_image.shape[1]
                error_x = center_x - (img_width / 2.0)
                
                person_found = True
                break  # Trovata una persona, ci fermiamo
                
        # Prepariamo il messaggio per il tracker
        track_msg = Point()
        if person_found:
            track_msg.x = float(error_x)  # Quanti pixel di errore
            track_msg.z = 1.0             # 1.0 significa "Bersaglio agganciato"
        else:
            track_msg.z = 0.0             # 0.0 significa "Bersaglio perso"
            
        # Spariamo il messaggio sul topic
        self.tracking_pub.publish(track_msg)
        
        # Disegna i riquadri e le etichette sul frame
        annotated_frame = results[0].plot()
        
        # Mostra il video a schermo (utile per il debug)
        cv2.imshow("Telecamera Robot - YOLOv8", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()
    try:
        rclpy.spin(yolo_detector)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_detector.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()