#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped  
from cv_bridge import CvBridge
from ultralytics import YOLO
import math
import message_filters  

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.bridge = CvBridge()
        
        # --- MODEL INITIALIZATION ---
        current_node_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(current_node_dir, 'yolov8n.pt')
        
        self.model = YOLO(model_path) 
        
        # --- SYNCHRONIZED SUBSCRIPTIONS ---
        self.rgb_sub = message_filters.Subscriber(self, Image, 'camera/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, 'camera/depth/image_raw')
        
        # Synchronize RGB and Depth messages based on timestamps (0.1s tolerance)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)
            
        # Target tracking publisher
        self.tracking_pub = self.create_publisher(
            PointStamped, 'intruder_tracking', 10)
            
        # Camera intrinsic parameters 
        self.fx = 224.15  
        self.fy = 224.15  

        self.get_logger().info('YOLO 3D Detector initialized in headless mode.')

    def sync_callback(self, rgb_msg, depth_msg):
        # Convert ROS Image messages to OpenCV formats
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        
        # --- GPU INFERENCE (Fail-Fast enabled, no display output) ---
        results = self.model.predict(source=cv_image, verbose=False, device=0)
        
        # Initialize tracking message
        track_msg = PointStamped()
        track_msg.header.stamp = rgb_msg.header.stamp 
        track_msg.header.frame_id = rgb_msg.header.frame_id   
        
        person_found = False
        
        for box in results[0].boxes:
            # Filter for 'Person' class (ID 0) with a confidence threshold >= 0.6
            if int(box.cls[0]) == 0 and float(box.conf[0]) >= 0.6:  
                x1, y1, x2, y2 = box.xyxy[0]
                u = int((x1 + x2) / 2.0)
                v = int((y1 + y2) / 2.0)
                
                # --- DEPTH EXTRACTION ---
                z = float(depth_image[v, u])
                
                if math.isnan(z) or z <= 0.0:
                    continue
                    
                # --- 3D COORDINATE PROJECTION (Pinhole Camera Model) ---
                cx = cv_image.shape[1] / 2.0
                cy = cv_image.shape[0] / 2.0
                
                x_meters = (u - cx) * z / self.fx
                y_meters = (v - cy) * z / self.fy
                
                track_msg.point.x = x_meters
                track_msg.point.y = y_meters
                track_msg.point.z = z
                
                person_found = True
                break  
                
        # --- LOST TARGET HANDLING ---
        if not person_found:
            track_msg.point.z = -1.0
            
        self.tracking_pub.publish(track_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()