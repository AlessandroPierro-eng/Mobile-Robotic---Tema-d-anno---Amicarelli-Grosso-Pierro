#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class MockYolo(Node):
    def __init__(self):
        super().__init__('mock_yolo_intruder')
        
        self.pub = self.create_publisher(PointStamped, '/robot1/intruder_tracking', 10)
        
        self.current_z = 7.0
        
        self.timer = self.create_timer(0.5, self.publish_mock_data)
        self.get_logger().info("Mock YOLO started. Intruder is at 7.0m and approaching...")

    def publish_mock_data(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'robot1/oakd_rgb_camera_frame'
        
        msg.point.x = 0.5
        msg.point.y = 0.0
        msg.point.z = self.current_z

        self.pub.publish(msg)
        self.get_logger().info(f"Target broadcasted at Z = {self.current_z:.2f}m")

        if self.current_z > 0.2:
            self.current_z -= 0.2

def main(args=None):
    rclpy.init(args=args)
    node = MockYolo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()