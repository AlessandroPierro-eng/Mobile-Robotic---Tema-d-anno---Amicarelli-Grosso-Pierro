#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import tf2_geometry_msgs
import math

class MockYoloNode(Node):
    def __init__(self):
        super().__init__('mock_yolo_node')
        
        # --- Fleet configuration and publisher instantiation ---
        self.robots = ['robot1', 'robot2', 'robot3']
        self.tracking_pubs = {}
        for robot in self.robots:
            topic_name = f'/{robot}/intruder_tracking'
            self.tracking_pubs[robot] = self.create_publisher(PointStamped, topic_name, 10)

        # --- RViz Visualization Marker ---
        self.marker_pub = self.create_publisher(Marker, '/mock_intruder_marker', 10)

        # --- Spatial transformation infrastructure ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Global State Subscription (End-Game trigger) ---
        self.state_sub = self.create_subscription(String, '/robot1/state', self.state_callback, 10)
        self.is_apprehended = False

        # --- Hardware constraints simulation (FOV and depth limits) ---
        # Temporarily extended spatial threshold to facilitate validation
        self.max_distance = 17.0
        self.fov_rad = math.radians(70.0 / 2.0)

        # --- Predetermined continuous piecewise linear trajectory (waypoints) ---
        self.waypoints = [
            (7.0, -4.84), (6.17, -2.74), (3.36, -2.9), 
            (3.15, -6.45), (3.36, -2.9), (1.22, -2.39), 
            (1.4, -0.15), (1.22, -2.39), (-4.78, -2.12), 
            (-4.9, -5.74), (-4.78, -2.12), (-6.04, -1.15), 
            (-5.72, 1.75), (-11.9, -0.59), (-5.57, -1.94), 
            (6.26, -2.91), (7.73, -6.46)
        ]
        
        # --- Intruder kinematics and temporal state initialization ---
        self.current_wp_idx = 0
        self.intruder_x, self.intruder_y = self.waypoints[0]
        self.intruder_z = 1.0  
        
        # Drastically reduced constant linear velocity (m/s)
        self.speed = 0.05       
        
        # Idle state temporal parameters
        self.is_paused = False
        self.pause_duration = 1.0  # Required idle duration at waypoints (seconds)
        self.pause_end_time = None
        
        # --- System control loop execution rate (10 Hz) ---
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        self.get_logger().info('Target Simulator initialized. Waiting for visual contact...')

    def state_callback(self, msg: String):
        """Monitors global tactical state to halt simulation upon target capture."""
        if msg.data.lower() == 'objective_reached':
            if not self.is_apprehended:
                self.is_apprehended = True
                self.get_logger().warn("TARGET APPREHENDED: Freezing kinematics immediately.")

    def publish_rviz_marker(self):
        """Generates a visual representation of the synthetic target for RViz."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'mock_intruder'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        marker.pose.position.x = float(self.intruder_x)
        marker.pose.position.y = float(self.intruder_y)
        marker.pose.position.z = float(self.intruder_z)
        
        # Spatial dimensions 
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 1.8 
        
        # Visual properties
        marker.color.a = 1.0 
        marker.color.r = 1.0 
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        self.marker_pub.publish(marker)

    def timer_callback(self):
        """Iterative update of kinematics, temporal states, and perception boundaries."""
        current_time = self.get_clock().now()

        # 1. Update intruder state via FSM (Transit vs. Idle vs. Apprehended)
        if self.is_apprehended:
            # Halt all kinematic operations; target is captured
            pass
        elif self.is_paused:
            # Temporal evaluation for idle state expiration
            if current_time >= self.pause_end_time:
                self.is_paused = False
                self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
        else:
            # Constant velocity kinematic execution
            target_x, target_y = self.waypoints[self.current_wp_idx]
            dx = target_x - self.intruder_x
            dy = target_y - self.intruder_y
            distance_to_wp = math.hypot(dx, dy)
            step = self.speed * self.dt

            if distance_to_wp < step:
                # Target achieved: Enforce spatial alignment and initiate idle state
                self.intruder_x, self.intruder_y = target_x, target_y
                self.is_paused = True
                self.pause_end_time = current_time + Duration(seconds=self.pause_duration)
            else:
                self.intruder_x += (dx / distance_to_wp) * step
                self.intruder_y += (dy / distance_to_wp) * step

        # Define global coordinate vector mapping
        intruder_global_point = PointStamped()
        intruder_global_point.header.frame_id = 'map'
        intruder_global_point.header.stamp = self.get_clock().now().to_msg()
        intruder_global_point.point.x = float(self.intruder_x)
        intruder_global_point.point.y = float(self.intruder_y)
        intruder_global_point.point.z = float(self.intruder_z)

        # Update visualizer
        self.publish_rviz_marker()

        # 2. Iterate perception synthesis for each agent
        for robot in self.robots:
            camera_frame = f'{robot}/oakd_rgb_camera_optical_frame'
            
            track_msg = PointStamped()
            track_msg.header.stamp = self.get_clock().now().to_msg()
            track_msg.header.frame_id = camera_frame
            track_msg.point.x = 0.0
            track_msg.point.y = 0.0
            track_msg.point.z = -1.0 # Default lost state
            
            try:
                transform = self.tf_buffer.lookup_transform(
                    camera_frame, 
                    'map', 
                    rclpy.time.Time()
                )
                
                intruder_cam_point = tf2_geometry_msgs.do_transform_point(intruder_global_point, transform)
                
                cx = intruder_cam_point.point.x
                cz = intruder_cam_point.point.z
                
                # Validate spatial bounds and field of view constraints
                if 0.0 < cz <= self.max_distance:
                    angle_rad = math.atan2(abs(cx), cz)
                    if angle_rad <= self.fov_rad:
                        track_msg.point = intruder_cam_point.point
                        self.get_logger().info(f'[{robot}] VISUAL CONTACT CONFIRMED! Distance: {cz:.2f}m', throttle_duration_sec=2.0)
                        
            except Exception:
                pass
                
            self.tracking_pubs[robot].publish(track_msg)

def main(args=None):
    # --- Internal Topic Remapping ---
    # Enforces subscription to custom aggregated TF topics seamlessly
    custom_args = [
        '--ros-args', 
        '-r', '/tf:=/rviz/tf', 
        '-r', '/tf_static:=/rviz/tf_static'
    ]
    
    if args is None:
        args = custom_args
    else:
        args.extend(custom_args)
        
    rclpy.init(args=args)
    node = MockYoloNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()