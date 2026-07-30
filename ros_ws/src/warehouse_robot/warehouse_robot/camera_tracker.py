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
        
        # --- TRACKING CONTROL PARAMETERS (Priority 1) ---
        self.kp = 4.0          # Proportional gain for active tracking
        self.max_vel = 3.0     # Maximum angular velocity limit
        self.deadband = 0.05   # Deadband threshold to prevent control jitter
        
        # --- STATE AND BEHAVIOR PARAMETERS (Priority 2 & 3) ---
        self.robot_state = 'patrol'             # Default operational state
        self.camera_joint_angle = 0.0           # Current joint angle state memory
        self.camera_joint_name = 'oakd_camera_bracket_joint'   
        
        self.patrol_angle = 1.57                # Default patrol angle (90 degrees in radians)
        self.kp_patrol = 2.0                    # Proportional gain for patrol reset
        self.sweep_speed = 1.5                  # Maximum angular velocity during search sweep
        
        # --- ROS 2 SUBSCRIPTIONS ---
        
        # Robot operational state subscription
        self.state_sub = self.create_subscription(
            String, 'state', self.state_callback, 10)
            
        # Camera bracket joint state subscription
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)
            
        # YOLO target tracking subscription
        self.subscription = self.create_subscription(
            PointStamped, 'intruder_tracking', self.tracking_callback, 10)
            
        # --- ROS 2 PUBLISHERS ---
        self.joint_pub = self.create_publisher(Float64, 'bracket_vel', 10)
            
        self.get_logger().info("Hybrid 3D Camera Tracker initialized. Initial state: PATROL")

    def state_callback(self, msg):
        """Updates internal robot operational state."""
        self.robot_state = msg.data.lower()

    def joint_callback(self, msg):
        """Updates current camera bracket joint angle memory."""
        try:
            # Locate the camera joint within the joint state array
            idx = msg.name.index(self.camera_joint_name)
            self.camera_joint_angle = msg.position[idx]
        except ValueError:
            pass # Target joint not present in the current message

    def tracking_callback(self, msg):
        """Main control loop prioritizing tracking, patrol, and search behaviors."""
        cmd_msg = Float64()
        
        # ==========================================
        # PRIORITY 1: TARGET ACQUIRED (Active Tracking)
        # ==========================================
        if msg.point.z > 0.0:
            error_x = msg.point.x 
            
            if abs(error_x) < self.deadband:
                angular_velocity = 0.0
            else:
                angular_velocity = -self.kp * error_x 
            
            # Velocity saturation
            if angular_velocity > self.max_vel:
                angular_velocity = self.max_vel
            elif angular_velocity < -self.max_vel:
                angular_velocity = -self.max_vel
                
            cmd_msg.data = float(angular_velocity)
            
        # ==========================================
        # PRIORITY 2 & 3: TARGET LOST / DEFAULT BEHAVIOR
        # ==========================================
        else:
            if self.robot_state == 'patrol':
                # --- Priority 2: Maintain Patrol Angle (P-Controller) ---
                error_angle = self.patrol_angle - self.camera_joint_angle
                vel = self.kp_patrol * error_angle
                
                # Soft velocity saturation for smooth positional reset
                cmd_msg.data = float(max(-1.5, min(1.5, vel)))
                
            elif self.robot_state in ['pursuit', 'tactical', 'search']:
                # --- Priority 3: Active Search Sweep ---
                # Generate a cosine wave based on absolute time for continuous scanning
                t = self.get_clock().now().nanoseconds / 1e9
                
                cmd_msg.data = float(math.cos(t) * self.sweep_speed)
                
            else:
                # Safety fallback for undefined states
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