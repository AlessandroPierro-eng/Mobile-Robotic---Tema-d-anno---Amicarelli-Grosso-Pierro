#!/usr/bin/env python3
"""
Module for aggregating isolated Transform (TF) topics in a multi-robot ROS 2 environment.
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy


class TFMerger(Node):
    """
    Aggregates namespace-isolated TF topics into unified global topics.
    Ensures compatibility with RViz2 by enforcing appropriate QoS profiles.
    """

    def __init__(self) -> None:
        super().__init__('tf_merger')
        
        # Define QoS profile for static transformations (Transient Local durability)
        qos_static = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Initialize unified publishers
        self.tf_pub = self.create_publisher(TFMessage, '/rviz/tf', 10)
        self.tf_static_pub = self.create_publisher(TFMessage, '/rviz/tf_static', qos_static)
        
        # Initialize namespace-specific subscribers
        robot_namespaces = ['robot1', 'robot2', 'robot3']
        self.tf_subs = []
        self.tf_static_subs = []
        
        for ns in robot_namespaces:
            self.tf_subs.append(
                self.create_subscription(
                    TFMessage, 
                    f'/{ns}/tf', 
                    self.tf_callback, 
                    10
                )
            )
            self.tf_static_subs.append(
                self.create_subscription(
                    TFMessage, 
                    f'/{ns}/tf_static', 
                    self.tf_static_callback, 
                    qos_static
                )
            )

    def tf_callback(self, msg: TFMessage) -> None:
        """Propagates dynamic transformations to the global topic."""
        self.tf_pub.publish(msg)

    def tf_static_callback(self, msg: TFMessage) -> None:
        """Propagates static transformations using the required QoS policy."""
        self.tf_static_pub.publish(msg)


def main(args=None) -> None:
    """Node execution entry point."""
    rclpy.init(args=args)
    node = TFMerger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()