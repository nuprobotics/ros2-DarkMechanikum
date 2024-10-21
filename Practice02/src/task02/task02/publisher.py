#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher')

        # Declare and get parameters
        self.declare_parameter('text', 'Hello, ROS2!')
        self.declare_parameter('topic_name', '/spgc/receiver')

        # Get parameter values
        text = self.get_parameter('text').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        # Set up publisher
        self.publisher_ = self.create_publisher(String, topic_name, 10)

        # Log that the node has started
        self.get_logger().info(f'Publishing message: "{text}" to topic: "{topic_name}"')

        # Publish the message
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    node = PublisherNode()

    try:
        # Spin to keep the node alive and publishing
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
