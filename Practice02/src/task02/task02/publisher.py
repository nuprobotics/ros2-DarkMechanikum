#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher')

        # Declare and get parameters
        self.declare_parameter('text', 'Hello, ROS2!')
        self.declare_parameter('topic_name', '/spgc/receiver')
        self.declare_parameter('publish_frequency', 1.0)  # Default to 1 Hz

        # Get parameter values
        self.text = self.get_parameter('text').get_parameter_value().string_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value

        # Set up publisher
        self.publisher_ = self.create_publisher(String, self.topic_name, 10)

        # Create a timer that calls the publish_message method periodically
        timer_period = 1.0 / self.publish_frequency  # Timer period in seconds
        self.timer = self.create_timer(timer_period, self.publish_message)

        self.get_logger().info(f'Starting publisher node with topic: "{self.topic_name}" and frequency: {self.publish_frequency} Hz')

    def publish_message(self):
        # Publish the message at each timer call
        msg = String()
        msg.data = self.text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published message: "{msg.data}"')

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
