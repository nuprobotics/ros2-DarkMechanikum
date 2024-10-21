#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReceiverNode(Node):
    def __init__(self):
        super().__init__('receiver')
        self.subscription = self.create_subscription(
            String,
            '/spgc/sender',
            self.listener_callback,
            10)
        self.get_logger().info('Receiver node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info('Received message: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = ReceiverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()