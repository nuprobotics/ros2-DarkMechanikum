#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import traceback

class TriggerNode(Node):
    def __init__(self):
        super().__init__('trigger_node')

        # Declare parameters and their default values
        self.declare_parameter('service_name', '/trigger_service')
        self.declare_parameter('default_string', 'No service available')

        # Get parameter values
        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self.default_string = self.get_parameter('default_string').get_parameter_value().string_value

        # Stored string value (starts with the default string)
        self.stored_string = self.default_string

        # Create a service client to call the /spgc/trigger service
        self.client = self.create_client(Trigger, '/spgc/trigger')

        # Create the service that responds with the stored string
        self.service = self.create_service(Trigger, self.service_name, self.handle_service_request)

        # Set up a timer to call the /spgc/trigger service every second
        self.timer = self.create_timer(1.0, self.call_trigger_service)

    def call_trigger_service(self):
        if self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Calling /spgc/trigger service...')
            request = Trigger.Request()

            # Send request to /spgc/trigger service
            future = self.client.call_async(request)
            future.add_done_callback(self.handle_trigger_response)
        else:
            self.get_logger().warning('/spgc/trigger service is unavailable. Using default string.')

    def handle_trigger_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.stored_string = response.message
                self.get_logger().info(f'Received response from /spgc/trigger: {response.message}')
            else:
                self.get_logger().warning('Service call to /spgc/trigger failed. Using last valid or default value.')
        except Exception as e:
            self.get_logger().error(f'Error during service call: {traceback.format_exc()}. Using last valid or default value.')

    def handle_service_request(self, request, response):
        # Respond with the stored string value (last received or default)
        response.success = True
        response.message = self.stored_string
        self.get_logger().info(f'Responding to service call with: {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TriggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
