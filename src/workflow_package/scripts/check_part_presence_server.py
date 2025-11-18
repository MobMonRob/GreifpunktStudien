#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from workflow_package.srv import CheckPartPresence as CheckPart

class CheckPartServer(Node):
    def __init__(self):
        super().__init__('check_part_server')
        self.get_logger().info('Check Part Server has been started.')
        self.srv = self.create_service(
            CheckPart, 'check_part', self.check_part_callback)

    def check_part_callback(self, request, response):
        self.get_logger().info('Received request to check part.')
        # Implement the logic to check the part here
        if request.request_scanning_for_parts == True:
            for i in range(5):
                print("Checking for part...")  # Simulate checking process
                time.sleep(1)
            response.response_parts_present = True  # Example response
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CheckPartServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()