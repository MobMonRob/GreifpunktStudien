#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from workflow_package.srv import CheckLabelPresence as CheckLabel

class CheckLabelPresenceServer(Node):
    def __init__(self):
        super().__init__('check_label_presence_server')
        self.srv = self.create_service(CheckLabel, 'check_label_presence', self.check_label_presence_callback)

    def check_label_presence_callback(self, request, response):
        # Implement your logic here
        response.label_present = True  # Example response
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CheckLabelPresenceServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()