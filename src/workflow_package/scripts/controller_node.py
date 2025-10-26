#!/usr/bin/env python3
from platform import node
import rclpy
from rclpy.node import Node

from workflow_package.srv import CheckPartPresence as CheckPart

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info('Controller Node has been started.')

        #create clients for services here
        self.cli = self.create_client(CheckPart, 'check_part')

        #wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')


    def send_request_part_presence(self,request_start: bool):
        req = CheckPart.Request()
        req.request_scanning_for_parts = request_start
        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    request_Flag = True
    result = node.send_request_part_presence(request_Flag)
    if result.response_parts_present:
        node.get_logger().info('Parts are present.')
    else:
        node.get_logger().info('No parts found.')

    #rclpy.spin(controller_node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
