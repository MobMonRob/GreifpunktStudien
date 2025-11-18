#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from workflow_package.srv import MoveToPosition as MoveToPosition
class MoveToPositionServer(Node):
    def __init__(self):
        super().__init__('move_to_position_server')
        self.get_logger().info('Move To Position Server has been started.')
        self.srv = self.create_service(
            MoveToPosition, 'move_to_position', self.move_to_position_callback)

    def move_to_position_callback(self, request, response):
        self.get_logger().info('Received request to move to position.')
        # Implement the logic to move to position here
        if request.request_move_to_position == True:
            for i in range(5):
                print(f"Moving to position {request.request_position_name} at coordinates {request.request_target_position}...")  # Simulate moving process
            response.response_move_successful = True  # Example response
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = MoveToPositionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()