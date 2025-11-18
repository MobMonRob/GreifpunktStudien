#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from workflow_package.srv import GetGraspPoints as GetGraspPoints

class GetGraspPointServer(Node):
    def __init__(self):
        super().__init__('get_grasp_point_server')
        self.get_logger().info('Get Grasp Point Server has been started.')
        self.srv = self.create_service(
            GetGraspPoints, 'get_grasp_points', self.get_grasp_points_callback)

    def get_grasp_points_callback(self, request, response):
        self.get_logger().info('Received request to get grasp points.')
        # Implement the logic to get grasp points here
        if request.request_grasp_points == True:
            for i in range(5):
                print("Calculating grasp points...")  # Simulate calculation process
            response.response_grasp_point_found = True  # Example response
            response.response_grasp_points = [0.1, 0.2, 0.3]  # Example response
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = GetGraspPointServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()