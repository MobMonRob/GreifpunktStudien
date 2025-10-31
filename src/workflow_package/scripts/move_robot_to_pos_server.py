#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from workflow_package.srv import MoveTheRobotTiPos as MoveRobot

from builtin_interfaces.msg import Duration
import time

class MoveRobotService(Node):
    def __init__(self):
        super().__init__('move_robot_to_pos_server')
        self.srv = self.create_service(MoveRobot, 'move_robot_to_pos', self.move_robot_callback)
        self.get_logger().info('Move Robot To Position Service is ready.')

    def move_robot_callback(self, request, response):
        self.get_logger().info(f'Received request to move robot to position: {request.target_position}')
        

        response.move_successful = True
        self.get_logger().info('Robot moved successfully.')
        return response
    

def main(args=None):
    rclpy.init(args=args)
    move_robot_service = MoveRobotService()
    rclpy.spin(move_robot_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
