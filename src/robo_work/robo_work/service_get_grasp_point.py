## Service um den Greifpunkt zu bestimmen --> über Abfrage von anderen Laptop ##
# srv import
from robo_work_msg.srv import GetGraspPoint
# Python Imports
import requests
# übliche imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# rebel bewegung import
from robo_work.rebel_mover import RebelMover

#------------- service klasse ------------------------------
class GetGraspPointService(Node):
    def __init__(self, mover):
        super().__init__('get_grasp_point_service')
        self.mover = mover
        self.cb_group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            GetGraspPoint,
            'get_grasp_point',
            self.get_grasp_point_callback,
            callback_group=self.cb_group
        )

    def get_grasp_point_callback(self, request, response):
        if request.start_detect_grasp_point == True:
            self.get_logger().info('Starte Greifpunkt Bestimmung.')
            # Fahre zu Start Position:
            dummyPosi = [82, 13, 82, 50, 83, -90]
            self.mover.move_to_dummy_position(dummyPosi)
            # Sende Signal an HappyPose Laptop
            # startDetectGraspPoint = requests.get('http://IPXX/DetectGraspPoint')
            # graspPoint = startDetectGraspPoint.json()
            graspPointDummy = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
            response.grasp_point_detected = True
            response.grasp_points = graspPointDummy
        return response

def main():
    rclpy.init()
    mover = RebelMover()
    get_grasp_point_service = GetGraspPointService(mover)

    executor = MultiThreadedExecutor()
    executor.add_node(mover)
    executor.add_node(get_grasp_point_service)

    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()