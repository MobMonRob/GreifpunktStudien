## Service um den Greifpunkt zu bestimmen --> über Abfrage von anderen Laptop ##

# srv import
from robo_work_msg.srv import getGraspPoint

# Python Imports
import requests

# übliche imports
import rclpy 
from rclpy.node import Node

# service klasse
class GetGraspPointService(Node):
    def __init__(self):
        super().__init__('get_grasp_point_service')
        self.srv = self.create_service(getGraspPoint, 'get_grasp_point', self.get_grasp_point_callback)

    def get_grasp_point_callback(self, request, response):
        if request.start_detect_grasp_point == True:
            self.get_logger().info('Starte Greifpunkt Bestimmung.')
            # Fahre zu Start Position:

            # Sende Signal an HappyPose Laptop
            #startDetectGraspPoint = requests.get('http://IPXX/DetectGraspPoint')
            #graspPoint = startDetectGraspPoint.json() # antwort noch korrekt formatieren
            graspPointDummy = [1,2,3,4,5,6]

            # Antwort zurückgeben
            graspPointsTest = graspPointDummy
            response.grasp_point_detected = True # detection was sucess
            response.grasp_points = graspPointsTest # greifpunkt speichern

            return response  # beide antworten zurückgeben

def main():
    rclpy.init()
    get_grasp_point_service = GetGraspPointService()
    rclpy.spin(get_grasp_point_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()