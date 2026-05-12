## Service um zu checken ob das Label der Flasche passt --> über Abfrage von anderen Laptop ##

# srv import
from robo_work_msg.srv import isLabelCorrect

# Python imports
import requests

# übliche imports
import rclpy
from rclpy.node import Node

# rebel bewegung import
from robo_work.rebel_mover import RebelMover


#-------------------- Service Node ---------------------------------------
# service klasse
class CheckLabelService(Node):
    def __init__(self):
        super().__init__('check_label_service')
        self.srv = self.create_service(isLabelCorrect, 'check_label', self.check_label_callback)

    def check_label_callback(self, request, response):
        self.get_logger().info('Starte Label Check.')
        
        # Fahre zur CheckPosition
        if request == True:
            print('Fahre blabla')

        # Sende Signal an Label Check Laptop --> schauen wegen der Bewegung, ggf. erstmal statisch??
        #startCheckLabel = requests.get('http://IPXX/DetectLabel')
        #labelClassification = startDetectGraspPoint.json() # antwort noch korrekt formatieren
        isLabelCorrectDummy = True
        startSortingBottleDummy = True

        response.label_classification = isLabelCorrect
        response.start_sorting_bottle = startSortingBottleDummy

        return response

def main():
    rclpy.init()
    mover = RebelMover()
    check_label_service = CheckLabelService(mover)

    rclpy.spin(check_label_service)
    rclpy.shutdown

if __name__ == '__main__':
    main()

        
