## Service um zu checken ob das Label der Flasche passt --> über Abfrage von anderen Laptop ##
# srv import
from robo_work_msg.srv import IsLabelCorrect
# Python imports
import requests
import time
# übliche imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# rebel bewegung import
from robo_work.rebel_mover import RebelMover

#-------------------- Service Node ---------------------------------------
class CheckLabelService(Node):
    def __init__(self, mover):
        super().__init__('check_label_service')
        self.mover = mover
        self.cb_group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            IsLabelCorrect,
            'check_label',
            self.check_label_callback,
            callback_group=self.cb_group
        )

    def check_label_callback(self, request, response):
        self.get_logger().info('Starte Label Check.')
        # Fahre zur CheckPosition
        dummyPosi = [50, 15, 85, 55, 85, -90]
        self.mover.move_to_dummy_position(dummyPosi)
        #self.mover.move_to_home()
        # Sende Signal an Label Check Laptop
        # startCheckLabel = requests.get('http://IPXX/DetectLabel')
        # labelClassification = startCheckLabel.json()

        # Fahre zur Ablage Pos dummy
        #dummyPosiAbl = [50, 15, 85, 55, 85, -90]
        #self.mover.move_to_dummy_position(dummyPosiAbl)
        time.sleep(3)
        # fahre zu Zero
        self.mover.move_to_home()

        isLabelCorrectDummy = True
        startSortingBottleDummy = True
        response.label_classification = isLabelCorrectDummy
        response.start_sorting_bottle = startSortingBottleDummy
        return response

def main():
    rclpy.init()
    mover = RebelMover()
    check_label_service = CheckLabelService(mover)

    executor = MultiThreadedExecutor()
    executor.add_node(mover)
    executor.add_node(check_label_service)

    executor.spin()
    rclpy.shutdown()  # ← Klammern ergänzt

if __name__ == '__main__':
    main()