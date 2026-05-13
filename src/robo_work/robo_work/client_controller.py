from robo_work_msg.srv import GetGraspPoint, IsLabelCorrect, IsPartPresent
import rclpy
from rclpy.node import Node
import time

class RoboController(Node):
    def __init__(self):
        super().__init__('robo_controller')

        ## Client für jeden Service
        self.cli_part = self.create_client(IsPartPresent, 'is_part_present')
        self.cli_label = self.create_client(IsLabelCorrect,'check_label')
        self.cli_grasp = self.create_client(GetGraspPoint,'get_grasp_point')

        ## warten bis alle services bereit
        for cli in [self.cli_part, self.cli_label, self.cli_grasp]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Warte auf Service...')

    def run(self):
        
        # Bauteil da?
        req = IsPartPresent.Request() ## leeres request objekt erstellen
        req.start_check_for_part = True ## request befüllen
        future = self.cli_part.call_async(req) ## request abschicken
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if not future.result().is_present:
            self.get_logger().info('Kein Bauteil gefunden.')
            return
        time.sleep(1)
        # Label korrekt?
        req = IsLabelCorrect.Request()
        req.start_detect_label = True
        future = self.cli_label.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if not future.result().label_classification:
            self.get_logger().info('Label falsch.')
            return
        time.sleep(1)
        # Greifpunkt holen
        req = GetGraspPoint.Request()
        req.start_detect_grasp_point = True
        future = self.cli_grasp.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        grasp = future.result().grasp_points
        self.get_logger().info(f'Greifpunkt: {grasp}')

def main():
    rclpy.init()
    controller = RoboController()
    controller.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()