import sys

from example_interfaces.srv import AddTwoInts
from robo_work_msg.srv import IsPartPresent
import rclpy
from rclpy.node import Node
import requests

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)

class PartChecker(Node):
    def __init__(self):
        super().__init__('part_checker_async')
        self.cli = self.create_client(IsPartPresent, 'is_part_present')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = IsPartPresent.Request()
    
    def ask_if_part_there(self, start_check_for_part):
        self.req.start_check_for_part = start_check_for_part
        return self.cli.call_async(self.req)
    
    ## Test um mit anderen Laptops zu kommunizieren --> später wo anders hin
    def abfrage_ablage(self, start):
        if start == True:
            antwortablage = requests.get('http://IPXX/check_part')
            data = antwortablage.json()
            print(data)
            return data


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    part_checker = PartChecker()
    bauteilda = part_checker.ask_if_part_there(True)
    rclpy.spin_until_future_complete(part_checker, bauteilda)
    antwort = bauteilda.result()
    part_checker.get_logger().info('Bauteil ist da? ' + str(antwort))
    
    ## Test: mit anderen Laptops im LAN kommunizieren
    #antwort = part_checker.abfrage_ablage(True)
    #part_checker.get_logger().info('Liegt da ein Bauteil? ' + str(antwort))

    minimal_client.destroy_node()
    part_checker.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()