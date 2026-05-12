## Service um die Anwesenheit eines Bauteils auf der Ablagefläche zu prüfen ##

# srv importieren --> definiert welche Datentypen Service server / client austauschen
from robo_work_msg.srv import IsPartPresent

# übliche imports
import time
import rclpy
from rclpy.node import Node

# rebel bewegung import
from robo_work.rebel_mover import RebelMover

# service klasse erstellen
class IsPartPresentService(Node):
    def __init__(self,mover):
        super().__init__('is_part_present_service')
        #Datentyp vom service (srv von oben), service name, callback
        self.srv = self.create_service(IsPartPresent, 'is_part_present',self.is_part_present_callback)

    # funktion die der service ausführen soll wenn er eine anfrage bekommt, wenn request XY kommt mache das und gebe zurück response
    def is_part_present_callback(self, request, response):
        if request.start_check_for_part == True:
            self.get_logger().info('Ich prüfe ob ein Bauteil vorhanden ist..')
            time.sleep(3)
            response.is_present = True

        return response
        
# allgemein main um service zu erstellen und auszuführen
def main():
    rclpy.init()

    mover = RebelMover()
    is_part_present_service = IsPartPresentService(mover)

    rclpy.spin(is_part_present_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()