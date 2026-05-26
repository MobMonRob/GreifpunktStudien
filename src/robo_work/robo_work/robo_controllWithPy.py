import time

from cri_lib import CRIController

robot = CRIController()
robot.connect("192.168.3.11",3920)
time.sleep(1)  # Warte kurz, damit die Verbindung stabil ist
robot.set_active_control(True)
robot.enable()
robot.wait_for_kinematics_ready()

# Gelenkwinkel direkt fahren (wie in igus Software)
robot.move_joints([0.0, -77.0, 138.0, 0.0, 81.0, 0.0], wait_move_finished=True)

robot.disable()
robot.close()