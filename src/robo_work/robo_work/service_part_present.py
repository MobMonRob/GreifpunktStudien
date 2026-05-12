## Service um die Anwesenheit eines Bauteils auf der Ablagefläche zu prüfen ##

# srv importieren --> definiert welche Datentypen Service server / client austauschen
from robo_work_msg.srv import IsPartPresent

# übliche imports
import time
import rclpy
from rclpy.node import Node


# rebel mover imports
import math
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, PlanningOptions, Constraints,
    JointConstraint,
)

PLANNING_GROUP   = "igus_rebel_arm"
VELOCITY_SCALING = 0.3

#------- Roboter Bewegung Steuern ---------------------------------------
class RebelMover(Node):
    def __init__(self):
        super().__init__("rebel_mover")
        self._action_client = ActionClient(self, MoveGroup, "/move_action")
        self._action_client.wait_for_server()

    def move_to_joint_pos(self, joint_values):
        joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6"]
        constraints = Constraints()
        for jname, jval in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name      = jname
            jc.position        = jval
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)
        return self._send_goal(constraints)

    def move_to_home(self):
        return self.move_to_joint_pos([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def move_to_label_check_pos(self):
        # ← hier deine echten Winkel eintragen
        pos = [math.radians(0), math.radians(0), math.radians(0),
               math.radians(0), math.radians(0), math.radians(0)]
        return self.move_to_joint_pos(pos)

    def _send_goal(self, constraints):
        request = MotionPlanRequest()
        request.group_name                      = PLANNING_GROUP
        request.max_velocity_scaling_factor     = VELOCITY_SCALING
        request.max_acceleration_scaling_factor = VELOCITY_SCALING
        request.allowed_planning_time           = 5.0
        request.num_planning_attempts           = 5
        request.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request                          = request
        goal.planning_options                 = PlanningOptions()
        goal.planning_options.plan_only       = False
        goal.planning_options.replan          = True
        goal.planning_options.replan_attempts = 3

        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.error_code.val == 1

# service klasse erstellen
class IsPartPresentService(Node):
    def __init__(self):
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

    is_part_present_service = IsPartPresentService()
    rclpy.spin(is_part_present_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()