## Service um den Greifpunkt zu bestimmen --> über Abfrage von anderen Laptop ##

# srv import
from robo_work_msg.srv import getGraspPoint

# Python Imports
import requests

# übliche imports
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


#------------- service klasse ------------------------------
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