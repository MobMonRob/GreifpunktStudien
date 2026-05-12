# rebel mover imports
import math
import rclpy
from rclpy.node import Node 
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

    def move_to_dummy_position(self,joint_angles_deg):
        # funktion zum testen
        pos = [math.radians(joint_angles_deg[0]), math.radians(joint_angles_deg[1]), math.radians(joint_angles_deg[2]),
               math.radians(joint_angles_deg[3]), math.radians(joint_angles_deg[4]), math.radians(joint_angles_deg[5])]
        return self.move_to_joint_pos(pos)

    def move_to_label_check_pos(self):
        # funktion anpassen
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