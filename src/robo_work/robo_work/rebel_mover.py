# um den roboter zu steuern
# rebel mover imports
import math
import rclpy
from rclpy.node import Node 
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, PlanningOptions, Constraints,
    JointConstraint, PositionConstraint, OrientationConstraint,
)
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from scipy.spatial.transform import Rotation as R

PLANNING_GROUP   = "igus_rebel_arm"
VELOCITY_SCALING = 0.3
END_EFFECTOR_LINK = "rg2_hand"   
BASE_FRAME        = "base_link" 

#------- Roboter Bewegung Steuern ---------------------------------------
class RebelMover(Node):
    def __init__(self):
        super().__init__("rebel_mover")
        self._action_client = ActionClient(self, MoveGroup, "/move_action")
        self._action_client.wait_for_server()

    # ---------- Gelenkwinkel-Steuerung (wie bisher) ----------
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
        self.move_to_joint_pos([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def move_to_dummy_position(self, joint_angles_deg):
        # funktion zum testen
        pos = [math.radians(a) for a in joint_angles_deg]
        return self.move_to_joint_pos(pos)

    def move_to_label_check_pos(self):
        # funktion anpassen
        pos = [math.radians(0), math.radians(0), math.radians(0),
               math.radians(0), math.radians(0), math.radians(0)]
        return self.move_to_joint_pos(pos)

    # ---------- Kartesische Steuerung (neu) ----------
    def move_to_pose(self, x, y, z, rotation_matrix,
                     frame_id=BASE_FRAME,
                     link_name=END_EFFECTOR_LINK,
                     pos_tolerance=0.005,
                     ori_tolerance=0.05):
        """
        Fährt den TCP an eine kartesische Pose.
        x, y, z         : Position in Metern (im frame_id)
        rotation_matrix : 3x3 Rotationsmatrix (z.B. von HappyPose)
        """
        # Rotationsmatrix -> Quaternion [x, y, z, w]
        quat = R.from_matrix(rotation_matrix).as_quat()

        constraints = Constraints()

        # --- Positions-Constraint ---
        pc = PositionConstraint()
        pc.header.frame_id = frame_id
        pc.link_name       = link_name
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tolerance]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        # --- Orientierungs-Constraint ---
        oc = OrientationConstraint()
        oc.header.frame_id = frame_id
        oc.link_name       = link_name
        oc.orientation.x = quat[0]
        oc.orientation.y = quat[1]
        oc.orientation.z = quat[2]
        oc.orientation.w = quat[3]
        oc.absolute_x_axis_tolerance = ori_tolerance
        oc.absolute_y_axis_tolerance = ori_tolerance
        oc.absolute_z_axis_tolerance = ori_tolerance
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        return self._send_goal(constraints)

    def move_to_pregrasp(self, x, y, z, rotation_matrix, approach_offset=0.10):
        """
        Fährt 'approach_offset' Meter über dem Greifpunkt an (entlang Welt-Z).
        Sinnvoll vor dem eigentlichen Zugreifen.
        """
        return self.move_to_pose(x, y, z + approach_offset, rotation_matrix)

    # ---------- Goal-Versand ----------
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