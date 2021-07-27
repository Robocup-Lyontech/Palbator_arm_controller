#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from rospy.rostime import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
import moveit_commander
from moveit_msgs.msg import Grasp
from tf.transformations import *
import copy

class Test_grasp:

    def __init__(self):
        """
        Initializes the global controller which will control Palbator Moveit arm and column controllers
        """
        rospy.init_node('test_grasp', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("pmb2_arm")
        # self._gripper_controller = moveit_commander.MoveGroupCommander("pmb2_gripper")

    def addCollionObject(self):
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_footprint"
        box_pose.pose.position.x = 0.4
        box_pose.pose.position.y = -0.2
        box_pose.pose.position.z = 0.6
        box_pose.pose.orientation.w = 1.0
        box_name = "table"
        self.scene.add_box(box_name, box_pose, size=(0.2, 0.4, 0.4))

        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_footprint"
        box_pose.pose.position.x = 0.4
        box_pose.pose.position.y = -0.2
        box_pose.pose.position.z = 0.9
        box_pose.pose.orientation.w = 1.0
        box_name = "object"
        self.scene.add_box(box_name, box_pose, size=(0.02, 0.02, 0.2))

        rospy.loginfo("{class_name} : Object spawned".format(class_name=self.__class__.__name__))

    def pick(self):
        grasps = Grasp()

        grasps.grasp_pose.header.frame_id = "base_footprint"
        orientation = quaternion_from_euler(0, 0, 0)
        grasps.grasp_pose.pose.orientation.x = orientation[0]
        grasps.grasp_pose.pose.orientation.y = orientation[1]
        grasps.grasp_pose.pose.orientation.z = orientation[2]
        grasps.grasp_pose.pose.orientation.w = orientation[3]
        grasps.grasp_pose.pose.position.x = 0.4
        grasps.grasp_pose.pose.position.y = -0.2
        grasps.grasp_pose.pose.position.z = 0.9

        grasps.pre_grasp_approach.direction.header.frame_id = "end_effector_link"
        grasps.pre_grasp_approach.direction.vector.x = 1.0
        grasps.pre_grasp_approach.min_distance = 0.05
        grasps.pre_grasp_approach.desired_distance = 0.115

        grasps.post_grasp_retreat.direction.header.frame_id = "end_effector_link"
        grasps.post_grasp_retreat.direction.vector.z = 1.0
        grasps.post_grasp_retreat.min_distance = 0.05
        grasps.post_grasp_retreat.desired_distance = 0.25

        grasps.pre_grasp_posture.joint_names.append("motor1_hand_joint")
        grasps.pre_grasp_posture.joint_names.append("motor2_hand_joint")
        points = JointTrajectoryPoint()
        points.positions.append(-0.6)
        points.positions.append(-0.6)
        points.time_from_start += Duration(5)
        grasps.pre_grasp_posture.points = [copy.deepcopy(points)]

        grasps.grasp_posture.joint_names.append("motor1_hand_joint")
        grasps.grasp_posture.joint_names.append("motor2_hand_joint")
        points = JointTrajectoryPoint()
        points.positions.append(0.3)
        points.positions.append(0.3)
        points.time_from_start += Duration(5)
        grasps.grasp_posture.points = [copy.deepcopy(points)]

        self.group.set_support_surface_name("table")
        self.group.pick("object", grasps)

    def place(self):
        grasps = Grasp()

        grasps.grasp_pose.header.frame_id = "base_footprint"
        orientation = quaternion_from_euler(0, 0, 0)
        grasps.grasp_pose.pose.orientation.x = orientation[0]
        grasps.grasp_pose.pose.orientation.y = orientation[1]
        grasps.grasp_pose.pose.orientation.z = orientation[2]
        grasps.grasp_pose.pose.orientation.w = orientation[3]
        grasps.grasp_pose.pose.position.x = 0.4
        grasps.grasp_pose.pose.position.y = -0.2
        grasps.grasp_pose.pose.position.z = 0.9

        grasps.pre_grasp_approach.direction.header.frame_id = "end_effector_link"
        grasps.pre_grasp_approach.direction.vector.z = -1.0
        grasps.pre_grasp_approach.min_distance = 0.05
        grasps.pre_grasp_approach.desired_distance = 0.115

        grasps.post_grasp_retreat.direction.header.frame_id = "end_effector_link"
        grasps.post_grasp_retreat.direction.vector.x = -1.0
        grasps.post_grasp_retreat.min_distance = 0.05
        grasps.post_grasp_retreat.desired_distance = 0.25

        grasps.pre_grasp_posture.joint_names.append("motor1_hand_joint")
        grasps.pre_grasp_posture.joint_names.append("motor2_hand_joint")
        points = JointTrajectoryPoint()
        points.positions.append(0.3)
        points.positions.append(0.3)
        points.time_from_start += Duration(5)
        grasps.pre_grasp_posture.points = [copy.deepcopy(points)]

        grasps.grasp_posture.joint_names.append("motor1_hand_joint")
        grasps.grasp_posture.joint_names.append("motor2_hand_joint")
        points = JointTrajectoryPoint()
        points.positions.append(-0.6)
        points.positions.append(-0.6)
        points.time_from_start += Duration(5)
        grasps.grasp_posture.points = [copy.deepcopy(points)]

        self.group.set_support_surface_name("table")
        self.group.place("object", grasps)

if __name__ == "__main__":

    grasp = Test_grasp()

    rospy.sleep(2)

    grasp.addCollionObject()
    grasp.pick()
    grasp.place()
    rospy.spin()
