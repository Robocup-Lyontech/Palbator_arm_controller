#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from rospy.rostime import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
import moveit_commander
from moveit_msgs.msg import Grasp, PlaceLocation
from tf.transformations import *
import copy
from pmb2_grasp.grasp_generator import GraspGenerator
from pmb2_grasp.grasp_filter import GraspFilter

class Test_grasp:

    def __init__(self):
        rospy.init_node('test_grasp', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("pmb2_arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("pmb2_gripper")
        self._base_link = "base_footprint"
        self.grasp_param = rospy.get_param("~pmb2_gripper")

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

    def pick(self, grasp):
        self.arm_group.set_support_surface_name("table")
        return self.arm_group.pick("object", grasp)

    def place(self):
        place = PlaceLocation()

        place.place_pose.header.frame_id = "base_footprint"
        orientation = quaternion_from_euler(0, 0, 0)
        place.place_pose.pose.orientation.x = orientation[0]
        place.place_pose.pose.orientation.y = orientation[1]
        place.place_pose.pose.orientation.z = orientation[2]
        place.place_pose.pose.orientation.w = orientation[3]
        place.place_pose.pose.position.x = 0.4
        place.place_pose.pose.position.y = -0.2
        place.place_pose.pose.position.z = 0.9

        place.pre_place_approach.direction.header.frame_id = "end_effector_link"
        place.pre_place_approach.direction.vector.z = -1.0
        place.pre_place_approach.min_distance = 0.05
        place.pre_place_approach.desired_distance = 0.115

        place.post_place_retreat.direction.header.frame_id = "end_effector_link"
        place.post_place_retreat.direction.vector.x = -1.0
        place.post_place_retreat.min_distance = 0.05
        place.post_place_retreat.desired_distance = 0.115

        place.post_place_posture.joint_names.append("motor1_hand_joint")
        place.post_place_posture.joint_names.append("motor2_hand_joint")
        points = JointTrajectoryPoint()
        points.positions.append(-0.6)
        points.positions.append(-0.6)
        points.time_from_start += Duration(5)
        place.post_place_posture.points = [copy.deepcopy(points)]

        self.arm_group.set_support_surface_name("table")
        self.arm_group.place("object", place)

    def test(self, graspGenerator, graspFilter):
        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = -0.2
        pose.position.z = 0.9
        pose.orientation.w = 1.0

        grasps = graspGenerator.generateGrasps(self._base_link, pose, self.grasp_param)

        grasps = graspFilter.filterGrasps(grasps, "object", self.grasp_param)
        for e in grasps:
            if self.pick(e) == 1:
                self.place()


if __name__ == "__main__":

    grasp = Test_grasp()
    graspGenerator = GraspGenerator()
    graspFilter = GraspFilter()

    rospy.sleep(2)

    grasp.addCollionObject()
    grasp.test(graspGenerator, graspFilter)
    rospy.spin()
