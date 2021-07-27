#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from pmb2_apps.msg import ArmControlActionGoal

def main():
    rospy.init_node("demo", anonymous=True)

    moveBaseCmd = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
    moveitCmd = rospy.Publisher("Moveit_Palbator_global_action/goal", ArmControlActionGoal, queue_size=1)
    rospy.sleep(1.0)

    move = PoseStamped()
    move.header.frame_id = "map"
    move.pose.position.x = -0.375368058681
    move.pose.position.y = 1.15571939945
    move.pose.orientation.z = 0.829983173919
    move.pose.orientation.w = 0.557788428539

    rospy.loginfo("go to apple")
    moveBaseCmd.publish(move)
    raw_input()

    arm = ArmControlActionGoal()
    arm.goal.action = 'GraspingXYZ'
    arm.goal.pose.position.x = -0.119
    arm.goal.pose.position.y = 1.72
    arm.goal.pose.position.z = 0.72
    arm.goal.solidPrimitive.type = 1
    arm.goal.solidPrimitive.dimensions = [0.04, 0.1, 0.15]

    rospy.loginfo("start grasp apple")
    moveitCmd.publish(arm)
    raw_input()

    move = PoseStamped()
    move.header.frame_id = "map"
    move.pose.position.x = 1.65076386929
    move.pose.position.y = 0.0711629390717
    move.pose.orientation.z = -0.534749479725
    move.pose.orientation.w = 0.845010647231

    rospy.loginfo("move to drop apple")
    moveBaseCmd.publish(move)
    raw_input()

    arm = ArmControlActionGoal()
    arm.goal.action = 'DroppingXYZ'
    arm.goal.pose.position.x = 1.74
    arm.goal.pose.position.y = -0.463
    arm.goal.pose.position.z = 0.55

    rospy.loginfo("drop apple")
    moveitCmd.publish(arm)
    raw_input()

    arm = ArmControlActionGoal()
    arm.goal.action = 'Travelling'

    rospy.loginfo("Travelling pose")
    moveitCmd.publish(arm)
    raw_input()

    move = PoseStamped()
    move.header.frame_id = "map"
    move.pose.position.x = 2.22508788109
    move.pose.position.y = 4.16058063507
    move.pose.orientation.z = 0.852918806314
    move.pose.orientation.w = 0.522043589977

    rospy.loginfo("move to shelf")
    moveBaseCmd.publish(move)
    raw_input()

    arm = ArmControlActionGoal()
    arm.goal.action = 'GraspingXYZ'
    arm.goal.pose.position.x = 2.22
    arm.goal.pose.position.y = 4.75
    arm.goal.pose.position.z = 0.9
    arm.goal.solidPrimitive.type = 1
    arm.goal.solidPrimitive.dimensions = [0.06, 0.06, 0.15]

    rospy.loginfo("grasp mustard")
    moveitCmd.publish(arm)
    raw_input()

    move = PoseStamped()
    move.header.frame_id = "map"
    move.pose.position.x = 0.887392520905
    move.pose.position.y = 3.92378330231
    move.pose.orientation.z = 0.99975343255
    move.pose.orientation.w = 0.0222052719987

    rospy.loginfo("move to person")
    moveBaseCmd.publish(move)
    raw_input()

    arm = ArmControlActionGoal()
    arm.goal.action = 'DroppingXYZ'
    arm.goal.pose.position.x = 0.5
    arm.goal.pose.position.y = 4.0
    arm.goal.pose.position.z = 0.1

    rospy.loginfo("drop mustard")
    moveitCmd.publish(arm)
    raw_input()

    arm = ArmControlActionGoal()
    arm.goal.action = 'Travelling'

    rospy.loginfo("Travelling pose")
    moveitCmd.publish(arm)
    rospy.spin()


if __name__ == "__main__":
    main()