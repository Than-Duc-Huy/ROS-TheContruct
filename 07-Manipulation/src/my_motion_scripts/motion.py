#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('motion', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

pose_target = geometry_msgs.msg.Pose()
while(True):
    # Motion 1
    pose_target.orientation.w = 1.0
    pose_target.position.x = 0.1
    pose_target.position.y = 0.4
    pose_target.position.z = 1.2
    group.set_pose_target(pose_target)

    plan1 = group.plan()
    rospy.loginfo("Planned")
    group.go()
    # Motion 2
    pose_target.orientation.w = 1.0
    pose_target.position.x = 0.5
    pose_target.position.y = 0.5
    pose_target.position.z = 1.2
    group.set_pose_target(pose_target)

    plan1 = group.plan()
    rospy.loginfo("Planned")
    group.go()


moveit_commander.roscpp_shutdown()
