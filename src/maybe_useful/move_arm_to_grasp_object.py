#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import tf
from scipy import linalg
from scipy import asarray
from scipy import array_equal
from scipy import negative
from scipy import dot
from scipy import math
from scipy import cross

from visualization_msgs.msg import MarkerArray
from agile_grasp.msg import Grasps
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from time import gmtime, strftime


left_arm = None 

def grasp_array_callback(my_grasp_array):

	global left_arm
	if (left_arm):
		# Try left hand first
		group1 = moveit_commander.MoveGroupCommander("arm_left")
		group1.set_planner_id("RRTConnectkConfigDefault")
		group1.set_goal_tolerance(0.01)
		group1.set_pose_targets(my_grasp_array.poses, end_effector_link="left_gripper_parm_0")
		print "============ Left arm moving ============"
		group1.go(wait=True)
		group1.clear_pose_targets()
		rospy.sleep(1)
		left_arm = False

	else:
		rospy.sleep(3)
		group2 = moveit_commander.MoveGroupCommander("arm_right")
		group2.set_planner_id("RRTConnectkConfigDefault")
		group2.set_goal_tolerance(0.01)
		group2.set_pose_targets(my_grasp_array.poses, end_effector_link="right_gripper_parm_0")
		plan2 = group2.plan()
		print "============ Right arm moving ============"
		group2.go(wait=True)
		group2.clear_pose_targets()
		rospy.sleep(1)
		left_arm = True
    


def grasp_callback(my_grasp):

	print my_grasp

	# Try left hand first
	group1 = moveit_commander.MoveGroupCommander("trunk_with_arm_left")
	group1.set_planner_id("RRTConnectkConfigDefault")
	group1.set_goal_tolerance(0.03)
	group1.set_pose_target(my_grasp, end_effector_link="left_gripper_parm_0")
	plan1 = group1.plan()

	if (plan1==True):
		print "============ Left arm moving ============"
		group1.go(wait=True)
	else:
		rospy.sleep(3)
		group2 = moveit_commander.MoveGroupCommander("trunk_with_arm_right")
		group2.set_planner_id("RRTConnectkConfigDefault")
		group2.set_goal_tolerance(0.03)
		group2.set_pose_target(my_grasp, end_effector_link="right_gripper_parm_0")
		plan2 = group2.plan()

		if (plan2==True):
			print "============ Right arm moving ============"
			group2.go(wait=True)    

	print "============ DONE PLANNING/MOVING ============"
  ## Sleep to give Rviz time to visualize the plan. */
	rospy.sleep(3)
	group1.clear_pose_targets()
	group2.clear_pose_targets()


if __name__ == '__main__':
  
  ## First initialize moveit_commander and rospy.
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_arm', anonymous=True)
  #listener = tf.TransformListener()
  
  ## Initializ moveit
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()

	# Subscriber
	rospy.Subscriber("/class/final_grasp", PoseStamped, grasp_callback)
	rospy.Subscriber("/class/final_grasp_array", PoseArray, grasp_array_callback)

	

  #print "============ SPIN ============"
	rospy.spin()
