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
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from time import gmtime, strftime

def grasp_callback_array(my_grasp):
  
	pose_array = geometry_msgs.msg.PoseArray();
	pose_array.header.stamp = my_grasp.markers[0].header.stamp
	pose_array.header.frame_id = "/base_footprint"  

	for i in range (0, len(my_grasp.markers)-1):
		pose_target = geometry_msgs.msg.PoseStamped()
		pose_target.header.stamp = my_grasp.markers[0].header.stamp
		pose_target.header.frame_id = "/xtion_rgb_optical_frame"
		pose_target.pose.position.x = my_grasp.markers[i].points[0].x
		pose_target.pose.position.y = my_grasp.markers[i].points[0].y
		pose_target.pose.position.z = my_grasp.markers[i].points[0].z

		## Convert to quaternion

		#u = [0,0,-1]
		u = [1,0,0]
		norm = linalg.norm([my_grasp.markers[i].points[0].x - my_grasp.markers[i].points[1].x, my_grasp.markers[i].points[0].y - my_grasp.markers[i].points[1].y, my_grasp.markers[i].points[0].z - my_grasp.markers[i].points[1].z])
		v = asarray([my_grasp.markers[i].points[0].x - my_grasp.markers[i].points[1].x, my_grasp.markers[i].points[0].y - my_grasp.markers[i].points[1].y, my_grasp.markers[i].points[0].z - my_grasp.markers[i].points[1].z])/norm 

		if (array_equal(u, v)):
		  pose_target.pose.orientation.w = 1
		  pose_target.pose.orientation.x = 0
		  pose_target.pose.orientation.y = 0
		  pose_target.pose.orientation.z = 0
		elif (array_equal(u, negative(v))):
		  pose_target.pose.orientation.w = 0
		  pose_target.pose.orientation.x = 0
		  pose_target.pose.orientation.y = 0
		  pose_target.pose.orientation.z = 1
		else:
		  half = [u[0]+v[0], u[1]+v[1], u[2]+v[2]]
		  pose_target.pose.orientation.w = dot(u, half)
		  temp = cross(u, half)
		  pose_target.pose.orientation.x = temp[0]
		  pose_target.pose.orientation.y = temp[1]
		  pose_target.pose.orientation.z = temp[2]
		norm = math.sqrt(pose_target.pose.orientation.x*pose_target.pose.orientation.x + pose_target.pose.orientation.y*pose_target.pose.orientation.y + 
		  pose_target.pose.orientation.z*pose_target.pose.orientation.z + pose_target.pose.orientation.w*pose_target.pose.orientation.w)
		
		if norm == 0:
			norm = 1
		
		pose_target.pose.orientation.x = pose_target.pose.orientation.x/norm
		pose_target.pose.orientation.y = pose_target.pose.orientation.y/norm
		pose_target.pose.orientation.z = pose_target.pose.orientation.z/norm
		pose_target.pose.orientation.w = pose_target.pose.orientation.w/norm

		# Publish
		pose_target_trans = geometry_msgs.msg.PoseStamped()
		now = rospy.Time.now()
		listener.waitForTransform("/map", "/xtion_rgb_optical_frame", now, rospy.Duration(1.0))
		pose_target_trans = listener.transformPose("/map", pose_target)
		pose_target_trans.header.frame_id = "/map"

		
		pose_array.poses.append(pose_target_trans.pose)


	my_grasp_pub_array.publish(pose_array)



def grasp_callback(my_grasp):
  
	my_pose = geometry_msgs.msg.PoseStamped();
	my_pose.header.stamp = my_grasp.markers[0].header.stamp
	my_pose.header.frame_id = "/xtion_rgb_optical_frame"  

	 
	pose_target = geometry_msgs.msg.Pose()
	pose_target.position.x = my_grasp.markers[0].points[0].x
	pose_target.position.y = my_grasp.markers[0].points[0].y
	pose_target.position.z = my_grasp.markers[0].points[0].z

	## Convert to quaternion

	u = [1,0,0]
	norm = linalg.norm([my_grasp.markers[i].points[0].x - my_grasp.markers[0].points[1].x, my_grasp.markers[0].points[0].y - my_grasp.markers[0].points[1].y, my_grasp.markers[0].points[0].z - my_grasp.markers[0].points[1].z])
	v = asarray([my_grasp.markers[0].points[0].x - my_grasp.markers[0].points[1].x, my_grasp.markers[0].points[0].y - my_grasp.markers[0].points[1].y, my_grasp.markers[0].points[0].z - my_grasp.markers[0].points[1].z])/norm 

	if (array_equal(u, v)):
	  pose_target.orientation.w = 1
	  pose_target.orientation.x = 0
	  pose_target.orientation.y = 0
	  pose_target.orientation.z = 0
	elif (array_equal(u, negative(v))):
	  pose_target.orientation.w = 0
	  pose_target.orientation.x = 0
	  pose_target.orientation.y = 0
	  pose_target.orientation.z = 1
	else:
	  half = [u[0]+v[0], u[1]+v[1], u[2]+v[2]]
	  pose_target.orientation.w = dot(u, half)
	  temp = cross(u, half)
	  pose_target.orientation.x = temp[0]
	  pose_target.orientation.y = temp[1]
	  pose_target.orientation.z = temp[2]
	norm = math.sqrt(pose_target.orientation.x*pose_target.orientation.x + pose_target.orientation.y*pose_target.orientation.y + 
	  pose_target.orientation.z*pose_target.orientation.z + pose_target.orientation.w*pose_target.orientation.w)
	
	if norm == 0:
		norm = 1
	
	my_pose.pose.orientation.x = pose_target.orientation.x/norm
	my_pose.pose.orientation.y = pose_target.orientation.y/norm
	my_pose_.pose.orientation.z = pose_target.orientation.z/norm
	my_pose.pose.orientation.w = pose_target.orientation.w/norm

	pose_target_trans = geometry_msgs.msg.PoseStamped()
	pose_target_trans.header.stamp = pose_target.header.stamp
	pose_target_trans.header.frame_id = "/map"
	now = rospy.Time.now()
	listener.waitForTransform("/map", "/xtion_rgb_optical_frame", now, rospy.Duration(1.0))
	pose_target_trans = listener.transformPose("/map", pose_target)

	my_grasp_pub.publish(pose_target_trans)


  

if __name__ == '__main__':
  
  ## First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('convert_grasp_to_pose', anonymous=True)
  listener = tf.TransformListener()
	
  # Get ClassName
  className = rospy.get_param("convert_grasp_to_pose/className")
  topicname_fga = '/' + className + '/final_grasp_array'
  topicname_fg = '/' + className + '/final_grasp'
  
  # Publisher 
  my_grasp_pub_array = rospy.Publisher(topicname_fga, PoseArray, queue_size=10)
  my_grasp_pub = rospy.Publisher(topicname_fg, PoseStamped, queue_size=10)
  # Subscriber
  rospy.Subscriber("/object_grasps/handles_visual", MarkerArray, grasp_callback_array)

  #print "============ SPIN ============"
  rospy.spin()
