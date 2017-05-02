#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <diagnostic_msgs/KeyValue.h>
#include <sensor_msgs/PointCloud2.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include "semantic_mapping/BoundingBox_data.h"

#include <boost/bind.hpp>

#include <sstream>
#include <iostream>
#include <cmath>
#include <signal.h>
#include <stdio.h> 

using namespace std;
using namespace pcl;
using namespace message_filters;


// Define links that are not checked for collsiion with octomap
const char* args_r[] = {"right_gripper_forefinger_1", "right_gripper_forefinger_0", "right_gripper_littlefinger_1",  "right_gripper_littlefinger_0", "right_gripper_thumb_1", "right_gripper_thumb_0", "right_gripper_parm_0", "right_gripper_parm_1", "right_gripper_parm_2", "right_wrist_2"};
std::vector<std::string> v_r(args_r, args_r + 10);

const char* args_l[] = {"left_gripper_forefinger_1", "left_gripper_forefinger_0", "left_gripper_littlefinger_1",  "left_gripper_littlefinger_0", "left_gripper_thumb_1", "left_gripper_thumb_0", "left_gripper_parm_0", "left_gripper_parm_1", "left_gripper_parm_2", "left_wrist_2"};
std::vector<std::string> v_l(args_l, args_l + 10);

ros::Publisher clear_regist_pub;
ros::Publisher planning_scene_diff_publisher;
ros::Publisher m_pub_scene;
ros::Publisher m_pub_gripper_right;
ros::Publisher m_pub_gripper_left;
ros::Publisher m_pub_arm_right;
ros::Publisher m_pub_arm_left;
ros::Publisher segm_pcl_pub;
ros::Publisher reg_pcl_pub;

ros::ServiceClient m_service_get_planning_scene;

robot_model::RobotModelPtr kinematic_model;

boost::shared_ptr<move_group_interface::MoveGroup> moveGroupPtr_r; 
boost::shared_ptr<move_group_interface::MoveGroup> moveGroupPtr_l;
boost::shared_ptr<move_group_interface::MoveGroup> moveGroupPtr_gr; 
boost::shared_ptr<move_group_interface::MoveGroup> moveGroupPtr_gl;

std::string className;
std::string list_exception ("handle");



// ----------------- Old joint trayectories ----------------


void set_gripper_right_open_old(){

	std::vector<std::string> names(3);
	names[0] = "right_gripper_thumb_joint_0";
  names[1] = "right_gripper_forefinger_joint_0";
  names[2] = "right_gripper_littlefinger_joint_0";

	std::vector<double> angles(3,0);
	angles[0] = -20.0/180.0*3.14;
  angles[1] = -20.0/180.0*3.14;
  angles[2] = +20.0/180.0*3.14;

	trajectory_msgs::JointTrajectory msg;
  if(names.size() != angles.size()){
		ROS_INFO("Error");	  
		return;
	}

  msg.header.stamp = ros::Time::now() + ros::Duration(0.2);
  msg.header.frame_id = "base_footprint";
  msg.points.resize(1);   // Only contain a single joint trajectory
  for(int i = 0; i < names.size(); ++i){
      msg.joint_names.push_back(names[i]);
      msg.points[0].positions.push_back(angles[i]);
  }
  msg.points[0].velocities.resize(names.size(),0);
  msg.points[0].accelerations.resize(names.size(),0);
  msg.points[0].effort.resize(names.size(),0);
  msg.points[0].time_from_start = ros::Duration(0.2);
	
	for (int i =0;i<5;i++)
	{	
		m_pub_gripper_right.publish(msg);
		sleep(1);
	}
	return;
}


void set_gripper_left_open_old(){

	std::vector<std::string> names(3);
	names[0] = "left_gripper_thumb_joint_0";
  names[1] = "left_gripper_forefinger_joint_0";
  names[2] = "left_gripper_littlefinger_joint_0";

	std::vector<double> angles(3,0);
	angles[0] = +20.0/180.0*3.14;
  angles[1] = +20.0/180.0*3.14;
  angles[2] = -20.0/180.0*3.14;

	trajectory_msgs::JointTrajectory msg;
  if(names.size() != angles.size()){
		ROS_INFO("Error");	  
		return;
	}

  msg.header.stamp = ros::Time::now() + ros::Duration(0.2);
  msg.header.frame_id = "base_footprint";
  msg.points.resize(1);   // Only contain a single joint trajectory
  for(int i = 0; i < names.size(); ++i){
      msg.joint_names.push_back(names[i]);
      msg.points[0].positions.push_back(angles[i]);
  }
  msg.points[0].velocities.resize(names.size(),0);
  msg.points[0].accelerations.resize(names.size(),0);
  msg.points[0].effort.resize(names.size(),0);
  msg.points[0].time_from_start = ros::Duration(0.2);

	for (int i =0;i<5;i++)
	{	
		m_pub_gripper_left.publish(msg);
		sleep(1);
	}
	return;
}


void set_gripper_right_close_old(){

	std::vector<std::string> names(3);
	names[0] = "right_gripper_thumb_joint_0";
  names[1] = "right_gripper_forefinger_joint_0";
  names[2] = "right_gripper_littlefinger_joint_0";
	std::vector<double> angles(3,0);
  angles[0] = +5.0/180.0*3.14;
  angles[1] = +5.0/180.0*3.14;
  angles[2] = -5.0/180.0*3.14;

	trajectory_msgs::JointTrajectory msg;
  if(names.size() != angles.size())   return;

  msg.header.stamp = ros::Time::now() + ros::Duration(0.2);
  msg.header.frame_id = "base_footprint";
  msg.points.resize(1);   // Only contain a single joint trajectory
  for(int i = 0; i < names.size(); ++i){
      msg.joint_names.push_back(names[i]);
      msg.points[0].positions.push_back(angles[i]);
  }
  msg.points[0].velocities.resize(names.size(),0);
  msg.points[0].accelerations.resize(names.size(),0);
  msg.points[0].effort.resize(names.size(),0);
  msg.points[0].time_from_start = ros::Duration(0.2);

	for (int i =0;i<5;i++)
	{	
		m_pub_gripper_right.publish(msg);
		sleep(1);
	}
	return;
}





void set_gripper_left_close_old(){

	std::vector<std::string> names(3);
	names[0] = "left_gripper_thumb_joint_0";
  names[1] = "left_gripper_forefinger_joint_0";
  names[2] = "left_gripper_littlefinger_joint_0";
	std::vector<double> angles(3,0);
  angles[0] = -5.0/180.0*3.14;
  angles[1] = -5.0/180.0*3.14;
  angles[2] = +5.0/180.0*3.14;

	trajectory_msgs::JointTrajectory msg;
  if(names.size() != angles.size())   return;

  msg.header.stamp = ros::Time::now() + ros::Duration(0.2);
  msg.header.frame_id = "base_footprint";
  msg.points.resize(1);   // Only contain a single joint trajectory
  for(int i = 0; i < names.size(); ++i){
      msg.joint_names.push_back(names[i]);
      msg.points[0].positions.push_back(angles[i]);
  }
  msg.points[0].velocities.resize(names.size(),0);
  msg.points[0].accelerations.resize(names.size(),0);
  msg.points[0].effort.resize(names.size(),0);
  msg.points[0].time_from_start = ros::Duration(0.2);

	for (int i =0;i<5;i++)
	{	
		m_pub_gripper_left.publish(msg);
		sleep(1);
	}
	return;
}

/*
void set_arm_right_home(){
    std::vector<std::string> names(8);
    names[0] = "right_shoulder_joint_0";
    names[1] = "right_shoulder_joint_1";
    names[2] = "right_shoulder_joint_2";
    names[3] = "right_shoulder_joint_3";
    names[4] = "right_elbow_joint_0";
    names[5] = "right_wrist_joint_0";
    names[6] = "right_wrist_joint_1";
    names[7] = "right_wrist_joint_2";

		std::vector<double> angles(8,0);
		angles[0] = -0.1;
		angles[1] = -0.5;
		angles[2] = 0.5;
		angles[3] = 0.0;
		angles[4] = 1.0;
		angles[5] = 0.0;
		angles[6] = 0.0;
		angles[7] = 0.0;

    trajectory_msgs::JointTrajectory msg;
		if(names.size() != angles.size())   return;

		msg.header.stamp = ros::Time::now() + ros::Duration(0.2);
		msg.header.frame_id = "base_footprint";
		msg.points.resize(1);   // Only contain a single joint trajectory
		for(int i = 0; i < names.size(); ++i){
		    msg.joint_names.push_back(names[i]);
		    msg.points[0].positions.push_back(angles[i]);
		}
		msg.points[0].velocities.resize(names.size(),0);
		msg.points[0].accelerations.resize(names.size(),0);
		msg.points[0].effort.resize(names.size(),0);
		msg.points[0].time_from_start = ros::Duration(0.2);

		m_pub_arm_right.publish(msg);
		sleep(1);
		return;
}


void set_arm_right_hello(){
    std::vector<std::string> names(8);
    names[0] = "right_shoulder_joint_0";
    names[1] = "right_shoulder_joint_1";
    names[2] = "right_shoulder_joint_2";
    names[3] = "right_shoulder_joint_3";
    names[4] = "right_elbow_joint_0";
    names[5] = "right_wrist_joint_0";
    names[6] = "right_wrist_joint_1";
    names[7] = "right_wrist_joint_2";

		std::vector<double> angles(8,0);
		angles[0] = -0.5;
		angles[1] = 1.0;
		angles[2] = 0.5;
		angles[3] = 0.0;
		angles[4] = 1.5707;
		angles[5] = 0.0;
		angles[6] = 0.0;
		angles[7] = 0.5;

    trajectory_msgs::JointTrajectory msg;
		if(names.size() != angles.size())   return;

		msg.header.stamp = ros::Time::now() + ros::Duration(0.2);
		msg.header.frame_id = "base_footprint";
		msg.points.resize(1);   // Only contain a single joint trajectory
		for(int i = 0; i < names.size(); ++i){
		    msg.joint_names.push_back(names[i]);
		    msg.points[0].positions.push_back(angles[i]);
		}
		msg.points[0].velocities.resize(names.size(),0);
		msg.points[0].accelerations.resize(names.size(),0);
		msg.points[0].effort.resize(names.size(),0);
		msg.points[0].time_from_start = ros::Duration(0.2);

		m_pub_arm_right.publish(msg);
		sleep(1);
		return;
}

void set_arm_left_home(){
    std::vector<std::string> names(8);
    names[0] = "left_shoulder_joint_0";
    names[1] = "left_shoulder_joint_1";
    names[2] = "left_shoulder_joint_2";
    names[3] = "left_shoulder_joint_3";
    names[4] = "left_elbow_joint_0";
    names[5] = "left_wrist_joint_0";
    names[6] = "left_wrist_joint_1";
    names[7] = "left_wrist_joint_2";

		std::vector<double> angles(8,0);
		angles[0] = 0.1;
		angles[1] = 0.5;
		angles[2] = -0.5;
		angles[3] = 0.0;
		angles[4] = -1.0;
		angles[5] = 0.0;
		angles[6] = 0.0;
		angles[7] = 0.0;

    trajectory_msgs::JointTrajectory msg;
		if(names.size() != angles.size())   return;

		msg.header.stamp = ros::Time::now() + ros::Duration(0.2);
		msg.header.frame_id = "base_footprint";
		msg.points.resize(1);   // Only contain a single joint trajectory
		for(int i = 0; i < names.size(); ++i){
		    msg.joint_names.push_back(names[i]);
		    msg.points[0].positions.push_back(angles[i]);
		}
		msg.points[0].velocities.resize(names.size(),0);
		msg.points[0].accelerations.resize(names.size(),0);
		msg.points[0].effort.resize(names.size(),0);
		msg.points[0].time_from_start = ros::Duration(0.2);

		m_pub_arm_left.publish(msg);
		sleep(1);
		return;
}

void set_arm_left_hello(){
    std::vector<std::string> names(8);
    names[0] = "left_shoulder_joint_0";
    names[1] = "left_shoulder_joint_1";
    names[2] = "left_shoulder_joint_2";
    names[3] = "left_shoulder_joint_3";
    names[4] = "left_elbow_joint_0";
    names[5] = "left_wrist_joint_0";
    names[6] = "left_wrist_joint_1";
    names[7] = "left_wrist_joint_2";

		std::vector<double> angles(8,0);
		angles[0] = 0.5;
		angles[1] = -1.0;
		angles[2] = -0.5;
		angles[3] = 0.0;
		angles[4] = -1.5707;
		angles[5] = 0.0;
		angles[6] = 0.0;
		angles[7] = 0.5;

    trajectory_msgs::JointTrajectory msg;
		if(names.size() != angles.size())   return;

		msg.header.stamp = ros::Time::now() + ros::Duration(0.2);
		msg.header.frame_id = "base_footprint";
		msg.points.resize(1);   // Only contain a single joint trajectory
		for(int i = 0; i < names.size(); ++i){
		    msg.joint_names.push_back(names[i]);
		    msg.points[0].positions.push_back(angles[i]);
		}
		msg.points[0].velocities.resize(names.size(),0);
		msg.points[0].accelerations.resize(names.size(),0);
		msg.points[0].effort.resize(names.size(),0);
		msg.points[0].time_from_start = ros::Duration(0.2);

		m_pub_arm_left.publish(msg);
		sleep(1);
		return;
}
*/


// ----------------------- Grasp Functions --------------------


void set_gripper_right_open(){
	
	moveit::planning_interface::MoveGroup::Plan my_plan_gr;
	moveGroupPtr_gr->allowReplanning(true); 

	std::vector<double> group_variable_values;
	moveGroupPtr_gr->getCurrentState()->copyJointGroupPositions(moveGroupPtr_gr->getCurrentState()->getRobotModel()->getJointModelGroup(moveGroupPtr_gr->getName()), group_variable_values); 

	group_variable_values[0] = +20.0/180.0*3.14;
	group_variable_values[1] = -20.0/180.0*3.14;
	group_variable_values[2] = +20.0/180.0*3.14;

	moveGroupPtr_gr->setJointValueTarget(group_variable_values);
	
	bool success_gr = false;
	while (!success_gr){
	success_gr = moveGroupPtr_gr->plan(my_plan_gr);
		if (success_gr){
			moveGroupPtr_gr->move();
		}
	}

}


void set_gripper_left_open(){
	
	moveit::planning_interface::MoveGroup::Plan my_plan_gl;
	moveGroupPtr_gl->allowReplanning(true); 

	std::vector<double> group_variable_values;
	moveGroupPtr_gl->getCurrentState()->copyJointGroupPositions(moveGroupPtr_gl->getCurrentState()->getRobotModel()->getJointModelGroup(moveGroupPtr_gl->getName()), group_variable_values); 

	group_variable_values[0] = +20.0/180.0*3.14;
	group_variable_values[1] = -20.0/180.0*3.14;
	group_variable_values[2] = +20.0/180.0*3.14;

	moveGroupPtr_gl->setJointValueTarget(group_variable_values);
	
	bool success_gl = false;
	while (!success_gl){
	success_gl = moveGroupPtr_gl->plan(my_plan_gl);
		if (success_gl){
			moveGroupPtr_gl->move();
		}
	}

}


void set_gripper_right_close(){
	
	moveit::planning_interface::MoveGroup::Plan my_plan_gr;
	moveGroupPtr_gr->allowReplanning(true); 

	std::vector<double> group_variable_values;
	moveGroupPtr_gr->getCurrentState()->copyJointGroupPositions(moveGroupPtr_gr->getCurrentState()->getRobotModel()->getJointModelGroup(moveGroupPtr_gr->getName()), group_variable_values); 

	group_variable_values[0] = +5.0/180.0*3.14;
	group_variable_values[1] = -5.0/180.0*3.14;
	group_variable_values[2] = +5.0/180.0*3.14;

	moveGroupPtr_gr->setJointValueTarget(group_variable_values);
	
	bool success_gr = false;
	while (!success_gr){
	success_gr = moveGroupPtr_gr->plan(my_plan_gr);
		if (success_gr){
			moveGroupPtr_gr->move();
		}
	}

}


void set_gripper_left_close(){
	
	moveit::planning_interface::MoveGroup::Plan my_plan_gl;
	moveGroupPtr_gl->allowReplanning(true); 

	std::vector<double> group_variable_values;
	moveGroupPtr_gl->getCurrentState()->copyJointGroupPositions(moveGroupPtr_gl->getCurrentState()->getRobotModel()->getJointModelGroup(moveGroupPtr_gl->getName()), group_variable_values); 

	group_variable_values[0] = -5.0/180.0*3.14;
	group_variable_values[1] = +5.0/180.0*3.14;
	group_variable_values[2] = -5.0/180.0*3.14;

	moveGroupPtr_gl->setJointValueTarget(group_variable_values);
	
	bool success_gl = false;
	while (!success_gl){
	success_gl = moveGroupPtr_gl->plan(my_plan_gl);
		if (success_gl){
			moveGroupPtr_gl->move();
		}
	}

}


void set_arm_right_home(){
	
	moveit::planning_interface::MoveGroup::Plan my_plan_r;
	moveGroupPtr_r->allowReplanning(true); 

	std::vector<double> group_variable_values;
	moveGroupPtr_r->getCurrentState()->copyJointGroupPositions(moveGroupPtr_r->getCurrentState()->getRobotModel()->getJointModelGroup(moveGroupPtr_r->getName()), group_variable_values); 

	group_variable_values[0] = -0.1;
	group_variable_values[1] = -0.5;
	group_variable_values[2] = 0.5;
	group_variable_values[3] = 0.0;
	group_variable_values[4] = 1.0;
	group_variable_values[5] = 0.0;
	group_variable_values[6] = 0.0;
	group_variable_values[7] = 0.0;
	moveGroupPtr_r->setJointValueTarget(group_variable_values);
	
	bool success_r = false;
	while (!success_r){
	success_r = moveGroupPtr_r->plan(my_plan_r);
		if (success_r){
			moveGroupPtr_r->move();
		}
	}

}



void set_arm_right_hello(){

	moveit::planning_interface::MoveGroup::Plan my_plan_r;
	moveGroupPtr_r->allowReplanning(true);

	std::vector<double> group_variable_values;
	moveGroupPtr_r->getCurrentState()->copyJointGroupPositions(moveGroupPtr_r->getCurrentState()->getRobotModel()->getJointModelGroup(moveGroupPtr_r->getName()), group_variable_values); 

	group_variable_values[0] = -0.5;
	group_variable_values[1] = 1.0;
	group_variable_values[2] = 0.5;
	group_variable_values[3] = 0.0;
	group_variable_values[4] = 1.5707;
	group_variable_values[5] = 0.0;
	group_variable_values[6] = 0.0;
	group_variable_values[7] = 0.5;
	moveGroupPtr_r->setJointValueTarget(group_variable_values);

	bool success_r = false;
	while (!success_r){
	success_r = moveGroupPtr_r->plan(my_plan_r);
		if (success_r){
			moveGroupPtr_r->move();
		}
	}

}

void set_arm_left_home(){

	moveit::planning_interface::MoveGroup::Plan my_plan_l;
	moveGroupPtr_l->allowReplanning(true);

	std::vector<double> group_variable_values;
	moveGroupPtr_l->getCurrentState()->copyJointGroupPositions(moveGroupPtr_l->getCurrentState()->getRobotModel()->getJointModelGroup(moveGroupPtr_l->getName()), group_variable_values); 

	group_variable_values[0] = 0.1;
	group_variable_values[1] = 0.5;
	group_variable_values[2] = -0.5;
	group_variable_values[3] = 0.0;
	group_variable_values[4] = -1.0;
	group_variable_values[5] = 0.0;
	group_variable_values[6] = 0.0;
	group_variable_values[7] = 0.0;

	moveGroupPtr_l->setJointValueTarget(group_variable_values);

	bool success_l = false;
	while (!success_l){
	success_l = moveGroupPtr_l->plan(my_plan_l);
		if (success_l){
			moveGroupPtr_l->move();
		}
	}

}

void set_arm_left_hello(){

	moveit::planning_interface::MoveGroup::Plan my_plan_l;
	moveGroupPtr_l->allowReplanning(true);

	std::vector<double> group_variable_values;
	moveGroupPtr_l->getCurrentState()->copyJointGroupPositions(moveGroupPtr_l->getCurrentState()->getRobotModel()->getJointModelGroup(moveGroupPtr_l->getName()), group_variable_values); 

	group_variable_values[0] = 0.5;
	group_variable_values[1] = -1.0;
	group_variable_values[2] = -0.5;
	group_variable_values[3] = 0.0;
	group_variable_values[4] = -1.5707;
	group_variable_values[5] = 0.0;
	group_variable_values[6] = 0.0;
	group_variable_values[7] = 0.5;

	moveGroupPtr_l->setJointValueTarget(group_variable_values);

	bool success_l = false;
	while (!success_l){
	success_l = moveGroupPtr_l->plan(my_plan_l);
		if (success_l){
			moveGroupPtr_l->move();
		}
	}


}

// ----------------------- Clear pointclouds --------------------

void clear_pointclouds()
{

	ROS_INFO("Clearing pointclouds - DOES NOT WORK");

	pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data
  empty_cloud_ptr->width    = 1;
  empty_cloud_ptr->height   = 1;
  empty_cloud_ptr->is_dense = false;
  empty_cloud_ptr->points.resize (empty_cloud_ptr->width * empty_cloud_ptr->height);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cloud_ptr (&empty_cloud);
	ros::Time time_st = ros::Time::now ();
	empty_cloud_ptr->header.stamp = time_st.toNSec()/1e3;
	empty_cloud_ptr->header.frame_id = "map";

	segm_pcl_pub.publish(empty_cloud_ptr);
	reg_pcl_pub.publish(empty_cloud_ptr);

/*
	sensor_msgs::PointCloud2 segm_pcl;
	sensor_msgs::PointCloud2 reg_pcl;

	segm_pcl.header.stamp = ros::Time::now();
	segm_pcl.header.frame_id = "map";
	reg_pcl.header.stamp = ros::Time::now();
	reg_pcl.header.frame_id = "map";

	segm_pcl_pub.publish(segm_pcl);
	reg_pcl_pub.publish(reg_pcl);
*/
}

// ----------------------- Move Functions --------------------

void grasp_array_callback(const geometry_msgs::PoseArrayConstPtr& my_grasp_array)
{

/*
	ROS_INFO("Killing specific processes");
	system("rosnode kill ICP_registration");
	system("rosnode kill BB_for_reg_pcl");
	system("rosnode kill object_grasps");
*/

	moveit::planning_interface::MoveGroup::Plan my_plan_r;
	moveit::planning_interface::MoveGroup::Plan my_plan_l;

	// --------- Open hands before everything else -----
	
	ROS_INFO("--- Opening both grippers ---");
	set_gripper_right_open_old();
	set_gripper_left_open_old();

	// -------- Disable collision checking -----------

  moveit_msgs::GetPlanningSceneResponse get_planning_scene_response;
  moveit_msgs::GetPlanningSceneRequest get_planning_scene_request;
  get_planning_scene_request.components.components
        = moveit_msgs::PlanningSceneComponents::ROBOT_STATE
        | moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS
        | moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX
        | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES
        | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
  m_service_get_planning_scene.call(get_planning_scene_request, get_planning_scene_response);
  if(planning_scene::PlanningScene::isEmpty(get_planning_scene_response.scene)){
      std::cout << "[ERROR] EMPTY SCENE IS ARRIVED" << std::endl;
      return;
  }

	ROS_INFO("---- Disabling collision checking ----");
	planning_scene::PlanningScene planning_scene(kinematic_model);
  planning_scene.setPlanningSceneMsg(get_planning_scene_response.scene);

	collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
	planning_scene.checkSelfCollision(collision_request, collision_result);

	collision_detection::AllowedCollisionMatrix& acm = planning_scene.getAllowedCollisionMatrixNonConst();
  robot_state::RobotState copied_state = planning_scene.getCurrentState();
	acm.setEntry("my_collision_object", v_r, true);
	acm.setEntry("my_collision_object", v_l, true);
	acm.setEntry("my_collision_object", "<octomap>", true);
	//planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
  moveit_msgs::PlanningScene planning_scene_diff_msg;
  planning_scene.getPlanningSceneDiffMsg(planning_scene_diff_msg);
  planning_scene_diff_msg.is_diff = true;

	m_pub_scene.publish(planning_scene_diff_msg);

	sleep(2);

	// ----------- Select Right or Left arm ----------

	geometry_msgs::Point leftright;
	int total_points = 0;
	for (int i = 0; i<my_grasp_array->poses.size(); i++){
		// Check no inf or nan
		if (my_grasp_array->poses[i].position.x < 10000.0 && my_grasp_array->poses[i].position.x > -10000.0 && my_grasp_array->poses[i].position.y < 10000.0 && my_grasp_array->poses[i].position.y > -10000.0 && my_grasp_array->poses[i].position.z < 10000.0 && my_grasp_array->poses[i].position.z > -10000.0)
		{
			leftright.x=leftright.x+my_grasp_array->poses[i].position.x;
			leftright.y=leftright.y+my_grasp_array->poses[i].position.y;
			leftright.z=leftright.z+my_grasp_array->poses[i].position.z;
			total_points++;
			//ROS_INFO("%f",my_grasp_array->poses[i].position.y);
		}
		else {continue;}
	}
	leftright.x=leftright.x/total_points;
	leftright.y=leftright.y/total_points;
	leftright.z=leftright.z/total_points;
	ROS_INFO("X-value of poses:  %f",leftright.x);
	ROS_INFO("Y-value of poses:  %f",leftright.y);
	ROS_INFO("Z-value of poses:  %f",leftright.z);

	// Get current end effector pose
	geometry_msgs::PoseStamped current_pose_r =
moveGroupPtr_r->getCurrentPose("right_gripper_parm_2");
	geometry_msgs::PoseStamped current_pose_l =
moveGroupPtr_l->getCurrentPose("left_gripper_parm_2");
	
	float distance_r = sqrt(pow(current_pose_r.pose.position.x - leftright.x,2.0)+pow(current_pose_r.pose.position.y - leftright.y,2.0)+pow(current_pose_r.pose.position.z - leftright.z,2.0));
	ROS_INFO("Distance to right eef: %f",distance_r);

	float distance_l = sqrt(pow(current_pose_l.pose.position.x - leftright.x,2.0)+pow(current_pose_l.pose.position.y - leftright.y,2.0)+pow(current_pose_l.pose.position.z - leftright.z,2.0));
	ROS_INFO("Distance to left eef: %f",distance_l);

	// IMPORTANT: If className = handle we must invert arm selection
	std::size_t found_exception = list_exception.find(className);
	if (found_exception!=std::string::npos)
	{
		distance_r=-distance_r;
		distance_l=-distance_l;
	}


	// ------------- Planning to grasp ---------------

  ROS_INFO("---- Planning to grasp ----");

	// Clear registered pcl
	diagnostic_msgs::KeyValue clear_regist;
	clear_regist.key = "Clear_registration";
	clear_regist.value = "clear";
	clear_regist_pub.publish(clear_regist);

	if (distance_r <= distance_l){

		
		ROS_INFO("---- Right arm selected ----");
		moveGroupPtr_r->setPoseTargets(my_grasp_array->poses);
		moveGroupPtr_r->setPoseReferenceFrame("map");
		moveGroupPtr_r->setGoalTolerance(0.01);
		moveGroupPtr_r->allowReplanning(true);
		bool success_r = moveGroupPtr_r->plan(my_plan_r);

		if (success_r)
		{

			/// Kill processes that detect objects
			ROS_INFO("Killing specific processes");
			system("rosnode kill ICP_registration");
			system("rosnode kill BB_for_reg_pcl");
			system("rosnode kill object_grasps");
			
			// Open gripper
			set_gripper_right_open_old();

			// Move arm
			ROS_INFO("---- Right arm moving ----");
			moveGroupPtr_r->move();
  		//Sleep to give time to execute the plan.
      //sleep(7);

			// Update ACM
			acm.setEntry("<octomap>", v_r, true);
			planning_scene.getPlanningSceneDiffMsg(planning_scene_diff_msg);
			planning_scene_diff_msg.is_diff = true;
			m_pub_scene.publish(planning_scene_diff_msg);
			sleep(2);
		
			// Close gripper
			ROS_INFO("--- Closing right gripper ---");
			set_gripper_right_close();
			
			// Attach object to robot
			ROS_INFO("Attach the object to the robot");
			//moveGroupPtr_r->attachObject("my_collision_object");
			
			// Move to hello position
			set_arm_right_hello();
			//sleep(7);

			// Clear pointclouds
			clear_pointclouds();

			// Update ACM
			acm.setEntry("<octomap>", v_r, false);
			planning_scene.getPlanningSceneDiffMsg(planning_scene_diff_msg);
			planning_scene_diff_msg.is_diff = true;
			m_pub_scene.publish(planning_scene_diff_msg);
			sleep(2);

			// Detach object to robot
			ROS_INFO("Detach the object to the robot");
			//moveGroupPtr_r->detachObject("my_collision_object");

			// Open gripper
			//ROS_INFO("--- Opening right gripper ---");
			//set_gripper_right_open_old();
		
			// Kill all proceses
			ROS_INFO("Press CONTROL+C");
			//system("rosnode kill select_object_to_grasp");
			//system("kill $(ps aux | grep select_object_to_grasp | grep -v grep | awk '{print $2}')");
			//system("pkill select_object_to_grasp");
		
		}
		else
		{
			ROS_INFO("---- Try Left arm instead ----");
			moveGroupPtr_l->setPoseTargets(my_grasp_array->poses);
			moveGroupPtr_l->setPoseReferenceFrame("map");
			moveGroupPtr_l->setGoalTolerance(0.01);
			moveGroupPtr_l->allowReplanning(true);
			bool success_l = moveGroupPtr_l->plan(my_plan_l);

			if (success_l)
			{
				// Kill processes that detect objects
				ROS_INFO("Killing specific processes");
				system("rosnode kill ICP_registration");
				system("rosnode kill BB_for_reg_pcl");
				system("rosnode kill object_grasps");

				// Open gripper
				set_gripper_left_open_old();

				// Move arm
				ROS_INFO("---- Left arm moving ----");
				moveGroupPtr_l->move();
				//Sleep to give time to execute the plan.
		    //sleep(7);

				// Update ACM
				acm.setEntry("<octomap>", v_l, true);
				planning_scene.getPlanningSceneDiffMsg(planning_scene_diff_msg);
				planning_scene_diff_msg.is_diff = true;
				m_pub_scene.publish(planning_scene_diff_msg);
				sleep(2);
		
				// Close gripper
				ROS_INFO("--- Closing left gripper ---");
				set_gripper_left_close();
			
				// Attach object to robot
				ROS_INFO("Attach the object to the robot");
				//moveGroupPtr_l->attachObject("my_collision_object");

				// Move to hello position
				set_arm_left_hello();
				//sleep(7);

				// Clear pointclouds
				clear_pointclouds();

				// Update ACM
				acm.setEntry("<octomap>", v_l, false);
				planning_scene.getPlanningSceneDiffMsg(planning_scene_diff_msg);
				planning_scene_diff_msg.is_diff = true;
				m_pub_scene.publish(planning_scene_diff_msg);
				sleep(2);

				// Detach object to robot
				ROS_INFO("Detach the object to the robot");
				//moveGroupPtr_l->detachObject("my_collision_object");
			
				// Open gripper
				//ROS_INFO("--- Opening left gripper ---");
				//set_gripper_left_open_old();

		
				// Kill all proceses
				//system("pkill roslaunch");
				ROS_INFO("Press CONTROL+C");
			}
		}

	}else if(distance_r >distance_l)
	{

		ROS_INFO("---- Left arm selected ----");
		moveGroupPtr_l->setPoseTargets(my_grasp_array->poses);
		moveGroupPtr_l->setPoseReferenceFrame("map");
		moveGroupPtr_l->setGoalTolerance(0.01);
		moveGroupPtr_l->allowReplanning(true);
  	bool success_l = moveGroupPtr_l->plan(my_plan_l);

		if (success_l)
		{

			// Kill processes that detect objects
			ROS_INFO("Killing specific processes");
			system("rosnode kill ICP_registration");
			system("rosnode kill BB_for_reg_pcl");
			system("rosnode kill object_grasps");

			// Open gripper
			set_gripper_left_open_old();

			// Move arm
			ROS_INFO("---- Left arm moving ----");
			moveGroupPtr_l->move();
			//Sleep to give time to execute the plan.
      //sleep(7);

			// Update ACM
			acm.setEntry("<octomap>", v_l, true);
			planning_scene.getPlanningSceneDiffMsg(planning_scene_diff_msg);
			planning_scene_diff_msg.is_diff = true;
			m_pub_scene.publish(planning_scene_diff_msg);
			sleep(2);
		
			// Close gripper
			ROS_INFO("--- Closing left gripper ---");
			set_gripper_left_close();
			
			// Attach object to robot
			ROS_INFO("Attach the object to the robot");
			//moveGroupPtr_l->attachObject("my_collision_object");

			// Move to hello position
			set_arm_left_hello();
			//sleep(7);

			// Clear pointclouds
			clear_pointclouds();

			// Update ACM
			acm.setEntry("<octomap>", v_l, false);
			planning_scene.getPlanningSceneDiffMsg(planning_scene_diff_msg);
			planning_scene_diff_msg.is_diff = true;
			m_pub_scene.publish(planning_scene_diff_msg);
			sleep(2);

			// Detach object to robot
			ROS_INFO("Detach the object to the robot");
			//moveGroupPtr_l->detachObject("my_collision_object");
			
			// Open gripper
			//ROS_INFO("--- Opening left gripper ---");
			//set_gripper_left_open_old();

		
			// Kill all proceses
			//system("pkill roslaunch");
			ROS_INFO("Press CONTROL+C");
		
		}
		else
		{
			ROS_INFO("---- Try right arm instead ----");
			moveGroupPtr_r->setPoseTargets(my_grasp_array->poses);
			moveGroupPtr_r->setPoseReferenceFrame("map");
			moveGroupPtr_r->setGoalTolerance(0.01);
			moveGroupPtr_r->allowReplanning(true);
			bool success_r = moveGroupPtr_r->plan(my_plan_r);

			if (success_r)
			{
				// Kill processes that detect objects
				ROS_INFO("Killing specific processes");
				system("rosnode kill ICP_registration");
				system("rosnode kill BB_for_reg_pcl");
				system("rosnode kill object_grasps");
				
				// Open gripper
				set_gripper_right_open_old();

				// Move arm
				ROS_INFO("---- Right arm moving ----");
				moveGroupPtr_r->move();
				//Sleep to give time to execute the plan.
		    //sleep(7);

				// Update ACM
				acm.setEntry("<octomap>", v_r, true);
				planning_scene.getPlanningSceneDiffMsg(planning_scene_diff_msg);
				planning_scene_diff_msg.is_diff = true;
				m_pub_scene.publish(planning_scene_diff_msg);
				sleep(2);
			
				// Close gripper
				ROS_INFO("--- Closing right gripper ---");
				set_gripper_right_close();
			
				// Attach object to robot
				ROS_INFO("Attach the object to the robot");
				//moveGroupPtr_r->attachObject("my_collision_object");

				// Move to home position
				set_arm_right_hello();
				//sleep(7);

				// Clear pointclouds
				clear_pointclouds();
		
				// Update ACM
				acm.setEntry("<octomap>", v_l, false);
				planning_scene.getPlanningSceneDiffMsg(planning_scene_diff_msg);
				planning_scene_diff_msg.is_diff = true;
				m_pub_scene.publish(planning_scene_diff_msg);
				sleep(2);

				// Detach object to robot
				ROS_INFO("Detach the object to the robot");
				//moveGroupPtr_r->detachObject("my_collision_object");

				// Open gripper
				//ROS_INFO("--- Opening right gripper ---");
				//set_gripper_right_open_old();
		
				// Kill all proceses
				ROS_INFO("Press CONTROL+C");
				//system("rosnode kill select_object_to_grasp");
				//system("kill $(ps aux | grep select_object_to_grasp | grep -v grep | awk '{print $2}')");
				//system("pkill select_object_to_grasp");
		
			}
		}
          
	}

	// Clear registered pcl
	diagnostic_msgs::KeyValue clear_regist_2;
	clear_regist_2.key = "Clear_registration";
	clear_regist_2.value = "clear";
	clear_regist_pub.publish(clear_regist_2);

	return;
                                                                        
}


void grasp_callback(const geometry_msgs::PoseStamped my_grasp)
{
	moveit::planning_interface::MoveGroup::Plan my_plan_r;
	moveit::planning_interface::MoveGroup::Plan my_plan_l;

  ROS_INFO("--------- Planning to grasp ----------");

	moveGroupPtr_r->setPoseTarget(my_grasp.pose);
  bool success_r = moveGroupPtr_r->plan(my_plan_r);

  if (success_r)
	{
		ROS_INFO("--------- Right arm moving ----------");
		
		moveGroupPtr_r->move();
		sleep(5);
      // mybot_action_node->grasp_by_right();
			// mybot_action_node->move_to_default_for_dual_arm();
			// mybot_action_node->release_by_right();
		
		/*	// Clear registered pcl
		diagnostic_msgs::KeyValue clear_regist;
		clear_regist.key = "Clear_registration";
		clear_regist.value = "clear";
		clear_regist_pub.publish(clear_regist);
		*/
	}
	else
	{
		moveGroupPtr_l->setPoseTarget(my_grasp.pose);
  	bool success_l = moveGroupPtr_l->plan(my_plan_l);
		if (success_l)
		{
			ROS_INFO("--------- Left arm moving ----------");
			
			moveGroupPtr_l->move();
			sleep(5);
      // mybot_action_node->grasp_by_left();
			// mybot_action_node->move_to_default_for_dual_arm();
			// mybot_action_node->release_by_left();

		/*	// Clear registered pcl
		diagnostic_msgs::KeyValue clear_regist;
		clear_regist.key = "Clear_registration";
		clear_regist.value = "clear";
		clear_regist_pub.publish(clear_regist);
		*/
		}
	}

	ROS_INFO("SALIDA");
       
}

void grasp_callback_test_right(const geometry_msgs::PoseStamped my_grasp)
{
	moveit::planning_interface::MoveGroup::Plan my_plan_r;

  ROS_INFO("--------- Planning to right grasp ----------");

	moveGroupPtr_r->setPoseTarget(my_grasp.pose);
	moveGroupPtr_r->setPoseReferenceFrame("base_footprint");
  bool success_r = moveGroupPtr_r->plan(my_plan_r);

	moveGroupPtr_r->move();
	sleep(5);
   
	ROS_INFO("--------- Done ----------");
}

void grasp_callback_test_left(const geometry_msgs::PoseStamped my_grasp)
{
	moveit::planning_interface::MoveGroup::Plan my_plan_l;

  ROS_INFO("--------- Planning to left grasp ----------");

	moveGroupPtr_l->setPoseTarget(my_grasp.pose);
	moveGroupPtr_l->setPoseReferenceFrame("map");
  bool success_l = moveGroupPtr_l->plan(my_plan_l);

	moveGroupPtr_l->move();
	sleep(5);

	ROS_INFO("--------- Done ----------");
   
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "move_arm_to_grasp");
  ros::NodeHandle nh;
	ros::AsyncSpinner spinner(4);
	spinner.start();

	// Get className from launcher
  nh.getParam("/move_arm_to_grasp/className", className);
	// Create topicNames
  std::string topicname_fga = "/" + className + "/final_grasp_array";
	std::string topicname_fg = "/" + className + "/final_grasp";
	std::string topicName_segm = "/pcl_results/"+ className;
  std::string topicName_registered = "/pcl_registered/"+ className;

	// ------------ Initialise MoveIt groups --------

  //moveGroupPtr_r = boost::shared_ptr<move_group_interface::MoveGroup>(new move_group_interface::MoveGroup("trunk_with_arm_right"));
	moveGroupPtr_r = boost::shared_ptr<move_group_interface::MoveGroup>(new move_group_interface::MoveGroup("arm_right")); 
  moveGroupPtr_r->setPlannerId("RRTConnectkConfigDefault");
  moveGroupPtr_r->setPlanningTime(5.0);
	moveGroupPtr_r->allowReplanning(true); 
	moveGroupPtr_r->setGoalTolerance(0.01);

	//moveGroupPtr_l = boost::shared_ptr<move_group_interface::MoveGroup>(new move_group_interface::MoveGroup("trunk_with_arm_left")); 
	moveGroupPtr_l = boost::shared_ptr<move_group_interface::MoveGroup>(new move_group_interface::MoveGroup("arm_left"));
  moveGroupPtr_l->setPlannerId("RRTConnectkConfigDefault");
	moveGroupPtr_l->setPlanningTime(5.0);
	moveGroupPtr_l->allowReplanning(true); 
  moveGroupPtr_l->setGoalTolerance(0.01);
/*
	//Set both arms to home position
	set_arm_right_home();
	set_arm_left_home();
*/

	moveGroupPtr_gr = boost::shared_ptr<move_group_interface::MoveGroup>(new move_group_interface::MoveGroup("gripper_right")); 
  moveGroupPtr_gr->setPlannerId("RRTConnectkConfigDefault");
  moveGroupPtr_gr->setPlanningTime(5.0);
	moveGroupPtr_gr->allowReplanning(true); 
	moveGroupPtr_gr->setGoalTolerance(0.01);

	moveGroupPtr_gl = boost::shared_ptr<move_group_interface::MoveGroup>(new move_group_interface::MoveGroup("gripper_left"));
  moveGroupPtr_gl->setPlannerId("RRTConnectkConfigDefault");
	moveGroupPtr_gl->setPlanningTime(5.0);
	moveGroupPtr_gl->allowReplanning(true); 
  moveGroupPtr_gl->setGoalTolerance(0.01); 

  
	// ---------- Initialise Robot loader --------

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	kinematic_model = robot_model_loader.getModel();
	
	 m_service_get_planning_scene = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
	 m_pub_scene = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

	// ------------ Publishers --------
	clear_regist_pub = nh.advertise<diagnostic_msgs::KeyValue>("/clear_registered_pcl", 1);
	m_pub_gripper_right = nh.advertise<trajectory_msgs::JointTrajectory>("/gripper_right_controller/command", 1);
	m_pub_gripper_left = nh.advertise<trajectory_msgs::JointTrajectory>("/gripper_left_controller/command", 1);
	m_pub_arm_right = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_right_controller/command", 100);
	m_pub_arm_left = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_left_controller/command", 100);
	reg_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(topicName_registered, 1);
	segm_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(topicName_segm, 1);

	// --------- Open hands before everything else -----
	
	ROS_INFO("--- Opening both grippers ---");
	set_gripper_right_open_old();
	set_gripper_left_open_old();
	
 
	// ------------ Subscribers --------
  
	ros::Subscriber grasp_array_sub = nh.subscribe(topicname_fga, 1, grasp_array_callback);
	ros::Subscriber grasp_sub = nh.subscribe(topicname_fg, 1, grasp_callback_test_left);

	
	ros::waitForShutdown();
  return 0;

}


