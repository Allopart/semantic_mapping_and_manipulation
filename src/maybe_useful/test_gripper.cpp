#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <boost/bind.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include "std_msgs/String.h"

#include <sstream>
#include <iostream>
#include <cmath> 

ros::Publisher m_pub_gripper_right;
ros::Publisher m_pub_gripper_left;

using namespace std;


void right_gripper_close(){

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

	m_pub_gripper_right.publish(msg);
	ROS_INFO("--- Closing right gripper ---");
	sleep(1);
	return;
}

void right_gripper_open(){

	std::vector<std::string> names(3);
	names[0] = "right_gripper_thumb_joint_0";
  names[1] = "right_gripper_forefinger_joint_0";
  names[2] = "right_gripper_littlefinger_joint_0";

	std::vector<double> angles(3,0);
	angles[0] = -25.0/180.0*3.14;
  angles[1] = -25.0/180.0*3.14;
  angles[2] = +25.0/180.0*3.14;

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

	m_pub_gripper_right.publish(msg);
	ROS_INFO("--- Opening right gripper ---");
	sleep(1);
	return;
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "test_gripper");
  ros::NodeHandle nh;
	//ros::AsyncSpinner spinner(4);
	//spinner.start();


	// ------------ Publishers --------
	m_pub_gripper_right = nh.advertise<trajectory_msgs::JointTrajectory>("/gripper_right_controller/command", 1);
	m_pub_gripper_left = nh.advertise<trajectory_msgs::JointTrajectory>("/gripper_left_controller/command", 1);

//while(ros::ok()){
for (int i =0;i<2;i++)	{
	right_gripper_open();
	//right_gripper_close();

// ros::spinOnce();
}
	//ros::waitForShutdown();
	//ros::spinOnce();  
	return 0;

}
