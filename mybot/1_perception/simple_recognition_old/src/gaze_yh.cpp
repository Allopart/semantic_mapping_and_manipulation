#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "gaze_yh"); 
  ros::AsyncSpinner spinner(0); 
  spinner.start(); 
  ros::NodeHandle node_handle("~"); 

  ros::Publisher m_pub_head = node_handle.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 1); 

  std::vector<std::string> names(4); 
  names[0] = "head_joint_0"; 
  names[1] = "head_joint_1"; 
  names[2] = "head_joint_2"; 
  names[3] = "head_joint_3"; 

  std::vector<double> angles(4, 0.0); 
  
  while(true) {

          std::cout << "Input angle: " << std::endl; 
          std::cout << "Angle 1: "; 
          std::cin >> angles[0];   
          std::cout << "Angle 2: "; 
          std::cin >> angles[1];   
          std::cout << "Angle 3: "; 
          std::cin >> angles[2];   
          std::cout << "Angle 4: "; 
          std::cin >> angles[3];   

          trajectory_msgs::JointTrajectory msg; 

          msg.header.stamp = ros::Time::now()+ros::Duration(0.1); 
          msg.header.frame_id = "base_footprint"; 
          msg.points.resize(1); 

          for (std::size_t i=0; i<names.size(); i++) {
                  msg.joint_names.push_back(names[i]); 
                  msg.points[0].positions.push_back(angles[i]*3.14/180.); 
          }

          msg.points[0].velocities.resize(names.size(), 0); 
          msg.points[0].accelerations.resize(names.size(), 0); 
          msg.points[0].effort.resize(names.size(), 0); 
          msg.points[0].time_from_start = ros::Duration(0.1); 


          m_pub_head.publish(msg); 

  }

  ros::waitForShutdown(); 
  return 0; 
}
