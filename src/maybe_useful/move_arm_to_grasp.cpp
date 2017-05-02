#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <boost/bind.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <boost/bind.hpp>

#include <iostream>
#include <cmath> 

boost::shared_ptr<move_group_interface::MoveGroup> moveGroupPtr_r; 
boost::shared_ptr<move_group_interface::MoveGroup> moveGroupPtr_l;


void grasp_array_callback(const geometry_msgs::PoseArray my_grasp_array)
{
	moveit::planning_interface::MoveGroup::Plan my_plan_r;
	moveit::planning_interface::MoveGroup::Plan my_plan_l;

  ROS_INFO("--------- Planning to grasp ----------");

	moveGroupPtr_r->setPoseTargets(my_grasp_array.poses);
  bool success_r = moveGroupPtr_r->plan(my_plan_r);

  if (success_r)
	{
		ROS_INFO("--------- Right arm moving ----------");
		moveGroupPtr_r->move();
		// mybot_action_node->move_to_default_for_dual_arm();
	}
	else
	{
		moveGroupPtr_l->setPoseTargets(my_grasp_array.poses);
  	bool success_l = moveGroupPtr_l->plan(my_plan_l);

		if (success_l)
		{
			ROS_INFO("--------- Left arm moving ----------");
			moveGroupPtr_l->move();
			// mybot_action_node->move_to_default_for_dual_arm();
		}
	}
	//Sleep to give Rviz time to visualize the plan.
	sleep(2.0);

                                                                                    
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "move_arm_to_grasp");
  ros::NodeHandle nh;
	ros::AsyncSpinner spinner(4);
	spinner.start();

	// ------------ Initialise MoveIt groups --------

  moveGroupPtr_r = boost::shared_ptr<move_group_interface::MoveGroup>(new move_group_interface::MoveGroup("arm_right")); 
  moveGroupPtr_r->setPlannerId("arm_right[RRTConnectkConfigDefault]"); 
  moveGroupPtr_r->setPlanningTime(3.); 

	moveGroupPtr_l = boost::shared_ptr<move_group_interface::MoveGroup>(new move_group_interface::MoveGroup("arm_left")); 
  moveGroupPtr_l->setPlannerId("arm_left[RRTConnectkConfigDefault]"); 
  moveGroupPtr_l->setPlanningTime(3.); 
  
	// ------------ Initialise MoveIt planning scene --------

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	// ---- Disable collision between fingers and octomap ----
/*	
	collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
	planning_scene_interface.getAllowedCollisionMatrixNonConst().setEntry("<octomap>", right_gripper_littlefinger_1, true);
	planning_scene_interface.getAllowedCollisionMatrixNonConst().setEntry("<octomap>", left_gripper_littlefinger_1, true);
*/

  
	// ------------ Subscribers --------
  ros::Subscriber grasp_sub = nh.subscribe("/class/final_grasp_array", 1, grasp_array_callback);

	ros::waitForShutdown();
  //ros::spin ();
  return 0;

}
