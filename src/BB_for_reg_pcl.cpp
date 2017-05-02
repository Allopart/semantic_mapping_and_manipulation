#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <sensor_msgs/PointCloud2.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <eigen_conversions/eigen_msg.h>

#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <iostream>
#include <string>
#include <sstream>

#include "semantic_mapping/BoundingBox_data.h"

using namespace pcl;

typedef pcl::PointXYZ PointT;

// Define links that are not checked for collsiion with octomap
const char* args_r[] = {"right_gripper_forefinger_1", "right_gripper_littlefinger_1", "right_gripper_thumb_1", "right_gripper_parm_0", "right_gripper_parm_1", "right_gripper_parm_2", "<octomap>"};
std::vector<std::string> v_r(args_r, args_r + 7);


ros::NodeHandle* nhPtr; // Give global access to the nodeHandle

ros::Publisher planning_scene_diff_publisher;
boost::shared_ptr<move_group_interface::MoveGroup> moveGroupPtr_r;
robot_model::RobotModelPtr kinematic_model;

ros::Subscriber BB_sub;


void collision_object_callback(const semantic_mapping::BoundingBox_dataConstPtr& BB_data)
{

	//ROS_INFO(" ---------- Adding collision object -------------");
	
	// -------------- Generate Collision Object ------------------
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = "xtion_rgb_optical_frame";

	// The id of the object is used to identify it. 
	collision_object.id = "my_collision_object";


	// Define a box to add to the world. 
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = BB_data->size.x;
	primitive.dimensions[1] = BB_data->size.y;
	primitive.dimensions[2] = BB_data->size.z;

	// A pose for the box (specified relative to frame_id) 
	geometry_msgs::Pose box_pose;
	box_pose.position.x =  BB_data->position.x;
	box_pose.position.y =  BB_data->position.y;
	box_pose.position.z =  BB_data->position.z;
  box_pose.orientation.x = BB_data->orientation.x;
	box_pose.orientation.y = BB_data->orientation.y;
	box_pose.orientation.z = BB_data->orientation.z;
	box_pose.orientation.w = BB_data->orientation.w;


	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);

	// -------- Add Collision Object to the environment -----------
	moveit_msgs::PlanningScene p_s;
	p_s.world.collision_objects.push_back(collision_object);
	p_s.is_diff = true;
	planning_scene_diff_publisher.publish(p_s);
  sleep(0.5);


}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "Bounding_box");

  ros::NodeHandle nh;
	nhPtr = &nh; // Give global access to this nodeHandle

	// Get className from launcher
  std::string className;
  nh.getParam("/BB_for_reg_pcl/className", className);
 
  // Create topicNames
  std::string topicName_BB = "/" + className + "/BB_data";

	// ---------- Initialise RobotModel --------

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	kinematic_model = robot_model_loader.getModel();
	

	// ---------- Initialise MoveIt planning scene Interface --------
	
	ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
		moveit_msgs::DisplayTrajectory display_trajectory;
		planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

	// Subsribe to clear_registered_pcl
	BB_sub = nh.subscribe(topicName_BB, 1, collision_object_callback);

  ros::spin ();
  return 0;

}


