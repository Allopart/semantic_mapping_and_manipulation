#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/surface/concave_hull.h>
#include <sensor_msgs/PointCloud2.h>
#include <diagnostic_msgs/KeyValue.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/bind.hpp>
#include <iostream>
#include <string>
#include <sstream>

#include "semantic_mapping/LabelledPointCloud2.h"
#include "semantic_mapping/BoundingBox_data.h"

using namespace message_filters;
using namespace std;
using namespace pcl;

typedef pcl::PointXYZ PointT;

std::string str_clear ("clear");

ros::NodeHandle* nhPtr; // Give global access to the nodeHandle

ros::Publisher pcl_pub;
ros::Publisher clear_regist_pub;
ros::Publisher BB_data_pub;
ros::Subscriber new_pcl_sub;
ros::Subscriber base_pcl_sub;
ros::Subscriber clear_regist_sub;

sensor_msgs::PointCloud2 base_pcl; // Global declaration of base pcl message

int cloud_counter; //Check if its the first cloud or not


void clear_pcl_callback(const diagnostic_msgs::KeyValue::ConstPtr clear_msg)
{
	// Get clear status
	std::string clear_or_reg = clear_msg->value;
	// Remove spaces from clear_or_reg
  clear_or_reg.erase (std::remove (clear_or_reg.begin(), clear_or_reg.end(), ' '), clear_or_reg.end());

  //Check if match found
	std::size_t found_clear = str_clear.find(clear_or_reg);

	if (found_clear!=std::string::npos)
	{
		ROS_INFO("---- Clearing registered pointcloud -----");
		cloud_counter=0;
		diagnostic_msgs::KeyValue regist;
		regist.key = "Clear_registration";
		regist.value = "register";
		clear_regist_pub.publish(regist);
	}
	else {
		//ROS_INFO("---- Failed clearing pointcloud -----");
	}
}

void get_base_callback(const sensor_msgs::PointCloud2 get_base_pcl)
{
	base_pcl = get_base_pcl;
}

void icp_callback(const sensor_msgs::PointCloud2 pcl_msg)
{

  sensor_msgs::PointCloud2 new_pcl;
  new_pcl = pcl_msg;

  cloud_counter++;
  if (cloud_counter==1) 
  { 
    // Create class-specific publisher
    ROS_INFO("First publish to base_pcl");
    //new_pcl.header.frame_id = std::string("/xtion_rgb_optical_frame");
	  pcl_pub.publish(new_pcl); 
  } 
  else 
  { 

		// Convert to PCL type
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud_Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(new_pcl, *new_cloud_Ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud_Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(base_pcl, *base_cloud_Ptr);

		ROS_INFO("ICP registration");


	 	// Perform ICP
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(base_cloud_Ptr);
		icp.setInputTarget(new_cloud_Ptr);

		icp.setMaximumIterations(50);
		icp.setTransformationEpsilon(1e-12);
		icp.setEuclideanFitnessEpsilon(0.1);
		
		pcl::PointCloud<pcl::PointXYZ> cloud_aligned;
		icp.align(cloud_aligned);

    if (icp.hasConverged () && icp.getFitnessScore()<1.0e-3)
  	{
		  // Merge base cloud and aligned cloud
			pcl::PointCloud<pcl::PointXYZ> cloud_base_aligned;
			cloud_base_aligned = *base_cloud_Ptr;
			cloud_base_aligned += cloud_aligned;

			// Publish results
			sensor_msgs::PointCloud2 output;
      pcl::toROSMsg (cloud_base_aligned, output);
      //output.header.frame_id = std::string("/xtion_rgb_optical_frame");
			pcl_pub.publish(output); 


			// ------- Get Bounding box around registered pointcloud -------
			pcl:: PointCloud<PointXYZ>::Ptr BB_ptr (new pcl::PointCloud<PointXYZ> (cloud_base_aligned	));

			// Compute principal direction
			Eigen::Vector4f centroid;
			pcl::compute3DCentroid(*BB_ptr, centroid);
			Eigen::Matrix3f covariance;
			computeCovarianceMatrixNormalized(*BB_ptr, centroid, covariance);
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
			Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
			eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

			// move the points to the that reference frame
			Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
			p2w.block<3,3>(0,0) = eigDx.transpose();
			p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
			pcl::PointCloud<PointT> cPoints;
			pcl::transformPointCloud(*BB_ptr, cPoints, p2w);

			PointT min_pt, max_pt;
			pcl::getMinMax3D(cPoints, min_pt, max_pt);
			const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

			// final transform
			const Eigen::Quaternionf qfinal(eigDx);
			const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

			//Publish BB data
			semantic_mapping::BoundingBox_data BB_data;
			BB_data.header.stamp = output.header.stamp;
			BB_data.header.frame_id = "xtion_rgb_optical_frame";
			BB_data.size.x=max_pt.x - min_pt.x;
			BB_data.size.y=max_pt.y - min_pt.y;
			BB_data.size.z=max_pt.z - min_pt.z;
			BB_data.position.x =  tfinal[0];
			BB_data.position.y =  tfinal[1];
			BB_data.position.z =  tfinal[2];
			BB_data.orientation.x = qfinal.x();
			BB_data.orientation.y = qfinal.y();
			BB_data.orientation.z = qfinal.z();
			BB_data.orientation.w = qfinal.w();
			BB_data_pub.publish(BB_data);

			sleep(0.2);
		}
		else
		{
		  PCL_ERROR ("\nICP has not converged. Fitness score %f\n", icp.getFitnessScore());
		  ROS_INFO("Re-publish to base_pcl");
      //new_pcl.header.frame_id = std::string("/xtion_rgb_optical_frame");
	  	pcl_pub.publish(new_pcl);
      cloud_counter=0;
		}

	}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_matching");

  ros::NodeHandle nh;
	nhPtr = &nh; // Give global access to this nodeHandle

	sleep(12);

  int cloud_counter=0; // Initialise cloud counter

  // Get className from launcher
  std::string className;
  nh.getParam("/ICP_registration/className", className);
 
  // Create topicNames
  std::string topicName_subs_new_pcl = "/pcl_results/"+ className;
  std::string topicName_registered = "/pcl_registered/"+ className;
	std::string topicName_BB = "/" + className + "/BB_data";

  // Subsribe to new_pcl
  new_pcl_sub = nh.subscribe(topicName_subs_new_pcl, 1, icp_callback);
  // Subsribe to base_pcl
	base_pcl_sub = nh.subscribe(topicName_registered, 1, get_base_callback);
	// Subsribe to clear_registered_pcl
	clear_regist_sub = nh.subscribe("/clear_registered_pcl", 1, clear_pcl_callback);


  // Create publisher
  pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(topicName_registered, 1);
	clear_regist_pub = nh.advertise<diagnostic_msgs::KeyValue>("/clear_registered_pcl", 1);
	BB_data_pub = nh.advertise<semantic_mapping::BoundingBox_data>(topicName_BB, 1);


  ros::spin ();
  return 0;

}
