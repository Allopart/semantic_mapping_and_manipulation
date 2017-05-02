#include <ros/callback_queue.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <boost/bind.hpp>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PolygonStamped.h>
#include <vector>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cmath> 
#include <stdio.h>
#include <list>
#include <iterator>
#include <string>


#include "dn_object_detect/DetectedObjects.h"
#include "semantic_mapping/LabelledPointCloud2.h"

using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;
using namespace pcl;
using namespace cv;
using namespace std;

typedef pcl::PointXYZ PointT;

ros::NodeHandle* nhPtr; // Give global access to the nodeHandle
ros::Publisher pcl_pub_result;
ros::Publisher labpcl_seg_pub;

std::string list_cylinder ("cup bottle baseballbat vase");

void segmentation_callback(const semantic_mapping::LabelledPointCloud2::ConstPtr& pcl_ptr)
{

	// Get mask data
	std::string className = pcl_ptr->label;
  // Remove spaces from className
  className.erase (std::remove (className.begin(), className.end(), ' '), className.end());

		// Check lists to see what clustering method to use
	std::size_t found_cylinder = list_cylinder.find(className);

	// *************** Cylinder segmentation ************** 
	
	if (found_cylinder!=std::string::npos) {

		//ROS_INFO("%s - Cylindrical\n", className.c_str());

		// Create class-specific publisher
		std::string topicName_result = "/pcl_results/"+ className;
		pcl_pub_result = nhPtr->advertise<sensor_msgs::PointCloud2>(topicName_result, 1);

		sensor_msgs::PointCloud2 pcl_helper;
		pcl_helper = pcl_ptr->pcl;

		// Convert to XYZ format
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::moveFromROSMsg(pcl_helper, *cloud_xyz);

		// Build a passthrough filter to remove spurious NaNs
		pcl::PassThrough<PointT> pass;
		pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
		pass.setInputCloud (cloud_xyz);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.1, 5.0);
		pass.filter (*cloud_filtered);
		/*pass.setInputCloud (cloud_filtered);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (0, 2);
		pass.filter (*cloud_filtered);*/
	//				std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;
	 	
		if (cloud_filtered->points.size() < 5) {
			ROS_INFO("Object too far");
			return;
		}

		
		// All the objects needed
		pcl::NormalEstimation<PointT, pcl::Normal> ne;
		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
	pcl::ExtractIndices<PointT> extract;
		pcl::ExtractIndices<pcl::Normal> extract_normals;
		pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

	  // Datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
		pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

		// Estimate point normals
		ne.setSearchMethod (tree);
		ne.setInputCloud (cloud_filtered);
		ne.setKSearch (50);
		ne.compute (*cloud_normals);

		// Create the segmentation object for the planar model and set all the parameters
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
		seg.setNormalDistanceWeight (0.1);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.03);
		seg.setInputCloud (cloud_filtered);
		seg.setInputNormals (cloud_normals);
		// Obtain the plane inliers and coefficients
		seg.segment (*inliers_plane, *coefficients_plane);
//				std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

		// Extract the planar inliers from the input cloud
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers_plane);
		extract.setNegative (false);

		// Write the planar inliers to disk
		pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
		extract.filter (*cloud_plane);
//				std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_filtered2);
		extract_normals.setNegative (true);
		extract_normals.setInputCloud (cloud_normals);
		extract_normals.setIndices (inliers_plane);
		extract_normals.filter (*cloud_normals2);

		// Create the segmentation object for cylinder segmentation and set all the parameters
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_CYLINDER);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setNormalDistanceWeight (0.1);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.05);
		seg.setRadiusLimits (0, 0.05);
		seg.setInputCloud (cloud_filtered2);
		seg.setInputNormals (cloud_normals2);

		// Obtain the cylinder inliers and coefficients
		seg.segment (*inliers_cylinder, *coefficients_cylinder);
//				std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

		// Write the cylinder inliers to disk
		extract.setInputCloud (cloud_filtered2);
		extract.setIndices (inliers_cylinder);
		extract.setNegative (false);
		pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
		extract.filter (*cloud_cylinder);
/*				if (cloud_cylinder->points.empty ()) 
			std::cerr << "Can't find the cylindrical component." << std::endl;
		else
		{
			std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
		}
*/
		// Publish the plane data 
		cloud_cylinder->header.frame_id = "xtion_rgb_optical_frame";
    ros::Time time_st = ros::Time::now ();
		cloud_cylinder->header.stamp = time_st.toNSec()/1e3;
		pcl_pub_result.publish (cloud_cylinder);
    

    // Publish labelled pointcloud
    semantic_mapping::LabelledPointCloud2 lab_pcl;
    sensor_msgs::PointCloud2 pcl_help;
    pcl::toROSMsg(*cloud_cylinder,pcl_help);
    lab_pcl.pcl=pcl_help;
    lab_pcl.label=className;
		labpcl_seg_pub.publish (lab_pcl);

		return;
	} // END OF CYLINDER SEGMENTATION

}


int main(int argc, char** argv)
{

  sleep(15);
  ros::init(argc, argv, "class_lasbpcl_seg_h");

  ros::NodeHandle nh;
  nhPtr = & nh; // Give global access to this nodeHandle

// ------------ Subscribers --------

  ros::Subscriber pcl_sub = nh.subscribe("/class/lab_pcl/full", 1, segmentation_callback);

// ------------ Publishers -----------

  labpcl_seg_pub = nh.advertise<semantic_mapping::LabelledPointCloud2>("/class/lab_pcl/segmented", 1);


  ros::spin ();
  return 0;

}
