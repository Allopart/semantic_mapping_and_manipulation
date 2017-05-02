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
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
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
ros::Publisher desk_pub;


std::string list_region ("person bird cat dog horse sheep cow elephant bear zebra giraffe backpack handbag");


void segmentation_callback(const semantic_mapping::LabelledPointCloud2::ConstPtr& pcl_ptr)
{

	// Get mask data
	std::string className = pcl_ptr->label;
  // Remove spaces from className
  className.erase (std::remove (className.begin(), className.end(), ' '), className.end());

	// Check lists to see what clustering method to use
	std::size_t found_region = list_region.find(className);;

	
	// *************** Planar_all segmentation ************** 
	
	if (found_region!=std::string::npos) {
		//ROS_INFO("%s - Region Growth \n", className.c_str());

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
		pcl::IndicesPtr indices (new std::vector <int>);
		pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
		pass.setInputCloud (cloud_xyz);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.1, 5.0);
		pass.filter (*cloud_filtered);
		pass.filter (*indices);

	 	
		if (cloud_filtered->points.size() < 5) {
			ROS_INFO("Object too far");
			return;
		}

		// ------- PLANAR SEGMENTATION ----------
		// Initialize planar segmentation 
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new     pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier (new pcl::PointCloud<pcl::PointXYZ> ());

		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.04);
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);

		// Extract the planar inliers and outliers
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
	  extract.setNegative (false);
		extract.filter (*cloud_plane);
		extract.setNegative (true);
		extract.filter (*cloud_outlier);


		// ------- REGION GROWTH ----------
		pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod (tree);
		normal_estimator.setInputCloud (cloud_outlier);
		normal_estimator.setKSearch (70);
		normal_estimator.compute (*normals);


		pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
		reg.setMinClusterSize (150);
		reg.setMaxClusterSize (1000000);
		reg.setSearchMethod (tree);
		reg.setNumberOfNeighbours (100);
		reg.setInputCloud (cloud_outlier);
		//reg.setIndices (indices);
		reg.setInputNormals (normals);
		reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
		reg.setCurvatureThreshold (1.0);

		std::vector <pcl::PointIndices> clusters;
		reg.extract (clusters);


		//Select biggest cluster 
		int biggest_itr;
		int cluster_size_biggest = 0;
		for (int i = 0; i<=clusters.size (); i++)
		{
			int cluster_size = clusters[i].indices.size ();
			if (cluster_size > cluster_size_biggest)
			{
				cluster_size_biggest=cluster_size;
				biggest_itr = i;
			}
		}


		// Convert to Pointcloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
					cloud_cluster->clear();
		for (std::vector<int>::const_iterator pit = clusters[biggest_itr].indices.begin (); pit != clusters[biggest_itr].indices.end (); pit++)
		{
			cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
		  cloud_cluster->width = cloud_cluster->points.size ();
		  cloud_cluster->height = 1;
		  cloud_cluster->is_dense = true;
		}

/*
		std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
		std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
		std::cout << "These are the indices of the points of the initial" <<
		  std::endl << "cloud that belong to the first cluster:" << std::endl;
		int counter = 0;
		while (counter < 5 || counter > clusters[0].indices.size ())
		{
		  std::cout << clusters[0].indices[counter] << std::endl;
		  counter++;
		}
*/

/*	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
		colored_cloud->header.frame_id = "xtion_rgb_optical_frame";
    ros::Time time_st = ros::Time::now ();
		colored_cloud->header.stamp = time_st.toNSec()/1e3;
  	sensor_msgs::PointCloud2 pcl_help;
    pcl::toROSMsg(*colored_cloud,pcl_help);
		pcl_pub_result.publish(pcl_help);
*/

		
		// Publish plane inliers
		cloud_plane->header.frame_id = "xtion_rgb_optical_frame";
		ros::Time time_st = ros::Time::now ();
		cloud_plane->header.stamp = time_st.toNSec()/1e3;
		desk_pub.publish (cloud_plane);

		// Publish region growth
		cloud_cluster->header.frame_id = "xtion_rgb_optical_frame";
		cloud_cluster->header.stamp = time_st.toNSec()/1e3;
  	sensor_msgs::PointCloud2 pcl_final;
    pcl::toROSMsg(*cloud_cluster,pcl_final);
		pcl_pub_result.publish(pcl_final);

		// Publish labelled pointcloud
		semantic_mapping::LabelledPointCloud2 lab_pcl;
		lab_pcl.pcl=pcl_final;
		lab_pcl.label=className;
		labpcl_seg_pub.publish (lab_pcl);


		return;
	}

}


int main(int argc, char** argv)
{

  sleep(15);
  ros::init(argc, argv, "class_lasbpcl_seg_region");

  ros::NodeHandle nh;
  nhPtr = & nh; // Give global access to this nodeHandle

// ------------ Subscribers --------

  ros::Subscriber pcl_sub = nh.subscribe("/class/lab_pcl/full", 1, segmentation_callback);

// ------------ Publishers -----------

  labpcl_seg_pub = nh.advertise<semantic_mapping::LabelledPointCloud2>("/class/lab_pcl/segmented", 1);
	desk_pub = nh.advertise<sensor_msgs::PointCloud2>("/class/desk", 1);
	



  ros::spin ();
  return 0;

}
