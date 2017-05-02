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
ros::Publisher desk_pub;

std::string list_plane_outlier ("mouse bowl firehydrant wineglass fork knife spoon banana sandwich carrot hotdog  donut cake pottedplant scissors remote teddybear hairdrier keyboard umbrella cellphone");

void segmentation_callback(const semantic_mapping::LabelledPointCloud2::ConstPtr& pcl_ptr)
{

	// Get mask data
	std::string className = pcl_ptr->label;
  // Remove spaces from className
  className.erase (std::remove (className.begin(), className.end(), ' '), className.end());

	// Check lists to see what clustering method to use
	std::size_t found_plane_outlier = list_plane_outlier.find(className);

	// *************** Planar_outlier segmentation ************** 
	
	if (found_plane_outlier!=std::string::npos) {

		//ROS_INFO("%s - Outlier\n", className.c_str());	

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
		pass.setFilterLimits (0.1, 10.0);
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

		// Initialize planar segmentation 
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new     pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier (new pcl::PointCloud<pcl::PointXYZ> ());

		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); 
		seg.setAxis(axis); 
		//seg.setEpsAngle((2*3.14)/180); 
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.003);
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      ROS_INFO ("Could not estimate a planar model for the given dataset.");
			/*
      pcl:: PointCloud<PointXYZ>::Ptr outlier_cheat_ptr (new pcl::PointCloud<PointXYZ> (*cloud_filtered));
  		
      // Publish all data as outliers
			outlier_cheat_ptr->header.frame_id = "xtion_rgb_optical_frame";
			ros::Time time_st = ros::Time::now ();
			outlier_cheat_ptr->header.stamp = time_st.toNSec()/1e3;
			pcl_pub_result.publish (outlier_cheat_ptr);
      

      // Publish labelled pointcloud
			semantic_mapping::LabelledPointCloud2 lab_pcl;
			sensor_msgs::PointCloud2 pcl_help;
			pcl::toROSMsg(*outlier_cheat_ptr,pcl_help);
			lab_pcl.pcl=pcl_help;
			lab_pcl.label=className;
			labpcl_seg_pub.publish (lab_pcl);
			*/
    }
    else {

			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud (cloud_filtered);
			extract.setIndices (inliers);
		  extract.setNegative (false);
			extract.filter (*cloud_plane);

			// Remove the planar inliers, extract the rest
			extract.setNegative (true);
			extract.filter (*cloud_outlier);
			if (cloud_outlier->points.size()==0){ROS_INFO("Returning, outlier inexistent: %s", className.c_str()); return;}
		  
			// Remove outliers below plane
			float Rx = coefficients->values[0];
			float Ry = coefficients->values[1];
			float Rz = coefficients->values[2];
			float Rd = coefficients->values[3];
		
			
			if (Rd>0.0){ ROS_INFO("Returning, Rd failed: %s",className.c_str()); return; } // Or else we will get stuff below plane instead of above

			pcl::PointCloud<pcl::PointXYZ> good_outliers; 
			
			//ROS_INFO("Rx: %f, Ry: %f, Rz: %f, Rd: %f",Rx,Ry,Rz,Rd);

			for (int i=0; i <= cloud_outlier->points.size() ; i++ ) {
				float Px = cloud_outlier->points[i].x;
				float Py = cloud_outlier->points[i].y;
				float Pz = cloud_outlier->points[i].z;

				float result = Px*Rx + Py*Ry + Pz*Rz + Rd;
				//ROS_INFO("Result %d: %f", i, result);
				if (result < 0.0)  {
					good_outliers.push_back(cloud_outlier->points[i]);    	
				}
			} 
			
			pcl:: PointCloud<PointXYZ>::Ptr good_outliers_ptr (new pcl::PointCloud<PointXYZ> (good_outliers)); 

			// Build a passthrough filter to remove spurious NaNs
			pcl::PassThrough<PointT> pass_out;
			pcl::PointCloud<PointT>::Ptr good_outliers_f_ptr (new pcl::PointCloud<PointT>);
			pass_out.setInputCloud (good_outliers_ptr);
			pass_out.setFilterFieldName ("z");
			pass_out.setFilterLimits (0.1, 10.0);
			pass_out.filter (*good_outliers_f_ptr);


		  if (good_outliers.size () == 0)
		  {
		    ROS_INFO ("No good outliers");
		    /*
		    // Publish the plane data as outliers
				cloud_plane->header.frame_id = "xtion_rgb_optical_frame";
				ros::Time time_st = ros::Time::now ();
				cloud_plane->header.stamp = time_st.toNSec()/1e3;
				pcl_pub_result.publish (cloud_plane);
        ROS_INFO("%s - Outlier\n", className.c_str());

		    // Publish labelled pointcloud
				semantic_mapping::LabelledPointCloud2 lab_pcl;
				sensor_msgs::PointCloud2 pcl_help;
				pcl::toROSMsg(*cloud_plane,pcl_help);
				lab_pcl.pcl=pcl_help;
				lab_pcl.label=className;
				labpcl_seg_pub.publish (lab_pcl);
				*/
		  }
			else {
		
				// Publish the outlier data (objects)
				good_outliers_f_ptr->header.frame_id = "xtion_rgb_optical_frame";
				ros::Time time_st = ros::Time::now ();
				good_outliers_f_ptr->header.stamp = time_st.toNSec()/1e3;
				pcl_pub_result.publish (good_outliers_f_ptr);
        //ROS_INFO("%s - Outlier\n", className.c_str());

		    // Publish labelled pointcloud
				semantic_mapping::LabelledPointCloud2 lab_pcl;
				sensor_msgs::PointCloud2 pcl_help;
				pcl::toROSMsg(*good_outliers_ptr,pcl_help);
				lab_pcl.pcl=pcl_help;
				lab_pcl.label=className;
				labpcl_seg_pub.publish (lab_pcl);

				// Publish plane inliers
				cloud_plane->header.frame_id = "xtion_rgb_optical_frame";
				cloud_plane->header.stamp = time_st.toNSec()/1e3;
				desk_pub.publish (cloud_plane);

		  } //End else if cloud outliers has points

		}//End else if cloud inlier has 0 points

		return;    
	} // END OF PLANAR_OUTLIER SEGMENTATION

}


int main(int argc, char** argv)
{

  sleep(15);
  ros::init(argc, argv, "class_lasbpcl_seg_outlier");

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
