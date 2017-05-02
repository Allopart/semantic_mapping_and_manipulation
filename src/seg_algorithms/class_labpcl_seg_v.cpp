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
#include <pcl/surface/convex_hull.h>
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
ros::Publisher convex_hull_pub;
ros::Publisher handle_pub;

geometry_msgs::PolygonStamped poly;

std::string list_plane_v ("clock stopsign tv laptop microwave oven refrigerator");

void segmentation_callback(const semantic_mapping::LabelledPointCloud2::ConstPtr& pcl_ptr)
{

	// Get mask data
	std::string className = pcl_ptr->label;
  // Remove spaces from className
  className.erase (std::remove (className.begin(), className.end(), ' '), className.end());

	// Check lists to see what clustering method to use
	std::size_t found_plane_v = list_plane_v.find(className);

  // *************** Planar_V segmentation ************** 
	
	if (found_plane_v!=std::string::npos) {

		//ROS_INFO("%s - Planar_v\n", className.c_str());

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
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_v (new     pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier (new pcl::PointCloud<pcl::PointXYZ> ());

		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		Eigen::Vector3f axis = Eigen::Vector3f(1.0,1.0,0.0); 
		seg.setAxis(axis); 
    seg.setEpsAngle((50*3.14)/180);
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.01);
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);
	  extract.filter (*cloud_plane_v);

    // Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_outlier);

		// Generate concave/convex hull
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
 		pcl::ConvexHull<pcl::PointXYZ> chull;
  	chull.setInputCloud (cloud_plane_v);
    //chull.setAlpha (0.1);
  	chull.reconstruct (*cloud_hull);

    // If more than 3 points are present, send a PolygonStamped hull too
		if (cloud_hull->points.size () >= 3)
		{

		  poly.polygon.points.resize (cloud_hull->points.size ());
		  // Get three consecutive points (without copying)
		  pcl::Vector4fMap O = cloud_hull->points[1].getVector4fMap ();
		  pcl::Vector4fMap B = cloud_hull->points[0].getVector4fMap ();
		  pcl::Vector4fMap A = cloud_hull->points[2].getVector4fMap ();
		  // Check the direction of points -- polygon must have CCW direction
		  Eigen::Vector4f OA = A - O;
		  Eigen::Vector4f OB = B - O;
		  Eigen::Vector4f N = OA.cross3 (OB);
		  double theta = N.dot (O);
		  bool reversed = false;
		  if (theta < (M_PI / 2.0))
		    reversed = true;
		  for (size_t i = 0; i < cloud_hull->points.size (); ++i)
		  {
		    if (reversed)
		    {
		      size_t j = cloud_hull->points.size () - i - 1;
		      poly.polygon.points[i].x = cloud_hull->points[j].x;
		      poly.polygon.points[i].y = cloud_hull->points[j].y;
		      poly.polygon.points[i].z = cloud_hull->points[j].z;
		    }
		    else
		    {
		      poly.polygon.points[i].x = cloud_hull->points[i].x;
		      poly.polygon.points[i].y = cloud_hull->points[i].y;
		      poly.polygon.points[i].z = cloud_hull->points[i].z;
		    }
		  }
		}

    if (cloud_outlier->points.size() > 0)
		{
		  // Remove outliers behind plane (focus on handles)
			float Rx = coefficients->values[0];
			float Ry = coefficients->values[1];
			float Rz = coefficients->values[2];
			float Rd = coefficients->values[3];
		 
			
			if (Rd>0.0){ 
				 //ROS_INFO("HERE");
				 return;
		  } // Or else we will get stuff below plane instead of above

			pcl::PointCloud<pcl::PointXYZ> cloud_handles; 

			for (int i=0; i <= cloud_outlier->points.size() ; i++ ) {

				float Px = cloud_outlier->points[i].x;
				float Py = cloud_outlier->points[i].y;
				float Pz = cloud_outlier->points[i].z;

				float result = Px*Rx + Py*Ry + Pz*Rz + Rd;


//				if (result < -0.02 && result > -0.1)  {
				if (result < 0.0 && result > -0.15 )  { // 
					cloud_handles.push_back(cloud_outlier->points[i]);  	
				}
			} 

			pcl:: PointCloud<PointXYZ>::Ptr cloud_handles_ptr (new pcl::PointCloud<PointXYZ> (cloud_handles)); 

			// Publish handles for grasping
			cloud_handles_ptr->header.frame_id = "xtion_rgb_optical_frame";
		  ros::Time time_st = ros::Time::now ();
			cloud_handles_ptr->header.stamp = time_st.toNSec()/1e3;
			handle_pub.publish (cloud_handles_ptr);

		} // Close if there are outliers

		// Publish the inlier data (vertical plane)
		cloud_plane_v->header.frame_id = "xtion_rgb_optical_frame";
    ros::Time time_st = ros::Time::now ();
		cloud_plane_v->header.stamp = time_st.toNSec()/1e3;
		pcl_pub_result.publish (cloud_plane_v);
    

    // Publish convex hull
		poly.header.frame_id = "xtion_rgb_optical_frame";
 		poly.header.stamp = ros::Time::now ();
		convex_hull_pub.publish (poly);

    // Publish labelled pointcloud
    semantic_mapping::LabelledPointCloud2 lab_pcl;
    sensor_msgs::PointCloud2 pcl_help;
    pcl::toROSMsg(*cloud_plane_v,pcl_help);
    lab_pcl.pcl=pcl_help;
    lab_pcl.label=className;
		labpcl_seg_pub.publish (lab_pcl);

    

		return;
	} // END OF PLANAR_V SEGMENTATION


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
  convex_hull_pub = nh.advertise<geometry_msgs::PolygonStamped>("/class/convex_hull", 1);
  handle_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl_results/handle", 1);
  

  ros::spin ();
  return 0;

}
