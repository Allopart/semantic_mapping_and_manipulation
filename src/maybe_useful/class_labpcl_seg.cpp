#include <ros/ros.h>
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
ros::Publisher labpcl_seg_pub;
ros::Publisher convex_hull_pub;
ros::Publisher handle_pub;
ros::Publisher desk_pub;

ros::Publisher pcl_pub_result;

geometry_msgs::PolygonStamped poly;

//ros::Publisher pcl_pub_plane;
//ros::Publisher pcl_pub_outlier;

static const char * VoClassNames[] ={"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "trafficlight", "firehydrant", "stopsign", "parkingmeter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
"sportsball", "kite", "baseballbat", "baseballglove", "skateboard", "surfboard", "tennisracket", "bottle", "wineglass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hotdog", "pizza", "donut", "cake", "chair", "couch", "pottedplant", "bed", "diningtable", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cellphone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddybear", "hairdrier", "toothbrush"};

std::string list_plane_all ("suitcase frisbee skis snowboard kite skateboard surfboard book cellphone toaster");

std::string list_plane_h ("bench chair couch bed diningtable toilet pizza");

std::string list_plane_v ("stopsign tv laptop microwave oven refrigerator");

std::string list_plane_outlier ("mouse bowl firehydrant wineglass fork knife spoon banana sandwich carrot hotdog  donut cake pottedplant clock vase scissors remote sink teddybear hairdrier keyboard umbrella");

std::string list_sphere ("sportsball apple orange");

std::string list_cylinder ("cup bottle baseballbat");

std::string list_color ("car airplane bus train truck boat baseballglove broccoli");

std::string list_region ("person bird cat dog horse sheep cow elephant bear zebra giraffe backpack handbag");

std::string list_other ("bicycle motorcycle trafficlight parkingmeter tie tennisracket toothbrush");

std::string list_nothing ("");


void segmentation_callback(const semantic_mapping::LabelledPointCloud2::ConstPtr& pcl_ptr)
{

	// Get mask data
	std::string className = pcl_ptr->label;
  // Remove spaces from className
  className.erase (std::remove (className.begin(), className.end(), ' '), className.end());

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

	// Check lists to see what clustering method to use
	std::size_t found_plane_all = list_plane_all.find(className);
	std::size_t found_plane_h = list_plane_h.find(className);
	std::size_t found_plane_v = list_plane_v.find(className);
	std::size_t found_plane_outlier = list_plane_outlier.find(className);
	std::size_t found_sphere = list_sphere.find(className);
	std::size_t found_cylinder = list_cylinder.find(className);
	std::size_t found_color = list_color.find(className);
	std::size_t found_region = list_region.find(className);
	std::size_t found_other = list_other.find(className);
  std::size_t found_nothing = list_nothing.find(className);

	// *************** Planar_all segmentation ************** 
	
	if (found_plane_all!=std::string::npos) {

		// Initialize planar segmentation 
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_all (new     pcl::PointCloud<pcl::PointXYZ> ());

		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		//seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.005);
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_plane_all);

		// Publish the inlier data (door)
		cloud_plane_all->header.frame_id = "xtion_rgb_optical_frame";
    ros::Time time_st = ros::Time::now ();
		cloud_plane_all->header.stamp = time_st.toNSec()/1e3;
		pcl_pub_result.publish (cloud_plane_all);
    //ROS_INFO("%s - Planar_all\n", className.c_str());

		// Publish labelled pointcloud
    semantic_mapping::LabelledPointCloud2 lab_pcl;
    sensor_msgs::PointCloud2 pcl_help;
    pcl::toROSMsg(*cloud_plane_all,pcl_help);
    lab_pcl.pcl=pcl_help;
    lab_pcl.label=className;
		labpcl_seg_pub.publish (lab_pcl);

		return;
	} // END OF PLANAR_ALL SEGMENTATION


	// *************** Planar_h segmentation ************** 
	
	if (found_plane_h!=std::string::npos) {

		//ROS_INFO("%s - Planar_h\n", className.c_str());

		// Initialize planar segmentation 
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_h (new     pcl::PointCloud<pcl::PointXYZ> ());

		/// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		//Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); 
		//seg.setAxis(axis); 
		//seg.setEpsAngle((30*3.14)/180); 
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		//seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.005);
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
   
    if (inliers->indices.size () == 0)
    {
      ROS_INFO ("Could not estimate a planar model for the given dataset.");
      return;
    }

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_plane_h);

		// Publish the inlier data 
		cloud_plane_h->header.frame_id = "xtion_rgb_optical_frame";
    ros::Time time_st = ros::Time::now ();
		cloud_plane_h->header.stamp = time_st.toNSec()/1e3;
		pcl_pub_result.publish (cloud_plane_h);
    

    // Publish labelled pointcloud
    semantic_mapping::LabelledPointCloud2 lab_pcl;
    sensor_msgs::PointCloud2 pcl_help;
    pcl::toROSMsg(*cloud_plane_h,pcl_help);
    lab_pcl.pcl=pcl_help;
    lab_pcl.label=className;
		labpcl_seg_pub.publish (lab_pcl);
    
		return;
	} // END OF PLANAR_H SEGMENTATION




	// *************** Planar_V segmentation ************** 
	
	if (found_plane_v!=std::string::npos) {

		//ROS_INFO("%s - Planar_v\n", className.c_str());

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
		//seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.005);
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
		 
			pcl::PointCloud<pcl::PointXYZ> cloud_handles; 

			for (int i=0; i <= cloud_outlier->points.size() ; i++ ) {

				float Px = cloud_outlier->points[i].x;
				float Py = cloud_outlier->points[i].y;
				float Pz = cloud_outlier->points[i].z;

				//std::cerr << "Iteration: " << i << std::endl;
		 
			 //std::cerr << "Point: " << Px << " " << Py << " " << Pz << std::endl;

				float result = Px*Rx + Py*Ry + Pz*Rz + Rd;
				//std::cerr << "Result: " << result << std::endl;

				if (result < 0.0)  {
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




// *************** Planar_outlier segmentation ************** 
	
	if (found_plane_outlier!=std::string::npos) {

		//ROS_INFO("%s - Outlier\n", className.c_str());		

		// Initialize planar segmentation 
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new     pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier (new pcl::PointCloud<pcl::PointXYZ> ());

		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); 
		seg.setAxis(axis); 
		seg.setEpsAngle((30*3.14)/180); 
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		//seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.005);
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

		  
			// Remove outliers below plane
			float Rx = coefficients->values[0];
			float Ry = coefficients->values[1];
			float Rz = coefficients->values[2];
			float Rd = coefficients->values[3];
		 
			pcl::PointCloud<pcl::PointXYZ> good_outliers; 

			for (int i=0; i <= cloud_outlier->points.size() ; i++ ) {
				float Px = cloud_outlier->points[i].x;
				float Py = cloud_outlier->points[i].y;
				float Pz = cloud_outlier->points[i].z;

				float result = Px*Rx + Py*Ry + Pz*Rz + Rd;

				if (result > 0.0)  {
					good_outliers.push_back(cloud_outlier->points[i]);    	
				}
			} 

			pcl:: PointCloud<PointXYZ>::Ptr good_outliers_ptr (new pcl::PointCloud<PointXYZ> (good_outliers)); 

			// Build a passthrough filter to remove spurious NaNs
			pcl::PassThrough<PointT> pass_out;
			pcl::PointCloud<PointT>::Ptr good_outliers_f_ptr (new pcl::PointCloud<PointT>);
			pass_out.setInputCloud (good_outliers_ptr);
			pass_out.setFilterFieldName ("z");
			pass_out.setFilterLimits (0.1, 5.0);
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



// *************** Cylinder segmentation ************** 
	
	if (found_cylinder!=std::string::npos) {

		//ROS_INFO("%s - Cylindrical\n", className.c_str());

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

// *************** Sphere segmentation ************** 
	
	if (found_sphere!=std::string::npos) {

		//ROS_INFO("%s - Sphere\n", className.c_str());

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
		pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_sphere (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_sphere (new pcl::PointIndices);


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

		// Create the segmentation object for sphere segmentation and set all the parameters
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_SPHERE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setNormalDistanceWeight (0.1);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.05);
		seg.setRadiusLimits (0, 0.2);
		seg.setInputCloud (cloud_filtered2);
		seg.setInputNormals (cloud_normals2);

		// Obtain the sphere inliers and coefficients
		seg.segment (*inliers_sphere, *coefficients_sphere);
//				std::cerr << "Sphere coefficients: " << *coefficients_sphere << std::endl;

		// Write the sphere inliers to disk
		extract.setInputCloud (cloud_filtered2);
		extract.setIndices (inliers_sphere);
		extract.setNegative (false);
		pcl::PointCloud<PointT>::Ptr cloud_sphere (new pcl::PointCloud<PointT> ());
		extract.filter (*cloud_sphere);
/*				if (cloud_sphere->points.empty ()) 
			std::cerr << "Can't find the spherical component." << std::endl;
		else
		{
			std::cerr << "PointCloud representing the spherical component: " << cloud_sphere->points.size () << " data points." << std::endl;
		}
*/
		// Publish the plane data 
		cloud_sphere->header.frame_id = "xtion_rgb_optical_frame";
    ros::Time time_st = ros::Time::now ();
		cloud_sphere->header.stamp = time_st.toNSec()/1e3;
		pcl_pub_result.publish (cloud_sphere);
    

    // Publish labelled pointcloud
    semantic_mapping::LabelledPointCloud2 lab_pcl;
    sensor_msgs::PointCloud2 pcl_help;
    pcl::toROSMsg(*cloud_sphere,pcl_help);
    lab_pcl.pcl=pcl_help;
    lab_pcl.label=className;
		labpcl_seg_pub.publish (lab_pcl);

		return;
	} // END OF SPHERE SEGMENTATION


	// *************** Nothing segmentation ************** 

	if (found_nothing!=std::string::npos) {

		//ROS_INFO("%s - None\n", className.c_str());

		pcl:: PointCloud<PointXYZ>::Ptr all_data_ptr (new pcl::PointCloud<PointXYZ> (*cloud_filtered));
 
    // Publish the all data without segmenting 
		all_data_ptr->header.frame_id = "xtion_rgb_optical_frame";
    ros::Time time_st = ros::Time::now ();
		all_data_ptr->header.stamp = time_st.toNSec()/1e3;
		pcl_pub_result.publish (all_data_ptr);
    

    // Publish labelled pointcloud
    semantic_mapping::LabelledPointCloud2 lab_pcl;
    sensor_msgs::PointCloud2 pcl_help;
    pcl::toROSMsg(*all_data_ptr,pcl_help);
    lab_pcl.pcl=pcl_help;
    lab_pcl.label=className;
		labpcl_seg_pub.publish (lab_pcl);

		return;
	} // END OF NOTHING SEGMENTATION


}


int main(int argc, char** argv)
{

  sleep(15);
  ros::init(argc, argv, "class_pcl_seg");

  ros::NodeHandle nh;
  nhPtr = & nh; // Give global access to this nodeHandle

// ------------ Subscribers --------

  ros::Subscriber pcl_sub = nh.subscribe("/class/lab_pcl/full", 1, segmentation_callback);

// ------------ Publishers -----------

  labpcl_seg_pub = nh.advertise<semantic_mapping::LabelledPointCloud2>("/class/lab_pcl/segmented", 1);

  convex_hull_pub = nh.advertise<geometry_msgs::PolygonStamped>("/class/convex_hull", 1);

  handle_pub = nh.advertise<sensor_msgs::PointCloud2>("/class/handle", 1);

  desk_pub = nh.advertise<sensor_msgs::PointCloud2>("/class/desk", 1);



  ros::spin ();
  return 0;

}
