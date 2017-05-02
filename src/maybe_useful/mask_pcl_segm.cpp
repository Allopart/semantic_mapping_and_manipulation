#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <boost/bind.hpp>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cmath> 

#include "dn_object_detect/DetectedObjects.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace pcl;
using namespace cv;
using namespace std;

typedef pcl::PointXYZ PointT;

ros::NodeHandle* nhPtr; // Give global access to the nodeHandle

ros::Publisher img_pub;
ros::Publisher depth_pub;

static const char * VoClassNames[] ={"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "trafficlight", "firehydrant", "stopsign", "parkingmeter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
"sportsball", "kite", "baseballbat", "baseballglove", "skateboard", "surfboard", "tennisracket", "bottle", "wineglass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hotdog", "pizza", "donut", "cake", "chair", "couch", "pottedplant", "bed", "diningtable", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cellphone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddybear", "hairdrier", "toothbrush"};

std::string list_plane_all ("suitcase frisbee skis snowboard kite skateboard surfboard book");

std::string list_plane_h ("bench chair couch bed diningtable toilet");

std::string list_plane_v ("stopsign tv laptop microwave oven refrigerator");

std::string list_plane_outlier ("firehydrant bottle wineglass fork knife spoon bowl banana apple sandwich orange broccoli carrot hotdog pizza donut cake pottedplant clock vase scissors mouse remote keyboard cellphone toaster sink teddybear hairdrier");

std::string list_sphere ("umbrella sportsball apple orange");

std::string list_cylinder ("cup umbrella baseballbat");

std::string list_color ("car airplane bus train truck boat baseballglove");

std::string list_region ("person bird cat dog horse sheep cow elephant bear zebra giraffe backpack handbag");

std::string list_other ("bicycle motorcycle trafficlight parkingmeter tie tennisracket toothbrush");


int thickness = 2;
float fx_rgb = 546.0500834384593;
float fy_rgb = 545.7495208147751;
float cx_rgb = 318.26543549764034;
float cy_rgb = 235.530988776277;


void callback(const dn_object_detect::DetectedObjects::ConstPtr& mask_data_ptr, const sensor_msgs::ImageConstPtr& img_sub, const sensor_msgs::ImageConstPtr& depth_sub)
{

  cv_bridge::CvImagePtr img_ptr;
  cv_bridge::CvImagePtr depth_ptr;
  try
  {
    img_ptr = cv_bridge::toCvCopy(img_sub, sensor_msgs::image_encodings::BGR8);
    depth_ptr = cv_bridge::toCvCopy(depth_sub, "32FC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Create conversions

  Mat img_src, img_cropped;
  img_src=img_ptr->image;

  Mat depth_src, depth_cropped;
  depth_src=depth_ptr->image;

 
  // If no objects detected -> break
  if (mask_data_ptr->objects.size()>0){

		// FOR ALL OBJECTS DETETCTED
		for(int n_object = 0; n_object <= mask_data_ptr->objects.size()-1; n_object++) {
		  	
			// Get mask data
			dn_object_detect::ObjectInfo obj = mask_data_ptr->objects[n_object];
			std_msgs::Header header;
			header = mask_data_ptr->header;
      // Get ClassName
      std::string className = obj.type;
      // Remove spaces from className
      className.erase (std::remove (className.begin(), className.end(), ' '), className.end());	

      // Create class-specific publisher
			std::string topicName_result = "/pcl_results/"+ className;
			ros::Publisher pcl_pub_result = nhPtr->advertise<sensor_msgs::PointCloud2>(topicName_result, 1);

			//Crop image
			cv::Mat mask_src = cv::Mat::zeros(depth_src.size(), CV_8U);
			cv::Rect ROI(obj.tl_x+thickness, obj.tl_y+thickness, obj.width-2*thickness, obj.height-2*thickness);
			mask_src(ROI).setTo(Scalar::all(255));

			// Generate cropped image
			img_src.copyTo(img_cropped, mask_src);

			// Generate cropped depth
			depth_src.copyTo(depth_cropped, mask_src);


			// Generate cropped pointcloud
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud <pcl::PointXYZ>); 
			float rgbFocalInvertedX = 1/fx_rgb;	// 1/fx
			float rgbFocalInvertedY = 1/fy_rgb;	// 1/fy

			pcl::PointXYZ newPoint;
			// pcl::PointXYZRGB newPoint;
			for (int i=0;i<depth_cropped.rows;i++)
			{
				 for (int j=0;j<depth_cropped.cols;j++)
				 {
				   float depthValue = depth_cropped.at<float>(i,j);
				   if (depthValue != 0) // if depthValue is not NaN
				   {
				     // Find 3D position respect to rgb frame:
			 		   newPoint.z = depthValue/1000;
				     newPoint.x = (j - cx_rgb) * newPoint.z * rgbFocalInvertedX;
				     newPoint.y = (i - cy_rgb) * newPoint.z * rgbFocalInvertedY;
			 			 //newPoint.r = img_src.at<cv::Vec3b>(i,j)[2];
			 			 //newPoint.g = img_src.at<cv::Vec3b>(i,j)[1];
						 //newPoint.b = img_src.at<cv::Vec3b>(i,j)[0];
						 cloud_xyz->push_back(newPoint);
					 }
				 }
			 }

			//Convert images to ROS type
			cv_bridge::CvImage img_bridge;
			cv_bridge::CvImage depth_bridge;
			img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img_cropped);
			depth_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, depth_cropped);

			// Publish images
			img_pub.publish(img_bridge.toImageMsg());
			depth_pub.publish(depth_bridge.toImageMsg());

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


			// *************** Planar_all segmentation ************** 
			
			if (found_plane_all!=std::string::npos) {

				ROS_INFO(" PLANAR_ALL SEGMENTATION OF %s\n", className.c_str());  
			
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
				seg.setDistanceThreshold (0.1);
				seg.setInputCloud (cloud_xyz);
				seg.segment (*inliers, *coefficients);

				// Extract the planar inliers from the input cloud
				pcl::ExtractIndices<pcl::PointXYZ> extract;
				extract.setInputCloud (cloud_xyz);
				extract.setIndices (inliers);
				extract.setNegative (false);
				extract.filter (*cloud_plane_all);

				// Publish the inlier data (door)
				cloud_plane_all->header.frame_id = "rgb_optical_frame";
        ros::Time time_st = ros::Time::now ();
				cloud_plane_all->header.stamp = time_st.toNSec()/1e3;
				pcl_pub_result.publish (cloud_plane_all);

			} // END OF PLANAR_ALL SEGMENTATION




    	// *************** Planar_h segmentation ************** 
			
			else if (found_plane_h!=std::string::npos) {

				ROS_INFO(" PLANAR_H SEGMENTATION OF %s\n", className.c_str());  
			
				// Initialize planar segmentation 
   			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_h (new     pcl::PointCloud<pcl::PointXYZ> ());

				/// Create the segmentation object
				pcl::SACSegmentation<pcl::PointXYZ> seg;
				Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); 
 				seg.setAxis(axis); 
 				seg.setEpsAngle((20*3.14)/180); 
				// Optional
				seg.setOptimizeCoefficients (true);
				// Mandatory
				seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
				seg.setMethodType (pcl::SAC_RANSAC);
				//seg.setMaxIterations (100);
				seg.setDistanceThreshold (0.01);
				seg.setInputCloud (cloud_xyz);
				seg.segment (*inliers, *coefficients);

				// Extract the planar inliers from the input cloud
				pcl::ExtractIndices<pcl::PointXYZ> extract;
				extract.setInputCloud (cloud_xyz);
				extract.setIndices (inliers);
				extract.setNegative (false);
				extract.filter (*cloud_plane_h);

				// Publish the inlier data (door)
				cloud_plane_h->header.frame_id = "rgb_optical_frame";
        ros::Time time_st = ros::Time::now ();
				cloud_plane_h->header.stamp = time_st.toNSec()/1e3;
				pcl_pub_result.publish (cloud_plane_h);

			} // END OF PLANAR_H SEGMENTATION




    	// *************** Planar_V segmentation ************** 
			
			if (found_plane_v!=std::string::npos) {

				ROS_INFO(" PLANAR_V SEGMENTATION OF %s\n", className.c_str());  
			
				// Initialize planar segmentation 
   			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_v (new     pcl::PointCloud<pcl::PointXYZ> ());

				// Create the segmentation object
				pcl::SACSegmentation<pcl::PointXYZ> seg;
				Eigen::Vector3f axis = Eigen::Vector3f(1.0,1.0,0.0); 
 				seg.setAxis(axis); 
				// Optional
				seg.setOptimizeCoefficients (true);
				// Mandatory
				seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
				seg.setMethodType (pcl::SAC_RANSAC);
				//seg.setMaxIterations (100);
				seg.setDistanceThreshold (0.005);
				seg.setInputCloud (cloud_xyz);
				seg.segment (*inliers, *coefficients);

				// Extract the planar inliers from the input cloud
				pcl::ExtractIndices<pcl::PointXYZ> extract;
				extract.setInputCloud (cloud_xyz);
				extract.setIndices (inliers);
				extract.setNegative (false);
				extract.filter (*cloud_plane_v);

				// Publish the inlier data (door)
				cloud_plane_v->header.frame_id = "rgb_optical_frame";
        ros::Time time_st = ros::Time::now ();
				cloud_plane_v->header.stamp = time_st.toNSec()/1e3;
				pcl_pub_result.publish (cloud_plane_v);

			} // END OF PLANAR_V SEGMENTATION




// *************** Planar_outlier segmentation ************** 
			
			else if (found_plane_outlier!=std::string::npos) {

				ROS_INFO(" PLANAR_OUTLIER SEGMENTATION OF %s\n", className.c_str());  
			
				// Initialize planar segmentation 
   			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier (new pcl::PointCloud<pcl::PointXYZ> ());

				// Create the segmentation object
				pcl::SACSegmentation<pcl::PointXYZ> seg;
				Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); 
 				seg.setAxis(axis); 
 				seg.setEpsAngle((20*3.14)/180); 
				// Optional
				seg.setOptimizeCoefficients (true);
				// Mandatory
				seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
				seg.setMethodType (pcl::SAC_RANSAC);
				//seg.setMaxIterations (100);
				seg.setDistanceThreshold (0.005);
				seg.setInputCloud (cloud_xyz);
				seg.segment (*inliers, *coefficients);
				// Extract the planar inliers from the input cloud
				pcl::ExtractIndices<pcl::PointXYZ> extract;
				extract.setInputCloud (cloud_xyz);
				extract.setIndices (inliers);
		
				// Remove the planar inliers, extract the rest
				extract.setNegative (true);
				extract.filter (*cloud_outlier);


				// Publish the outlier data (handle)
				cloud_outlier->header.frame_id = "rgb_optical_frame";
				ros::Time time_st = ros::Time::now ();
				cloud_outlier->header.stamp = time_st.toNSec()/1e3;
				pcl_pub_result.publish (cloud_outlier);
			} // END OF PLANANR_OUTLIER SEGMENTATION



// *************** Cylinder segmentation ************** 
			
			else if (found_cylinder!=std::string::npos) {

				ROS_INFO(" CYLINDER SEGMENTATION OF %s\n", className.c_str());  
			
				// All the objects needed
				pcl::NormalEstimation<PointT, pcl::Normal> ne;
				pcl::PassThrough<PointT> pass;
				pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
			pcl::ExtractIndices<PointT> extract;
				pcl::ExtractIndices<pcl::Normal> extract_normals;
				pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

			  // Datasets
				pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
				pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
				pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
				pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
				pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

		
				// Build a passthrough filter to remove spurious NaNs
				pass.setInputCloud (cloud_xyz);
				pass.setFilterFieldName ("z");
				pass.setFilterLimits (0, 1.5);
				pass.filter (*cloud_filtered);
//				std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

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
				seg.setMaxIterations (10000);
				seg.setDistanceThreshold (0.05);
				seg.setRadiusLimits (0, 0.2);
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
				cloud_cylinder->header.frame_id = "rgb_optical_frame";
        ros::Time time_st = ros::Time::now ();
				cloud_cylinder->header.stamp = time_st.toNSec()/1e3;
				pcl_pub_result.publish (cloud_cylinder);
				
			} // END OF CYLINDER SEGMENTATION

// *************** Sphere segmentation ************** 
			
			else if (found_sphere!=std::string::npos) {

				ROS_INFO(" SPHERE SEGMENTATION OF %s\n", className.c_str());  
			
				// All the objects needed
				pcl::NormalEstimation<PointT, pcl::Normal> ne;
				pcl::PassThrough<PointT> pass;
				pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
			pcl::ExtractIndices<PointT> extract;
				pcl::ExtractIndices<pcl::Normal> extract_normals;
				pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

			  // Datasets
				pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
				pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
				pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
				pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
				pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_sphere (new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_sphere (new pcl::PointIndices);

		
				// Build a passthrough filter to remove spurious NaNs
				pass.setInputCloud (cloud_xyz);
				pass.setFilterFieldName ("z");
				pass.setFilterLimits (0, 1.5);
				pass.filter (*cloud_filtered);
//				std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

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
				seg.setMaxIterations (10000);
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
				cloud_sphere->header.frame_id = "rgb_optical_frame";
        ros::Time time_st = ros::Time::now ();
				cloud_sphere->header.stamp = time_st.toNSec()/1e3;
				pcl_pub_result.publish (cloud_sphere);
				
			} // END OF SPHERE SEGMENTATION

/* ---------------  END SEGMENTATION ------------------------*/

		} // Close for all objects loop
  }  // Close IF object exists loop
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mask_data_2_pcl");

  ros::NodeHandle nh;
  nhPtr = & nh; // Give global access to this nodeHandle
  
  message_filters::Subscriber<dn_object_detect::DetectedObjects> mask_data(nh, "/dn_object_detect/detected_objects", 1);
  message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera/rgb/image_rect_color", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_rect_raw", 1);

  img_pub = nh.advertise<sensor_msgs::Image> ("/class/rect_rgb", 1);
  depth_pub = nh.advertise<sensor_msgs::Image> ("/class/rect_depth", 1);

  typedef sync_policies::ApproximateTime<dn_object_detect::DetectedObjects, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), mask_data, img_sub, depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));


  ros::spin ();
  return 0;

}
