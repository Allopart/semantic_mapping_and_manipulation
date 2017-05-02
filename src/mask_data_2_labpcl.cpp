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
#include "semantic_mapping/LabelledPointCloud2.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace pcl;
using namespace cv;

ros::Publisher img_pub;
ros::Publisher depth_pub;
ros::Publisher pcl_full_pub;

int thickness = 2;
/*float fx_rgb = 546.0500834384593;
float fy_rgb = 545.7495208147751;
float cx_rgb = 318.26543549764034;
float cy_rgb = 235.530988776277;     XTION_ALONE? 
float fx_rgb = 534.912739751989;
float fy_rgb = 533.854358032728;
float cx_rgb = 330.985394535914;
float cy_rgb = 228.556435595447;*/
float fx_rgb = 267.4563698759945;
float fy_rgb = 266.927179016364;
float cx_rgb = 165.492697267957;
float cy_rgb = 114.2782177977235;


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
      std::string className = obj.type;
      // Remove spaces from className
      className.erase (std::remove (className.begin(), className.end(), ' '), className.end());

			//Crop image
			cv::Mat mask_src = cv::Mat::zeros(depth_src.size(), CV_8U);
			cv::Rect ROI(obj.tl_x+thickness, obj.tl_y+thickness, obj.width-2*thickness, obj.height-2*thickness);
			mask_src(ROI).setTo(Scalar::all(255));


      // Clear previously cropped data
      img_cropped = Mat::zeros(img_src.rows,img_src.cols,img_src.type());
      depth_cropped = Mat::zeros(depth_src.rows,depth_src.cols,depth_src.type());

			// Generate cropped image
			img_src.copyTo(img_cropped, mask_src);

			// Generate cropped depth
			depth_src.copyTo(depth_cropped, mask_src);


			// Generate cropped pointcloud
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullPointcloud (new pcl::PointCloud <pcl::PointXYZRGB>); 
			float rgbFocalInvertedX = 1/fx_rgb;	// 1/fx
			float rgbFocalInvertedY = 1/fy_rgb;	// 1/fy

			pcl::PointXYZRGB newPoint;
			for (int i=0;i<depth_cropped.rows;i++)
			{
				 for (int j=0;j<depth_cropped.cols;j++)
				 {
				   float depthValue = depth_cropped.at<float>(i,j);
				   if (depthValue != 0) // if depthValue is not NaN
				   {
				     // Find 3D position respect to rgb frame:
			 		   // Only XTION: newPoint.z = depthValue/1000;
						 newPoint.z = depthValue; //For Mybot
				     newPoint.x = (j - cx_rgb) * newPoint.z * rgbFocalInvertedX;
				     newPoint.y = (i - cy_rgb) * newPoint.z * rgbFocalInvertedY;
			 			 newPoint.r = img_src.at<cv::Vec3b>(i,j)[2];
			 			 newPoint.g = img_src.at<cv::Vec3b>(i,j)[1];
						 newPoint.b = img_src.at<cv::Vec3b>(i,j)[0];
						 fullPointcloud->push_back(newPoint);
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

			// Publish pointcloud
			fullPointcloud->header.frame_id = header.frame_id;
			fullPointcloud->header.stamp = header.stamp.toNSec()/1e3;
			
      semantic_mapping::LabelledPointCloud2 lab_pcl;
      sensor_msgs::PointCloud2 pcl_help;
      pcl::toROSMsg(*fullPointcloud,pcl_help);
      lab_pcl.pcl=pcl_help;
      lab_pcl.label=className;
			pcl_full_pub.publish (lab_pcl);

		} // Close for all objects loop
  }  // Close IF object exists loop
}


int main(int argc, char** argv)
{
  sleep(15); 
 
  ros::init(argc, argv, "mask_data_2_pcl");
  ros::NodeHandle nh;

// ------------ Xtion Subscribers --------
/*
  message_filters::Subscriber<dn_object_detect::DetectedObjects> mask_data(nh, "/dn_object_detect/detected_objects", 1);

  message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera/rgb/image_rect_color", 1);

  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_rect_raw", 1);
*/

// ------------ Mybot Subscribers --------

  message_filters::Subscriber<dn_object_detect::DetectedObjects> mask_data(nh, "/dn_object_detect/detected_objects", 1);

  message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/rgbd_receiver/rgb/image_rect_color", 1);

  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/rgbd_receiver/depth_registered/image_rect", 1);



// ------------ Publishers -----------

  img_pub = nh.advertise<sensor_msgs::Image> ("/class/rect_rgb", 1);

  depth_pub = nh.advertise<sensor_msgs::Image> ("/class/rect_depth", 1);

  pcl_full_pub = nh.advertise<semantic_mapping::LabelledPointCloud2> ("/class/lab_pcl/full", 1);

  typedef sync_policies::ApproximateTime<dn_object_detect::DetectedObjects, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), mask_data, img_sub, depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));


  ros::spin ();
  return 0;

}
