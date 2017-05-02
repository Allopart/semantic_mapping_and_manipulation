#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
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

using namespace sensor_msgs;
using namespace message_filters;
using namespace pcl;
using namespace cv;

ros::Publisher img_pub;
ros::Publisher depth_pub;
//ros::Publisher pcl_full_pub;


float fx_rgb = 267.4563698759945;
float fy_rgb = 266.927179016364;
float cx_rgb = 165.492697267957;
float cy_rgb = 114.2782177977235;

void callback(const sensor_msgs::ImageConstPtr& mask_sub, const sensor_msgs::ImageConstPtr& img_sub, const sensor_msgs::ImageConstPtr& depth_sub)
{

  cv_bridge::CvImagePtr mask_ptr;
  cv_bridge::CvImagePtr img_ptr;
  cv_bridge::CvImagePtr depth_ptr;
  try
  {
    mask_ptr = cv_bridge::toCvCopy(mask_sub, sensor_msgs::image_encodings::MONO8);
    img_ptr = cv_bridge::toCvCopy(img_sub, sensor_msgs::image_encodings::BGR8);
    depth_ptr = cv_bridge::toCvCopy(depth_sub, "32FC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Update GUI Window
  //cv::imshow("Mask viewer", mask_ptr->image);
  //cv::imshow("Image viewer", img_cropped);
  //cv::imshow("Depth viewer", depth_cropped);
  //cv::waitKey(1);

// Create conversions
  Mat mask_src;
  mask_src=mask_ptr->image;

  Mat img_src, img_cropped;
  img_src=img_ptr->image;

  Mat depth_src, depth_cropped;
  depth_src=depth_ptr->image;

  // Generate cropped image
  img_src.copyTo(img_cropped, mask_src);

  // Generate cropped depth
  depth_src.copyTo(depth_cropped, mask_src);

/*
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
	 newPoint.z = depthValue;
	 newPoint.x = (j - cx_rgb) * newPoint.z * rgbFocalInvertedX;
	 newPoint.y = (i - cy_rgb) * newPoint.z * rgbFocalInvertedY;
   newPoint.r = img_src.at<cv::Vec3b>(i,j)[2];
	 newPoint.g = img_src.at<cv::Vec3b>(i,j)[1];
	 newPoint.b = img_src.at<cv::Vec3b>(i,j)[0];
	 fullPointcloud->push_back(newPoint);
	 }
      }
   }

*/ 
  //Convert images to ROS type
  std_msgs::Header header;
  header.frame_id = "camera_rgb_optical_frame";
  header.stamp = ros::Time::now();
  cv_bridge::CvImage img_bridge;
  cv_bridge::CvImage depth_bridge;
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img_cropped);
  depth_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, depth_cropped);

  

  // Publish images
  img_pub.publish(img_bridge.toImageMsg());
  depth_pub.publish(depth_bridge.toImageMsg());
/*
  // Publish pointcloud
  fullPointcloud->header.frame_id = "camera_rgb_optical_frame";
  ros::Time time_st = ros::Time::now ();
  fullPointcloud->header.stamp = time_st.toNSec()/1e3;
  pcl_full_pub.publish (fullPointcloud);
*/
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "crop_2_pcl");
/*
  cv::namedWindow("Mask viewer");
  cv::namedWindow("Image viewer");
  cv::namedWindow("Depth viewer");
*/
  
  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> mask_sub(nh, "/cropped/person_mask", 1);
  message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera/rgb/image_rect_color", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/sw_registered/image_rect", 1);
  //pcl_full_pub = nh.advertise<sensor_msgs::PointCloud2> ("/person/full_roi_pcl", 1);
  img_pub = nh.advertise<sensor_msgs::Image> ("/person/person_rect_color", 1);
  depth_pub = nh.advertise<sensor_msgs::Image> ("/person/person_rect_depth", 1);

  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(7), mask_sub, img_sub, depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));


  ros::spin ();
  return 0;

}
