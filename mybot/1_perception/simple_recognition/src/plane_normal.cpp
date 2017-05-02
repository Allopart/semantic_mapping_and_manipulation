// STL Header
#include <iostream>
#include <numeric>
#include <memory>
#include "cbf.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <compressed_depth_image_transport/compressed_depth_subscriber.h>
#include <compressed_image_transport/compressed_subscriber.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <moveit_msgs/PlanningSceneWorld.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>

// Eigen Header
#include <eigen3/Eigen/Dense>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

// plaincode header
#include "Core.hpp"
#include "Color.hpp"

// Boost Header
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

cv::Mat originRGB, originDepth;
cv::Mat rawRGB, rawDepth;
cv::Rect imgRect(20,20,280,200); 
Eigen::VectorXf parameter = Eigen::VectorXf(3); 

void cropImage(cv::Mat& image, cv::Mat& result) {
    result = image(imgRect);
}

bool is_running = false;

int getNormal(double fx, double fy, double cx, double cy, ros::Publisher& normal_pub, bool saveMode=false){

  static bool tuneMode=false;
  
  ros::WallDuration sleep_time(0.05);
  ros::Time current_time = ros::Time::now();

  Eigen::MatrixXf K, Kinv;
  plaincode::setCameraMat(fx, fy, cx-20.0, cy-20.0, K, Kinv);

  cv::Mat RGB, Depth;
  
  Eigen::MatrixXf points;

  double time = cv::getTickCount();
  cropImage(originRGB, RGB);
  cropImage(originDepth, Depth);
  plaincode::crossBilateralFilter(RGB, Depth);
  plaincode::getPoint3d(Depth, points, Kinv);
  plaincode::planeParameter(Depth, points, parameter);

  geometry_msgs::Vector3 normMsgs; 
  normMsgs.x = parameter(0); 
  normMsgs.y = parameter(1); 
  normMsgs.z = parameter(2); 
  
  normal_pub.publish(normMsgs); 

  time = ((double)cv::getTickCount()-time)/cv::getTickFrequency();
//  std::cout << "time = " << time << std::endl;
  char key = cv::waitKey(1);
  if (key=='q') {
      exit(0);
  }
}

bool is_updated_rgb = false;
bool is_updated_depth = false;

void cameraCallbackForRGB(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& camera_info_msg, ros::Publisher& normal_pub){
    
    static bool colorTuned=false;

    double fx = camera_info_msg->K[0];
    double fy = camera_info_msg->K[4];
    double cx = camera_info_msg->K[2];
    double cy = camera_info_msg->K[5];

    try{
        rawRGB = cv_bridge::toCvShare(msg, "bgr8")->image;
        if(!rawRGB.empty()){
            if( is_updated_depth && !is_running){
                is_running = true;
                cv::resize(rawRGB, originRGB, cv::Size(320,240));

             		float image_scale = (float)originRGB.rows/(float)rawRGB.rows;
                is_updated_rgb = true;
                getNormal(fx*image_scale, fy*image_scale, cx*image_scale, cy*image_scale, normal_pub); 
                is_running = false;
                is_updated_rgb = false;
                is_updated_depth = false;
           }
        }
    } catch (cv_bridge::Exception& e){
        //ROS_ERROR("Could not convert into cv::Mat %s", msg->encoding.c_str());
    }
}


void imageCallbackForDepth(const sensor_msgs::ImageConstPtr& msg){
    try{
        //rawDepth = cv::imdecode(msg->data, cv::IMREAD_ANYDEPTH);
        rawDepth = cv_bridge::toCvShare(msg)->image;

        if(rawDepth.type() == CV_32FC1){
//            std::cout << "RAW DEPTH1: " << rawDepth.at<float>(200,200) << std::endl;
            rawDepth.convertTo(rawDepth, CV_32FC1, 1000.0/1.0);
        }
        cv::Mat image_float;
        rawDepth.convertTo(image_float, CV_16UC1);
        if(!image_float.empty()){
            if(!is_running || !is_updated_depth){
                cv::resize(image_float, originDepth, cv::Size(320,240));
                is_updated_depth = true;
            }
        }
    } catch (cv_bridge::Exception& e){
        //ROS_ERROR("Could not convert into cv::Mat %s", msg->encoding.c_str());
    } catch (cv::Exception& e){
    }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "plane_normal");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  plaincode::th::distAbove=0; // 실제 값은 -100
  plaincode::th::thSize=100;
  plaincode::th::thC=20;
 
  ros::Publisher normal_pub = nh.advertise<geometry_msgs::Vector3>("/simple_recognition/normal",1);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_for_depth = it.subscribe("/rgbd_receiver/depth_registered/image_raw", 1, imageCallbackForDepth);
  image_transport::CameraSubscriber sub_for_rgb = it.subscribeCamera("/rgbd_receiver/rgb/image_raw", 1, boost::bind(cameraCallbackForRGB, _1, _2, normal_pub));
  ros::spin();

  return 0;
}
