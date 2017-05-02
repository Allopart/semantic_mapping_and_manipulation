#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>

#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <memory>
#include <vector>
#include <list>

#include <mutex>



#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <compressed_depth_image_transport/compressed_depth_subscriber.h>
#include <compressed_image_transport/compressed_subscriber.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PointStamped.h>



#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <simple_recognition/RecogObject.h>

int numCategory; 
std::vector<std::string> objectList; 
std::vector<int> objectCount; 
std::vector<int> objectCategory; 

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> RGBDWithCameraInfoPolicy;

static cv::Mat g_curr_depth;
static cv::Mat g_curr_image;

boost::mutex g_mutex;
std::vector<std::string> g_received_object_names;
std::vector<Eigen::Vector3d> g_received_object_points;


void imagePointCallback(const simple_recognition::RecogObjectConstPtr& msg){
    geometry_msgs::PointStamped image_point;
    image_point.point = msg->stamped_point.point;
    image_point.header = msg->stamped_point.header;

    //std::cout << "RECEIVED OBJECT: " << msg->object_name << std::endl;
    geometry_msgs::PointStamped transformed_image_point;
    geometry_msgs::PointStamped image_head_3d_point;


    Eigen::Vector3d object_point;
    object_point(0) = transformed_image_point.point.x;
    object_point(1) = transformed_image_point.point.y;
    object_point(2) = transformed_image_point.point.z;

    const double min_distance = 0.10;

    g_mutex.lock();
    for(std::size_t i = 0; i < g_received_object_names.size(); ++i){
        Eigen::Vector3d obj_xy_1 = g_received_object_points[i];
        Eigen::Vector3d obj_xy_2 = object_point;
        Eigen::Vector3d diff = obj_xy_1-obj_xy_2;
        double distance = std::sqrt(diff(0)*diff(0)+diff(1)*diff(1));
        if(distance < min_distance){
            g_received_object_names.erase(g_received_object_names.begin()+i);
            g_received_object_points.erase(g_received_object_points.begin()+i);
            i--;
        }
    }

    bool sw=0; 
    for (std::size_t i=0; i<objectList.size(); i++) {
        if (objectList[i]==msg->object_name) { 
          objectCount[i]++; 
          objectCategory[i]=0; 
          sw=1;
          break; 
        }
    }
    
    if (sw==0) {
      objectList.push_back(msg->object_name); 
      objectCount.push_back(0); 
      objectCategory.push_back(0);
    }

    for (std::size_t i=0; i<objectList.size(); i++) {
      std::cout << objectList[i] << ": " << objectCount[i] << std::endl; 
    }
    std::cout << std::endl; 

    g_mutex.unlock();
}


void handleImages(
        const sensor_msgs::Image::ConstPtr& rgb_image_msg,
        const sensor_msgs::Image::ConstPtr& depth_image_msg,
        const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info_msg,
        const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info_msg)
{
    if(depth_camera_info_msg->width != rgb_camera_info_msg->width || depth_camera_info_msg->height != rgb_camera_info_msg->height)
    {
        ROS_WARN("RGB and depth image have different size!");
        return;
    }

    cv::Mat curr_image = cv_bridge::toCvShare(rgb_image_msg,"bgr8")->image;
    cv::Mat curr_depth_meter = cv_bridge::toCvShare(depth_image_msg)->image;
    if(curr_depth_meter.type() == CV_16UC1)
    {
        curr_depth_meter.convertTo(curr_depth_meter, CV_32FC1, 1./1000.0);
        //return;
    }
    else if(curr_depth_meter.type() == CV_32FC1){
    }
    else{
        std::cout << "WRONG DEPTH" << std::endl;
        return;
    }
    if(curr_image.type() != CV_8UC3)
    {
        std::cout << "WRONG IMAGE" << std::endl;
        return;
    }


    cv::imshow("Select Target Objects", curr_image);

    curr_image.copyTo(g_curr_image);
    curr_depth_meter.copyTo(g_curr_depth);

    char key = cv::waitKey(1);
}



int main(int argc, char** argv){

    ros::init(argc, argv, "simple_memory_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle node_handle("~");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_image_subscriber_(nh, "/rgbd_receiver/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_image_subscriber_(nh, "/rgbd_receiver/depth_registered/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> rgb_camera_info_subscriber_(nh, "/rgbd_receiver/rgb/camera_info", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> depth_camera_info_subscriber_(nh, "/rgbd_receiver/depth_registered/camera_info", 1);

    cv::namedWindow( "Select Target Objects");

    ros::Subscriber sub_image_point = node_handle.subscribe<simple_recognition::RecogObject>("/recognition/object_point", 100, imagePointCallback);

    message_filters::Synchronizer<RGBDWithCameraInfoPolicy> synchronizer(RGBDWithCameraInfoPolicy(1), rgb_image_subscriber_, depth_image_subscriber_, rgb_camera_info_subscriber_, depth_camera_info_subscriber_);
    message_filters::Connection connection = synchronizer.registerCallback(handleImages);


    ros::waitForShutdown();

    return 0;
}

