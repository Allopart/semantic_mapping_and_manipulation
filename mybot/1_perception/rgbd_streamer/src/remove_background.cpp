#include <iostream>
#include <vector>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/filters/crop_box.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>



tf::TransformListener*  g_transform_listener;

image_transport::CameraPublisher g_pub_depth;

sensor_msgs::CameraInfo g_camera_info;

boost::mutex mutex_for_camera_info;

bool is_start = false;

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg){
    if(mutex_for_camera_info.try_lock()){
        g_camera_info = *camera_info_msg;
        is_start = true;
        mutex_for_camera_info.unlock();
    }
}


void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg){
    mutex_for_camera_info.lock();
    if(!is_start){
        mutex_for_camera_info.unlock();
        return;   
    }
    sensor_msgs::CameraInfo copied_camera_info = g_camera_info;
    mutex_for_camera_info.unlock();

    tf::StampedTransform transform_base_footprint_tf;
    try{
    g_transform_listener->lookupTransform("base_footprint", copied_camera_info.header.frame_id, ros::Time(0), transform_base_footprint_tf);
    } catch (tf::TransformException& ex){
        return ;
    }

    Eigen::Affine3d transform_base_footprint_eigen;
    tf::transformTFToEigen(transform_base_footprint_tf, transform_base_footprint_eigen);
    Eigen::Affine3d transform_base_footprint_inverse_eigen = transform_base_footprint_eigen.inverse();



    geometry_msgs::TransformStamped transform_base_footprint;
    geometry_msgs::TransformStamped transform_base_footprint_inverse;

    transform_base_footprint.header = copied_camera_info.header;
    transform_base_footprint.header.frame_id = "base_footprint";
    transform_base_footprint_inverse.header = copied_camera_info.header;

    tf::transformEigenToMsg(transform_base_footprint_eigen, transform_base_footprint.transform);
    tf::transformEigenToMsg(transform_base_footprint_inverse_eigen, transform_base_footprint_inverse.transform);


    sensor_msgs::PointCloud2 transformed_point_cloud;
    tf2::doTransform(*point_cloud_msg, transformed_point_cloud, transform_base_footprint); 


    pcl::PCLPointCloud2 pre_pcl_ros;
    pcl::PCLPointCloud2 filtered_pcl_ros;
    pcl_conversions::toPCL(transformed_point_cloud, pre_pcl_ros);

    pcl::CropBox<pcl::PCLPointCloud2> crop_box_filter;
    crop_box_filter.setInputCloud(boost::make_shared<pcl::PCLPointCloud2>(pre_pcl_ros));
    Eigen::Vector4f min_point_box(0, -1, 0, 1);
    Eigen::Vector4f max_point_box(2, 1, 1.5, 1);

    crop_box_filter.setMin(min_point_box);
    crop_box_filter.setMax(max_point_box);
    crop_box_filter.filter(filtered_pcl_ros);


    sensor_msgs::PointCloud2 removed_point_cloud;
    sensor_msgs::PointCloud2 transformed_removed_point_cloud;
    pcl_conversions::fromPCL(filtered_pcl_ros, removed_point_cloud);
    tf2::doTransform(removed_point_cloud, transformed_removed_point_cloud, transform_base_footprint_inverse); 

    sensor_msgs::PointCloud2ConstIterator<float> x_in(transformed_removed_point_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_in(transformed_removed_point_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_in(transformed_removed_point_cloud, "z");

    const float fx = copied_camera_info.K[0];
    const float fy = copied_camera_info.K[4];
    const float cx = copied_camera_info.K[2];
    const float cy = copied_camera_info.K[5];

    cv::Mat remove_depth_image = cv::Mat::zeros(cv::Size(copied_camera_info.width, copied_camera_info.height), CV_16UC1);
    for(; x_in != x_in.end(); ++x_in, ++y_in, ++z_in){
        Eigen::Vector3f point(*x_in, *y_in, *z_in);
        int x = (int)(point(0)*fx/point(2) + cx);
        int y = (int)(point(1)*fy/point(2) + cy);

        if(x >= 0 && x < copied_camera_info.width && y >=0 && y < copied_camera_info.height){
            remove_depth_image.at<unsigned short>(y,x) = (unsigned short)(point(2)*1000.0);
        }
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "16UC1", remove_depth_image).toImageMsg();

    g_pub_depth.publish(*msg, copied_camera_info);
}


int main(int argc, char* argv[] ){


    ros::init(argc, argv, "remove_background");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::cout << "START " << std::endl;
    ros::NodeHandle nh("~");

    std::cout << "START " << std::endl;
    ros::NodeHandle node_handle;
    image_transport::ImageTransport it(node_handle);
    g_pub_depth = it.advertiseCamera("/rgbd_receiver/depth_registered/image_removed", 1);

    ros::Subscriber sub_camera_info = nh.subscribe<sensor_msgs::CameraInfo>("/rgbd_receiver/depth_registered/camera_info", 1, cameraInfoCallback);
    std::cout << "START " << std::endl;
    ros::Subscriber sub_point_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/rgbd_receiver/depth_registered/points", 1, pointCloudCallback);

    std::cout << "START " << std::endl;
    g_transform_listener = new tf::TransformListener();

    std::cout << "START " << std::endl;

    ros::waitForShutdown();

    return 0;
}
