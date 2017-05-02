//#include "server.hpp"
#include <iostream>
#include <vector>
#include <fstream>
#include <list>
#include <thread>
#include <mutex>
#include <string>

#include <opencv2/opencv.hpp>
#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/timer/timer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>



#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <compressed_depth_image_transport/compressed_depth_subscriber.h>
#include <compressed_image_transport/compressed_subscriber.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Empty.h>

#include <openni2_camera/openni2_device.h>
#include <openni2_camera/openni2_device_manager.h>
#include <openni2_camera/openni2_video_mode.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


using boost::asio::ip::tcp;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> RGBDWithCameraInfoPolicy;


boost::shared_ptr<openni2_wrapper::OpenNI2Device> g_openni2_dev;

bool is_updated_color = false;
bool is_updated_depth = false;

cv::Mat g_color, g_depth;
bool g_is_sending = false;

std::mutex g_mutex;


void sendImages(cv::Mat& cur_image, cv::Mat& cur_depth, tcp::socket& socket_for_image)
{
    std::vector<int> params(2);

    params[0] = cv::IMWRITE_PNG_COMPRESSION;
    params[1] = 5;

    std::vector<int> params_for_image(2);

    params_for_image[0] = cv::IMWRITE_JPEG_QUALITY;
    params_for_image[1] = 70;


    //cv::resize(cur_image, cur_image, cv::Size(640,480));
    //cv::resize(cur_depth, cur_depth, cv::Size(640,480));
    cv::resize(cur_image, cur_image, cv::Size(320,240));
    cv::resize(cur_depth, cur_depth, cv::Size(320,240));

    std::string end_indicator("!!!END!!!");

    cv::Mat cur_depth_short;
    if(cur_depth.type() != CV_16UC1){
        cur_depth.convertTo(cur_depth_short, CV_16UC1, 1000.0);
    }
    else{
        cur_depth.copyTo(cur_depth_short);
    }

    std::vector<unsigned char> cur_image_vec;
    std::vector<unsigned char> cur_depth_vec;

    auto lambda_for_depth_encode = [&](){
        cv::imencode(".png", cur_depth_short, cur_depth_vec, params);
    };
    auto lambda_for_image_encode = [&](){
        cv::imencode(".jpeg", cur_image, cur_image_vec, params_for_image);
    };

    std::thread thread_for_image(lambda_for_image_encode);
    std::thread thread_for_depth(lambda_for_depth_encode);

    thread_for_image.join();
    thread_for_depth.join();

    //std::cout << "SESSION FOR IMAGE" << std::endl;
    //std::cout << "IMAGE SIZE: " << cur_image_vec.size() << std::endl;
    cur_image_vec.insert(cur_image_vec.end(), end_indicator.begin(), end_indicator.end());

    
    if(socket_for_image.is_open()){
        boost::system::error_code ec;
        boost::asio::write(socket_for_image, boost::asio::buffer(cur_image_vec.data(), cur_image_vec.size()), boost::asio::transfer_all(), ec);
    }
    //socket_for_image.write_some(boost::asio::buffer(cur_image_vec.data(), cur_image_vec.size()));

    //std::cout << "SESSION FOR DEPTH" << std::endl;
    //std::cout << "DEPTH SIZE: " << cur_depth_vec.size() << std::endl;
    cur_depth_vec.insert(cur_depth_vec.end(), end_indicator.begin(), end_indicator.end());
    if(socket_for_image.is_open()){
        boost::system::error_code ec;
        boost::asio::write(socket_for_image, boost::asio::buffer(cur_depth_vec.data(), cur_depth_vec.size()), boost::asio::transfer_all(), ec);

    }
    //socket_for_depth.write_some(boost::asio::buffer(cur_depth_vec.data(), cur_depth_vec.size()));
    //cv::imshow("cur_image" , cur_image);
    //cv::imshow("cur_depth" , cur_depth_short);

    //char key = cv::waitKey(1);
}




void color_callback(sensor_msgs::ImagePtr image){

    //static int count = 0;
    //if(count++ % 3 != 0) return;

    cv::Mat cur_color = cv_bridge::toCvShare(image,"bgr8")->image;
    if(g_mutex.try_lock()){
        bool is_sending = g_is_sending;
        g_mutex.unlock();

        if(!is_sending){
            cur_color.copyTo(g_color);
            is_updated_color = true;
        }
    }
    //std::cout << "COLOR CALLBACK" << std::endl;
}

void depth_callback(sensor_msgs::ImagePtr image){

    //static int count = 0;
    //if(count++ % 3 != 0) return;

    //std::cout << "DEPTH CALLBACK" << std::endl;
    cv::Mat cur_depth = cv_bridge::toCvShare(image)->image;
    if(g_mutex.try_lock()){
        bool is_sending = g_is_sending;
        g_mutex.unlock();

        if(!is_sending){
            cur_depth.copyTo(g_depth);
            is_updated_depth = true;
        }
    }
}

void enable_autofunction(const std_msgs::EmptyConstPtr& msg){
    g_openni2_dev->setAutoExposure(true);
    g_openni2_dev->setAutoWhiteBalance(true);
    std::cout << "ENABLE AUTO FUNCTION FOR OPENNI DEVICE" << std::endl;
}

void disable_autofunction(const std_msgs::EmptyConstPtr& msg){
    g_openni2_dev->setAutoExposure(false);
    g_openni2_dev->setAutoWhiteBalance(false);
    std::cout << "DISABLE AUTO FUNCTION FOR OPENNI DEVICE" << std::endl;
}


int main(int argc, char** argv){

    ros::init(argc, argv, "rgbd_sender");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    ros::NodeHandle node_handle("~");


    boost::asio::io_service io;


    std::string device_uri("");

    int port_for_image = 3213;
    int port_for_depth = 3214;

    node_handle.getParam("device_uri_", device_uri);
    node_handle.getParam("port_image_", port_for_image);
    node_handle.getParam("port_depth_", port_for_depth);



    //goodguy::session* session_for_image = NULL;
    //goodguy::session* session_for_depth = NULL;

    //goodguy::server server_for_image(io, port_for_image, &session_for_image);
    //goodguy::server server_for_depth(io, port_for_depth, &session_for_depth);
    using boost::asio::ip::tcp;

    tcp::acceptor acceptor_image(io, tcp::endpoint(tcp::v4(), port_for_image));
    tcp::acceptor acceptor_depth(io, tcp::endpoint(tcp::v4(), port_for_depth));

    boost::thread thread_for_asio([&]() { io.run(); });

    openni2_wrapper::OpenNI2DeviceManager g_openni2_dev_manager;

    if(device_uri != ""){
        std::cout << "TARGET DEVICE: " << device_uri << std::endl;
        g_openni2_dev = g_openni2_dev_manager.getDevice(device_uri);
    }
    else{
        g_openni2_dev = g_openni2_dev_manager.getAnyDevice();
    }
    g_openni2_dev->stopAllStreams();
    g_openni2_dev->setColorFrameCallback(color_callback);
    g_openni2_dev->setDepthFrameCallback(depth_callback);

    
    boost::shared_ptr<std::vector<std::string>> lists = g_openni2_dev_manager.getConnectedDeviceURIs();

    for(int i = 0; i < lists->size(); ++i){
        std::cout << "URI[" << i << "]: " << (*lists)[i] << std::endl;
    }


    const std::vector<openni2_wrapper::OpenNI2VideoMode>& supported_color_mode = g_openni2_dev->getSupportedColorVideoModes();
    const std::vector<openni2_wrapper::OpenNI2VideoMode>& supported_depth_mode = g_openni2_dev->getSupportedDepthVideoModes();

    ros::Subscriber sub_enable_autofunction = node_handle.subscribe<std_msgs::Empty>("auto_enable", 1, enable_autofunction);
    ros::Subscriber sub_disable_autofunction = node_handle.subscribe<std_msgs::Empty>("auto_disable", 1, disable_autofunction);

    for(auto mode : supported_depth_mode){
        if(mode.x_resolution_ == 640 && mode.y_resolution_ == 480){
            std::cout << "DEPTH FRAME RATE: " << mode.frame_rate_ << std::endl;
        }
        //if(mode.x_resolution_ == 640 && mode.y_resolution_ == 480 && mode.frame_rate_ == 30){
        if(mode.x_resolution_ == 320 && mode.y_resolution_ == 240 && mode.frame_rate_ == 30){
            g_openni2_dev->setDepthVideoMode(mode);
            std::cout << "SUCCESS To configure Depth sensor" << std::endl;
            break;
        }
    }
    for(auto mode : supported_color_mode){
        //if(mode.x_resolution_ == 640 && mode.y_resolution_ == 480 && mode.frame_rate_ == 30){
        if(mode.x_resolution_ == 320 && mode.y_resolution_ == 240){
            std::cout << "COLOR FRAME RATE: " << mode.frame_rate_ << std::endl;
        }
        if(mode.x_resolution_ == 320 && mode.y_resolution_ == 240 && mode.frame_rate_ == 30){
            g_openni2_dev->setColorVideoMode(mode);
            std::cout << "SUCCESS To configure Color sensor" << std::endl;
            break;
        }
    }

    g_openni2_dev->startColorStream();
    g_openni2_dev->startDepthStream();
    g_openni2_dev->setImageRegistrationMode(true);

    while(ros::ok()){
        tcp::socket socket_for_image(io);
        tcp::socket socket_for_depth(io);
        //std::cout << "ACCEPT MODE" << std::endl;
        if(!g_openni2_dev->isColorStreamStarted()){
            g_openni2_dev->startColorStream();
        }
        if(!g_openni2_dev->isDepthStreamStarted()){
            g_openni2_dev->startDepthStream();
        }
        acceptor_image.accept(socket_for_image);
        //std::cout << "IMAGE ACCEPT" << std::endl;
        //acceptor_depth.accept(socket_for_depth);
        //std::cout << "DEPTH ACCEPT" << std::endl;

        //boost::asio::ip::tcp::no_delay option(true);
        //socket_for_image.set_option(option);
        //socket_for_depth.set_option(option);
        //socket_for_image.set_option( boost::asio::socket_base::send_buffer_size( 65536 ));
        //socket_for_image.set_option( boost::asio::socket_base::receive_buffer_size( 65536 ));
        //socket_for_depth.set_option( boost::asio::socket_base::send_buffer_size( 65536 ));
        //socket_for_depth.set_option( boost::asio::socket_base::receive_buffer_size( 65536 ));

        g_mutex.lock();
        g_is_sending = true;
        g_mutex.unlock();
        if(is_updated_depth && is_updated_color){
            sendImages(g_color, g_depth, socket_for_image);
            is_updated_color = false;
            is_updated_depth = false;
        }
        g_mutex.lock();
        g_is_sending = false;
        g_mutex.unlock();

        socket_for_image.close();
        //socket_for_depth.close();
    }


    //message_filters::Subscriber<sensor_msgs::Image> rgb_image_subscriber_(nh, "/camera/rgb/image_raw", 1);
    //message_filters::Subscriber<sensor_msgs::Image> depth_image_subscriber_(nh, "/camera/depth_registered/image_raw", 1);

    //message_filters::Synchronizer<RGBDWithCameraInfoPolicy> synchronizer(RGBDWithCameraInfoPolicy(1), rgb_image_subscriber_, depth_image_subscriber_);
    //message_filters::Connection connection = synchronizer.registerCallback(boost::bind(handleImages, _1, _2, &session_for_image, &session_for_depth));

    ros::waitForShutdown();
    g_openni2_dev->stopAllStreams();

    io.stop();
    thread_for_asio.join();





    return 0;
}
