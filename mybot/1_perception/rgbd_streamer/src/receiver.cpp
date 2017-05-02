#include <iostream>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>

#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>


#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Empty.h>

using boost::asio::deadline_timer;
using boost::asio::ip::tcp;
using boost::lambda::bind;
using boost::lambda::var;
using boost::lambda::_1;

double g_fx_vga; 
double g_cx_vga; 
double g_fy_vga; 
double g_cy_vga; 

std::string g_head;
std::string g_optical_frame_name;

void publish_image_topic(const cv::Mat& image, const ros::Time& received_time, image_transport::CameraPublisher& pub_image){

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    msg->header.frame_id = g_optical_frame_name;
    msg->header.stamp = received_time;
    double scale = ((double)image.cols)/640.0;
    sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo());
    info->header.frame_id = g_optical_frame_name;
    info->header.stamp = msg->header.stamp;
    info->height = image.rows;
    info->width = image.cols;
    info->K[0] = g_fx_vga * scale;
    info->K[2] = g_cx_vga * scale;
    info->K[4] = g_fy_vga * scale;
    info->K[5] = g_cy_vga * scale; 
    info->K[8] = 1;
    info->P[0] = info->K[0];
    info->P[2] = info->K[2];
    info->P[5] = info->K[4];
    info->P[6] = info->K[5];
    info->P[10] = 1;

    if(pub_image.getNumSubscribers() > 0)
        pub_image.publish(msg, info);
}

void publish_depth_topic(const cv::Mat& depth, const ros::Time& received_time, image_transport::CameraPublisher& pub_depth){

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "16UC1", depth).toImageMsg();
    msg->header.frame_id = g_optical_frame_name;
    msg->header.stamp = received_time;
    double scale = ((double)depth.cols)/640.0;
    sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo());
    info->header.frame_id = g_optical_frame_name;
    info->header.stamp =  msg->header.stamp;
    info->height = depth.rows;
    info->width = depth.cols;
    info->K[0] = g_fx_vga * scale;
    info->K[2] = g_cx_vga * scale;
    info->K[4] = g_fy_vga * scale;
    info->K[5] = g_cy_vga * scale; 
    info->K[8] = 1;
    info->P[0] = info->K[0];
    info->P[2] = info->K[2];
    info->P[5] = info->K[4];
    info->P[6] = info->K[5];
    info->P[10] = 1;
    if(pub_depth.getNumSubscribers() > 0)
        pub_depth.publish(msg, info);
}


void check_deadline(deadline_timer* deadline, tcp::socket* socket_for_image){
//void check_deadline(deadline_timer* deadline, tcp::socket* socket_for_image, tcp::socket* socket_for_depth){

    if(deadline->expires_at() <= deadline_timer::traits_type::now()){
        socket_for_image->close();
        //socket_for_depth->close();
        std::cout << "TIMER EXPIRED" << std::endl;
        deadline->expires_at(boost::posix_time::pos_infin);
    }
    //deadline->async_wait(boost::bind(check_deadline, deadline, socket_for_image, socket_for_depth));
    deadline->async_wait(boost::bind(check_deadline, deadline, socket_for_image));

}


void run(boost::asio::io_service& io, 
                const std::string& address,
                unsigned int port_image, 
                unsigned int port_depth)

{
    tcp::resolver resolver_image(io);
    //tcp::resolver resolver_depth(io);
    tcp::resolver::query query_for_image(tcp::v4(), address, boost::lexical_cast<std::string>(port_image));
    //tcp::resolver::query query_for_depth(tcp::v4(), address, boost::lexical_cast<std::string>(port_depth));
    tcp::resolver::iterator iterator_for_image = resolver_image.resolve(query_for_image);
    //tcp::resolver::iterator iterator_for_depth = resolver_depth.resolve(query_for_depth);

    tcp::socket socket_for_image(io);
    //tcp::socket socket_for_depth(io);


    boost::posix_time::milliseconds timeout(100);
    boost::posix_time::milliseconds timeout_for_connect(1000);


    deadline_timer deadline(io);
    deadline.expires_at(boost::posix_time::pos_infin);


    deadline.async_wait(boost::bind(check_deadline, &deadline, &socket_for_image));
    //deadline.async_wait(boost::bind(check_deadline, &deadline, &socket_for_image, &socket_for_depth));

    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher pub_image = it.advertiseCamera("rgb/image_raw", 1);
    image_transport::CameraPublisher pub_depth = it.advertiseCamera("depth_registered/image_raw", 1);
    image_transport::CameraPublisher pub_depth2 = it.advertiseCamera("depth_registered_with_max/image_raw", 1);

    while(ros::ok()){
        bool is_active = false;

        try{
            deadline.expires_from_now(timeout_for_connect);
            boost::system::error_code ec = boost::asio::error::would_block;

            boost::asio::async_connect(socket_for_image,iterator_for_image, var(ec) =  boost::lambda::_1);
            //boost::asio::async_connect(socket_for_depth,iterator_for_depth, var(ec) =  boost::lambda::_1);
            //boost::asio::connect(socket_for_image, iterator_for_image);
            //boost::asio::connect(socket_for_depth, iterator_for_depth);
            do io.run_one(); while (ec == boost::asio::error::would_block);
            deadline.expires_at(boost::posix_time::pos_infin);

            if (ec || !socket_for_image.is_open())
            //if (ec || !socket_for_image.is_open() || !socket_for_depth.is_open())
            {
                //throw boost::system::system_error(
                //        ec ? ec : boost::asio::error::operation_aborted);
            }
            else{
                is_active = true;
                //std::cout << "Connection Success" << std::endl;
                //boost::asio::ip::tcp::no_delay option(true);
                //socket_for_image.set_option(option);
                //socket_for_depth.set_option(option);
                //socket_for_image.set_option( boost::asio::socket_base::send_buffer_size( 65536 ));
                //socket_for_image.set_option( boost::asio::socket_base::receive_buffer_size( 65536 ));
                //socket_for_depth.set_option( boost::asio::socket_base::send_buffer_size( 65536 ));
                //socket_for_depth.set_option( boost::asio::socket_base::receive_buffer_size( 65536 ));
            }


        }catch(std::exception& e){
            std::cerr << "Exception: " << e.what() << std::endl;
            std::cout << "Connection failed" << std::endl;
            is_active = false;
        }

        if(is_active){
            try{
                boost::system::error_code error;

                std::vector<unsigned char> received_buffer_for_image;
                std::vector<unsigned char> received_buffer_for_depth;


                //while(1)
                {
                    boost::system::error_code ec = boost::asio::error::would_block;
                    deadline.expires_from_now(timeout);
                    //std::vector<unsigned char> received_buffer(1024*1024);
                    boost::asio::streambuf received_buffer;
                    received_buffer.prepare(1024*1024*4);
                    boost::asio::async_read_until(socket_for_image, received_buffer, std::string("!!!END!!!"), var(ec) = boost::lambda::_1);
                    //size_t len = socket_for_image.read_some(, error);
                    //size_t len = socket_for_image.read_some(boost::asio::buffer(received_buffer, 65536), error);
                    //size_t len = boost::asio::read(socket_for_image, boost::asio::buffer(received_buffer), boost::asio::transfer_all(), error);
                    do io.run_one(); while (ec == boost::asio::error::would_block);
                    deadline.expires_at(boost::posix_time::pos_infin);
                    //if(ec) throw boost::system::system_error(ec);
                    //if(len != 0){
                    //    std::cout << "size_t" << len << std::endl;
                    //}
                    std::istream is(&received_buffer);
                    std::vector<char> received_data(received_buffer.size()-std::string("!!!END!!!").size());
                    is.read(received_data.data(), received_buffer.size()-std::string("!!!END!!!").size());
                    received_buffer_for_image.insert(received_buffer_for_image.end(), received_data.begin(), received_data.end());
                    //std::cout << "size_t" << received_data.size() << std::endl;

                    if(ec == boost::asio::error::eof){

                    }
                    else if(ec){
                        std::cout << "ERROR OCCURED" << std::endl;
                        throw boost::system::system_error(error);
                    }
                }

                //std::cout << "exit" << std::endl;

                //while(1)
                {
                    boost::system::error_code ec = boost::asio::error::would_block;
                    deadline.expires_from_now(timeout);
                    //std::vector<unsigned char> received_buffer(1024*1024);
                    boost::asio::streambuf received_buffer;
                    received_buffer.prepare(1024*1024*4);
                    boost::asio::async_read_until(socket_for_image, received_buffer, std::string("!!!END!!!"), var(ec) =  boost::lambda::_1);
                    do io.run_one(); while (ec == boost::asio::error::would_block);
                    deadline.expires_at(boost::posix_time::pos_infin);
                    //if(ec) throw boost::system::system_error(ec);
                    //size_t len = socket_for_depth.read_some(boost::asio::buffer(received_buffer, 65536), error);
                    //size_t len = boost::asio::read(socket_for_depth, boost::asio::buffer(received_buffer), boost::asio::transfer_all(), error);


                    std::istream is(&received_buffer);
                    std::vector<char> received_data(received_buffer.size()-std::string("!!!END!!!").size());
                    is.read(received_data.data(), received_buffer.size()-std::string("!!!END!!!").size());
                    received_buffer_for_depth.insert(received_buffer_for_depth.end(), received_data.begin(), received_data.end());
                    //std::cout << "size_t" << received_data.size() << std::endl;
                    if(ec == boost::asio::error::eof){

                    }
                    else if(ec){
                        std::cout << "ERROR OCCURED" << std::endl;
                        throw boost::system::system_error(error);
                    }
                    //if(error == boost::asio::error::eof)
                    //    break; 
                    //else if(error){
                    //    std::cout << "ERROR OCCURED" << std::endl;
                    //    throw boost::system::system_error(error);
                    //}
                }
                //std::cout << "exit2" << std::endl;
                ros::Time received_time = ros::Time::now();

                std::vector<unsigned char> encoded_data_for_image(received_buffer_for_image.begin(), received_buffer_for_image.end());
                std::vector<unsigned char> encoded_data_for_depth(received_buffer_for_depth.begin(), received_buffer_for_depth.end());
                cv::Mat image = cv::imdecode(encoded_data_for_image, CV_LOAD_IMAGE_COLOR);
                cv::Mat depth = cv::imdecode(encoded_data_for_depth, cv::IMREAD_ANYDEPTH);

                if(image.cols == 0 || image.rows == 0){
                    continue;
                }
                if(depth.cols == 0 || depth.rows == 0){
                    continue;   
                }

                cv::Mat depth_short = cv::Mat(depth.size(), CV_16UC1);
                depth.convertTo(depth_short, CV_16UC1);

                float max_distance = 4.0*1000;

                cv::Mat depth_short_with_max;
                depth_short.copyTo(depth_short_with_max);

                for(int i = 0; i < depth_short.cols; ++i){
                    for(int j = 0; j < depth_short.rows; ++j){
                        //if(depth_short.at<unsigned short>(j,i) < 0.3*1000){
                        if(depth_short.at<unsigned short>(j,i) < 0.3*1000){
                            depth_short.at<unsigned short>(j,i) = 0;
                            depth_short_with_max.at<unsigned short>(j,i) = max_distance;
                        }
                        else if(depth_short.at<unsigned short>(j,i) > max_distance){
                            depth_short_with_max.at<unsigned short>(j,i) = max_distance;
                        }

                    }
                }

                cv::Mat depth_short_median;
                //cv::medianBlur(depth_short_with_max, depth_short_median, 5); 

                socket_for_image.close();
                //socket_for_depth.close();


                publish_image_topic(image, received_time, pub_image);
                publish_depth_topic(depth_short, received_time, pub_depth);
                publish_depth_topic(depth_short_with_max, received_time, pub_depth2);


            }catch(std::exception& e){
                //std::cerr << "Exception: " << e.what() << std::endl;
                //std::cout << "READ failed" << std::endl;
                is_active = false;
            }

        }
    }

}


int main(int argc, char* argv[] ){


    ros::init(argc, argv, "rgbd_receiver");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh("~");

    std::string server_ip;
    nh.getParam("server_ip", server_ip);
    nh.getParam("fx_vga", g_fx_vga);
    nh.getParam("fy_vga", g_fy_vga);
    nh.getParam("cx_vga", g_cx_vga);
    nh.getParam("cy_vga", g_cy_vga);


    std::cout << "Try Connection to " << server_ip << std::endl;


    int port_for_image = 3213;
    int port_for_depth = 3214;

    nh.getParam("port_image_", port_for_image);
    nh.getParam("port_depth_", port_for_depth);

    nh.getParam("head_", g_head);

    if(g_head == "/head_left"){
        g_optical_frame_name = "/left_xtion_rgb_optical_frame";
    }
    else if(g_head == "/head_right"){
        g_optical_frame_name = "/right_xtion_rgb_optical_frame";
    }
    else{
        g_optical_frame_name = "/xtion_rgb_optical_frame";
    }
    boost::asio::io_service io;

    boost::thread thread_for_receiver([&]() { run(io, server_ip, port_for_image, port_for_depth); } );
    //boost::thread thread_for_asio([&]() { io.run(); });


    ros::Publisher pub_enable_autofunction = nh.advertise<std_msgs::Empty>(g_head + std::string("/rgbd_sender/auto_enable"), 1);
    ros::Publisher pub_disable_autofunction = nh.advertise<std_msgs::Empty>(g_head + std::string("/rgbd_sender/auto_disable"), 1);

    bool is_quit = false;
    while(!is_quit && ros::ok()){
        char key = 0x00;
        std::cin >> key;

        if(key == 'q'){
            is_quit = true;
        }
        else if(key == 'd'){
            std::cout << "REQUEST DISABLE AUTO FUNCTION" <<std::endl;
            std_msgs::Empty msg;
            pub_disable_autofunction.publish(msg);
        }
        else if(key == 'e'){
            std::cout << "REQUEST ENABLE AUTO FUNCTION" <<std::endl;
            std_msgs::Empty msg;
            pub_enable_autofunction.publish(msg);
        }

    }

    ros::waitForShutdown();
    std::cout << "EXIT" << std::endl;
    io.stop();
    //thread_for_asio.join();

    return 0;
}
