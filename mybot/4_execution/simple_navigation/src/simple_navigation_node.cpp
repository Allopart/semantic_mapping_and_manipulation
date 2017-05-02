#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <memory>

#include <urdf/model.h>


#include <iostream>
#include <vector>
#include <string>

#include <trajectory_msgs/JointTrajectory.h>


#include <decision_maker/ActionResult.h>
#include <decision_maker/ApproachToObjectAction.h>
#include <decision_maker/GraspObjectAction.h>
#include <decision_maker/MoveToObjectAction.h>
#include <decision_maker/ReleaseAction.h>
#include <decision_maker/TiltObjectAction.h>

#include <simple_navigation/GoTarget.h>


tf::TransformListener* g_tf_listener;

ros::Publisher g_pub_mobile;

bool g_is_quit = false;

double g_target_x = 0.0;
double g_target_y = 0.0;
double g_target_theta = 0.0;



boost::mutex mutex;

template <typename T>
T getInput()
{
    T result;
    std::cin >> result;
    if (std::cin.fail() || std::cin.get() != '\n') {
        std::cin.clear();
        while (std::cin.get() != '\n');
        throw std::ios_base::failure("Invalid input.");
    }
    return result;
}



void update_target_pose(double x, double y, double theta){
    mutex.lock();
    g_target_x = x;
    g_target_y = y;
    g_target_theta = theta;
    mutex.unlock();
}


bool get_curr_pose(Eigen::Affine3d& eigen_curr_pose){
    tf::StampedTransform curr_pose;
    try{
        g_tf_listener->lookupTransform("/map", "/base_footprint", ros::Time(0), curr_pose);
        tf::poseTFToEigen(curr_pose, eigen_curr_pose);
        return true;
    } catch (tf::TransformException& ex){
        std::cout << "TRANSFOMR EXCEPTION" << std::endl;
        return false;
    }
}

void run(bool* is_quit){
    ros::Duration period(0.2);

    const double max_linear_velocity = 0.10;
    const double max_angular_velocity = 30.0*3.14/180.0;

    bool is_first = true;

    while(!*is_quit){
        Eigen::Affine3d curr_pose;
        if(!get_curr_pose(curr_pose)){
            period.sleep();
            continue;
        }
        Eigen::Matrix2d rotate_matrix_z = curr_pose.matrix().block<2,2>(0,0);

        double curr_theta = std::atan2(rotate_matrix_z.coeff(1,0), rotate_matrix_z.coeff(0,0));
        double curr_x = (curr_pose.translation())(0);
        double curr_y = (curr_pose.translation())(1);

        //std::cout << "X: " << curr_x << "\tY: " << curr_y << "\tTheta: " << curr_theta*180.0/3.14 << std::endl;
        mutex.lock();
        double error_target_x = g_target_x-curr_x;
        double error_target_y = g_target_y-curr_y;
        double error_target_theta = g_target_theta-curr_theta;     
        while(error_target_theta >  3.14) error_target_theta -= 3.14*2;
        while(error_target_theta < -3.14) error_target_theta += 3.14*2;        
        mutex.unlock();


        Eigen::Vector2d error_target;
        error_target << error_target_x, error_target_y;
        Eigen::Vector2d rotated_error_target = rotate_matrix_z.inverse()*error_target;


        double vel_x = rotated_error_target(0);
        double vel_y = rotated_error_target(1);
        double vel_theta = error_target_theta;


        if(vel_x > max_linear_velocity) vel_x = max_linear_velocity;
        else if(vel_x < -max_linear_velocity) vel_x = -max_linear_velocity;

        if(vel_y > max_linear_velocity) vel_y = max_linear_velocity;
        else if(vel_y < -max_linear_velocity) vel_y = -max_linear_velocity;

        if(vel_theta > max_angular_velocity) vel_theta = max_angular_velocity;
        else if(vel_theta < -max_angular_velocity) vel_theta = -max_angular_velocity;

        geometry_msgs::Twist msg;

        msg.linear.x = vel_x;
        msg.linear.y = 0;
        msg.linear.y = vel_y;
        msg.angular.z = vel_theta;

        g_pub_mobile.publish(msg);

        period.sleep();
    }
}


void go_target(double target_x, double target_y, double target_theta){

    ros::Duration period(0.5);
    update_target_pose(target_x, target_y, target_theta);

    const double min_error_trans = 0.02;
    const double min_error_rotate = 5.0*3.14/180.0;

    double error_trans = 10.0;
    double error_rotate = 10.0;

    bool is_quit = false;
    boost::thread thread_for_navi(boost::bind(run, &is_quit));
    do{
        period.sleep();
        Eigen::Affine3d curr_pose;
        if(!get_curr_pose(curr_pose)){
            continue;
        }
        Eigen::Matrix2d rotate_matrix_z = curr_pose.matrix().block<2,2>(0,0);

        double curr_theta = std::atan2(rotate_matrix_z.coeff(1,0), rotate_matrix_z.coeff(0,0));
        double curr_x = (curr_pose.translation())(0);
        double curr_y = (curr_pose.translation())(1);
        error_trans = std::sqrt((curr_x - target_x)*(curr_x - target_x)+(curr_y - target_y)*(curr_y - target_y));
        error_rotate = std::abs(curr_theta-target_theta);
    }while(error_trans > min_error_trans || error_rotate > min_error_rotate);
    is_quit = true;
    thread_for_navi.join();
}

bool go_target_service(simple_navigation::GoTarget::Request  &req, simple_navigation::GoTarget::Response &res){
    std::cout << "Go Position: " << req.pose << std::endl;
    go_target(req.pose.x, req.pose.y, req.pose.theta);
    res.result = true;
    return true;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "simple_navigation_node");

    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::NodeHandle node_handle("~");

    g_tf_listener = new tf::TransformListener();
    g_pub_mobile = node_handle.advertise<geometry_msgs::Twist>("/mobile_controller/cmd_vel", 1);



    ros::ServiceServer service = node_handle.advertiseService("go_target", go_target_service);

    while(!g_is_quit){
        char key;
        try{
            key = getInput<char>();
        } catch( std::exception& e){
            key = 0x00;
            std::cout << "Wrong key input" << std::endl;
        }


        switch(key){
            case 'q': 
                {
                    g_is_quit = true; 
                    break;
                }
            case '1':
                {
                    go_target(-0.1, 0, 0);
                    go_target(-0.7, 0.5, +90*3.14/180.0);
                    go_target(-0.7, 0.6, +90*3.14/180.0);
                    break;
                }
            case '2':
                {
                    go_target(-0.7, 0.5, +90*3.14/180.0);
                    go_target(-0.1, 0, 0);
                    go_target(0, 0, 0);
                    break;
                }
            case 'p':
                {
                    Eigen::Affine3d curr_pose;
                    if(!get_curr_pose(curr_pose)){
                        break;
                    }
                    Eigen::Matrix2d rotate_matrix_z = curr_pose.matrix().block<2,2>(0,0);

                    double curr_theta = std::atan2(rotate_matrix_z.coeff(1,0), rotate_matrix_z.coeff(0,0));
                    double curr_x = (curr_pose.translation())(0);
                    double curr_y = (curr_pose.translation())(1);

                    std::cout << "X: " << curr_x << "\tY: " << curr_y << "\tTheta: " << curr_theta*180.0/3.14 << std::endl;
                    break;
                }
            case 'a':
                {
                    Eigen::Affine3d curr_pose;
                    if(!get_curr_pose(curr_pose)){
                        break;
                    }
                    Eigen::Matrix2d rotate_matrix_z = curr_pose.matrix().block<2,2>(0,0);

                    double curr_theta = std::atan2(rotate_matrix_z.coeff(1,0), rotate_matrix_z.coeff(0,0));
                    double curr_x = (curr_pose.translation())(0);
                    double curr_y = (curr_pose.translation())(1);

                    std::cout << "X: " << curr_x << "\tY: " << curr_y << "\tTheta: " << curr_theta*180.0/3.14 << std::endl;

                    std::cout << "Enter x(m): " ;
                    double x = getInput<double>();
                    std::cout << "Enter y(m): " ;
                    double y = getInput<double>();
                    std::cout << "Enter theta(degree): " ;
                    double theta = getInput<double>();
                    go_target(x, y, theta*3.14/180.0);
                    break;
                }
        }
    }



    ros::waitForShutdown();
    g_is_quit = true;
    delete g_tf_listener;

    return 0;
}
