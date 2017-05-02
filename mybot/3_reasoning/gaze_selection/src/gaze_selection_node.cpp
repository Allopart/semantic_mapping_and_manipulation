#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <memory>

#include <urdf/model.h>


#include <iostream>
#include <vector>
#include <string>

#include <trajectory_msgs/JointTrajectory.h>
#include <gaze_selection/PanTilt.h>

tf::TransformListener* g_tf_listener;

ros::Publisher g_pub_head;

std::string g_head;


trajectory_msgs::JointTrajectory getSingleJointTrajectoryMsg(const std::vector<std::string>& names, const std::vector<double>& angles, double time_from_start = 0.1){
    trajectory_msgs::JointTrajectory msg;
    if(names.size() != angles.size())   return msg;

    msg.header.stamp = ros::Time::now() + ros::Duration(0.1);
    msg.header.frame_id = "base_footprint";
    msg.points.resize(1);   // Only contain a single joint trajectory
    for(std::size_t i = 0; i < names.size(); ++i){
        msg.joint_names.push_back(names[i]);
        msg.points[0].positions.push_back(angles[i]);
    }
    msg.points[0].velocities.resize(names.size(),0);
    msg.points[0].accelerations.resize(names.size(),0);
    msg.points[0].effort.resize(names.size(),0);
    msg.points[0].time_from_start = ros::Duration(time_from_start);

    return msg;
}

trajectory_msgs::JointTrajectory getJointTrajectoryMsg(const std::vector<double>& angles, double time_from_start = 0.2){
    std::vector<std::string> names(4);
    if(g_head == "/head_right"){
        names[0] = "right_head_joint_0";
        names[1] = "right_head_joint_1";
        names[2] = "right_head_joint_2";
        names[3] = "right_head_joint_3";
    }
    else if(g_head == "/head_left"){
        names[0] = "left_head_joint_0";
        names[1] = "left_head_joint_1";
        names[2] = "left_head_joint_2";
        names[3] = "left_head_joint_3";
    }
    else{
        names[0] = "head_joint_0";
        names[1] = "head_joint_1";
        names[2] = "head_joint_2";
        names[3] = "head_joint_3";
    }
    return getSingleJointTrajectoryMsg(names, angles, time_from_start);
}

void move_head_to_target_angle(double target_head_angle_0, double target_head_angle_1){
    std::vector<double> angles(4,0);
    angles[0] = target_head_angle_0;
    angles[1] = target_head_angle_1;
    angles[2] = 0;
    angles[3] = 0;
    g_pub_head.publish(getJointTrajectoryMsg(angles,0.2));
}


void move_gaze_to_point(const Eigen::Vector3d point){
    geometry_msgs::PointStamped base_point;
    geometry_msgs::PointStamped transformed_base_point;
    base_point.point.x = point(0);
    base_point.point.y = point(1);
    base_point.point.z = point(2);
    base_point.header.stamp = ros::Time(0);
    base_point.header.frame_id = "/base_footprint";

    try{
        if(g_head == "/head_right"){
            g_tf_listener->transformPoint("/right_head_mount", ros::Time(0), base_point, base_point.header.frame_id, transformed_base_point);
        }
        else if(g_head == "/head_left"){
            g_tf_listener->transformPoint("/left_head_mount", ros::Time(0), base_point, base_point.header.frame_id, transformed_base_point);
        }
        else{
            g_tf_listener->transformPoint("/head_mount", ros::Time(0), base_point, base_point.header.frame_id, transformed_base_point);
        }

        const double x = transformed_base_point.point.x;
        const double y = transformed_base_point.point.y;
        const double z = transformed_base_point.point.z;

        double theta0 = std::atan2(y, x);
        if(g_head == "/head_right" && theta0 > 90.0* 3.14/180.0){
            theta0 -= 360.0*3.14/180.0;
        }
        else if(g_head == "/head_left" && theta0 < -90.0* 3.14/180.0){
            theta0 += 360.0*3.14/180.0;
        }

if(g_head=="/head_right")
	std::cout << "value: " << theta0 << " / " << base_point << std::endl;

        double theta1 = -std::atan2(z, std::sqrt(x*x + y*y));
         
        move_head_to_target_angle(theta0, theta1);


    } catch (tf::TransformException& ex){
        std::cout << "TRANSFOMR EXCEPTION" << std::endl;
        return;
    }
}

void pan_tilt_callback(const gaze_selection::PanTiltConstPtr msg){
    move_head_to_target_angle(msg->pan_angle.data, msg->tilt_angle.data);
}
void gaze_point_callback(const geometry_msgs::PointConstPtr msg){
    Eigen::Vector3d gaze_point;
    gaze_point(0) = msg->x;
    gaze_point(1) = msg->y;
    gaze_point(2) = msg->z;
    move_gaze_to_point(gaze_point);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "gaze_selection_node");

    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::NodeHandle node_handle("~");

    node_handle.getParam("head_", g_head);

    g_tf_listener = new tf::TransformListener();

    ros::Subscriber sub_gaze_point = node_handle.subscribe<geometry_msgs::Point>(g_head+std::string("/gaze_point"), 1, gaze_point_callback);
    ros::Subscriber sub_pan_tilt_angle = node_handle.subscribe<gaze_selection::PanTilt>(g_head+std::string("/pan_tilt_angle"), 1, pan_tilt_callback);

    if(g_head == "/head_right"){
        g_pub_head = node_handle.advertise<trajectory_msgs::JointTrajectory>("/head_right_controller/command", 1);
    }
    else if(g_head == "/head_left"){
        g_pub_head = node_handle.advertise<trajectory_msgs::JointTrajectory>("/head_left_controller/command", 1);
    }
    else{
        g_pub_head = node_handle.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 1);
    }


    ros::waitForShutdown();
    delete g_tf_listener;

    return 0;
}
