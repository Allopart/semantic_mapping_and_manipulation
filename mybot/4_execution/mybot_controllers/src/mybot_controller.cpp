#include <ros/ros.h>
#include "pugixml.hpp"
#include "server.hpp"
#include <Eigen/Eigen>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <thread>         
#include <chrono>         
#include <sstream>

#include <mutex>

#include "../modules/controller/motor.hpp"
#include "../modules/controller/packet.hpp"

#include "RobotHead_withFace.hpp"



template <class SOCKET>
class Mybot : public hardware_interface::RobotHW
{
    public:
        Mybot(ros::NodeHandle& node_handle, ros::Publisher& joint_pub, tf::TransformBroadcaster& tf_broadcaster, SOCKET& socket_for_arms, SOCKET& socket_for_hand, SOCKET& socket_for_head, SOCKET& socket_for_mobile) 
            : m_node_handle(node_handle), m_joint_pub(joint_pub), m_tf_broadcaster(tf_broadcaster), m_socket_for_arms(socket_for_arms), m_socket_for_hand(socket_for_hand), m_socket_for_head(socket_for_head), m_robot_head(socket_for_head), m_socket_for_mobile(socket_for_mobile), m_is_active(true), m_is_torque_enable_for_arm_right(true), m_is_torque_enable_for_arm_left(true) 
        { 


            std::vector<std::string> joint_names_for_arms;
            joint_names_for_arms.push_back("waist_joint_0");
            joint_names_for_arms.push_back("waist_joint_1");
            joint_names_for_arms.push_back("right_shoulder_joint_0");
            joint_names_for_arms.push_back("right_shoulder_joint_1");
            joint_names_for_arms.push_back("right_shoulder_joint_2");
            joint_names_for_arms.push_back("right_shoulder_joint_3");
            joint_names_for_arms.push_back("right_elbow_joint_0");
            joint_names_for_arms.push_back("left_shoulder_joint_0");
            joint_names_for_arms.push_back("left_shoulder_joint_1");
            joint_names_for_arms.push_back("left_shoulder_joint_2");
            joint_names_for_arms.push_back("left_shoulder_joint_3");
            joint_names_for_arms.push_back("left_elbow_joint_0");

            std::vector<std::string> joint_names_for_hand;
            joint_names_for_hand.push_back("right_wrist_joint_0");
            joint_names_for_hand.push_back("right_wrist_joint_1");
            joint_names_for_hand.push_back("right_wrist_joint_2");
            joint_names_for_hand.push_back("right_gripper_thumb_joint_0");
            joint_names_for_hand.push_back("right_gripper_forefinger_joint_0");
            joint_names_for_hand.push_back("right_gripper_littlefinger_joint_0");
            joint_names_for_hand.push_back("left_wrist_joint_0");
            joint_names_for_hand.push_back("left_wrist_joint_1");
            joint_names_for_hand.push_back("left_wrist_joint_2");
            joint_names_for_hand.push_back("left_gripper_thumb_joint_0");
            joint_names_for_hand.push_back("left_gripper_forefinger_joint_0");
            joint_names_for_hand.push_back("left_gripper_littlefinger_joint_0");

            std::vector<std::string> joint_names_for_head;
            joint_names_for_head.push_back("head_joint_0");
            joint_names_for_head.push_back("head_joint_1");
            joint_names_for_head.push_back("head_joint_2");
            joint_names_for_head.push_back("head_joint_3");
			
            std::vector<std::string> joint_names_for_mobile;
            joint_names_for_mobile.push_back("wheel_joint_0");
            joint_names_for_mobile.push_back("wheel_joint_1");
            joint_names_for_mobile.push_back("wheel_joint_2");

            std::vector<std::string> joint_names_for_bottle_mouth;
            joint_names_for_bottle_mouth.push_back("right_bottle_mouth_joint_x");
            joint_names_for_bottle_mouth.push_back("right_bottle_mouth_joint_y");
            joint_names_for_bottle_mouth.push_back("right_bottle_mouth_joint_z");
            joint_names_for_bottle_mouth.push_back("left_bottle_mouth_joint_x");
            joint_names_for_bottle_mouth.push_back("left_bottle_mouth_joint_y");
            joint_names_for_bottle_mouth.push_back("left_bottle_mouth_joint_z");




            m_cmd_for_arms.resize(joint_names_for_arms.size(),0);
            m_cmd_vel_for_arms.resize(joint_names_for_arms.size(),0);
            m_cmd_prev_for_arms.resize(joint_names_for_arms.size(),0);
            m_pos_for_arms.resize(joint_names_for_arms.size());
            m_vel_for_arms.resize(joint_names_for_arms.size());
            m_eff_for_arms.resize(joint_names_for_arms.size());


            m_cmd_for_hand.resize(joint_names_for_hand.size(),0);
            m_cmd_vel_for_hand.resize(joint_names_for_hand.size(),0);
            m_cmd_prev_for_hand.resize(joint_names_for_hand.size(),0);
            m_pos_for_hand.resize(joint_names_for_hand.size());
            m_vel_for_hand.resize(joint_names_for_hand.size());
            m_eff_for_hand.resize(joint_names_for_hand.size());

            m_cmd_for_head.resize(joint_names_for_head.size(),0);
            m_cmd_vel_for_head.resize(joint_names_for_head.size(),0);
            m_cmd_prev_for_head.resize(joint_names_for_head.size(),0);
            m_pos_for_head.resize(joint_names_for_head.size());
            m_vel_for_head.resize(joint_names_for_head.size());
            m_eff_for_head.resize(joint_names_for_head.size());

            m_cmd_for_mobile.resize(joint_names_for_mobile.size(),0);
            m_cmd_vel_for_mobile.resize(joint_names_for_mobile.size(),0);
            m_cmd_prev_for_mobile.resize(joint_names_for_mobile.size(),0);
            m_pos_for_mobile.resize(joint_names_for_mobile.size());
            m_vel_for_mobile.resize(joint_names_for_mobile.size());
            m_eff_for_mobile.resize(joint_names_for_mobile.size());


            m_cmd_for_bottle_mouth.resize(joint_names_for_bottle_mouth.size(),0);
            m_cmd_vel_for_bottle_mouth.resize(joint_names_for_bottle_mouth.size(),0);
            m_cmd_prev_for_bottle_mouth.resize(joint_names_for_bottle_mouth.size(),0);
            m_pos_for_bottle_mouth.resize(joint_names_for_bottle_mouth.size());
            m_vel_for_bottle_mouth.resize(joint_names_for_bottle_mouth.size());
            m_eff_for_bottle_mouth.resize(joint_names_for_bottle_mouth.size());

            // for facial expression
            m_lip_angles.resize(4, 0);
            m_chin_angles.resize(4, 0);
            m_eyebrow_angles.resize(4, 0);


            for(auto it = joint_names_for_arms.begin(); it != joint_names_for_arms.end(); ++it){
                std::size_t index = it - joint_names_for_arms.begin();
                hardware_interface::JointStateHandle state_handle(it->c_str(), &m_pos_for_arms[index], &m_vel_for_arms[index], &m_eff_for_arms[index]);
                jnt_state_interface.registerHandle(state_handle);
                ROS_INFO("%s\n",it->c_str());
            }
            for(auto it = joint_names_for_hand.begin(); it != joint_names_for_hand.end(); ++it){
                std::size_t index = it - joint_names_for_hand.begin();
                hardware_interface::JointStateHandle state_handle(it->c_str(), &m_pos_for_hand[index], &m_vel_for_hand[index], &m_eff_for_hand[index]);
                jnt_state_interface.registerHandle(state_handle);
                ROS_INFO("%s\n",it->c_str());
            }
            for(auto it = joint_names_for_head.begin(); it != joint_names_for_head.end(); ++it){
                std::size_t index = it - joint_names_for_head.begin();
                hardware_interface::JointStateHandle state_handle(it->c_str(), &m_pos_for_head[index], &m_vel_for_head[index], &m_eff_for_head[index]);
                jnt_state_interface.registerHandle(state_handle);
                ROS_INFO("%s\n",it->c_str());
            }
            for(auto it = joint_names_for_mobile.begin(); it != joint_names_for_mobile.end(); ++it){
                std::size_t index = it - joint_names_for_mobile.begin();
                hardware_interface::JointStateHandle state_handle(it->c_str(), &m_pos_for_mobile[index], &m_vel_for_mobile[index], &m_eff_for_mobile[index]);
                jnt_state_interface.registerHandle(state_handle);
                ROS_INFO("%s\n",it->c_str());
            }
            for(auto it = joint_names_for_bottle_mouth.begin(); it != joint_names_for_bottle_mouth.end(); ++it){
                std::size_t index = it - joint_names_for_bottle_mouth.begin();
                hardware_interface::JointStateHandle state_handle(it->c_str(), &m_pos_for_bottle_mouth[index], &m_vel_for_bottle_mouth[index], &m_eff_for_bottle_mouth[index]);
                jnt_state_interface.registerHandle(state_handle);
                ROS_INFO("%s\n",it->c_str());
            }
            registerInterface(&jnt_state_interface);

            for(auto it = joint_names_for_mobile.begin(); it != joint_names_for_mobile.end(); ++it){
                std::size_t index = it - joint_names_for_mobile.begin();
                hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(it->c_str()), &m_cmd_vel_for_mobile[index]);
                jnt_vel_interface.registerHandle(vel_handle);
            }
            registerInterface(&jnt_vel_interface);


            for(auto it = joint_names_for_arms.begin(); it != joint_names_for_arms.end(); ++it){
                std::size_t index = it - joint_names_for_arms.begin();
                hardware_interface::PosVelJointHandle pos_vel_handle(jnt_state_interface.getHandle(it->c_str()), &m_cmd_for_arms[index], &m_cmd_vel_for_arms[index]);
                jnt_pos_vel_interface.registerHandle(pos_vel_handle);
            }
            for(auto it = joint_names_for_hand.begin(); it != joint_names_for_hand.end(); ++it){
                std::size_t index = it - joint_names_for_hand.begin();
                hardware_interface::PosVelJointHandle pos_vel_handle(jnt_state_interface.getHandle(it->c_str()), &m_cmd_for_hand[index], &m_cmd_vel_for_hand[index]);
                jnt_pos_vel_interface.registerHandle(pos_vel_handle);
            }
            for(auto it = joint_names_for_head.begin(); it != joint_names_for_head.end(); ++it){
                std::size_t index = it - joint_names_for_head.begin();
                hardware_interface::PosVelJointHandle pos_vel_handle(jnt_state_interface.getHandle(it->c_str()), &m_cmd_for_head[index], &m_cmd_vel_for_head[index]);
                jnt_pos_vel_interface.registerHandle(pos_vel_handle);
            }
            registerInterface(&jnt_pos_vel_interface);

            for(auto it = joint_names_for_bottle_mouth.begin(); it != joint_names_for_bottle_mouth.end(); ++it){
                std::size_t index = it - joint_names_for_bottle_mouth.begin();
                hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(it->c_str()), &m_cmd_for_bottle_mouth[index]);
                jnt_pos_interface.registerHandle(pos_handle);
            }
            registerInterface(&jnt_pos_interface);


            //set_position();
            //torque_on();

            //registerInterface(&act_to_jnt_state);
            //registerInterface(&jnt_to_act_pos);

            m_motors_for_arms.push_back(goodguy::Motor(0, goodguy::Motor::DYNAMIXEL_PRO));
            m_motors_for_arms.push_back(goodguy::Motor(1, goodguy::Motor::DYNAMIXEL_PRO));


            m_motors_for_arms.push_back(goodguy::Motor(2, goodguy::Motor::DYNAMIXEL_PRO));
            m_motors_for_arms.push_back(goodguy::Motor(3, goodguy::Motor::DYNAMIXEL_PRO));
            m_motors_for_arms.push_back(goodguy::Motor(4, goodguy::Motor::DYNAMIXEL_PRO));
            m_motors_for_arms.push_back(goodguy::Motor(5, goodguy::Motor::DYNAMIXEL_PRO));
            m_motors_for_arms.push_back(goodguy::Motor(6, goodguy::Motor::DYNAMIXEL_PRO));
            m_motors_for_arms.push_back(goodguy::Motor(13, goodguy::Motor::DYNAMIXEL_PRO));
            m_motors_for_arms.push_back(goodguy::Motor(14, goodguy::Motor::DYNAMIXEL_PRO));
            m_motors_for_arms.push_back(goodguy::Motor(15, goodguy::Motor::DYNAMIXEL_PRO));
            m_motors_for_arms.push_back(goodguy::Motor(16, goodguy::Motor::DYNAMIXEL_PRO));
            m_motors_for_arms.push_back(goodguy::Motor(17, goodguy::Motor::DYNAMIXEL_PRO));

            m_motors_for_hand.push_back(goodguy::Motor(7, goodguy::Motor::DYNAMIXEL_MX64));
            m_motors_for_hand.push_back(goodguy::Motor(8, goodguy::Motor::DYNAMIXEL_MX64));
            m_motors_for_hand.push_back(goodguy::Motor(9, goodguy::Motor::DYNAMIXEL_MX64));
            m_motors_for_hand.push_back(goodguy::Motor(10, goodguy::Motor::DYNAMIXEL_MX28));
            m_motors_for_hand.push_back(goodguy::Motor(11, goodguy::Motor::DYNAMIXEL_MX28));
            m_motors_for_hand.push_back(goodguy::Motor(12, goodguy::Motor::DYNAMIXEL_MX28));
            m_motors_for_hand.push_back(goodguy::Motor(18, goodguy::Motor::DYNAMIXEL_MX64));
            m_motors_for_hand.push_back(goodguy::Motor(19, goodguy::Motor::DYNAMIXEL_MX64));
            m_motors_for_hand.push_back(goodguy::Motor(20, goodguy::Motor::DYNAMIXEL_MX64));
            m_motors_for_hand.push_back(goodguy::Motor(21, goodguy::Motor::DYNAMIXEL_MX28));
            m_motors_for_hand.push_back(goodguy::Motor(22, goodguy::Motor::DYNAMIXEL_MX28));
            m_motors_for_hand.push_back(goodguy::Motor(23, goodguy::Motor::DYNAMIXEL_MX28));



            m_is_angle_updated_for_arms.resize(m_motors_for_arms.size(), false);
            m_is_angle_updated_for_hand.resize(m_motors_for_hand.size(), false);
            m_is_angle_updated_for_head.resize(m_motors_for_head.size(), false);



            std::vector<unsigned char> packet;
            goodguy::makeStatusReturnSettingPacket(packet, goodguy::Motor::DYNAMIXEL_PRO);
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_arms);
            m_socket_for_arms.write_some(boost::asio::buffer(m_send_buffer_for_arms, packet.size()));

            packet.clear();
            for(int i = 0; i < m_motors_for_arms.size(); ++i){
                m_motors_for_arms[i].enable();
                std::vector<unsigned char> p1;
                goodguy::makeMotorPacket(p1, m_motors_for_arms[i]);
                packet.insert(packet.end(), p1.begin(), p1.end());
            }
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_arms);
            m_socket_for_arms.write_some(boost::asio::buffer(m_send_buffer_for_arms, packet.size()));
            goodguy::makeActionPacket(packet, goodguy::Motor::DYNAMIXEL_PRO);
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_arms);
            m_socket_for_arms.write_some(boost::asio::buffer(m_send_buffer_for_arms, packet.size()));

            packet.clear();
            for(int i = 0; i < m_motors_for_hand.size(); ++i){
                m_motors_for_hand[i].enable();
                std::vector<unsigned char> p1;
                goodguy::makeMotorPacket(p1, m_motors_for_hand[i]);
                packet.insert(packet.end(), p1.begin(), p1.end());
            }
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_hand);
            m_socket_for_hand.write_some(boost::asio::buffer(m_send_buffer_for_hand, packet.size()));
            goodguy::makeActionPacket(packet, goodguy::Motor::DYNAMIXEL_MX64);
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_hand);
            m_socket_for_hand.write_some(boost::asio::buffer(m_send_buffer_for_hand, packet.size()));

            const double maximum_rpm = 0.418*3.0/2.0/3.14*60.0;
            packet.clear();
            for(int i = 0; i < m_motors_for_arms.size(); ++i){
                m_motors_for_arms[i].enable();

                const double gear_ratio_for_waist_1 = 78.0/24.0;
                std::vector<unsigned char> packet;

                if(m_motors_for_arms[i].getMotorId() == 1){
                    m_motors_for_arms[i].setGoalSpeedWithUpdate(maximum_rpm*gear_ratio_for_waist_1);
                }
                else{
                    m_motors_for_arms[i].setGoalSpeedWithUpdate(maximum_rpm);
                }
                std::vector<unsigned char> p1;

                goodguy::makeMotorPacket(p1, m_motors_for_arms[i]);
                packet.insert(packet.end(), p1.begin(), p1.end());
            }
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_arms);
            m_socket_for_arms.write_some(boost::asio::buffer(m_send_buffer_for_arms, packet.size()));
            goodguy::makeActionPacket(packet, goodguy::Motor::DYNAMIXEL_PRO);
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_arms);
            m_socket_for_arms.write_some(boost::asio::buffer(m_send_buffer_for_arms, packet.size()));

            packet.clear();
            for(int i = 0; i < m_motors_for_hand.size(); ++i){
                m_motors_for_hand[i].enable();
                std::vector<unsigned char> p1;
                goodguy::makeMotorPacket(p1, m_motors_for_hand[i]);
                packet.insert(packet.end(), p1.begin(), p1.end());
            }
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_hand);
            m_socket_for_hand.write_some(boost::asio::buffer(m_send_buffer_for_hand, packet.size()));
            goodguy::makeActionPacket(packet, goodguy::Motor::DYNAMIXEL_MX64);
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_hand);
            m_socket_for_hand.write_some(boost::asio::buffer(m_send_buffer_for_hand, packet.size()));


            // for facial expression
            m_lip_sub = m_node_handle.subscribe<std_msgs::Float64MultiArray>("/facial_expression/lip_cmd", 100, boost::bind(&Mybot::handle_write_for_lip, this, _1));
            m_chin_sub = m_node_handle.subscribe<std_msgs::Float64MultiArray>("/facial_expression/chin_cmd", 100, boost::bind(&Mybot::handle_write_for_chin, this, _1));
            m_eyebrow_sub = m_node_handle.subscribe<std_msgs::Float64MultiArray>("/facial_expression/eyebrow_cmd", 100, boost::bind(&Mybot::handle_write_for_eyebrow, this, _1));

            m_torque_enable_head_sub = m_node_handle.subscribe<std_msgs::Bool>("/torque_enable/head", 100, boost::bind(&Mybot::handle_torque_enable_head, this, _1));
            m_torque_enable_arm_left_sub = m_node_handle.subscribe<std_msgs::Bool>("/torque_enable/arm_left", 100, boost::bind(&Mybot::handle_torque_enable_arm_left, this, _1));
            m_torque_enable_arm_right_sub = m_node_handle.subscribe<std_msgs::Bool>("/torque_enable/arm_right", 100, boost::bind(&Mybot::handle_torque_enable_arm_right, this, _1));

            m_lip_pub = m_node_handle.advertise<std_msgs::Float64MultiArray>("/facial_expression/lip_pos", 100);
            m_chin_pub = m_node_handle.advertise<std_msgs::Float64MultiArray>("/facial_expression/chin_pos", 100);
            m_eyebrow_pub = m_node_handle.advertise<std_msgs::Float64MultiArray>("/facial_expression/eyebrow_pos", 100);


            m_socket_for_arms.async_read_some(boost::asio::buffer(m_recv_buffer_for_arms,1024), boost::bind(&Mybot::handle_read_for_arms, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
            m_socket_for_hand.async_read_some(boost::asio::buffer(m_recv_buffer_for_hand,1024), boost::bind(&Mybot::handle_read_for_hand, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
            m_socket_for_mobile.async_read_some(boost::asio::buffer(m_recv_buffer_for_mobile,1024), boost::bind(&Mybot::handle_read_for_mobile, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

            // from RobotHead.hpp
            std::copy(m_robot_head.sendPacket.begin(), m_robot_head.sendPacket.end(), m_send_buffer_for_head);
            m_socket_for_head.write_some(boost::asio::buffer(m_send_buffer_for_head, m_robot_head.sendPacket.size()));
			
			// for facial expression
            std::copy(m_robot_head.sendPacket_face_0.begin(), m_robot_head.sendPacket_face_0.end(), m_send_buffer_for_head_face_0);
            m_socket_for_head.write_some(boost::asio::buffer(m_send_buffer_for_head_face_0, m_robot_head.sendPacket_face_0.size()));
            std::copy(m_robot_head.sendPacket_face_1.begin(), m_robot_head.sendPacket_face_1.end(), m_send_buffer_for_head_face_1);
            m_socket_for_head.write_some(boost::asio::buffer(m_send_buffer_for_head_face_1, m_robot_head.sendPacket_face_1.size()));
            std::copy(m_robot_head.sendPacket_face_2.begin(), m_robot_head.sendPacket_face_2.end(), m_send_buffer_for_head_face_2);
            m_socket_for_head.write_some(boost::asio::buffer(m_send_buffer_for_head_face_2, m_robot_head.sendPacket_face_2.size()));
	

            m_socket_for_head.async_read_some(boost::asio::buffer(m_recv_buffer_for_head,1024), boost::bind(&Mybot::handle_read_for_head, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

        }

        void handle_read_for_arms(const boost::system::error_code& error, size_t bytes_transferred){
            //std::cout << "HANDLE READ: " << bytes_transferred << std::endl;
            // When receive packet through serial port
            if(!error){
                m_received_packet_buffer_for_arms.reserve(m_received_packet_buffer_for_arms.size()+bytes_transferred);
                for(std::size_t i = 0; i < bytes_transferred; ++i){
                    m_received_packet_buffer_for_arms.push_back(m_recv_buffer_for_arms[i]);
                }

                m_socket_for_arms.async_read_some(boost::asio::buffer(m_recv_buffer_for_arms,1024), boost::bind(&Mybot::handle_read_for_arms, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
            }
            else{
                std::cout<< "error on read " << std::endl;
                m_is_active = false;
            }
        }
        void handle_read_for_hand(const boost::system::error_code& error, size_t bytes_transferred){
            //std::cout << "HANDLE READ: " << bytes_transferred << std::endl;
            // When receive packet through serial port
            if(!error){
                //m_received_packet_buffer_for_hand.reserve(m_received_packet_buffer_for_hand.size()+bytes_transferred);
                for(std::size_t i = 0; i < bytes_transferred; ++i){
                    m_received_packet_buffer_for_hand.push_back(m_recv_buffer_for_hand[i]);
                }

                m_socket_for_hand.async_read_some(boost::asio::buffer(m_recv_buffer_for_hand,1024), boost::bind(&Mybot::handle_read_for_hand, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
            }
            else{
                std::cout<< "error on read " << std::endl;
                m_is_active = false;
            }
        }
        void handle_read_for_mobile(const boost::system::error_code& error, size_t bytes_transferred){
            //std::cout << "HANDLE READ: " << bytes_transferred << std::endl;
            // When receive packet through serial port
            if(!error){
                m_received_packet_buffer_for_mobile.reserve(m_received_packet_buffer_for_mobile.size()+bytes_transferred);
                for(std::size_t i = 0; i < bytes_transferred; ++i){
                    m_received_packet_buffer_for_mobile.push_back(m_recv_buffer_for_mobile[i]);
                }

                m_socket_for_mobile.async_read_some(boost::asio::buffer(m_recv_buffer_for_mobile,1024), boost::bind(&Mybot::handle_read_for_mobile, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
            }
            else{
                std::cout<< "error on read " << std::endl;
                m_is_active = false;
            }
        }
        // from RobotHead.hpp
        void handle_read_for_head(const boost::system::error_code& error, size_t bytes_transferred){
            //std::cout << "HANDLE READ: " << bytes_transferred << std::endl;
            // When receive packet through serial port
            if(!error){

                m_robot_head.fullPacket.insert(m_robot_head.fullPacket.end(), m_recv_buffer_for_head, m_recv_buffer_for_head+bytes_transferred);

                if (m_robot_head.readPacket(m_robot_head.fullPacket, m_robot_head.recvPacket, m_robot_head.header)) {
                    for (auto i=0; i<m_robot_head.recvAngle.size(); i++){
                        m_robot_head.recvAngle[i] = (m_robot_head.recvPacket[3+i*2])+(m_robot_head.recvPacket[4+i*2]<<8);
                    }
                }

                m_received_packet_buffer_for_head.reserve(m_received_packet_buffer_for_head.size()+bytes_transferred);
                for(std::size_t i = 0; i < bytes_transferred; ++i){
                    m_received_packet_buffer_for_head.push_back(m_recv_buffer_for_head[i]);
                }

                m_socket_for_head.async_read_some(boost::asio::buffer(m_recv_buffer_for_head,1024), boost::bind(&Mybot::handle_read_for_head, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
            }
            else{
                std::cout<< "error on read " << std::endl;
                m_is_active = false;
            }
        }

        void read(){
            read_for_arms();
            read_for_hand();
            read_for_head();
            read_for_mobile();

        }


        void read_for_mobile(){

            //std::cout << "READ : " <<  m_received_packet_buffer.size() << std::endl;
            std::vector<std::string> splited_packets;
            boost::split(splited_packets, m_received_packet_buffer_for_mobile, boost::is_any_of(";"));
            m_received_packet_buffer_for_mobile.clear();

            //std::cout << "SPLITED: " << splited_packets.size() << std::endl;

            const unsigned int  base_enc = 5*1000*1000;
            const float resolution = 1024.0;
            const float gear_ratio = 4.0*30.0;



            for(auto it = splited_packets.begin(); it != (splited_packets.end()-1); ++it){
                std::vector<unsigned char> pk(it->begin(), it->end());
                if(pk.size() > 2 && (pk[0] == 'R' && pk[1] == 'N')){
                    std::vector<unsigned char> encoder_data(pk.begin()+2, pk.end());
                    std::vector<std::string> splited_encoder_packets;
                    boost::split(splited_encoder_packets, encoder_data, boost::is_any_of(","));

                    if(splited_encoder_packets.size() != 3){
                        continue;
                    }

                    std::vector<unsigned int> encoder_uint_data(splited_encoder_packets.size());
                    for(std::size_t i = 0; i < encoder_uint_data.size(); ++i){
                        std::stringstream ss;
                        ss << std::hex << splited_encoder_packets[i];
                        ss >> encoder_uint_data[i];
                    }

                    if(m_wheel_start_pos.size() == 0){
                        for(std::size_t i = 0; i < encoder_uint_data.size(); ++i){
                            int enc_val = ((int)encoder_uint_data[i]-(int)base_enc);
                            float angle = ((float)enc_val)/resolution/gear_ratio*2.0*3.14;
                            m_wheel_start_pos.push_back(angle);
                        }
                    }

                    for(std::size_t i = 0; i < encoder_uint_data.size(); ++i){
                        int enc_val = ((int)encoder_uint_data[i]-(int)base_enc);
                        float angle = ((float)enc_val)/resolution/gear_ratio*2.0*3.14;
                        m_pos_for_mobile[i] = angle - m_wheel_start_pos[i];
                    }

                }
            }



        }



        void read_for_arms(){
            //std::cout << "READ : " <<  m_received_packet_buffer.size() << std::endl;
            std::string header;
            header.push_back(0xff);
            header.push_back(0xff);
            header.push_back(0xfd);

            std::vector<std::string> splited_packets;
            boost::split_regex(splited_packets, m_received_packet_buffer_for_arms, boost::regex(header));
            m_received_packet_buffer_for_arms.clear();

            //std::cout << "SPLITED: " << splited_packets.size() << std::endl;

            for(auto it = splited_packets.begin(); it != splited_packets.end(); ++it){
                std::vector<unsigned char> pk(header.begin(), header.end());
                pk.insert(pk.end(), it->begin(), it->end());
                //goodguy::printPacket(pk.begin(), pk.end());

                try{

                    auto lambda_read_packet = [&](std::vector<unsigned char>& raw_packet){
                        std::vector<unsigned char> packet_data;
                        int packet_id = 0;
                        goodguy::DynamixelProStatusType packet_error;
                        if(goodguy::getDataForDynamixelPro(packet_data, packet_id, packet_error, raw_packet)){
                            goodguy::DefaultBulkState state = getDefaultBulkStateFromStream(packet_data, goodguy::Motor::DYNAMIXEL_PRO);
                            //std::cout << "ID: " << packet_id << "\tANG: " << state.m_angle << "\tVEL: " << state.m_speed << "\tTORQUE: " << state.m_torque << std::endl;
                            for(int i = 0; i < m_motors_for_arms.size(); ++i){
                                if(packet_id == 1 && (m_motors_for_arms[i].getMotorId() == packet_id)){
                                    const double gear_ratio_for_waist_1 = 78.0/24.0;
                                    m_pos_for_arms[i] = (state.m_angle/gear_ratio_for_waist_1)*3.14/180.0;
                                    m_is_angle_updated_for_arms[i] = true;
                                    if(m_is_torque_enable_for_arm_right && m_motors_for_arms[i].getMotorId() >=2 && m_motors_for_arms[i].getMotorId() <=9){
                                        m_cmd_for_arms[i] = m_pos_for_arms[i];
                                    }
                                    else if(m_is_torque_enable_for_arm_left && m_motors_for_arms[i].getMotorId() >=13 && m_motors_for_arms[i].getMotorId() <=20){
                                        m_cmd_for_arms[i] = m_pos_for_arms[i];
                                    }
                                    break;
                                }
                                else if(m_motors_for_arms[i].getMotorId() == packet_id){
                                    m_pos_for_arms[i] = state.m_angle*3.14/180.0;
                                    m_is_angle_updated_for_arms[i] = true;
                                    if(m_is_torque_enable_for_arm_right && m_motors_for_arms[i].getMotorId() >=2 && m_motors_for_arms[i].getMotorId() <=9){
                                        m_cmd_for_arms[i] = m_pos_for_arms[i];
                                    }
                                    else if(m_is_torque_enable_for_arm_left && m_motors_for_arms[i].getMotorId() >=13 && m_motors_for_arms[i].getMotorId() <=20){
                                        m_cmd_for_arms[i] = m_pos_for_arms[i];
                                    }
                                    if(m_motors_for_arms[i].getMotorId() == 4){

                                    }
                                    break;
                                }
                            }
                        }
                    };
                    lambda_read_packet(pk);
                } catch (goodguy::InvalidPacketException& e){
                    // Nothing to do
                }

            }


        }

        void read_for_hand(){
            //std::cout << "READ : " <<  m_received_packet_buffer.size() << std::endl;
            std::string header;
            header.push_back(0xff);
            header.push_back(0xff);


            std::vector<std::string> splited_packets;
            boost::split_regex(splited_packets, m_received_packet_buffer_for_hand, boost::regex(header));
            m_received_packet_buffer_for_hand.clear();

            //std::cout << "SPLITED: " << splited_packets.size() << std::endl;
            for(auto it = splited_packets.begin(); it != splited_packets.end(); ++it){
                //goodguy::printPacket(it->begin(), it->end());
                std::vector<unsigned char> pk(header.begin(), header.end());
                pk.insert(pk.end(), it->begin(), it->end());

                if(pk.size() >= 22){
                    pk.resize(21);
                }
                //std::cout << "SIZE: "<<pk.size() << std::endl;
                //goodguy::printPacket(pk.begin(), pk.end());
                try{

                    auto lambda_read_packet = [&](std::vector<unsigned char>& raw_packet){
                        std::vector<unsigned char> packet_data;
                        int packet_id = 0;
                        goodguy::DynamixelMXStatusType packet_error;
                        if(goodguy::getDataForDynamixelMX(packet_data, packet_id, packet_error, raw_packet)){
                            goodguy::DefaultBulkState state = getDefaultBulkStateFromStream(packet_data, goodguy::Motor::DYNAMIXEL_MX64);
                            //std::cout << "ID: " << packet_id << "\tANG: " << state.m_angle << "\tVEL: " << state.m_speed << "\tTORQUE: " << state.m_torque << std::endl;
                            for(int i = 0; i < m_motors_for_hand.size(); ++i){
                                if(m_motors_for_hand[i].getMotorId() == packet_id){
                                    m_pos_for_hand[i] = state.m_angle*3.14/180.0;
                                    m_is_angle_updated_for_hand[i] = true;
                                    if(m_is_torque_enable_for_arm_right && m_motors_for_hand[i].getMotorId() >=2 && m_motors_for_hand[i].getMotorId() <=9){
                                        m_cmd_for_hand[i] = m_pos_for_hand[i];
                                    }
                                    else if(m_is_torque_enable_for_arm_left && m_motors_for_hand[i].getMotorId() >=13 && m_motors_for_hand[i].getMotorId() <=20){
                                        m_cmd_for_hand[i] = m_pos_for_hand[i];
                                    }
                                    break;
                                }
                            }
                        }

                    };
                    lambda_read_packet(pk);
                } catch (goodguy::InvalidPacketException& e){
                    // Nothing to do
                }
            }



        }

        void read_for_head(){

            std::vector<double> angles = m_robot_head.getCurrentAngles();

            for(int i = 0; i < angles.size(); ++i){
                m_pos_for_head[i] = angles[i];
            }

            // for facial expression
            std_msgs::Float64MultiArray msg;
            
            msg.data = m_lip_angles;
            m_lip_pub.publish(msg);

            msg.data = m_chin_angles;
            m_chin_pub.publish(msg);

            msg.data = m_eyebrow_angles;
            m_eyebrow_pub.publish(msg);

        }

        bool is_activated() const {
            auto lambda_for_check_update = [](const std::vector<bool>& input){
                for(int i = 0; i < input.size(); ++i){
                    if(!input[i])    return false;
                }
                return true;
            };

            if(!lambda_for_check_update(m_is_angle_updated_for_arms)){
                std::cout << "ARMS IS NOT INITIALIZED" << std::endl;
                return false;
            }
            if(!lambda_for_check_update(m_is_angle_updated_for_hand)){
                std::cout << "HAND IS NOT INITIALIZED" << std::endl;
                return false;
            }
            if(!lambda_for_check_update(m_is_angle_updated_for_head)){
                std::cout << "HEAD IS NOT INITIALIZED" << std::endl;
                return false;
            }
            return true;
        }

        void initialize_angles_with_current(){
            auto lambda_for_init = [](const std::vector<double>& a, std::vector<double>& b, std::vector<double>& c, std::vector<double>& d){
                for(int i = 0; i < a.size(); ++i){
                    b[i] = a[i];
                    c[i] = a[i];
                    d[i] = a[i];
                }
            };

            lambda_for_init(m_pos_for_arms, m_cmd_for_arms, m_cmd_prev_for_arms, m_cmd_for_arms);
            lambda_for_init(m_pos_for_hand, m_cmd_for_hand, m_cmd_prev_for_hand, m_cmd_for_hand);
            lambda_for_init(m_pos_for_head, m_cmd_for_head, m_cmd_prev_for_head, m_cmd_for_head);
        }


        void request_angles(){
            std::vector<unsigned char> packet;
            goodguy::makeDefaultBulkReadPacket(packet, m_motors_for_arms, goodguy::Motor::DYNAMIXEL_PRO);
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_arms);
            m_socket_for_arms.write_some(boost::asio::buffer(m_send_buffer_for_arms, packet.size()));

            packet.clear();
            goodguy::makeDefaultBulkReadPacket(packet, m_motors_for_hand, goodguy::Motor::DYNAMIXEL_MX64);
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_hand);
            m_socket_for_hand.write_some(boost::asio::buffer(m_send_buffer_for_hand, packet.size()));


        }

        void request_encoder(){
            std::string packet("CN;");
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_mobile);
            m_socket_for_mobile.write_some(boost::asio::buffer(m_send_buffer_for_mobile, packet.size()));
        }
        void write(){
            std::lock_guard<std::mutex> lck(m_mutex_for_sub);

            //m_cmd_vel_for_hand[3] = 1.0;
            //m_cmd_vel_for_hand[4] = 1.0; 
            //m_cmd_vel_for_hand[5] = 1.0; 
            //m_cmd_vel_for_hand[9] = 1.0;
            //m_cmd_vel_for_hand[10] = 1.0;
            //m_cmd_vel_for_hand[11] = 1.0;

            for(int i = 0; i < m_motors_for_arms.size(); ++i){
                //std::cout << m_cmd_for_arms[i] << "[" << m_cmd_vel_for_arms[i] << "]" << "  ";
            }
            //std::cout << std::endl;

            write_for_hand();
            write_for_arms();
            write_for_head();
            write_for_mobile();
            request_angles();

            //connect();
            //disconnect();
        }

        void write_for_mobile(){

            std::string packet("CD");
            double maximum_rpm = 2700; // 0.1 rpm
            float gear_ratio = 30.0;

            for(std::size_t i = 0; i < m_cmd_vel_for_mobile.size(); ++i){
                int rpm = m_cmd_vel_for_mobile[i]*60.0/(2.0*3.14159)*gear_ratio;

                if(std::abs(rpm) > maximum_rpm){
                    if(rpm < 0) rpm = -maximum_rpm;
                    else        rpm =  maximum_rpm;
                }

                packet = packet + boost::lexical_cast<std::string>(rpm);
                if(i == m_cmd_vel_for_mobile.size()-1){
                    packet += std::string(";");
                }
                else{
                    packet += std::string(",");
                }
            }

            //std::cout << "Mobile Packet: " << packet << std::endl;

            std::copy(packet.begin(), packet.end(), m_send_buffer_for_mobile);
            m_socket_for_mobile.write_some(boost::asio::buffer(m_send_buffer_for_mobile, packet.size()));



        }



        void write_for_arms(){

            const double gear_ratio_for_waist_1 = 78.0/24.0;
            std::vector<unsigned char> packet;
            double minimum_rpm = 0.5; // 0.1 rpm

            for(int i = 0; i < m_motors_for_arms.size(); ++i){
                // 2~9: arm_right
                // 13~20: arm_left
                if(!m_is_torque_enable_for_arm_right && m_motors_for_arms[i].getMotorId() >=2 && m_motors_for_arms[i].getMotorId() <=9){
                    continue;
                }
                else if(!m_is_torque_enable_for_arm_left && m_motors_for_arms[i].getMotorId() >=13 && m_motors_for_arms[i].getMotorId() <=20){
                    continue;
                }

                if(m_motors_for_arms[i].getMotorId() == 4){

                }


                if(m_motors_for_arms[i].getMotorId() == 1){
                    m_motors_for_arms[i].setGoalSpeedWithUpdate((std::abs(m_cmd_vel_for_arms[i]/2.0/3.14*60.0)*1.4+minimum_rpm)*gear_ratio_for_waist_1);
                }
                else{
                    m_motors_for_arms[i].setGoalSpeedWithUpdate(std::abs(m_cmd_vel_for_arms[i]/2.0/3.14*60.0)*1.4+minimum_rpm);
                }

                std::vector<unsigned char> p1;
                goodguy::makeMotorPacket(p1, m_motors_for_arms[i]);
                packet.insert(packet.end(), p1.begin(), p1.end());
            }
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_arms);
            m_socket_for_arms.write_some(boost::asio::buffer(m_send_buffer_for_arms, packet.size()));
            goodguy::makeActionPacket(packet, goodguy::Motor::DYNAMIXEL_PRO);
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_arms);
            m_socket_for_arms.write_some(boost::asio::buffer(m_send_buffer_for_arms, packet.size()));

            packet.clear();
            for(int i = 0; i < m_motors_for_arms.size(); ++i){
                // 2~9: arm_right
                // 13~20: arm_left
                if(!m_is_torque_enable_for_arm_right && m_motors_for_arms[i].getMotorId() >=2 && m_motors_for_arms[i].getMotorId() <=9){
                    continue;
                }
                else if(!m_is_torque_enable_for_arm_left && m_motors_for_arms[i].getMotorId() >=13 && m_motors_for_arms[i].getMotorId() <=20){
                    continue;
                }

                if(m_motors_for_arms[i].getMotorId() == 1){
                    m_motors_for_arms[i].setGoalAngleWithUpdate(m_cmd_for_arms[i]*gear_ratio_for_waist_1*180.0/3.14);
                }
                else{
                    m_motors_for_arms[i].setGoalAngleWithUpdate(m_cmd_for_arms[i]*180.0/3.14);
                }
                std::vector<unsigned char> p1;
                goodguy::makeMotorPacket(p1, m_motors_for_arms[i]);
                packet.insert(packet.end(), p1.begin(), p1.end());
            }

            std::copy(packet.begin(), packet.end(), m_send_buffer_for_arms);
            m_socket_for_arms.write_some(boost::asio::buffer(m_send_buffer_for_arms, packet.size()));
            goodguy::makeActionPacket(packet, goodguy::Motor::DYNAMIXEL_PRO);
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_arms);
            m_socket_for_arms.write_some(boost::asio::buffer(m_send_buffer_for_arms, packet.size()));


        }




        void write_for_hand(){

            std::vector<unsigned char> packet;
            double minimum_rpm = 0.5; // 0.1 rpm

            for(int i = 0; i < m_motors_for_hand.size(); ++i){
                // 2~9: arm_right
                // 13~20: arm_left
                if(!m_is_torque_enable_for_arm_right && m_motors_for_hand[i].getMotorId() >=2 && m_motors_for_hand[i].getMotorId() <=9){
                    continue;
                }
                else if(!m_is_torque_enable_for_arm_left && m_motors_for_hand[i].getMotorId() >=13 && m_motors_for_hand[i].getMotorId() <=20){
                    continue;
                }

                m_motors_for_hand[i].setGoalSpeedWithUpdate(std::abs(m_cmd_vel_for_hand[i]/2.0/3.14*60.0)*1.4+minimum_rpm);
                std::vector<unsigned char> p1;
                goodguy::makeMotorPacket(p1, m_motors_for_hand[i]);
                packet.insert(packet.end(), p1.begin(), p1.end());
            }
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_hand);
            m_socket_for_hand.write_some(boost::asio::buffer(m_send_buffer_for_hand, packet.size()));
            goodguy::makeActionPacket(packet, goodguy::Motor::DYNAMIXEL_MX64);
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_hand);
            m_socket_for_hand.write_some(boost::asio::buffer(m_send_buffer_for_hand, packet.size()));
            packet.clear();
            for(int i = 0; i < m_motors_for_hand.size(); ++i){
                // 2~9: arm_right
                // 13~20: arm_left
                if(!m_is_torque_enable_for_arm_right && m_motors_for_hand[i].getMotorId() >=2 && m_motors_for_hand[i].getMotorId() <=9){
                    continue;
                }
                else if(!m_is_torque_enable_for_arm_left && m_motors_for_hand[i].getMotorId() >=13 && m_motors_for_hand[i].getMotorId() <=20){
                    continue;
                }


                m_motors_for_hand[i].setGoalAngleWithUpdate(m_cmd_for_hand[i]*180.0/3.14);
                std::vector<unsigned char> p1;
                goodguy::makeMotorPacket(p1, m_motors_for_hand[i]);
                packet.insert(packet.end(), p1.begin(), p1.end());
            }
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_hand);
            m_socket_for_hand.write_some(boost::asio::buffer(m_send_buffer_for_hand, packet.size()));
            goodguy::makeActionPacket(packet, goodguy::Motor::DYNAMIXEL_MX64);
            std::copy(packet.begin(), packet.end(), m_send_buffer_for_hand);
            m_socket_for_hand.write_some(boost::asio::buffer(m_send_buffer_for_hand, packet.size()));


            for(std::size_t i = 0; i < m_pos_for_bottle_mouth.size(); ++i){
                m_pos_for_bottle_mouth[i] = m_cmd_for_bottle_mouth[i];
                m_vel_for_bottle_mouth[i] = m_cmd_vel_for_bottle_mouth[i];
            }


        }

        void write_for_head(){

            if(m_cmd_for_head.size() != 4)  return;

            // from RobotHead.hpp
            m_robot_head.setCurrentAngles(m_cmd_for_head, m_robot_head.min_angles_dc, m_robot_head.max_angles_dc);
            std::copy(m_robot_head.sendPacket.begin(), m_robot_head.sendPacket.end(), m_send_buffer_for_head);
            m_socket_for_head.write_some(boost::asio::buffer(m_send_buffer_for_head, m_robot_head.sendPacket.size()));			
            //std::cout << "packet for head: \t" ;
            //goodguy::printPacket(m_robot_head.sendPacket.begin(), m_robot_head.sendPacket.end());

            // for facial expression
            m_robot_head.setCurrentAngles_face(m_lip_angles, m_robot_head.init_face_0, m_robot_head.min_angles_servo, m_robot_head.max_angles_servo, m_robot_head.sendAngle_face_0, m_robot_head.sendPacket_face_0);
            std::copy(m_robot_head.sendPacket_face_0.begin(), m_robot_head.sendPacket_face_0.end(), m_send_buffer_for_head_face_0);
            m_socket_for_head.write_some(boost::asio::buffer(m_send_buffer_for_head_face_0, m_robot_head.sendPacket_face_0.size()));
            //std::cout << "packet for lip: \t" ;
            //goodguy::printPacket(m_robot_head.sendPacket_face_0.begin(), m_robot_head.sendPacket_face_0.end());

            m_robot_head.setCurrentAngles_face(m_chin_angles, m_robot_head.init_face_1, m_robot_head.min_angles_servo, m_robot_head.max_angles_servo, m_robot_head.sendAngle_face_1, m_robot_head.sendPacket_face_1);
            std::copy(m_robot_head.sendPacket_face_1.begin(), m_robot_head.sendPacket_face_1.end(), m_send_buffer_for_head_face_1);
            m_socket_for_head.write_some(boost::asio::buffer(m_send_buffer_for_head_face_1, m_robot_head.sendPacket_face_1.size()));
            //std::cout << "packet for chin: \t" ;
            //goodguy::printPacket(m_robot_head.sendPacket_face_1.begin(), m_robot_head.sendPacket_face_1.end());

            m_robot_head.setCurrentAngles_face(m_eyebrow_angles, m_robot_head.init_face_2, m_robot_head.min_angles_servo, m_robot_head.max_angles_servo, m_robot_head.sendAngle_face_2, m_robot_head.sendPacket_face_2);
            std::copy(m_robot_head.sendPacket_face_2.begin(), m_robot_head.sendPacket_face_2.end(), m_send_buffer_for_head_face_2);
            m_socket_for_head.write_some(boost::asio::buffer(m_send_buffer_for_head_face_2, m_robot_head.sendPacket_face_2.size()));
            //std::cout << "packet for eyebrow: \t" ;
            //goodguy::printPacket(m_robot_head.sendPacket_face_2.begin(), m_robot_head.sendPacket_face_2.end());

     }


    // for facial expression
    void handle_write_for_lip(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        if(msg->data.size() == 4) {
            for(std::size_t i = 0; i < m_lip_angles.size(); ++i) {
                m_lip_angles[i] = msg->data[i];
            }
        } else {
            std::cout << "error on cmd read for lip" << std::endl;
        }
    }

    void handle_torque_enable_head(const std_msgs::Bool::ConstPtr& msg){
        std::cout << "Torque Enable for Head" << std::endl;
    }

    void disable_arm(bool is_right){
        std::vector<unsigned char> packet;
        for(int i = 0; i < m_motors_for_arms.size(); ++i){
            // 2~9: arm_right
            // 13~20: arm_left
            if(is_right && m_motors_for_arms[i].getMotorId() >=2 && m_motors_for_arms[i].getMotorId() <=9){
            }
            else if(!is_right && m_motors_for_arms[i].getMotorId() >=13 && m_motors_for_arms[i].getMotorId() <=20){
            }
            else{
                continue;
            }
            m_motors_for_arms[i].disable();
            std::vector<unsigned char> p1;
            goodguy::makeMotorPacket(p1, m_motors_for_arms[i]);
            packet.insert(packet.end(), p1.begin(), p1.end());
        }
        std::copy(packet.begin(), packet.end(), m_send_buffer_for_arms);
        m_socket_for_arms.write_some(boost::asio::buffer(m_send_buffer_for_arms, packet.size()));
        goodguy::makeActionPacket(packet, goodguy::Motor::DYNAMIXEL_PRO);
        std::copy(packet.begin(), packet.end(), m_send_buffer_for_arms);
        m_socket_for_arms.write_some(boost::asio::buffer(m_send_buffer_for_arms, packet.size()));

        packet.clear();
        for(int i = 0; i < m_motors_for_hand.size(); ++i){
            if(is_right && m_motors_for_hand[i].getMotorId() >=2 && m_motors_for_hand[i].getMotorId() <=9){
            }
            else if(!is_right && m_motors_for_hand[i].getMotorId() >=13 && m_motors_for_hand[i].getMotorId() <=20){
            }
            else{
                continue;
            }
            m_motors_for_hand[i].disable();
            std::vector<unsigned char> p1;
            goodguy::makeMotorPacket(p1, m_motors_for_hand[i]);
            packet.insert(packet.end(), p1.begin(), p1.end());
        }
        std::copy(packet.begin(), packet.end(), m_send_buffer_for_hand);
        m_socket_for_hand.write_some(boost::asio::buffer(m_send_buffer_for_hand, packet.size()));
        goodguy::makeActionPacket(packet, goodguy::Motor::DYNAMIXEL_MX64);
        std::copy(packet.begin(), packet.end(), m_send_buffer_for_hand);
        m_socket_for_hand.write_some(boost::asio::buffer(m_send_buffer_for_hand, packet.size()));
    }
    void enable_arm(bool is_right){
        std::vector<unsigned char> packet;
        for(int i = 0; i < m_motors_for_arms.size(); ++i){
            // 2~9: arm_right
            // 13~20: arm_left
            if(is_right && m_motors_for_arms[i].getMotorId() >=2 && m_motors_for_arms[i].getMotorId() <=9){
            }
            else if(!is_right && m_motors_for_arms[i].getMotorId() >=13 && m_motors_for_arms[i].getMotorId() <=20){
            }
            else{
                continue;
            }
            m_motors_for_arms[i].enable();
            std::vector<unsigned char> p1;
            goodguy::makeMotorPacket(p1, m_motors_for_arms[i]);
            packet.insert(packet.end(), p1.begin(), p1.end());
        }
        std::copy(packet.begin(), packet.end(), m_send_buffer_for_arms);
        m_socket_for_arms.write_some(boost::asio::buffer(m_send_buffer_for_arms, packet.size()));
        goodguy::makeActionPacket(packet, goodguy::Motor::DYNAMIXEL_PRO);
        std::copy(packet.begin(), packet.end(), m_send_buffer_for_arms);
        m_socket_for_arms.write_some(boost::asio::buffer(m_send_buffer_for_arms, packet.size()));

        packet.clear();
        for(int i = 0; i < m_motors_for_hand.size(); ++i){
            if(is_right && m_motors_for_hand[i].getMotorId() >=2 && m_motors_for_hand[i].getMotorId() <=9){
            }
            else if(!is_right && m_motors_for_hand[i].getMotorId() >=13 && m_motors_for_hand[i].getMotorId() <=20){
            }
            else{
                continue;
            }
            m_motors_for_hand[i].enable();
            std::vector<unsigned char> p1;
            goodguy::makeMotorPacket(p1, m_motors_for_hand[i]);
            packet.insert(packet.end(), p1.begin(), p1.end());
        }
        std::copy(packet.begin(), packet.end(), m_send_buffer_for_hand);
        m_socket_for_hand.write_some(boost::asio::buffer(m_send_buffer_for_hand, packet.size()));
        goodguy::makeActionPacket(packet, goodguy::Motor::DYNAMIXEL_MX64);
        std::copy(packet.begin(), packet.end(), m_send_buffer_for_hand);
        m_socket_for_hand.write_some(boost::asio::buffer(m_send_buffer_for_hand, packet.size()));
    }


    void handle_torque_enable_arm_left(const std_msgs::Bool::ConstPtr& msg){
        std::lock_guard<std::mutex> lck(m_mutex_for_sub);
        m_is_torque_enable_for_arm_left = msg->data;
        if(msg->data){
            std::cout << "Torque Enabled for Arm Left" << std::endl;
            enable_arm(false);
        }
        else{
            std::cout << "Torque Disabled for Arm Left" << std::endl;
            disable_arm(false);
        }
        ros::Duration(0.1).sleep();
    }
    void handle_torque_enable_arm_right(const std_msgs::Bool::ConstPtr& msg){
        std::lock_guard<std::mutex> lck(m_mutex_for_sub);
        m_is_torque_enable_for_arm_right = msg->data;
        if(msg->data){
            std::cout << "Torque Enabled for Arm Right" << std::endl;
            enable_arm(true);
        }
        else{
            std::cout << "Torque Disabled for Arm Right" << std::endl;
            disable_arm(true);
        }
        ros::Duration(0.1).sleep();
    }
    
    void handle_write_for_chin(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        if(msg->data.size() == 4) {
            for(std::size_t i = 0; i < m_chin_angles.size(); ++i) {
                m_chin_angles[i] = msg->data[i];
            }
        } else {
            std::cout << "error on cmd read for chin" << std::endl;
        }
    }
    void handle_write_for_eyebrow(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        if(msg->data.size() == 4) {
            for(std::size_t i = 0; i < m_eyebrow_angles.size(); ++i) {
                m_eyebrow_angles[i] = msg->data[i];
            }
        } else {
            std::cout << "error on cmd read for eyebrow" << std::endl;
        }   
    }


    private:

        ros::NodeHandle& m_node_handle;
        ros::Publisher& m_joint_pub;

        tf::TransformBroadcaster& m_tf_broadcaster;

        SOCKET& m_socket_for_arms;
        SOCKET& m_socket_for_hand;

        SOCKET& m_socket_for_mobile;

        SOCKET& m_socket_for_head;


        plaincode::RobotHead<SOCKET> m_robot_head;

        std::vector<unsigned char> m_received_packet_buffer_for_arms;
        unsigned char m_recv_buffer_for_arms[1024];
        unsigned char m_send_buffer_for_arms[1024];

        std::vector<unsigned char> m_received_packet_buffer_for_hand;
        unsigned char m_recv_buffer_for_hand[1024];
        unsigned char m_send_buffer_for_hand[1024];

        std::vector<unsigned char> m_received_packet_buffer_for_head;
        unsigned char m_recv_buffer_for_head[1024];
        unsigned char m_send_buffer_for_head[1024];
		
		// for facial expression
		unsigned char m_send_buffer_for_head_face_0[1024];
		unsigned char m_send_buffer_for_head_face_1[1024];
		unsigned char m_send_buffer_for_head_face_2[1024];

        std::vector<unsigned char> m_received_packet_buffer_for_mobile;
        unsigned char m_recv_buffer_for_mobile[1024];
        unsigned char m_send_buffer_for_mobile[1024];

 
        bool m_is_active;


        bool m_is_torque_enable_for_arm_right;
        bool m_is_torque_enable_for_arm_left;

        std::vector<goodguy::Motor> m_motors_for_arms;
        std::vector<goodguy::Motor> m_motors_for_hand;
        std::vector<goodguy::Motor> m_motors_for_head;


        hardware_interface::JointStateInterface                  jnt_state_interface;
        hardware_interface::PositionJointInterface               jnt_pos_interface;
        hardware_interface::VelocityJointInterface               jnt_vel_interface;
        hardware_interface::PosVelJointInterface                 jnt_pos_vel_interface;


        std::vector<double> m_cmd_for_arms;
        std::vector<double> m_cmd_vel_for_arms;
        std::vector<double> m_cmd_prev_for_arms;
        std::vector<double> m_pos_for_arms;
        std::vector<double> m_vel_for_arms;
        std::vector<double> m_eff_for_arms;

        std::vector<double> m_cmd_for_hand;
        std::vector<double> m_cmd_vel_for_hand;
        std::vector<double> m_cmd_prev_for_hand;
        std::vector<double> m_pos_for_hand;
        std::vector<double> m_vel_for_hand;
        std::vector<double> m_eff_for_hand;

        std::vector<double> m_cmd_for_head;
        std::vector<double> m_cmd_vel_for_head;
        std::vector<double> m_cmd_prev_for_head;
        std::vector<double> m_pos_for_head;
        std::vector<double> m_vel_for_head;
        std::vector<double> m_eff_for_head;

        std::vector<double> m_cmd_for_mobile;
        std::vector<double> m_cmd_vel_for_mobile;
        std::vector<double> m_cmd_prev_for_mobile;
        std::vector<double> m_pos_for_mobile;
        std::vector<double> m_vel_for_mobile;
        std::vector<double> m_eff_for_mobile;

        std::vector<double> m_cmd_for_bottle_mouth;
        std::vector<double> m_cmd_vel_for_bottle_mouth;
        std::vector<double> m_cmd_prev_for_bottle_mouth;
        std::vector<double> m_pos_for_bottle_mouth;
        std::vector<double> m_vel_for_bottle_mouth;
        std::vector<double> m_eff_for_bottle_mouth;

        std::vector<double> m_wheel_start_pos;


        std::vector<bool> m_is_angle_updated_for_arms;
        std::vector<bool> m_is_angle_updated_for_hand;
        std::vector<bool> m_is_angle_updated_for_head;


        // for facial expression
        ros::Subscriber m_lip_sub;
        ros::Subscriber m_chin_sub;
        ros::Subscriber m_eyebrow_sub;

        ros::Subscriber m_torque_enable_head_sub;
        ros::Subscriber m_torque_enable_arm_left_sub;
        ros::Subscriber m_torque_enable_arm_right_sub;

        ros::Publisher m_lip_pub;
        ros::Publisher m_chin_pub;
        ros::Publisher m_eyebrow_pub;

        std::vector<double> m_lip_angles;
        std::vector<double> m_chin_angles;
        std::vector<double> m_eyebrow_angles;

        std::mutex m_mutex_for_sub;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mybot_controller");


    ros::AsyncSpinner spinner(3);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    //ros::Rate rate(1.0);
    ROS_INFO("Start Controller With Serial COMM");

    ros::NodeHandle node_handle;


    boost::asio::io_service io;
    boost::asio::serial_port serial_for_arms(io);
    serial_for_arms.open("/dev/arm");
    serial_for_arms.set_option(boost::asio::serial_port::baud_rate(2000000UL));

    boost::asio::serial_port serial_for_hand(io);
    serial_for_hand.open("/dev/hand");
    serial_for_hand.set_option(boost::asio::serial_port::baud_rate(1000000UL));
    serial_for_hand.set_option(boost::asio::serial_port::flow_control());
    serial_for_hand.set_option(boost::asio::serial_port::stop_bits());
    serial_for_hand.set_option(boost::asio::serial_port::character_size());
    serial_for_hand.set_option(boost::asio::serial_port::parity());


    boost::asio::serial_port serial_for_head(io);
    serial_for_head.open("/dev/head");
    serial_for_head.set_option(boost::asio::serial_port::baud_rate(115200));

    ros::Publisher joint_pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster tf_broadcaster;

    boost::asio::serial_port serial_for_mobile(io);
    serial_for_mobile.open("/dev/mobile");
    serial_for_mobile.set_option(boost::asio::serial_port::baud_rate(115200));

    
    Mybot<boost::asio::serial_port> robot(node_handle, joint_pub, tf_broadcaster, serial_for_arms, serial_for_hand, serial_for_head, serial_for_mobile);

    controller_manager::ControllerManager cm(&robot, node_handle);
    boost::thread thread_for_io(boost::bind(&boost::asio::io_service::run, &io));

    ros::Duration period(0.02);

    while(!robot.is_activated()){
        robot.read();
        cm.update(ros::Time::now(), period);
        robot.request_angles();
        period.sleep();
    }

    std::cout << "ALL RECEIVED !!!!!!!!!!!!!!!" << std::endl;
    robot.initialize_angles_with_current();


    while (true)
    {
        robot.read();
        cm.update(ros::Time::now(), period*2);
        robot.write();
        period.sleep();
        robot.request_encoder();
        period.sleep();
        //std::cout << "========================================================" << std::endl;
    }

    return 0;
}
