#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>

#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>


#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <thread>         
#include <chrono>         

#include "../modules/controller/motor.hpp"
#include "../modules/controller/packet.hpp"

#include "RobotHead.hpp"
#include "pugixml.hpp"

#include <Eigen/Eigen>
#include <tf_conversions/tf_eigen.h>


using boost::asio::ip::tcp;


struct xml_string_writer: pugi::xml_writer{
    std::string result;

    virtual void write(const void* data, size_t size){
        result.append(static_cast<const char*>(data), size);
    }
};



class MybotMobile : public hardware_interface::RobotHW
{
    public:
        MybotMobile(ros::NodeHandle& node_handle, ros::Publisher& joint_pub, tf::TransformBroadcaster& tf_broadcaster, tcp::socket& socket_for_mobile, tcp::resolver::iterator& iterator_for_mobile ) 
            : m_node_handle(node_handle), m_joint_pub(joint_pub), m_tf_broadcaster(tf_broadcaster), m_socket_for_mobile(socket_for_mobile), m_iterator_for_mobile(iterator_for_mobile), m_is_active(true), sim_trans(1) 
        { 


            std::vector<std::string> joint_names_for_mobile;
            joint_names_for_mobile.push_back("right_wheel_joint_0");
            joint_names_for_mobile.push_back("left_wheel_joint_0");

            a_state_data_for_mobile.resize(joint_names_for_mobile.size());
            a_cmd_data_for_mobile.resize(joint_names_for_mobile.size());
            j_state_data_for_mobile.resize(joint_names_for_mobile.size());
            j_cmd_data_for_mobile.resize(joint_names_for_mobile.size());

            a_cmd_for_mobile.resize(joint_names_for_mobile.size(),0);
            a_cmd_vel_for_mobile.resize(joint_names_for_mobile.size(),0);
            a_cmd_prev_for_mobile.resize(joint_names_for_mobile.size(),0);
            a_pos_for_mobile.resize(joint_names_for_mobile.size());
            a_vel_for_mobile.resize(joint_names_for_mobile.size());
            a_eff_for_mobile.resize(joint_names_for_mobile.size());

            j_cmd_for_mobile.resize(joint_names_for_mobile.size());
            j_cmd_vel_for_mobile.resize(joint_names_for_mobile.size());
            j_pos_for_mobile.resize(joint_names_for_mobile.size());
            j_vel_for_mobile.resize(joint_names_for_mobile.size());
            j_eff_for_mobile.resize(joint_names_for_mobile.size());

            for(auto it = joint_names_for_mobile.begin(); it != joint_names_for_mobile.end(); ++it){
                std::size_t index = it - joint_names_for_mobile.begin();
                hardware_interface::JointStateHandle state_handle(it->c_str(), &j_pos_for_mobile[index], &j_vel_for_mobile[index], &j_eff_for_mobile[index]);
                jnt_state_interface.registerHandle(state_handle);
                ROS_INFO("%s\n",it->c_str());
            }
            registerInterface(&jnt_state_interface);

            for(auto it = joint_names_for_mobile.begin(); it != joint_names_for_mobile.end(); ++it){
                std::size_t index = it - joint_names_for_mobile.begin();
                hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(it->c_str()), &j_cmd_vel_for_mobile[index]);
                jnt_vel_interface.registerHandle(vel_handle);
            }
            registerInterface(&jnt_vel_interface);

            for(int i = 0; i < joint_names_for_mobile.size(); ++i){
                a_state_data_for_mobile[i].position.push_back(&a_pos_for_mobile[i]);
                a_state_data_for_mobile[i].velocity.push_back(&a_vel_for_mobile[i]);
                a_state_data_for_mobile[i].effort.push_back(&a_eff_for_mobile[i]);
                j_state_data_for_mobile[i].position.push_back(&j_pos_for_mobile[i]);
                j_state_data_for_mobile[i].velocity.push_back(&j_vel_for_mobile[i]);
                j_state_data_for_mobile[i].effort.push_back(&j_eff_for_mobile[i]);

                a_cmd_data_for_mobile[i].position.push_back(&a_cmd_for_mobile[i]);
                j_cmd_data_for_mobile[i].position.push_back(&j_cmd_for_mobile[i]);
                a_cmd_data_for_mobile[i].velocity.push_back(&a_cmd_vel_for_mobile[i]);
                j_cmd_data_for_mobile[i].velocity.push_back(&j_cmd_vel_for_mobile[i]);

                std::string sim_trans_str = std::string("sim_trans") + boost::lexical_cast<std::string>(i);

                act_to_jnt_state.registerHandle(transmission_interface::ActuatorToJointStateHandle(sim_trans_str, &sim_trans, a_state_data_for_mobile[i], j_state_data_for_mobile[i]));
                jnt_to_act_pos.registerHandle(transmission_interface::JointToActuatorPositionHandle(sim_trans_str, &sim_trans, a_cmd_data_for_mobile[i], j_cmd_data_for_mobile[i]));
                jnt_to_act_vel.registerHandle(transmission_interface::JointToActuatorVelocityHandle(sim_trans_str, &sim_trans, a_cmd_data_for_mobile[i], j_cmd_data_for_mobile[i]));
            }


            //torque_on();
            set_position();

        }

        void connect(){
            try{
                boost::asio::connect(m_socket_for_mobile, m_iterator_for_mobile);
            }catch(std::exception& e){
                std::cout << "Connection failed" << std::endl;
            }
        }

        void read(){
            connect();
            read_for_mobile();
            act_to_jnt_state.propagate();
            disconnect();
        }

        void disconnect(){
            if(m_socket_for_mobile.is_open()){
                m_socket_for_mobile.close();
            }
        }

        void torque_off(){

            connect();
            pugi::xml_document doc;
            pugi::xml_node node = doc.root();

            node = doc.append_child("method_call");
            node = node.append_child("method_name");
            node.text().set("ServoOff");
            xml_string_writer string_writer;
            doc.save(string_writer);

            std::cout << string_writer.result << std::endl;
            std::string start = std::string("DSPHAL-1.1:")+boost::lexical_cast<std::string>(string_writer.result.size())+std::string("\n");
            std::vector<char> packet;
            packet.clear();
            packet.insert(packet.end(), start.begin(), start.end());
            m_socket_for_mobile.write_some(boost::asio::buffer(packet.data(), packet.size()));
            packet.clear();
            packet.insert(packet.end(), string_writer.result.begin(), string_writer.result.end());
            m_socket_for_mobile.write_some(boost::asio::buffer(packet.data(), packet.size()));
            disconnect();
        }

        void torque_on(){

            connect();
            pugi::xml_document doc;
            pugi::xml_node node = doc.root();

            node = doc.append_child("method_call");
            node = node.append_child("method_name");
            node.text().set("ServoOn");
            xml_string_writer string_writer;
            doc.save(string_writer);

            std::cout << string_writer.result << std::endl;
            std::string start = std::string("DSPHAL-1.1:")+boost::lexical_cast<std::string>(string_writer.result.size())+std::string("\n");
            std::vector<char> packet;
            packet.clear();
            packet.insert(packet.end(), start.begin(), start.end());
            m_socket_for_mobile.write_some(boost::asio::buffer(packet.data(), packet.size()));
            packet.clear();
            packet.insert(packet.end(), string_writer.result.begin(), string_writer.result.end());
            m_socket_for_mobile.write_some(boost::asio::buffer(packet.data(), packet.size()));
            disconnect();
        }


        void read_for_mobile(){

            pugi::xml_document doc;
            pugi::xml_node node = doc.root();

            node = doc.append_child("method_call");
            node = node.append_child("method_name");
            node.text().set("ReadPosition");
            xml_string_writer string_writer;
            doc.save(string_writer);

            //std::cout << string_writer.result << std::endl;
            std::string start = std::string("DSPHAL-1.1:")+boost::lexical_cast<std::string>(string_writer.result.size())+std::string("\n");

            std::vector<unsigned char> packet;
            packet.clear();
            packet.insert(packet.end(), start.begin(), start.end());
            m_socket_for_mobile.write_some(boost::asio::buffer(packet.data(), packet.size()));
            packet.clear();
            packet.insert(packet.end(), string_writer.result.begin(), string_writer.result.end());
            m_socket_for_mobile.write_some(boost::asio::buffer(packet.data(), packet.size()));

            std::string header("<method_response>");
            std::string last_header("</method_response>");
            boost::asio::streambuf b;
            std::size_t read_size = boost::asio::read_until(m_socket_for_mobile, b, last_header);
            const char* read_str = boost::asio::buffer_cast<const char*>(b.data());



            std::vector<std::string> splited_packets;
            boost::split_regex(splited_packets, read_str, boost::regex(header));

            for(auto it = splited_packets.begin(); it != splited_packets.end(); ++it){
                std::string pk(header.begin(), header.end());
                pk.insert(pk.end(), it->begin(), it->end());
                //goodguy::printPacket(pk.begin(), pk.end());

                std::size_t found = pk.find(last_header);
                if(found == std::string::npos)  continue;
    
                pugi::xml_document doc;
                pugi::xml_parse_result parse_result = doc.load_string(pk.c_str());

                if(!parse_result)   continue;

                int x = doc.child("method_response").child("method_datalist_ret").child("datalist").child("data").child("int").text().as_int();
                int y = doc.child("method_response").child("method_datalist_ret").child("datalist").child("data").next_sibling().child("int").text().as_int();
                int theta = doc.child("method_response").child("method_datalist_ret").child("datalist").child("data").next_sibling().next_sibling().child("int").text().as_int();

                double x_d = x/1000.0;
                double y_d = y/1000.0;
                double theta_d = theta/10.0*3.14/180.0;

                std::cout << "x: " << x_d<< "\t";
                std::cout << "y: " << y_d<< "\t";
                std::cout << "theta: " << theta_d*180.0/3.14<< "\n\n";
                
                Eigen::Affine3d odom = Eigen::Affine3d::Identity();

                odom.translate(Eigen::Vector3d(x_d, y_d, 0));
                Eigen::AngleAxisd rotate_y(0, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd rotate_x(0, Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd rotate_z(theta_d, Eigen::Vector3d::UnitZ());
                odom.rotate(rotate_z*rotate_y*rotate_x);

                tf::Transform odom_tf;
                tf::transformEigenToTF(odom, odom_tf);

                m_tf_broadcaster.sendTransform(tf::StampedTransform(odom_tf, ros::Time::now(), "odom", "base_footprint"));


            }


            
            

            for(int i = 0; i < a_vel_for_mobile.size(); ++i){
                a_vel_for_mobile[i] = 0;
                a_pos_for_mobile[i] = 0;
            }



        }

        void set_position(double x = 0, double y = 0, double theta = 0){
            connect();
            pugi::xml_document doc;
            pugi::xml_node node = doc.root();

            node = doc.append_child("method_call");
            node = node.append_child("method_name");
            node.text().set("ChangePosition");
            node = node.parent();
            node = node.append_child("method_datalist_arg");
            node = node.append_child("datalist");
            node = node.append_child("data").append_child("int");

            node.text().set((int)(x*1000.0));
            node = node.parent().parent().append_child("data").append_child("int");
            node.text().set((int)(y*1000.0));
            node = node.parent().parent().append_child("data").append_child("int");
            node.text().set((int)(theta*10.0*180.0/3.14));

            xml_string_writer string_writer;
            doc.save(string_writer);
            std::cout << string_writer.result << std::endl;
            std::string start = std::string("DSPHAL-1.1:")+boost::lexical_cast<std::string>(string_writer.result.size())+std::string("\n");

            std::vector<unsigned char> packet;
            packet.clear();
            packet.insert(packet.end(), start.begin(), start.end());
            m_socket_for_mobile.write_some(boost::asio::buffer(packet.data(), packet.size()));
            packet.clear();
            packet.insert(packet.end(), string_writer.result.begin(), string_writer.result.end());
            m_socket_for_mobile.write_some(boost::asio::buffer(packet.data(), packet.size()));

            disconnect();
        }

        void write(){

            connect();
            jnt_to_act_pos.propagate();
            write_for_mobile();
            disconnect();

        }

        void write_for_mobile(){
            //std::cout << "MOBILE: " << a_cmd_vel_for_mobile[0] << "\t" << a_cmd_vel_for_mobile[1] << std::endl;

            pugi::xml_document doc;
            pugi::xml_node node = doc.root();

            node = doc.append_child("method_call");
            node = node.append_child("method_name");
            node.text().set("VelocityControl");
            node = node.parent();
            node = node.append_child("method_datalist_arg");
            node = node.append_child("datalist");
            node = node.append_child("data").append_child("int");
            const double r = 0.012;
            double vel_right = r*a_cmd_vel_for_mobile[0]*1000.0;
            double vel_left  = r*a_cmd_vel_for_mobile[1]*1000.0;

            node.text().set((int)vel_left);
            node = node.parent().parent().append_child("data").append_child("int");
            node.text().set((int)vel_right);

            xml_string_writer string_writer;
            doc.save(string_writer);
            //std::cout << string_writer.result << std::endl;
            std::string start = std::string("DSPHAL-1.1:")+boost::lexical_cast<std::string>(string_writer.result.size())+std::string("\n");

            std::vector<unsigned char> packet;
            packet.clear();
            packet.insert(packet.end(), start.begin(), start.end());
            m_socket_for_mobile.write_some(boost::asio::buffer(packet.data(), packet.size()));
            packet.clear();
            packet.insert(packet.end(), string_writer.result.begin(), string_writer.result.end());
            m_socket_for_mobile.write_some(boost::asio::buffer(packet.data(), packet.size()));


        }

    private:

        ros::NodeHandle& m_node_handle;
        ros::Publisher& m_joint_pub;
        tf::TransformBroadcaster& m_tf_broadcaster;

        tcp::socket& m_socket_for_mobile;
        tcp::resolver::iterator& m_iterator_for_mobile;


        std::vector<unsigned char> m_received_packet_buffer_for_mobile;
        unsigned char m_recv_buffer_for_mobile[1024];
        unsigned char m_send_buffer_for_mobile[1024];

        bool m_is_active;

        transmission_interface::ActuatorToJointStateInterface    act_to_jnt_state;
        transmission_interface::JointToActuatorPositionInterface jnt_to_act_pos;
        transmission_interface::JointToActuatorVelocityInterface jnt_to_act_vel;
        transmission_interface::SimpleTransmission               sim_trans;
        hardware_interface::JointStateInterface                  jnt_state_interface;
        hardware_interface::PositionJointInterface               jnt_pos_interface;
        hardware_interface::VelocityJointInterface               jnt_vel_interface;



        std::vector<transmission_interface::ActuatorData> a_state_data_for_mobile;
        std::vector<transmission_interface::ActuatorData> a_cmd_data_for_mobile;
        std::vector<transmission_interface::JointData>    j_state_data_for_mobile;
        std::vector<transmission_interface::JointData>    j_cmd_data_for_mobile;

        std::vector<double> a_cmd_for_mobile;
        std::vector<double> a_cmd_vel_for_mobile;
        std::vector<double> a_cmd_prev_for_mobile;
        std::vector<double> a_pos_for_mobile;
        std::vector<double> a_vel_for_mobile;
        std::vector<double> a_eff_for_mobile;

        std::vector<double> j_cmd_for_mobile;
        std::vector<double> j_cmd_vel_for_mobile;
        std::vector<double> j_pos_for_mobile;
        std::vector<double> j_vel_for_mobile;
        std::vector<double> j_eff_for_mobile;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mybot_mobile_controller");


    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    //ros::Rate rate(1.0);
    ROS_INFO("Start Controller With TCP COMM");

    ros::NodeHandle node_handle;


    boost::asio::io_service io;
    int port_mobile = 50010;


    tcp::socket socket_for_mobile(io);
    tcp::resolver resolver_mobile(io);
    tcp::resolver::query query_for_mobile(tcp::v4(), "192.168.51.10", boost::lexical_cast<std::string>(port_mobile));
    tcp::resolver::iterator iterator_for_mobile = resolver_mobile.resolve(query_for_mobile);



    ros::Publisher joint_pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster tf_broadcaster;


    MybotMobile robot(node_handle, joint_pub, tf_broadcaster, socket_for_mobile, iterator_for_mobile);

    controller_manager::ControllerManager cm(&robot, node_handle);
    boost::thread thread_for_io(boost::bind(&boost::asio::io_service::run, &io));



    ros::Duration period(0.10);

    while (true)
    {
        //std::cout << "START" << std::endl;
        robot.read();
        cm.update(ros::Time::now(), period);
        robot.write();
        period.sleep();


    }

    return 0;
}
