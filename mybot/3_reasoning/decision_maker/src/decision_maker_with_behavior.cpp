#include <ros/ros.h>

#include <shape_tools/solid_primitive_dims.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <simple_recognition/RecogObject.h>
#include <decision_maker/EventList.h>

#include <std_srvs/Empty.h>

#include <boost/algorithm/string.hpp>

#include <boost/format.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <thread>
#include <mutex>
#include <tuple>
#include <behavior.hpp>
#include <pugixml.hpp>


#include <std_msgs/UInt16.h>

#include <simple_navigation/GoTarget.h>
#include <gaze_selection/PanTilt.h>

namespace goodguy{

    class Object{
        public:
            Object() { }
            Object(const std::string& object_xml){
                pugi::xml_document doc;
                pugi::xml_parse_result parse_result = doc.load_string(object_xml.c_str());

                if(!parse_result){
                    std::cout << "SOMETHING WRONG TO PARSE XML!!!!!!!!!!!!!!!!" << std::endl;
                    return;
                }


                m_real_name = doc.child("object").child("real_name").text().as_string();
                std::cout << "REAL_NAME: " << m_real_name << std::endl;

                pugi::xml_node name_node = doc.child("object").child("name");
                while(!name_node.empty()){
                    m_names.push_back(name_node.text().as_string());
                    std::cout << "NAME: " << m_names.back() << std::endl;
                    name_node = name_node.next_sibling("name");
                }

                pugi::xml_node collision_node = doc.child("object").child("collision").first_child();
                while(!collision_node.empty()){

                    shape_msgs::SolidPrimitive solid;
                    geometry_msgs::Pose solid_pose;

                    std::string type = collision_node.name();
                    //std::cout << "Collision Type: " << type << std::endl;
                    if(type == "cylinder"){
                        solid.type = shape_msgs::SolidPrimitive::CYLINDER;

                        solid.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
                        double dim_radius = collision_node.child("dimension").attribute("radius").as_double();
                        double dim_height = collision_node.child("dimension").attribute("height").as_double();
                        solid.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = dim_radius;
                        solid.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = dim_height;

                        //std::cout << "dim_radius: " << dim_radius << "\tdim_height: " << dim_height << std::endl;
                    }
                    else if(type == "box"){
                        solid.type = shape_msgs::SolidPrimitive::BOX;
                        solid.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
                        double dim_x = collision_node.child("dimension").attribute("x").as_double();
                        double dim_y = collision_node.child("dimension").attribute("y").as_double();
                        double dim_z = collision_node.child("dimension").attribute("z").as_double();
                        solid.dimensions[shape_msgs::SolidPrimitive::BOX_X] = dim_x;
                        solid.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = dim_y;
                        solid.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = dim_z;

                        //std::cout << "dim_x: " << dim_x << "\tdim_y: " << dim_y << "\tdim_z: " << dim_z << std::endl;
                    }
                    solid_pose.position.x = collision_node.child("pose").child("position").attribute("x").as_double();
                    solid_pose.position.y = collision_node.child("pose").child("position").attribute("y").as_double();
                    solid_pose.position.z = collision_node.child("pose").child("position").attribute("z").as_double();
                    double rpy_r = collision_node.child("pose").child("rpy").attribute("r").as_double();
                    double rpy_p = collision_node.child("pose").child("rpy").attribute("p").as_double();
                    double rpy_y = collision_node.child("pose").child("rpy").attribute("y").as_double();

                    Eigen::AngleAxisd rotate_r(rpy_r,  Eigen::Vector3d::UnitX());
                    Eigen::AngleAxisd rotate_p(rpy_p,  Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd rotate_y(rpy_y,  Eigen::Vector3d::UnitZ());
                    Eigen::Quaterniond rotation = rotate_r*rotate_p*rotate_y;

                    solid_pose.orientation.x = rotation.x();
                    solid_pose.orientation.y = rotation.y();
                    solid_pose.orientation.z = rotation.z();
                    solid_pose.orientation.w = rotation.w();

                    m_co.primitives.push_back(solid);
                    m_co.primitive_poses.push_back(solid_pose);

                    collision_node = collision_node.next_sibling();

                }
                m_co.id = get_real_name();

                //std::cout << m_co << std::endl;

                pugi::xml_node behavior_node = doc.child("object").child("behavior");
                if(!behavior_node.child("approach_point").empty()){
                    m_approach_right_point(0) = behavior_node.child("approach_point").attribute("x").as_double();
                    m_approach_right_point(1) = behavior_node.child("approach_point").attribute("y").as_double();
                    m_approach_right_point(2) = behavior_node.child("approach_point").attribute("z").as_double();
                    m_approach_left_point = m_approach_right_point;
                }
                else if(!behavior_node.child("approach_right_point").empty()){
                    m_approach_right_point(0) = behavior_node.child("approach_right_point").attribute("x").as_double();
                    m_approach_right_point(1) = behavior_node.child("approach_right_point").attribute("y").as_double();
                    m_approach_right_point(2) = behavior_node.child("approach_right_point").attribute("z").as_double();
                    m_approach_left_point = m_approach_right_point;
                    m_approach_left_point(1) = -m_approach_left_point(1);
                }
                else if(!behavior_node.child("approach_left_point").empty()){
                    m_approach_left_point(0) = behavior_node.child("approach_left_point").attribute("x").as_double();
                    m_approach_left_point(1) = behavior_node.child("approach_left_point").attribute("y").as_double();
                    m_approach_left_point(2) = behavior_node.child("approach_left_point").attribute("z").as_double();
                    m_approach_right_point = m_approach_left_point;
                    m_approach_right_point(1) = -m_approach_right_point(1);
                }


                if(!behavior_node.child("put_point").empty()){
                    m_put_right_point(0) = behavior_node.child("put_point").attribute("x").as_double();
                    m_put_right_point(1) = behavior_node.child("put_point").attribute("y").as_double();
                    m_put_right_point(2) = behavior_node.child("put_point").attribute("z").as_double();
                    m_put_left_point = m_put_right_point;
                }
                else if(!behavior_node.child("put_right_point").empty()){
                    m_put_right_point(0) = behavior_node.child("put_right_point").attribute("x").as_double();
                    m_put_right_point(1) = behavior_node.child("put_right_point").attribute("y").as_double();
                    m_put_right_point(2) = behavior_node.child("put_right_point").attribute("z").as_double();
                    m_put_left_point = m_put_right_point;
                    m_put_left_point(1) = -m_put_left_point(1);
                }
                else if(!behavior_node.child("put_left_point").empty()){
                    m_put_left_point(0) = behavior_node.child("put_left_point").attribute("x").as_double();
                    m_put_left_point(1) = behavior_node.child("put_left_point").attribute("y").as_double();
                    m_put_left_point(2) = behavior_node.child("put_left_point").attribute("z").as_double();
                    m_put_right_point = m_put_left_point;
                    m_put_right_point(1) = -m_put_right_point(1);
                }


                if(!behavior_node.child("grasp_point").empty()){
                    m_grasp_right_point(0) = behavior_node.child("grasp_point").attribute("x").as_double();
                    m_grasp_right_point(1) = behavior_node.child("grasp_point").attribute("y").as_double();
                    m_grasp_right_point(2) = behavior_node.child("grasp_point").attribute("z").as_double();
                    m_grasp_left_point = m_grasp_right_point;
                }
                else if(!behavior_node.child("grasp_right_point").empty()){
                    m_grasp_right_point(0) = behavior_node.child("grasp_right_point").attribute("x").as_double();
                    m_grasp_right_point(1) = behavior_node.child("grasp_right_point").attribute("y").as_double();
                    m_grasp_right_point(2) = behavior_node.child("grasp_right_point").attribute("z").as_double();
                    m_grasp_left_point = m_grasp_right_point;
                    m_grasp_left_point(1) = -m_grasp_left_point(1);
                }
                else if(!behavior_node.child("grasp_left_point").empty()){
                    m_grasp_left_point(0) = behavior_node.child("grasp_left_point").attribute("x").as_double();
                    m_grasp_left_point(1) = behavior_node.child("grasp_left_point").attribute("y").as_double();
                    m_grasp_left_point(2) = behavior_node.child("grasp_left_point").attribute("z").as_double();
                    m_grasp_right_point = m_grasp_left_point;
                    m_grasp_right_point(1) = -m_grasp_right_point(1);
                }

                if(!behavior_node.child("bottle_mouth_point").empty()){
                    m_bottle_mouth_right_point(0) = behavior_node.child("bottle_mouth_point").attribute("x").as_double();
                    m_bottle_mouth_right_point(1) = behavior_node.child("bottle_mouth_point").attribute("y").as_double();
                    m_bottle_mouth_right_point(2) = behavior_node.child("bottle_mouth_point").attribute("z").as_double();
                    m_bottle_mouth_left_point = m_bottle_mouth_right_point;
                }
                else if(!behavior_node.child("bottle_mouth_right_point").empty()){
                    m_bottle_mouth_right_point(0) = behavior_node.child("bottle_mouth_right_point").attribute("x").as_double();
                    m_bottle_mouth_right_point(1) = behavior_node.child("bottle_mouth_right_point").attribute("y").as_double();
                    m_bottle_mouth_right_point(2) = behavior_node.child("bottle_mouth_right_point").attribute("z").as_double();
                    m_bottle_mouth_left_point = m_bottle_mouth_right_point;
                    m_bottle_mouth_left_point(1) = -m_bottle_mouth_left_point(1);
                }
                else if(!behavior_node.child("bottle_mouth_left_point").empty()){
                    m_bottle_mouth_left_point(0) = behavior_node.child("bottle_mouth_left_point").attribute("x").as_double();
                    m_bottle_mouth_left_point(1) = behavior_node.child("bottle_mouth_left_point").attribute("y").as_double();
                    m_bottle_mouth_left_point(2) = behavior_node.child("bottle_mouth_left_point").attribute("z").as_double();
                    m_bottle_mouth_right_point = m_bottle_mouth_left_point;
                    m_bottle_mouth_right_point(1) = -m_bottle_mouth_right_point(1);
                }
            }

            std::vector<std::string> get_names() const { return m_names; }
            std::string get_real_name() const { return m_real_name; }

            Eigen::Vector3d get_behavior_approach_point(bool is_right) const {
                if(is_right)    return m_approach_right_point;
                else            return m_approach_left_point;
            }
            Eigen::Vector3d get_behavior_put_point(bool is_right) const { 
                if(is_right)    return m_put_right_point;
                else            return m_put_left_point;
            }
            Eigen::Vector3d get_behavior_bottle_mouth_point(bool is_right) const { 
                if(is_right)    return m_bottle_mouth_right_point;
                else            return m_bottle_mouth_right_point;
            }
            Eigen::Vector3d get_behavior_grasp_point(bool is_right) const { 
                if(is_right)    return m_grasp_right_point;
                else            return m_grasp_left_point;
            }
            moveit_msgs::CollisionObject get_collision_object() const { return m_co; }

            void set_real_name(const std::string& real_name){
                std::cout << "SET REAL NAME: " << real_name << std::endl;
                m_real_name = real_name;
                m_co.id = real_name;
            }

        private:
            moveit_msgs::CollisionObject m_co;
            std::vector<std::string> m_names;
            std::string m_real_name;

            Eigen::Vector3d m_approach_left_point;
            Eigen::Vector3d m_approach_right_point;
            Eigen::Vector3d m_put_left_point;
            Eigen::Vector3d m_put_right_point;
            Eigen::Vector3d m_bottle_mouth_left_point;
            Eigen::Vector3d m_bottle_mouth_right_point;
            Eigen::Vector3d m_grasp_left_point;
            Eigen::Vector3d m_grasp_right_point;
    };

    bool operator==(const Object& obj, const std::string& obj_name){
        for(auto obj_name_candidate : obj.get_names()){
            if( obj_name_candidate == obj_name ){
                return true;
            }
        }
        return false;
    }

    class DetectedObject{
        public:
            DetectedObject() {}
            DetectedObject(const Object& object, const Eigen::Affine3d& pose, const double& theta, std::size_t index)
                : m_detected_time(0.0), m_object(object), m_pose(pose), m_theta(theta), m_index(index)
            {  }

            void update_pose(const Eigen::Affine3d& pose, double theta){
                m_pose = pose;
                m_theta = theta;
            }

            moveit_msgs::CollisionObject get_collision_object() const {
                moveit_msgs::CollisionObject co = m_object.get_collision_object();
                for(std::size_t i = 0; i < co.primitive_poses.size(); ++i){
                    co.primitive_poses[i].position.x += m_pose.translation()(0);
                    co.primitive_poses[i].position.y += m_pose.translation()(1);
                    co.primitive_poses[i].position.z += m_pose.translation()(2);
                }
                return co;
            }

        public:
            ros::Duration m_detected_time;
            Object m_object;
            Eigen::Affine3d m_pose;
            double m_theta;
            std::size_t m_index;
    };
    bool operator==(const DetectedObject& obj1, const DetectedObject& obj2){
        const double minimum_distance = 0.30;
        Eigen::Vector3d pos_obj1 = obj1.m_pose.translation();
        Eigen::Vector3d pos_obj2 = obj2.m_pose.translation();
        Eigen::Vector3d diff = pos_obj1 - pos_obj2;
        const double distance = std::sqrt(diff(0)*diff(0) + diff(1)*diff(1) + diff(2)*diff(2));
        if((obj1.m_object.get_real_name() == obj2.m_object.get_real_name())){ // && (distance < minimum_distance)){
            return true;
        }
        return false;
    }
    bool operator<(const DetectedObject& obj1, const DetectedObject& obj2){
        return obj1.m_detected_time > obj2.m_detected_time;
    }

    class Behavior{
        public:
            enum Type{
                NONE,
                MOVE_ARM,
                MOVE_HEAD,
                MOVE_BASE,
                GRIPPER_LEFT,
                GRIPPER_RIGHT,
                FACIAL_EXPRESSION,
                VOICE
            };
        public:
            Behavior() : m_wait_after_finish(0.0), m_type(NONE) { }

        public:
            ros::Duration m_wait_after_finish;
            Type m_type;

            std::shared_ptr<moveit_msgs::RobotTrajectory> m_move_arm_msg;
            std::shared_ptr<geometry_msgs::Point> m_move_head_msg;
            std::shared_ptr<geometry_msgs::Pose2D> m_move_base_msg;
            std::shared_ptr<trajectory_msgs::JointTrajectory> m_gripper_left_msg;
            std::shared_ptr<trajectory_msgs::JointTrajectory> m_gripper_right_msg;
            std_msgs::UInt16 m_facial_expression_msg;
            std::shared_ptr<std_msgs::String> m_voice_msg;
    };


    class DecisionMaker{
        public:
            enum action_type{
                APPROACH,
                GRASP,
                POUR_GRASP,
                POUR_MOVE,
                POUR_TILT,
                POUR_STANDUP,
                LOCATE,
                MOVE,
                THROW,
                KEEP,
                RELEASE
            };

            enum emotion_type{
                EMOTION_NEUTRAL,
                EMOTION_ANGRY,
                EMOTION_DISGRUST,
                EMOTION_FEAR,
                EMOTION_HAPPINESS,
                EMOTION_SADNESS,
                EMOTION_SURPRISE
            };

            enum gripper{
                GRIPPER_RIGHT,
                GRIPPER_LEFT,
                GRIPPER_NONE
            };

            enum grasping_state{
                GRIPPER_STATE_GRASPING,
                GRIPPER_STATE_NOT_GRASPING
            };

            typedef std::shared_ptr<goodguy::DetectedObject> DetectedObjectPtr;
            typedef std::tuple<DetectedObjectPtr, goodguy::DecisionMaker::action_type, DetectedObjectPtr, gripper> event_type;
            typedef std::tuple<goodguy::DecisionMaker::grasping_state, goodguy::DetectedObject> gripper_state;


        public:
            DecisionMaker(): m_start_time(ros::Time::now()), m_sleep_time(0.3), m_object_index(0) {
                ros::NodeHandle node_handle("~");

                std::string object_path;
                std::string head;

                node_handle.getParam("object_path", object_path);
                std::cout << "object_path: " << object_path << std::endl;

                node_handle.getParam("save_path", m_save_path);
                std::cout << "save_path: " << m_save_path << std::endl;
                node_handle.getParam("head_", head);

                boost::filesystem::path object_path_dir(object_path);
                if(!boost::filesystem::exists(object_path_dir) || !boost::filesystem::is_directory(object_path_dir)){
                    std::cout << "OBJECT PATH IS NOT EXIST!!" << std::endl;
                }
                std::vector<boost::filesystem::path> object_files;
                std::copy(boost::filesystem::directory_iterator(object_path_dir), boost::filesystem::directory_iterator(), std::back_inserter(object_files));

                std::sort(object_files.begin(), object_files.end());

                for(auto it = object_files.begin(); it != object_files.end(); ++it){
                    std::ifstream ifs(it->c_str());
                    std::string content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
                    std::cout << it->c_str() << std::endl;
                    std::cout << content << std::endl;
                    Object obj(content);
                    m_support_objects.push_back(obj);
                }

                m_thread_for_erase_objects = std::thread(&DecisionMaker::erase_old_objects, this);

                m_pub_co = node_handle.advertise<moveit_msgs::CollisionObject>("/collision_object", 1);
                m_pub_aco = node_handle.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 1);
                m_pub_gripper_right = node_handle.advertise<trajectory_msgs::JointTrajectory>("/gripper_right_controller/command", 1);
                m_pub_gripper_left = node_handle.advertise<trajectory_msgs::JointTrajectory>("/gripper_left_controller/command", 1);
                m_pub_target_pose = node_handle.advertise<geometry_msgs::PoseArray>("target_pose", 1);


                if(head == "/head_left" || head == "/head_right"){
                    m_pub_gaze = node_handle.advertise<geometry_msgs::Point>(head+std::string("/gaze_point"), 1);
                    m_pub_pan_tilt = node_handle.advertise<gaze_selection::PanTilt>(head+std::string("/pan_tilt_angle"), 1);
                }
                else{
                    m_pub_gaze = node_handle.advertise<geometry_msgs::Point>("/gaze_point", 1);
                    m_pub_pan_tilt = node_handle.advertise<gaze_selection::PanTilt>("/pan_tilt_angle", 1);
                }


                m_service_clear_octomap = node_handle.serviceClient<std_srvs::Empty>("/clear_octomap");
                m_service_go_target = node_handle.serviceClient<simple_navigation::GoTarget>("/simple_navigation_node/go_target");

                m_pub_facial_expression = node_handle.advertise<std_msgs::UInt16>("/facial_expression/emotion", 1);
                m_pub_voice = node_handle.advertise<std_msgs::String>("/speech_interaction/tts/text", 1);



            }

            void clear_octomap(){
                std_srvs::EmptyRequest clear_octomap_request;
                std_srvs::EmptyResponse clear_octomap_response;
                m_service_clear_octomap.call(clear_octomap_request, clear_octomap_response);
                m_upper_body_motion.clearObjects();
            }

            void erase_old_objects(){
                ros::Duration timer(1.0);
                const double erase_time = 5.0; // sec
                while(ros::ok()){
                    const double current_time = (ros::Time::now()-m_start_time).toSec();
                    auto lambda_erase_objects = [current_time, erase_time](const DetectedObject& element){
                        double detected_object_time_distance = std::abs(current_time - element.m_detected_time.toSec());
                        if(detected_object_time_distance > erase_time) { 
                            //std::cout << "ERASE: " << element.m_object.get_real_name() << std::endl;
                            return true;
                        }
                        else
                            return false;
                    };
                    m_mutex_for_objects.lock();
                    //m_objects.remove_if(lambda_erase_objects);
                    m_objects.sort();
                    //for(auto object: m_objects){
                    //    std::cout << "DETECTED OBJECT: " << object.m_object.get_real_name() << "\t" << object.m_detected_time.toSec() << std::endl;
                    //}
                    m_mutex_for_objects.unlock();
                    timer.sleep();
                }
            }

            void image_point_callback(const simple_recognition::RecogObjectConstPtr& msg){

                geometry_msgs::PointStamped image_point;
                image_point.point = msg->stamped_point.point;
                image_point.header = msg->stamped_point.header;

                geometry_msgs::PointStamped transformed_image_point;
                geometry_msgs::PointStamped image_head_3d_point;

                try{
                    m_tf_listener.transformPoint("/base_footprint", ros::Time(0), image_point, image_point.header.frame_id, transformed_image_point);
                    m_tf_listener.transformPoint("/head_1", ros::Time(0), image_point, image_point.header.frame_id, image_head_3d_point);
                } catch (tf::TransformException& ex){
                    ROS_ERROR("TRANSFORM EXCEPTION: %s", ex.what());
                    return;
                }

                Eigen::Vector3d object_point;
                object_point(0) = transformed_image_point.point.x;
                object_point(1) = transformed_image_point.point.y;
                object_point(2) = transformed_image_point.point.z;

                Eigen::Affine3d object_candidate_pose = Eigen::Affine3d::Identity();
                object_candidate_pose.translation() = object_point;

                

                DetectedObject object_candidate;
                for(auto support_object : m_support_objects){
                    if(support_object == msg->object_name){
                        object_candidate.m_object = support_object;
                        object_candidate.m_pose = object_candidate_pose;
                        object_candidate.m_theta = msg->theta.data;
                    }
                }

                m_mutex_for_objects.lock();
                bool is_new = true;
                for(auto& registered_object : m_objects){
                    if(registered_object == object_candidate){
                        registered_object.m_detected_time = ros::Time::now()-m_start_time;
                        registered_object.update_pose(object_candidate_pose, msg->theta.data);

                        is_new = false;
                    }
                }
                if(is_new){
                    object_candidate.m_index = m_object_index++;
                    object_candidate.m_detected_time = ros::Time::now()-m_start_time;
                    m_objects.push_back(object_candidate);
                }

                //std::cout << "POINT: " << msg->object_name << "\t" << object_point << std::endl;
                m_objects.sort();
                m_mutex_for_objects.unlock();

            }

            std::shared_ptr<goodguy::Behavior> get_behavior_to_see_point(const Eigen::Vector3d& point){
                geometry_msgs::Point msg;
                msg.x = point(0);
                msg.y = point(1);
                msg.z = point(2);

                std::shared_ptr<goodguy::Behavior> behavior = std::make_shared<goodguy::Behavior>(Behavior());
                behavior->m_type = goodguy::Behavior::Type::MOVE_HEAD;
                behavior->m_move_head_msg = std::make_shared<geometry_msgs::Point>(msg);
                return behavior;
            }

            std::shared_ptr<goodguy::Behavior> get_behavior_to_see_target(const goodguy::DetectedObject& target){
                return get_behavior_to_see_point(target.m_pose.translation());
            }


            gripper select_grasp_gripper(
                    const goodguy::DetectedObject& target, 
                    gripper_state& gripper_right, 
                    gripper_state& gripper_left)
            {
                grasping_state is_grasping_by_left = std::get<0>(gripper_left);
                grasping_state is_grasping_by_right = std::get<0>(gripper_right);
                std::cout << "gripper_left: " << std::get<0>(gripper_left) << " / "  << target.m_index << " / " << std::get<1>(gripper_left).m_index << std::endl;
                std::cout << "gripper_right: " << std::get<0>(gripper_right) << " / "  << target.m_index << " / " << std::get<1>(gripper_right).m_index << std::endl;
                if(std::get<0>(gripper_left) == GRIPPER_STATE_GRASPING){
                    if(target.m_index == std::get<1>(gripper_left).m_index){
                        return GRIPPER_LEFT;
                    }
                }
                if(std::get<0>(gripper_right) == GRIPPER_STATE_GRASPING){
                    if(target.m_index == std::get<1>(gripper_right).m_index){
                        return GRIPPER_RIGHT;
                    }
                }
                return GRIPPER_NONE;
            }

            gripper select_empty_gripper(
                    const goodguy::DetectedObject& target, 
                    gripper_state& gripper_right, 
                    gripper_state& gripper_left)
            {
                grasping_state is_grasping_by_left = std::get<0>(gripper_left);
                grasping_state is_grasping_by_right = std::get<0>(gripper_right);
                if(is_grasping_by_left == GRIPPER_STATE_GRASPING  && is_grasping_by_right == GRIPPER_STATE_GRASPING ){
                    return GRIPPER_NONE;
                }
                else if(is_grasping_by_left == GRIPPER_STATE_GRASPING){
                    return GRIPPER_RIGHT;
                }
                else if(is_grasping_by_right == GRIPPER_STATE_GRASPING){
                    return GRIPPER_LEFT;
                }
                else{
                    if(target.m_pose.translation()(1) > 0){
                        return GRIPPER_LEFT;
                    }
                    else{
                        return GRIPPER_RIGHT;
                    }
                }
            }


            std::vector<Eigen::Quaterniond> get_gripper_rotations_for_pour(gripper selected_gripper){

                std::vector<Eigen::Quaterniond> rotations_for_grasp;

                const std::size_t num_grasp_candidates = 5;
                const float interval_grasp_candidates = 30.0*3.14/180.0; // degree

                const float tilt_angle = 120.0*3.14/180.0;

                for(std::size_t i = 0; i < num_grasp_candidates; ++i){
                    double rotation_for_grasp_x = 90.0*3.14/180.0 + (i-std::floor(num_grasp_candidates/2.0))*interval_grasp_candidates;

                    Eigen::Quaterniond rotation_for_grasp;
                    if(selected_gripper == GRIPPER_RIGHT){
                        Eigen::AngleAxisd rotate1(-1.57, Eigen::Vector3d::UnitY());
                        Eigen::AngleAxisd rotate2(rotation_for_grasp_x, Eigen::Vector3d::UnitX());
                        Eigen::AngleAxisd rotate3(tilt_angle, Eigen::Vector3d::UnitZ());
                        rotation_for_grasp = rotate1*rotate2*rotate3;
                    }
                    else{
                        Eigen::AngleAxisd rotate1(-1.57, Eigen::Vector3d::UnitY());
                        Eigen::AngleAxisd rotate2(-rotation_for_grasp_x, Eigen::Vector3d::UnitX());
                        Eigen::AngleAxisd rotate3(-tilt_angle, Eigen::Vector3d::UnitZ());
                        rotation_for_grasp = rotate1*rotate2*rotate3;
                    }
                    rotations_for_grasp.push_back(rotation_for_grasp);
                }

                return rotations_for_grasp;
            }

            std::vector<Eigen::Quaterniond> get_gripper_rotations_for_locate(gripper selected_gripper, Eigen::Quaterniond grasp_pose){

                std::vector<Eigen::Quaterniond> rotations_for_grasp;

                const std::size_t num_grasp_candidates = 5;
                const float interval_grasp_candidates = 45.0*3.14/180.0; // degree

                for(std::size_t i = 0; i < num_grasp_candidates; ++i){
                    double rotation_for_grasp_x = (i-std::floor(num_grasp_candidates/2.0))*interval_grasp_candidates;
                    Eigen::AngleAxisd rotate1(rotation_for_grasp_x, Eigen::Vector3d::UnitX());
                    //Eigen::AngleAxisd rotate1(rotation_for_grasp_z, Eigen::Vector3d::UnitZ());
                    Eigen::Quaterniond rotation_for_grasp = grasp_pose * rotate1;
                    //Eigen::Quaterniond rotation_for_grasp = rotate1 * grasp_pose;
                    rotations_for_grasp.push_back(rotation_for_grasp);
                }

                return rotations_for_grasp;
            }

            std::vector<Eigen::Quaterniond> get_gripper_rotations(gripper selected_gripper, double last_y_scale = 1.0){

                std::vector<Eigen::Quaterniond> rotations_for_grasp;

                const std::size_t num_grasp_candidates = 4;
                const float interval_grasp_candidates = 45.0*3.14/180.0; // degree

                for(std::size_t i = 0; i < num_grasp_candidates; ++i){
                    double rotation_for_grasp_x = 90.0*3.14/180.0 + (i-std::floor(num_grasp_candidates/2.0))*interval_grasp_candidates;

                    for(std::size_t j = 0; j < num_grasp_candidates; ++j){
                        Eigen::Quaterniond rotation_for_grasp;
                        double rotation_for_grasp_y = j*interval_grasp_candidates*last_y_scale;
                        if(selected_gripper == GRIPPER_RIGHT){
                            Eigen::AngleAxisd rotate1(-1.57, Eigen::Vector3d::UnitY());
                            Eigen::AngleAxisd rotate2(rotation_for_grasp_x, Eigen::Vector3d::UnitX());
                            Eigen::AngleAxisd rotate3(rotation_for_grasp_y, Eigen::Vector3d::UnitY());
                            rotation_for_grasp = rotate1*rotate2*rotate3;
                        }
                        else{
                            Eigen::AngleAxisd rotate1(-1.57, Eigen::Vector3d::UnitY());
                            Eigen::AngleAxisd rotate2(-rotation_for_grasp_x, Eigen::Vector3d::UnitX());
                            Eigen::AngleAxisd rotate3(rotation_for_grasp_y, Eigen::Vector3d::UnitY());
                            rotation_for_grasp = rotate1*rotate2*rotate3;
                        }
                        rotations_for_grasp.push_back(rotation_for_grasp);
                    }
                }
                Eigen::Quaterniond rotation_for_grasp;
                Eigen::AngleAxisd rotate1(1.57, Eigen::Vector3d::UnitZ());
                Eigen::AngleAxisd rotate2(-1.57, Eigen::Vector3d::UnitX());
                rotation_for_grasp = rotate1*rotate2;
                rotations_for_grasp.push_back(rotation_for_grasp);

                return rotations_for_grasp;
            }

            std::string get_eef_name(gripper selected_gripper){
                if(selected_gripper == GRIPPER_RIGHT){
                    return std::string("right_bottle_mouth_z");
                }
                else{
                    return std::string("left_bottle_mouth_z");
                }
            }

            std::string get_arm_group_name(gripper selected_gripper){
                if(selected_gripper == GRIPPER_RIGHT){
                    return std::string("trunk_with_arm_right");
                }
                else{
                    return std::string("trunk_with_arm_left");
                }

            }

            std::string get_default_motion_str(gripper selected_gripper){
                if(selected_gripper == GRIPPER_RIGHT){
                    return std::string("right_grasp_default");
                    //return std::string("trunk_with_arm_right_home");
                }
                else{
                    return std::string("left_grasp_default");
                    //return std::string("trunk_with_arm_left_home");
                }
            }

            std::vector<geometry_msgs::Pose> get_gripper_pose(
                    const goodguy::DetectedObject& target_object,
                    const goodguy::DetectedObject& target_place,
                    action_type action_t,
                    gripper selected_gripper)
            {

                std::vector<geometry_msgs::Pose> gripper_poses;
                std::vector<Eigen::Quaterniond> rotations_for_grasp;
                Eigen::Vector3d point_for_gripper;
            
                // get last grasping pose for move_to_object 
                Eigen::Affine3d eef_pose_for_grasp = Eigen::Affine3d::Identity();
                if(m_robot_state_for_grasp != NULL){
                    eef_pose_for_grasp= get_grasp_eef_pose(selected_gripper);
                    if(action_t == LOCATE){
                        std::cout << "  1) END EFFECTOR POSE: " << eef_pose_for_grasp.rotation() << std::endl;
                        std::cout << "  2) END EFFECTOR POSE: " << eef_pose_for_grasp.translation() << std::endl;
                    }
                }

                Eigen::Vector3d target_gripper_point;


                // set gripper pose for each action
                if(action_t == APPROACH){
                    point_for_gripper = target_object.m_object.get_behavior_approach_point(selected_gripper == GRIPPER_RIGHT);
                    point_for_gripper += target_object.m_pose.translation();
                    target_gripper_point = point_for_gripper;
                    rotations_for_grasp = get_gripper_rotations(selected_gripper);
                }
                else if(action_t  == GRASP){
                    point_for_gripper = target_object.m_object.get_behavior_grasp_point(selected_gripper == GRIPPER_RIGHT);
                    point_for_gripper += target_object.m_pose.translation();
                    target_gripper_point = point_for_gripper;
                    rotations_for_grasp = get_gripper_rotations(selected_gripper);
                }
                else if(action_t  == POUR_GRASP){
                    point_for_gripper = target_object.m_object.get_behavior_grasp_point(selected_gripper == GRIPPER_RIGHT);
                    point_for_gripper += target_object.m_pose.translation();
                    target_gripper_point = point_for_gripper;
                    // small motion for rotaton at y-axis 
                    rotations_for_grasp = get_gripper_rotations(selected_gripper, 0.01);
                }
                else if(action_t == POUR_MOVE){
                    point_for_gripper = target_place.m_object.get_behavior_put_point(selected_gripper == GRIPPER_RIGHT);
                    point_for_gripper += target_place.m_pose.translation();
                    target_gripper_point = point_for_gripper;
                    // small motion for rotaton at y-axis 
                    rotations_for_grasp = get_gripper_rotations(selected_gripper, 1.0);
                    float tilt_prepare_angle = 45.0*3.14/180.0; // degree
                    if(selected_gripper == GRIPPER_LEFT)    tilt_prepare_angle = -tilt_prepare_angle;

                    for(std::size_t i = 0; i < rotations_for_grasp.size(); ++i){
                        Eigen::AngleAxisd rotate1(tilt_prepare_angle, Eigen::Vector3d::UnitZ());
                        Eigen::Quaterniond rotation_for_grasp = rotations_for_grasp[i] * rotate1;
                        rotations_for_grasp[i] = rotation_for_grasp;
                    }
                }
                else if(action_t == POUR_TILT){
                    target_gripper_point = target_place.m_object.get_behavior_put_point(selected_gripper == GRIPPER_RIGHT);
                    target_gripper_point += target_place.m_pose.translation();
                    point_for_gripper = Eigen::Vector3d::Zero();
                    // small motion for rotaton at y-axis 
                    rotations_for_grasp = get_gripper_rotations_for_pour(selected_gripper);
                }
                else if(action_t == POUR_STANDUP){
                    target_gripper_point = target_place.m_object.get_behavior_put_point(selected_gripper == GRIPPER_RIGHT);
                    target_gripper_point += target_place.m_pose.translation();
                    target_gripper_point(2) += 0.10;
                    point_for_gripper = target_gripper_point;
                    // small motion for rotaton at y-axis 
                    rotations_for_grasp = get_gripper_rotations(selected_gripper, 0.1);
                }
                else if(action_t == LOCATE){
                    point_for_gripper = target_object.m_object.get_behavior_grasp_point(selected_gripper == GRIPPER_RIGHT);
                    point_for_gripper += target_place.m_pose.translation();
                    point_for_gripper(2) += 0.01;
                    target_gripper_point = point_for_gripper;
                    // previous grasping pose should be considered!!
                    rotations_for_grasp = get_gripper_rotations_for_locate(selected_gripper, Eigen::Quaterniond(eef_pose_for_grasp.rotation()));
                }
                else if(action_t == THROW){
                    point_for_gripper = target_place.m_object.get_behavior_put_point(selected_gripper == GRIPPER_RIGHT);
                    point_for_gripper += target_place.m_pose.translation();
                    target_gripper_point = point_for_gripper;
                    rotations_for_grasp = get_gripper_rotations(selected_gripper);
                }

                for(auto rotation : rotations_for_grasp){
                    geometry_msgs::Pose gripper_pose;
                    gripper_pose.position.x = point_for_gripper(0);
                    gripper_pose.position.y = point_for_gripper(1);
                    gripper_pose.position.z = point_for_gripper(2);
                    gripper_pose.orientation.x = rotation.x();
                    gripper_pose.orientation.y = rotation.y();
                    gripper_pose.orientation.z = rotation.z();
                    gripper_pose.orientation.w = rotation.w();
                    gripper_poses.push_back(gripper_pose);
                }

                geometry_msgs::PoseArray pose_array_msg;
                pose_array_msg.header.stamp = ros::Time::now();
                pose_array_msg.header.frame_id = "base_footprint";
                for(auto rotation : rotations_for_grasp){
                    Eigen::AngleAxisd rotate1(1.57, Eigen::Vector3d::UnitY());
                    rotation = rotation * rotate1;
                    geometry_msgs::Pose gripper_pose;
                    gripper_pose.position.x = target_gripper_point(0);
                    gripper_pose.position.y = target_gripper_point(1);
                    gripper_pose.position.z = target_gripper_point(2);
                    gripper_pose.orientation.x = rotation.x();
                    gripper_pose.orientation.y = rotation.y();
                    gripper_pose.orientation.z = rotation.z();
                    gripper_pose.orientation.w = rotation.w();
                    pose_array_msg.poses.push_back(gripper_pose);
                }

                m_pub_target_pose.publish(pose_array_msg);



                return gripper_poses;
            }

            moveit_msgs::AttachedCollisionObject get_attach_object_msg(gripper selected_gripper, const goodguy::DetectedObject& target){

                Eigen::Affine3d eef_pose_for_grasp = Eigen::Affine3d::Identity();
                if(m_robot_state_for_grasp != NULL){
                    eef_pose_for_grasp= get_grasp_eef_pose(selected_gripper);
                }
                moveit_msgs::AttachedCollisionObject aco;
                aco.object = target.get_collision_object();
                std::string touch_link_name;
                std::vector<std::string> attached_link_names;
                if(selected_gripper == GRIPPER_RIGHT){
                    touch_link_name  = "right_gripper_parm_0";
                    attached_link_names.push_back("right_gripper_parm_0");
                    attached_link_names.push_back("right_gripper_thumb_0");
                    attached_link_names.push_back("right_gripper_thumb_1");
                    attached_link_names.push_back("right_gripper_forefinger_0");
                    attached_link_names.push_back("right_gripper_forefinger_1");
                    attached_link_names.push_back("right_gripper_littlefinger_0");
                    attached_link_names.push_back("right_gripper_littlefinger_1");
                }
                else if(selected_gripper == GRIPPER_LEFT){
                    touch_link_name  = "left_gripper_parm_0";
                    attached_link_names.push_back("left_gripper_parm_0");
                    attached_link_names.push_back("left_gripper_thumb_0");
                    attached_link_names.push_back("left_gripper_thumb_1");
                    attached_link_names.push_back("left_gripper_forefinger_0");
                    attached_link_names.push_back("left_gripper_forefinger_1");
                    attached_link_names.push_back("left_gripper_littlefinger_0");
                    attached_link_names.push_back("left_gripper_littlefinger_1");
                }

                aco.link_name = touch_link_name;
                aco.touch_links = attached_link_names;
                aco.object.header.frame_id = touch_link_name;
                geometry_msgs::Pose grasp_pose;

                Eigen::Affine3d global_grasp_pose = eef_pose_for_grasp;
                Eigen::Affine3d global_grasp_pose_inverse = global_grasp_pose.inverse();
                Eigen::Affine3d object_pose = target.m_pose;
                object_pose.translation() += target.m_object.get_behavior_grasp_point(selected_gripper == GRIPPER_RIGHT);

                Eigen::Affine3d grasp_pose_local =  global_grasp_pose_inverse * object_pose;

                tf::poseEigenToMsg(grasp_pose_local, grasp_pose);

                for(auto& object_pose:aco.object.primitive_poses){
                    object_pose = grasp_pose;
                }

                aco.object.operation = moveit_msgs::CollisionObject::ADD;
                update_robot_state();
                return aco;

            }

            

            void update_robot_state(){
                if(m_robot_state == NULL){
                    m_robot_state = std::make_shared<robot_state::RobotState>(m_upper_body_motion.get_current_robot_state());
                }

                std::shared_ptr<robot_state::RobotState> robot_state_pre = m_robot_state;
                m_robot_state = std::make_shared<robot_state::RobotState>(m_upper_body_motion.get_current_robot_state());

                const std::vector<std::string>& variable_names = robot_state_pre->getVariableNames();

                for(auto name : variable_names){
                    if(name.find("bottle") == std::string::npos){
                        m_robot_state->setVariablePosition(name, robot_state_pre->getVariablePosition(name));
                    }
                }
            }

            void publish_attach_object(gripper selected_gripper, const goodguy::DetectedObject& target){

                moveit_msgs::AttachedCollisionObject aco = get_attach_object_msg(selected_gripper, target);
                //aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
                //m_pub_aco.publish(aco);
                //m_sleep_time.sleep();
                aco.object.operation = moveit_msgs::CollisionObject::ADD;
                m_pub_aco.publish(aco);
                m_sleep_time.sleep();
                std::cout << "ATTACH OBJECT : " << target.m_object.get_real_name() << std::endl;
                update_robot_state();

            }

            void publish_detach_object(gripper selected_gripper, const goodguy::DetectedObject& target){

                moveit_msgs::AttachedCollisionObject aco = get_attach_object_msg(selected_gripper, target);
                aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
                m_pub_aco.publish(aco);
                m_sleep_time.sleep();
                std::cout << "DETACH OBJECT " << target.m_object.get_real_name()  << std::endl;
                update_robot_state();
            }
            void publish_add_collision_object(const goodguy::DetectedObject& target){
                moveit_msgs::CollisionObject co = target.get_collision_object();
                co.operation = moveit_msgs::CollisionObject::ADD;
                co.header.frame_id = "base_footprint";
                m_pub_co.publish(co);
                m_sleep_time.sleep();
                std::cout << "ADD COLLISION OBJECT " << target.m_object.get_real_name()  << std::endl;
                update_robot_state();
            }

            void publish_remove_collision_object(const goodguy::DetectedObject& target){
                moveit_msgs::CollisionObject co = target.get_collision_object();
                co.operation = moveit_msgs::CollisionObject::REMOVE;
                co.header.frame_id = "base_footprint";
                m_pub_co.publish(co);
                std::cout << "REMOVE COLLISION OBJECT " << target.m_object.get_real_name()  << std::endl;
                m_sleep_time.sleep();
                update_robot_state();
            }

            std::shared_ptr<goodguy::Behavior> get_behavior_for_arm_default_motion_rrt(
                    std::string default_position_name,
                    gripper selected_gripper)
            {
                std::vector<moveit_msgs::Constraints> goal_constraints;

                std::string arm_group = get_arm_group_name(selected_gripper);
                const robot_model::JointModelGroup* joint_model_group = m_robot_state->getJointModelGroup(arm_group);

                robot_state::RobotState goal_state(m_upper_body_motion.m_robot_model);
                goal_state.setToDefaultValues(joint_model_group, default_position_name);
                moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
                goal_constraints.push_back(goal_constraint);
                moveit_msgs::Constraints path_constraints;

                moveit_msgs::RobotTrajectory generated_traj;

                if(m_upper_body_motion.generate_motion_plan(
                    arm_group,
                    goal_constraints,
                    path_constraints, 
                    generated_traj,
                    *m_robot_state,
                    "RRTConnectkConfigDefault"
                    ))
                {
                    std::shared_ptr<goodguy::Behavior> behavior = std::make_shared<goodguy::Behavior>(goodguy::Behavior());
                    behavior->m_move_arm_msg = std::make_shared<moveit_msgs::RobotTrajectory>(generated_traj);
                    behavior->m_type = goodguy::Behavior::Type::MOVE_ARM;

                    moveit::core::jointTrajPointToRobotState(generated_traj.joint_trajectory, generated_traj.joint_trajectory.points.size()-1, *m_robot_state);

                    return behavior;
                }
                else{
                    return NULL;
                }

            }


            std::shared_ptr<goodguy::Behavior> get_behavior_for_arm_motion_rrt(
                    std::vector<geometry_msgs::Pose>& gripper_poses,
                    gripper selected_gripper, 
                    moveit_msgs::Constraints path_constraints = moveit_msgs::Constraints(),
                    double tolerance_pos = 0.003, 
                    double tolerance_angle = 1.0*3.14/180.0 )
            {
                std::vector<moveit_msgs::Constraints> goal_constraints;
                std::string eef_link_name = get_eef_name(selected_gripper);

                for(auto eef_link_pose :gripper_poses){
                    moveit_msgs::Constraints goal_constraint;
                    if(eef_link_pose.position.x == 0.0 && eef_link_pose.position.y == 0.0 && eef_link_pose.position.z == 0.0){
                        goal_constraint = MybotActionNode::getOrientationConstraint(eef_link_name, eef_link_pose.orientation, tolerance_angle);
                    }
                    else{
                        goal_constraint = MybotActionNode::getPoseConstraint(eef_link_name, eef_link_pose, tolerance_pos, tolerance_angle);
                    }
                    goal_constraints.push_back(goal_constraint);
                }

                moveit_msgs::RobotTrajectory generated_traj;

                std::string arm_group = get_arm_group_name(selected_gripper);

                if(m_upper_body_motion.generate_motion_plan(
                    arm_group,
                    goal_constraints,
                    path_constraints, 
                    generated_traj,
                    *m_robot_state,
                    "RRTConnectkConfigDefault"
                    ))
                {
                    std::shared_ptr<goodguy::Behavior> behavior = std::make_shared<goodguy::Behavior>(goodguy::Behavior());
                    behavior->m_move_arm_msg = std::make_shared<moveit_msgs::RobotTrajectory>(generated_traj);
                    behavior->m_type = goodguy::Behavior::Type::MOVE_ARM;

                    moveit::core::jointTrajPointToRobotState(generated_traj.joint_trajectory, generated_traj.joint_trajectory.points.size()-1, *m_robot_state);

                    return behavior;
                }
                else{
                    return NULL;
                }

            }


            std::shared_ptr<goodguy::Behavior> get_behavior_for_arm_default_motion(
                    std::string default_position_name,
                    gripper selected_gripper)
            {
                std::vector<moveit_msgs::Constraints> goal_constraints;

                std::string arm_group = get_arm_group_name(selected_gripper);
                const robot_model::JointModelGroup* joint_model_group = m_robot_state->getJointModelGroup(arm_group);

                robot_state::RobotState goal_state(m_upper_body_motion.m_robot_model);
                goal_state.setToDefaultValues(joint_model_group, default_position_name);
                moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
                goal_constraints.push_back(goal_constraint);
                moveit_msgs::Constraints path_constraints;

                moveit_msgs::RobotTrajectory generated_traj;

                if(m_upper_body_motion.generate_motion_plan(
                    arm_group,
                    goal_constraints,
                    path_constraints, 
                    generated_traj,
                    *m_robot_state))
                {
                    std::shared_ptr<goodguy::Behavior> behavior = std::make_shared<goodguy::Behavior>(goodguy::Behavior());
                    behavior->m_move_arm_msg = std::make_shared<moveit_msgs::RobotTrajectory>(generated_traj);
                    behavior->m_type = goodguy::Behavior::Type::MOVE_ARM;

                    moveit::core::jointTrajPointToRobotState(generated_traj.joint_trajectory, generated_traj.joint_trajectory.points.size()-1, *m_robot_state);

                    return behavior;
                }
                else{
                    return NULL;
                }

            }

            std::shared_ptr<goodguy::Behavior> get_behavior_for_move_base(const goodguy::DetectedObject& target){

                geometry_msgs::PoseStamped target_local_pose;
                tf::poseEigenToMsg(target.m_pose, target_local_pose.pose);
                target_local_pose.header.stamp = ros::Time::now();
                target_local_pose.header.frame_id = "/base_footprint";

                geometry_msgs::PoseStamped target_global_pose;

                try{
                    m_tf_listener.transformPose("/map", ros::Time(0), target_local_pose, target_local_pose.header.frame_id, target_global_pose);
                } catch (tf::TransformException& ex){
                    ROS_ERROR("TRANSFORM EXCEPTION: %s", ex.what());
                    return NULL;
                }
                Eigen::Affine3d target_global_pose_eigen;
                tf::poseMsgToEigen(target_global_pose.pose, target_global_pose_eigen);


                geometry_msgs::Pose2D base_pose;

                base_pose.x = target_global_pose_eigen.translation().x();
                base_pose.y = target_global_pose_eigen.translation().y();
                base_pose.theta = target.m_theta;
                //base_pose.theta = std::acos(target_pose_3d.rotation()(0,0));
                std::cout << "MOVE ========= " << base_pose << std::endl;

                std::shared_ptr<goodguy::Behavior> behavior = std::make_shared<goodguy::Behavior>(goodguy::Behavior());
                behavior->m_move_base_msg = std::make_shared<geometry_msgs::Pose2D>(base_pose);
                behavior->m_type = goodguy::Behavior::Type::MOVE_BASE;

                return behavior;
            }


            std::shared_ptr<goodguy::Behavior> get_behavior_for_arm_motion(
                    std::vector<geometry_msgs::Pose>& gripper_poses,
                    gripper selected_gripper, 
                    moveit_msgs::Constraints path_constraints = moveit_msgs::Constraints(),
                    double tolerance_pos = 0.01, 
                    double tolerance_angle = 1.0*3.14/180.0 )
            {
                std::vector<moveit_msgs::Constraints> goal_constraints;
                std::string eef_link_name = get_eef_name(selected_gripper);

                for(auto eef_link_pose :gripper_poses){
                    moveit_msgs::Constraints goal_constraint;
                    if(eef_link_pose.position.x == 0.0 && eef_link_pose.position.y == 0.0 && eef_link_pose.position.z == 0.0){
                        goal_constraint = MybotActionNode::getOrientationConstraint(eef_link_name, eef_link_pose.orientation, tolerance_angle);
                    }
                    else{
                        goal_constraint = MybotActionNode::getPoseConstraint(eef_link_name, eef_link_pose, tolerance_pos, tolerance_angle);
                    }
                    goal_constraints.push_back(goal_constraint);
                }

                moveit_msgs::RobotTrajectory generated_traj;

                std::string arm_group = get_arm_group_name(selected_gripper);

                if(m_upper_body_motion.generate_motion_plan(
                    arm_group,
                    goal_constraints,
                    path_constraints, 
                    generated_traj,
                    *m_robot_state))
                {
                    std::shared_ptr<goodguy::Behavior> behavior = std::make_shared<goodguy::Behavior>(goodguy::Behavior());
                    behavior->m_move_arm_msg = std::make_shared<moveit_msgs::RobotTrajectory>(generated_traj);
                    behavior->m_type = goodguy::Behavior::Type::MOVE_ARM;

                    moveit::core::jointTrajPointToRobotState(generated_traj.joint_trajectory, generated_traj.joint_trajectory.points.size()-1, *m_robot_state);

                    return behavior;
                }
                else{
                    return NULL;
                }

            }

            void set_allow_collision_wholebody(gripper selected_gripper, const std::string& target, const std::string& grasp_object, bool is_allowed){
                std::vector<std::string> link_names;
                link_names.push_back("left_gripper_parm_0");
                link_names.push_back("left_gripper_thumb_0");
                link_names.push_back("left_gripper_thumb_1");
                link_names.push_back("left_gripper_forefinger_0");
                link_names.push_back("left_gripper_forefinger_1");
                link_names.push_back("left_gripper_littlefinger_0");
                link_names.push_back("left_gripper_littlefinger_1");
                link_names.push_back("left_wrist_0");
                link_names.push_back("left_wrist_1");
                link_names.push_back("left_wrist_2");
                link_names.push_back("left_elbow_0");
                link_names.push_back("left_shoulder_0");
                link_names.push_back("left_shoulder_1");
                link_names.push_back("left_shoulder_2");
                link_names.push_back("right_gripper_parm_0");
                link_names.push_back("right_gripper_thumb_0");
                link_names.push_back("right_gripper_thumb_1");
                link_names.push_back("right_gripper_forefinger_0");
                link_names.push_back("right_gripper_forefinger_1");
                link_names.push_back("right_gripper_littlefinger_0");
                link_names.push_back("right_gripper_littlefinger_1");
                link_names.push_back("right_wrist_0");
                link_names.push_back("right_wrist_1");
                link_names.push_back("right_wrist_2");
                link_names.push_back("right_elbow_0");
                link_names.push_back("right_shoulder_0");
                link_names.push_back("right_shoulder_1");
                link_names.push_back("right_shoulder_2");
                link_names.push_back(grasp_object);
                m_upper_body_motion.setAllowCollisionToObject(target, is_allowed, link_names);
                update_robot_state();
            }

            void set_bottle_mouth(gripper selected_gripper, const goodguy::DetectedObject& target){
                if(selected_gripper == GRIPPER_RIGHT){
                    m_upper_body_motion.set_bottle_mouth_right(target.m_object.get_behavior_bottle_mouth_point(selected_gripper == GRIPPER_RIGHT));
                }
                else if(selected_gripper == GRIPPER_LEFT){
                    m_upper_body_motion.set_bottle_mouth_left(target.m_object.get_behavior_bottle_mouth_point(selected_gripper == GRIPPER_RIGHT));
                }
                ros::Duration wait_time(2.0);
                wait_time.sleep();
                update_robot_state();
            }

            void clear_bottle_mouth(gripper selected_gripper){
                if(selected_gripper == GRIPPER_RIGHT){
                    m_upper_body_motion.set_bottle_mouth_right(Eigen::Vector3d::Zero());
                }
                else if(selected_gripper == GRIPPER_LEFT){
                    m_upper_body_motion.set_bottle_mouth_left(Eigen::Vector3d::Zero());
                }
                ros::Duration wait_time(2.0);
                wait_time.sleep();
                update_robot_state();
            }

            void set_allow_collision(gripper selected_gripper, const std::string& target, const std::string& grasp_object, bool is_allowed){
                std::vector<std::string> link_names;
                if(selected_gripper == GRIPPER_RIGHT){
                    link_names.push_back("right_gripper_parm_0");
                    link_names.push_back("right_gripper_thumb_0");
                    link_names.push_back("right_gripper_thumb_1");
                    link_names.push_back("right_gripper_forefinger_0");
                    link_names.push_back("right_gripper_forefinger_1");
                    link_names.push_back("right_gripper_littlefinger_0");
                    link_names.push_back("right_gripper_littlefinger_1");
                }
                else if(selected_gripper == GRIPPER_LEFT){
                    link_names.push_back("left_gripper_parm_0");
                    link_names.push_back("left_gripper_thumb_0");
                    link_names.push_back("left_gripper_thumb_1");
                    link_names.push_back("left_gripper_forefinger_0");
                    link_names.push_back("left_gripper_forefinger_1");
                    link_names.push_back("left_gripper_littlefinger_0");
                    link_names.push_back("left_gripper_littlefinger_1");
                }
                link_names.push_back(grasp_object);
                m_upper_body_motion.setAllowCollisionToObject(target, is_allowed, link_names);
                update_robot_state();
            }
            void set_allow_collision(gripper selected_gripper, const std::string& object_name, bool is_allowed){
                if(selected_gripper == GRIPPER_RIGHT){
                    m_upper_body_motion.setAllowCollisionToObjectForRight(object_name, is_allowed);
                }
                else if(selected_gripper == GRIPPER_LEFT){
                    m_upper_body_motion.setAllowCollisionToObjectForLeft(object_name, is_allowed);
                }
                update_robot_state();
            }

            std::shared_ptr<goodguy::Behavior> get_behavior_to_gripper_grasp(
                    const goodguy::DetectedObject& target,
                    const goodguy::DetectedObject& place,
                    action_type action_t,
                    gripper selected_gripper,
                    gripper_state& gripper_right,
                    gripper_state& gripper_left)
            {

                //gripper selected_gripper = select_empty_gripper(target, gripper_right, gripper_left);
                if(selected_gripper == GRIPPER_NONE)    return NULL;

                std::shared_ptr<goodguy::Behavior> behavior = std::make_shared<goodguy::Behavior>(Behavior());
                if(selected_gripper == GRIPPER_RIGHT){
                    behavior->m_type = Behavior::Type::GRIPPER_RIGHT;
                    std::vector<double> angles(3,0);
                    angles[0] = +5.0/180.0*3.14;
                    angles[1] = +5.0/180.0*3.14;
                    angles[2] = -5.0/180.0*3.14;
                    behavior->m_gripper_right_msg = std::make_shared<trajectory_msgs::JointTrajectory>(m_upper_body_motion.getJointTrajectoryMsgForGripperRight(angles));
                    std::get<0>(gripper_right) = GRIPPER_STATE_GRASPING;
                    std::get<1>(gripper_right) = target;
                    moveit::core::jointTrajPointToRobotState(*(behavior->m_gripper_right_msg), behavior->m_gripper_right_msg->points.size()-1, *m_robot_state);
                }
                else if(selected_gripper == GRIPPER_LEFT){
                    behavior->m_type = Behavior::Type::GRIPPER_LEFT;
                    std::vector<double> angles(3,0);
                    angles[0] = -5.0/180.0*3.14;
                    angles[1] = -5.0/180.0*3.14;
                    angles[2] = +5.0/180.0*3.14;
                    behavior->m_gripper_left_msg = std::make_shared<trajectory_msgs::JointTrajectory>(m_upper_body_motion.getJointTrajectoryMsgForGripperLeft(angles));
                    std::get<0>(gripper_left) = GRIPPER_STATE_GRASPING;
                    std::get<1>(gripper_left) = target;
                    moveit::core::jointTrajPointToRobotState(*(behavior->m_gripper_left_msg), behavior->m_gripper_left_msg->points.size()-1, *m_robot_state);
                }

                m_robot_state_for_grasp = std::make_shared<robot_state::RobotState>(*m_robot_state);

                return behavior;
            }

            std::shared_ptr<goodguy::Behavior> get_behavior_to_make_voice(const std::string& str){
                std::shared_ptr<goodguy::Behavior> behavior = std::make_shared<goodguy::Behavior>(Behavior());
                behavior->m_type = Behavior::VOICE;
                behavior->m_voice_msg = std::make_shared<std_msgs::String>();
                behavior->m_voice_msg->data = str;
                return behavior;
            }
            std::shared_ptr<goodguy::Behavior> get_behavior_to_express_emotion(emotion_type emotion){
                std::shared_ptr<goodguy::Behavior> behavior = std::make_shared<goodguy::Behavior>(Behavior());
                behavior->m_type = Behavior::FACIAL_EXPRESSION;
                unsigned short emotion_short = 0;
                switch(emotion){
                    case EMOTION_NEUTRAL:   emotion_short = 0;	break;  
                    case EMOTION_ANGRY:     emotion_short = 1;	break;
                    case EMOTION_DISGRUST:  emotion_short = 2;	break;
                    case EMOTION_FEAR:      emotion_short = 3;	break;
                    case EMOTION_HAPPINESS: emotion_short = 4;	break;
                    case EMOTION_SADNESS:   emotion_short = 5;	break;
                    case EMOTION_SURPRISE:  emotion_short = 6;	break;
                    default:                emotion_short = 0;	break;

                }
                behavior->m_facial_expression_msg.data = emotion_short;
                behavior->m_wait_after_finish = ros::Duration(3.0);
                return behavior;
            }


            std::shared_ptr<goodguy::Behavior> get_behavior_to_gripper_release(
                    const goodguy::DetectedObject& target,
                    const goodguy::DetectedObject& place,
                    action_type action_t,
                    gripper selected_gripper,
                    gripper_state& gripper_right,
                    gripper_state& gripper_left)
            {

                //gripper selected_gripper = select_grasp_gripper(target, gripper_right, gripper_left);
                if(selected_gripper == GRIPPER_NONE)    return NULL;

                std::shared_ptr<goodguy::Behavior> behavior = std::make_shared<goodguy::Behavior>(Behavior());
                if(selected_gripper == GRIPPER_RIGHT){
                    behavior->m_type = Behavior::Type::GRIPPER_RIGHT;
                    std::vector<double> angles(3,0);
                    angles[0] = -15.0/180.0*3.14;
                    angles[1] = -15.0/180.0*3.14;
                    angles[2] = +15.0/180.0*3.14;
                    behavior->m_gripper_right_msg = std::make_shared<trajectory_msgs::JointTrajectory>(m_upper_body_motion.getJointTrajectoryMsgForGripperRight(angles));
                    std::get<0>(gripper_right) = GRIPPER_STATE_NOT_GRASPING;
                    std::get<1>(gripper_right) = target;
                    moveit::core::jointTrajPointToRobotState(*(behavior->m_gripper_right_msg), behavior->m_gripper_right_msg->points.size()-1, *m_robot_state);
                }
                else if(selected_gripper == GRIPPER_LEFT){
                    behavior->m_type = Behavior::Type::GRIPPER_LEFT;
                    std::vector<double> angles(3,0);
                    angles[0] = +15.0/180.0*3.14;
                    angles[1] = +15.0/180.0*3.14;
                    angles[2] = -15.0/180.0*3.14;
                    behavior->m_gripper_left_msg = std::make_shared<trajectory_msgs::JointTrajectory>(m_upper_body_motion.getJointTrajectoryMsgForGripperLeft(angles));
                    std::get<0>(gripper_left) = GRIPPER_STATE_NOT_GRASPING;
                    std::get<1>(gripper_left) = target;
                    moveit::core::jointTrajPointToRobotState(*(behavior->m_gripper_left_msg), behavior->m_gripper_left_msg->points.size()-1, *m_robot_state);
                }

                return behavior;
            }



            void perform_behavior(std::shared_ptr<Behavior> behavior){
                switch(behavior->m_type){
                    case Behavior::MOVE_ARM:
                        {
                            behavior->m_move_arm_msg->joint_trajectory.header.stamp = ros::Time::now() + m_sleep_time;
                            //std::cout << "======== MOVE ARM ========" << std::endl;
                            //std::cout << *(behavior->m_move_arm_msg) << std::endl;
                            m_upper_body_motion.execute_trajectory(*(behavior->m_move_arm_msg));
                        }
                        break;
                    case Behavior::MOVE_HEAD:
                        {
                            //std::cout << "======== MOVE HEAD ========" << std::endl;
                            //std::cout << *(behavior->m_move_head_msg) << std::endl;
                            m_pub_gaze.publish(*(behavior->m_move_head_msg));
                            m_sleep_time.sleep();

                        }
                        break;
                    case Behavior::MOVE_BASE:
                        {
                            simple_navigation::GoTarget go_target_srv;
                            go_target_srv.request.pose = *behavior->m_move_base_msg;
                            if(m_service_go_target.call(go_target_srv)){
                                std::cout << "Successfully reached!!" << std::endl;
                            }
                            else{
                                std::cout << "Reaching to target is failed!!" << std::endl;
                            }

                        }
                        break;
                    case Behavior::GRIPPER_LEFT:
                        {
                            behavior->m_gripper_left_msg->header.stamp = ros::Time::now() + m_sleep_time;
                            //std::cout << "======== GRIPPER LEFT ========" << std::endl;
                            //std::cout << *(behavior->m_gripper_left_msg) << std::endl;
                            m_pub_gripper_left.publish(*(behavior->m_gripper_left_msg));
                            m_sleep_time.sleep();
                            m_sleep_time.sleep();
                        }
                        break;
                    case Behavior::GRIPPER_RIGHT:
                        {
                            behavior->m_gripper_right_msg->header.stamp = ros::Time::now() + m_sleep_time;
                            //std::cout << "======== GRIPPER RIGHT ========" << std::endl;
                            //std::cout << *(behavior->m_gripper_right_msg) << std::endl;
                            m_pub_gripper_right.publish(*(behavior->m_gripper_right_msg));
                            m_sleep_time.sleep();
                            m_sleep_time.sleep();
                        }
                        break;
                    case Behavior::FACIAL_EXPRESSION:
                        {
                            //std::cout << "======== FACIAL EXPRESSION ========" << std::endl;
                            //std::cout << behavior->m_facial_expression_msg << std::endl;
                            m_pub_facial_expression.publish(behavior->m_facial_expression_msg);
                            behavior->m_wait_after_finish.sleep();
                        }
                        break;
                    case Behavior::VOICE:
                        {
                            m_pub_voice.publish(*behavior->m_voice_msg);
                        }
                        break;
                    default:
                        {

                        }
                        break;
                }
            }

            std::string get_event_description(const event_type& event) const {
                std::string description;
                const DetectedObjectPtr& object = std::get<0>(event);
                const DetectedObjectPtr& place = std::get<2>(event);
                const action_type& action_t = std::get<1>(event);

                switch(action_t){
                    case APPROACH:  description = std::string("approach, ") + object->m_object.get_real_name();  break;
                    case GRASP:   
                    case POUR_GRASP:  description = std::string("grasp, ") + object->m_object.get_real_name(); break;
                    case POUR_MOVE:  description = std::string("move to, ") + place->m_object.get_real_name(); break;
                    case POUR_TILT:  description = std::string("tilt, ") + object->m_object.get_real_name(); break;
                    case POUR_STANDUP:  description = std::string("stand up, ") + object->m_object.get_real_name(); break;
                    case LOCATE:  description = std::string("locate, ") + object->m_object.get_real_name(); break;
                    case MOVE:  description = std::string("move to, ") + place->m_object.get_real_name(); break;
                    case THROW:  description = std::string("locate, ") + object->m_object.get_real_name(); break;
                    case KEEP:  description = std::string("keep, ") + object->m_object.get_real_name(); break;
                    case RELEASE:  description = std::string("release, ") + object->m_object.get_real_name(); break;
                }
                return description;
            }

            void print_event(const event_type& event) const {
                std::cout << "EVENT: ";
                std::cout << get_event_description(event) << std::endl;
            }

            void pan_tilt(double pan_angle, double tilt_angle){
                gaze_selection::PanTilt msg;
                msg.pan_angle.data = pan_angle;
                msg.tilt_angle.data = tilt_angle;
                m_pub_pan_tilt.publish(msg);

            }

            void see_center(){

                double pan_center = 0.0;
                double tilt_center = 30.0*3.14/180.0;
                ros::Duration wait_time(2.0);
                pan_tilt(pan_center, tilt_center);
                wait_time.sleep();
            }

            void init_sub_episode_behavior(){
                // clear octomap
                double pan_max_ang = 15.0*3.14/180.0;
                double pan_min_ang = -15.0*3.14/180.0;

                double pan_center = 0.0;
                double tilt_center = 30.0*3.14/180.0;

                ros::Duration wait_time(2.0);
                pan_tilt(pan_max_ang, tilt_center);
                voice("Searching objects");
                wait_time.sleep();
                pan_tilt(pan_min_ang, tilt_center);
                wait_time.sleep();
                see_center();
                wait_time.sleep();
                wait_time.sleep();
                clear_bottle_mouth(GRIPPER_RIGHT);
                clear_bottle_mouth(GRIPPER_LEFT);
            }

            void init_episode_behavior(){
                // clear octomap
                double pan_max_ang = 60.0*3.14/180.0;
                double pan_min_ang = -60.0*3.14/180.0;
                double tilt_max_ang = 40.0*3.14/180.0;
                double tilt_min_ang = -5.0*3.14/180.0;

                double pan_center = 0.0;
                double tilt_center = 30.0*3.14/180.0;

                voice("Start 3-D map building");
                ros::Duration wait_time(2.0);
                pan_tilt(pan_max_ang, tilt_max_ang);
                wait_time.sleep();
                clear_octomap();
                pan_tilt(pan_min_ang, tilt_max_ang);
                wait_time.sleep();
                pan_tilt(pan_min_ang, tilt_min_ang);
                wait_time.sleep();
                pan_tilt(pan_max_ang, tilt_min_ang);
                wait_time.sleep();
                pan_tilt(pan_max_ang, tilt_max_ang);
                wait_time.sleep();
                voice("3-D map building is finished.");
                see_center();
            }

            Eigen::Affine3d get_grasp_eef_pose(gripper selected_gripper){
                Eigen::Affine3d eef_pose_eigen = Eigen::Affine3d::Identity();
                if(m_robot_state_for_grasp != NULL){
                    Eigen::Affine3d eef_global_pose_eigen = m_robot_state_for_grasp->getGlobalLinkTransform(get_eef_name(selected_gripper));

                    geometry_msgs::PoseStamped eef_global_pose;
                    tf::poseEigenToMsg(eef_global_pose_eigen, eef_global_pose.pose);
                    eef_global_pose.header.stamp = ros::Time::now();
                    eef_global_pose.header.frame_id = "/map";

                    geometry_msgs::PoseStamped eef_pose;

                    try{
                        m_tf_listener.transformPose("/base_footprint", ros::Time(0), eef_global_pose, eef_global_pose.header.frame_id, eef_pose);
                    } catch (tf::TransformException& ex){
                        ROS_ERROR("TRANSFORM EXCEPTION: %s", ex.what());
                        return eef_pose_eigen;
                    }
                    tf::poseMsgToEigen(eef_pose.pose, eef_pose_eigen);
                }

                return eef_pose_eigen;
            }

            Eigen::Affine3d get_eef_pose(gripper selected_gripper){
                Eigen::Affine3d eef_pose_eigen = Eigen::Affine3d::Identity();
                if(m_robot_state != NULL){
                    Eigen::Affine3d eef_global_pose_eigen = m_robot_state->getGlobalLinkTransform(get_eef_name(selected_gripper));

                    geometry_msgs::PoseStamped eef_global_pose;
                    tf::poseEigenToMsg(eef_global_pose_eigen, eef_global_pose.pose);
                    eef_global_pose.header.stamp = ros::Time::now();
                    eef_global_pose.header.frame_id = "/map";

                    geometry_msgs::PoseStamped eef_pose;

                    try{
                        m_tf_listener.transformPose("/base_footprint", ros::Time(0), eef_global_pose, eef_global_pose.header.frame_id, eef_pose);
                    } catch (tf::TransformException& ex){
                        ROS_ERROR("TRANSFORM EXCEPTION: %s", ex.what());
                        return eef_pose_eigen;
                    }
                    tf::poseMsgToEigen(eef_pose.pose, eef_pose_eigen);
                }

                return eef_pose_eigen;
            }

            bool get_behavior_lists(
                    const DetectedObjectPtr& object, 
                    const DetectedObjectPtr& place, 
                    const action_type& action_t, 
                    const gripper& selected_gripper,
                    gripper_state& gripper_state_for_left, 
                    gripper_state& gripper_state_for_right,
                    std::vector<DetectedObject>& prev_objects,
                    std::vector<std::shared_ptr<goodguy::Behavior>>& behavior_list,
                    bool is_rrt = false)
            {
                Eigen::Vector3d center_point;
                center_point << 0.6, 0, 0.7;

                
                switch(action_t){
                    case APPROACH:
                        {
                            behavior_list.push_back(get_behavior_to_see_target(*object));
                            //gripper selected_gripper = select_empty_gripper(*object, gripper_state_for_right, gripper_state_for_left);
                            if(selected_gripper == GRIPPER_NONE){
                                behavior_list.push_back(NULL);
                                break;
                            }
                            double t_pos = 0.05;
                            double t_ang = 10.0*3.14/180.0;
                            publish_add_collision_object(*object);
                            std::vector<geometry_msgs::Pose> gripper_poses = get_gripper_pose(*object, *place, action_t, selected_gripper);
                            if(is_rrt){
                                behavior_list.push_back(get_behavior_for_arm_motion_rrt(gripper_poses, selected_gripper, moveit_msgs::Constraints(), t_pos, t_ang));
                            }
                            else{
                                behavior_list.push_back(get_behavior_for_arm_motion(gripper_poses, selected_gripper, moveit_msgs::Constraints(), t_pos, t_ang));
                            }
                            break;
                        }
                    case GRASP:
                    case POUR_GRASP:
                        {
                            behavior_list.push_back(get_behavior_to_see_target(*object));
                            //gripper selected_gripper = select_empty_gripper(*object, gripper_state_for_right, gripper_state_for_left);
                            gripper grasping_gripper = select_grasp_gripper(*object, gripper_state_for_right, gripper_state_for_left);
                            if(grasping_gripper != GRIPPER_NONE){
                                std::cout << "ALEADY GRASP OBJECT" << std::endl;
                                break;
                            }
                            if(selected_gripper == GRIPPER_NONE){
                                behavior_list.push_back(NULL);
                                break;
                            }
                            publish_add_collision_object(*object);
                            set_allow_collision(selected_gripper, object->m_object.get_real_name(), false);

                            std::vector<geometry_msgs::Pose> gripper_poses = get_gripper_pose(*object, *place, action_t, selected_gripper);
                            std::shared_ptr<goodguy::Behavior> behavior_motion;
                            if(is_rrt){
                                behavior_motion = get_behavior_for_arm_motion_rrt(gripper_poses, selected_gripper);   
                            }
                            else{
                                behavior_motion = get_behavior_for_arm_motion(gripper_poses, selected_gripper);   
                            }
                            behavior_list.push_back(behavior_motion);
                            behavior_list.push_back(get_behavior_to_gripper_grasp(*object, *place, action_t, selected_gripper, gripper_state_for_right, gripper_state_for_left));
                            if(behavior_motion != NULL){
                                publish_attach_object(selected_gripper, *object);   
                                DetectedObject prev_object = *object;
                                prev_object.m_object.set_real_name(std::string("PrevObject"));
                                prev_objects.push_back(prev_object);
                                publish_add_collision_object(prev_object);
                                set_allow_collision(selected_gripper, prev_object.m_object.get_real_name(), object->m_object.get_real_name(), true);
                            }
                            break;
                        }
                    case POUR_MOVE:
                    case THROW:
                    case POUR_STANDUP:
                    case POUR_TILT:
                    case LOCATE:
                        {
                            behavior_list.push_back(get_behavior_to_see_target(*place));
                            //gripper selected_gripper = select_grasp_gripper(*object, gripper_state_for_right, gripper_state_for_left);
                            if(selected_gripper == GRIPPER_NONE){
                                std::cout << "SELECTED GRIPPER IS NONE: " << std::get<1>(gripper_state_for_right).m_object.get_real_name() << std::endl;
                                std::cout << "SELECTED GRIPPER IS NONE: " << std::get<1>(gripper_state_for_left).m_object.get_real_name() << std::endl;
                                behavior_list.push_back(NULL);
                                break;
                            }

                            DetectedObject temp_place = *place;
                            if(place->m_index == object->m_index){
                                std::cout << "IT IS EQUAL OBJECT" << std::endl;
                                temp_place.m_object.set_real_name(std::string("Temp"));
                            }
                            else{
                                // 오브젝트를 잡고 다른 위치로 이동할경우 기존의 오브젝트의 Octomap을 지우기위함
                                DetectedObject temp_object = *object;
                                temp_object.m_object.set_real_name(std::string("TempObject"));
                                prev_objects.push_back(temp_object);
                                publish_add_collision_object(temp_object);
                                set_allow_collision_wholebody(selected_gripper, temp_object.m_object.get_real_name(), object->m_object.get_real_name(), true);
                            }


                            publish_add_collision_object(temp_place);
                            if(action_t == LOCATE)
                            {
                                set_allow_collision_wholebody(selected_gripper, temp_place.m_object.get_real_name(), object->m_object.get_real_name(), true);
                            }


                            moveit_msgs::Constraints path_constraints = moveit_msgs::Constraints();

                            std::vector<geometry_msgs::Pose> gripper_poses = get_gripper_pose(*object, *place, action_t, selected_gripper);
                            std::cout << "GRIPPER POSES +++++++++++++++" << std::endl;
                            for(std::size_t index = 0; index < gripper_poses.size(); ++index){
                                std::cout << "-----------------------------" << std::endl;
                                std::cout << gripper_poses[index] << std::endl;
                            }
                            std::cout << "+++++++++++++++++++++++++++++" << std::endl;
                            // bottle mouth configuration
                            if(action_t == POUR_STANDUP || action_t == POUR_TILT || action_t == POUR_MOVE){

                                set_bottle_mouth(selected_gripper, *object);
                            }
                            else{
                                clear_bottle_mouth(selected_gripper);
                            }


                            if(action_t == POUR_STANDUP || action_t == POUR_TILT){
                                if(m_robot_state!= NULL){
                                    Eigen::Affine3d eef_pose= Eigen::Affine3d::Identity();
                                    eef_pose= get_eef_pose(selected_gripper);
                                    double tolerence_for_constraint = 0.04;
                                    path_constraints = goodguy::MybotActionNode::getPointConstraint(get_eef_name(selected_gripper), eef_pose.translation(), tolerence_for_constraint);
                                }
                            }
                            else if(action_t == POUR_MOVE || action_t == LOCATE){
                                if(m_robot_state!= NULL){
                                    Eigen::Affine3d eef_pose= Eigen::Affine3d::Identity();
                                    eef_pose= get_eef_pose(selected_gripper);
                                    double tolerence_for_constraint = 45.0*3.14/180.0;
                                    Eigen::Vector3d tolerence_for_axis;
                                    //tolerence_for_axis(0) = tolerence_for_constraint;
                                    //tolerence_for_axis(1) = tolerence_for_constraint;
                                    //tolerence_for_axis(2) = 4*tolerence_for_constraint;
                                    //tolerence_for_axis = eef_pose.rotation().inverse() * tolerence_for_axis;
                                    //path_constraints = goodguy::MybotActionNode::getOrientationConstraint(get_eef_name(selected_gripper), Eigen::Quaterniond(eef_pose.rotation()), tolerence_for_constraint);
                                    //for(auto& ori_constraint: path_constraints.orientation_constraints){
                                    //    ori_constraint.absolute_x_axis_tolerance = tolerence_for_axis(0);
                                    //    ori_constraint.absolute_y_axis_tolerance = tolerence_for_axis(1);
                                    //    ori_constraint.absolute_z_axis_tolerance = tolerence_for_axis(2);
                                    //}
                                }
                            }
                            double t_pos = 0.01;
                            double t_ang = 1.0*3.14/180.0;

                            if(action_t == THROW){
                                t_pos = 0.05;
                                t_ang = 10.0*3.14/180.0;
                            }
                            else if(action_t == POUR_STANDUP){
                                t_pos = 0.20;
                                t_ang = 10.0*3.14/180.0;;
                            }
                            else if(action_t == POUR_TILT){
                                t_pos = 0.03;
                                t_ang = 10.0*3.14/180.0;;
                            }
                            else if(action_t == LOCATE){
                                t_pos = 0.03;

                            }


                            if(is_rrt){
                                behavior_list.push_back(get_behavior_for_arm_motion_rrt(gripper_poses, selected_gripper, path_constraints, t_pos, t_ang));
                            }
                            else{
                                behavior_list.push_back(get_behavior_for_arm_motion(gripper_poses, selected_gripper, path_constraints, t_pos, t_ang));
                            }
                            publish_remove_collision_object(temp_place);


                            for(auto prev_object : prev_objects){
                                publish_remove_collision_object(prev_object);
                            }
                            prev_objects.clear();

                            // SAVE PREV PLACE
                            DetectedObject prev_place = *place;
                            prev_objects.push_back(prev_place);
                            publish_add_collision_object(prev_place);
                            set_allow_collision(selected_gripper, prev_place.m_object.get_real_name(), object->m_object.get_real_name(), false);
                            break;
                        }
                    case MOVE:
                        {
                            behavior_list.push_back(get_behavior_for_move_base(*object));

                            break;
                        }
                    case KEEP:
                        {
                            //behavior_list.push_back(get_behavior_to_see_target(*place));
                            //gripper selected_gripper = select_grasp_gripper(*object, gripper_state_for_right, gripper_state_for_left);
                            if(selected_gripper == GRIPPER_NONE){
                                behavior_list.push_back(NULL);
                                break;
                            }

                            behavior_list.push_back(get_behavior_for_arm_default_motion(get_default_motion_str(selected_gripper), selected_gripper));
                            behavior_list.push_back(get_behavior_to_see_point(center_point));
                            for(auto prev_object : prev_objects){
                                publish_remove_collision_object(prev_object);
                            }
                            prev_objects.clear();
                            break;
                        }
                    case RELEASE:
                        {
                            behavior_list.push_back(get_behavior_to_see_target(*place));
                            //gripper selected_gripper = select_grasp_gripper(*object, gripper_state_for_right, gripper_state_for_left);
                            if(selected_gripper == GRIPPER_NONE){
                                behavior_list.push_back(NULL);
                                break;
                            }
                            publish_detach_object(selected_gripper, *object);   
                            behavior_list.push_back(get_behavior_to_gripper_release(*object, *place, action_t, selected_gripper, gripper_state_for_right, gripper_state_for_left));

                            // 오브젝트를 잡고 다른 위치로 이동할경우 기존의 오브젝트의 Octomap을 지우기위함
                            DetectedObject temp_object = *object;
                            temp_object.m_object.set_real_name(std::string("TempObject"));
                            prev_objects.push_back(temp_object);
                            publish_add_collision_object(temp_object);
                            set_allow_collision_wholebody(selected_gripper, temp_object.m_object.get_real_name(), object->m_object.get_real_name(), true);
                            // update object pose to place pose
                            object->m_pose = place->m_pose;
                            publish_add_collision_object(*object);
                            //set_allow_collision(selected_gripper, object->m_object.get_real_name(), true);
                            set_allow_collision(selected_gripper, object->m_object.get_real_name(), false);

                            if(is_rrt){
                                behavior_list.push_back(get_behavior_for_arm_default_motion_rrt(get_default_motion_str(selected_gripper), selected_gripper));
                            }
                            else{
                                behavior_list.push_back(get_behavior_for_arm_default_motion(get_default_motion_str(selected_gripper), selected_gripper));
                            }

                            behavior_list.push_back(get_behavior_to_see_point(center_point));
                            publish_remove_collision_object(*object);
                            for(auto prev_object : prev_objects){
                                publish_remove_collision_object(prev_object);
                            }
                            prev_objects.clear();
                            break;
                        }
                }

                for(auto behavior : behavior_list){
                    if(behavior == NULL){
                        return false;
                    }
                }
                return true;
            }

            void voice(const std::string& text){
                std_msgs::String msg;
                msg.data = text;
                m_pub_voice.publish(msg);
            }


            void event_handler(){
                using goodguy::DecisionMaker;


                std::cout << "START EVENT HANDLER " << std::endl;

                std::vector<std::vector<std::vector<std::string>>> sub_episode_list_list;
                {
                    std::vector<std::vector<std::string>> sub_episode_list;
                    sub_episode_list.clear();


                    std::vector<std::string> sub_episode; 
                    for(std::size_t i = 0; i < m_current_event_list.size(); ++i){
                        std::string event = m_current_event_list[i];
                        std::vector<std::string> splited_event;
                        boost::split(splited_event, event, boost::is_any_of(";"));
                        std::string object_str = splited_event[0];
                        std::string action_str = splited_event[1];
                        std::string place_str = splited_event[2];
                        sub_episode.push_back(event);
                        if((action_str == "locate") || (action_str == "throw") || (action_str == "release") || (action_str == "move") || (i == (m_current_event_list.size()-1))){
                            sub_episode_list.push_back(sub_episode);
                            sub_episode.clear();
                            if(action_str == "move" || (i == (m_current_event_list.size()-1))){
                                sub_episode_list_list.push_back(sub_episode_list);
                                sub_episode_list.clear();
                            }
                        }
                    }
                }

                gripper_state gripper_state_for_left = std::make_tuple(GRIPPER_STATE_NOT_GRASPING, goodguy::DetectedObject());
                gripper_state gripper_state_for_right = std::make_tuple(GRIPPER_STATE_NOT_GRASPING, goodguy::DetectedObject());

                for(std::size_t index_for_sub_episode_list = 0; index_for_sub_episode_list < sub_episode_list_list.size(); index_for_sub_episode_list++){

                    bool retry = false;


                    bool is_required_init = true;

                    if(sub_episode_list_list[index_for_sub_episode_list].size() >= 1 && sub_episode_list_list[index_for_sub_episode_list][0].size() == 1){
                        std::string event = sub_episode_list_list[index_for_sub_episode_list][0][0];
                        std::vector<std::string> splited_event;
                        boost::split(splited_event, event, boost::is_any_of(";"));
                        std::string action_str = splited_event[1];
                        if(action_str == "move"){
                            is_required_init = false;
                        }
                    }

                    if(is_required_init){
                        init_episode_behavior();
                    }


                    for(std::size_t index_for_sub_episode = 0; index_for_sub_episode < sub_episode_list_list[index_for_sub_episode_list].size(); ++index_for_sub_episode){

                        std::vector<std::vector<std::string>>& sub_episode_list = sub_episode_list_list[index_for_sub_episode_list];

                        bool is_possible_to_perform_episode = true;


                        if(retry){
                            voice("Retry, again");
                            ros::Duration voice_time(2);
                            voice_time.sleep();

                            if(is_required_init){
                                init_episode_behavior();
                            }
                            gripper_state_for_left = std::make_tuple(GRIPPER_STATE_NOT_GRASPING, goodguy::DetectedObject());
                            gripper_state_for_right = std::make_tuple(GRIPPER_STATE_NOT_GRASPING, goodguy::DetectedObject());
                        }
                        if(is_required_init){
                            init_sub_episode_behavior();
                        }

                        retry = false;

                        std::vector<DetectedObjectPtr> objects;
                        m_mutex_for_objects.lock();
                        for(auto object:m_objects){
                            objects.push_back(std::make_shared<DetectedObject>(object));
                        }
                        m_mutex_for_objects.unlock();

                        std::vector<event_type> event_list;

                        bool is_rrt = false;

                        for(auto event : sub_episode_list[index_for_sub_episode]){
                            std::vector<std::string> splited_event;
                            boost::split(splited_event, event, boost::is_any_of(";"));
                            std::string object_str = splited_event[0];
                            std::string action_str = splited_event[1];
                            std::string place_str = splited_event[2];
                            std::string hand_str = "NONE";
                            if(splited_event.size() >= 4){
                                hand_str = splited_event[3];
                            }

                            std::cout << "Object: " << object_str << "  Action: " << action_str << "  Place: " << place_str <<  "  HAND: " << hand_str << std::endl;

                            std::vector<goodguy::DecisionMaker::action_type> action_list;

                            bool is_uni_action = false;
                            if(action_str == "approach"){
                                action_list.push_back(APPROACH);
                                is_uni_action = true;
                            }
                            else if(action_str == "grasp"){
                                action_list.push_back(GRASP);
                                is_uni_action = true;
                            }
                            else if(action_str == "pour"){
                                action_list.push_back(POUR_GRASP);
                                action_list.push_back(POUR_MOVE);
                                action_list.push_back(POUR_TILT);
                                action_list.push_back(POUR_STANDUP);
                            }
                            else if(action_str == "keep"){
                                action_list.push_back(KEEP);
                                is_uni_action = true;
                            }
                            else if(action_str == "locate"){
                                action_list.push_back(LOCATE);
                                action_list.push_back(RELEASE);
                            }
                            else if(action_str == "throw"){
                                action_list.push_back(THROW);
                                action_list.push_back(RELEASE);
                            }
                            else if(action_str == "move"){
                                action_list.push_back(MOVE);
                                is_uni_action = true;
                            }
                            else if(action_str == "release"){
                                action_list.push_back(RELEASE);
                            }
                            else{
                                is_possible_to_perform_episode = false;
                                std::cout << "Not Known Action : " << action_str << std::endl;
                            }
                            bool is_detected = false;
                            DetectedObjectPtr target_object;
                            DetectedObjectPtr target_place;
                            for(auto& object : objects){
                                if(object->m_object == object_str){
                                    target_object = object;
                                    is_detected = true;
                                    break;
                                }
                            }
                            if(is_detected == false){
                                is_possible_to_perform_episode = false;
                                std::cout << "Not Detected Object: " << object_str << std::endl;
                            }

                            if(!is_uni_action){
                                is_detected = false;
                                for(auto& object : objects){
                                    if(object->m_object == place_str){
                                        target_place = object;
                                        is_detected = true;
                                        break;
                                    }
                                }
                                if(is_detected == false){
                                    is_possible_to_perform_episode = false;
                                    std::cout << "Not Detected Object: " << place_str << std::endl;
                                }
                            }
                            else{
                                target_place = target_object;
                            }

                            if(is_possible_to_perform_episode == false)   break;

                            gripper selected_gripper = select_empty_gripper(*target_object, gripper_state_for_right, gripper_state_for_left);

                            if(hand_str == "left" || hand_str == "LEFT" || hand_str == "Left"){
                                selected_gripper = GRIPPER_LEFT;
                            }
                            else if(hand_str == "right" || hand_str == "RIGHT"  || hand_str == "Right"){
                                selected_gripper = GRIPPER_RIGHT;
                            }
                            else if(hand_str == "rrt"){
                                is_rrt = true;

                            }

                            for(auto action: action_list){
                                event_list.push_back(std::make_tuple(target_object, action, target_place, selected_gripper));
                            }
                        }

                        if(is_possible_to_perform_episode){

                            std::vector<std::shared_ptr<goodguy::Behavior>> behavior_list;
                            m_robot_state = std::make_shared<robot_state::RobotState>(m_upper_body_motion.get_current_robot_state());


                            std::cout << get_eef_pose(GRIPPER_RIGHT).matrix() << std::endl;;
                            std::vector<DetectedObject> prev_objects;

                            for(std::size_t i = 0 ; i < event_list.size(); ++i){
                                event_type& event = event_list[i];
                                DetectedObjectPtr& object = std::get<0>(event);
                                DetectedObjectPtr& place = std::get<2>(event);
                                action_type& action_t = std::get<1>(event);
                                gripper& selected_gripper = std::get<3>(event);

                                print_event(event);
                                voice(std::string("Planning ") + get_event_description(event));
                                behavior_list.push_back(get_behavior_to_make_voice(get_event_description(event)));
                                std::vector<std::shared_ptr<goodguy::Behavior>> event_behavior_list;
                                if(get_behavior_lists(object, place, action_t, selected_gripper, gripper_state_for_left, gripper_state_for_right, prev_objects, event_behavior_list, is_rrt)){
                                    behavior_list.insert(behavior_list.end(), event_behavior_list.begin(), event_behavior_list.end());
                                }
                                else{
                                    behavior_list.insert(behavior_list.end(), event_behavior_list.begin(), event_behavior_list.end());
                                    break;
                                }
                            }

                            bool is_can_do_behavior = true;
                            for(auto behavior : behavior_list){
                                if(behavior == NULL){
                                    std::cout << "FAIL TO PERFORM BEHAVIOR!!" << std::endl;
                                    is_can_do_behavior = false;
                                    break;
                                }
                            }
                            if(is_can_do_behavior){
                                for(auto behavior : behavior_list){
                                    perform_behavior(behavior);
                                }
                                see_center();
                                voice("Wow, Success!");
                                perform_behavior(get_behavior_to_express_emotion(EMOTION_SURPRISE));
                                perform_behavior(get_behavior_to_express_emotion(EMOTION_NEUTRAL));
                            }                
                            else{
                                voice("Sorry, I failed");
                                perform_behavior(get_behavior_to_express_emotion(EMOTION_SADNESS));
                                perform_behavior(get_behavior_to_express_emotion(EMOTION_NEUTRAL));
                                voice("Do you  wanna retry?");
                                std::cout << "Are you retry?? [Y/N] : " ;
                                char input = get_input<char>();
                                if(input == 'y' || input == 'Y')    retry = true;
                            }
                            behavior_list.clear();
                        }
                        else{
                            voice("Sorry, I failed");
                            perform_behavior(get_behavior_to_express_emotion(EMOTION_SADNESS));
                            perform_behavior(get_behavior_to_express_emotion(EMOTION_NEUTRAL));
                            voice("Are you retry?");
                            std::cout << "FAIL TO PERFORM EPISODE!!" << std::endl;
                            std::cout << "Are you retry?? [Y/N] : " ;
                            char input = get_input<char>();
                            if(input == 'y' || input == 'Y')    retry = true;
                        }

                        if(retry) --index_for_sub_episode;

                    }
                }




                m_mutex_for_event.unlock();
            }

            template <typename T>
            static T getInput(){
                T result;
                std::cin >> result;
                if(std::cin.fail() || std::cin.get() != '\n'){
                    std::cin.clear();
                    while (std::cin.get() != '\n');
                    throw std::ios_base::failure("Invalid input");
                }
                return result;
            }



            template <typename T> T get_input()
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
            void event_callback(const decision_maker::EventListConstPtr& msg){
                if(m_mutex_for_event.try_lock()){
                    m_current_event_list = msg->event_list;
                    for(auto event : m_current_event_list){
                        std::cout << "EVENT: " << event << std::endl;
                    }
                    if(m_thread_for_event.joinable()){
                        m_thread_for_event.join();
                    }
                    //event_handler();
                    m_thread_for_event = std::thread(&DecisionMaker::event_handler, this);
                }
                else{
                    std::cout << "CURRENT EVENT IS NOT FINISHED!!" << std::endl;
                }
            }


        private:

            ros::Time m_start_time;
            ros::Duration m_sleep_time;

            std::size_t m_object_index;

            ros::Publisher m_pub_co;
            ros::Publisher m_pub_aco;
            ros::Publisher m_pub_gaze;
            ros::Publisher m_pub_gripper_right;
            ros::Publisher m_pub_gripper_left;
            ros::Publisher m_pub_target_pose;
            ros::Publisher m_pub_facial_expression;
            ros::Publisher m_pub_voice;
            ros::Publisher m_pub_pan_tilt;

            ros::ServiceClient m_service_clear_octomap;
            ros::ServiceClient m_service_go_target;

            std::string m_save_path;

            MybotActionNode m_upper_body_motion;

            std::shared_ptr<robot_state::RobotState> m_robot_state;
            std::shared_ptr<robot_state::RobotState> m_robot_state_for_grasp;

            std::vector<std::string> m_current_event_list;
            std::mutex m_mutex_for_event;

            std::vector<Object> m_support_objects;

            std::list<DetectedObject> m_objects;

            tf::TransformListener m_tf_listener;


            std::thread m_thread_for_erase_objects;
            std::thread m_thread_for_event;

            std::mutex m_mutex_for_objects;
    };
}





int main(int argc, char** argv){

    ros::init(argc, argv, "decision_maker");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle node_handle("~");


    //ros::Subscriber sub_event = node_handle.subscribe<std_msgs::String>("event", 1, boost::bind(decision_maker, image_point_callback, _1));
    goodguy::DecisionMaker decision_maker;

    ros::Subscriber sub_image_point = node_handle.subscribe<simple_recognition::RecogObject>("/recognition/object_point", 1, &goodguy::DecisionMaker::image_point_callback, &decision_maker);
    ros::Subscriber sub_event = node_handle.subscribe<decision_maker::EventList>("event_list", 1, &goodguy::DecisionMaker::event_callback, &decision_maker);

    ros::waitForShutdown();
    ros::shutdown();

    return 0;
}
