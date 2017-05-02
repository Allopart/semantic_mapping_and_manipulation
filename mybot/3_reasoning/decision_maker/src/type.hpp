#include <ros/ros.h>

#include <shape_tools/solid_primitive_dims.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <std_msgs/String.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <simple_recognition/RecogObject.h>

#include <decision_maker/ApproachToObjectAction.h>
#include <decision_maker/GraspObjectAction.h>
#include <decision_maker/MoveToObjectAction.h>
#include <decision_maker/ReleaseAction.h>
#include <decision_maker/ActionResult.h>
#include <decision_maker/TiltObjectAction.h>
#include <decision_maker/InitializeAction.h>

#include <boost/algorithm/string.hpp>

namespace goodguy{
    enum object_type{
        TOY_CYLINDER,
        TOY_TRIANGLE,
        TOY_ARCH,
        TOY_SQUARE,
        TOY_BOX,
        CEREAL,
        MILK,
        JUICE,
        BOWL,
        CUP,
        LOC
    };

    enum action_type{
        APPROACH_TO_OBJECT,
        GRASP_OBJECT,
        MOVE_TO_OBJECT,
        MOVE_TO_LOCATION,
        RELEASE,
        GO_TO_INITIAL,
        TILT
    };

    enum action_result_type{
        ACTION_FAIL,
        ACTION_SUCCESS
    };

    enum arm_type{
        ARM_RIGHT,
        ARM_LEFT,
        ARM_NONE
    };

    class Object{
        public:
            Object(): missed_count(0) {}

            Object(const std::string& name_, goodguy::object_type type_, const Eigen::Vector3d& position_)
                : name(name_), type(type_), pose(Eigen::Affine3d::Identity()), missed_count(0), is_attached(false)
            {
                pose(0,3) = position_(0);
                pose(1,3) = position_(1);
                pose(2,3) = position_(2);
            }
            


        public:
            std::string name;
            goodguy::object_type type;
            Eigen::Affine3d pose;
            bool is_attached;
            Eigen::Quaterniond grasping_pose;
            arm_type grasp_arm_type;

        private:
            int missed_count;

        public:
            void set_name(const std::string& name_){
                name = name_;
            }

            void set_type(goodguy::object_type type_){
                type = type_;
            }

            void set_position(const Eigen::Vector3d& position_){
                pose = Eigen::Affine3d::Identity();
                pose(0,3) = position_(0);
                pose(1,3) = position_(1);
                pose(2,3) = position_(2);
                missed_count = 0;
            }

            void update_position(const Eigen::Vector3d& received_position){
                pose.translation().x() = received_position(0);
                pose.translation().y() = received_position(1);
                pose.translation().z() = received_position(2);
            }


            void attach_mode(arm_type arm_t, Eigen::Quaterniond grasp_pose = Eigen::Quaterniond::Identity()){
                grasping_pose = grasp_pose;
                grasp_arm_type = arm_t;
                is_attached = true;
            }
            void detach_mode(){
                is_attached = false;
            }

            moveit_msgs::AttachedCollisionObject attach_object(arm_type arm_t, Eigen::Quaterniond grasp_pose = Eigen::Quaterniond::Identity()) const {
                moveit_msgs::AttachedCollisionObject aco;

                std::string touch_link_name;
                std::vector<std::string> attached_link_names;

                if(arm_t == ARM_LEFT){
                    touch_link_name  = "left_gripper_parm_0";
                    attached_link_names.push_back("left_gripper_parm_0");
                    attached_link_names.push_back("left_gripper_parm_1");
                    attached_link_names.push_back("left_gripper_thumb_0");
                    attached_link_names.push_back("left_gripper_thumb_1");
                    attached_link_names.push_back("left_gripper_forefinger_0");
                    attached_link_names.push_back("left_gripper_forefinger_1");
                    attached_link_names.push_back("left_gripper_littlefinger_0");
                    attached_link_names.push_back("left_gripper_littlefinger_1");
                }
                else if(arm_t == ARM_RIGHT){
                    touch_link_name  = "right_gripper_parm_0";
                    attached_link_names.push_back("right_gripper_parm_0");
                    attached_link_names.push_back("right_gripper_parm_1");
                    attached_link_names.push_back("right_gripper_thumb_0");
                    attached_link_names.push_back("right_gripper_thumb_1");
                    attached_link_names.push_back("right_gripper_forefinger_0");
                    attached_link_names.push_back("right_gripper_forefinger_1");
                    attached_link_names.push_back("right_gripper_littlefinger_0");
                    attached_link_names.push_back("right_gripper_littlefinger_1");
                }
                aco.link_name = touch_link_name;
                aco.touch_links = attached_link_names;
                aco.object = collision_object();
                aco.object.header.frame_id = touch_link_name;
                geometry_msgs::Pose pose;
                pose.orientation.w = 1.0;

                aco.object.primitive_poses.clear();

                Eigen::Quaterniond inverse_grasp_pose = grasp_pose.inverse();
                for(int i = 0; i < aco.object.primitives.size(); ++i){
                    aco.object.primitive_poses.push_back(pose);

                    aco.object.primitive_poses[i].orientation.x = inverse_grasp_pose.x();
                    aco.object.primitive_poses[i].orientation.y = inverse_grasp_pose.y();
                    aco.object.primitive_poses[i].orientation.z = inverse_grasp_pose.z();
                    aco.object.primitive_poses[i].orientation.w = inverse_grasp_pose.w();
                }

                return aco;
            }

            moveit_msgs::CollisionObject collision_object() const {
                moveit_msgs::CollisionObject co;
                co.id = name;
                switch(type){
                    case goodguy::TOY_CYLINDER:
                        {
                            co.primitives.resize(5);
                            co.primitive_poses.resize(co.primitives.size());
                            for(int i = 0; i < co.primitives.size(); ++i){
                                co.primitives[i].type = shape_msgs::SolidPrimitive::CYLINDER;
                                co.primitives[i].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.01 + i*0.005;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.04 + i*0.01;
                                tf::poseEigenToMsg(pose, co.primitive_poses[i]);
                                co.primitive_poses[i].position.z += 0.04;
                            }
                        }
                        break;
                    case goodguy::TOY_TRIANGLE:
                        {
                            co.primitives.resize(4);
                            co.primitive_poses.resize(co.primitives.size());
                            for(int i = 0; i < co.primitives.size(); ++i){
                                co.primitives[i].type = shape_msgs::SolidPrimitive::BOX;
                                co.primitives[i].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.01 + i*0.01;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.05 + i*0.01;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05 + i*0.01;
                                tf::poseEigenToMsg(pose, co.primitive_poses[i]);
                                Eigen::AngleAxisd rotate_1(45.0*3.14/180.0, Eigen::Vector3d::UnitX());
                                Eigen::AngleAxisd rotate_2(0, Eigen::Vector3d::UnitY());

                                Eigen::Quaterniond rotation = rotate_1*rotate_2;
                                tf::poseEigenToMsg(pose, co.primitive_poses[i]);
                                co.primitive_poses[i].orientation.x = rotation.x();
                                co.primitive_poses[i].orientation.y = rotation.y();
                                co.primitive_poses[i].orientation.z = rotation.z();
                                co.primitive_poses[i].orientation.w = rotation.w();
                            }
                        }
                        break;
                    case goodguy::TOY_ARCH:
                        {
                            co.primitives.resize(5);
                            co.primitive_poses.resize(co.primitives.size());
                            for(int i = 0; i < co.primitives.size(); ++i){
                                co.primitives[i].type = shape_msgs::SolidPrimitive::BOX;
                                co.primitives[i].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.02 + i*0.005;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.04 + i*0.01;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.02 + i*0.005;
                                tf::poseEigenToMsg(pose, co.primitive_poses[i]);
                                co.primitive_poses[i].position.z += 0.02;
                            }
                        }
                        break;
                    case goodguy::TOY_SQUARE:
                        {
                            co.primitives.resize(7);
                            co.primitive_poses.resize(co.primitives.size());
                            for(int i = 0; i < co.primitives.size(); ++i){
                                co.primitives[i].type = shape_msgs::SolidPrimitive::BOX;
                                co.primitives[i].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.02 + i*0.006;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.04 + i*0.01;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.04 + i*0.01;
                                tf::poseEigenToMsg(pose, co.primitive_poses[i]);
                                co.primitive_poses[i].position.z += 0.04;
                            }
                        }
                        break;
                    case goodguy::TOY_BOX:
                        {
                            co.primitives.resize(6);
                            co.primitive_poses.resize(co.primitives.size());
                            for(int i = 0; i < co.primitives.size(); ++i){
                                co.primitives[i].type = shape_msgs::SolidPrimitive::BOX;
                                co.primitives[i].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.06 + i*0.06;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.08 + i*0.02;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.10 + i*0.01;
                                tf::poseEigenToMsg(pose, co.primitive_poses[i]);
                                co.primitive_poses[i].position.x += 0.13;
                                co.primitive_poses[i].position.z += 0.07;
                            }
                        }
                        break;
                    case goodguy::CEREAL:
                        {
                            co.primitives.resize(3);
                            co.primitive_poses.resize(co.primitives.size());
                            for(int i = 0; i < co.primitives.size(); ++i){
                                co.primitives[i].type = shape_msgs::SolidPrimitive::BOX;
                                co.primitives[i].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);

                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.02 + i*0.01;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.045 + i*0.02;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.065 + i*0.02;
                                tf::poseEigenToMsg(pose, co.primitive_poses[i]);
                                co.primitive_poses[i].position.z += 0.02;
                            }
                        }
                        break;
                    case goodguy::MILK:
                        {
                            co.primitives.resize(3);
                            co.primitive_poses.resize(co.primitives.size());
                            for(int i = 0; i < co.primitives.size(); ++i){
                                co.primitives[i].type = shape_msgs::SolidPrimitive::BOX;
                                co.primitives[i].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.02 + i*0.02;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.02 + i*0.02;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.065 + i*0.02;
                                tf::poseEigenToMsg(pose, co.primitive_poses[i]);
                                co.primitive_poses[i].position.z += 0.05;
                            }
                        }
                        break;

                    case goodguy::JUICE:
                        {

                            co.primitives.resize(3);
                            co.primitive_poses.resize(co.primitives.size());
                            for(int i = 0; i < co.primitives.size(); ++i){
                                co.primitives[i].type = shape_msgs::SolidPrimitive::CYLINDER;
                                co.primitives[i].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);

                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.025 + i*0.005;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.140+ i*0.02;
                                tf::poseEigenToMsg(pose, co.primitive_poses[i]);
                                co.primitive_poses[i].position.z += 0.09;
                            }
                        }
                        break;
                    case goodguy::BOWL:
                        {
                            co.primitives.resize(3);
                            co.primitive_poses.resize(co.primitives.size());
                            for(int i = 0; i < co.primitives.size(); ++i){
                                co.primitives[i].type = shape_msgs::SolidPrimitive::CYLINDER;
                                co.primitives[i].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.03 + i*0.02;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.03 + i*0.01;
                                tf::poseEigenToMsg(pose, co.primitive_poses[i]);
                                co.primitive_poses[i].position.z += 0.025;
                            }
                        }
                        break;
                    case goodguy::CUP:
                        {
                            co.primitives.resize(3);
                            co.primitive_poses.resize(co.primitives.size());
                            for(int i = 0; i < co.primitives.size(); ++i){
                                co.primitives[i].type = shape_msgs::SolidPrimitive::CYLINDER;
                                co.primitives[i].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.025 + i*0.005;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.065 + i*0.02;
                                tf::poseEigenToMsg(pose, co.primitive_poses[i]);
                                co.primitive_poses[i].position.z += 0.05;
                            }
                        }
                        break;
                    case goodguy::LOC:
                        {
                            co.primitives.resize(6);
                            co.primitive_poses.resize(co.primitives.size());
                            for(int i = 0; i < co.primitives.size(); ++i){
                                co.primitives[i].type = shape_msgs::SolidPrimitive::BOX;
                                co.primitives[i].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.06 + i*0.06;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.08 + i*0.02;
                                co.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.10 + i*0.01;
                                tf::poseEigenToMsg(pose, co.primitive_poses[i]);
                                co.primitive_poses[i].position.x += 0.13;
                                co.primitive_poses[i].position.z += 0.07;
                            }
                        }
                        break;
                }

                return co;
            }
    };

    bool operator==(const goodguy::Object& obj1, const goodguy::Object& obj2){
        return obj1.name == obj2.name;
    }


    struct ArmState{
        enum State{
            IDLE,
            GRASP,
            RELEASE
        };

        ArmState(): curr_state(IDLE), prev_state(IDLE)
        { }

        bool is_grasp_target(const goodguy::Object& target) const {
            return (curr_state == GRASP && target == grasp_object);
        }
        bool is_grasp() const{
            return curr_state == GRASP;
        }

        bool is_release() const{
            return curr_state == RELEASE;
        }

        bool is_idle() const{
            return curr_state == IDLE;
        }

        void update(State state, Object obj = Object()){
            prev_state = curr_state;
            curr_state = state;
            grasp_object = obj;
        }

        State curr_state;
        State prev_state;
        Object grasp_object;
    };
}

