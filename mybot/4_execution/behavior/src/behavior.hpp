#include <ros/ros.h>
#include <iostream>

#include <pluginlib/class_loader.h>

#include <actionlib/client/simple_action_client.h>

#include <shape_tools/solid_primitive_dims.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>


#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <eigen_conversions/eigen_msg.h>


#include <moveit/pick_place/pick_place.h>
#include <moveit/move_group/move_group_context.h>


#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/PlanningSceneComponents.h>
#include <std_srvs/Empty.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <simple_recognition/RecogObject.h>
#include <decision_maker/ActionResult.h>
#include <decision_maker/ApproachToObjectAction.h>
#include <decision_maker/GraspObjectAction.h>
#include <decision_maker/MoveToObjectAction.h>
#include <decision_maker/ReleaseAction.h>
#include <decision_maker/TiltObjectAction.h>
#include <decision_maker/InitializeAction.h>

#include <std_msgs/UInt16.h>



namespace goodguy{
    class MybotActionNode{
        public:
            MybotActionNode();
            void clearObjects();

            void setAllowCollisionToObject(const std::string& object_name, bool is_allowed, const std::vector<std::string>& link_names);
            void setAllowCollisionToObjectForLeft(const std::string& object_name, bool is_allowed);
            void setAllowCollisionToObjectForRight(const std::string& object_name, bool is_allowed);
            bool rotate_hand_left(const std::vector<Eigen::Quaterniond>& rotations, const moveit_msgs::Constraints& path_constraints = moveit_msgs::Constraints());
            bool rotate_hand_right(const std::vector<Eigen::Quaterniond>& rotations, const moveit_msgs::Constraints& path_constraints = moveit_msgs::Constraints());
            bool move_arm_left(const Eigen::Vector3d& position, const std::vector<Eigen::Quaterniond>& rotations, const moveit_msgs::Constraints& path_constraints = moveit_msgs::Constraints());
            bool move_only_arm_left(const Eigen::Vector3d& position, const std::vector<Eigen::Quaterniond>& rotations, const moveit_msgs::Constraints& path_constraints = moveit_msgs::Constraints());
            bool move_arm_right(const Eigen::Vector3d& position, const std::vector<Eigen::Quaterniond>& rotations, const moveit_msgs::Constraints& path_constraints = moveit_msgs::Constraints());
            bool move_only_arm_right(const Eigen::Vector3d& position, const std::vector<Eigen::Quaterniond>& rotations, const moveit_msgs::Constraints& path_constraints = moveit_msgs::Constraints());
            bool move_point_arm_right(const Eigen::Vector3d& position, const moveit_msgs::Constraints& path_constraints = moveit_msgs::Constraints());
            bool move_point_arm_left(const Eigen::Vector3d& position, const moveit_msgs::Constraints& path_constraints = moveit_msgs::Constraints());
            bool move_dual_arm(const Eigen::Vector3d& position_r, const Eigen::Quaterniond& rotation_r, const Eigen::Vector3d& position_l, const Eigen::Quaterniond& rotation_l, const moveit_msgs::Constraints& path_constraints = moveit_msgs::Constraints());

            void move_linear_mobile(double dx, double dy);
            void rotate_mobile(double d_theta);
            void waist_up();
            void waist_down();
            void waist_cw_rotation();
            void waist_ccw_rotation();
            void waist_standup();
            void strech_elbow_right();
            void flex_elbow_right();
            void move_arm_right_back();
            void move_arm_right_center();
            void move_arm_right_forward();
            double get_tilt_angle_arm_left();
            double get_tilt_angle_arm_right();
            void tilt_arm_left(double tilt_angle);
            void tilt_arm_right(double tilt_angle);
            void move_arm_left_back();
            void move_arm_left_center();
            void move_arm_left_forward();
            bool move_to_default_for_arm_left();
            bool move_to_default_for_arm_right();
            bool move_to_default_for_dual_arm();
            void grasp_by_left();
            void release_by_left();
            void grasp_by_right();
            void release_by_right();


            void set_bottle_mouth_right(const Eigen::Vector3d& pos);
            void set_bottle_mouth_left(const Eigen::Vector3d& pos);


            static moveit_msgs::Constraints getPointConstraint(
                    const std::string& eef_link_name, 
                    const Eigen::Vector3d& eef_link_point, 
                    double tolerance_point)
            {

                geometry_msgs::Point eef_link_point_;
                eef_link_point_.x = eef_link_point(0);
                eef_link_point_.y = eef_link_point(1);
                eef_link_point_.z = eef_link_point(2);
                return getPointConstraint(eef_link_name, eef_link_point_, tolerance_point);
            }


            static moveit_msgs::Constraints getPointConstraint(
                    const std::string& eef_link_name, 
                    const geometry_msgs::Point& eef_link_point, 
                    double tolerance_point)
            {

                geometry_msgs::PointStamped stamped_eef_link_point;
                stamped_eef_link_point.header.frame_id = "base_footprint";
                stamped_eef_link_point.point = eef_link_point;
                return kinematic_constraints::constructGoalConstraints(eef_link_name, stamped_eef_link_point, tolerance_point);
            }
            static moveit_msgs::Constraints getPoseConstraint(
                    const std::string& eef_link_name, 
                    const Eigen::Vector3d& eef_link_point,
                    const Eigen::Quaterniond& eef_link_orientation,
                    double t_p, 
                    double t_a)
            {

                geometry_msgs::Pose eef_link_pose;
                eef_link_pose.position.x = eef_link_point(0);
                eef_link_pose.position.y = eef_link_point(1);
                eef_link_pose.position.z = eef_link_point(2);
                eef_link_pose.orientation.x = eef_link_orientation.x();
                eef_link_pose.orientation.y = eef_link_orientation.y();
                eef_link_pose.orientation.z = eef_link_orientation.z();
                eef_link_pose.orientation.w = eef_link_orientation.w();
                return getPoseConstraint(eef_link_name, eef_link_pose, t_p, t_a);

            }
            static moveit_msgs::Constraints getPoseConstraint(
                    const std::string& eef_link_name, 
                    const geometry_msgs::Pose& eef_link_pose, 
                    double t_p, 
                    double t_a)
            {

                geometry_msgs::PoseStamped stamped_eef_link_pose;
                stamped_eef_link_pose.header.frame_id = "base_footprint";
                stamped_eef_link_pose.pose = eef_link_pose;
                std::vector<double> tolerance_pose(3, t_p);
                std::vector<double> tolerance_angle(3, t_a);

                tolerance_angle[1] = t_a*10.0; 
                return kinematic_constraints::constructGoalConstraints(eef_link_name, stamped_eef_link_pose, tolerance_pose, tolerance_angle);

            }

            static moveit_msgs::Constraints getOrientationConstraint(
                    const std::string& eef_link_name, 
                    const Eigen::Quaterniond& eef_link_orientation, 
                    double tolerance_angle)
            {
                geometry_msgs::Quaternion eef_link_orientation_;
                eef_link_orientation_.x = eef_link_orientation.x();
                eef_link_orientation_.y = eef_link_orientation.y();
                eef_link_orientation_.z = eef_link_orientation.z();
                eef_link_orientation_.w = eef_link_orientation.w();
                return getOrientationConstraint(eef_link_name, eef_link_orientation_, tolerance_angle);
            }

            static moveit_msgs::Constraints getOrientationConstraint(
                    const std::string& eef_link_name, 
                    const geometry_msgs::Quaternion& eef_link_orientation, 
                    double tolerance_angle)
            {
                geometry_msgs::QuaternionStamped stamped_eef_link_orientation;
                stamped_eef_link_orientation.header.frame_id = "base_footprint";
                stamped_eef_link_orientation.quaternion = eef_link_orientation;
                return kinematic_constraints::constructGoalConstraints(eef_link_name, stamped_eef_link_orientation, tolerance_angle);
            }


            //static constexpr const auto& default_config = "QRRTstarConfigDefault";
            //static constexpr const auto& default_config = "KRRTstarConfigDefault";
            static constexpr const auto& default_config = "RRTConnectkConfigDefault";

            bool generate_motion_plan(const std::string& move_arm_group_name, const std::vector<moveit_msgs::Constraints>& goal_constraints, const moveit_msgs::Constraints& path_constraint, moveit_msgs::RobotTrajectory& generated_traj, robot_state::RobotState start_state, const std::string& motion = std::string(default_config));
            bool generate_motion_plan(const std::string& move_arm_group_name, const std::vector<moveit_msgs::Constraints>& goal_constraints, const moveit_msgs::Constraints& path_constraint, moveit_msgs::RobotTrajectory& generated_traj, moveit_msgs::RobotState& start_state_msg, const std::string& motion = std::string(default_config));
            bool execute_trajectory(const moveit_msgs::RobotTrajectory& traj);
            robot_state::RobotState get_current_robot_state();

            std::vector<double> getCurrentGripperRightAngles() ;
            std::vector<double> getCurrentGripperLeftAngles() ;
            std::vector<double> getCurrentArmRightAngles() ;
            std::vector<double> getCurrentArmLeftAngles() ;
            std::vector<double> getCurrentTorsoAngles() ;
            std::vector<double> getCurrentAngles(const std::string& joint_group_name) ;

            bool is_something_wrong(moveit_msgs::MoveItErrorCodes& error);
            bool move_to_default(const std::string& move_arm_group_name, const std::string& position_name);
            bool get_trajectory_to_default(const std::string& move_arm_group_name, const std::string& position_name, moveit_msgs::RobotTrajectory& generated_traj, const robot_state::RobotState& robot_state);
            bool move_point_arm(const std::string& move_arm_group_name, const std::string& eef_link_name, const Eigen::Vector3d& position, const moveit_msgs::Constraints& path_constraints);
            bool generate_point_arm(const std::string& move_arm_group_name, const std::string& eef_link_name, const Eigen::Vector3d& position, const moveit_msgs::Constraints& path_constraints, moveit_msgs::RobotTrajectory& generated_traj);
            bool move_dual_pose_arm(const std::string& move_arm_group_name, const std::string& eef_link_name_1, const Eigen::Vector3d& position_1, const Eigen::Quaterniond& rotation_1, const std::string& eef_link_name_2, const Eigen::Vector3d& position_2, const Eigen::Quaterniond& rotation_2, const moveit_msgs::Constraints& path_constraints);
            bool generate_dual_pose_arm(const std::string& move_arm_group_name, const std::string& eef_link_name_1, const Eigen::Vector3d& position_1, const Eigen::Quaterniond& rotation_1, const std::string& eef_link_name_2, const Eigen::Vector3d& position_2, const Eigen::Quaterniond& rotation_2, const moveit_msgs::Constraints& path_constraints, moveit_msgs::RobotTrajectory& generated_traj);
            bool move_arm(const std::string& move_arm_group_name, 
                    const std::string& eef_link_name, const Eigen::Vector3d& position, const std::vector<Eigen::Quaterniond>& rotations, const moveit_msgs::Constraints& path_constraints);
            trajectory_msgs::JointTrajectory getSingleJointTrajectoryMsg(const std::vector<std::string>& names, const std::vector<double>& angles, double time_from_start = 1.0);
            trajectory_msgs::JointTrajectory getJointTrajectoryMsgForTrunk(const std::vector<double>& angles, double time_from_start = 1.0);
            trajectory_msgs::JointTrajectory getJointTrajectoryMsgForArmLeft(const std::vector<double>& angles, double time_from_start = 1.0);
            trajectory_msgs::JointTrajectory getJointTrajectoryMsgForArmRight(const std::vector<double>& angles, double time_from_start = 1.0);
            trajectory_msgs::JointTrajectory getJointTrajectoryMsgForGripperRight(const std::vector<double>& angles, double time_from_start = 1.0);
            trajectory_msgs::JointTrajectory getJointTrajectoryMsgForGripperLeft(const std::vector<double>& angles, double time_from_start = 1.0);

        public:
            robot_model::RobotModelPtr  m_robot_model;

        private:

            std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>> m_action_execute;


            ros::ServiceClient m_service_get_planning_scene;
            ros::ServiceClient m_service_motion_planning;
            ros::ServiceClient m_service_execute;
            ros::ServiceClient m_service_clear_octomap;

            bool m_is_image_point_update;



            ros::Publisher m_pub_mobile;
            ros::Publisher m_pub_bottle_right;
            ros::Publisher m_pub_bottle_left;
            ros::Publisher m_pub_facial;
            ros::Publisher m_pub_trunk;
            ros::Publisher m_pub_arm_right;
            ros::Publisher m_pub_arm_left;
            ros::Publisher m_pub_gripper_right;
            ros::Publisher m_pub_gripper_left;
            ros::Publisher m_pub_display;
            ros::Publisher m_pub_speech;

            ros::Publisher m_pub_scene;

            ros::WallDuration m_sleep_time;

            unsigned long m_counter = 0;


    };
};

