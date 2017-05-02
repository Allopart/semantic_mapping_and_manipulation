#include "behavior.hpp"

namespace goodguy{

MybotActionNode::MybotActionNode(): m_sleep_time(0.3), m_is_image_point_update(false) {
    ros::NodeHandle node_handle;
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    m_robot_model = robot_model_loader.getModel();

    m_pub_trunk = node_handle.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1);
    m_pub_mobile = node_handle.advertise<geometry_msgs::Twist>("/mobile_controller/cmd_vel", 1);
    m_pub_bottle_right = node_handle.advertise<trajectory_msgs::JointTrajectory>("/bottle_mouth_right_controller/command", 1);
    m_pub_bottle_left = node_handle.advertise<trajectory_msgs::JointTrajectory>("/bottle_mouth_left_controller/command", 1);
    m_pub_arm_right = node_handle.advertise<trajectory_msgs::JointTrajectory>("/arm_right_controller/command", 100);
    m_pub_arm_left = node_handle.advertise<trajectory_msgs::JointTrajectory>("/arm_left_controller/command", 100);
    m_pub_gripper_right = node_handle.advertise<trajectory_msgs::JointTrajectory>("/gripper_right_controller/command", 1);
    m_pub_gripper_left = node_handle.advertise<trajectory_msgs::JointTrajectory>("/gripper_left_controller/command", 1);
    m_pub_display = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1);
    m_pub_facial = node_handle.advertise<std_msgs::UInt16>("/facial_expression/emotion", 1);
    m_pub_speech = node_handle.advertise<std_msgs::String>("/speech_interaction/tts/text", 1);

    m_pub_scene = node_handle.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

    m_action_execute.reset(new actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>(std::string("execute_trajectory"), true));


    m_service_get_planning_scene = node_handle.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    m_service_motion_planning = node_handle.serviceClient<moveit_msgs::GetMotionPlan>("/plan_kinematic_path");
    m_service_execute = node_handle.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");
    m_service_clear_octomap = node_handle.serviceClient<std_srvs::Empty>("/clear_octomap");

    m_sleep_time.sleep();

}



void MybotActionNode::clearObjects(){
    planning_scene::PlanningScene planning_scene(m_robot_model);
    moveit_msgs::PlanningScene planning_scene_diff_msg;
    planning_scene.getPlanningSceneDiffMsg(planning_scene_diff_msg);
    planning_scene_diff_msg.is_diff = false;
    m_pub_scene.publish(planning_scene_diff_msg);
    //std::cout << planning_scene_diff_msg.allowed_collision_matrix << std::endl;
    m_sleep_time.sleep();
}

void MybotActionNode::setAllowCollisionToObject(const std::string& object_name, bool is_allowed, const std::vector<std::string>& link_names){
    moveit_msgs::GetPlanningSceneResponse get_planning_scene_response;
    moveit_msgs::GetPlanningSceneRequest get_planning_scene_request;
    get_planning_scene_request.components.components 
        = moveit_msgs::PlanningSceneComponents::ROBOT_STATE 
        | moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS 
        | moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX 
        | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES 
        | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
    m_service_get_planning_scene.call(get_planning_scene_request, get_planning_scene_response);
    if(planning_scene::PlanningScene::isEmpty(get_planning_scene_response.scene)){
        std::cout << "[ERROR] EMPTY SCENE IS ARRIVED" << std::endl;
        return;
    }
    planning_scene::PlanningScene planning_scene(m_robot_model);
    planning_scene.setPlanningSceneMsg(get_planning_scene_response.scene);
    collision_detection::AllowedCollisionMatrix& acm = planning_scene.getAllowedCollisionMatrixNonConst();
    acm.setEntry(object_name, link_names, is_allowed);

    moveit_msgs::PlanningScene planning_scene_diff_msg;
    planning_scene.getPlanningSceneDiffMsg(planning_scene_diff_msg);
    planning_scene_diff_msg.is_diff = true;
    m_pub_scene.publish(planning_scene_diff_msg);
    //std::cout << planning_scene_diff_msg.allowed_collision_matrix << std::endl;
    m_sleep_time.sleep();
}


void MybotActionNode::setAllowCollisionToObjectForLeft(const std::string& object_name, bool is_allowed){
    std::vector<std::string> link_names;
    link_names.push_back("left_gripper_parm_0");
    link_names.push_back("left_gripper_parm_1");
    link_names.push_back("left_gripper_thumb_0");
    link_names.push_back("left_gripper_thumb_1");
    link_names.push_back("left_gripper_forefinger_0");
    link_names.push_back("left_gripper_forefinger_1");
    link_names.push_back("left_gripper_littlefinger_0");
    link_names.push_back("left_gripper_littlefinger_1");
    setAllowCollisionToObject(object_name, is_allowed, link_names);
}
void MybotActionNode::setAllowCollisionToObjectForRight(const std::string& object_name, bool is_allowed){
    std::vector<std::string> link_names;
    link_names.push_back("right_gripper_parm_0");
    link_names.push_back("right_gripper_parm_1");
    link_names.push_back("right_gripper_thumb_0");
    link_names.push_back("right_gripper_thumb_1");
    link_names.push_back("right_gripper_forefinger_0");
    link_names.push_back("right_gripper_forefinger_1");
    link_names.push_back("right_gripper_littlefinger_0");
    link_names.push_back("right_gripper_littlefinger_1");
    setAllowCollisionToObject(object_name, is_allowed, link_names);
}

bool MybotActionNode::rotate_hand_left(const std::vector<Eigen::Quaterniond>& rotations, const moveit_msgs::Constraints& path_constraints){
    return move_arm("hand_left", "left_bottle_mouth_z", Eigen::Vector3d::Zero(), rotations, path_constraints);
}
bool MybotActionNode::rotate_hand_right(const std::vector<Eigen::Quaterniond>& rotations, const moveit_msgs::Constraints& path_constraints){
    return move_arm("hand_right", "right_bottle_mouth_z", Eigen::Vector3d::Zero(), rotations, path_constraints);
}


bool MybotActionNode::move_arm_left(const Eigen::Vector3d& position, const std::vector<Eigen::Quaterniond>& rotations, const moveit_msgs::Constraints& path_constraints){
    return move_arm("trunk_with_arm_left", "left_gripper_parm_0", position, rotations, path_constraints);
    //return move_arm("trunk_with_arm_left", "left_bottle_mouth_z", position, rotations, path_constraints);
}
bool MybotActionNode::move_only_arm_left(const Eigen::Vector3d& position, const std::vector<Eigen::Quaterniond>& rotations, const moveit_msgs::Constraints& path_constraints){
    return move_arm("arm_left", "left_bottle_mouth_z", position, rotations, path_constraints);
}
bool MybotActionNode::move_arm_right(const Eigen::Vector3d& position, const std::vector<Eigen::Quaterniond>& rotations, const moveit_msgs::Constraints& path_constraints){
    //return move_arm("trunk_with_arm_right", "right_bottle_mouth_z", position, rotations, path_constraints);
    return move_arm("trunk_with_arm_right", "right_gripper_parm_0", position, rotations, path_constraints);
}
bool MybotActionNode::move_only_arm_right(const Eigen::Vector3d& position, const std::vector<Eigen::Quaterniond>& rotations, const moveit_msgs::Constraints& path_constraints){
    return move_arm("arm_right", "right_bottle_mouth_z", position, rotations, path_constraints);
}
bool MybotActionNode::move_point_arm_right(const Eigen::Vector3d& position, const moveit_msgs::Constraints& path_constraints){
    return move_point_arm("trunk_with_arm_right", "right_bottle_mouth_z", position, path_constraints);
}
bool MybotActionNode::move_point_arm_left(const Eigen::Vector3d& position, const moveit_msgs::Constraints& path_constraints){
    return move_point_arm("trunk_with_arm_left", "left_bottle_mouth_z", position, path_constraints);
}
bool MybotActionNode::move_dual_arm(const Eigen::Vector3d& position_r, const Eigen::Quaterniond& rotation_r, const Eigen::Vector3d& position_l, const Eigen::Quaterniond& rotation_l, const moveit_msgs::Constraints& path_constraints){

    return move_dual_pose_arm("dual_arm", "right_bottle_mouth_z", position_r, rotation_r, "left_bottle_mouth_z", position_l, rotation_l, path_constraints);
}


void MybotActionNode::move_linear_mobile(double dx, double dy){
    double linear_velocity = 0.05;
    double distance = std::sqrt(dx*dx+dy*dy);
    double wait_time_sec = distance / linear_velocity;

    double vel_x = dx / distance * linear_velocity;
    double vel_y = dy / distance * linear_velocity;

    double consume_sec = 0.0;
    ros::Duration period(0.1);
    geometry_msgs::Twist msg;

    msg.linear.x = vel_x;
    msg.linear.y = 0;
    msg.linear.y = vel_y;

    while(consume_sec < wait_time_sec){
        consume_sec += 0.1;
        m_pub_mobile.publish(msg);
        period.sleep();
    }


}
void MybotActionNode::rotate_mobile(double d_theta){
    double angular_velocity = 15.0*3.14/180.0;
    double wait_time_sec = std::abs(d_theta) / angular_velocity;
    double consume_sec = 0.0;
    ros::Duration period(0.1);
    geometry_msgs::Twist msg;
    if(d_theta < 0){
        msg.angular.z = -angular_velocity;
    }
    else{
        msg.angular.z = angular_velocity;
    }

    while(consume_sec < wait_time_sec){
        consume_sec += 0.1;
        m_pub_mobile.publish(msg);
        period.sleep();
    }

}


void MybotActionNode::waist_up(){
    std::vector<double> angles(2);
    angles[0] = +0.0/180.0*3.14;
    angles[1] = +70.0/180.0*3.14;
    m_pub_trunk.publish(getJointTrajectoryMsgForTrunk(angles, 5));
}
void MybotActionNode::waist_down(){
    std::vector<double> angles(2);
    angles[0] = +0.0/180.0*3.14;
    angles[1] = -70.0/180.0*3.14;
    m_pub_trunk.publish(getJointTrajectoryMsgForTrunk(angles, 5));
}
void MybotActionNode::waist_cw_rotation(){
    std::vector<double> angles(2);
    angles[0] = -50.0/180.0*3.14;
    angles[1] = 0.0/180.0*3.14;
    m_pub_trunk.publish(getJointTrajectoryMsgForTrunk(angles, 5));
}
void MybotActionNode::waist_ccw_rotation(){
    std::vector<double> angles(2);
    angles[0] = 50.0/180.0*3.14;
    angles[1] = 0.0/180.0*3.14;
    m_pub_trunk.publish(getJointTrajectoryMsgForTrunk(angles, 5));
}

void MybotActionNode::waist_standup(){
    std::vector<double> angles(2);
    angles[0] = 0.0/180.0*3.14;
    angles[1] = 0.0/180.0*3.14;
    m_pub_trunk.publish(getJointTrajectoryMsgForTrunk(angles, 5));
}

void MybotActionNode::strech_elbow_right(){
    std::vector<double> angles = getCurrentArmRightAngles();
    angles[3] += -5.0/180.0*3.14;
    std::cout << "Current Elbow : " << angles[3] * 180.0 / 3.14 << std::endl;
    m_pub_arm_right.publish(getJointTrajectoryMsgForArmRight(angles));
}
void MybotActionNode::flex_elbow_right(){
    std::vector<double> angles = getCurrentArmRightAngles();
    angles[3] += +5.0/180.0*3.14;
    std::cout << "Current Elbow : " << angles[3] * 180.0 / 3.14 << std::endl;
    m_pub_arm_right.publish(getJointTrajectoryMsgForArmRight(angles));
}

void MybotActionNode::move_arm_right_back(){
    std::vector<double> angles = getCurrentArmRightAngles();
    angles[0] = -60.0/180.0*3.14 -30.0/180.0*3.14;
    m_pub_arm_right.publish(getJointTrajectoryMsgForArmRight(angles, 3));
}
void MybotActionNode::move_arm_right_center(){
    std::vector<double> angles = getCurrentArmRightAngles();
    angles[0] = 0.0/180.0*3.14 -30.0/180.0*3.14;
    m_pub_arm_right.publish(getJointTrajectoryMsgForArmRight(angles, 3));
}
void MybotActionNode::move_arm_right_forward(){
    std::vector<double> angles = getCurrentArmRightAngles();
    angles[0] = 60.0/180.0*3.14 -30.0/180.0*3.14;
    m_pub_arm_right.publish(getJointTrajectoryMsgForArmRight(angles, 3));
}


double MybotActionNode::get_tilt_angle_arm_left(){
    std::vector<double> angles = getCurrentArmLeftAngles();
    return angles[4];
}
double MybotActionNode::get_tilt_angle_arm_right(){
    std::vector<double> angles = getCurrentArmRightAngles();
    return angles[4];
}

void MybotActionNode::tilt_arm_left(double tilt_angle){
    std::vector<double> angles = getCurrentArmLeftAngles();
    angles[4] = tilt_angle;
    m_pub_arm_left.publish(getJointTrajectoryMsgForArmLeft(angles, 3));
}
void MybotActionNode::tilt_arm_right(double tilt_angle){
    std::vector<double> angles = getCurrentArmRightAngles();
    angles[4] = tilt_angle;
    m_pub_arm_right.publish(getJointTrajectoryMsgForArmRight(angles, 3));
}

void MybotActionNode::move_arm_left_back(){
    std::vector<double> angles = getCurrentArmLeftAngles();
    angles[0] = +60.0/180.0*3.14 +30.0/180.0*3.14;
    m_pub_arm_left.publish(getJointTrajectoryMsgForArmLeft(angles, 3));
}
void MybotActionNode::move_arm_left_center(){
    std::vector<double> angles = getCurrentArmLeftAngles();
    angles[0] = +0.0/180.0*3.14 +30.0/180.0*3.14;
    m_pub_arm_left.publish(getJointTrajectoryMsgForArmLeft(angles, 3));
}
void MybotActionNode::move_arm_left_forward(){
    std::vector<double> angles = getCurrentArmLeftAngles();
    angles[0] = -60.0/180.0*3.14 +30.0/180.0*3.14;
    m_pub_arm_left.publish(getJointTrajectoryMsgForArmLeft(angles, 3));
}


bool MybotActionNode::move_to_default_for_arm_left(){
    return move_to_default("trunk_with_arm_left", "left_grasp_default");
    //return move_to_default("trunk_with_arm_left", "trunk_with_arm_left_home");
}
bool MybotActionNode::move_to_default_for_arm_right(){
    return move_to_default("trunk_with_arm_right", "right_grasp_default");
    //return move_to_default("trunk_with_arm_right", "trunk_with_arm_right_home");
}
bool MybotActionNode::move_to_default_for_dual_arm(){
    return move_to_default("dual_arm", "dual_arm_home");
}


void MybotActionNode::set_bottle_mouth_right(const Eigen::Vector3d& pos){
    std::vector<std::string> names(3);
    names[0] = "right_bottle_mouth_joint_x";
    names[1] = "right_bottle_mouth_joint_y";
    names[2] = "right_bottle_mouth_joint_z";
    std::vector<double> pos_vec(3, 0);
    pos_vec[0] = pos(0);
    pos_vec[1] = pos(1);
    pos_vec[2] = pos(2);
    m_pub_bottle_right.publish(getSingleJointTrajectoryMsg(names, pos_vec, 0.1));
}

void MybotActionNode::set_bottle_mouth_left(const Eigen::Vector3d& pos){
    std::vector<std::string> names(3);
    names[0] = "left_bottle_mouth_joint_x";
    names[1] = "left_bottle_mouth_joint_y";
    names[2] = "left_bottle_mouth_joint_z";
    std::vector<double> pos_vec(3, 0);
    pos_vec[0] = pos(0);
    pos_vec[1] = pos(1);
    pos_vec[2] = pos(2);
    m_pub_bottle_left.publish(getSingleJointTrajectoryMsg(names, pos_vec, 0.1));
}
void MybotActionNode::grasp_by_left(){
    std::vector<double> angles(3,0);
    angles[0] = -5.0/180.0*3.14;
    angles[1] = -5.0/180.0*3.14;
    angles[2] = +5.0/180.0*3.14;
    m_pub_gripper_left.publish(getJointTrajectoryMsgForGripperLeft(angles));
}

void MybotActionNode::release_by_left(){
    std::vector<double> angles(3,0);
    angles[0] = +15.0/180.0*3.14;
    angles[1] = +15.0/180.0*3.14;
    angles[2] = -15.0/180.0*3.14;
    m_pub_gripper_left.publish(getJointTrajectoryMsgForGripperLeft(angles));
}

void MybotActionNode::grasp_by_right(){
    std::vector<double> angles(3,0);
    angles[0] = +5.0/180.0*3.14;
    angles[1] = +5.0/180.0*3.14;
    angles[2] = -5.0/180.0*3.14;
    m_pub_gripper_right.publish(getJointTrajectoryMsgForGripperRight(angles));
}

void MybotActionNode::release_by_right(){
    std::vector<double> angles(3,0);
    angles[0] = -15.0/180.0*3.14;
    angles[1] = -15.0/180.0*3.14;
    angles[2] = +15.0/180.0*3.14;
    m_pub_gripper_right.publish(getJointTrajectoryMsgForGripperRight(angles));
}


std::vector<double> MybotActionNode::getCurrentGripperRightAngles() {
    return getCurrentAngles("gripper_right");
}
std::vector<double> MybotActionNode::getCurrentGripperLeftAngles() {
    return getCurrentAngles("gripper_left");
}
std::vector<double> MybotActionNode::getCurrentArmRightAngles() {
    return getCurrentAngles("arm_right");
}
std::vector<double> MybotActionNode::getCurrentArmLeftAngles() {
    return getCurrentAngles("arm_left");
}
std::vector<double> MybotActionNode::getCurrentTorsoAngles() {
    return getCurrentAngles("trunk");
}

std::vector<double> MybotActionNode::getCurrentAngles(const std::string& joint_group_name) {
    std::vector<double> angles;
    robot_state::RobotState robot_state = get_current_robot_state();
    const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup(joint_group_name);
    robot_state.copyJointGroupPositions(joint_model_group, angles);
    return angles;
}

bool MybotActionNode::is_something_wrong(moveit_msgs::MoveItErrorCodes& error){
    switch(error.val){
        case 1:
            {
                std::cout << "SUCCESS!!" << std::endl;
                return false;
            }
            break;
        case 99999:
            {
                std::cout << "FAILURE!!" << std::endl;
                return true;
            }
            break;
        case -1:
            {
                std::cout << "PLANNING FAILED!!!!!!!!!!!!!!" << std::endl;
                return true;
            }
            break;
        case -2:
            {
                std::cout << "INVALID MOTION PLAN!!!!!!!!!!" << std::endl;
                return true;
            }
            break;
        case -3:
            {
                std::cout << "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE!!!!!!!!!" << std::endl;
                return true;
            }
            break;

        case -4:
            {
                std::cout << "CONTROL_FAILED PLAN!!!!!!!!!!" << std::endl;
                return true;
            }
            break;
        default:
            {
                std::cout << "FAIL !!!!!!!!!!" << std::endl;
                return true;
            }
            break;


    }
}

bool MybotActionNode::move_to_default(const std::string& move_arm_group_name, const std::string& position_name){
    moveit_msgs::RobotTrajectory generated_traj;
    const robot_state::RobotState robot_state = get_current_robot_state();
    if(!get_trajectory_to_default(move_arm_group_name, position_name, generated_traj, robot_state)){
        std::cout << "Fail to generate motion" << std::endl;
        return false;
    }
    return execute_trajectory(generated_traj);
}

bool MybotActionNode::get_trajectory_to_default(const std::string& move_arm_group_name, const std::string& position_name, moveit_msgs::RobotTrajectory& generated_traj, const robot_state::RobotState& start_state){

    robot_state::RobotState robot_state = get_current_robot_state();
    const robot_model::JointModelGroup* joint_model_group = robot_state.getJointModelGroup(move_arm_group_name);

    robot_state::RobotState goal_state(m_robot_model);
    goal_state.setToDefaultValues(joint_model_group, position_name);
    moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    moveit_msgs::Constraints path_constraints;

    std::vector<moveit_msgs::Constraints> goal_constraints;
    goal_constraints.push_back(goal_constraint);
    
    return generate_motion_plan(move_arm_group_name, goal_constraints, path_constraints, generated_traj, start_state);
}

bool MybotActionNode::move_point_arm(const std::string& move_arm_group_name, const std::string& eef_link_name, const Eigen::Vector3d& position, const moveit_msgs::Constraints& path_constraints){
    
    moveit_msgs::RobotTrajectory generated_traj;
    generate_point_arm(move_arm_group_name, eef_link_name, position, path_constraints, generated_traj);
    return execute_trajectory(generated_traj);
}

bool MybotActionNode::generate_point_arm(const std::string& move_arm_group_name, const std::string& eef_link_name, const Eigen::Vector3d& position, const moveit_msgs::Constraints& path_constraints, moveit_msgs::RobotTrajectory& generated_traj){
    bool is_success = false;
    geometry_msgs::Point eef_link_point;
    eef_link_point.x = position(0);
    eef_link_point.y = position(1);
    eef_link_point.z = position(2);

    double t_pos = 0.01;
    //double t_angle = 0.1;
    moveit_msgs::Constraints goal_constraint = getPointConstraint(eef_link_name, eef_link_point, t_pos);

    std::vector<moveit_msgs::Constraints> goal_constraints;
    goal_constraints.push_back(goal_constraint);
    robot_state::RobotState start_state = get_current_robot_state();
    if(!generate_motion_plan(move_arm_group_name, goal_constraints, path_constraints, generated_traj, start_state)){
        std::cout << "Fail to generate motion" << std::endl;
        return false;
    }

    return true;
}

bool MybotActionNode::move_dual_pose_arm(const std::string& move_arm_group_name, const std::string& eef_link_name_1, const Eigen::Vector3d& position_1, const Eigen::Quaterniond& rotation_1, const std::string& eef_link_name_2, const Eigen::Vector3d& position_2, const Eigen::Quaterniond& rotation_2, const moveit_msgs::Constraints& path_constraints)
{
    moveit_msgs::RobotTrajectory generated_traj;
    generate_dual_pose_arm(move_arm_group_name, eef_link_name_1, position_1, rotation_1, eef_link_name_2, position_2, rotation_2, path_constraints, generated_traj);
    return execute_trajectory(generated_traj);
}

bool MybotActionNode::generate_dual_pose_arm(const std::string& move_arm_group_name, const std::string& eef_link_name_1, const Eigen::Vector3d& position_1, const Eigen::Quaterniond& rotation_1, const std::string& eef_link_name_2, const Eigen::Vector3d& position_2, const Eigen::Quaterniond& rotation_2, const moveit_msgs::Constraints& path_constraints,  moveit_msgs::RobotTrajectory& generated_traj)
{
    bool is_success = false;
    geometry_msgs::Pose eef_link_pose_1;
    eef_link_pose_1.position.x = position_1(0);
    eef_link_pose_1.position.y = position_1(1);
    eef_link_pose_1.position.z = position_1(2);
    eef_link_pose_1.orientation.x = rotation_1.x();
    eef_link_pose_1.orientation.y = rotation_1.y();
    eef_link_pose_1.orientation.z = rotation_1.z();
    eef_link_pose_1.orientation.w = rotation_1.w();
    geometry_msgs::Pose eef_link_pose_2;
    eef_link_pose_2.position.x = position_2(0);
    eef_link_pose_2.position.y = position_2(1);
    eef_link_pose_2.position.z = position_2(2);
    eef_link_pose_2.orientation.x = rotation_2.x();
    eef_link_pose_2.orientation.y = rotation_2.y();
    eef_link_pose_2.orientation.z = rotation_2.z();
    eef_link_pose_2.orientation.w = rotation_2.w();
    double t_pos = 0.01;//0.01
    double t_angle = 10.0*3.14/180.0;//0.1

    moveit_msgs::Constraints goal_constraint_1;
    moveit_msgs::Constraints goal_constraint_2;


    if(position_1 != Eigen::Vector3d::Zero()){
        goal_constraint_1 = getPoseConstraint(eef_link_name_1, eef_link_pose_1, t_pos, t_angle);
    }
    else{
        goal_constraint_1 = getOrientationConstraint(eef_link_name_1, rotation_1, t_angle);
    }
    if(position_2 != Eigen::Vector3d::Zero()){
        goal_constraint_2 = getPoseConstraint(eef_link_name_2, eef_link_pose_2, t_pos, t_angle);
    }
    else{
        goal_constraint_2 = getOrientationConstraint(eef_link_name_2, rotation_2, t_angle);
    }
    moveit_msgs::Constraints goal_constraint_dual_arm = kinematic_constraints::mergeConstraints(goal_constraint_1, goal_constraint_2);

    std::vector<moveit_msgs::Constraints> goal_constraints;
    goal_constraints.push_back(goal_constraint_dual_arm);

    robot_state::RobotState start_state = get_current_robot_state();
    if(!generate_motion_plan(move_arm_group_name, goal_constraints, path_constraints, generated_traj, start_state)){
        std::cout << "Fail to generate motion" << std::endl;
        return false;
    }

    return true;
}

bool MybotActionNode::move_arm(const std::string& move_arm_group_name, const std::string& eef_link_name, const Eigen::Vector3d& position, const std::vector<Eigen::Quaterniond>& rotations, const moveit_msgs::Constraints& path_constraints){
    bool is_success = false;

    std::vector<moveit_msgs::Constraints> goal_constraints;

    for(auto it = rotations.begin(); it != rotations.end(); ++it){

        geometry_msgs::Pose eef_link_pose;
        eef_link_pose.position.x = position(0);
        eef_link_pose.position.y = position(1);
        eef_link_pose.position.z = position(2);
        eef_link_pose.orientation.x = it->x();
        eef_link_pose.orientation.y = it->y();
        eef_link_pose.orientation.z = it->z();
        eef_link_pose.orientation.w = it->w();
        double t_pos = 0.005;//0.01
        double t_angle = 1.0*3.14/180.0;//0.1

        moveit_msgs::Constraints goal_constraint;
        if(position != Eigen::Vector3d::Zero()){
            goal_constraint = getPoseConstraint(eef_link_name, eef_link_pose, t_pos, t_angle);
        }
        else{
            goal_constraint = getOrientationConstraint(eef_link_name, *it, t_angle*30);
        }

        goal_constraints.push_back(goal_constraint);
    }

    moveit_msgs::RobotTrajectory generated_traj;
    robot_state::RobotState start_state = get_current_robot_state();
    if(!generate_motion_plan(move_arm_group_name, goal_constraints, path_constraints, generated_traj, start_state)){
        std::cout << "Fail to generate motion" << std::endl;
        return false;
    }

    return execute_trajectory(generated_traj);
}

bool MybotActionNode::generate_motion_plan(const std::string& move_arm_group_name, const std::vector<moveit_msgs::Constraints>& goal_constraints, const moveit_msgs::Constraints& path_constraint, moveit_msgs::RobotTrajectory& generated_traj, robot_state::RobotState start_state, const std::string& motion){
    moveit_msgs::RobotState start_state_msg;
    robot_state::robotStateToRobotStateMsg(start_state, start_state_msg);

    return generate_motion_plan(move_arm_group_name, goal_constraints, path_constraint, generated_traj, start_state_msg, motion);
}

bool MybotActionNode::generate_motion_plan(const std::string& move_arm_group_name, const std::vector<moveit_msgs::Constraints>& goal_constraints, const moveit_msgs::Constraints& path_constraint, moveit_msgs::RobotTrajectory& generated_traj, moveit_msgs::RobotState& start_state_msg, const std::string& motion){

    moveit_msgs::GetMotionPlanRequest req;
    moveit_msgs::GetMotionPlanResponse res;
    
    req.motion_plan_request.start_state = start_state_msg;
    req.motion_plan_request.goal_constraints.clear();
    req.motion_plan_request.goal_constraints.insert(req.motion_plan_request.goal_constraints.begin(), goal_constraints.begin(), goal_constraints.end());

    req.motion_plan_request.trajectory_constraints.constraints.push_back(path_constraint);
    req.motion_plan_request.path_constraints = path_constraint;
    //req.motion_plan_request.planner_id = "RRTstarkConfigDefault";
    //req.motion_plan_request.planner_id = "PRMkConfigDefault";
    //req.motion_plan_request.planner_id = "RRTConnectkConfigDefault";
    req.motion_plan_request.planner_id = motion;
    req.motion_plan_request.group_name = move_arm_group_name;
    req.motion_plan_request.num_planning_attempts = 3;
    req.motion_plan_request.allowed_planning_time = 100.0;
    //req.motion_plan_request.allowed_planning_time = 3.0 * (float)req.motion_plan_request.goal_constraints.size();
    req.motion_plan_request.max_velocity_scaling_factor = 1.0;

    req.motion_plan_request.workspace_parameters.header.frame_id = "base_footprint";
    req.motion_plan_request.workspace_parameters.min_corner.x = -0.0;
    req.motion_plan_request.workspace_parameters.min_corner.y = -1.0; 
    req.motion_plan_request.workspace_parameters.min_corner.z = -0.5; 
    req.motion_plan_request.workspace_parameters.max_corner.x = 1.0;
    req.motion_plan_request.workspace_parameters.max_corner.y = 1.0; 
    req.motion_plan_request.workspace_parameters.max_corner.z = 2.0; 

    m_service_motion_planning.call(req, res);

    if(is_something_wrong(res.motion_plan_response.error_code)){
        return false;
    }
    generated_traj = res.motion_plan_response.trajectory;
    std::cout << "Planning Time: " << res.motion_plan_response.planning_time << " seconds"  << std::endl;

    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = start_state_msg;
    display_trajectory.trajectory.push_back(res.motion_plan_response.trajectory);
    m_pub_display.publish(display_trajectory);

    //std::cout << generated_traj << std::endl;
    return true;
}

bool MybotActionNode::execute_trajectory(const moveit_msgs::RobotTrajectory& traj){
    moveit_msgs::ExecuteTrajectoryGoal execute_trajectory_goal;

    //execute_trajectory_goal.header.seq = m_counter++;
    //execute_trajectory_goal.header.stamp = ros::Time::now()+ros::Duration(2.0);
    //execute_trajectory_goal.header.frame_id = "base_footprint";
    //execute_trajectory_goal.goal_id.stamp = execute_trajectory_goal.header.stamp; 
    //execute_trajectory_goal.goal_id.id = "" ; 
    execute_trajectory_goal.trajectory = traj;
    m_action_execute->sendGoal(execute_trajectory_goal);
    bool finished_before_timeout = m_action_execute->waitForResult(ros::Duration(30.0));
    return true;
    /*
    moveit_msgs::ExecuteKnownTrajectoryRequest execute_trajectory_request;
    moveit_msgs::ExecuteKnownTrajectoryResponse execute_trajectory_response;
    execute_trajectory_request.wait_for_execution = true;
    execute_trajectory_request.trajectory = traj;
    execute_trajectory_request.trajectory.joint_trajectory.header.frame_id = "base_footprint";
    execute_trajectory_request.trajectory.joint_trajectory.header.stamp = ros::Time::now()+ros::Duration(10.0);
    execute_trajectory_request.trajectory.joint_trajectory.header.seq = m_counter++;

    m_action_execute.
    m_service_execute.call(execute_trajectory_request,execute_trajectory_response);

    if(is_something_wrong(execute_trajectory_response.error_code)){
        return false;
    }
    return true;

    */

}


robot_state::RobotState MybotActionNode::get_current_robot_state(){
    moveit_msgs::GetPlanningSceneResponse get_planning_scene_response;
    moveit_msgs::GetPlanningSceneRequest get_planning_scene_request;
    get_planning_scene_request.components.components 
        = moveit_msgs::PlanningSceneComponents::ROBOT_STATE 
        | moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS 
        | moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX 
        | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES 
        | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    m_service_get_planning_scene.call(get_planning_scene_request, get_planning_scene_response);
    planning_scene::PlanningScene planning_scene(m_robot_model);
    if(planning_scene::PlanningScene::isEmpty(get_planning_scene_response.scene)){
        std::cout << "[ERROR] EMPTY SCENE IS ARRIVED" << std::endl;
        return planning_scene.getCurrentStateNonConst();
    }
    planning_scene.setPlanningSceneMsg(get_planning_scene_response.scene);
    return planning_scene.getCurrentStateNonConst();
}



trajectory_msgs::JointTrajectory MybotActionNode::getSingleJointTrajectoryMsg(const std::vector<std::string>& names, const std::vector<double>& angles, double time_from_start){
    trajectory_msgs::JointTrajectory msg;
    if(names.size() != angles.size())   return msg;

    msg.header.stamp = ros::Time::now() + ros::Duration(0.2);
    msg.header.frame_id = "base_footprint";
    msg.header.seq = m_counter++;
    msg.points.resize(1);   // Only contain a single joint trajectory
    for(int i = 0; i < names.size(); ++i){
        msg.joint_names.push_back(names[i]);
        msg.points[0].positions.push_back(angles[i]);
    }
    msg.points[0].velocities.resize(names.size(),0);
    msg.points[0].accelerations.resize(names.size(),0);
    msg.points[0].effort.resize(names.size(),0);
    msg.points[0].time_from_start = ros::Duration(time_from_start);

    return msg;
}
trajectory_msgs::JointTrajectory MybotActionNode::getJointTrajectoryMsgForTrunk(const std::vector<double>& angles, double time_from_start){
    std::vector<std::string> names(2);
    names[0] = "waist_joint_0";
    names[1] = "waist_joint_1";
    return getSingleJointTrajectoryMsg(names, angles, time_from_start);
}

trajectory_msgs::JointTrajectory MybotActionNode::getJointTrajectoryMsgForArmLeft(const std::vector<double>& angles, double time_from_start){
    std::vector<std::string> names(8);
    names[0] = "left_shoulder_joint_0";
    names[1] = "left_shoulder_joint_1";
    names[2] = "left_shoulder_joint_2";
    names[3] = "left_shoulder_joint_3";
    names[4] = "left_elbow_joint_0";
    names[5] = "left_wrist_joint_0";
    names[6] = "left_wrist_joint_1";
    names[7] = "left_wrist_joint_2";
    return getSingleJointTrajectoryMsg(names, angles, time_from_start);
}
trajectory_msgs::JointTrajectory MybotActionNode::getJointTrajectoryMsgForArmRight(const std::vector<double>& angles, double time_from_start){
    std::vector<std::string> names(8);
    names[0] = "right_shoulder_joint_0";
    names[1] = "right_shoulder_joint_1";
    names[2] = "right_shoulder_joint_2";
    names[3] = "right_shoulder_joint_3";
    names[4] = "right_elbow_joint_0";
    names[5] = "right_wrist_joint_0";
    names[6] = "right_wrist_joint_1";
    names[7] = "right_wrist_joint_2";
    return getSingleJointTrajectoryMsg(names, angles, time_from_start);
}

trajectory_msgs::JointTrajectory MybotActionNode::getJointTrajectoryMsgForGripperRight(const std::vector<double>& angles, double time_from_start){
    std::vector<std::string> names(3);
    names[0] = "right_gripper_thumb_joint_0";
    names[1] = "right_gripper_forefinger_joint_0";
    names[2] = "right_gripper_littlefinger_joint_0";
    return getSingleJointTrajectoryMsg(names, angles, time_from_start);
}
trajectory_msgs::JointTrajectory MybotActionNode::getJointTrajectoryMsgForGripperLeft(const std::vector<double>& angles, double time_from_start){
    std::vector<std::string> names(3);
    names[0] = "left_gripper_thumb_joint_0";
    names[1] = "left_gripper_forefinger_joint_0";
    names[2] = "left_gripper_littlefinger_joint_0";
    return getSingleJointTrajectoryMsg(names, angles, time_from_start);
}


}
