#include "behavior.hpp"

#include <gaze_selection/PanTilt.h>

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



void get_rotation_candidates(const std::vector<geometry_msgs::Pose>& gripper_poses, std::vector<Eigen::Quaterniond>& rotations_for_grasp){
    for(std::size_t i = 0; i < gripper_poses.size(); ++i){
        Eigen::Quaterniond rotation_for_grasp;
        rotation_for_grasp.x() = gripper_poses[i].orientation.x;
        rotation_for_grasp.y() = gripper_poses[i].orientation.y;
        rotation_for_grasp.z() = gripper_poses[i].orientation.z;
        rotation_for_grasp.w() = gripper_poses[i].orientation.w;
        rotations_for_grasp.push_back(rotation_for_grasp);
    }
}



void approaching_action_callback(const decision_maker::ApproachToObjectAction::ConstPtr& msg, goodguy::MybotActionNode* mybot_action_node, ros::Publisher& pub_action_result){
    std::cout << "Approach to object for grasping" << std::endl;
    std::string arm_name = msg->arm_name;
    std::string object_name = msg->object_name;
    decision_maker::ActionResult result_state;
    Eigen::Vector3d object_point;
    object_point(0) = msg->gripper_pose[0].position.x;
    object_point(1) = msg->gripper_pose[0].position.y;
    object_point(2) = msg->gripper_pose[0].position.z;
    Eigen::Vector3d object_point_close;
    object_point_close = object_point;
    std::vector<Eigen::Quaterniond> rotations_for_grasp;
    get_rotation_candidates(msg->gripper_pose, rotations_for_grasp);

    if(arm_name == "ARM_LEFT"){
        std::cout << "Try Move Arm Left close to object: ";
        std::cout << object_point(0) << ", ";
        std::cout << object_point(1) << ", ";
        std::cout << object_point(2) << std::endl;
        moveit_msgs::Constraints path_constraints;
        //if(!mybot_action_node->move_only_arm_left(object_point, rotation_for_grasp, path_constraints)){
        if(!mybot_action_node->move_arm_left(object_point, rotations_for_grasp, path_constraints)){
            std::cout << "Fail to Move arm" << std::endl;
//            mybot_action_node->move_to_default_for_arm_left();
            result_state.result = "FAIL";
            pub_action_result.publish(result_state);
            return;
        }
        mybot_action_node->setAllowCollisionToObjectForLeft(object_name, true);
    }
    else{
        std::cout << "Try Move Arm Right close to object: ";
        std::cout << object_point(0) << ", ";
        std::cout << object_point(1) << ", ";
        std::cout << object_point(2) << std::endl;
        moveit_msgs::Constraints path_constraints;
        //if(!mybot_action_node->move_only_arm_right(object_point, rotation_for_grasp, path_constraints)){
        if(!mybot_action_node->move_arm_right(object_point, rotations_for_grasp, path_constraints)){
            std::cout << "Fail to Move arm" << std::endl;
//            mybot_action_node->move_to_default_for_arm_right();
            result_state.result = "FAIL";
            pub_action_result.publish(result_state);
            return;
        }     
        mybot_action_node->setAllowCollisionToObjectForRight(object_name, true);
    }
    result_state.result = "SUCCESS";
    pub_action_result.publish(result_state);

}


void grasping_action_callback(const decision_maker::GraspObjectAction::ConstPtr& msg, goodguy::MybotActionNode* mybot_action_node, ros::Publisher& pub_action_result){
    
    std::string arm_name = msg->arm_name;
    std::string object_name = msg->object_name;
    std::cout << "Approaching to " << object_name << "  for moving action" << std::endl;
    decision_maker::ActionResult result_state;
    Eigen::Vector3d object_point;
    object_point(0) = msg->gripper_pose[0].position.x;
    object_point(1) = msg->gripper_pose[0].position.y;
    object_point(2) = msg->gripper_pose[0].position.z;
    std::vector<Eigen::Quaterniond> rotations_for_grasp;
    get_rotation_candidates(msg->gripper_pose, rotations_for_grasp);

    if(arm_name == "ARM_LEFT"){
        mybot_action_node->setAllowCollisionToObjectForLeft(object_name, true);
        std::cout << "Try Move Arm Left close to object: ";
        std::cout << object_point(0) << ", ";
        std::cout << object_point(1) << ", ";
        std::cout << object_point(2) << std::endl;
        moveit_msgs::Constraints path_constraints; // = mybot_action_node->getOrientationConstraint("left_gripper_parm_0", rotation_for_grasp, 0.5);
        //path_constraints = mybot_action_node->getOrientationConstraint("left_gripper_parm_0", rotation_for_grasp, 60.0*3.14/180.0);
        //if(!mybot_action_node->move_only_arm_left(object_point, rotation_for_grasp, path_constraints)){
        if(!mybot_action_node->move_arm_left(object_point, rotations_for_grasp, path_constraints)){
            std::cout << "Fail to Move arm" << std::endl;
//            mybot_action_node->move_to_default_for_arm_left();
            result_state.result = "FAIL";
            pub_action_result.publish(result_state);
            return;
        }
    }
    else{
        mybot_action_node->setAllowCollisionToObjectForRight(object_name, true);
        std::cout << "Try Move Arm Right close to object: ";
        std::cout << object_point(0) << ", ";
        std::cout << object_point(1) << ", ";
        std::cout << object_point(2) << std::endl;
        moveit_msgs::Constraints path_constraints; // = mybot_action_node->getOrientationConstraint("right_gripper_parm_0", rotation_for_grasp, 0.5);
        //path_constraints = mybot_action_node->getOrientationConstraint("right_gripper_parm_0", rotation_for_grasp, 60.0*3.14/180.0);
        //if(!mybot_action_node->move_only_arm_right(object_point, rotation_for_grasp, path_constraints)){
        if(!mybot_action_node->move_arm_right(object_point, rotations_for_grasp, path_constraints)){
            std::cout << "Fail to Move arm" << std::endl;
//            mybot_action_node->move_to_default_for_arm_right();
            result_state.result = "FAIL";
            pub_action_result.publish(result_state);
            return;
        }
    }
    result_state.result = "SUCCESS";
    pub_action_result.publish(result_state);

    std::cout << "Grasp" << std::endl;
    if(arm_name == "ARM_LEFT"){
        mybot_action_node->grasp_by_left();
    }
    else{
        mybot_action_node->grasp_by_right();
    }
//string arm_name
//string object_name
//geometry_msgs/Pose gripper_pose

}


void tilting_action_callback(const decision_maker::TiltObjectAction::ConstPtr& msg, goodguy::MybotActionNode* mybot_action_node, ros::Publisher& pub_action_result){

    std::string arm_name = msg->arm_name;
    std::string object_name = msg->object_name;
    std::cout << "Tilt " << object_name << "  for tilting action" << std::endl;
    decision_maker::ActionResult result_state;
    std::vector<Eigen::Quaterniond> rotations_for_grasp;
    get_rotation_candidates(msg->gripper_pose, rotations_for_grasp);
    Eigen::Vector3d object_point = Eigen::Vector3d::Zero();
    //object_point(0) = gripper_pose.position.x;
    //object_point(1) = gripper_pose.position.y;
    //object_point(2) = gripper_pose.position.z;



    rotations_for_grasp.resize(1);

    Eigen::Quaterniond rotation_for_tilt;
    Eigen::AngleAxisd rotate1(0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotate2(60.0*3.14/180.0, Eigen::Vector3d::UnitY());
    rotation_for_tilt = rotate1*rotate2;
    rotations_for_grasp[0] = rotations_for_grasp[0]*rotation_for_tilt;
    /*
    
    double tilt_angle = 90.0*3.14/180.0;
    double tilt_back_angle = 0.0;
    if(arm_name == "ARM_LEFT"){
        tilt_back_angle = mybot_action_node->get_tilt_angle_arm_left();
    }
    else{
        tilt_back_angle = mybot_action_node->get_tilt_angle_arm_right();
    }

    if(arm_name == "ARM_LEFT"){
        mybot_action_node->tilt_arm_left(+tilt_angle+tilt_back_angle);
    }
    else{
        mybot_action_node->tilt_arm_right(-tilt_angle+tilt_back_angle);
    }

    ros::Duration sleep_time(5);
    sleep_time.sleep();


    if(arm_name == "ARM_LEFT"){
        mybot_action_node->tilt_arm_left(tilt_back_angle);
    }
    else{
        mybot_action_node->tilt_arm_right(tilt_back_angle);
    }
    sleep_time.sleep();
    */
    

    double tolerence_for_constraint = 0.02;    
    if(arm_name == "ARM_LEFT"){

        mybot_action_node->setAllowCollisionToObjectForLeft(object_name, true);
        std::cout << "Try Move Arm Left close to object: ";
        moveit_msgs::Constraints path_constraints = mybot_action_node->getPointConstraint("left_bottle_mouth_z", object_point, tolerence_for_constraint);
        //moveit_msgs::Constraints path_constraints;
        if(!mybot_action_node->move_arm_left(object_point, rotations_for_grasp, path_constraints)){
            std::cout << "Fail to Move arm" << std::endl;
            result_state.result = "FAIL";
            pub_action_result.publish(result_state);
            return;
        }
    }
    else{
        mybot_action_node->setAllowCollisionToObjectForRight(object_name, true);
        std::cout << "Try Move Arm Right close to object: ";
        moveit_msgs::Constraints path_constraints = mybot_action_node->getPointConstraint("right_bottle_mouth_z", object_point, tolerence_for_constraint);
        if(!mybot_action_node->move_arm_right(object_point, rotations_for_grasp, path_constraints)){
            std::cout << "Fail to Move arm" << std::endl;
            result_state.result = "FAIL";
            pub_action_result.publish(result_state);
            return;
        }
    }

    rotations_for_grasp[0].x() = msg->gripper_pose[0].orientation.x;
    rotations_for_grasp[0].y() = msg->gripper_pose[0].orientation.y;
    rotations_for_grasp[0].z() = msg->gripper_pose[0].orientation.z;
    rotations_for_grasp[0].w() = msg->gripper_pose[0].orientation.w;
    
    if(arm_name == "ARM_LEFT"){

        mybot_action_node->setAllowCollisionToObjectForLeft(object_name, true);
        std::cout << "Try Move Arm Left close to object: ";
        moveit_msgs::Constraints path_constraints = mybot_action_node->getPointConstraint("left_bottle_mouth_z", object_point, tolerence_for_constraint);
        //moveit_msgs::Constraints path_constraints;
        if(!mybot_action_node->move_arm_left(object_point, rotations_for_grasp, path_constraints)){
            std::cout << "Fail to Move arm" << std::endl;
            result_state.result = "FAIL";
            pub_action_result.publish(result_state);
            return;
        }
    }
    else{
        mybot_action_node->setAllowCollisionToObjectForRight(object_name, true);
        std::cout << "Try Move Arm Right close to object: ";
        moveit_msgs::Constraints path_constraints = mybot_action_node->getPointConstraint("right_bottle_mouth_z", object_point, tolerence_for_constraint);
        //moveit_msgs::Constraints path_constraints;
        if(!mybot_action_node->move_arm_right(object_point, rotations_for_grasp, path_constraints)){
            std::cout << "Fail to Move arm" << std::endl;
            result_state.result = "FAIL";
            pub_action_result.publish(result_state);
            return;
        }
    }
    


    result_state.result = "SUCCESS";
    pub_action_result.publish(result_state);
                                                                                           
}

void moving_action_callback(const decision_maker::MoveToObjectAction::ConstPtr& msg, goodguy::MybotActionNode* mybot_action_node, ros::Publisher& pub_action_result){

    std::string arm_name = msg->arm_name;
    std::string object_name = msg->object_name;
    geometry_msgs::Pose gripper_pose;
    std::cout << "Move to " << object_name << "  for moving action" << std::endl;
    decision_maker::ActionResult result_state;
    Eigen::Vector3d object_point;
    object_point(0) = msg->gripper_pose[0].position.x;
    object_point(1) = msg->gripper_pose[0].position.y;
    object_point(2) = msg->gripper_pose[0].position.z;
    std::vector<Eigen::Quaterniond> rotations_for_grasp;
    get_rotation_candidates(msg->gripper_pose, rotations_for_grasp);

    if(arm_name == "ARM_LEFT"){
        mybot_action_node->setAllowCollisionToObjectForLeft(object_name, true);
        std::cout << "Try Move Arm Left close to object: ";
        moveit_msgs::Constraints path_constraints;
        //moveit_msgs::Constraints path_constraints = mybot_action_node->getOrientationConstraint("left_gripper_parm_0", rotation_for_grasp, 60.0*3.14/180.0);
        //if(!mybot_action_node->move_only_arm_left(object_point, rotation_for_grasp, path_constraints)){
        if(!mybot_action_node->move_arm_left(object_point, rotations_for_grasp, path_constraints)){
            std::cout << "Fail to Move arm" << std::endl;
//            mybot_action_node->move_to_default_for_arm_left();
            result_state.result = "FAIL";
            pub_action_result.publish(result_state);
            return;
        }
    }
    else{
        mybot_action_node->setAllowCollisionToObjectForRight(object_name, true);
        std::cout << "Try Move Arm Right close to object: ";
        moveit_msgs::Constraints path_constraints;
        //moveit_msgs::Constraints path_constraints = mybot_action_node->getOrientationConstraint("right_gripper_parm_0", rotation_for_grasp, 60.0*3.14/180.0);
        //if(!mybot_action_node->move_only_arm_right(object_point, rotation_for_grasp, path_constraints)){
        if(!mybot_action_node->move_arm_right(object_point, rotations_for_grasp, path_constraints)){
            std::cout << "Fail to Move arm" << std::endl;
//            mybot_action_node->move_to_default_for_arm_right();
            result_state.result = "FAIL";
            pub_action_result.publish(result_state);
            return;
        }
    }
    result_state.result = "SUCCESS";
    pub_action_result.publish(result_state);
                                                                                           
}



void releasing_action_callback(const decision_maker::ReleaseAction::ConstPtr& msg, goodguy::MybotActionNode* mybot_action_node, ros::Publisher& pub_action_result){

    decision_maker::ActionResult result_state;
    std::string arm_name = msg->arm_name;
    std::string object_name = msg->object_name;
    std::cout << "release" << std::endl;
    if(arm_name == "ARM_LEFT"){
        mybot_action_node->release_by_left();
        mybot_action_node->setAllowCollisionToObjectForLeft(object_name, false);
    }
    else{
        mybot_action_node->release_by_right();
        mybot_action_node->setAllowCollisionToObjectForRight(object_name, false);
    }

    result_state.result = "SUCCESS";
    pub_action_result.publish(result_state);
    //string arm_name
    //string object_name

}

void initialize_action_callback(const decision_maker::InitializeAction::ConstPtr& msg, goodguy::MybotActionNode* mybot_action_node, ros::Publisher& pub_action_result){

    decision_maker::ActionResult result_state;
    std::string arm_name = msg->arm_name;
    std::cout << "initialize" << std::endl;
    if(arm_name == "ARM_LEFT"){
        if(!mybot_action_node->move_to_default_for_arm_left()){
            std::cout << "Fail to Move arm" << std::endl;
            result_state.result = "FAIL";
            pub_action_result.publish(result_state);
            return;
        }
    }
    else{
        if(!mybot_action_node->move_to_default_for_arm_right()){
            std::cout << "Fail to Move arm" << std::endl;
            result_state.result = "FAIL";
            pub_action_result.publish(result_state);
            return;
        }
    }

    result_state.result = "SUCCESS";
    pub_action_result.publish(result_state);
    //string arm_name
    //string object_name

}


void test_tilt(goodguy::MybotActionNode* mybot_action_node){

    static bool is_tilted = false;

    std::vector<Eigen::Quaterniond> rotations_for_tilt(1);
    Eigen::AngleAxisd rotate1(-1.57, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rotate2(+1.57, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotate3(90.0*3.14/180.0, Eigen::Vector3d::UnitY());
    if(is_tilted)   rotate3 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
    rotations_for_tilt[0] = rotate1*rotate2*rotate3;


    double x, y, z;
    std::cout << "Enter x y z: ";
    std::cin >> x >> y >> z;

    Eigen::Vector3d object_point;
    object_point(0) = x;
    object_point(1) = y;
    object_point(2) = z;

    std::cout << "Try Move Arm Right close to object: ";
    moveit_msgs::Constraints path_constraints;
    path_constraints = mybot_action_node->getPointConstraint("right_bottle_mouth_z", object_point, 0.01);
    if(!mybot_action_node->move_only_arm_right(Eigen::Vector3d::Zero(), rotations_for_tilt, path_constraints)){
        std::cout << "Fail to Move arm" << std::endl;
        return;
    }

    is_tilted = !is_tilted;
                                                                                           
}

void test_bottle_mouth(goodguy::MybotActionNode* mybot_action_node){
    double x, y, z;
    std::cout << "Enter x y z: ";
    std::cin >> x >> y >> z;

    Eigen::Vector3d mouth_point;
    mouth_point(0) = x;
    mouth_point(1) = y;
    mouth_point(2) = z;

    mybot_action_node->set_bottle_mouth_right(mouth_point);

}

void test_move(goodguy::MybotActionNode* mybot_action_node, bool is_pose_consist = false){

    std::vector<Eigen::Quaterniond> rotations_for_grasp;
    Eigen::AngleAxisd rotate1(-1.57, Eigen::Vector3d::UnitY());
    //Eigen::AngleAxisd rotate2(-1.57, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotate2(+0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotate3(-45.0*3.14/180.0, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rotate4(0*3.14/180.0, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rotate5(45*3.14/180.0, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rotate6(90*3.14/180.0, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rotate7(-90*3.14/180.0, Eigen::Vector3d::UnitZ());
    rotations_for_grasp.push_back(rotate1*rotate2*rotate3);
    rotations_for_grasp.push_back(rotate1*rotate2*rotate4);
    rotations_for_grasp.push_back(rotate1*rotate2*rotate5);
    rotations_for_grasp.push_back(rotate1*rotate2*rotate6);
    rotations_for_grasp.push_back(rotate1*rotate2*rotate7);

    double x, y, z;
    std::cout << "Enter x y z: ";
    std::cin >> x >> y >> z;

    Eigen::Vector3d object_point;
    object_point(0) = x;
    object_point(1) = y;
    object_point(2) = z;

    std::cout << "Try Move Arm Right close to object: ";
    moveit_msgs::Constraints path_constraints;
    if(is_pose_consist) path_constraints = mybot_action_node->getOrientationConstraint("right_bottle_mouth_z", rotations_for_grasp[0], 0.1);
    //if(is_pose_consist) path_constraints = mybot_action_node->getOrientationConstraint("right_gripper_parm_0", rotation_for_grasp, 0.1);
    //if(!mybot_action_node->move_only_arm_left(object_point, rotations_for_grasp, path_constraints)){
    if(!mybot_action_node->move_arm_right(object_point, rotations_for_grasp, path_constraints)){
    //if(!mybot_action_node->move_only_arm_right(object_point, rotations_for_grasp, path_constraints)){
        std::cout << "Fail to Move arm" << std::endl;
        return;
    }
                                                                                           
}




void test_dual_arm(goodguy::MybotActionNode* mybot_action_node, bool is_pose_consist = false){

    Eigen::Quaterniond rotation_for_grasp_r;
    Eigen::AngleAxisd rotate1(-1.57, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rotate2(+1.57, Eigen::Vector3d::UnitX());
    rotation_for_grasp_r = rotate1*rotate2;

    Eigen::Quaterniond rotation_for_grasp_l;
    Eigen::AngleAxisd rotate3(-1.57, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rotate4(-1.57, Eigen::Vector3d::UnitX());
    rotation_for_grasp_l = rotate3*rotate4;

    if(is_pose_consist){
        Eigen::AngleAxisd rotate5(+0.5, Eigen::Vector3d::UnitY());
        rotation_for_grasp_r = rotate1*rotate2*rotate5;
    }
    else{
        rotation_for_grasp_r = rotate1*rotate2;
    }


    double x, y, z;
    std::cout << "Enter x y z: ";
    std::cin >> x >> y >> z;

    Eigen::Vector3d gripper_point_r;
    gripper_point_r(0) = x;
    gripper_point_r(1) = y;
    gripper_point_r(2) = z;

    Eigen::Vector3d gripper_point_l;
    gripper_point_l(0) = x;
    gripper_point_l(1) = y+0.1;
    gripper_point_l(2) = z-0.1;

    std::cout << "Try Move DUAL ARM : ";
    moveit_msgs::Constraints path_constraints;
    if(is_pose_consist){
         moveit_msgs::Constraints path_constraint_r = mybot_action_node->getPointConstraint("right_bottle_mouth", gripper_point_r, 0.2);
         moveit_msgs::Constraints path_constraint_l = mybot_action_node->getPoseConstraint("left_bottle_mouth", gripper_point_l, rotation_for_grasp_l, 0.2, 0.1);
         path_constraints = kinematic_constraints::mergeConstraints(path_constraint_r, path_constraint_l);
         gripper_point_r = Eigen::Vector3d::Zero();
    }
    if(!mybot_action_node->move_dual_arm(gripper_point_r, rotation_for_grasp_r, gripper_point_l, rotation_for_grasp_l, path_constraints)){
        std::cout << "Fail to Move Dual ARM" << std::endl;
        return;
    }
                                                                                           
}



int main(int argc, char** argv){
    ros::init(argc, argv, "behavior");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle node_handle("~");


    goodguy::MybotActionNode* mybot_action_node = new goodguy::MybotActionNode();


    //mybot_action_node->move_to_default_for_dual_arm();

    //mybot_action_node->release_by_right();
    //mybot_action_node->release_by_left();


    

    std::string head;
    node_handle.getParam("head_", head);

    ros::Publisher pub_pan_tilt = node_handle.advertise<gaze_selection::PanTilt>(head + std::string("/pan_tilt_angle"), 1, true);







    ros::Publisher pub_action_result = node_handle.advertise<decision_maker::ActionResult>("action_result", 1, true);

    ros::Subscriber sub_approach_action = node_handle.subscribe<decision_maker::ApproachToObjectAction>("/decision_maker/approach_to_object_action", 1, boost::bind(approaching_action_callback, _1, mybot_action_node, pub_action_result));
    ros::Subscriber sub_grasp_action = node_handle.subscribe<decision_maker::GraspObjectAction>("/decision_maker/grasp_object_action", 1, boost::bind(grasping_action_callback, _1, mybot_action_node, pub_action_result));
    ros::Subscriber sub_move_action = node_handle.subscribe<decision_maker::MoveToObjectAction>("/decision_maker/move_to_object_action", 1, boost::bind(moving_action_callback, _1, mybot_action_node, pub_action_result));
    ros::Subscriber sub_tilt_action = node_handle.subscribe<decision_maker::TiltObjectAction>("/decision_maker/tilt_object_action", 1, boost::bind(tilting_action_callback, _1, mybot_action_node, pub_action_result));
    ros::Subscriber sub_release_action = node_handle.subscribe<decision_maker::ReleaseAction>("/decision_maker/release_action", 1, boost::bind(releasing_action_callback, _1, mybot_action_node, pub_action_result));
    ros::Subscriber sub_initialize_action = node_handle.subscribe<decision_maker::InitializeAction>("/decision_maker/initialize_action", 1, boost::bind(initialize_action_callback, _1, mybot_action_node, pub_action_result));

    bool is_quit = false;

    gaze_selection::PanTilt pan_tilt_msg;
    pan_tilt_msg.pan_angle.data = 0;
    pan_tilt_msg.tilt_angle.data = 0;

    while(!is_quit){

        std::cout << "=============  COMMAND  =============" << std::endl;
        std::cout << " q : quit" << std::endl;
        std::cout << " e : test dual arm" << std::endl;
        std::cout << " t : test right arm" << std::endl;
        std::cout << " s : set bottle mouth" << std::endl;
        std::cout << " a : test tilt action" << std::endl;
        std::cout << " g : test grasp" << std::endl;
        std::cout << " i : initialize each arms" << std::endl;
        std::cout << " I : initialize both arms" << std::endl;
        std::cout << " r : see right" << std::endl;
        std::cout << " l : see left" << std::endl;
        std::cout << " c : see center" << std::endl;
        std::cout << " b : see bottom" << std::endl;
        std::cout << " f : see front" << std::endl;
        std::cout << std::endl;
        std::cout << "Enter your Key: ";

        char key;
        try{
            key = getInput<char>();
        } catch( std::exception& e){
            key = 0x00;
            std::cout << "Wrong key input" << std::endl;
        }

        switch(key){
            case 'q': is_quit = true; break;
            case 'f': 
            {
                pan_tilt_msg.pan_angle.data = 0.0*3.14/180.0;
                pan_tilt_msg.tilt_angle.data = 0.0*3.14/180.0;
                pub_pan_tilt.publish(pan_tilt_msg);
                break;
            }
            case 'b': 
            {
                pan_tilt_msg.tilt_angle.data = +40.0*3.14/180.0;
                pub_pan_tilt.publish(pan_tilt_msg);
                break;
            }
            case 'c': 
            {
                pan_tilt_msg.pan_angle.data = -0.0*3.14/180.0;
                pub_pan_tilt.publish(pan_tilt_msg);
                break;
            }
            case 'l': 
            {
                pan_tilt_msg.pan_angle.data = -60.0*3.14/180.0;
                pub_pan_tilt.publish(pan_tilt_msg);
                break;
            }
            case 'r': 
            {
                pan_tilt_msg.pan_angle.data = 60.0*3.14/180.0;
                pub_pan_tilt.publish(pan_tilt_msg);
                break;
            }
            case 'e': 
            {
                std::cout << "Test Dual ARM " << std::endl;
                test_dual_arm(mybot_action_node, false);
                break;
            }
            case 'p':{
                mybot_action_node->move_to_default("trunk_with_arm_right", "ibj_exp_pose_1");
                break;
            }
            case 't': 
            {
                std::cout << "Test Move " << std::endl;
                test_move(mybot_action_node, false);
                break;
            }
            case 's': 
            {
                std::cout << "Set Bottle Mouth " << std::endl;
                test_bottle_mouth(mybot_action_node);
                break;
            }
            case 'a': 
            {
                std::cout << "Test Tilt " << std::endl;
                test_tilt(mybot_action_node);
                break;
            }
            case 'g': 
            {
                static bool is_grasped_right = true;
                static bool is_grasped_left = true;
                char key2;
                try{
                    std::cout << "CHOOSE Right/Left [r/n] ? : ";
                    key2 = getInput<char>();
                    if(key2 == 'r'){
                        if(is_grasped_right)  
                            mybot_action_node->grasp_by_right();
                        else  
                            mybot_action_node->release_by_right();

                        is_grasped_right = !is_grasped_right;

                    }
                    else if(key2 == 'l'){
                        if(is_grasped_left)  
                            mybot_action_node->grasp_by_left();
                        else  
                            mybot_action_node->release_by_left();

                        is_grasped_left = !is_grasped_left;


                    }
                } catch( std::exception& e){
                    key2 = 0x00;
                    std::cout << "Wrong key input" << std::endl;
                }
            }
            break;
            case 'i': 
            {
                std::cout << "Initialization for Mybot" << std::endl;
                mybot_action_node->move_to_default_for_arm_right();
                mybot_action_node->move_to_default_for_arm_left();
                //mybot_action_node->release_by_right();
                //mybot_action_node->release_by_left();
                break;
            }
            case 'I': 
            {
                std::cout << "Initialization for Mybot with Dual ARM" << std::endl;
                mybot_action_node->move_to_default_for_dual_arm();
                mybot_action_node->release_by_right();
                mybot_action_node->release_by_left();
                break;
            }

            
            
        }

    }


    ros::waitForShutdown();

    delete mybot_action_node;

    return 0;
}
