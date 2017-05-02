// STL Header
#include <iostream>
#include <numeric>
#include <limits>
#include <memory>
#include "cbf.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <compressed_depth_image_transport/compressed_depth_subscriber.h>
#include <compressed_image_transport/compressed_subscriber.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


#include <moveit_msgs/PlanningSceneWorld.h>
#include <geometry_msgs/PointStamped.h>

#include <sensor_msgs/PointCloud2.h>

// Eigen Header
#include <eigen3/Eigen/Dense>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// plaincode header
#include "Core.hpp"
#include "Color.hpp"
#include "Object.hpp"
#include "patch.hpp"
#include "contact.hpp"

// Boost Header
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <simple_recognition/RecogObject.h>

lua_State *lua;
plaincode::Object patched_things;
plaincode::Object colored_things;
plaincode::Object other_things; 
plaincode::Color color;
std::vector<std::string> nameTags;
std::vector<cv::Scalar> allLut; 
std::vector<plaincode::Blob> objects; 
std::vector<double> dist2hand; 
bool isHand=false; 
bool isSelected=false; 
int selectedID=0; 

std::string gObjName; 
std::size_t gObjectDetectCount = 0;
cv::Mat labelMat, prLabelMat; 

cv::Mat prevLabelMat,prevPrLabelMat; 
std::vector<plaincode::Blob> prevObjects; 
cv::Mat prevDepth; 
bool isRotated=false;

//std::vector<cv::Scalar> lut;
cv::Mat originRGB, originDepth;
cv::Mat rawRGB, rawDepth;

Eigen::VectorXf parameter = Eigen::VectorXf(3);
tf::TransformListener* g_tf_listener;


typedef enum SELECT_STATE{
    SELECT_STATE_IDLE,
    SELECT_STATE_APPROACHING,
    SELECT_STATE_APPROACHED,
    SELECT_STATE_FARAWAYING,
    SELECT_STATE_FARAWAYED
} SelectState;

SelectState gSelectState = SELECT_STATE_IDLE;


cv::Point object_center_point;
bool is_object_center_point_updated = false;


void cropImage(cv::Mat& image, cv::Mat& result) {
        cv::Rect rect = cv::Rect(20, 20, 280, 200);
        result = image(rect);
}

void bilateralFilter(cv::Mat& imgDepth, cv::Mat& resultDepth, int filterSize, double varS, double varR) {
        cv::Mat tempDepth;
        imgDepth.convertTo(tempDepth, CV_32F);
        cv::bilateralFilter(tempDepth, resultDepth, filterSize, varS, varR);
        resultDepth.convertTo(resultDepth, CV_16U);
}


bool is_running = false;
int test(double fx, double fy, double cx, double cy, ros::Publisher& point_pub, ros::Publisher& pub_psw, bool saveMode=false){

    ros::WallDuration sleep_time(0.05);
    ros::Time current_time = ros::Time::now();

    Eigen::MatrixXf K, Kinv;
    plaincode::setCameraMat(fx, fy, cx-20.0, cy-20.0, K, Kinv);

    cv::Mat gray;
    cv::Mat RGB, Depth;
    cv::Mat onPlane, underPlane;
    cv::Mat distMat;
    cv::Mat Mask, Label;
    cv::Mat absoluteRGB;
    cv::Mat handMask;

    cv::Mat feature;
    Eigen::MatrixXf points;
    // Sort*320.0/640.0 toys.
    double time = cv::getTickCount();
    cropImage(originRGB, RGB);
    cropImage(originDepth, Depth);

    std::vector<cv::Mat> inputMat;
    std::vector<cv::Mat> patchMat; 

    std::vector<int> patchLabel; 
    std::vector<int> outputLabel;


    plaincode::crossBilateralFilter(RGB, Depth);
    plaincode::getPoint3d(Depth, points, Kinv);
    /*std::cout << "Point: " << points(200, 0) << "\t" << points(200,1) << "\t" << points(200,2) << std::endl;*/
    plaincode::planeParameter(Depth, points, parameter);

//    plaincode::planeParameter_pcl(Depth, points, parameter); 
    plaincode::getDistMat(Depth, distMat, points, parameter);

    color.RGB2Mask(RGB, handMask, 0);
    handMask.setTo(0, distMat>-30);

    plaincode::getCandidate(distMat, onPlane, handMask, points, parameter, K, -20.);
    plaincode::removePlane(distMat, underPlane, plaincode::CONSTANT, 1200., 80.);

    cv::Mat tuneMask;
    cv::medianBlur(handMask, handMask, 3);
    onPlane.copyTo(tuneMask, ~handMask);
    // segmentMask 중, 가장 큰 것을 찾자.
    plaincode::Blob handBlob;
    cv::Mat handMat = cv::Mat::zeros(RGB.size(), CV_8UC1); 

    handBlob.centerPos3d = Eigen::Vector3f(-100, -100, -100); 
    if (plaincode::findMaxBlob(handMask, handMat, handBlob, 80)) {
        plaincode::findCoM(handBlob, handMat, points); 
    }


    cv::moveWindow("hand", 100+280*3, 100); cv::imshow("hand", handMat); 

    objects.clear(); 
    dist2hand.clear(); 

    labelMat.setTo(0); 
    prLabelMat.setTo(0); 

    patched_things.clearBlob();
    cv::Mat tempRGB = RGB.clone(); 
    tempRGB.setTo(cv::Scalar(0,0,0), distMat>-10);  
    patched_things.project(points, parameter, K, 0); 
    std::cout << "PATCH" << std::endl; 
    patched_things.lua_classify(lua, patchMat, patchLabel, 0.99); 
    patched_things.reflect(patchMat, patchLabel); 
    std::cout << "PATCH END" << std::endl; 

    objects.insert(objects.end(), patched_things.blobs.begin(), patched_things.blobs.end()); 
    patched_things.labelMat.copyTo(labelMat, patched_things.labelMat); 
    patched_things.prLabelMat.copyTo(prLabelMat, patched_things.prLabelMat); 
    if (saveMode) patched_things.save(patchMat); 

    onPlane.setTo(0, patched_things.labelMat); 
    ////////////////////////
    // color
    cv::Mat colored; 
    colored_things.clearBlob(); 
    for (int i=0; i<4; i++) {
        color.RGB2Mask(RGB, colored, i+1); 
        cv::medianBlur(colored, colored, 5); 
        colored_things.append(colored, 50); 
    }

    colored_things.project(points, parameter, K); 
    colored_things.filter(underPlane); 
    colored_things.accumulate(originRGB, inputMat); 
    std::cout << "COLOR" << std::endl; 
    colored_things.lua_classify(lua, inputMat, outputLabel, 0.82);
    std::cout << "COLOR END" << std::endl; 
    colored_things.reflect(inputMat, outputLabel); 
    if (saveMode) colored_things.save(inputMat); 

    objects.insert(objects.end(), colored_things.blobs.begin(), colored_things.blobs.end()); 
    colored_things.labelMat.copyTo(labelMat, colored_things.labelMat); 
    colored_things.prLabelMat.copyTo(prLabelMat, colored_things.prLabelMat); 

    onPlane.setTo(0, colored_things.labelMat|handMask); 
    other_things.clearBlob(); 
    other_things.append(onPlane); 
    other_things.project(points, parameter, K); 
    other_things.filter(underPlane); 
    other_things.accumulate(originRGB, inputMat); 
    std::cout << "OBJECT" << std::endl; 
    other_things.lua_classify(lua, inputMat, outputLabel, 0.8); 
    std::cout << "OBJECT END" << std::endl; 
    other_things.reflect(inputMat, outputLabel); 

    objects.insert(objects.end(), other_things.blobs.begin(), other_things.blobs.end()); 
    other_things.labelMat.copyTo(labelMat, other_things.labelMat); 
    other_things.prLabelMat.copyTo(prLabelMat, other_things.prLabelMat); 
    if (saveMode) other_things.save(inputMat); 
    prevLabelMat = labelMat.clone(); 
    prevPrLabelMat = prLabelMat.clone(); 
    prevObjects.clear(); 
    prevObjects.assign(objects.begin(), objects.end()); 
    prevDepth = Depth.clone(); 

    cv::Mat proj_for_objects;
    originRGB.copyTo(proj_for_objects);


    // Find the selected object
    if (objects.size()>0) {

        std::vector<Eigen::Vector3f> hand_points;
        bool is_hand_exist = false;
        for(int i=0; i<handMat.rows; i++) {
            for (int j=0; j<handMat.cols; j++) {
                if(handMat.at<unsigned char>(i,j)){
                    std::size_t loc_pixel = i*handMat.cols + j;
                    Eigen::Vector3f point = points.row(loc_pixel);
                    if(point(2) > 0.1){
                         hand_points.push_back(point);
                         is_hand_exist = true;
                    }
                }
            }
        }

        if(is_hand_exist){

            std::vector<geometry_msgs::PointStamped> hand_ros_points(hand_points.size());
            ros::Time current_time = ros::Time::now();
            for(std::size_t i = 0; i < hand_points.size(); ++i){
                hand_ros_points[i].point.x = hand_points[i](0);
                hand_ros_points[i].point.y = hand_points[i](1);
                hand_ros_points[i].point.z = hand_points[i](2);
                hand_ros_points[i].header.stamp = current_time;
                hand_ros_points[i].header.frame_id = "/xtion_rgb_optical_frame";
            }

            std::vector<geometry_msgs::PointStamped> hand_ros_transformed_points(hand_points.size());
            bool is_transform_failed = false;
            try{
                for(std::size_t i = 0; i < hand_points.size(); ++i){
                    g_tf_listener->transformPoint("/base_0", ros::Time(0), hand_ros_points[i], hand_ros_points[i].header.frame_id, hand_ros_transformed_points[i]);
                }
            } catch(tf::TransformException& ex){
                is_transform_failed = true;
            }

            if(!is_transform_failed){

                std::size_t closest_point_index = 0;
                float closest_distance = std::numeric_limits<float>::max();
                for(std::size_t i = 0; i < (hand_ros_transformed_points.size()-1); ++i){
                    for(std::size_t j = i+1; j < hand_ros_transformed_points.size(); ++j){
                        if(hand_ros_transformed_points[i].point.x < closest_distance){
                            closest_point_index = i;
                            closest_distance = hand_ros_transformed_points[i].point.x;
                        }
                    }
                }


                Eigen::Vector3f hand_end_point = hand_points[closest_point_index];

                for (std::size_t i=0; i<objects.size(); i++){
                    dist2hand.push_back((objects[i].prCenterPos3d-hand_end_point).norm()); 
                    //dist2hand.push_back((objects[i].prCenterPos3d-handBlob.centerPos3d).norm()); 
                }


                int minDistIdx = std::distance(dist2hand.begin(), std::min_element(dist2hand.begin(), dist2hand.end())); 

                float distance_approaching = std::numeric_limits<float>::max();
                for(std::size_t i=0; i<objects.size(); i++){
                    if (objects[i].name==gObjName){
                        distance_approaching = dist2hand[i];
                        break;  
                    }
                }
                switch(gSelectState){
                    case SELECT_STATE_IDLE:
                        {
                            //std::cout << "IDLE" << std::endl;
                            if(dist2hand[minDistIdx]<150){
                                gSelectState = SELECT_STATE_APPROACHING;
                                gObjName = objects[minDistIdx].name;
                                gObjectDetectCount = 0;
                            }
                            else{
                                gSelectState = SELECT_STATE_IDLE;
                            }
                        }
                        break;
                    case SELECT_STATE_APPROACHING:
                        {
                            //std::cout << "SELECT_STATE_APPROACHING:" << std::endl;
                            if(objects[minDistIdx].name == gObjName && distance_approaching <150){
                                gObjectDetectCount++;
                                if(gObjectDetectCount > 2){
                                    gSelectState = SELECT_STATE_APPROACHED;
                                }
                                else{
                                    gSelectState = SELECT_STATE_APPROACHING;
                                }
                            }
                            else{
                                gSelectState = SELECT_STATE_IDLE;
                            }
                        }
                        break;
                    case SELECT_STATE_APPROACHED:
                        {

                            //std::cout << "SELECT_STATE_APPROACHED::" << std::endl;
                            if(distance_approaching>150){
                                gObjectDetectCount = 0;
                                gSelectState = SELECT_STATE_FARAWAYING;
                            }
                            else if(objects[minDistIdx].name != gObjName){
                                gSelectState = SELECT_STATE_IDLE;
                            }
                            else{
                                gSelectState = SELECT_STATE_APPROACHED;
                            }
                        }
                        break;
                    case SELECT_STATE_FARAWAYING:
                        {
                            isSelected=true;
                            std::cout << "far away from " << gObjName << std::endl; 
                            for(std::size_t i=0; i<objects.size(); i++){
                                if (objects[i].name==gObjName){
                                    selectedID=i; 
                                }
                            }
                            gSelectState = SELECT_STATE_IDLE;
                        }
                        break;
                    case SELECT_STATE_FARAWAYED:
                        {
                            isSelected=true;
                            std::cout << "far away from " << gObjName << std::endl; 
                            for(std::size_t i=0; i<objects.size(); i++){
                                if (objects[i].name==gObjName){
                                    selectedID=i; 
                                }
                            }
                            gSelectState = SELECT_STATE_IDLE;
                        }
                        break;

                }

                /*
                if(isHand==false && dist2hand[minDistIdx]<150){
                    isHand=true; 
                    gObjName = objects[minDistIdx].name; 
                    std::cout << "approach to " << gObjName << std::endl; 
                }
                else if(isHand==true){
                    if(cv::countNonZero(handMat)<=10){
                        isHand=false;
                    }
                    else{
                        for(std::size_t i=0; i<objects.size(); i++){
                            if (objects[i].name==gObjName && (objects[i].prCenterPos3d-hand_end_point).norm() > 150){
                                isSelected=true;
                                std::cout << "far away from " << gObjName << std::endl; 
                                selectedID=i; 
                                break;  
                            }
                        }
                    }

                }
                else{
                    std::cout << "FAIL" << gObjName << std::endl; 
                    isHand = false;
                }
                */
            }
            else{
                std::cout << "TRANSFORMED FAILED!" << std::endl;

            }
        }

    }

    for (std::size_t i=0; i<objects.size(); i++) {
        cv::Point object_center = (objects[i].prCenterPos+cv::Point(20,20))*2; 
        cv::circle(proj_for_objects, object_center, 10, cv::Scalar(255,0,0),3);

        double distance = (objects[i].prCenterPos3d(2))/1000.0;

        if (distance >0.2) {
            geometry_msgs::Point object_position;

            object_position.x = objects[i].prCenterPos3d(0)/1000.0;
            object_position.y = objects[i].prCenterPos3d(1)/1000.0;
            object_position.z = objects[i].prCenterPos3d(2)/1000.0;

            geometry_msgs::PointStamped stamped_object_position;
            std_msgs::Header header_for_object;
            header_for_object.frame_id = std::string("/xtion_rgb_optical_frame");
            header_for_object.stamp = current_time;

            stamped_object_position.point = object_position;
            stamped_object_position.header = header_for_object;

            simple_recognition::RecogObject recog_object;

            recog_object.stamped_point = stamped_object_position;
            recog_object.object_name = objects[i].name;
            if (isSelected && i==selectedID) {
                std::cout << objects[i].name << " is selected" << std::endl; 
                isSelected=false; 
                isHand=false; 
            }
            point_pub.publish(recog_object);
        }
    }

    cv::Mat result, prResult; 
    result = cv::Mat::zeros(RGB.size(), CV_8UC3); 
    prResult = cv::Mat::zeros(RGB.size(), CV_8UC3); 

    plaincode::paint(result, labelMat, objects, allLut); 
    plaincode::paint(prResult, prLabelMat, objects, allLut); 

    cv::addWeighted(RGB, 0.4, result, 0.6, 0.0, result);
    cv::addWeighted(RGB, 0.4, prResult, 0.6, 0.0, prResult);

    plaincode::putText(result, objects); 
    plaincode::putText(prResult, objects); 

    for (std::size_t i=0; i<objects.size(); i++) {
        int pc=0; 
        double ratio = objects[i].eigenVal[0]/objects[i].eigenVal[1]; 
        if (objects[i].eigenVal[0]<objects[i].eigenVal[1]) {
            pc=1;
            ratio = 1./ratio; 
        }

        if (ratio>=2.0) {
            cv::Point pts = objects[i].prCenterPos + cv::Point(objects[i].eigenVec[pc].x*objects[i].eigenVal[pc], objects[i].eigenVec[pc].y*objects[i].eigenVal[pc]); 
            cv::line(prResult, objects[i].prCenterPos, pts, cv::Scalar(255,255,255), 2); 
        }
        else {
            cv::circle(prResult, objects[i].prCenterPos, 5, cv::Scalar(255,255,255), 3); 
        }
    }

    cv::Mat onPlaneMat = cv::Mat::zeros(distMat.size(), distMat.type());
    distMat.copyTo(onPlaneMat, tuneMask);

    cv::moveWindow("RGB", 100, 100); cv::imshow("RGB", RGB);
    cv::moveWindow("output", 100+280, 100); cv::imshow("output", result); 
    cv::moveWindow("prOutput", 100+280+280, 100); cv::imshow("prOutput", prResult); 

    /*
       cv::cvtColor(RGB, gray, CV_BGR2GRAY); 
       for (std::size_t i=0; i<objects.size(); i++) {
       std::vector<cv::RotatedRect> rRects; 
       rRects = locationSearch(RGB, gray, cv::Point2d(objects[i].rect.x, objects[i].rect.y), 
       cv::Point2d(objects[i].rect.x + objects[i].rect.width, objects[i].rect.y + objects[i].rect.height)); 
       DrawRotatedRects(RGB, rRects); 
       }

       cv::imshow("Contact", RGB); cv::moveWindow("Contact", 100, 400); 
     */ 
    time = ((double)cv::getTickCount()-time)/cv::getTickFrequency();

    //std::cout << "time = " << time << std::endl;
    char key = cv::waitKey(1);
    if (key=='q') {
        exit(0);
    }
}





bool colorSetting(ros::Publisher& point_pub, ros::Publisher& pub_psw, bool saveMode=false){

    ros::WallDuration sleep_time(0.05);
    ros::Time current_time = ros::Time::now();

    cv::Mat RGB;
    cropImage(originRGB, RGB);

    cv::moveWindow("RGB", 100, 100); cv::imshow("RGB", RGB);
    color.display(RGB);

    char key = cv::waitKey(100);
    if (key=='q')
        return true;

    else if (key-'0'>=0 && key-'0'<=9) {
        color.modifyColor(key-'0');
    }
    else if (key=='a') color.addColor();
    else if (key=='d') color.deleteColor();
    else if (key=='v') {
        std::cout << "-------------------------------" << std::endl;
        std::cout << "Palette list" << std::endl;
        std::cout << "-------------------------------" << std::endl;
        for (std::size_t i=0; i<color.isObject.size(); i++)
            std::cout <<" " <<  i << ": " << color.colorName[i] << std::endl;
        std::cout << "-------------------------------" << std::endl;
    }
    return false; 
}

bool is_updated_rgb = false;
bool is_updated_depth = false;

void cameraCallbackForRGB(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& camera_info_msg, ros::Publisher& point_pub, ros::Publisher& pub_psw){

    static bool colorTuned=false; 

    double fx = camera_info_msg->K[0];
    double fy = camera_info_msg->K[4];
    double cx = camera_info_msg->K[2];
    double cy = camera_info_msg->K[5];

    try{
        rawRGB = cv_bridge::toCvShare(msg, "bgr8")->image;
        //rawRGB = cv::imdecode(msg->data, CV_LOAD_IMAGE_COLOR);
        //rawRGB = cv_bridge::toCvShare(msg, "bgr8")->image;
        if(!rawRGB.empty()){
            if( is_updated_depth && !is_running){
                is_running = true;
                cv::resize(rawRGB, originRGB, cv::Size(320,240));
                float image_scale = (float)originRGB.rows/(float)rawRGB.rows;
                is_updated_rgb = true;

                if (!colorTuned) 
                    colorTuned=colorSetting(point_pub, pub_psw);  
                else {
                    test(fx*image_scale, fy*image_scale, cx*image_scale, cy*image_scale, point_pub, pub_psw, false);
                }
                is_running = false;
                is_updated_rgb = false;
                is_updated_depth = false;
            }
        }

        } catch (cv_bridge::Exception& e){
                //ROS_ERROR("Could not convert into cv::Mat %s", msg->encoding.c_str());
        }
}


void imageCallbackForDepth(const sensor_msgs::ImageConstPtr& msg){
        try{
                //rawDepth = cv::imdecode(msg->data, cv::IMREAD_ANYDEPTH);
                rawDepth = cv_bridge::toCvShare(msg)->image;

                if(rawDepth.type() == CV_32FC1){
                        //            std::cout << "RAW DEPTH1: " << rawDepth.at<float>(200,200) << std::endl;
                        rawDepth.convertTo(rawDepth, CV_32FC1, 1000.0/1.0);
                }
                cv::Mat image_float;
                rawDepth.convertTo(image_float, CV_16UC1);
                if(!image_float.empty()){
                        if(!is_running || !is_updated_depth){

                                cv::resize(image_float, originDepth, cv::Size(320,240));
                                //std::cout << "RESOLUTION: " << originDepth.rows << "\t" << originDepth.cols << std::endl;
                                is_updated_depth = true;
                                //cv::imshow("Depth", image_float*1000);
                                //cv::imshow("originDepth", originDepth);

                                //cv::waitKey(1);
                        }
                }
        } catch (cv_bridge::Exception& e){
                //ROS_ERROR("Could not convert into cv::Mat %s", msg->encoding.c_str());
        } catch (cv::Exception& e){

        }

}

int main(int argc, char** argv){
        ros::init(argc, argv, "recognition");
        ros::NodeHandle nh("~");
        ros::NodeHandle node_handle;

        //ros::AsyncSpinner spinner(2);
        //spinner.start();


        g_tf_listener = new tf::TransformListener();

        lua = luaL_newstate();
        luaL_openlibs(lua);

        plaincode::th::distAbove=0; // 실제 값은 -100
        plaincode::th::thSize=100;
        plaincode::th::thC=20;
        cv::Rect imgRect(20,20,280,200); 

        std::string lua_files, object_files, color_file, setup_files; 
        nh.getParam("lua_files", lua_files);
        nh.getParam("setup_files", setup_files); 

        color.setFile(std::string(setup_files+"/palette"));
        std::cout << setup_files << std::endl; 

        luaL_dofile(lua, std::string(lua_files + "/0_parameter.lua").c_str());
        allLut.push_back(cv::Scalar(0,0,0)); 

        // patched_things setting // 	
        patched_things.setup(lua, lua_files, std::string(setup_files+"/patched_things.info"), imgRect, nameTags);
        allLut.insert(allLut.end(), patched_things.lut.begin(), patched_things.lut.end()); 

        // colored_things setting //
        colored_things.setup(lua, lua_files, std::string(setup_files+"/colored_things.info"), imgRect, nameTags, patched_things.labelOffset+patched_things.subDir.size()); 
        allLut.insert(allLut.end(), colored_things.lut.begin(), colored_things.lut.end()); 

        // other_things setting // 
        other_things.setup(lua, lua_files, std::string(setup_files+"/other_things.info"), imgRect, nameTags, colored_things.labelOffset+colored_things.subDir.size()); 
        allLut.insert(allLut.end(), other_things.lut.begin(), other_things.lut.end()); 

        boost::filesystem::path patchSaveDir = patched_things.rootDir/patched_things.rawDir; 
        boost::filesystem::path colorSaveDir = colored_things.rootDir/colored_things.rawDir; 
        boost::filesystem::path objectSaveDir =  other_things.rootDir/other_things.rawDir; 

        std::string ext = "_result.jpg";
        for (std::size_t i=0; i<patched_things.subDir.size(); i++)
                patched_things.rename(patched_things.rootDir/patched_things.resultDir/patched_things.subDir[i], ext);
        lua_pushstring(lua, patched_things.rootDir.c_str()); lua_setglobal(lua, "class");
        luaL_dofile(lua, std::string(lua_files + "/4_loadModel.lua").c_str());

        for (std::size_t i=0; i<colored_things.subDir.size(); i++)
                colored_things.rename(colored_things.rootDir/colored_things.resultDir/colored_things.subDir[i], ext);
        lua_pushstring(lua, colored_things.rootDir.c_str()); lua_setglobal(lua, "class");
        std::cout << colored_things.rootDir.c_str() << std::endl; 
        luaL_dofile(lua, std::string(lua_files + "/4_loadModel.lua").c_str());

        for (std::size_t i=0; i<other_things.subDir.size(); i++)
                other_things.rename(other_things.rootDir/other_things.resultDir/other_things.subDir[i], ext);
        lua_pushstring(lua, other_things.rootDir.c_str()); lua_setglobal(lua, "class");
        luaL_dofile(lua, std::string(lua_files + "/4_loadModel.lua").c_str());

        std::cout << "===============================" << std::endl; 
        std::cout << "Color setting" << std::endl; 
        std::cout << " a : Add new palette" << std::endl; 
        std::cout << " d : Delete palette" << std::endl; 
        std::cout << " v : View current palette" << std::endl; 
        std::cout << " q : Quit color setting" << std::endl; 
        std::cout << "===============================" << std::endl; 

        ros::Publisher pub_psw = node_handle.advertise<moveit_msgs::PlanningSceneWorld>("planning_scene_world", 1);
        ros::Publisher point_pub = nh.advertise<simple_recognition::RecogObject>("object_point",100);

        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub_for_depth = it.subscribe("/rgbd_receiver/depth_registered/image_raw", 1, imageCallbackForDepth);
        image_transport::CameraSubscriber sub_for_rgb = it.subscribeCamera("/rgbd_receiver/rgb/image_raw", 1, boost::bind(cameraCallbackForRGB, _1, _2,  point_pub, pub_psw));

        //ros::waitForShutdown();
        ros::spin();


        lua_close(lua);
        return 0;
}
