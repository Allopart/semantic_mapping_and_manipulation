// STL Header
#include <iostream>
#include <numeric>
#include <memory>
#include "cbf.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <compressed_depth_image_transport/compressed_depth_subscriber.h>
#include <compressed_image_transport/compressed_subscriber.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

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
bool isRotated=false;

//std::vector<cv::Scalar> lut;
cv::Mat originRGB, originDepth;
cv::Mat rawRGB, rawDepth;

Eigen::VectorXf parameter = Eigen::VectorXf(3);


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

int accumulate(double fx, double fy, double cx, double cy, ros::Publisher& point_pub, ros::Publisher& pub_psw, bool saveMode=false){

  boost::filesystem::path patchSaveDir = patched_things.rootDir/patched_things.rawDir; 
	boost::filesystem::path colorSaveDir = colored_things.rootDir/colored_things.rawDir;
	boost::filesystem::path objectSaveDir = other_things.rootDir/other_things.rawDir; 

  static bool tuneMode=false;

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
  std::vector<std::vector<cv::Point2f>> rotatedPt;
  std::vector<cv::Point2f> refPt(4);
  refPt[0] = cv::Point(0,0);
  refPt[1] = cv::Point(64,0);
  refPt[2] = cv::Point(64,64);
  refPt[3] = cv::Point(0,64);

  int refIdx=0;
  for (auto i=0; i<4; i++) {
      rotatedPt.push_back(refPt);
      for (auto j=0;j<4; j++) rotatedPt[i][j]=refPt[(refIdx+j)%4];
      refIdx++;
  }

  cv::Mat feature;
  Eigen::MatrixXf points;

  double time = cv::getTickCount();
  cropImage(originRGB, RGB);
  cropImage(originDepth, Depth);

  plaincode::crossBilateralFilter(RGB, Depth);
  plaincode::getPoint3d(Depth, points, Kinv);
  plaincode::imshow("Depth", Depth, 1);

  plaincode::planeParameter(Depth, points, parameter);
//  plaincode::planeParameter_pcl(Depth, points, parameter); 
  plaincode::getDistMat(Depth, distMat, points, parameter);

  color.RGB2Mask(RGB, handMask, 0);
  handMask.setTo(0, distMat>-30);

//  plaincode::getCandidate(distMat, onPlane, handMask, points, parameter, K, -20.);
  plaincode::removePlane(distMat, underPlane, plaincode::CONSTANT, 1200., 80.);

  cv::Mat tuneMask;
  cv::imshow("hand", handMask);
  onPlane.copyTo(tuneMask, ~handMask);
  plaincode::tuneThreshold(tuneMode, distMat, tuneMask);
  // segmentMask 중, 가장 큰 것을 찾자.

  std::vector<cv::Mat> patchMat;
	patched_things.clearBlob(); 
	patched_things.appendPatches(RGB, patchMat, rotatedPt);
  patched_things.save(patchMat, patchSaveDir); 

  // color
	cv::Mat colored; 
	colored_things.clearBlob(); // modd 
	for (int i=0; i<color.isObject.size(); i++) {
    if (color.isObject[i]==0) continue; 
		color.RGB2Mask(RGB, colored, i); 
		cv::medianBlur(colored, colored, 5); 
		colored_things.append(colored); 
	}

	colored_things.project(points, parameter, K);
  std::vector<cv::Mat> inputMat;
	inputMat.clear(); 
	colored_things.accumulate(originRGB, inputMat); 
	colored_things.save(inputMat, colorSaveDir); 

  // object
	other_things.clearBlob(); 
  cv::Mat prPlane; 
  other_things.projectOnPlane(distMat, onPlane, prPlane, handMask, points, parameter, K, -20.); 
  other_things.projectBlob(prPlane, 50); 
  other_things.reconstructBlob(onPlane, points, parameter, K); 
  other_things.filter(underPlane); 
   
	other_things.accumulate(originRGB, inputMat); 	
	other_things.save(inputMat, objectSaveDir); 	

	cv::moveWindow("RGB", 100, 100); cv::imshow("RGB", RGB);

  time = ((double)cv::getTickCount()-time)/cv::getTickFrequency();
//  std::cout << "time = " << time << std::endl;
  char key = cv::waitKey(1);
  if (key=='q') {
      exit(0);
  }
  else if (key=='t') {
    tuneMode=!tuneMode;
  }
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
        if(!rawRGB.empty()){
            if( is_updated_depth && !is_running){
                is_running = true;
                cv::resize(rawRGB, originRGB, cv::Size(320,240));

             		float image_scale = (float)originRGB.rows/(float)rawRGB.rows;
                is_updated_rgb = true;
                if (!colorTuned) {
                  colorTuned=colorSetting(point_pub, pub_psw);
                  }
                else accumulate(fx*image_scale, fy*image_scale, cx*image_scale, cy*image_scale, point_pub, pub_psw); 
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
                is_updated_depth = true;
            }
        }
    } catch (cv_bridge::Exception& e){
        //ROS_ERROR("Could not convert into cv::Mat %s", msg->encoding.c_str());
    } catch (cv::Exception& e){

    }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "accumulate");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

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
  luaL_dofile(lua, std::string(lua_files + "/4_loadModel.lua").c_str());

	for (std::size_t i=0; i<other_things.subDir.size(); i++)
		other_things.rename(other_things.rootDir/other_things.resultDir/other_things.subDir[i], ext);
  lua_pushstring(lua, other_things.rootDir.c_str()); lua_setglobal(lua, "class");
  luaL_dofile(lua, std::string(lua_files + "/4_loadModel.lua").c_str());
  
  std::cout << "===============================" << std::endl; 
  std::cout << "Color setting" << std::endl; 
  std::cout << " a: Add new palette" << std::endl; 
  std::cout << " d: Delete palette" << std::endl; 
  std::cout << " v: View current palette" << std::endl; 
  std::cout << " q: Quit color setting " << std::endl; 
  std::cout << "===============================" << std::endl; 

	ros::Publisher pub_psw = node_handle.advertise<moveit_msgs::PlanningSceneWorld>("planning_scene_world", 1);
	ros::Publisher point_pub = nh.advertise<simple_recognition::RecogObject>("object_point",100);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_for_depth = it.subscribe("/rgbd_receiver/depth_registered/image_raw", 1, imageCallbackForDepth);
  image_transport::CameraSubscriber sub_for_rgb = it.subscribeCamera("/rgbd_receiver/rgb/image_raw", 1, boost::bind(cameraCallbackForRGB, _1, _2,  point_pub, pub_psw));
		
  ros::spin();

	lua_close(lua);
  return 0;
}
