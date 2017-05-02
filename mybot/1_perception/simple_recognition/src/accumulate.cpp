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
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>

// Eigen Header
#include <eigen3/Eigen/Dense>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

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
plaincode::Object objects; 
plaincode::Color color;

//std::vector<cv::Scalar> lut;
cv::Mat originRGB, originDepth;
cv::Mat rawRGB, rawDepth;
cv::Rect imgRect(20,20,280,200); 

Eigen::VectorXf parameter = Eigen::VectorXf(3);
cv::Mat dilateMask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)); 

cv::Point object_center_point;
bool is_object_center_point_updated = false;


void cropImage(cv::Mat& image, cv::Mat& result) {
    result = image(imgRect);
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

	boost::filesystem::path objectSaveDir = objects.rootDir/objects.rawDir; 

  static pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBCloudPtr; 
  static boost::shared_ptr<pcl::visualization::PCLVisualizer> RGBCloudViewer, RGBCloudViewer2; 
  std::vector<pcl::ModelCoefficients> pclModels; 

  static boost::once_flag flag = BOOST_ONCE_INIT; 
  boost::call_once([] 
                  {
                  plaincode::createViewer(RGBCloudViewer, "3D Viewer"); 
                  plaincode::createViewer(RGBCloudViewer2, "3d Viewer2"); 
                  plaincode::setViewer(RGBCloudViewer, cv::Point(400,100), cv::Size(300,500), cv::Scalar(0,0,0)); // row, col
                  plaincode::setViewer(RGBCloudViewer2, cv::Point(400,600), cv::Size(300,500), cv::Scalar(0,0,0)); // row, col

                  RGBCloudPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); // for rgb scale
                  }, flag); 

  ros::WallDuration sleep_time(0.05);
  ros::Time current_time = ros::Time::now();

  Eigen::MatrixXf K, Kinv;
  plaincode::setCameraMat(fx, fy, cx-20.0, cy-20.0, K, Kinv);

  cv::Mat RGB, Depth;
  cv::Mat underPlane;
  cv::Mat distMat;
  cv::Mat Mask, Label;
  cv::Mat absoluteRGB;
  cv::Mat handMask;

  Eigen::MatrixXf points;

  double time = cv::getTickCount();
  cropImage(originRGB, RGB);
  cropImage(originDepth, Depth);

  plaincode::crossBilateralFilter(RGB, Depth);
  plaincode::getPoint3d(Depth, points, Kinv);
  plaincode::getDistMat(Depth, distMat, points, parameter);
  plaincode::removePlane(distMat, underPlane, plaincode::CONSTANT, 1200, 80); 

  color.RGB2Mask(RGB, handMask, 0);
  handMask.setTo(0, distMat>-30);
  cv::dilate(handMask, handMask, dilateMask); 

  cv::Mat handIdMat; 
  std::vector<plaincode::Blob> handTempBlob; 
  plaincode::findBlob(handMask, handIdMat, handTempBlob, 225); 
  handMask = handIdMat!=0; 

  Eigen::MatrixXf handPoints; 
  plaincode::findSamplePoints(handMask, points, handPoints); 

  std::vector<cv::Mat> inputMat; 
  cv::Mat onPlane, prPlane; 

  objects.clearBlobs(); 
  
  objects.projectOnPlane(distMat, onPlane, prPlane, handMask, points, parameter, K, -20.); 
  cv::imshow("onPlane", onPlane); 
  objects.refine(RGB, onPlane, prPlane, points, parameter, K, color.hsvMin, color.hsvMax, color.isObject); 
  objects.projectBlob(prPlane, 50); 
  objects.reconstructBlob(onPlane, points, parameter, K); 
  objects.filter(underPlane); 

  // filtering of manipulated objects 
  std::vector<bool> isObject(objects.blobs.size(), true); 
  std::vector<cv::Scalar> dispColors(objects.blobs.size(), cv::Scalar(0,0,0)); 
  for (std::size_t i=0; i<objects.blobs.size(); i++) {
    int count = plaincode::countSphereMask(objects.blobs[i].centerPos3d, handPoints, 150); 
    if (count>500) {
      dispColors[i] = cv::Scalar(0,0,255); 
      isObject[i] = false; 
    }
  }

  objects.accumulate(originRGB, inputMat); 
  objects.save(inputMat, objects.rootDir/objects.rawDir, isObject); 

	cv::moveWindow("RGB", 100, 100); cv::imshow("RGB", RGB);
    plaincode::imshow("Depth", originDepth, 1); 
  plaincode::getPointCloud(RGB, points, RGBCloudPtr); 

  for (std::size_t i=0; i<objects.blobs.size(); i++) 
    plaincode::getSphereCloud(pclModels, objects.blobs[i].centerPos3d); 

  plaincode::applyPointCloud(RGBCloudViewer, RGBCloudPtr);  
  plaincode::applyPointCloud(RGBCloudViewer2, RGBCloudPtr);  

  plaincode::applySphereCloud(RGBCloudViewer, pclModels, dispColors); 
 

  RGBCloudViewer->spinOnce(1, true); 
  RGBCloudViewer2->spinOnce(1, true); 

  time = ((double)cv::getTickCount()-time)/cv::getTickFrequency();
//  std::cout << "time = " << time << std::endl;
  char key = cv::waitKey(1);
  if (key=='q') {
      exit(0);
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
                else { 
                  accumulate(fx*image_scale, fy*image_scale, cx*image_scale, cy*image_scale, point_pub, pub_psw); 
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
                is_updated_depth = true;
            }
        }
    } catch (cv_bridge::Exception& e){

        //ROS_ERROR("Could not convert into cv::Mat %s", msg->encoding.c_str());
    } catch (cv::Exception& e){

    }

}

void normalCallback(const geometry_msgs::Vector3::ConstPtr& normalMsg) {
  parameter(0) = normalMsg->x; 
  parameter(1) = normalMsg->y; 
  parameter(2) = normalMsg->z; 
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
  
  std::string lua_files, object_files, color_file, setup_files, setup_file; 
  nh.getParam("lua_files", lua_files);
  nh.getParam("setup_files", setup_files); 
  if (setup_files[0]=='~') setup_files.replace(0,1,getenv("HOME")); 

  std::cout << setup_files << std::endl; 

  color.setFile(std::string(setup_files+"/palette"));

  luaL_dofile(lua, std::string(lua_files + "/0_parameter.lua").c_str());

  // other_things setting // 
	objects.setup(lua, lua_files, setup_files, imgRect); 

	boost::filesystem::path objectSaveDir =  objects.rootDir/objects.rawDir; 

	std::string ext = "_result.jpg";

	for (std::size_t i=0; i<objects.subDir.size(); i++)
		plaincode::rename(objects.rootDir/objects.resultDir/objects.subDir[i], ext);
  lua_pushstring(lua, objects.rootDir.c_str()); lua_setglobal(lua, "class");
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
  ros::Subscriber normal_sub = nh.subscribe("/simple_recognition/normal", 1, normalCallback); 

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_for_depth = it.subscribe("/rgbd_receiver/depth_registered/image_raw", 1, imageCallbackForDepth);
  image_transport::CameraSubscriber sub_for_rgb = it.subscribeCamera("/rgbd_receiver/rgb/image_raw", 1, boost::bind(cameraCallbackForRGB, _1, _2,  point_pub, pub_psw));
		
  ros::spin();

	lua_close(lua);
  return 0;
}
