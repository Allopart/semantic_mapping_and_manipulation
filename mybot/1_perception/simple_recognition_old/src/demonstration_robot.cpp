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
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

// plaincode header
#include "Core.hpp"
#include "Color.hpp"
#include "Object.hpp"
#include "patch.hpp"
#include "contact.hpp"
#include "Trajectory.hpp"
#include "Thermal.hpp"

// Boost Header
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <simple_recognition/RecogObject.h>

#include <std_msgs/String.h>

lua_State *lua;
plaincode::Object patched_things;
plaincode::Object colored_things;
plaincode::Object other_things; 
plaincode::Color color;
std::vector<std::string> nameTags;
std::vector<cv::Scalar> allLut; 
std::vector<plaincode::Blob> objects; 
cv::Mat labelMat, prLabelMat; 
cv::Mat prevLabelMat,prevPrLabelMat; 
std::vector<plaincode::Blob> prevObjects; 
int resultPosition = 100; 

std::string RNN_files; 

int drawMode=0; 
std::vector<cv::Scalar> drawColor;
std::vector<plaincode::Trajectory> handPath; 

std::vector<cv::Mat> RGBs;
std::vector<cv::Mat> Depths; 
std::vector<cv::Mat> Results; 
std::vector<cv::Mat> Thermals; 

std::vector<std::vector<double>> channels1; 
std::vector<std::vector<double>> channels2; 
std::vector<std::vector<double>> channels3; 

std::ofstream memory_txt; 

std::vector<std::vector<double>> contextAct; 
std::vector<double> contextObj; 
Eigen::MatrixXf derivedPath;
bool drawTrajectory=false;
Eigen::Vector3f objectPos1, objectPos2; 

cv::Mat prevDepth; 
int minDistIdx;  
int graspedIdx; 
bool isRotated=false;
int trial=0;
int state=0; 

ros::Publisher pub_state; 
ros::Publisher pub_start; 
int publish_flag = 0;

#define FadeOut 0
#define DoubleTake 1
#define ObjectInHand 2
#define ObjectOverlap 3

//std::vector<cv::Scalar> lut;
cv::Mat originRGB, originDepth, originThermal;
cv::Mat rawRGB, rawDepth;
cv::Mat RGBDT_transmat;
std::string RGBDT_transmat_file; 

Eigen::VectorXf parameter = Eigen::VectorXf(3);
cv::Mat dilateMask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)); 

cv::Point object_center_point;
bool is_object_center_point_updated = false;


void cropImage(cv::Mat& image, cv::Mat& result) {
        cv::Rect rect = cv::Rect(20, 20, 280, 200);
        result = image(rect);
}

std::string thermalMode; 

bool is_running = false;
bool is_updated_rgb = false;
bool is_updated_depth = false;
bool is_updated_thermal = false; 

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBCloudPtr; 
boost::shared_ptr<pcl::visualization::PCLVisualizer> RGBCloudViewer; 
boost::shared_ptr<pcl::visualization::PCLVisualizer> RGBCloudViewer2; 

int test(double fx, double fy, double cx, double cy, ros::Publisher& point_pub, ros::Publisher& pub_psw, bool saveMode=false){

        static boost::once_flag flag = BOOST_ONCE_INIT;
        boost::call_once([]
                        {
                        RGBCloudViewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("3D Viewer"));
                        RGBCloudViewer->initCameraParameters ();
                        RGBCloudViewer->setBackgroundColor(0,0,0);
                        RGBCloudViewer->setPosition(500,600);
                        RGBCloudViewer->setSize(500,300);

                        RGBCloudViewer2 = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("3D Viewer2"));
                        RGBCloudViewer2->initCameraParameters ();
                        RGBCloudViewer2->setBackgroundColor(0,0,0);
                        RGBCloudViewer2->setPosition(500,600);
                        RGBCloudViewer2->setSize(500,300);

                        }, flag);

        static boost::filesystem::path demonstrationDir("/home/yongho/Demonstration_robot"); 
        boost::filesystem::create_directories(demonstrationDir); 
        static boost::filesystem::path subDemoDir;
        static boost::filesystem::path rgbDir; 
        static boost::filesystem::path depthDir; 
        static boost::filesystem::path recognitionDir; 
        static boost::filesystem::path trajectoryDir; 
        static boost::filesystem::path thermalDir; 

        static std::fstream fileTrajectory; 

        bool isHappened=false;
        std::vector<double> dist2Hand; 
        static bool tuneMode=false;
        std::string graspedName; 

        std::ostringstream eventString; 
        eventString.clear(); 

        // for memory. 
        std::vector<double> channel1(5,0); // approach, grasp, move, pour, release 
        std::vector<double> channel2(nameTags.size()+1,0); 
        std::vector<double> channel3(nameTags.size()+1,0); // object + table

   
        ros::WallDuration sleep_time(0.05);

        ros::Time current_time = ros::Time::now();
  
        Eigen::MatrixXf K, Kinv;

        plaincode::setCameraMat(fx, fy, cx-20.0, cy-20.0, K, Kinv);

        cv::Mat gray;
        cv::Mat RGB, Depth, Thermal; 
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
        
        if (thermalMode=="on") {
            cv::Mat thermalTemp; 
            cropImage(originThermal, thermalTemp); 
            plaincode::getTransformed(RGB, Depth, thermalTemp, Thermal, RGBDT_transmat); 
            double min,max; 
            cv::minMaxLoc(originThermal, &min, &max); 
            Thermal.setTo((unsigned short)min, Thermal<(unsigned short)8015);
        }

        std::vector<cv::Mat> inputMat;
        std::vector<cv::Mat> patchMat; 

        std::vector<int> patchLabel; 
        std::vector<int> outputLabel;

        plaincode::crossBilateralFilter(RGB, Depth);
        plaincode::getPoint3d(Depth, points, Kinv);

        plaincode::planeParameter(Depth, points, parameter);
        plaincode::getDistMat(Depth, distMat, points, parameter);

        color.RGB2Mask(RGB, handMask, 0);
        handMask.setTo(0, distMat>-30);
        cv::dilate(handMask, handMask, dilateMask); 
    
        plaincode::removePlane(distMat, underPlane, plaincode::CONSTANT, 1200., 80.);

        cv::Mat tuneMask;
        cv::medianBlur(handMask, handMask, 5);
        cv::imshow("hand", handMask);
        onPlane.copyTo(tuneMask, ~handMask);
        plaincode::tuneThreshold(tuneMode, distMat, tuneMask);
        // segmentMask 중, 가장 큰 것을 찾자.
        plaincode::Blob handBlob;
        cv::Mat handMat = cv::Mat::zeros(RGB.size(), CV_8UC1); 

        handBlob.centerPos3d = Eigen::Vector3f(-100, -100, -100); 
        if (plaincode::findMaxBlob(handMask, handMat, handBlob)) {
                plaincode::findCoM(handBlob, handMat, points); 
        }

        switch(state) {
                case FadeOut:
                        {
                                objects.clear(); 
                                labelMat.setTo(0); 
                                prLabelMat.setTo(0); 

                                patched_things.clearBlob();
                                cv::Mat tempRGB = RGB.clone(); 
                                tempRGB.setTo(cv::Scalar(0,0,0), distMat>-10);  
/*                                patched_things.appendPatches(tempRGB, patchMat, rotatedPt);
                                patched_things.project(points, parameter, K, 0); 
                                patched_things.lua_classify(lua, patchMat, patchLabel, 0.99); 
                                patched_things.reflect(patchMat, patchLabel); 

                                objects.insert(objects.end(), patched_things.blobs.begin(), patched_things.blobs.end()); 
                                patched_things.labelMat.copyTo(labelMat, patched_things.labelMat); 
                                patched_things.prLabelMat.copyTo(prLabelMat, patched_things.prLabelMat); 
                                if (saveMode) patched_things.save(patchMat); */

                                onPlane.setTo(0, patched_things.labelMat); 
                                ////////////////////////
                                // color
                                cv::Mat colored; 
                                colored_things.clearBlob(); 
                                for (int i=0; i<color.isObject.size(); i++) {
                                        if (color.isObject[i]==0) continue; 

                                        color.RGB2Mask(RGB, colored, i); 
                                        cv::medianBlur(colored, colored, 5); 
                                        colored_things.append(colored, 100); 
                                }

                                colored_things.project(points, parameter, K); 
                                colored_things.filter(underPlane); 
                                colored_things.accumulate(originRGB, inputMat); 
                                colored_things.lua_classify(lua, inputMat, outputLabel, 0.9999);
                                colored_things.reflect(inputMat, outputLabel); 
                                if (saveMode) colored_things.save(inputMat); 

                                objects.insert(objects.end(), colored_things.blobs.begin(), colored_things.blobs.end()); 
                                colored_things.labelMat.copyTo(labelMat, colored_things.labelMat); 
                                colored_things.prLabelMat.copyTo(prLabelMat, colored_things.prLabelMat); 

//                                onPlane.setTo(0, colored_things.labelMat|handMask); 
                                other_things.clearBlob(); 
                                
                                cv::Mat prPlane; 
                                other_things.projectOnPlane(distMat, onPlane, prPlane, colored_things.labelMat|handMask, points, parameter, K, -20.); 
                                other_things.projectBlob(prPlane, 120); 
                                other_things.reconstructBlob(onPlane, points, parameter, K); 
                                other_things.filter(underPlane); 
                                plaincode::imshow("id", other_things.idMat, 1); 

//                                other_things.append(onPlane); 
//                                other_things.project(points, parameter, K); 
//                                other_things.filter(underPlane); 
                                other_things.accumulate(originRGB, inputMat); 
                                other_things.lua_classify(lua, inputMat, outputLabel, 0.995); 
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

                                if (objects.size()>0) {
                                        for (std::size_t i=0; i<objects.size(); i++) {
                                                dist2Hand.push_back((objects[i].centerPos3d-handBlob.centerPos3d).norm()); 
                                        }
                                        minDistIdx = std::distance(dist2Hand.begin(), std::min_element(dist2Hand.begin(), dist2Hand.end())); 
                                        if (dist2Hand[minDistIdx]<150) {
                                                state=DoubleTake; 
                                                // Approach
                                                isHappened=true; 
                                                
                                                if (drawMode!=1) {
                                                  RGBs.clear(); 
                                                  Depths.clear(); 
                                                  Results.clear(); 
                                                  if (thermalMode=="on") Thermals.clear(); 
                                                  handPath.clear(); 
                                                  channels1.clear(); channels2.clear(); channels3.clear();
                                                  contextAct.clear(); 
                                                  
                                                  objectPos1 = Eigen::Vector3f(0.0, 0.0, 0.0); 
                                                  objectPos2 = Eigen::Vector3f(0.0, 0.0, 0.0); 
                                                 }

                                                drawMode=1; 
                                                                                               
                                                std::cout << std::endl; 
                                                std::cout << "#" << trial << " Demonstration --------" << std::endl; 
                                                eventString << "Approach " << objects[minDistIdx].name << " on the TABLE"; 
                                                objectPos1 = objects[minDistIdx].prCenterPos3d; 
                                                std::cout << objects[minDistIdx].name << std::endl; 
                                                
                                                memory_txt.close(); 
                                                memory_txt.open("/home/yongho/catkin_ws/src/mybot2_new/2_memory/deep_art/file/cueList.txt", std::ios::in & std::ios::trunc);
                                                memory_txt << " " << objects[minDistIdx].name << " " << std::endl; 
                                                
                                                if (publish_flag == 0)
                                                {
                                                        std_msgs::String start_msgs;
                                                        start_msgs.data = "START";
                                                        pub_start.publish(start_msgs);

                                                        ros::Duration wait_until_publish(2.0);
                                                        wait_until_publish.sleep();

                                                        std_msgs::String state_msgs;
                                                        state_msgs.data = "SUCCESS";
                                                        pub_state.publish(state_msgs);

                                                        publish_flag = 1;
                                                }

                                                std::fill(channel1.begin(), channel1.end(), 0.); 
                                                channel1[0]=1; // approach 
                                                
                                                std::fill(channel2.begin(), channel2.end(), 0.); 
                                                channel2[objects[minDistIdx].label-1]=1; 
 
                                                std::fill(channel3.begin(), channel3.end(), 0.); 
                                                channel3.back() = 1; 

                                                channels1.push_back(channel1); 
                                                channels2.push_back(channel2); 
                                                channels3.push_back(channel3); 

                                                int startIdx=-1, endIdx=-1;                                                 

                                                if (objects[minDistIdx].label-1==5 || objects[minDistIdx].label-1==6) { // cereal, milk
                                                  startIdx = minDistIdx; 
                                                  for (std::size_t i=0; i<objects.size(); i++) {
                                                      if (objects[i].label-1==3) { 
                                                        std::cout << "CEREAL or MILK and BOWL" << std::endl; 
                                                        endIdx = i; 
                                                        break;  
                                                      }
                                                  }                                               
                                                } 
                                                else if (objects[minDistIdx].label-1==7) {
                                                  startIdx = minDistIdx; 
                                                  for (std::size_t i=0; i<objects.size(); i++) {
                                                     if (objects[i].label-1==4) {
                                                        std::cout << "BOTTLE and CUP" << std::endl; 
                                                        endIdx = i; 
                                                        break;
                                                      }
                                                  }
                                                }
                                                drawTrajectory=false; 
                                                if (endIdx!=-1 && startIdx!=-1) { 
                                                  auto objectPos1 = objects[startIdx].prCenterPos3d; 
                                                  auto objectPos2 = objects[endIdx].prCenterPos3d; 
                                                  auto handPos3d = handBlob.centerPos3d;  
                                                  
                                                  Eigen::MatrixXf matT(4,4); matT.setIdentity(4,4); 
                                                  Eigen::MatrixXf matR(4,4); matR.setIdentity(4,4); 
                                                  Eigen::MatrixXf matFlip(4,4); matFlip.setIdentity(4,4); 
                                                  matFlip(1,1)=-1; 
                                                  
                                                  matT.block(0,3,3,1)=-objectPos1; 
                                                  matT(3,3)=1.0; 
                                                  
                                                  Eigen::Vector3f xAxis = objectPos2 - objectPos1; float scaleFactor = xAxis.norm(); xAxis = xAxis/scaleFactor; 
                                                  Eigen::Vector3f zAxis(parameter); zAxis = zAxis/zAxis.norm(); 
                                                  Eigen::Vector3f yAxis = zAxis.cross(xAxis); 
                                                  
                                                  matR.block(1,0,1,3) = yAxis.transpose(); 
                                                  matR.block(2,0,1,3) = xAxis.transpose(); 
                                                  matR.block(0,0,1,3) = zAxis.transpose(); 

                                                  Eigen::MatrixXf allHandPath(4,1); 
                                                  allHandPath(0,0) = handPos3d.x();
                                                  allHandPath(1,0) = handPos3d.y();
                                                  allHandPath(2,0) = handPos3d.z();
                                                  allHandPath(3,0) = 1.0; 
                                            
                                                  Eigen::MatrixXf normHandPath; 
                                                  normHandPath = 1/scaleFactor*matR*matT*allHandPath; 

                                                  bool isFlip=false; 
                                                  if (normHandPath(1,0)<0) {
                                                    isFlip=true; 
                                                    normHandPath = matFlip*normHandPath; 
                                                  }
                                                  
                                                  objects[startIdx].label-1; 
                                                  objects[endIdx].label-1; 

                                                  contextObj.resize(channel2.size()); 
                                                  std::fill(contextObj.begin(), contextObj.end(), 0.0); 
                                                  contextObj[objects[startIdx].label-1]=1; 
                                                  contextObj[objects[endIdx].label-1]=1; 

                                                  int key=1; 
                                                  lua_newtable(lua); 

                                                  for (std::size_t i=3; i<contextObj.size(); i++) {
                                                    lua_pushnumber(lua, key++); 
                                                    lua_pushnumber(lua, contextObj[i]); 
                                                    lua_settable(lua, -3); 
                                                  }
/*
                                                  for (std::size_t i=0; i<5; i++) {
                                                    if (i==0) {
                                                      std::cout << 1 << "\t"; 
                                                      lua_pushnumber(lua, key++); 
                                                      lua_pushnumber(lua, 1); 
                                                      lua_settable(lua, -3); 
                                                    }
                                                    else {
                                                      std::cout << 0 << "\t"; 
                                                      lua_pushnumber(lua, key++); 
                                                      lua_pushnumber(lua, 0); 
                                                      lua_settable(lua, -3); 
                                                    }
                                                  }*/

                                                  for (std::size_t i=0; i<3; i++) {
                                                    std::cout << normHandPath(i,0) << "\t"; 
                                                    lua_pushnumber(lua, key++); 
                                                    lua_pushnumber(lua,normHandPath(i,0)); 
                                                    lua_settable(lua, -3); 
                                                  }
                                                  std::cout << std::endl; 


                                                lua_setglobal(lua, "trajectory"); 
                                                std::string rnnDir = RNN_files; 
                                                luaL_dofile(lua, (rnnDir+"/test.lua").c_str());

                                                // lua로부터 data 받아오기. 
                                                Eigen::MatrixXf resultPath(4, 25);                                                 
                                                lua_getglobal(lua, "result");
                                                for (std::size_t i=0; i<25; i++) {
                                                  lua_pushnumber(lua, i+1); 
                                                  lua_gettable(lua, -2); 
                                                  for (std::size_t j=0; j<3; j++) {
                                                    lua_pushnumber(lua, j+7); 
                                                    lua_gettable(lua, -2); 
                                                    resultPath(j,i)=lua_tonumber(lua,-1); 
                                                    lua_pop(lua, 1); 
                                                  }
                                                  resultPath(3,i)=1/scaleFactor;
                                                  lua_pop(lua, 1); 
                                                }

//                                                derivedPath = matT.inverse()*matR.inverse()*matFlip.inverse()*resultPath;
                                                if (isFlip) 
                                                   resultPath = matFlip.inverse()*resultPath; 

                                                derivedPath = matT.inverse()*matR.inverse()*scaleFactor*resultPath;
                                                //std::cout << derivedPath << std::endl; 
                                                drawTrajectory=true; 
                                                // draw
                                             }

                                             graspedIdx = minDistIdx; 
                                             state=DoubleTake;   
                                        }
                                }
  
                        }
                        break;

                case DoubleTake:
                        {
                                for (std::size_t i=0; i<objects.size(); i++) {
                                        dist2Hand.push_back((objects[i].centerPos3d-handBlob.centerPos3d).norm()); 
                                }
                                minDistIdx = std::distance(dist2Hand.begin(), std::min_element(dist2Hand.begin(), dist2Hand.end()));

                                if (dist2Hand[minDistIdx]>150) {
                                        isHappened=false;
                                        drawMode=0; 
                                        state=FadeOut; 
                                }
                                else if (dist2Hand[minDistIdx]<100) {
                                        isHappened=true; 
                                        drawMode=2; 
                                        eventString << "Grasp " << objects[minDistIdx].name << " on the TABLE"; 
                                        // When grasping, save all objects' projected position. 
                                        

                                        std::fill(channel1.begin(), channel1.end(), 0.); 
                                        channel1[1]=1; // grasp

                                        std::fill(channel2.begin(), channel2.end(), 0.); 
                                        channel2[objects[minDistIdx].label-1]=1; 

                                        std::fill(channel3.begin(), channel3.end(), 0.); 
                                        channel3.back() = 1; 

                                        channels1.push_back(channel1); 
                                        channels2.push_back(channel2); 
                                        channels3.push_back(channel3); 

                                        graspedIdx = minDistIdx; 
                                        state = ObjectInHand; 
                                }	
                        }
                        break;

       	case ObjectInHand:
			{
				plaincode::Blob blob;
				cv::Mat inHandMat;
				Eigen::Vector3f sumPos3d = (handBlob.centerPos3d+3*objects[graspedIdx].centerPos3d)/4.;

				if (objects[graspedIdx].label<=other_things.labelOffset) {

					cv::Mat sphereMask  = cv::Mat::zeros(RGB.size(), CV_8UC1), hsv = cv::Mat::zeros(RGB.size(), CV_8UC1);
					plaincode::getSphereMask(sumPos3d, points, sphereMask, 50);

					// 가장 색깔이 많이 포함되어있는 index를 찾아보자.
					int maxIdx=0, maxPixel=0;
					for (std::size_t i=0; i<color.isObject.size(); i++) {
						if (color.isObject[i]==0) continue;
						cv::Mat temp;
						color.RGB2Mask(RGB, temp, i);
						temp.setTo(0, ~sphereMask);
						int numPixel = cv::countNonZero(temp);
						if (numPixel>maxPixel) {
							maxPixel = numPixel;
							maxIdx=i;
						}
					}
					if (maxIdx!=0) color.RGB2Mask(RGB, hsv, maxIdx);

					plaincode::getColorSphereBlob(hsv, sumPos3d, handMat, distMat, inHandMat, points, 50, 180);
				}

				if (objects[graspedIdx].label>other_things.labelOffset) {
					plaincode::getSphereBlob(RGB, sumPos3d, handMat, distMat, inHandMat, points, 50, 180);
				}

				cv::Mat deleteMat = labelMat.clone(); // 이전결과.
				deleteMat.setTo(0, labelMat==objects[graspedIdx].label);
				cv::Mat diffDepth = (prevDepth-Depth)>30;
				deleteMat.setTo(0, diffDepth);

				inHandMat.setTo(0, handMat);
				inHandMat.setTo(0, deleteMat);

				inHandMat.setTo(0, Depth>Depth.at<unsigned short>(handBlob.centerPos)+50);

				std::vector<plaincode::Blob> localObjects;
				plaincode::Blob localObject;
				cv::Mat localLabelMat, prLocalLabelMat;


				if (objects[graspedIdx].label<=other_things.labelOffset) {
					colored_things.clearBlob();
					colored_things.append(inHandMat,30);
					colored_things.project(points, parameter, K, 0);
					colored_things.filter(underPlane);
					colored_things.accumulate(originRGB, inputMat);
					colored_things.lua_classify(lua, inputMat, outputLabel, 0.0001, objects[graspedIdx].label-colored_things.labelOffset);
					colored_things.reflect(inputMat, outputLabel);
					if (saveMode) colored_things.save(inputMat);
					localObjects.insert(localObjects.end(), colored_things.blobs.begin(), colored_things.blobs.end());
					localLabelMat = colored_things.labelMat.clone();
					prLocalLabelMat = colored_things.prLabelMat.clone();
				}

				if (objects[graspedIdx].label>other_things.labelOffset) {
					other_things.clearBlob();
					other_things.append(inHandMat);
					other_things.project(points, parameter, K, 0);
					other_things.filter(underPlane);
					other_things.accumulate(originRGB, inputMat);
					other_things.lua_classify(lua, inputMat, outputLabel, 0.0001, objects[graspedIdx].label-other_things.labelOffset);
					other_things.reflect(inputMat, outputLabel);
					if (saveMode) other_things.save(inputMat);
					localObjects.insert(localObjects.end(), other_things.blobs.begin(), other_things.blobs.end());
					localLabelMat = other_things.labelMat.clone();
					prLocalLabelMat = other_things.prLabelMat.clone();
				}

        // localObjects.size()가 0이면 놓아버린 것.
        if (localObjects.size()==0) {

                isHappened=true;
                drawMode=5; 
                eventString << "Release " << objects[graspedIdx].name << " on the TABLE";

                std::fill(channel1.begin(), channel1.end(), 0.); 
                channel1[4]=1; 

                std::fill(channel2.begin(), channel2.end(), 0.); 
                channel2[objects[graspedIdx].label-1]=1;

                std::fill(channel3.begin(), channel3.end(), 0.); 
                channel3.back() = 1; 


                channels1.push_back(channel1); 
                channels2.push_back(channel2); 
                channels3.push_back(channel3); 


                trial++;
                state = FadeOut;
        }

        else {
                objects[graspedIdx] = localObjects[0];

                // prevLabel -> labelMat으로 복사.
					labelMat = prevLabelMat.clone();
					prLabelMat = prevPrLabelMat.clone();

					// label에서 기존 영역 제거
					labelMat.setTo(0, labelMat==objects[graspedIdx].label);
					prLabelMat.setTo(0, prLabelMat==objects[graspedIdx].label);

					// 새로운 영역 추가.
					labelMat.setTo(objects[graspedIdx].label, localLabelMat);
					prLabelMat.setTo(objects[graspedIdx].label, prLocalLabelMat);

					dist2Hand.clear();
					for (std::size_t i=0; i<objects.size(); i++)
						dist2Hand.push_back((objects[i].prCenterPos3d-objects[graspedIdx].prCenterPos3d).norm());
					dist2Hand[graspedIdx]=9999.9;

					minDistIdx = std::distance(dist2Hand.begin(), std::min_element(dist2Hand.begin(), dist2Hand.end()));
					if (dist2Hand[minDistIdx]<50) {
                  isHappened=true;
                  drawMode=3; 
                  eventString << "Move " << objects[graspedIdx].name << " to " << objects[minDistIdx].name;
                  objectPos2 = objects[minDistIdx].prCenterPos3d; 
                  std::fill(channel1.begin(), channel1.end(), 0.); 
                  channel1[2]=1; 

                  std::fill(channel2.begin(), channel2.end(), 0.); 
                  channel2[objects[graspedIdx].label-1]=1; 

                  std::fill(channel3.begin(), channel3.end(), 0.); 
                  channel3[objects[minDistIdx].label-1]=1; 
                  std::cout << "label - 1 = " << objects[minDistIdx].label-1 << std::endl; 
                  contextObj[objects[minDistIdx].label-1]=1; 

                  channels1.push_back(channel1); 
                  channels2.push_back(channel2); 
                  channels3.push_back(channel3); 

                  state = ObjectOverlap;
          }
        }
      }

			break;

                case ObjectOverlap:
                        {
                                plaincode::Blob blob;
                                cv::Mat inHandMat;
                                Eigen::Vector3f sumPos3d = (handBlob.centerPos3d+3*objects[graspedIdx].centerPos3d)/4.;

                                if (objects[graspedIdx].label<=other_things.labelOffset) {

                                        cv::Mat sphereMask = cv::Mat::zeros(RGB.size(), CV_8UC1), hsv = cv::Mat::zeros(RGB.size(), CV_8UC1);
                                        plaincode::getSphereMask(sumPos3d, points, sphereMask, 50);

                                        // 가장 색깔이 많이 포함되어있는 index를 찾아보자.
                                        int maxIdx=0, maxPixel=0;
                                        for (std::size_t i=0; i<color.isObject.size(); i++) {
                                                if (color.isObject[i]==0) continue;
                                                cv::Mat temp;
                                                color.RGB2Mask(RGB, temp, i);
                                                temp.setTo(0, ~sphereMask);
                                                int numPixel = cv::countNonZero(temp);
                                                if (numPixel>maxPixel) {
                                                        maxPixel = numPixel;
                                                        maxIdx=i;
                                                }
                                        }
                                        if (maxIdx!=0) color.RGB2Mask(RGB, hsv, maxIdx);

                                        plaincode::getColorSphereBlob(hsv, sumPos3d, handMat, distMat, inHandMat, points, 50, 180);
                                }

                                else if (objects[graspedIdx].label>other_things.labelOffset) {
                                        plaincode::getSphereBlob(RGB, sumPos3d, handMat, distMat, inHandMat, points, 50, 180);
                                }

                                cv::Mat deleteMat = labelMat.clone(); // 이전결과.
                                deleteMat.setTo(0, labelMat==objects[graspedIdx].label);
                                cv::Mat diffDepth = (prevDepth-Depth)>30;
                                deleteMat.setTo(0, diffDepth);

                                inHandMat.setTo(0, deleteMat|handMat);
                                inHandMat.setTo(0, Depth>Depth.at<unsigned short>(handBlob.centerPos)+50);

                                std::vector<plaincode::Blob> localObjects;
                                plaincode::Blob localObject;
                                cv::Mat localLabelMat, prLocalLabelMat;
                                int rotated=0;
                                // objects[graspedIdx].label에 따라서, colored_things을 고를지 other_things를 고를지 정할 수 있다.
                                if (objects[graspedIdx].label<=other_things.labelOffset) {
                                        colored_things.clearBlob();
                                        colored_things.append(inHandMat, 30);
                                        colored_things.project(points, parameter, K, 0);
                                        colored_things.filter(underPlane);
                                        colored_things.accumulate(originRGB, inputMat);

                                        rotated = colored_things.lua_classify(lua, inputMat, outputLabel, 0.0001, objects[graspedIdx].label-colored_things.labelOffset);
                                        colored_things.reflect(inputMat, outputLabel);
                                        if (saveMode) colored_things.save(inputMat);
                                        localObjects.insert(localObjects.end(), colored_things.blobs.begin(), colored_things.blobs.end());
                                        localLabelMat = colored_things.labelMat.clone();
                                        prLocalLabelMat = colored_things.prLabelMat.clone();
                                }
                                else {
                                        other_things.clearBlob();
                                        other_things.append(inHandMat);
                                        other_things.project(points, parameter, K, 0);
                                        other_things.filter(underPlane);
                                        other_things.accumulate(originRGB, inputMat);

                                        rotated = other_things.lua_classify(lua, inputMat, outputLabel, 0.0001, objects[graspedIdx].label-other_things.labelOffset);
                                        other_things.reflect(inputMat, outputLabel);
                                        if (saveMode) other_things.save(inputMat);
                                        localObjects.insert(localObjects.end(), other_things.blobs.begin(), other_things.blobs.end());
                                        localLabelMat = other_things.labelMat.clone();
                                        prLocalLabelMat = other_things.prLabelMat.clone();
                                }

                                if (rotated && !isRotated) {
                                        /* LfD showing*/
                                        isHappened=true;
                                        drawMode=4; 
                                        eventString << "Pour " << objects[graspedIdx].name << " into " << objects[minDistIdx].name;
 
                                       std::fill(channel1.begin(), channel1.end(), 0.0); 
                                        channel1[3]=1; 

                                        std::fill(channel2.begin(), channel2.end(), 0.0); 
                                        channel2[objects[graspedIdx].label-1]=1; 

                                        std::fill(channel3.begin(), channel3.end(), 0.0); 
                                        channel3[objects[minDistIdx].label-1]=1; 

                                        channels1.push_back(channel1); 
                                        channels2.push_back(channel2); 
                                        channels3.push_back(channel3); 

                                        isRotated=true;
                                }
                                // localObjects.size()가 0이면 놓아버린 것.
                                if (localObjects.size()>0) {
                                        objects[graspedIdx] = localObjects[0];
                                        // prevLabel -> labelMat으로 복사.
                                        labelMat = prevLabelMat.clone();
                                        prLabelMat = prevPrLabelMat.clone();
                                        // label에서 기존 영역 제거
                                        labelMat.setTo(0, labelMat==objects[graspedIdx].label);
                                        prLabelMat.setTo(0, prLabelMat==objects[graspedIdx].label);

                                        // 새로운 영역 추가.
                                        labelMat.setTo(objects[graspedIdx].label, localLabelMat);
                                        prLabelMat.setTo(objects[graspedIdx].label, prLocalLabelMat);
                                }

                                double dist2hand = (handBlob.centerPos3d-objects[graspedIdx].centerPos3d).norm();
                                double dist2objects = (objects[minDistIdx].prCenterPos3d-objects[graspedIdx].prCenterPos3d).norm();

                                if (dist2hand>120) {
                                        /* LfD showing*/
                                        isHappened=true;
                                        drawMode=5; 
                                        if (dist2objects<50) {
                                                eventString << "Release " << objects[graspedIdx].name << " on " << objects[minDistIdx].name;

                                                std::fill(channel1.begin(), channel1.end(), 0.0); 
                                                channel1[4]=1; 

                                                std::fill(channel2.begin(), channel2.end(), 0.0); 
                                                channel2[objects[graspedIdx].label-1]=1; 
                                            
                                                std::fill(channel3.begin(), channel3.end(), 0.0); 
                                                channel3.back() = 1; 

                                                channels1.push_back(channel1); 
                                                channels2.push_back(channel2); 
                                                channels3.push_back(channel3); 


                                        }
                                        else {
                                                eventString << "Release " << objects[graspedIdx].name << " on the TABLE";
                                                // YHYOO
                                                std::fill(channel1.begin(), channel1.end(), 0.0); 
                                                channel1[4]=1; 

                                                std::fill(channel2.begin(), channel2.end(), 0.0); 
                                                channel2[objects[graspedIdx].label-1]=1; 

                                                std::fill(channel3.begin(), channel3.end(), 0.0); 
                                                channel3.back() = 1; 

                                                channels1.push_back(channel1); 
                                                channels2.push_back(channel2); 
                                                channels3.push_back(channel3); 
                                        }
                                        state=FadeOut;
                                        isRotated=false;
                                        trial++;
                                }

                        }
                        break; 
        }





        cv::Mat proj_for_objects;
        originRGB.copyTo(proj_for_objects);

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
                        point_pub.publish(recog_object);
                }
        }

        cv::imshow("Projected Image", proj_for_objects);

        cv::Mat result, prResult; 
        result = cv::Mat::zeros(RGB.size(), CV_8UC3); 
        prResult = cv::Mat::zeros(RGB.size(), CV_8UC3); 

        plaincode::paint(result, labelMat, objects, allLut); 
        plaincode::paint(prResult, prLabelMat, objects, allLut); 

        cv::addWeighted(RGB, 0.4, result, 0.6, 0.0, result);
        cv::addWeighted(RGB, 0.4, prResult, 0.6, 0.0, prResult);

        cv::Mat result_pcl; 
        result_pcl = result.clone(); 

        plaincode::putText(result, objects); 
        plaincode::putText(prResult, objects); 

        // draw principle line
        for (std::size_t i=0; i<objects.size(); i++) {
                // find point

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

        cv::Mat trajectoryMat = RGB.clone();  
        cv::addWeighted(trajectoryMat, 0.5, trajectoryMat, 0.0, 0.0, trajectoryMat); 


        if (drawTrajectory) {
            Eigen::MatrixXf derivedPath2d =(K*derivedPath.block(0,0,3,25)).transpose(); 
            derivedPath2d = derivedPath2d.cwiseQuotient(derivedPath2d.col(2).replicate(1,3)); 
//            std::cout << derivedPath2d << std::endl; 

          for (std::size_t i=0; i<24; i++) {
            cv::line(trajectoryMat, cv::Point(derivedPath2d(i,0),derivedPath2d(i,1)), cv::Point(derivedPath2d(i+1,0), derivedPath2d(i+1,1)), cv::Scalar(0,255,255), 5); 
            cv::circle(trajectoryMat, cv::Point(derivedPath2d(i,0),derivedPath2d(i,1)), 3, cv::Scalar(0,255,255), -1); 
          }
        }

        if (drawMode) {

                RGBs.push_back(RGB.clone()); 
                Depths.push_back(Depth); 
                Results.push_back(result); 
                contextAct.push_back(channels1.back()); 

                plaincode::Trajectory eachHandPath; 
                eachHandPath.code=drawMode; 
                eachHandPath.pos=handBlob.centerPos;
                if (eachHandPath.pos==cv::Point(0,0) && handPath.size()!=0) eachHandPath.pos=handPath.back().pos; 
                eachHandPath.pos3d=handBlob.centerPos3d; 
                handPath.push_back(eachHandPath); 

                if (thermalMode=="on") Thermals.push_back(Thermal); 


        }

        if (drawMode==5) {
  
                drawMode=0;  
                subDemoDir = plaincode::extractFilename(demonstrationDir); 

                cv::Mat temp = RGB.clone(); 
                cv::resize(RGB, temp, cv::Size(320, 240));  

                rgbDir = subDemoDir/"RGB";
                depthDir = subDemoDir/"Depth";
                recognitionDir = subDemoDir/"Recognition";
                trajectoryDir = subDemoDir/"Trajectory";

                if (thermalMode=="on") thermalDir = subDemoDir/"Thermal"; 

                boost::filesystem::create_directory(subDemoDir); 
                boost::filesystem::create_directory(rgbDir); 
                boost::filesystem::create_directory(depthDir); 
                boost::filesystem::create_directory(recognitionDir); 
                boost::filesystem::create_directory(trajectoryDir); 
                
                if (thermalMode=="on") boost::filesystem::create_directory(thermalDir); 


                for (std::size_t i=0; i<RGBs.size(); i++) {
                    cv::imwrite(plaincode::extractFilename(rgbDir, ".jpg"), RGBs[i]); 
                    plaincode::imwrite(plaincode::extractFilename(depthDir, ".jpg"), Depths[i], 1); 
                    cv::imwrite(plaincode::extractFilename(recognitionDir, ".jpg"), Results[i]); 
                    if (thermalMode=="on") plaincode::imwrite(plaincode::extractFilename(thermalDir, ".jpg"), Thermals[i], 1); 
                }

                Eigen::MatrixXf allHandPath(4, handPath.size()); 
                Eigen::MatrixXf matT(4,4); matT.setIdentity(4,4); 
                Eigen::MatrixXf matR(4,4); matR.setIdentity(4,4);   
                Eigen::MatrixXf matFlip(4,4); matFlip.setIdentity(4,4); 
                matFlip(1,1)=-1; 

                // translation matrix
//                matT.block(0,3,3,1) = -handPath[0].pos3d;
                matT.block(0,3,3,1) = -objectPos1; 
                matT(3,3)= 1.0; 

                // rotation matrix
                Eigen::Vector3f xAxis = objectPos2 - objectPos1; float scaleFactor = xAxis.norm(); xAxis = xAxis/scaleFactor; 
                Eigen::Vector3f zAxis(parameter); zAxis = zAxis/zAxis.norm();  
                Eigen::Vector3f yAxis = zAxis.cross(xAxis); 

                matR.block(1,0,1,3) = yAxis.transpose(); 
                matR.block(2,0,1,3) = xAxis.transpose(); 
                matR.block(0,0,1,3) = zAxis.transpose(); 

                for (std::size_t i=0; i<handPath.size(); i++) {
                        allHandPath(0,i)=handPath[i].pos3d.x(); 
                        allHandPath(1,i)=handPath[i].pos3d.y(); 
                        allHandPath(2,i)=handPath[i].pos3d.z(); 
                        allHandPath(3,i)=1.0; 
                }

                Eigen::MatrixXf normHandPath;
                normHandPath = 1/scaleFactor*matR*matT*allHandPath; 
                
                if (normHandPath(1,0)<0) {
                      normHandPath = matFlip*normHandPath; 
                }
        }

        cv::Mat onPlaneMat = cv::Mat::zeros(distMat.size(), distMat.type());
        distMat.copyTo(onPlaneMat, tuneMask);

        cv::moveWindow("RGB", 100, 100); cv::imshow("RGB", RGB);
        cv::moveWindow("output", 100+280+20, 100); cv::imshow("output", result); 
//        cv::moveWindow("prOutput", 100+280+280, 100); cv::imshow("prOutput", prResult); 
        cv::moveWindow("trajectory", 100+280+280+40, 100); cv::imshow("trajectory", trajectoryMat); 

        if (thermalMode=="on") { 
            cv::moveWindow("Thermal", 100, 400); plaincode::imshow("Thermal", Thermal, 1); 
        }

        plaincode::getPointCloud(result_pcl, points, RGBCloudPtr); 

        plaincode::updatePointCloud(RGBCloudViewer, RGBCloudPtr); 
        plaincode::updatePointCloud(RGBCloudViewer2, RGBCloudPtr); 
        
        if (drawTrajectory) {
        
          Eigen::VectorXf p1(3), p2(3); 
          for (std::size_t i=0; i<24; i++) {
            derivedPath(0,i);
            p1(0) = derivedPath(0,i); p1(1) = -derivedPath(1,i); p1(2) = derivedPath(2,i); 
            p2(0) = derivedPath(0,i+1); p2(1) = -derivedPath(1,i+1); p2(2) = derivedPath(2,i+1); 
            plaincode::updateLineCloud(RGBCloudViewer, p1, p2, cv::Scalar(0,255,255), i);             
            plaincode::updateLineCloud(RGBCloudViewer2, p1, p2, cv::Scalar(0,255,255), i);  
          }

          RGBCloudViewer->spinOnce(10); 
          RGBCloudViewer2->spinOnce(10); 
        }



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
        else if (key=='t') {
                tuneMode=!tuneMode;
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

                        if((thermalMode=="on"&&is_updated_thermal&&is_updated_depth&&!is_running) ||(thermalMode!="on"&&is_updated_depth&&!is_running) ){
                                is_running = true;
                                cv::resize(rawRGB, originRGB, cv::Size(320,240));
                                float image_scale = (float)originRGB.rows/(float)rawRGB.rows;
                                is_updated_rgb = true;


                                if (!colorTuned) 
                                        colorTuned=colorSetting(point_pub, pub_psw);  
                                else test(fx*image_scale, fy*image_scale, cx*image_scale, cy*image_scale, point_pub, pub_psw,true);
                                is_running = false;
                                is_updated_rgb = false;
                                is_updated_depth = false;
                                is_updated_thermal = false; 
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

void imageCallbackForThermal(const sensor_msgs::ImageConstPtr& msg) {
  try{
          cv::Mat rawThermal = cv_bridge::toCvShare(msg)->image; 

          //       originThermal = cv_bridge::toCvShare(msg, "mono16")->image; 
          if (!rawThermal.empty()) {
                  if (!is_running || !is_updated_thermal) {
                          if (rawThermal.type()==CV_32FC1) rawThermal.convertTo(rawThermal, CV_32FC1, 1000.0/1.0); 
                          originThermal = rawThermal.clone(); 
                          originThermal.convertTo(originThermal, CV_16UC1); 
                          is_updated_thermal=true; 
                  }
          }
  }
  catch(cv_bridge::Exception& e) {
  }
  catch (cv::Exception& e){
  }
}

void handleEvent(const std_msgs::StringConstPtr& msg){
        std::string event = msg->data;
        std::string check ("end");
        if(event.compare(check) != 0)
        {
                std::cout << event << std::endl;
                std_msgs::String result_msgs;
                result_msgs.data = "SUCCESS";
                pub_state.publish(result_msgs);
        }
        else
                publish_flag = 0;
}

int main(int argc, char** argv){
        ros::init(argc, argv, "active_lfd");
        ros::NodeHandle nh("~");
        ros::NodeHandle node_handle;
        ros::NodeHandle node_handle_event("~"); 

        lua = luaL_newstate();
        luaL_openlibs(lua);

        memory_txt.open("/home/yongho/catkin_ws/src/mybot2_new/2_memory/deep_art/file/cueList.txt", std::ios::in & std::ios::trunc);         
        plaincode::th::distAbove=0; // 실제 값은 -100
        plaincode::th::thSize=100;
        plaincode::th::thC=20;
        cv::Rect imgRect(20,20,280,200); 

        std::string lua_files, object_files, color_file, setup_files; 
        nh.getParam("lua_files", lua_files);
        nh.getParam("setup_files", setup_files); 
        nh.getParam("RNN_files", RNN_files); 
        nh.getParam("thermalMode", thermalMode); 
        
        std::string memory_module_name; 
        nh.getParam("memory_module", memory_module_name); 

        RGBDT_transmat_file = std::string(setup_files + "/RGBDT_transmat.xml"); 



        cv::FileStorage transmatFile(RGBDT_transmat_file, cv::FileStorage::READ); 
        transmatFile["RGBDT_transmat"] >> RGBDT_transmat; 
        transmatFile.release(); 
        
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


        for (std::size_t i=0; i<nameTags.size(); i++) {
            std::cout << i << ": " << nameTags[i] << std::endl; 
        }
        std::cout << std::endl; 
    
/*        for (std::size_t i=0; i<patched_things.subDir.size(); i++)
                patched_things.rename(patched_things.rootDir/patched_things.resultDir/patched_things.subDir[i], ext);
        lua_pushstring(lua, patched_things.rootDir.c_str()); lua_setglobal(lua, "class");
        luaL_dofile(lua, std::string(lua_files + "/4_loadModel.lua").c_str());*/

        for (std::size_t i=0; i<colored_things.subDir.size(); i++)
                colored_things.rename(colored_things.rootDir/colored_things.resultDir/colored_things.subDir[i], ext);
        lua_pushstring(lua, colored_things.rootDir.c_str()); lua_setglobal(lua, "class");
        std::cout << colored_things.rootDir.c_str() << std::endl; 
        luaL_dofile(lua, std::string(lua_files + "/4_loadModel.lua").c_str());

        for (std::size_t i=0; i<other_things.subDir.size(); i++)
                other_things.rename(other_things.rootDir/other_things.resultDir/other_things.subDir[i], ext);
        lua_pushstring(lua, other_things.rootDir.c_str()); lua_setglobal(lua, "class");
        luaL_dofile(lua, std::string(lua_files + "/4_loadModel.lua").c_str());

        RGBCloudPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 

         std::string rnnDir = RNN_files; 
        luaL_dofile(lua, (rnnDir+"/load.lua").c_str());
        std::cout << rnnDir << std::endl; 
       
        std::cout << "===============================" << std::endl; 
        std::cout << "Color setting" << std::endl; 
        std::cout << " a : Add new palette" << std::endl; 
        std::cout << " d : Delete palette" << std::endl; 
        std::cout << " v : View current palette" << std::endl; 
        std::cout << " q : Quit color setting" << std::endl; 
        std::cout << "===============================" << std::endl; 

        drawColor.push_back(cv::Scalar(0,255,255)); 
        drawColor.push_back(cv::Scalar(0,0,255));
        drawColor.push_back(cv::Scalar(0,255,0));
        drawColor.push_back(cv::Scalar(255,0,0));

        ros::Publisher pub_psw = node_handle.advertise<moveit_msgs::PlanningSceneWorld>("planning_scene_world", 1);
        ros::Publisher point_pub = nh.advertise<simple_recognition::RecogObject>("object_point",100);

        pub_state = nh.advertise<std_msgs::String>(memory_module_name+std::string("/state_command"), 100);

        pub_start = nh.advertise<std_msgs::String>(memory_module_name+std::string("/start_command"), 100);

        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub_for_depth = it.subscribe("/rgbd_receiver/depth_registered/image_raw", 1, imageCallbackForDepth);
        image_transport::Subscriber sub_for_thermal = it.subscribe("/thermal/image", 1, imageCallbackForThermal); 
        image_transport::CameraSubscriber sub_for_rgb = it.subscribeCamera("/rgbd_receiver/rgb/image_raw", 1, boost::bind(cameraCallbackForRGB, _1, _2,  point_pub, pub_psw));

        ros::Subscriber sub_event = node_handle_event.subscribe<std_msgs::String>(memory_module_name+std::string("/event"), 1, boost::bind(handleEvent, _1));       

        ros::spin();

        lua_close(lua);
        return 0;
}
