// STL Header
#include <iostream>
#include <numeric>
#include <limits>
#include <memory>
#include <mutex> 

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
#include <std_msgs/String.h>

// Eigen Header
#include <eigen3/Eigen/Dense>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

// plaincode header
#include "Core.hpp"
#include "Color.hpp"
#include "Object.hpp"
#include "patch.hpp"
#include "contact.hpp"
#include "Thermal.hpp"

// Boost Header
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <simple_recognition/RecogObject.h>
#include <simple_recognition/RecogObjectArray.h>

boost::mutex g_mutex; 

lua_State *lua;
plaincode::Object objects; 
plaincode::Color color;

std::vector<double> dist2hand; 
bool isSelected=false; 
int selectedID=0; 

std::string gObjName; 
std::size_t gObjectDetectCount = 0;
cv::Mat labelMat, prLabelMat; 
std::vector<plaincode::Blob> marker; 
std::vector<int> markerCount;
std::vector<std::vector<cv::Point>> markerCorner; 
ros::Publisher event_pub;

bool isRotated=false;

//std::vector<cv::Scalar> lut;
cv::Mat originRGB, originDepth, originThermal;
cv::Mat rawRGB, rawDepth;
cv::Mat RGBDT_transmat; 
std::string RGBDT_transmat_file; 

cv::Mat dilateMask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)); 

Eigen::VectorXf parameter = Eigen::VectorXf(3);

cv::Point object_center_point;
bool is_object_center_point_updated = false;
double objectProb = 0.99; 
double softMax_c = 1000; 

void cropImage(cv::Mat& image, cv::Mat& result) {
        cv::Rect rect = cv::Rect(20, 20, 280, 200);
        result = image(rect);
}

bool is_running = false;
bool thermal_mode = false; 
bool load_mode = false; 
bool save_mode = false; 

boost::filesystem::path demonstrationDir;
boost::filesystem::path subDemoDir;
boost::filesystem::path rgbDir;
boost::filesystem::path depthDir;
boost::filesystem::path thermalDir;

std::fstream fileGeometry;
std::fstream fileTrajectory; 
std::fstream fileSymbol; 

std::vector<Eigen::Vector3f> trajectory; 
std::vector<double> symbol; 
std::vector<double> geometry; 

int test(double fx, double fy, double cx, double cy, ros::Publisher& point_pub, ros::Publisher& pub_psw, bool saveMode=false){

        // PCL1. "Cloud", "Viewer" declaration
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBD_Cloud, recognition_Cloud, symbolic_Cloud, geometric_Cloud, thermal_Cloud, plane_Cloud, cand1_Cloud, cand2_Cloud;
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr geo1_Cloud, geo2_Cloud; 

        static boost::shared_ptr<pcl::visualization::PCLVisualizer> RGBD_Viewer, recognition_Viewer, symbolic_Viewer, geometric_Viewer, thermal_Viewer, plane_Viewer, cand1_Viewer, cand2_Viewer;
        static boost::shared_ptr<pcl::visualization::PCLVisualizer> geo1_Viewer, geo2_Viewer; 

        static boost::once_flag flag = BOOST_ONCE_INIT; 


		boost::call_once([]
						{
						cv::Size sz(300,400); 

						// PCL1-1) Create viewer
						plaincode::createViewer(recognition_Viewer, "recognition_Viewer"); 
						plaincode::createViewer(geometric_Viewer, "geometric_Viewer");
	//					plaincode::createViewer(geo1_Viewer, "geo1_Viewer");
	//					plaincode::createViewer(geo2_Viewer, "geo2_Viewer");

						if (thermal_mode) plaincode::createViewer(thermal_Viewer, "thermal_Viewer"); 
						plaincode::createViewer(RGBD_Viewer, "RGBD_Viewer");
						plaincode::createViewer(symbolic_Viewer, "symbolic_Viewer");
	//					plaincode::createViewer(plane_Viewer, "plane_Viewer"); 

	//					plaincode::createViewer(cand1_Viewer, "cand1_Viewer"); 
	//					plaincode::createViewer(cand2_Viewer, "cand2_Viewer"); 

						// PCL1-2) Set viewer
						plaincode::setViewer(RGBD_Viewer, cv::Point(100,300), sz, cv::Scalar(255,255,255)); 
						plaincode::setViewer(recognition_Viewer, cv::Point(500,300), sz, cv::Scalar(255,255,255)); 
						plaincode::setViewer(symbolic_Viewer, cv::Point(100,900), sz, cv::Scalar(255,255,255)); 
						plaincode::setViewer(geometric_Viewer, cv::Point(500,900), sz, cv::Scalar(255,255,255)); 

	//					plaincode::setViewer(geo1_Viewer, cv::Point(100,400), sz, cv::Scalar(255,255,255)); 
	//					plaincode::setViewer(geo2_Viewer, cv::Point(700,400), sz, cv::Scalar(255,255,255)); 

						if (thermal_mode) plaincode::setViewer(thermal_Viewer, cv::Point(700,600), sz, cv::Scalar(255,255,255)); 

	//					plaincode::setViewer(plane_Viewer, cv::Point(700,300), sz, cv::Scalar(255,255,255));

	//					plaincode::setViewer(cand1_Viewer, cv::Point(100,500), sz, cv::Scalar(255,255,255)); 
	//					plaincode::setViewer(cand2_Viewer, cv::Point(700,500), sz, cv::Scalar(255,255,255)); 


						// PCL1-3) create cloud
						RGBD_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 
						recognition_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 
						symbolic_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 
						geometric_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 

	//					geo1_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 
	//					geo2_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 

						if (thermal_mode) thermal_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 
//						plane_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 

	//					cand1_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 
	//					cand2_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 



						},flag); 



		ros::WallDuration sleep_time(0.05);
		ros::Time current_time = ros::Time::now();

		Eigen::MatrixXf K, Kinv;
		plaincode::setCameraMat(fx, fy, cx-20.0, cy-20.0, K, Kinv);

		static bool inHand = false;
		bool eventOn = false, approached = false; 
		static plaincode::Blob grabBlob; 
		static int grabIdx = -1, targetIdx=-1, patchIdx=-1; 
		static std::vector<std::string> eventStrings; 
		static Eigen::Vector3f prevPos = Eigen::Vector3f(-100,-100,-100); 
		std::ostringstream eventString;

		cv::Mat RGB, Depth, Thermal;  
		cv::Mat onPlane, prPlane, underPlane;
		cv::Mat candidateMat; 
		cv::Mat result, prResult; 
		cv::Mat result_pcl, prResult_pcl; 
		cv::Mat distMat;
		// cv::Mat Mask, Label;
		cv::Mat handMask;

		cv::Mat feature;
		Eigen::MatrixXf points;

        double time = cv::getTickCount();

        cropImage(originRGB,RGB);
        cropImage(originDepth, Depth);

        if (thermal_mode && !originThermal.empty()) {
                cv::Mat thermalTemp;

                cropImage(originThermal, thermalTemp);
                plaincode::getTransformed(RGB, Depth, thermalTemp, Thermal, RGBDT_transmat);
                double min,max;
                cv::minMaxLoc(originThermal, &min, &max);
                Thermal.setTo((unsigned short)min, Thermal<(unsigned short)8000);
        }

        /********************************************/
        plaincode::crossBilateralFilter(RGB, Depth);
        plaincode::getPoint3d(Depth, points, Kinv);
        plaincode::getDistMat(Depth, distMat, points, parameter);
        plaincode::removePlane(distMat, underPlane, plaincode::CONSTANT, 1200., 80.);
        color.RGB2Mask(RGB, handMask, 0);

        // temp
        cv::Mat temp = cv::Mat::zeros(RGB.size(), CV_8UC1); 
        temp.setTo(255, distMat>-25 & distMat<30);
        cv::imshow("temp", temp); 

        // find handBlob and remove the blob which # of pixel is lower than threshold
        handMask.setTo(0, distMat>-30);
        cv::dilate(handMask, handMask, dilateMask); 

        cv::Mat handIdMat; 
        std::vector<plaincode::Blob> handTempBlob; 
        plaincode::findBlob(handMask, handIdMat, handTempBlob, 225); 
        handMask = handIdMat!=0; 

        Eigen::MatrixXf handPoints; 

        plaincode::findSamplePoints(handMask, points, handPoints); 

        cv::Mat sphereMat = cv::Mat::zeros(RGB.size(), CV_8UC1); 
        cv::Mat sphereMat_near = cv::Mat::zeros(RGB.size(), CV_8UC1); 

        for (std::size_t i=0; i<marker.size(); i++) {

                plaincode::getPoint3d(marker[i].centerPos, Depth.at<unsigned short>(marker[i].centerPos), marker[i].centerPos3d, Kinv); 
                plaincode::getSphereMask(marker[i].centerPos3d, points, sphereMat, 100); 
                plaincode::getSphereMask(marker[i].centerPos3d, points, sphereMat_near, 100); 

                Eigen::MatrixXf allCorner(4,3); 
                for (std::size_t j=0; j<3; j++) {                    
                        Eigen::Vector3f eachCorner;
                        cv::Point centerPos; 
                        plaincode::getPoint3d(markerCorner[i][j], Depth.at<unsigned short>(markerCorner[i][j]), eachCorner, Kinv); 
                        allCorner.row(j) = eachCorner; 
                }

                // get patch normal
                Eigen::Vector3f vec1 = allCorner.row(0)-allCorner.row(1); 
                Eigen::Vector3f vec2 = allCorner.row(2)-allCorner.row(1); 
                Eigen::Vector3f p_normal = vec1.cross(vec2); 

                p_normal /= p_normal.norm(); 
                marker[i].centerPos3d -= p_normal*125; 
                plaincode::getSphereMask(marker[i].centerPos3d, points, sphereMat_near, 100); 
                Eigen::Vector3f n_normal = parameter/parameter.norm(); 
                marker[i].centerPos3d -= n_normal*100; 

        }


        for (std::size_t i=0; i<marker.size(); i++) {
                plaincode::getSphereMask(marker[i].centerPos3d, points, sphereMat, 100); 
        }

                // there is no object in hand
                if (inHand==false) {

                        std::vector<cv::Mat> inputMat; 
                        std::vector<int> outputLabel; 

                        if (!approached) {
                                // cv::imshow("sp", sphereMat); 
                                objects.clearBlobs(); 
                                objects.projectOnPlane(distMat, onPlane, prPlane, sphereMat|handMask, points, parameter, K, -20.); 
                                // sphereMat: 'a','b','c' patch mask
                                // color setting
                                objects.refine(RGB, onPlane, prPlane, points, parameter, K, color.hsvMin, color.hsvMax, color.isObject); 
                                objects.projectBlob(prPlane, 100); 
                                objects.reconstructBlob(onPlane, points, parameter, K); 

                                objects.filter(underPlane); 
                                objects.accumulate(originRGB, inputMat); 
                                objects.lua_classify(lua, inputMat, outputLabel, objectProb); 
                                objects.reflect(inputMat, outputLabel); 

                                if (saveMode) objects.save(inputMat); 
                                objects.paint(RGB, result, prResult); 
                                objects.paint(RGB, result_pcl, prResult_pcl, false); 

                                cv::Mat proj_for_objects;
                                originRGB.copyTo(proj_for_objects);

                                for (std::size_t i=0; i<objects.blobs.size(); i++) {
                                        cv::Point object_center = (objects.blobs[i].prCenterPos+cv::Point(20,20))*2; 
                                        cv::circle(proj_for_objects, object_center, 10, cv::Scalar(255,0,0),3);

                                        double distance = (objects.blobs[i].prCenterPos3d(2))/1000.0;

                                        if (distance >0.2) {
                                                geometry_msgs::Point object_position;

                                                object_position.x = objects.blobs[i].prCenterPos3d(0)/1000.0;
                                                object_position.y = objects.blobs[i].prCenterPos3d(1)/1000.0;
                                                object_position.z = objects.blobs[i].prCenterPos3d(2)/1000.0;

                                                geometry_msgs::PointStamped stamped_object_position;
                                                std_msgs::Header header_for_object;
                                                header_for_object.frame_id = std::string("/xtion_rgb_optical_frame");
                                                header_for_object.stamp = current_time;

                                                stamped_object_position.point = object_position;
                                                stamped_object_position.header = header_for_object;

                                                simple_recognition::RecogObject recog_object;

                                                recog_object.stamped_point = stamped_object_position;
                                                recog_object.object_name = objects.blobs[i].name;
                                                point_pub.publish(recog_object);
                                        }
                                }

                        }

                        for (std::size_t i=0; i<objects.blobs.size(); i++) {
                                int pc=0; 
                                double ratio = objects.blobs[i].eigenVal[0]/objects.blobs[i].eigenVal[1]; 
                                if (objects.blobs[i].eigenVal[0]<objects.blobs[i].eigenVal[1]) {
                                        pc=1;
                                        ratio = 1./ratio; 
                                }

                                if (ratio>=2.0) {
                                        cv::Point pts = objects.blobs[i].prCenterPos + cv::Point(objects.blobs[i].eigenVec[pc].x*objects.blobs[i].eigenVal[pc], objects.blobs[i].eigenVec[pc].y*objects.blobs[i].eigenVal[pc]); 
                                        cv::line(prResult, objects.blobs[i].prCenterPos, pts, cv::Scalar(255,255,255), 2); 
                                }
                                else {
                                        cv::circle(prResult, objects.blobs[i].prCenterPos, 5, cv::Scalar(255,255,255), 3); 
                                }
                        }

                        cv::Mat boundBoxRGB = RGB.clone(); 
                        for (std::size_t i=0; i<objects.blobs.size(); i++) 
                                cv::rectangle(boundBoxRGB, objects.blobs[i].rect, cv::Scalar(0,0,255), 3);

                        // PCL2. Get point cloud
                        plaincode::getPointCloud(RGB, points, RGBD_Cloud);  
                        plaincode::getPointCloud(result_pcl, points, recognition_Cloud);  
                        plaincode::getPointCloud(result_pcl, points, symbolic_Cloud);  
                        cv::Mat whiteRGB; 
                        whiteRGB = cv::Mat(RGB.size(), RGB.type()); 
                        whiteRGB.setTo(255); 
                        cv::addWeighted(RGB, 0.35, whiteRGB, 0.65, 0, whiteRGB); 

                        plaincode::getPointCloud(whiteRGB, points, geometric_Cloud);  
//                        plaincode::getPointCloud(whiteRGB, points, geo1_Cloud);  
//                        plaincode::getPointCloud(whiteRGB, points, geo2_Cloud);  

                        cv::Mat pRGB = whiteRGB.clone(); 
                        pRGB.setTo(cv::Scalar(0,0,180), temp); 
//                        plaincode::getPointCloud(pRGB, points, plane_Cloud);  
                     

                        if (thermal_mode) {
                                cv::Mat scaledThermal; 
                                plaincode::getScaledImage(Thermal, scaledThermal, 1); 
                                double min, max; 
                                cv::minMaxIdx(Thermal, &min, &max); 
                                scaledThermal.setTo(cv::Scalar(255,255,255), Thermal==min); 
                                whiteRGB.copyTo(scaledThermal, Thermal==min); 
                                plaincode::getPointCloud(scaledThermal, points, thermal_Cloud); 
                        }

                        // PCL3. Apply cloud to viewer
                        plaincode::applyPointCloud(RGBD_Viewer, RGBD_Cloud); 
                        plaincode::applyPointCloud(recognition_Viewer, recognition_Cloud); 
                        plaincode::applyPointCloud(symbolic_Viewer, symbolic_Cloud); 
                        plaincode::applyPointCloud(geometric_Viewer, geometric_Cloud); 
//                              plaincode::applyPointCloud(geo1_Viewer, geo1_Cloud); 
//                              plaincode::applyPointCloud(geo2_Viewer, geo2_Cloud); 

                        if (thermal_mode) plaincode::applyPointCloud(thermal_Viewer, thermal_Cloud); 
//                        plaincode::applyPointCloud(plane_Viewer, plane_Cloud); 



                        if (handPoints.rows()>0 && objects.blobs.size()>0) {
                                std::vector<int> count(objects.blobs.size(), 0); 
                                for (std::size_t i=0; i<objects.blobs.size(); i++) {
                                        count[i] = plaincode::countSphereMask(objects.blobs[i].centerPos3d, handPoints, 150); 
                                }

                                int maxCount = std::distance(count.begin(), std::max_element(count.begin(), count.end())); 
                                if (count[maxCount]>800) 
                                        grabIdx=maxCount;     
                                else if (count[maxCount]>200) approached=true; 
                                else approached=false; 
                        }


                        for (std::size_t i=0; i<objects.blobs.size(); i++) {
                                if (grabIdx!=i)
                                        plaincode::updateSphereCloud(symbolic_Viewer, i, objects.blobs[i].centerPos3d); 
                                else plaincode::updateSphereCloud(symbolic_Viewer, i, objects.blobs[i].centerPos3d, cv::Scalar(255,0,0)); 
                        }

                        for (std::size_t i=0; i<marker.size(); i++) {
                                plaincode::updateSphereCloud(symbolic_Viewer, 100+i, marker[i].centerPos3d, cv::Scalar(255,255,0));
                        }                

                        if (grabIdx!=-1) {
                                inHand=true; 
                                prevPos = objects.blobs[grabIdx].prCenterPos3d; 
                                eventString << "grasp " << objects.blobs[grabIdx].name;
                                eventStrings.clear(); 
                                eventStrings.push_back(eventString.str()); 

                                // make subDemoDir & rgb, depth, recognition, trajectory, segmented. 
                                // 1) declare directories
                               subDemoDir = plaincode::extractFilename(demonstrationDir); 
                               rgbDir = subDemoDir/"RGB";
                               depthDir = subDemoDir/"Depth"; 
                               if (thermal_mode) thermalDir = subDemoDir/"Thermal"; 
                               
                               // 2) make directories
                               boost::filesystem::create_directory(subDemoDir);
                               boost::filesystem::create_directory(rgbDir);
                               boost::filesystem::create_directory(depthDir);
                               if (thermal_mode) boost::filesystem::create_directory(thermalDir);

                               std::fill(geometry.begin(), geometry.end(), 0); 

                               if (fileTrajectory.is_open()) fileTrajectory.close(); 
                               fileTrajectory.open((subDemoDir/"Trajectory.txt").c_str(), std::ios::out);
    
                               std::fill(symbol.begin(), symbol.end(), 0); 

                               symbol[grabIdx]=1;
                               geometry[0] = objects.blobs[grabIdx].prCenterPos3d(0)/1000.;
                               geometry[1] = objects.blobs[grabIdx].prCenterPos3d(1)/1000.; 
                               geometry[2] = objects.blobs[grabIdx].prCenterPos3d(2)/1000.; 
                                
                               geometry[6] = parameter(0); 
                               geometry[7] = parameter(1); 
                               geometry[8] = parameter(2); 


                        }

                        candidateMat = objects.idMat.clone(); 
                        plaincode::getScaledImage(candidateMat, candidateMat, 1); 
                        candidateMat.setTo(cv::Scalar(0,0,0), objects.labelMat==0); 

//                        plaincode::getPointCloud(candidateMat, points, cand1_Cloud); 
//                        plaincode::getPointCloud(candidateMat, points, cand2_Cloud); 

//                        plaincode::applyPointCloud(cand1_Viewer, cand1_Cloud); 
//                        plaincode::applyPointCloud(cand2_Viewer, cand2_Cloud); 

//                        cv::moveWindow("BoundingBox", 100+280+20,100); cv::imshow("BoundingBox", boundBoxRGB); 

                }
                  
                // there is an object in hand
                else if (inHand==true) {
                        cv::Mat inHandMat; 

                        std::vector<int> count(objects.blobs.size(), 0); // check the number of pixels inside each object's sphere.
                        for (std::size_t i=0; i<objects.blobs.size(); i++) {
                                count[i] = plaincode::countSphereMask(objects.blobs[i].centerPos3d, handPoints, 100);
                        }

                        int maxCount = std::distance(count.begin(), std::max_element(count.begin(), count.end()));

                        Eigen::Vector3f handCenter=Eigen::Vector3f(-100,-100,-100); 
                        if (count[maxCount]!=0) {
                                plaincode::countSphereMask(objects.blobs[maxCount].centerPos3d, handPoints, handCenter, count[maxCount], 100);
                        }

                        // Add trajectory
                        if (handCenter!=Eigen::Vector3f(-100,-100,-100)) {
                                trajectory.push_back(handCenter);
                        }
                        else if (trajectory.size()!=0) {
                                trajectory.push_back(trajectory.back()); 
                        }
                        // save rgb, depth, thermal images
                        cv::imwrite(plaincode::extractFilename(rgbDir, ".jpg"), RGB); 
                        plaincode::imwrite(plaincode::extractFilename(depthDir, ".jpg"), Depth, 1);
                        if (thermal_mode) plaincode::imwrite(plaincode::extractFilename(thermalDir, ".jpg"), Thermal, 1); 

                        // save trajectories
                        // add trajectories to geometric information 
                        if (fileGeometry.is_open()) fileGeometry.close(); 
                        fileGeometry.open((subDemoDir/"Geometry.txt").c_str(), std::ios::out);
                        for (std::size_t i=0; i<geometry.size(); i++) fileGeometry << geometry[i] << "\t";
                        
                        if (trajectory.size()!=0) fileTrajectory << trajectory.back()(0)/1000. << "\t" << trajectory.back()(1)/1000. << "\t" << trajectory.back()(2)/1000. << std::endl; 
                        if (fileSymbol.is_open()) fileSymbol.close(); 
                        fileSymbol.open((subDemoDir/"Symbol.txt").c_str(), std::ios::out);
                        for (std::size_t i=0; i<symbol.size(); i++) fileSymbol << symbol[i] << "\t";


                        plaincode::getSphereBlob(RGB, objects.blobs[grabIdx].centerPos3d, handMask, distMat, inHandMat, points, 30, 150);

                        if (count[grabIdx]<15 || count[maxCount]>count[grabIdx]*1.5 || cv::countNonZero(inHandMat)==0) {
                                Eigen::Vector3f pos = objects.blobs[grabIdx].prCenterPos3d; 
                                float distance = (prevPos-pos).norm();
                                if (prevPos!=Eigen::Vector3f(-100,-100,-100)) std::cout << "distance = " << distance << std::endl; 
                                if (prevPos!=Eigen::Vector3f(-100,-100,-100) && distance<120) {
                                        eventString << "locate " << objects.blobs[grabIdx].name << " at " << objects.blobs[grabIdx].name; 
                                        prevPos = Eigen::Vector3f(-100,-100,-100); 

                                }
                                else { 
                                        eventString << "locate " << objects.blobs[grabIdx].name << " at " << "(" << (int)pos(0) << "," << (int)pos(1) << "," << (int)pos(2) << ")"; 
                                }
                                eventStrings.push_back(eventString.str()); 
                                eventOn=true; 
                                if (targetIdx==-1) eventOn=false; 
                                grabIdx=-1; targetIdx=-1; patchIdx=-1;   
                                trajectory.clear(); 
                                inHand=false; 
                                result = cv::Mat::zeros(RGB.size(), CV_8UC1);  
                                prResult = cv::Mat::zeros(RGB.size(), CV_8UC1); 
                                candidateMat = cv::Mat::zeros(RGB.size(), CV_8UC1); 
                        }

                        else {
                                cv::Mat newObjPoints; 
                                cv::findNonZero(inHandMat, newObjPoints); 
                                objects.blobs[grabIdx].rect = cv::boundingRect(newObjPoints);

                                cv::Mat tempLabelMat, tempPrLabelMat; 
                                candidateMat = objects.idMat.clone(); 
                                tempLabelMat = objects.labelMat.clone(); 
                                tempPrLabelMat = objects.prLabelMat.clone(); 

                                candidateMat.setTo(0, candidateMat==objects.blobs[grabIdx].id); 
                                candidateMat.setTo(objects.blobs[grabIdx].id, inHandMat); 

                                tempLabelMat.setTo(0, tempLabelMat==objects.blobs[grabIdx].label); 
                                inHandMat.setTo(0, (tempLabelMat!=objects.blobs[grabIdx].label)&(tempLabelMat)|(sphereMat_near)); 
                                tempLabelMat.setTo(objects.blobs[grabIdx].label, inHandMat); 

                                tempPrLabelMat.setTo(0, tempPrLabelMat==objects.blobs[grabIdx].label); 
                                //                    tempPrLabelMat.setTo(objects.blobs[grabIdx].label); 
                                prResult = tempPrLabelMat.clone(); 

                                // remove other objects in inHandMat
//                                cv::imshow("inHandMat", inHandMat); 

                                // find new grabbed object's center
                                Eigen::MatrixXf newPoints; 
                                plaincode::findSamplePoints(inHandMat, points, newPoints);
                                objects.blobs[grabIdx].centerPos3d = newPoints.colwise().mean(); 

                                Eigen::MatrixXf distM=(newPoints*parameter).array()-1; 
                                distM=distM.array()/parameter.norm(); 
                                newPoints-=(distM*parameter.transpose()/parameter.norm()); 
                                objects.blobs[grabIdx].prCenterPos3d = newPoints.colwise().mean(); 
                                newPoints = newPoints.cwiseQuotient(newPoints.col(2).replicate(1,3)); 
                                Eigen::MatrixXf prIdx = newPoints*K.transpose(); 

                                for (int i=0; i<prIdx.rows(); i++) {
                                        int col = prIdx(i,0), row = prIdx(i,1); 
                                        if (col>=0 && col<RGB.cols && row>=0 && row<RGB.rows) {
                                                int index=row*RGB.cols+col; 
                                                tempPrLabelMat.data[index]=objects.blobs[grabIdx].label; 
                                        }
                                }

                                // find hand center
                                objects.blobs[grabIdx].centerPos3d; 
                                cv::Mat handCandidate = cv::Mat::zeros(RGB.size(), CV_8UC1); 
                                plaincode::getSphereMask(objects.blobs[grabIdx].centerPos3d, points, handCandidate, 100); 
                                handCandidate.setTo(0, ~handMask); 
//                                cv::imshow("handCandidate", handCandidate); 


                                if (targetIdx==-1) {

                                        // find near object from the grabbed object 
                                        std::vector<int> objCount(objects.blobs.size(), 0);   
                                        for (std::size_t i=0; i<objects.blobs.size(); i++) {
                                                if (i!=grabIdx) {
                                                        cv::Mat eachObjMask = objects.blobs[i].id==objects.idMat; 
                                                        //                            cv::Mat eachObjMask = objects.blobs[i].id==candidateMat; 

                                                        Eigen::MatrixXf eachObjPoints; 
                                                        plaincode::findSamplePoints(eachObjMask, points, eachObjPoints); 
                                                        objCount[i] = plaincode::countSphereMask(objects.blobs[grabIdx].centerPos3d, eachObjPoints, 50); 
                                                }
                                        }

                                        int maxCount = std::distance(objCount.begin(), std::max_element(objCount.begin(), objCount.end())); 
                                        if (objCount[maxCount]>15) {
                                                targetIdx=maxCount; 
                                                eventString << "pour " << objects.blobs[grabIdx].name << " to " << objects.blobs[targetIdx].name; 
                                                eventStrings.back() = eventString.str(); 
                                                symbol[targetIdx]=1; 
                                                geometry[3] = objects.blobs[targetIdx].prCenterPos3d(0)/1000.; 
                                                geometry[4] = objects.blobs[targetIdx].prCenterPos3d(1)/1000.; 
                                                geometry[5] = objects.blobs[targetIdx].prCenterPos3d(2)/1000.; 
                                        }

                                        else if (marker.size()>0) {
                                                std::vector<int> patchCount(marker.size(), 0);
                                                cv::Mat grabObjMask = inHandMat; 
                                                Eigen::MatrixXf eachPatchPoints; 
                                                plaincode::findSamplePoints(grabObjMask, points, eachPatchPoints); 

                                                for (std::size_t i=0; i<marker.size(); i++) {
                                                        patchCount[i] = plaincode::countSphereMask(marker[i].centerPos3d, eachPatchPoints, 50); 
                                                }

                                                int maxPatchCount = std::distance(patchCount.begin(), std::max_element(patchCount.begin(), patchCount.end())); 
                                                if (patchCount[maxPatchCount]>10) {
                                                        patchIdx=maxPatchCount; 
                                                        eventString << "throw " << objects.blobs[grabIdx].name << " to " << marker[patchIdx].name; 
                                                        eventStrings.push_back(eventString.str()); 
                                                        inHand=false; eventOn=true; 
                                                        trajectory.clear(); 
                                                        grabIdx=-1; 
                                                        targetIdx=-1; 
                                                }
                                        }

                                }

                                objects.paint(RGB, result, prResult, tempLabelMat, tempPrLabelMat); 
                                objects.paint(RGB, result_pcl, prResult_pcl, tempLabelMat, tempPrLabelMat, false);

                                plaincode::getScaledImage(candidateMat, candidateMat, 1); 

                                // PCL2. Get point cloud
                                plaincode::getPointCloud(RGB, points, RGBD_Cloud);  
                                plaincode::getPointCloud(result_pcl, points, recognition_Cloud); 
                                plaincode::getPointCloud(result_pcl, points, symbolic_Cloud); 
                                cv::Mat whiteRGB; 
                                whiteRGB = cv::Mat(RGB.size(), RGB.type()); 
                                whiteRGB.setTo(255); 
                                cv::addWeighted(RGB, 0.35, whiteRGB, 0.65, 0, whiteRGB); 
                                plaincode::getPointCloud(whiteRGB, points, geometric_Cloud); 
//                                plaincode::getPointCloud(whiteRGB, points, geo1_Cloud); 
//                                plaincode::getPointCloud(whiteRGB, points, geo2_Cloud); 
                                cv::Mat pRGB = whiteRGB.clone(); 
                                pRGB.setTo(cv::Scalar(0,0,180), temp); 
//                                plaincode::getPointCloud(pRGB, points, plane_Cloud);  


                                if (thermal_mode) {
                                        cv::Mat scaledThermal; 
                                        plaincode::getScaledImage(Thermal, scaledThermal, 1); 
                                        double min, max; 
                                        cv::minMaxIdx(Thermal, &min, &max); 
                                        scaledThermal.setTo(cv::Scalar(255,255,255), Thermal==min); 
                                        whiteRGB.copyTo(scaledThermal, Thermal==min); 
                                        plaincode::getPointCloud(scaledThermal, points, thermal_Cloud); 
                                }

                        candidateMat.setTo(cv::Scalar(0,0,0), objects.labelMat==0); 

//                        plaincode::getPointCloud(candidateMat, points, cand1_Cloud); 
//                        plaincode::getPointCloud(candidateMat, points, cand2_Cloud); 

//                        plaincode::applyPointCloud(cand1_Viewer, cand1_Cloud); 
//                        plaincode::applyPointCloud(cand2_Viewer, cand2_Cloud); 


                                // PCL3. Apply cloud to viewer
                                plaincode::applyPointCloud(RGBD_Viewer, RGBD_Cloud); 
                                plaincode::applyPointCloud(recognition_Viewer, recognition_Cloud); 
                                plaincode::applyPointCloud(symbolic_Viewer, symbolic_Cloud);
                                plaincode::applyPointCloud(geometric_Viewer, geometric_Cloud); 
//                                plaincode::applyPointCloud(geo1_Viewer, geo1_Cloud); 
//                                plaincode::applyPointCloud(geo2_Viewer, geo2_Cloud); 
//                                plaincode::applyPointCloud(plane_Viewer, plane_Cloud); 


                                if (thermal_mode) plaincode::applyPointCloud(thermal_Viewer, thermal_Cloud); 

                                for (std::size_t i=0; i<objects.blobs.size(); i++) {
                                        if (i!=grabIdx && i!=targetIdx)
                                                plaincode::updateSphereCloud(symbolic_Viewer, i, objects.blobs[i].centerPos3d); 
                                        else if (i==grabIdx)  plaincode::updateSphereCloud(symbolic_Viewer, i, objects.blobs[i].centerPos3d, cv::Scalar(255,0,0)); 
                                        else plaincode::updateSphereCloud(symbolic_Viewer, i, objects.blobs[i].centerPos3d, cv::Scalar(0,0,255)); 
                                }

                                for (std::size_t i=0; i<marker.size(); i++) 
                                        plaincode::updateSphereCloud(symbolic_Viewer, 100+i, marker[i].centerPos3d, cv::Scalar(255,255,0));

                        }
                }

                for (std::size_t i=0; i<marker.size(); i++) {
                        cv::circle(result, marker[i].centerPos, 10, cv::Scalar(255,255,0), 3); 
                        cv::circle(prResult, marker[i].centerPos, 10, cv::Scalar(255,255,0), 3); 
                }

                cv::Mat eventMat = RGB.clone();
                cv::Mat message = cv::Mat::zeros(cv::Size(250,160), CV_8UC3); 
                message.setTo(cv::Scalar(0,200,180)); 
                cv::addWeighted(RGB(cv::Rect(15,20,250,160)), 0.15, message, 0.85, 0.0, message); 
                message.copyTo(eventMat(cv::Rect(15,20,250, 160)));

                for (std::size_t i=0; i<eventStrings.size(); i++) {
                        cv::Scalar color = cv::Scalar(0,0,0); 
                        if (i==eventStrings.size()-1) color = cv::Scalar(0,0,255); 
                        cv::putText(eventMat, std::to_string(i+1)+". " + eventStrings[i], cv::Point(20,20+(i+1)*30), cv::FONT_ITALIC, 0.4, color, 2, 8); 
                        std_msgs::String eventMsg;
                        eventMsg.data = eventStrings[i].c_str(); 
                        if (eventOn==true) event_pub.publish(eventMsg); 
                }

                /*
                   cv::moveWindow("RGB", 100, 100); cv::imshow("RGB", RGB);
                   cv::moveWindow("Depth", 100, 340); plaincode::imshow("Depth", Depth,1); 
                   cv::moveWindow("Output", 100+280+280+40, 100); cv::imshow("Output", result); 
                   cv::moveWindow("Projection", 100+280+280+40, 340); cv::imshow("Projection", prResult); */
//                   cv::moveWindow("Candidate", 100+280+20, 340); cv::imshow("Candidate", candidateMat); 
/*                  cv::moveWindow("Event", 100+280+280+280+60, 100); cv::imshow("Event", eventMat); 
                 */
                if (thermal_mode) {
                       //         cv::moveWindow("Thermal", 100+280+280+280+60, 340); plaincode::imshow("Thermal", Thermal, 1); 
                }


                Eigen::Vector3f p1, p2; 

                for (int i=0; i<(int)trajectory.size()-1; i++) {
                        plaincode::updateLineCloud(geometric_Viewer, trajectory[i], trajectory[i+1], cv::Scalar(0,0,225), i);
//                      plaincode::updateLineCloud(geo1_Viewer, trajectory[i], trajectory[i+1], cv::Scalar(0,0,225), i);
//                      plaincode::updateLineCloud(geo2_Viewer, trajectory[i], trajectory[i+1], cv::Scalar(0,0,225), i);

                }

                // PCL4. spin viewer
                recognition_Viewer->spinOnce(10, true); 
                symbolic_Viewer->spinOnce(10, true); 
                geometric_Viewer->spinOnce(10, true); 
//                geo1_Viewer->spinOnce(10, true); 
//                geo2_Viewer->spinOnce(10, true); 
                RGBD_Viewer->spinOnce(10, true); 
//                plane_Viewer->spinOnce(10, true); 
//                cand1_Viewer->spinOnce(10, true); 
//                cand2_Viewer->spinOnce(10, true); 


                if (thermal_mode) thermal_Viewer->spinOnce(10, true); 

                // PCL. set camera view point
                std::vector<pcl::visualization::Camera> cam; 
                RGBD_Viewer->getCameras(cam); 
                recognition_Viewer->setCameraParameters(cam[0]); 
                symbolic_Viewer->setCameraParameters(cam[0]); 
                geometric_Viewer->setCameraParameters(cam[0]); 
                if (thermal_mode) thermal_Viewer->setCameraParameters(cam[0]); 
//                plane_Viewer->setCameraParameters(cam[0]); 
//                cand1_Viewer->setCameraParameters(cam[0]); 

                //*******************************************************//
                time = ((double)cv::getTickCount()-time)/cv::getTickFrequency();

                //std::cout << "time = " << time << std::endl;
                char key = cv::waitKey(1);
                if (key=='q') 
                        exit(0);
                else if (key=='p') {
                        std::cout << "Current probability: " << objectProb << std::endl; 
                        std::cout << "Type new probability: "; 
                        std::cin >> objectProb; 
                        // load model, and change the constant, then save. 
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
bool is_updated_thermal = false; 

void tempCallback(const simple_recognition::RecogObjectArrayConstPtr& msg) {

        g_mutex.lock(); 

        for (std::size_t i=0; i<markerCount.size(); i++) markerCount[i]--;           

        for (int i=0; i<msg->instances.size(); i++){
                const simple_recognition::RecogObject &data = msg->instances[i]; 
                plaincode::Blob eachMarker; 
                std::vector<cv::Point> eachCorner(4); 

                for (std::size_t j=0; j<3; j++) {
                  eachCorner[j]= cv::Point(data.pos_x[j+1]-20, data.pos_y[j+1]-20); 
                }

                eachMarker.centerPos = cv::Point(data.pos_x[0]-20, data.pos_y[0]-20); 
                eachMarker.name = data.object_name; 

                bool isOverlap=false; 
                for (std::size_t j=0; j<marker.size(); j++) {
                    if (marker[j].name==eachMarker.name) {
                        markerCount[j]=3; 
                        marker[j].centerPos = eachMarker.centerPos; 
                        markerCorner[j] = eachCorner; 
                        isOverlap=true; 
                        break; 
                    }
                }
                
                if (isOverlap==false) {
                  marker.push_back(eachMarker); 
                  markerCount.push_back(3); 
                  markerCorner.push_back(eachCorner); 
                }
        }

        for (std::size_t i=0; i<marker.size(); i++) {
          if (markerCount[i]<=0 || marker[i].name=="") {
              marker.erase(marker.begin()+i, marker.begin()+i+1); 
              markerCount.erase(markerCount.begin()+i, markerCount.begin()+i+1); 
              markerCorner.erase(markerCorner.begin()+i, markerCorner.begin()+i+1); 
              i--; 
          }
        }

        g_mutex.unlock(); 
        return; 
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
//            if( is_updated_depth && !is_running){
       if((thermal_mode&&is_updated_thermal&&is_updated_depth&&!is_running) ||(!thermal_mode&&is_updated_depth&&!is_running) ){


                is_running = true;
                cv::resize(rawRGB, originRGB, cv::Size(320,240));
                float image_scale = (float)originRGB.rows/(float)rawRGB.rows;
                is_updated_rgb = true;

                if (!colorTuned) 
                    colorTuned=colorSetting(point_pub, pub_psw);  
                else {
                    test(fx*image_scale, fy*image_scale, cx*image_scale, cy*image_scale, point_pub, pub_psw, true);
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

void imageCallbackForThermal(const sensor_msgs::ImageConstPtr& msg) {
        try{
                cv::Mat rawThermal = cv_bridge::toCvShare(msg, "mono16")->image;

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


void normalCallback(const geometry_msgs::Vector3::ConstPtr& normalMsg) {
        parameter(0) = normalMsg->x;
        parameter(1) = normalMsg->y;
        parameter(2) = normalMsg->z;
}


int main(int argc, char** argv){
        ros::init(argc, argv, "recognition");
        ros::NodeHandle nh("~");
        ros::NodeHandle node_handle;

        //ros::AsyncSpinner spinner(2);
        //spinner.start();

        std::string ext = "_result.jpg";
        lua = luaL_newstate();
        luaL_openlibs(lua);

        plaincode::th::distAbove=0; // 실제 값은 -100
        plaincode::th::thSize=100;
        plaincode::th::thC=20;
        cv::Rect imgRect(20,20,280,200); 

        std::string lua_files, object_files, color_file, setup_files, parameter_files, demonstration_files; 

        nh.getParam("lua_files", lua_files); // lua
        nh.getParam("setup_files", setup_files); // objects
        nh.getParam("demonstration_files", demonstration_files); // demonstration
        nh.getParam("parameter_files", parameter_files); //?
        nh.getParam("thermal_mode", thermal_mode); // 

        nh.getParam("loadMode", load_mode); 
        nh.getParam("saveMode", save_mode); 

        if (setup_files[0]=='~') setup_files.replace(0,1,getenv("HOME")); 
        if (demonstration_files[0]=='~') demonstration_files.replace(0,1,getenv("HOME")); 
        demonstration_files.pop_back(); 

        demonstrationDir = boost::filesystem::path(demonstration_files);
        boost::filesystem::create_directories(demonstrationDir); 

        RGBDT_transmat_file = std::string(parameter_files + "/RGBDT_transmat.xml"); 
        cv::FileStorage transmatFile(RGBDT_transmat_file, cv::FileStorage::READ); 
        transmatFile["RGBDT_transmat"] >> RGBDT_transmat; 
        transmatFile.release(); 

        color.setFile(std::string(setup_files+"/palette"));
        luaL_dofile(lua, std::string(lua_files + "/0_parameter.lua").c_str());

        // other_things setting // 
        objects.setup(lua, lua_files, setup_files, imgRect); 
        boost::filesystem::path objectSaveDir =  objects.rootDir/objects.rawDir; 
           
        symbol.resize(objects.subDir.size()); 
        geometry.resize(9); // object pos1, object pos2, normal

        for (std::size_t i=0; i<objects.subDir.size(); i++)
                plaincode::rename(objects.rootDir/objects.resultDir/objects.subDir[i], "_result.jpg");  
        lua_pushstring(lua, objects.rootDir.c_str()); lua_setglobal(lua, "class");
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
        ros::Subscriber normal_sub = nh.subscribe("/simple_recognition/normal", 1, normalCallback);         

        event_pub = nh.advertise<std_msgs::String>("event",100);

        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub_for_depth = it.subscribe("/rgbd_receiver/depth_registered/image_raw", 1, imageCallbackForDepth);
        image_transport::Subscriber sub_for_thermal = it.subscribe("/thermal/image", 1, imageCallbackForThermal); 
        image_transport::CameraSubscriber sub_for_rgb = it.subscribeCamera("/rgbd_receiver/rgb/image_raw", 1, boost::bind(cameraCallbackForRGB, _1, _2,  point_pub, pub_psw));

        ros::Subscriber sub_point = nh.subscribe<simple_recognition::RecogObjectArray>("/recognition/object_points", 1, tempCallback);
        //ros::waitForShutdown();
        ros::spin();

        lua_close(lua);
        return 0;
}
