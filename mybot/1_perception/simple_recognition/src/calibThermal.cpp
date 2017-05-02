// STL Header
#include <iostream>
#include <fstream>
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
#include "Thermal.hpp"
#include "contact.hpp"

// Boost Header
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <simple_recognition/RecogObject.h>

cv::Mat originRGB, originDepth, originThermal; 
cv::Mat colorMask, heatMask, extracted; 
cv::Mat mColorPoints, mHeatPoints, pColorPoints, pHeatPoints; 
cv::Mat transmat = (cv::Mat_<float>(3,3) << 1,0,0,0,1,0,0,0,1); 

std::string RGBDT_transmat_file; 

bool is_updated_rgb = false; 
bool is_updated_depth = false; 
bool created_trackbar = false; 
bool isMatched = false; 

cv::Scalar_<int> hsvMin(0,0,0), hsvMax(255,255,255); 
int heatMin(0), heatMax(1000); 
int heatOffset=7500; 

char code; 

void cropImage(cv::Mat& image, cv::Mat& result, cv::Rect imgRect) {
        result = image(imgRect);
}

void tune(cv::Mat& originRGB, cv::Mat& originDepth, cv::Mat& originThermal) {

        cv::Mat RGB, Depth, Thermal; 
        cropImage(originRGB, RGB, cv::Rect(20,20,280,200)); 
        cropImage(originDepth, Depth, cv::Rect(20,20,280,200)); 
        cropImage(originThermal, Thermal, cv::Rect(20,20,280,200)); 

        plaincode::Color::RGB2Mask(RGB, colorMask, hsvMin, hsvMax); 

        cv::medianBlur(colorMask, colorMask, 3); 
        heatMask = (Thermal>(heatOffset+heatMin))&(Thermal<(heatOffset+heatMax)); 

        // Plane detection
        cv::imshow("RGB", RGB);  cv::moveWindow("RGB", 100, 100); 
        plaincode::imshow("THERMAL", Thermal, 1); cv::moveWindow("THERMAL", 400, 100); 
        cv::imshow("ColorMask", colorMask); cv::moveWindow("ColorMask", 100, 400); 
        cv::imshow("heatMask", heatMask); cv::moveWindow("heatMask", 400, 400); 
        char key = cv::waitKey(1); 
        if (key=='q') {
                code='t'; 
                cv::destroyAllWindows(); 
        }
}

void calibration(cv::Mat& originRGB, cv::Mat& originDepth, cv::Mat& originThermal) {

        cv::Mat RGB, Depth, Thermal; 
        cv::Mat colorResult, heatResult; 

        cropImage(originRGB, RGB, cv::Rect(20,20,280,200)); 
        cropImage(originDepth, Depth, cv::Rect(20,20,280,200)); 
        cropImage(originThermal, Thermal, cv::Rect(20,20,280,200)); 

        plaincode::Color::RGB2Mask(RGB, colorMask, hsvMin, hsvMax); 

        cv::medianBlur(colorMask, colorMask, 3); 
        heatMask = (Thermal>(heatOffset+heatMin))&(Thermal<(heatOffset+heatMax)); 

        std::vector<std::vector<cv::Point>> colorContours, heatContours;
        std::vector<cv::Point> colorPoints, heatPoints;
        plaincode::findPoints(colorMask, colorContours, colorPoints, colorResult);
        plaincode::findPoints(heatMask, heatContours, heatPoints, heatResult);

        if (colorPoints.size()==heatPoints.size()) {
                // color image와, heat image에서 sample된 point의 개수가 동일할 경우, data에 차례대로 등록한다.
                for (std::size_t i=0; i<colorPoints.size(); i++) {
                        // 해당 point의 depth값을 받아와야 한다.
                        float z = (float)(Depth.at<unsigned short>(colorPoints[i].y, colorPoints[i].x));
                        cv::Mat mColorPoint = (cv::Mat_<float>(1,3)<<(float)colorPoints[i].x*z, (float)colorPoints[i].y*z, z); // x Depth
                        mColorPoints.push_back(mColorPoint);

                        cv::Mat heatPoint = (cv::Mat_<float>(1,3)<<(float)heatPoints[i].x*z, (float)heatPoints[i].y*z, z); // x Depth
                        mHeatPoints.push_back(heatPoint);
                }

                if (mColorPoints.rows>100) {
                        std::vector<int> inlierIdx;
                        plaincode::randomSample(mColorPoints, mHeatPoints, inlierIdx, transmat);
                        isMatched=true;
                        // inlierIdx에 해당하는 mColorPoints와 mHeatPoints를 골라준다.
                        plaincode::deleteOutlier(mColorPoints, mHeatPoints, pColorPoints, pHeatPoints, inlierIdx);

                }
        }

        if (isMatched) {

                plaincode::getTransformed(RGB, Depth, Thermal, extracted, transmat); // get the transformed image. 

                double min, max;
                cv::minMaxLoc(Thermal, &min, &max);
                cv::Mat beforeFusion, afterFusion;
                cv::Mat Thermal2 = Thermal.clone();

                extracted.setTo((unsigned short)min, extracted<(unsigned short)8015);
                Thermal2.setTo((unsigned short)min, Thermal2<(unsigned short)8015);

                plaincode::getScaledImage(Thermal2, beforeFusion, 1);
                plaincode::getScaledImage(extracted, afterFusion, 1);

                cv::addWeighted(RGB, 0.6, beforeFusion, 0.4, 0.1, beforeFusion);
                cv::addWeighted(RGB, 0.6, afterFusion, 0.4, 0.1, afterFusion);

                plaincode::imshow("extracted", extracted, 1);
                cv::moveWindow("Before", 400, 800); cv::imshow("Before", beforeFusion);
                cv::moveWindow("weighted", 700, 800); cv::imshow("weighted", afterFusion);
        }

        cv::imshow("RGB", RGB); cv::moveWindow("RGB", 100, 50); 
        plaincode::imshow("Depth", Depth, 1); cv::moveWindow("Depth", 400, 50);
        plaincode::imshow("Thermal", Thermal, 1); cv::moveWindow("Thermal", 700, 50);
        cv::imshow("ColorMask", colorMask); cv::moveWindow("ColorMask", 400,300);
        cv::imshow("HeatMask", heatMask); cv::moveWindow("HeatMask", 700, 300);
        cv::imshow("colorResult", colorResult); cv::moveWindow("colorResult", 400, 550);
        cv::imshow("heatResult", heatResult); cv::moveWindow("heatResult",700, 550);

        char key  = cv::waitKey(1); 
        if (key=='q') { 
                std::fstream fp;
                fp.open(RGBDT_transmat_file, std::ios::out); // just create in case that the file doesn't exist. 
                fp.close(); 
                cv::FileStorage transmatFile(RGBDT_transmat_file, cv::FileStorage::WRITE); 
                transmatFile << "RGBDT_transmat" << transmat; 
                transmatFile.release(); 
                exit(0); 
        }

}

void display(cv::Mat& originRGB, cv::Mat& originDepth, cv::Mat& originThermal) {

        cv::Mat RGB, Depth, Thermal; 
        cv::Mat colorResult, heatResult; 

        cropImage(originRGB, RGB, cv::Rect(20,20,280,200)); 
        cropImage(originDepth, Depth, cv::Rect(20,20,280,200)); 
        cropImage(originThermal, Thermal, cv::Rect(20,20,280,200)); 
        
        plaincode::getTransformed(RGB, Depth, Thermal, extracted, transmat); // transformed된 image를 얻어온다.

        double min, max;
        cv::minMaxLoc(Thermal, &min, &max);
        cv::Mat Thermal2 = Thermal.clone();

        extracted.setTo((unsigned short)min, extracted<(unsigned short)8015);
        cv::imshow("RGB", RGB); 
        plaincode::imshow("extracted", extracted, 1);
        char key = cv::waitKey(1); 
        if (key=='q') exit(0); 


}


void imageCallbackForThermal(const sensor_msgs::ImageConstPtr& msg){
        try{
                originThermal = cv_bridge::toCvShare(msg, "mono16")->image; 
                if (!originThermal.empty() && is_updated_rgb && is_updated_depth) { 

                        if (code=='c') {
                                if (!created_trackbar) {
                                        cv::namedWindow("ColorSetting", 1); cv::moveWindow("ColorSetting", 100, 700);
                                        cv::createTrackbar("Hue(Min)", "ColorSetting", &hsvMin.val[0], 179);
                                        cv::createTrackbar("Hue(Max)", "ColorSetting", &hsvMax.val[0], 179);
                                        cv::createTrackbar("Sat(Min)", "ColorSetting", &hsvMin.val[1], 255);
                                        cv::createTrackbar("Sat(Max)", "ColorSetting", &hsvMax.val[1], 255);
                                        cv::createTrackbar("Val(Min)", "ColorSetting", &hsvMin.val[2], 255);
                                        cv::createTrackbar("Val(Max)", "ColorSetting", &hsvMax.val[2], 255);
                                        cv::createTrackbar("Heat(Min)", "ColorSetting", &heatMin, 1500);
                                        cv::createTrackbar("Heat(Max)", "ColorSetting", &heatMax, 1500);
                                        created_trackbar=true; 

                                }
                                else tune(originRGB, originDepth, originThermal); 
                        }
                        else if (code=='t') {
                          calibration(originRGB, originDepth, originThermal); 

                        }

                        else {
                          display(originRGB, originDepth, originThermal); 
                        }

                        is_updated_rgb = false; 
                        is_updated_depth = false; 

                        char key = cv::waitKey(1); 
                        if (key=='q') exit(0); 
                }
        }
        catch (cv_bridge::Exception& e) {
        }
        catch (cv::Exception& e) {
        }
}

void imageCallbackForRGB(const sensor_msgs::ImageConstPtr& msg) {
  
  try {
    cv::Mat rawRGB = cv_bridge::toCvShare(msg, "bgr8")->image; 
    if(!rawRGB.empty()){
        originRGB = rawRGB.clone(); 
        if (rawRGB.size()!=cv::Size(320,240)) cv::resize(originRGB, originRGB, cv::Size(320,240)); 
        is_updated_rgb = true; 
    }
  }
  catch (cv_bridge::Exception& e) {
  }
  catch (cv::Exception& e) {
  }
}

void imageCallbackForDepth(const sensor_msgs::ImageConstPtr& msg) {
  
  try {
    cv::Mat rawDepth = cv_bridge::toCvShare(msg)->image; 
    if(!rawDepth.empty()){
        if (rawDepth.type()==CV_32FC1) rawDepth.convertTo(rawDepth, CV_32FC1, 1000.0/1.0); 
        originDepth = rawDepth.clone(); 
        originDepth.convertTo(originDepth, CV_16UC1); 
        if (originDepth.size()!=cv::Size(320,240)) cv::resize(originDepth, originDepth, cv::Size(320,240)); 
        is_updated_depth = true; 
    }
  }
  catch (cv_bridge::Exception& e) {
  }
  catch (cv::Exception& e) {
  }
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "calibThermal"); 
  ros::NodeHandle nh("~"); 

  std::string setup_files; 
  nh.getParam("setup_files", setup_files); 
  RGBDT_transmat_file = std::string(setup_files + "/RGBDT_transmat.xml");
  
  std::cout << "===============================" << std::endl; 
  std::cout << "Calibration" << std::endl;
  std::cout << "Calibration matrix is saved in " << RGBDT_transmat_file << std::endl; 

  code = 'c'; 

  image_transport::ImageTransport it(nh); 
  image_transport::Subscriber sub_for_rgb = it.subscribe("/rgbd_receiver/rgb/image_raw", 1, imageCallbackForRGB); 
  image_transport::Subscriber sub_for_thermal = it.subscribe("/thermal/image", 1, imageCallbackForThermal); 
  image_transport::Subscriber sub_for_depth = it.subscribe("/rgbd_receiver/depth_registered/image_raw", 1, imageCallbackForDepth); 
  ros::spin();
  
  return 0; 
}
