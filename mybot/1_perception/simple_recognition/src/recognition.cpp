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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

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
plaincode::Object objects; 
plaincode::Color color;

std::vector<double> dist2hand; 

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
cv::Mat dilateMask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)); 

Eigen::VectorXf parameter = Eigen::VectorXf(3);
tf::TransformListener* g_tf_listener;

cv::Point object_center_point;
bool is_object_center_point_updated = false;
double objectProb = 0.95; 

void cropImage(cv::Mat& image, cv::Mat& result) {
  cv::Rect rect = cv::Rect(20, 20, 280, 200);
  result = image(rect);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBCloudPtr; 
boost::shared_ptr<pcl::visualization::PCLVisualizer> RGBCloudViewer; 

bool is_running = false;
int test(double fx, double fy, double cx, double cy, ros::Publisher& point_pub, ros::Publisher& pub_psw, bool saveMode=false){

	// PCL1. "Cloud", "Viewer" declaration. 
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBD_Cloud, recognition_Cloud; 
	static boost::shared_ptr<pcl::visualization::PCLVisualizer> RGBD_Viewer, recognition_Viewer;

    static boost::once_flag flag = BOOST_ONCE_INIT; 
	boost::call_once([]
					{
					cv::Size sz(300,400); 
					// PCL1-1) Create viewer 
					plaincode::createViewer(RGBD_Viewer, "RGBD_Viewer"); 
					plaincode::createViewer(recognition_Viewer, "recognition_Viewer"); 

					// PCL1-2) Set viewer
					plaincode::setViewer(RGBD_Viewer, cv::Point(600,100), sz, cv::Scalar(255,255,255)); 
					plaincode::setViewer(recognition_Viewer, cv::Point(600,100), sz, cv::Scalar(255,255,255)); 
					
					// PCL1-3) create cloud
					RGBD_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 
					recognition_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 

					},flag); 

    ros::WallDuration sleep_time(0.05);
    ros::Time current_time = ros::Time::now();

    Eigen::MatrixXf K, Kinv;
    plaincode::setCameraMat(fx, fy, cx-20.0, cy-20.0, K, Kinv);

    
    cv::Mat RGB, Depth;
    cv::Mat onPlane, prPlane, underPlane;
    cv::Mat result, prResult; 
    cv::Mat distMat;
//    cv::Mat Mask, Label;
    cv::Mat handMask;

    cv::Mat feature;
    Eigen::MatrixXf points;
    // Sort*320.0/640.0 toys.
    double time = cv::getTickCount();
    cropImage(originRGB, RGB);
    cropImage(originDepth, Depth);

    plaincode::crossBilateralFilter(RGB, Depth);
    plaincode::getPoint3d(Depth, points, Kinv);
    plaincode::getDistMat(Depth, distMat, points, parameter);
    plaincode::removePlane(distMat, underPlane, plaincode::CONSTANT, 1200., 80.);

    color.RGB2Mask(RGB, handMask, 0);
    handMask.setTo(0, distMat>-30);
    cv::dilate(handMask, handMask, dilateMask); 

    Eigen::MatrixXf handPoints; 
    cv::Mat handMat; 

    plaincode::findSamplePoints(handMask, points, handPoints); 
   
    std::vector<cv::Mat> inputMat; 
    std::vector<int> outputLabel; 

    int prevObjNum = objects.blobs.size(); 

    objects.clearBlobs(); 

    cv::Mat temp_sum = cv::Mat::ones(RGB.size(), CV_8UC1); 
    for (std::size_t i=0; i<color.isObject.size(); i++) {
        if (color.isObject[i]==0) continue; 
        cv::Mat temp; 
        color.RGB2Mask(RGB, temp, i); 
        temp_sum.setTo(0, temp); 
    }

    objects.projectOnPlane(distMat, onPlane, prPlane, handMask, points, parameter, K, -20.); 
//  objects.projectOnPlane(distMat, onPlane, prPlane, removeMat, points, parameter, K, -20.); 
// 
//    cv::imshow("prPlane", prPlane); 
    // color setting
    objects.refine(RGB, onPlane, prPlane, points, parameter, K, color.hsvMin, color.hsvMax, color.isObject); 
    
    objects.projectBlob(prPlane, 50); 
    objects.reconstructBlob(onPlane, points, parameter, K); 

    objects.filter(underPlane); 
    objects.accumulate(originRGB, inputMat); 

    std::vector<std::vector<float>> prediction; 

    objects.lua_classifies(lua, inputMat, outputLabel, objectProb, prediction); 
//  objects.lua_classify(lua, inputMat, outputLabel, objectProb); 


    objects.reflect(inputMat, outputLabel); 
/*
    for (std::size_t i=0; i<6; i++) {
      cv::Mat eachInputMat; 

      if (inputMat.size()<=i) {
        eachInputMat = cv::Mat::zeros(cv::Size(64,64), CV_8UC3); 
      }
      else { 
        eachInputMat = inputMat[i].clone(); 
      }
      cv::resize(eachInputMat, eachInputMat, cv::Size(64,64)); 
      std::string windowName("Candidate"); 
      windowName+=std::to_string(i); 
      cv::moveWindow(windowName, 100+280+280+60, (i+1)*100); cv::imshow(windowName, eachInputMat); 
    }
*/
/*
    for (std::size_t i=objects.blobs.size(); i<prevObjNum; i++) {
      std::string windowName("Candidate"); 
      windowName+=std::to_string(i); 
      cv::destroyWindow(windowName); 
    }
*/

/*
    for (std::size_t i=0; i<6; i++) {
     int colBar=20, rowBar=20, widthBar=10, heightBar=60;   

      cv::Mat plotMat = cv::Mat::ones(cv::Size(120,64), CV_8UC3); 
      std::string windowName("Prob"); 
      windowName+=std::to_string(i); 
      if (inputMat.size()>i) {
              for (std::size_t j=0; j<objects.subDir.size(); j++) {
                      int eachRowBar = (int)((1-std::pow(10, prediction[i][j]))*heightBar); 
                      int eachHeightBar = heightBar-eachRowBar; 

                      cv::rectangle(plotMat, cv::Rect(colBar+=12, eachRowBar, widthBar, eachHeightBar), objects.lut[j+1], CV_FILLED); 
              } 
      }

      cv::moveWindow(windowName, 100+280+280+160, (i+1)*100); cv::imshow(windowName, plotMat); 
    }
    */
/*
    for (std::size_t i=objects.blobs.size(); i<prevObjNum; i++) {
      std::string windowName("Prob"); 
      windowName+=std::to_string(i); 
      cv::destroyWindow(windowName); 
    }
*/

    if (saveMode) objects.save(inputMat); 
    objects.paint(RGB, result, prResult); 

    cv::Mat result_pcl, prResult_pcl; 
    objects.paint(RGB, result_pcl, prResult_pcl, false); 

    cv::Mat proj_for_objects;
    originRGB.copyTo(proj_for_objects);

    // Find the selected object
    if (objects.blobs.size()>0) {

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
                dist2hand.clear();
                for (std::size_t i=0; i<objects.blobs.size(); i++){
                    dist2hand.push_back((objects.blobs[i].prCenterPos3d-hand_end_point).norm()); 
                }


                int minDistIdx = std::distance(dist2hand.begin(), std::min_element(dist2hand.begin(), dist2hand.end())); 

                float distance_approaching = std::numeric_limits<float>::max();
                for(std::size_t i=0; i<objects.blobs.size(); i++){
                    if (objects.blobs[i].name==gObjName){
                        distance_approaching = dist2hand[i];
                        break;  
                    }
                }
            }
        }

    }

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

    cv::moveWindow("RGB", 100, 100); cv::imshow("RGB", RGB);
    cv::moveWindow("Depth", 100, 340); plaincode::imshow("Depth", Depth,1); 
    cv::Mat candidateMat = objects.idMat.clone(); 
    plaincode::getScaledImage(candidateMat, candidateMat, 1); 
    candidateMat.setTo(cv::Scalar(0,0,0), objects.labelMat==0); 
    cv::moveWindow("Candidate", 100+280+30, 340); cv::imshow("Candidate", candidateMat); 


    cv::Mat boundBoxRGB = RGB.clone(); 
    for (std::size_t i=0; i<objects.blobs.size(); i++) 
      cv::rectangle(boundBoxRGB, objects.blobs[i].rect, cv::Scalar(0,0,255), 3);


    cv::moveWindow("BoundingBox", 100+280+30,100); cv::imshow("BoundingBox", boundBoxRGB); 
    cv::moveWindow("Output", 100+280+280+40, 100); cv::imshow("Output", result); 
    cv::moveWindow("Projection", 100+280+280+40, 340); cv::imshow("Projection", prResult); 

	// PCL2. Get point cloud
	plaincode::getPointCloud(RGB, points, RGBD_Cloud); 
	plaincode::getPointCloud(result_pcl, points, recognition_Cloud); 

	// PCL3. Apply cloud to viewer
	plaincode::applyPointCloud(RGBD_Viewer, RGBD_Cloud); 
	plaincode::applyPointCloud(recognition_Viewer, recognition_Cloud); 


    int grabIdx = -1; 
    if (handPoints.rows()>0 && objects.blobs.size()>0) {
      std::vector<int> count; 
      for (std::size_t i=0; i<objects.blobs.size(); i++) {
        count.push_back(plaincode::countSphereMask(objects.blobs[i].centerPos3d, handPoints, 150)); 
      }

      int maxCount = std::distance(count.begin(), std::max_element(count.begin(), count.end())); 
      if (count[maxCount]>800) grabIdx=maxCount; 
    }

/*
    for (std::size_t i=0; i<objects.blobs.size(); i++) {
      if (grabIdx!=i) 
        plaincode::updateSphereCloud(recognition_Viewer, i, objects.blobs[i].centerPos3d); 
      else plaincode::updateSphereCloud(recognition_Viewer, i, objects.blobs[i].centerPos3d, cv::Scalar(255,0,0)); 
    }
*/

	// PCL4. spin viewer
    RGBD_Viewer->spinOnce(10,true);
	recognition_Viewer->spinOnce(10,true); 
	
	// PCL. set camera view point
	std::vector<pcl::visualization::Camera> cam; 
	RGBD_Viewer->getCameras(cam); 
	recognition_Viewer->setCameraParameters(cam[0]); 

   // RGB, Depth, RGBCloudPtr
//    cv::moveWindow("Hand", 100+280+280+280, 100); cv::imshow("Hand", handMat); 


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

        g_tf_listener = new tf::TransformListener();

        std::string ext = "_result.jpg";
        lua = luaL_newstate();
        luaL_openlibs(lua);

        plaincode::th::distAbove=0; // 실제 값은 -100
        plaincode::th::thSize=100;
        plaincode::th::thC=20;
        cv::Rect imgRect(20,20,280,200); 

        std::string lua_files, object_files, color_file, setup_files; 
        nh.getParam("lua_files", lua_files);
        nh.getParam("setup_files", setup_files); 

        if (setup_files[0]=='~') setup_files.replace(0,1,getenv("HOME")); 

        color.setFile(std::string(setup_files+"/palette"));
        luaL_dofile(lua, std::string(lua_files + "/0_parameter.lua").c_str());

        // other_things setting // 
        objects.setup(lua, lua_files, setup_files, imgRect); 
        boost::filesystem::path objectSaveDir =  objects.rootDir/objects.rawDir; 

        RGBCloudPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); // for rgb scale.

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

        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub_for_depth = it.subscribe("/rgbd_receiver/depth_registered/image_raw", 1, imageCallbackForDepth);
        image_transport::CameraSubscriber sub_for_rgb = it.subscribeCamera("/rgbd_receiver/rgb/image_raw", 1, boost::bind(cameraCallbackForRGB, _1, _2,  point_pub, pub_psw));

        //ros::waitForShutdown();
        ros::spin();

        lua_close(lua);
        return 0;
}
