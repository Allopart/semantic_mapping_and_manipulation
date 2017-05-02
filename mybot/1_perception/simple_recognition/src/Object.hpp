#pragma once

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <ros/ros.h>
#include "patch.hpp"

extern "C" {
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
}


struct nnParameter {
	int maxIter;
	int channel;
	int batchSize;
	int numClass;
	std::string categoryName;
	std::vector<std::string> className;

};

#define MAXITER 30

namespace plaincode {


void paint(cv::Mat& result, cv::Mat& labelMat, std::vector<plaincode::Blob>& objects, std::vector<cv::Scalar>& allLut, int bgColor=0) 
{
    result = cv::Mat::zeros(labelMat.size(), CV_8UC3); 
  if (bgColor!=0) result.setTo(255);

	for (std::size_t i=0; i<objects.size(); i++) 
		result.setTo(allLut[objects[i].label], labelMat==objects[i].label); 
}

void putText(cv::Mat& result, std::vector<plaincode::Blob>& objects) {
	double scale=0.45;
	int thickness=1;

	for (std::size_t i=0; i<objects.size(); i++) 
		cv::putText(result, objects[i].name, cv::Point(objects[i].rect.x, objects[i].rect.y), cv::FONT_ITALIC, scale, cv::Scalar(255,255,255), thickness, 8); 
}


class Object {
public:
	// 물체에 대한 정보를 기억
public:

    Object() {}

//	void setup(lua_State *lua, std::string _luaDir, std::string confFile, cv::Rect _imgRect) {
	void setup(lua_State *lua, std::string _luaDir, std::string _rootDir, cv::Rect _imgRect, int numIter=100) {

    rootDir = _rootDir; 
		boost::filesystem::create_directory(rootDir);

    std::string setupFile = std::string(rootDir.string()+"/objects.info"); 
		std::ifstream setupStream(setupFile);

    if (!setupStream.is_open()) {
      std::ofstream tempStream(setupFile); 
      tempStream.close(); 
      setupStream.open(setupFile.c_str(), std::ios::in); 
    }
    
    imgRect = _imgRect; 

		std::string line;
		std::vector<std::string> allLine;
		while (std::getline(setupStream, line)) {

			std::string delimiter = "#";
			std::string token = line.substr(0, line.find(delimiter));
			if (token.length()!=0) allLine.push_back(token);
		}

    if (allLine.size()==0) { 
      ROS_ERROR("The object list is empty."); 
      ROS_ERROR("Add the \"objects.info\" in %s", rootDir.c_str()); 
      ROS_ERROR(" like \"OBJECT_NAME BLUE GREEN RED\"");
      ROS_ERROR(" e.g. \"MILK 0 255 0\"");
      exit(0); 
    }

		luaDir = _luaDir; 
		imageDir = "image";
		featureDir = "feature";
		modelDir = "model";
		rawDir = "unsorted";
		resultDir = "result";
  	
    lut.push_back(cv::Scalar(0,0,0)); 

		for (std::size_t i=0; i<allLine.size(); i++) {
			std::istringstream iss(allLine[i]);
			std::string name;
			cv::Scalar eachLUT;
			iss >> name >> eachLUT[0] >> eachLUT[1] >> eachLUT[2];
			subDir.push_back(name);
			lut.push_back(eachLUT);
		}


		nn.batchSize = 5;
		nn.maxIter = numIter;
 
		nn.numClass = (int)subDir.size();
		nn.categoryName = rootDir.string();
		generateDirectory();
		//lua에 각 parameter들을 전달 -> lua에 파라미터를 전달

		lua_getglobal(lua, "maxIter");
		lua_pushstring(lua, rootDir.c_str());
		lua_pushnumber(lua, nn.maxIter);
		lua_settable(lua, -3);
		lua_setglobal(lua, "maxIter");

		lua_getglobal(lua, "batchSize");
		lua_pushstring(lua, rootDir.c_str());
		lua_pushnumber(lua, nn.batchSize);
		lua_settable(lua, -3);
		lua_setglobal(lua, "batchSize");

		lua_getglobal(lua, "numClass");
		lua_pushstring(lua, rootDir.c_str());
		lua_pushnumber(lua, (int)subDir.size());
		lua_settable(lua, -3);
		lua_setglobal(lua, "numClass");

		lua_getglobal(lua, "className");
		lua_pushstring(lua, rootDir.c_str());
		lua_newtable(lua); // 1
		for (std::size_t i=0; i<subDir.size(); i++) {
			lua_pushstring(lua, subDir[i].c_str());
			lua_rawseti(lua, -2, i+1);
		}
		lua_settable(lua, -3);
		lua_setglobal(lua, "className");

		for (std::size_t i=0; i<subDir.size(); i++) {
			nn.className.push_back(subDir[i].string());
		}

		clearBlobs();
    setupStream.close(); 
	}

	void setLUT(std::vector<cv::Scalar>& _lut) {
		lut = _lut;
	}

	void generateDirectory() {
		boost::filesystem::create_directory(rootDir);
		boost::filesystem::create_directory(rootDir/imageDir);
		boost::filesystem::create_directory(rootDir/featureDir);
		boost::filesystem::create_directory(rootDir/modelDir);
    boost::filesystem::create_directory(rootDir/modelDir/"iteration");
		boost::filesystem::create_directory(rootDir/rawDir);
		boost::filesystem::create_directory(rootDir/resultDir);

		// directory 생성
		for (std::size_t i=0;i<subDir.size(); i++) {
			boost::filesystem::create_directory(rootDir/imageDir/subDir[i]);
			boost::filesystem::create_directory(rootDir/featureDir/subDir[i]);
			boost::filesystem::create_directory(rootDir/resultDir/subDir[i]);
		}

	}

	void clearBlobs() {
		blobs.clear();
		idMat = cv::Mat::zeros(imgRect.size(), CV_8UC1);
		prIdMat = cv::Mat::zeros(imgRect.size(), CV_8UC1);
		labelMat = cv::Mat::zeros(imgRect.size(), CV_8UC1);
		prLabelMat = cv::Mat::zeros(imgRect.size(), CV_8UC1);
	}

	void extendBlob(Blob& blob, int extSize, cv::Point imgSize, int shape=0) {
		blob.rect.x = MAX(0, blob.rect.x-extSize);
		blob.rect.y = MAX(0, blob.rect.y-extSize);
		blob.rect.width = MIN(imgSize.x-blob.rect.x, blob.rect.width+extSize*2);
		blob.rect.height = MIN(imgSize.y-blob.rect.y, blob.rect.height+extSize*2);
	}

  void extendBlob(cv::Rect& rect, int extSize, cv::Point imgSize, int shape=0) {
    rect.x = MAX(0, rect.x-extSize);
    rect.y = MAX(0, rect.y-extSize);
    rect.width = MIN(imgSize.x-rect.x, rect.width+extSize*2);
    rect.height = MIN(imgSize.y-rect.y, rect.height+extSize*2);
  }

	void append(cv::Mat& tempId, std::vector<Blob> tempBlobs) {
		unsigned int size= blobs.size();

//		idMat+=tempId;
    
    cv::add(tempId, size, tempId, tempId!=0); 
    tempId.copyTo(idMat, tempId); 

		for (std::size_t i=0; i<tempBlobs.size(); i++) {
			tempBlobs[i].id += size;
		}
		blobs.insert(blobs.end(), tempBlobs.begin(), tempBlobs.end());
	}

	void append(cv::Mat& colored, int minPixel=100) {
		cv::Mat tempLabel;
		std::vector<plaincode::Blob> tempBlob;
		plaincode::findBlob(colored, tempLabel, tempBlob, minPixel);
		append(tempLabel, tempBlob);  
	}


	void appendPatches(cv::Mat& RGB, std::vector<cv::Mat>& patchMat, std::vector<std::vector<cv::Point2f>>& rotatedPt) {

		cv::Mat gray;
		std::vector<std::vector<cv::Point>> contours;
		std::vector<std::vector<cv::Point2f>> corner;
		cv::cvtColor(RGB, gray, CV_BGR2GRAY);
		gray = ~gray;
		Patch::gray2Contours(gray, contours);
		Patch::findRectangle(contours, corner);
		Patch::sortCorners(corner);

		std::vector<cv::Point> sampleMarkPoint(4);
		std::vector<std::vector<cv::Point>> eachMarkPoint;
		std::vector<std::vector<cv::Point>> markPoint;

		eachMarkPoint.resize(1);
		std::fill(eachMarkPoint.begin(), eachMarkPoint.end(), sampleMarkPoint);
		markPoint.resize(corner.size());
		std::fill(markPoint.begin(), markPoint.end(), sampleMarkPoint);

        int count=2; 

		for (std::size_t i=0; i<corner.size(); i++) {
			cv::Mat rotated;
			plaincode::Patch::extractImage(RGB, rotated, corner[i], rotatedPt[0]);
			patchMat.push_back(rotated);
			for (std::size_t j=0; j<4; j++) {
				markPoint[i][j] = cv::Point(corner[i][j].x, corner[i][j].y);
			}
			eachMarkPoint[0] = markPoint[i];

			Blob tempBlob;
			tempBlob.id = count;

			cv::fillPoly(idMat, eachMarkPoint, count, 4);
			tempBlob.numPixel = cv::countNonZero(idMat==count); 
	        
            if (tempBlob.numPixel<=360 || tempBlob.numPixel>=10000) continue; 

        	cv::Moments mu = cv::moments(idMat==count, true);
			tempBlob.centerPos = cv::Point((int)(mu.m10/mu.m00), (int)mu.m01/mu.m00);

            
			cv::Mat points;
			cv::findNonZero(idMat==count, points);
			tempBlob.rect = cv::boundingRect(points);

			blobs.push_back(tempBlob);
            count++; 

		}
	}

	
	void project(Eigen::MatrixXf& points, Eigen::VectorXf& parameter, Eigen::MatrixXf& K, bool merge=true) {

		// mask==1일때의 index를 받아와야 한다.
		for (std::size_t i=0; i<blobs.size(); i++) {

			cv::Mat idx;
			std::vector<cv::Mat> splited;
			cv::Mat mask = idMat==blobs[i].id;

			cv::findNonZero(mask, idx);
			cv::split(idx, splited);
            if(splited.size() != 2){
                continue;
            }
			cv::Mat idx2 = splited[0] + splited[1]*idMat.cols;

			Eigen::MatrixXf sampledPoints(idx2.rows, 3);
			std::vector<int> eachId(idx2.rows);

			for (std::size_t j=0; j<idx2.rows; j++) {
				sampledPoints.row(j) = points.row(((int*)idx2.data)[j]);
				eachId[j] = blobs[i].id;
			}

			blobs[i].centerPos3d = sampledPoints.colwise().mean();

			Eigen::MatrixXf distM = (sampledPoints*parameter).array()-1;
			distM = distM.array()/parameter.norm();
			std::vector<float> distVec(distM.data(), distM.data()+distM.rows());

			sampledPoints-=(distM*parameter.transpose()/parameter.norm());
			blobs[i].prCenterPos3d = sampledPoints.colwise().mean();
			sampledPoints = sampledPoints.cwiseQuotient(sampledPoints.col(2).replicate(1,3));

			Eigen::MatrixXf prIdx = sampledPoints*K.transpose();


			for (int j=0; j<prIdx.rows(); j++) {
				int col = prIdx(j,0);
				int row = prIdx(j,1);
				if (col>=0 && col<idMat.cols && row>=0 && row<idMat.rows) {
					int index = row*idMat.cols+col;
					prIdMat.data[index]=eachId[j];
				}
			}
		}

		for (std::size_t i=0; i<blobs.size(); i++) {
			cv::Moments mu = cv::moments(prIdMat==blobs[i].id, true);
			blobs[i].prCenterPos =cv::Point((int)(mu.m10/mu.m00), (int)(mu.m01/mu.m00));
		}
	}

	void filter(cv::Mat& forbidden) {
		cv::Mat erosion = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(125,15));
		cv::dilate(forbidden, forbidden, erosion);

		for (std::size_t i=0; i<blobs.size(); i++) {
			cv::Mat mask = prIdMat==blobs[i].id;
			// mask와 forbidden과 겹치는 부분이 있는지 확인한다.
			int totalSum = cv::sum(mask)[0];
			mask.setTo(0, ~forbidden);
			int sum = cv::sum(mask)[0];
			if (sum/(double)totalSum>0.05) {
				idMat.setTo(0, idMat==blobs[i].id);
				prIdMat.setTo(0, prIdMat==blobs[i].id);
				blobs.erase(blobs.begin()+i, blobs.begin()+i+1);
				i--;
			}
    }
  }

  void filter(cv::Mat& forbidden, cv::Mat& deletedMat) {

          deletedMat = cv::Mat::zeros(forbidden.size(), CV_8UC1);
          cv::Mat erosion = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(125,15));
          cv::dilate(forbidden, forbidden, erosion);

          for (std::size_t i=0; i<blobs.size(); i++) {
                  cv::Mat mask = prIdMat==blobs[i].id;
                  // mask와 forbidden과 겹치는 부분이 있는지 확인한다.
                  int totalSum = cv::sum(mask)[0];
                  mask.setTo(0, ~forbidden);
                  int sum = cv::sum(mask)[0];
                  if (sum/(double)totalSum>0.05) {
                          deletedMat.setTo(255, idMat==blobs[i].id);
                          idMat.setTo(0, idMat==blobs[i].id);
                          prIdMat.setTo(0, prIdMat==blobs[i].id);
                          blobs.erase(blobs.begin()+i, blobs.begin()+i+1);
                          i--;
                  }
          }
  }

  void paint(cv::Mat& result, cv::Mat& prResult) {

          for (std::size_t i=0; i<blobs.size(); i++) {
                  if (blobs[i].label!=0) {
				result.setTo(lut[blobs[i].label], labelMat==blobs[i].label);
				prResult.setTo(lut[blobs[i].label], prLabelMat==blobs[i].label);
			}
		}
	}

	void imageSave(std::string path, cv::Mat& image) {
		cv::imwrite(path, image);
	}

	void xmlSave(std::string path, cv::Mat& feature) {
		cv::FileStorage file(path, cv::FileStorage::WRITE);
		file << "feature" << feature;
		file.release();
	}

	void makeFeatureSet(cv::Mat& trainingMat, cv::Mat& labelMat) {
		cv::Mat feature;
		for (std::size_t i = 0; i<subDir.size(); i++) {
			// label = i
			for (boost::filesystem::directory_iterator iter(rootDir/featureDir/subDir[i]); iter!=boost::filesystem::directory_iterator(); iter++) {
				cv::FileStorage file(iter->path().string(), cv::FileStorage::READ);
				file["feature"] >> feature;
				trainingMat.push_back(feature);
				labelMat.push_back(i);
			}
		}

		trainingMat.convertTo(trainingMat, CV_32FC1);
	}

//	void saveModel(cv::Mat& trainingMat, cv::Mat& labelMat) {
//		cv::Ptr<cv::ml::SVM> svm;
//
//		for (std::size_t i=0; i<subDir.size(); i++) {
//			cv::Mat binLabelMat = -cv::Mat::ones(labelMat.size(), labelMat.type());
//			binLabelMat.setTo(1, labelMat==i);
//
//			svm = cv::ml::SVM::create();
//			svm->setType(cv::ml::SVM::NU_SVC);
//			svm->setKernel(cv::ml::SVM::LINEAR);
//			svm->setGamma(20);
//			svm->setNu(0.2);
//			svm->train(trainingMat, cv::ml::ROW_SAMPLE, binLabelMat);
//			svm->save((rootDir/modelDir/subDir[i]).string()+".txt");
//		}
//
//	}

	void siftFeature(cv::Mat& im, cv::Mat& feature) {
		// im: gray scale image to be normalized as 64x64 pixels.
		// feature: 128 dimension vector.

		cv::Mat gray;
		im.copyTo(gray);

		if (gray.channels()==3) cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);

		cv::resize(gray, gray, cv::Size(64,64));
		gray.convertTo(gray, CV_64F);
		feature = cv::Mat::zeros(cv::Size(128,1), gray.type());

		// sift feature를 얻는다.
		cv::Mat G_Y = (cv::Mat_<double>(5,5) << 0.0102, 0.1060, 0.2316, 0.1060, 0.0102,
				0.0118, 0.1225, 0.2675, 0.1225, 0.0118,
				0,      0,      0,      0,      0,
				-0.0118, -0.1225, -0.2675, -0.1225, -0.0118,
				-0.0102, -0.1060, -0.2316, -0.1060, -0.0102);

		cv::Mat G_X = G_Y.t();
		//
		double alpha=9;
		int num_angles=8;
		int num_bins=4;
		int H=64, W=64;

		std::vector<double> angles;
		double acc_angle=0.;
		double angle_step = 2*M_PI/num_angles;
		double num_samples = num_bins*num_bins;

		do {
			angles.push_back(acc_angle);
			acc_angle+=angle_step;
		} while(acc_angle<2*M_PI);

		// sample point�� �����ش�.

		Eigen::MatrixXd sample_sub(1,num_bins);
		double interval = (double)(H-1)/num_bins;

		Eigen::MatrixXd sample_x = Eigen::RowVectorXd::LinSpaced(num_bins, 0, num_bins-1).replicate(num_bins,1);
		Eigen::MatrixXd sample_y = Eigen::VectorXd::LinSpaced(num_bins, 0, num_bins-1).replicate(1,num_bins);

		sample_x=(sample_x*interval).array()+interval/2.;
		sample_y=(sample_y*interval).array()+interval/2.;

		sample_x.resize(1, num_samples);
		sample_y.resize(1, num_samples);

		cv::Mat kernel = (cv::Mat_<double>(3,3) << -0, -1, 0, -1, 5, -1, 0, -1, 0);
		cv::Mat temp = cv::Mat::eye(4, 4, kernel.type());

		cv::Mat I_X, I_Y;

		cv::filter2D(gray, I_X, gray.depth(), G_X, cv::Point(-1,-1), 0, 0);
		cv::filter2D(gray, I_Y, gray.depth(), G_Y, cv::Point(-1,-1), 0, 0);

		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> I_X_eigen;
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> I_Y_eigen;

		cv::cv2eigen(I_X, I_X_eigen);
		cv::cv2eigen(I_Y, I_Y_eigen);

		Eigen::MatrixXd Theta = I_Y_eigen.binaryExpr(I_X_eigen, std::ptr_fun(atan2));

		//  I_X, I_Y�� magnitude, orientation
		Eigen::MatrixXd mag = (I_X_eigen.cwiseProduct(I_X_eigen)+I_Y_eigen.cwiseProduct(I_Y_eigen)).cwiseSqrt();

		// Find coordinates of pixels
		Eigen::MatrixXd position_x = Eigen::RowVectorXd::LinSpaced(W, 0,W-1).replicate(H,1);
		Eigen::MatrixXd position_y = Eigen::VectorXd::LinSpaced(H, 0, H-1).replicate(1,W);

		position_x.resize(H*W, 1);
		position_y.resize(H*W, 1);

		// Find (horiz, vert) distance between each pixel and each grid sample
		// sample_x, sample_y가 sample_x_t, sample_y_t와 같은 건지를 확인해봐야함.
		Eigen::MatrixXd weight_x=(position_x.replicate(1, num_samples)-sample_x.replicate(H*W, 1)).cwiseAbs();
		Eigen::MatrixXd weight_y=(position_y.replicate(1, num_samples)-sample_y.replicate(H*W, 1)).cwiseAbs();

		weight_x = weight_x.array()/(16.);
		weight_y = weight_y.array()/(16.);

		weight_x = (weight_x.array()<=1).select(1-weight_x.array(), 0);
		weight_y = (weight_y.array()<=1).select(1-weight_y.array(), 0);

		Eigen::MatrixXd weights = weight_x.cwiseProduct(weight_y);

		for (auto i=0, idx=0;i<num_angles;i++, idx+=16){

			Eigen::MatrixXd orientation = (Theta.array()-angles[i]).cos();
			orientation = orientation.array().pow(alpha);
			orientation = (orientation.array()>0).select(orientation, 0);
			Eigen::MatrixXd tmp1 = orientation.cwiseProduct(mag);
			Eigen::MatrixXd tmp2 = Eigen::Map<Eigen::MatrixXd>(tmp1.data(), 1, H*W);
			Eigen::MatrixXd feat = tmp2*weights;
			for (int j=0; j<16; j++) feature.at<double>(j+idx)=feat(0,j);

		}

		feature = feature/cv::norm(feature, cv::NORM_L2);
		feature.convertTo(feature, CV_32FC1);
	}

	void saveFeature() {

		cv::Mat feature;
		std::string ext = ".xml";

		for (std::size_t i = 0; i<subDir.size(); i++) {

			for (boost::filesystem::directory_iterator iter(rootDir/featureDir/subDir[i]); iter!=boost::filesystem::directory_iterator(); iter++) {

				cv::Mat selected = cv::imread(iter->path().c_str());
				boost::filesystem::path saveDir = rootDir/featureDir/subDir[i];

				std::string xmlfilePath = extractFilename(saveDir, ext, iter->path().stem().string());

				siftFeature(selected, feature);
				xmlSave(xmlfilePath, feature);
			}
		}
	}


	void lua_saveModel(lua_State* lua) {

		// inputMat 생성

		std::cout << "in lua save model" << std::endl; 
		int key=1;
		lua_newtable(lua); // 1
		for (auto i=0; i<subDir.size(); i++) {
			for (boost::filesystem::directory_iterator iter(rootDir/imageDir/subDir[i]); iter!=boost::filesystem::directory_iterator(); iter++) {
				lua_pushnumber(lua, key++);
				cv::Mat selected = cv::imread(iter->path().c_str());
				cv::resize(selected, selected, cv::Size(32,32));
				mat2lua(lua, selected);
				lua_settable(lua, -3);
			}
		}
		lua_setglobal(lua, "inputMat");
		lua_newtable(lua);

		// labelMat 생성
		key = 1;
		for (auto i=0; i<subDir.size(); i++) {
			for (boost::filesystem::directory_iterator iter(rootDir/imageDir/subDir[i]); iter!=boost::filesystem::directory_iterator(); iter++) {
				lua_pushnumber(lua, i+1);
				lua_rawseti(lua, -2, key++);
			}
		}
		lua_setglobal(lua, "labelMat");

		std::cout << "lua_save model end" << std::endl; 

		lua_pushstring(lua, rootDir.c_str()); lua_setglobal(lua, "class");
		luaL_dofile(lua, (luaDir/"1_data.lua").c_str());
		luaL_dofile(lua, (luaDir/"2_model.lua").c_str());
		luaL_dofile(lua, (luaDir/"3_train.lua").c_str());

		std::cout << "lua_save model endend" << std::endl; 
	}

	void accumulate(cv::Mat& originRGB, std::vector<cv::Mat>& inputMat) {

    inputMat.clear();

		for (std::size_t i=0; i<blobs.size(); i++) {
      cv::Rect rect = blobs[i].rect; 

      rect.x += imgRect.x; 
      rect.y += imgRect.y; 

      extendBlob(rect, 3, originRGB.size()); 

      int offset = rect.width-rect.height; 

      if (offset>0) {
        rect.y = std::max(rect.y-offset/2, 0); 
        rect.height=rect.width; 
      }
      else {
        rect.x = std::max(rect.x+offset/2, 0); 
        rect.width = rect.height; 
      }

      rect.height = std::min(rect.height, originRGB.rows-rect.y); 
      rect.width = std::min(rect.width, originRGB.cols-rect.x); 

			cv::Mat selected = originRGB(rect);
			cv::resize(selected, selected, cv::Size(32,32));
			inputMat.push_back(selected);
		}
	}


  void projectOnPlane(cv::Mat& distMat, cv::Mat& onPlane, cv::Mat& prPlane, cv::Mat handMatId, Eigen::MatrixXf& points, Eigen::VectorXf& parameter, Eigen::MatrixXf& K, double th=-20.) {

    cv::Mat dilateMask1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1,9));
    cv::Mat dilateMask2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,1));
    cv::Mat dilateMask3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)); 

    plaincode::removePlane(distMat, onPlane, plaincode::CONSTANT, th);
    cv::Mat tempHandMatId = handMatId!=0; 
    cv::Canny(tempHandMatId, tempHandMatId, 30, 100); 
    cv::dilate(tempHandMatId, tempHandMatId, dilateMask3); 
    
    onPlane.setTo(0, handMatId);
    plaincode::project(onPlane, prPlane, points, parameter, K);
    //
    cv::Mat btwPlane;
    plaincode::removePlane(distMat, btwPlane, plaincode::CONSTANT, -5, th);

    cv::Mat btwPrPlane;
    plaincode::project(btwPlane, btwPrPlane, points, parameter, K, prPlane);
    onPlane.setTo(255, btwPrPlane);

    cv::dilate(prPlane, prPlane, dilateMask1); 
    cv::erode(prPlane, prPlane, dilateMask2); 
    cv::medianBlur(prPlane, prPlane, 3);

  }

  void refine(cv::Mat& RGB, cv::Mat onPlane, cv::Mat& prPlane, Eigen::MatrixXf& points, Eigen::VectorXf& parameter, Eigen::MatrixXf& K, std::vector<cv::Scalar_<int>>& hsvMin, std::vector<cv::Scalar_<int>>& hsvMax, std::vector<int>& isObject) {
    std::vector<cv::Mat> tempMasks, prMasks, Masks;
    // prPlane의 각 blob에 대한 id얻기.
    // 각각의 id에 대하여 돌기.
    cv::Mat tempIdMat, tempColorIdMat;
    std::vector<Blob> tempBlobs, tempColorBlobs;
    plaincode::findBlob(prPlane, tempIdMat, tempBlobs, 100);

    plaincode::Color::RGB2MaskAll(RGB, tempMasks, hsvMin, hsvMax, isObject);

    for (std::size_t i=0; i<tempMasks.size(); i++) {
      cv::Mat prMask;
      plaincode::project(tempMasks[i], prMask, points, parameter, K);
      if (cv::countNonZero(prMask)<30) continue;
      Masks.push_back(tempMasks[i]);
      prMasks.push_back(prMask);
    }

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    for (std::size_t i=0; i<tempBlobs.size(); i++) {
      // 해당 id로 편입할지 안할지를 정한다.
      // 편입이 되면, break;
      cv::Mat selectedId = tempIdMat==tempBlobs[i].id;
      cv::Mat nonSelectedId = tempIdMat.clone();
      nonSelectedId.setTo(0, selectedId);

      for (std::size_t j=0; j<prMasks.size(); j++) {
        tempColorIdMat.setTo(0);
        plaincode::findBlob(prMasks[j], tempColorIdMat, tempColorBlobs, 30);

        for (std::size_t k=0; k<tempColorBlobs.size(); k++) {
          //1. nonSelectedId와 겹치는 것이 없어야 한다.
          //2. selectedId와 겹치는 것이 있어야 한다. ->
          cv::Mat oneColorMat = tempColorIdMat==tempColorBlobs[k].id;
//          cv::erode(oneColorMat, oneColorMat, kernel);
          cv::Mat rejectMat = oneColorMat&nonSelectedId;

          if (cv::countNonZero(rejectMat)) continue;

          cv::Mat acceptMat = oneColorMat|selectedId;
          if (cv::countNonZero(acceptMat)) {
            prPlane.setTo(255, acceptMat);
            onPlane.setTo(255, Masks[j]);
            break;
          }
        }

      }
    }
  }


  void reconstructBlob(cv::Mat& onPlane , Eigen::MatrixXf& points, Eigen::VectorXf& parameter, Eigen::MatrixXf& K) {

    // mask==1일때의 index를 받아와야 한다.
    idMat = cv::Mat::zeros(onPlane.size(), CV_8UC1);
    std::vector<cv::Point> locations;
    cv::Mat mask, prMask;

    if (cv::countNonZero(onPlane)==0) return;
    cv::findNonZero(onPlane, locations);
    std::vector<int> originIdx;

    Eigen::MatrixXf sampledPoints(locations.size(), 3);
    Eigen::MatrixXf onPoints, prPoints;
    std::vector<int> eachId(locations.size());

    for (std::size_t j=0; j<locations.size(); j++) {
      int index = locations[j].x+locations[j].y*onPlane.cols;
      sampledPoints.row(j) = points.row(index);
      originIdx.push_back(index);
    }

    onPoints = sampledPoints;
    Eigen::MatrixXf distM = (sampledPoints*parameter).array()-1;
    distM = distM.array()/parameter.norm();
    sampledPoints-=(distM*parameter.transpose()/parameter.norm());
    prPoints = sampledPoints;

    sampledPoints = sampledPoints.cwiseQuotient(sampledPoints.col(2).replicate(1,3));

    Eigen::MatrixXf prIdx = sampledPoints*K.transpose();

    for (int j=0; j<prIdx.rows(); j++) {
      int col = prIdx(j,0);
      int row = prIdx(j,1);
      int index = row*onPlane.cols+col;
      if (col>=0 && col<onPlane.cols && row>=0 && row<onPlane.rows && prIdMat.data[index]) 
        idMat.data[originIdx[j]]=prIdMat.data[index];
    }

    for (std::size_t i=0; i<blobs.size(); i++) {

      // blob의 centerPos와, rect를 구한다.
      mask = idMat==blobs[i].id;
      prMask = prIdMat==blobs[i].id;

      cv::Moments mu = cv::moments(mask, true);
      cv::Moments prMu = cv::moments(prMask, true);

      blobs[i].centerPos = cv::Point((int)(mu.m10/mu.m00), (int)(mu.m01/mu.m00));
      blobs[i].prCenterPos = cv::Point((int)(prMu.m10/prMu.m00), (int)(prMu.m01/prMu.m00));

      cv::Mat point;
      blobs[i].numPixel = cv::countNonZero(mask);
      blobs[i].prNumPixel = cv::countNonZero(prMask);

      if (blobs[i].numPixel!=0) {
        cv::findNonZero(mask, point);
        blobs[i].rect = cv::boundingRect(point);
      }

      cv::Mat idx;
      std::vector<cv::Mat> splited;
      cv::findNonZero(mask, idx);
      cv::split(idx, splited);
      cv::Mat idx2 = splited[0] + splited[1]*idMat.cols;

      Eigen::MatrixXf sampledPoints(idx2.rows, 3);

      for (int j=0; j<idx2.rows; j++) {

        sampledPoints.row(j) = points.row(((int*)idx2.data)[j]);
      }

      blobs[i].centerPos3d = sampledPoints.colwise().mean();

      Eigen::MatrixXf distM = (sampledPoints*parameter).array()-1;
      distM = distM.array()/parameter.norm();
      std::vector<float> distVec(distM.data(), distM.data()+distM.rows());

      sampledPoints-=(distM*parameter.transpose()/parameter.norm());
      blobs[i].prCenterPos3d = sampledPoints.colwise().mean();
    }
  }


  void projectBlob(cv::Mat& binary, int minPixel=100) {

    cv::Mat mask, tempPrIdMat;
    prIdMat = cv::Mat::zeros(binary.size(), CV_8UC1);
    cv::Mat shadowMask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));

    mask = binary/255;
    mask.convertTo(tempPrIdMat, CV_8UC1);
    tempPrIdMat.setTo(1, tempPrIdMat);

    int count=2; // starts at 2, because 0,1 are alread used.
    for (int i=0; i<tempPrIdMat.rows; i++) {
      for (int j=0; j<tempPrIdMat.cols; j++) {
        int index = i*tempPrIdMat.cols+j;

        if(tempPrIdMat.data[index]!=1)    {
          // The pixel is already labeled, or background.
          continue;
        }

        Blob tempBlob;
        cv::floodFill(tempPrIdMat, cv::Point(j,i), count, &tempBlob.rect, 0, 0, 4);
        // cv::Point(j,i): seed point
        mask = tempPrIdMat==count;
        tempBlob.id = count; // "Label" image 상에서의 숫자와, id가 동일하게 한다.
        double numPixel = cv::countNonZero(mask);

        if (numPixel<minPixel || numPixel>32000) {
          tempPrIdMat.setTo(0, mask);
          continue;
        }

        // mask를 확장
        // 거기에 count라는 숫자를 집어넣기.
        cv::dilate(mask, mask, shadowMask);
        prIdMat.setTo(count, mask);
        cv::Moments mu = cv::moments(mask, true);
        tempBlob.prCenterPos = cv::Point((int)(mu.m10/mu.m00), (int)(mu.m01/mu.m00));
        blobs.push_back(tempBlob);
        count++;

      }
    }

  }













































//	int classify(cv::Mat& feature, Blob& blob) {
//		cv::Ptr<cv::ml::SVM> svm;
//		std::vector<float> costOfObject(subDir.size());
//
//		for (std::size_t i=0; i<subDir.size(); i++) {
//			std::string modelFile = (rootDir/modelDir/subDir[i]).string() + ".txt";
//			svm = cv::ml::StatModel::load<cv::ml::SVM>(modelFile);
//			cv::Mat res;
//			svm->predict(feature, res, true);
//			costOfObject[i] = res.at<float>(0,0);
//		}
//
//		int index = std::min_element(costOfObject.begin(), costOfObject.end())-costOfObject.begin();
//		return index;
//	}

  int lua_classifies(lua_State* lua, std::vector<cv::Mat>& inputMat, std::vector<int>& outputLabel, float prob, 
std::vector<std::vector<float>>& prediction) {

          int rotated=0;

          std::vector<float> value; 

          std::string inputMatName("inputMat");
          mats2lua(lua, inputMat, inputMatName);

          outputLabel.resize(inputMat.size());
          value.resize(inputMat.size()); 

          // test_all.lua로 보내자.
          // logProb를 보내자.
          lua_pushstring(lua, rootDir.c_str()); lua_setglobal(lua, "class");
          //          if (classNumber==0) {
          luaL_dofile(lua, (luaDir/"5_test.lua").c_str()); 
          //        }
          /*      else {
                  lua_pushnumber(lua, classNumber);lua_setglobal(lua, "classNumber");

                  luaL_dofile(lua, (luaDir/"5_test_if.lua").c_str());
                  lua_getglobal(lua, "rotated");
                  rotated = lua_tonumber(lua,-1);
                  }
           */
          lua_getglobal(lua, "prediction"); 
          for (std::size_t i=0; i<inputMat.size(); i++) {
                  lua_pushnumber(lua, i+1);
                  lua_gettable(lua, -2);
                  std::vector<float> eachPrediction((int)subDir.size());
                  for (std::size_t j=0; j<subDir.size(); j++) {
                          lua_pushnumber(lua, j+1);
                          lua_gettable(lua,-2);
                          eachPrediction[j]=lua_tonumber(lua, -1);
                          lua_pop(lua, 1);
                  }
                  prediction.push_back(eachPrediction);
                  lua_pop(lua, 1);
          }

 //         std::cout << "Prob = " << prob << std::endl; 
          // prediction에서, max id와, 그 때의 value를 구한다.
          for (std::size_t i=0; i<prediction.size(); i++) {
                  // prediction[i]에서 가장 큰 index와, 그 index에 해당하는 값을 저장한다.
                  // index는 outputLabel[i]로 저장하고, index에 해당하는 값은 value[i]로 저장한다
                  outputLabel[i] = std::distance(prediction[i].begin(), std::max_element(prediction[i].begin(), prediction[i].end()))+1;
                  value[i] = prediction[i][outputLabel[i]-1];
//                  std::cout << "value[" << i << "]:" << value[i] << std::endl; 
                  blobs[i].label = outputLabel[i];
                  blobs[i].name = subDir[outputLabel[i]-1].string();
          }

          // 중복된 label을 제거한다.

          for (std::size_t i=0; i<blobs.size(); i++) {
                  if (value[i]<prob) outputLabel[i]=0;
          }

          // Background Label을 제거한다
          for (std::size_t i=0; i<blobs.size(); i++) {
                  if (blobs[i].name == "OTHERS") outputLabel[i]=0;
          }

          for (std::size_t i=0; i<blobs.size(); i++) {
                  if (outputLabel[i]==0) {
                          blobs.erase(blobs.begin()+i, blobs.begin()+i+1);
                          outputLabel.erase(outputLabel.begin()+i, outputLabel.begin()+i+1);
                          value.erase(value.begin()+i, value.begin()+i+1);
                          inputMat.erase(inputMat.begin()+i, inputMat.begin()+i+1);
                          prediction.erase(prediction.begin()+i, prediction.begin()+i+1); 
                          i--;
                          continue;
                  }
          }
          return rotated;
  }



  int lua_classify(lua_State* lua, std::vector<cv::Mat>& inputMat, std::vector<int>& outputLabel, float prob=0.9, int classNumber=0) {

          int rotated=0;

          std::vector<float> value; 
          std::vector<std::vector<float>> prediction;     

          std::string inputMatName("inputMat");
          mats2lua(lua, inputMat, inputMatName);

          outputLabel.resize(inputMat.size());
          value.resize(inputMat.size()); 

          // test_all.lua로 보내자.
          // logProb를 보내자.
          lua_pushstring(lua, rootDir.c_str()); lua_setglobal(lua, "class");
          //          if (classNumber==0) {
          luaL_dofile(lua, (luaDir/"5_test.lua").c_str()); 
          //        }
          /*      else {
                  lua_pushnumber(lua, classNumber);lua_setglobal(lua, "classNumber");

                  luaL_dofile(lua, (luaDir/"5_test_if.lua").c_str());
                  lua_getglobal(lua, "rotated");
                  rotated = lua_tonumber(lua,-1);
                  }
           */
          lua_getglobal(lua, "prediction"); 
          for (std::size_t i=0; i<inputMat.size(); i++) {
                  lua_pushnumber(lua, i+1);
                  lua_gettable(lua, -2);
                  std::vector<float> eachPrediction((int)subDir.size());
                  for (std::size_t j=0; j<subDir.size(); j++) {
                          lua_pushnumber(lua, j+1);
                          lua_gettable(lua,-2);
                          eachPrediction[j]=lua_tonumber(lua, -1);
                          lua_pop(lua, 1);
                  }
                  prediction.push_back(eachPrediction);
                  lua_pop(lua, 1);
          }

          // prediction에서, max id와, 그 때의 value를 구한다.
          for (std::size_t i=0; i<prediction.size(); i++) {
                  // prediction[i]에서 가장 큰 index와, 그 index에 해당하는 값을 저장한다.
                  // index는 outputLabel[i]로 저장하고, index에 해당하는 값은 value[i]로 저장한다
                  outputLabel[i] = std::distance(prediction[i].begin(), std::max_element(prediction[i].begin(), prediction[i].end()))+1;
                  value[i] = prediction[i][outputLabel[i]-1];
                  blobs[i].label = outputLabel[i];
                  blobs[i].name = subDir[outputLabel[i]-1].string();
          }

          for (std::size_t i=0; i<blobs.size(); i++) {
                  if (value[i]<prob) outputLabel[i]=0;
          }

          for (std::size_t i=0; i<blobs.size(); i++) {
                  if (outputLabel[i]==0) {
                          blobs.erase(blobs.begin()+i, blobs.begin()+i+1);
                          outputLabel.erase(outputLabel.begin()+i, outputLabel.begin()+i+1);
                          value.erase(value.begin()+i, value.begin()+i+1);
                          inputMat.erase(inputMat.begin()+i, inputMat.begin()+i+1);
                          prediction.erase(prediction.begin()+i, prediction.begin()+i+1); 
                          i--;
                          continue;
                  }
          }


          for (int i=0; i<(int)blobs.size()-1; i++) {
            for (int j=i+1; j<(int)blobs.size(); j++) {
              if (blobs[i].name==blobs[j].name) {
                float distance = (blobs[i].centerPos3d-blobs[j].centerPos3d).norm(); 
                if (distance<100) {
                  idMat.setTo(blobs[i].id, idMat==blobs[j].id); 
                  labelMat.setTo(blobs[i].label, labelMat==blobs[j].label); 
                  prIdMat.setTo(blobs[i].id, prIdMat==blobs[j].id); 
                  prLabelMat.setTo(blobs[i].label, prLabelMat==blobs[j].label); 
                  blobs.erase(blobs.begin()+j, blobs.begin()+j+1); 
                  j--; 
                  continue; 
                } 
              }

            }
          }

          return rotated;
  }


  void reflect(std::vector<cv::Mat>& inputMat, std::vector<int>& outputLabel) {
          for (std::size_t i=0; i<blobs.size(); i++) {
                  labelMat.setTo(blobs[i].label, idMat==blobs[i].id);
                  prLabelMat.setTo(blobs[i].label, prIdMat==blobs[i].id);
                  cv::Mat shadow = prIdMat==blobs[i].id; 
                  cv::Mat pts; 
                  cv::findNonZero(shadow,pts); 
                  if (pts.rows!=0) 
                          graspDirection(pts, blobs[i]); 
                  else {
                          blobs[i].eigenVal.resize(2); 
                          blobs[i].eigenVec.resize(2);  
                          blobs[i].eigenVal[0]=0.1;
                          blobs[i].eigenVal[1]=0.1; 
                          blobs[i].eigenVec[0] = cv::Point2d(1.0, 1.0); 
        blobs[i].eigenVec[1] = cv::Point2d(1.0,-1.0); 

//        blobs[i].eigenVec[0] = cv::Point2d(1.0,1.0); 
//        blobs[i].eigenVec[1] = cv::Point2d(1.0,-1.0); 
      }
		}
	}

  
  void paint(cv::Mat& RGB, cv::Mat& result, cv::Mat& prResult, bool test=true) {

    if (!test) {
      plaincode::paint(result, labelMat, blobs, lut, 1);
      plaincode::paint(prResult, prLabelMat, blobs, lut, 1);
    }
    else {
      plaincode::paint(result, labelMat, blobs, lut);
      plaincode::paint(prResult, prLabelMat, blobs, lut);
    }
    

    cv::addWeighted(RGB, 0.35, result,   0.65, 0.0, result);
    cv::addWeighted(RGB, 0.35, prResult, 0.65, 0.0, prResult);

    if (test) {
      plaincode::putText(result,   blobs);
      plaincode::putText(prResult, blobs);
    }
  }

   void paint(cv::Mat& RGB, cv::Mat& result, cv::Mat& prResult, cv::Mat& labelMat, cv::Mat& prLabelMat, bool test=true) {

    if (!test) { // whitening
      plaincode::paint(result, labelMat, blobs, lut, 1); 
      plaincode::paint(prResult, prLabelMat, blobs, lut, 1); 
    }
    else {
      plaincode::paint(result, labelMat, blobs, lut); 
      plaincode::paint(prResult, prLabelMat, blobs, lut); 
    }

    cv::addWeighted(RGB, 0.3, result,   0.7, 0.0, result);
    cv::addWeighted(RGB, 0.3, prResult, 0.7, 0.0, prResult);

    if (test) {
      plaincode::putText(result,   blobs);
      plaincode::putText(prResult, blobs);
    }
  }
 

  void graspDirection(const cv::Mat& _pts, Blob& blob) {
    cv::Mat pts(cv::Size(2, _pts.rows), CV_64FC1); 
    for (int i=0; i<_pts.rows; i++) {
      pts.at<double>(i,0) = _pts.at<cv::Point>(i).x; 
      pts.at<double>(i,1) = _pts.at<cv::Point>(i).y; 
    }

    cv::PCA pca_analysis(pts, cv::Mat(), CV_PCA_DATA_AS_ROW); 

    blob.eigenVec.resize(2); 
    blob.eigenVal.resize(2); 

    for (int i=0; i<2; i++) {
      blob.eigenVec[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i,0), pca_analysis.eigenvectors.at<double>(i,1)); 
      blob.eigenVal[i] = pca_analysis.eigenvalues.at<double>(0,i); 
    }
  }

	void save(std::vector<cv::Mat>& inputMat) {
	   for (std::size_t i=0; i<blobs.size(); i++) {
            boost::filesystem::path imgDir = rootDir/resultDir/subDir[blobs[i].label-1-labelOffset];
            std::string imagefilePath = extractFilename(imgDir, "_result.jpg");
	        cv::imwrite(imagefilePath, inputMat[i]);
	
    	}
   } 

	void save(std::vector<cv::Mat>& inputMat, const boost::filesystem::path& imgDir) {
		for (std::size_t i=0; i<inputMat.size(); i++) {
			std::string imagefilePath = extractFilename(imgDir, ".jpg");
			cv::imwrite(imagefilePath, inputMat[i]);
		}
	}

	void save(std::vector<cv::Mat>& inputMat, const boost::filesystem::path& imgDir, std::vector<bool>& isObject) {
		for (std::size_t i=0; i<inputMat.size(); i++) {
      if (isObject[i]==false) continue; 
			std::string imagefilePath = extractFilename(imgDir, ".jpg");
			cv::imwrite(imagefilePath, inputMat[i]);
		}
	}


//	void lua_classify(lua_State* lua, cv::Mat& im, std::string& modelname, Blob& blob) {
//
//		cv::resize(im, im, cv::Size(32,32));
//		std::vector<cv::Mat> splited;
//		cv::split(im, splited);
//
//		lua_newtable(lua);
//		for (auto channel=0; channel<3; channel++) {
//			lua_pushnumber(lua, channel+1);
//			lua_newtable(lua);
//
//			int pCount=0;
//			for (auto row=0; row<splited[channel].rows; row++) {
//				lua_pushnumber(lua, row+1); //
//				lua_newtable(lua); //
//				for (auto col=0; col<splited[channel].cols; col++) {
//					lua_pushnumber(lua, splited[channel].data[pCount++]);
//					lua_rawseti(lua, -2, col+1);
//				}
//				lua_settable(lua,-3);
//			}
//			lua_settable(lua, -3);
//		}
//
//		lua_setglobal(lua, "inputMat");
//		lua_pushstring(lua, rootDir.c_str());
//		lua_setglobal(lua, "ClassName");
//		lua_pushstring(lua, modelname.c_str());
//		lua_setglobal(lua, "modelName");
//		luaL_dofile(lua, "Lua/4_test.lua");
//
//		lua_getglobal(lua, "index");
//		blob.label = lua_tonumber(lua, -1);
//		if (blob.label>0) {
//			blob.name = subDir[blob.label-1].string();
//			blob.category = rootDir.stem().string();
//		}
//	}


	void makeDisplay(cv::Mat& displayMat, double scale) {
		displayMat.setTo(scale);
	}

	void makeDisplay(cv::Mat& displayMat, cv::Mat& mask, cv::Scalar_<double> lut) {
		displayMat.setTo(lut, mask);
	}


	void track(int option, double ratio, double onTime=0.0) {
		// option이 0일 때, off
		// 1일 때, on
		// 2일 때, onTime (특정시간이 지난 후, on되며, object recognition이 안정화된 후, track을 하기 위함이다.)
		if (option==0) {
			return;
		}
		else if (option==1) {
		}

	}

	void putText(cv::Mat& outputMat, int pr=0) {

		double scale=0.5;
		int thickness=1;
		if (pr==0) {
			for (std::size_t i=0; i<blobs.size(); i++) {
				if (blobs[i].label!=0)
					cv::putText(outputMat, blobs[i].name, cv::Point(blobs[i].rect.x, blobs[i].rect.y), cv::FONT_ITALIC, scale, cv::Scalar(255,255,255), thickness, 8);
			}
		}
		else {
			for (std::size_t i=0; i<blobs.size(); i++) {
				if (blobs[i].label!=0)
					cv::putText(outputMat, blobs[i].name, cv::Point(blobs[i].rect.x, blobs[i].rect.y), cv::FONT_ITALIC, scale, cv::Scalar(255,255,255), thickness, 8);
			}
		}
	}

	void mat2lua(lua_State* lua, cv::Mat& img) {
		// img 파일을 lua로 보내는 함수로, img뿐만 아니라 img의 channel수와 함께 보낸다.

		lua_newtable(lua);
		int channel = img.channels();
		std::vector<cv::Mat> splited;
		cv::split(img, splited);

		for (auto channel=0; channel<3; channel++) {
			lua_pushnumber(lua, channel+1);
			lua_newtable(lua);

			int pCount=0;
			for (auto row=0; row<splited[channel].rows; row++) {
				lua_pushnumber(lua, row+1); //
				lua_newtable(lua); //
				for (auto col=0; col<splited[channel].cols; col++) {
					lua_pushnumber(lua, splited[channel].data[pCount++]);
					lua_rawseti(lua, -2, col+1);
				}
				lua_settable(lua,-3);
			}
			lua_settable(lua, -3);
		}
	}

	void mats2lua(lua_State* lua, std::vector<cv::Mat>& imgs, std::string& matName, int maxIter=100) {
		// 여러개의 mats를 lua로 보내는 함수.
		lua_newtable(lua);
		for (std::size_t i=0;i<imgs.size(); i++) {
			lua_pushnumber(lua, i+1);
			mat2lua(lua, imgs[i]);
			lua_settable(lua,-3);
		}
		lua_setglobal(lua, matName.c_str());
	}

public:
	boost::filesystem::path rootDir;
	boost::filesystem::path luaDir;
	boost::filesystem::path imageDir;
	boost::filesystem::path featureDir;
	boost::filesystem::path modelDir;
	boost::filesystem::path rawDir;
	boost::filesystem::path resultDir;
	std::vector<boost::filesystem::path> subDir;
	std::vector<cv::Scalar> lut;


	cv::Mat prevLabel;
	nnParameter nn;

	std::vector<Blob> blobs; 
	cv::Mat idMat; 
	cv::Mat prIdMat;
	cv::Mat labelMat;
	cv::Mat prLabelMat; 

	int labelOffset; 

//	cv::Size imgSize; 
  cv::Rect imgRect; 

};

}
