#pragma once

#include<iostream>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace plaincode {

enum PLANE_FILTER {
	CONSTANT=0,
	ADAPTIVE=1
};

// Blob class
struct Blob {
	int id;
	int label;
	int numPixel;
  int prNumPixel;
	cv::Rect rect;

	// 2d points in 2d image
	cv::Point centerPos;
	cv::Point prCenterPos;

	// 3d points in 3d point clouds
	Eigen::Vector3f centerPos3d;
	Eigen::Vector3f prCenterPos3d; 
  std::vector<double> eigenVal; 
  std::vector<cv::Point2d> eigenVec; 

	std::string category;
	std::string name;
};

namespace th {
	int distAbove;
	int distBelow;
	int thSize;
	int thC;
};


void remove(boost::filesystem::path& currentDir) {
  // Remove the files in the currentDir.
  for (boost::filesystem::directory_iterator iter(currentDir); iter!=boost::filesystem::directory_iterator(); iter++) {
    boost::filesystem::remove(iter->path());

  }
}

int rename(const boost::filesystem::path& currentDir, std::string ext=".jpg") {

  int fileCount=0;
  for (boost::filesystem::directory_iterator iter(currentDir); iter!=boost::filesystem::directory_iterator(); iter++) {
    std::string temp("000000");
    std::string strnum = std::to_string(fileCount); // fileCount는 새로 생성해야 하는 file의 number
    temp.replace(temp.length()-strnum.length(), strnum.length(), strnum);

    std::string filename = currentDir.filename().string()+"_temp_" + temp+ext;
    std::string path = currentDir.string() + "/" + filename; // 임시로 이 파일로 다 저장해놓자.
    boost::filesystem::rename(iter->path(), path);  // old_path, new_path
    fileCount++;
  }

  fileCount=0;

  for (boost::filesystem::directory_iterator iter(currentDir); iter!=boost::filesystem::directory_iterator(); iter++) {
    std::string temp("000000");
    std::string strnum = std::to_string(fileCount); // fileCount는 새로 생성해야 하는 file의 number
    temp.replace(temp.length()-strnum.length(), strnum.length(), strnum);

    std::string filename = currentDir.filename().string()+ "_"+ temp+ext;
    std::string path = currentDir.string() + "/" + filename; // 임시로 이 파일로 다 저장해놓자.
    boost::filesystem::rename(iter->path(), path);  // old_path, new_path
    fileCount++;
  }

  return fileCount;
}

std::string extractFilename(const boost::filesystem::path& currentDir, std::string ext) {
  // filename이 없을 경우, 찾아준다.
  int fileCount=0;
  for (boost::filesystem::directory_iterator iter(currentDir); iter!=boost::filesystem::directory_iterator(); iter++) fileCount++;

  std::string temp("000000");
  std::string strnum = std::to_string(fileCount); // fileCount는 새로 생성해야 하는 file의 number
  temp.replace(temp.length()-strnum.length(), strnum.length(), strnum);

  std::string filename = currentDir.filename().string()+"_"+temp + ext;
  std::string path = currentDir.string() + "/" + filename;
  return path;
}

std::string extractFilename(const boost::filesystem::path& currentDir) {
  // filename이 없을 경우, 찾아준다.
  int fileCount=0;
  for (boost::filesystem::directory_iterator iter(currentDir); iter!=boost::filesystem::directory_iterator(); iter++) fileCount++;

  std::string temp("000000");
  std::string strnum = std::to_string(fileCount); // fileCount는 새로 생성해야 하는 file의 number
  temp.replace(temp.length()-strnum.length(), strnum.length(), strnum);

  std::string filename = currentDir.filename().string()+"_"+temp;
  std::string path = currentDir.string() + "/" + filename;
  return path;
}

std::string extractFilename(boost::filesystem::path& currentDir, std::string ext, std::string filename) {
  // filename이 없을 경우, 찾아준다.
  std::string path = currentDir.string() + "/" + filename + ext;
  return path;
}













void getScaledImage(const cv::Mat& src, cv::Mat& dst, int colored=1) {
	double min,max;
	cv::minMaxIdx(src, &min, &max);
	double scale = 255./(max-min);
	src.convertTo(dst, CV_8UC1, scale, -min*scale);
	if (!colored) return;
	cv::applyColorMap(dst, dst, cv::COLORMAP_JET);
}

void imshow(const std::string& winname, const cv::Mat& src, int colored=0) {
	cv::Mat dst;
	getScaledImage(src, dst, colored);
	cv::imshow(winname, dst);
}

void imwrite(const std::string& filename, cv::Mat& img, int colored=0) {
  double min,max;
  cv::Mat dst; 
  cv::minMaxIdx(img, &min, &max); 
  double scale=255./(max-min); 
  img.convertTo(dst, CV_8UC1, scale, -min*scale); 
  cv::applyColorMap(dst, dst, cv::COLORMAP_JET); 
  cv::imwrite(filename, dst); 
}

/*
void findBlob(cv::Mat& binary, cv::Mat& idMat, std::vector<Blob>& blobs) {
	// binary라 함은, object 영역은 255로, background 영역은 0으로 표현한 image
	blobs.clear();
	cv::Mat mask;
	mask = binary/255;
	mask.convertTo(idMat, CV_8UC1);
	idMat.setTo(1, idMat);

	int count=2; // starts at 2, because 0,1 are already used.

	for (auto i=0; i<idMat.rows; i++) {
		for (auto j=0; j<idMat.cols; j++) {
			int index = i*idMat.cols+j;

			if(idMat.data[index]!=1)    {
				// The pixel is already labeled, or background.
				continue;
			}

			Blob tempBlob;
			cv::floodFill(idMat, cv::Point(j,i), count, &tempBlob.rect, 0, 0, 4);

			// cv::Point(j,i): seed point
			mask = idMat==count;
			tempBlob.id = count; // "Label" image 상에서의 숫자와, id가 동일하게 한다.
			tempBlob.numPixel = cv::countNonZero(mask);

			if (tempBlob.numPixel<200 || tempBlob.numPixel>16000) {
				idMat.setTo(0, mask);
				continue;
			}

			// Find center position using moment
			cv::Moments mu = cv::moments(mask, true);
			tempBlob.centerPos = cv::Point((int)(mu.m10/mu.m00), (int)(mu.m01/mu.m00));


			int offset = tempBlob.rect.width-tempBlob.rect.height;

			if (offset>0) {
				tempBlob.rect.y= std::max(tempBlob.rect.y-offset/2, 0);
				tempBlob.rect.height=tempBlob.rect.width;
			}
			else {
				tempBlob.rect.x= std::max(tempBlob.rect.x+offset/2, 0);
				tempBlob.rect.width=tempBlob.rect.height;
			}

			tempBlob.rect.height = std::min(tempBlob.rect.height, idMat.rows-tempBlob.rect.y);
			tempBlob.rect.width = std::min(tempBlob.rect.width, idMat.cols-tempBlob.rect.x);

			blobs.push_back(tempBlob);
			count++;
		}
	}
}
*/



void findBlob(cv::Mat& binary, cv::Mat& idMat, std::vector<Blob>& blobs, int minIndex=144) {
  // binary라 함은, object 영역은 255로, background 영역은 0으로 표현한 image

  blobs.clear(); // temp blobs
  cv::Mat mask;
  mask = binary/255;
  mask.convertTo(idMat, CV_8UC1);
  idMat.setTo(1, idMat);

  int count=2; // starts at 2, because 0,1 are already used.

  for (auto i=0; i<idMat.rows; i++) {
    for (auto j=0; j<idMat.cols; j++) {
      int index = i*idMat.cols+j;

      if(idMat.data[index]!=1)    {
        // The pixel is already labeled, or background.
        continue;
      }

      Blob tempBlob;
      cv::floodFill(idMat, cv::Point(j,i), count, &tempBlob.rect, 0, 0, 4);

      // cv::Point(j,i): seed point
      mask = idMat==count;
      tempBlob.id = count; // "Label" image 상에서의 숫자와, id가 동일하게 한다.
      tempBlob.numPixel = cv::countNonZero(mask);
      
      if (tempBlob.numPixel<minIndex || tempBlob.numPixel>32000) {
        idMat.setTo(0, mask);
        continue;
      }

      // Find center position using moment
      cv::Moments mu = cv::moments(mask, true);
      tempBlob.centerPos = cv::Point((int)(mu.m10/mu.m00), (int)(mu.m01/mu.m00));
      tempBlob.centerPos3d = Eigen::Vector3f(0.0,0.0,0.0);
      blobs.push_back(tempBlob);
      count++;
    }
  }
}

bool findMaxBlob(cv::Mat& binary, cv::Mat& idMat, Blob& blob, int minIndex=120) {
	// binary라 함은, object 영역은 255로, background 영역은 0으로 표현한 image
	int maxIdx=0, maxPixel=-1;
	std::vector<Blob> blobs;
	cv::Mat mask;
	mask = binary/255;
	mask.convertTo(idMat, CV_8UC1);
	idMat.setTo(1, idMat);

	int count=2; // starts at 2, because 0,1 are already used.

	for (auto i=0; i<idMat.rows; i++) {
		for (auto j=0; j<idMat.cols; j++) {
			int index = i*idMat.cols+j;

			if(idMat.data[index]!=1)    {
				// The pixel is already labeled, or background.
				continue;
			}

			Blob tempBlob;
			cv::floodFill(idMat, cv::Point(j,i), count, &tempBlob.rect, 0, 0, 4);

			// cv::Point(j,i): seed point
			mask = idMat==count;
			tempBlob.id = count; // "Label" image 상에서의 숫자와, id가 동일하게 한다.
			tempBlob.numPixel = cv::countNonZero(mask);

			if (tempBlob.numPixel<minIndex || tempBlob.numPixel>16000) {
				idMat.setTo(0, mask);
				continue;
			}


			// Find center position using moment
			cv::Moments mu = cv::moments(mask, true);
			tempBlob.centerPos = cv::Point((int)(mu.m10/mu.m00), (int)(mu.m01/mu.m00));

			int offset = tempBlob.rect.width-tempBlob.rect.height;

			if (offset>0) {
				tempBlob.rect.y= std::max(tempBlob.rect.y-offset/2, 0);
				tempBlob.rect.height=tempBlob.rect.width;
			}
			else {
				tempBlob.rect.x= std::max(tempBlob.rect.x+offset/2, 0);
				tempBlob.rect.width=tempBlob.rect.height;
			}

			tempBlob.rect.height = std::min(tempBlob.rect.height, idMat.rows-tempBlob.rect.y);
			tempBlob.rect.width = std::min(tempBlob.rect.width, idMat.cols-tempBlob.rect.x);

			blobs.push_back(tempBlob);
			count++;
		}
	}

	if (blobs.size()==0) return false;

	for (std::size_t i=0; i<blobs.size(); i++) {
		if (blobs[i].numPixel>maxPixel) {
			maxIdx=i;
			maxPixel = blobs[i].numPixel;
		}
	}

	blob = blobs[maxIdx];
	idMat.setTo(0, idMat!=blob.id);
	idMat.setTo(255, idMat==blob.id);

	return true;
}

void findCoM(Blob& blob, cv::Mat& mask, Eigen::MatrixXf& points) {
	cv::Mat idx;
	std::vector<cv::Mat> splited;
	cv::findNonZero(mask, idx);
	cv::split(idx, splited);
	cv::Mat idx2 = splited[0]+splited[1]*mask.cols;
	Eigen::MatrixXf sampledPoints(idx2.rows, 3);

	for (auto j=0; j<idx2.rows; j++)
		sampledPoints.row(j) = points.row(((int*)idx2.data)[j]);
	blob.centerPos3d = sampledPoints.colwise().mean();
}


void findCoM_with_points(Blob& blob, cv::Mat& mask, Eigen::MatrixXf& points, Eigen::MatrixXf& sampledPoints) {
	cv::Mat idx;
	std::vector<cv::Mat> splited;
	cv::findNonZero(mask, idx);
	cv::split(idx, splited);
	cv::Mat idx2 = splited[0]+splited[1]*mask.cols;
  sampledPoints = Eigen::MatrixXf(idx2.rows, 3); 

	for (auto j=0; j<idx2.rows; j++)
		sampledPoints.row(j) = points.row(((int*)idx2.data)[j]);
	blob.centerPos3d = sampledPoints.colwise().mean();
}


void randperm(int* randNumber, int N, int k) {
	int count=0;
	int* index = new int [N];

	for (int i=0; i<N; i++) index[i]=i;

	do {
		int idx = rand()%N;
		randNumber[count++]=index[idx];
		index[idx]=index[--N];
	} while(--k);

	delete index;
}


void getPoint3d(int row, int col, unsigned short depth, Eigen::Vector3f& point, Eigen::MatrixXf& Kinv) {
	//row, col에 해당하는 depth를 3차원 point로 변환하자.
	point = Eigen::Vector3f(col*depth, row*depth, depth);
	point = Kinv*point;
}

void getPoint3d(cv::Point centerPos, unsigned short depth, Eigen::Vector3f& point, Eigen::MatrixXf& Kinv) {
	//row, col에 해당하는 depth를 3차원 point로 변환하자.
	point = Eigen::Vector3f(centerPos.x*depth, centerPos.y*depth, depth);
	point = Kinv*point;

}


void getPoint3d(cv::Mat& depth_mat, Eigen::MatrixXf& points, Eigen::MatrixXf& Kinv) {

        points = Eigen::MatrixXf(depth_mat.rows*depth_mat.cols,3);
        Eigen::MatrixXf imgCoordinate = Eigen::MatrixXf(depth_mat.rows*depth_mat.cols, 3);
        int count=0;

        for (std::size_t i=0; i<depth_mat.rows; i++) {
                for (std::size_t j=0; j<depth_mat.cols; j++) {
                        float z = depth_mat.at<unsigned short>(i, j);
                        imgCoordinate(count,0)=j*z;
                        imgCoordinate(count,1)=i*z;
                        imgCoordinate(count,2)=z;
                        count++;
                }
        }

        points = imgCoordinate* Kinv.transpose();
}

int countSphereMask(Eigen::Vector3f& center, Eigen::MatrixXf& points, float thDistance=50.) {

  int count=0; 

  Eigen::MatrixXf centerXf = points.rowwise()-center.transpose(); 
  Eigen::MatrixXf distanceXf = centerXf.rowwise().norm(); 

  for (int i=0; i<points.rows(); i++) {
    if (distanceXf(i)<thDistance) count++; 
  }

  return count; 
}

void getSphereMask(Eigen::Vector3f& center, Eigen::MatrixXf& points, cv::Mat& sphereMat, float thDistance=50.) {

	Eigen::MatrixXf centerXf = points.rowwise() - center.transpose();
	Eigen::MatrixXf distanceXf = centerXf.rowwise().norm();
	// 모든 포인트별로 하니가 느리다.

	for (int i=0; i<points.rows(); i++) {
		if (distanceXf(i)<thDistance)
			sphereMat.data[i] = 255;
	}
}

void getSphereMask(Eigen::MatrixXf& distanceXf, cv::Mat& sphereMat, float thDistance=50.) {

	for (int i=0; i<distanceXf.rows(); i++) {
		if (distanceXf(i)<thDistance)
			sphereMat.data[i] = 255;
	}
}

void getSphereBlob(cv::Mat& RGB, Eigen::Vector3f& center, cv::Mat& handMat, cv::Mat& distMat, cv::Mat& inHandMat, Eigen::MatrixXf& points, float inRadius, float outRadius) {

	cv::Mat inSphereMask=cv::Mat::zeros(handMat.size(), CV_8UC1);
	cv::Mat outSphereMask=cv::Mat::zeros(handMat.size(), CV_8UC1);

	Eigen::MatrixXf centerXf = points.rowwise() - center.transpose();
	Eigen::MatrixXf distanceXf = centerXf.rowwise().norm();

	plaincode::getSphereMask(distanceXf, inSphereMask, inRadius);
	plaincode::getSphereMask(distanceXf, outSphereMask, outRadius);

	inSphereMask.setTo(0, handMat);
	outSphereMask.setTo(0, handMat);

	cv::Mat handMatEdge;
	cv::Canny(handMat, handMatEdge, inRadius, outRadius);
	cv::Mat edgeDilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7));
	cv::dilate(handMatEdge, handMatEdge, edgeDilate);
	handMat.setTo(255, handMatEdge);

	inSphereMask.setTo(0, handMatEdge);
	outSphereMask.setTo(0, handMatEdge);

	inSphereMask.setTo(0, distMat>-10);
	outSphereMask.setTo(0, distMat>-10);

	cv::Mat idHandMat = cv::Mat::zeros(handMat.size(), CV_8UC1);
	std::vector<Blob> inHandBlobs;

	findBlob(outSphereMask, idHandMat, inHandBlobs);

	for (std::size_t i=0; i<inHandBlobs.size(); i++) {
		cv::Mat eachHandMat = cv::Mat::zeros(handMat.size(), CV_8UC1);
		eachHandMat = idHandMat==inHandBlobs[i].id;
		eachHandMat.setTo(0, ~inSphereMask);
		int sum = cv::countNonZero(eachHandMat);
		if (sum==0) {
			// delete the blob
			idHandMat.setTo(0, idHandMat==inHandBlobs[i].id);
			inHandBlobs.erase(inHandBlobs.begin()+i, inHandBlobs.begin()+i+1);
			i--;
		}
	}

	inHandMat = cv::Mat::zeros(handMat.size(), handMat.type());
	inHandMat.setTo(255, idHandMat);
}



void getColorSphereBlob(cv::Mat& hsv, Eigen::Vector3f& center, cv::Mat& handMat, cv::Mat& distMat, cv::Mat& inHandMat, Eigen::MatrixXf& points, float inRadius, float outRadius){

  cv::Mat inSphereMask=cv::Mat::zeros(handMat.size(), CV_8UC1);
  cv::Mat outSphereMask=cv::Mat::zeros(handMat.size(), CV_8UC1);

  Eigen::MatrixXf centerXf = points.rowwise() - center.transpose();
  Eigen::MatrixXf distanceXf = centerXf.rowwise().norm();

  plaincode::getSphereMask(distanceXf, inSphereMask, inRadius);
  plaincode::getSphereMask(distanceXf, outSphereMask, outRadius);

  inSphereMask.setTo(0, ~hsv|handMat);
  outSphereMask.setTo(0, ~hsv|handMat);

  cv::Mat idHandMat = cv::Mat::zeros(handMat.size(), CV_8UC1);
  std::vector<plaincode::Blob> inHandBlobs;

  findBlob(outSphereMask, idHandMat, inHandBlobs, 5);

  for (std::size_t i=0; i<inHandBlobs.size(); i++) {
    cv::Mat eachHandMat = cv::Mat::zeros(handMat.size(), CV_8UC1);
    eachHandMat = idHandMat==inHandBlobs[i].id;
    eachHandMat.setTo(0, ~inSphereMask);
    int sum = cv::countNonZero(eachHandMat);
    if (sum==0) {
      // delete the blob
      idHandMat.setTo(0, idHandMat==inHandBlobs[i].id);
      inHandBlobs.erase(inHandBlobs.begin()+i, inHandBlobs.begin()+i+1);
      i--;
    }
  }

  inHandMat = cv::Mat::zeros(handMat.size(), handMat.type());
  inHandMat.setTo(255, idHandMat);
}

void planeParameter(cv::Mat& imgDepth, Eigen::MatrixXf& points, Eigen::VectorXf& parameter) {

	int  maxIter = 999;

	float ec=20;
	float inlierRatio = 0.0;
	float bestInlierRatio = 0.3;
	int gridSpace;
	// 유효한 인덱스만 골라내보자.
	cv::Mat idx;
	cv::Mat mask = cv::Mat::zeros(imgDepth.size(), CV_8UC1);

	cv::Mat maskElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(125,5));
	cv::Mat depthFilterMat = imgDepth<2000;
	cv::dilate(depthFilterMat, depthFilterMat, maskElement);
	mask.setTo(1, (imgDepth!=0)&depthFilterMat);
	cv::findNonZero(mask, idx);

	if (idx.rows<999)
		gridSpace=1;
	else gridSpace = idx.rows/999;

	std::vector<cv::Mat> channels;
	cv::split(idx, channels);
	cv::Mat idx2 = (channels[1]*mask.cols+channels[0]);

	int sampledNumData = (int)((idx2.rows-idx2.rows%gridSpace)/gridSpace)+1;
	Eigen::MatrixXf sampledPoints = Eigen::MatrixXf(sampledNumData,3);

	for (auto i=0, si=0; i<idx2.rows; i+=gridSpace, si++)
		sampledPoints.row(si) = points.row(idx2.at<int>(i,0));

	Eigen::VectorXf plane = parameter;
	std::vector<std::size_t> inlier_idx;
	std::vector<std::size_t> best_inlier_idx;
	std::vector<std::size_t> final_inlier_idx;

	int* randNumber = new int [3];
	Eigen::MatrixXf distance;
	int i;
	for (i=0; i<maxIter; i++) {

		float plane_norm = plane.norm();
		// Get a distance from a selected plane and each 3d point.
		//            			float error=0;
		inlier_idx.clear();

		distance = ((sampledPoints*plane).array()-1).array()/plane_norm;
		for (auto j=0; j<sampledNumData; j++) {
			if (fabs(distance(j,0))<ec) inlier_idx.push_back(j);
		}

		if (inlier_idx.size()/(float)sampledNumData>=inlierRatio) {
			// 개수가 특정 %이상 될 때 종료한다.
			// 이것을 best_inlier_idx에 집어넣고 사용한다.
			inlierRatio = inlier_idx.size()/(float)sampledNumData;
			best_inlier_idx.clear();
			best_inlier_idx = inlier_idx;

			if (inlierRatio>=bestInlierRatio) {

				final_inlier_idx.clear();
				final_inlier_idx = best_inlier_idx;
				break;
			}
		}

		Eigen::MatrixXf A(3,3);
		randperm(randNumber, sampledNumData, 3);
		for (auto j=0; j<3; j++) {
			A.row(j) = sampledPoints.row(randNumber[j]);
		}

		Eigen::Vector3f meanValue = A.colwise().mean();
		Eigen::MatrixXf u = A.rowwise() - meanValue.transpose();
		Eigen::MatrixXf utu = u.transpose()*u;
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> es(utu);
		Eigen::MatrixXf V = es.eigenvectors();

		plane = V.block(0,0,3,1);
		double d = plane.dot(meanValue);
		plane = plane*(1./d);
	}

	if (i==maxIter) {
		return ;
	}

	// Inlier index
	Eigen::MatrixXf pts_inlier(final_inlier_idx.size(),3);
	for (std::size_t i=0; i<final_inlier_idx.size(); i++) {
		pts_inlier.row(i) = sampledPoints.row(final_inlier_idx[i]);
	}

	Eigen::Vector3f meanValue = pts_inlier.colwise().mean();

	Eigen::MatrixXf u = pts_inlier.rowwise() - meanValue.transpose();
	Eigen::MatrixXf utu = u.transpose()*u;
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> es(utu);
	Eigen::MatrixXf V = es.eigenvectors();
	plane = V.block(0,0,3,1);
	double d = plane.dot(meanValue);
	plane = plane*(1./d);

	parameter = plane;
	delete randNumber;
}



void getPointCloud(cv::Mat& RGB_mat, Eigen::MatrixXf& points, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr) {
    float p[3];
    pcl::PointXYZRGB point;
    pointCloudPtr->points.clear();
    
    int idx=0;
    
    for (int i=0; i<RGB_mat.rows; i++) { 
      for (int j=0; j<RGB_mat.cols; j++) {
        cv::Vec3b rgbPoint = RGB_mat.at<cv::Vec3b>(i,j);

        point.x = points(idx, 0)/1000.;
        point.y = -points(idx, 1)/1000.;
        point.z = points(idx++, 2)/1000.;

        point.r = rgbPoint[2];
        point.g = rgbPoint[1];
        point.b = rgbPoint[0];

        pointCloudPtr->points.push_back(point);
      }
    }
    
    pointCloudPtr->width = (int)pointCloudPtr->points.size();
    pointCloudPtr->height = 1;
}


void updatePointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer>& pointCloudViewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloudPtr) {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointCloudPtr);

        pointCloudViewer->removeAllPointClouds();
        pointCloudViewer->removeAllShapes();
        pointCloudViewer->addPointCloud<pcl::PointXYZRGB> (pointCloudPtr, rgb, "sample cloud");
        pointCloudViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

}


void updateSphereCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer>& pointCloudViewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloudPtr, int code, Eigen::Vector3f centerPos3d, cv::Scalar color = cv::Scalar(255,255,255)) {
        pcl::ModelCoefficients sphere_coeff; 

        sphere_coeff.values.resize (4);    // We need 4 values
        sphere_coeff.values[0] = centerPos3d(0)/1000.;
        sphere_coeff.values[1] = -centerPos3d(1)/1000.;
        sphere_coeff.values[2] = centerPos3d(2)/1000.;
        sphere_coeff.values[3] = 0.1;
        
        std::string name = std::string("object")+std::to_string(code);         

        pointCloudViewer->addSphere(sphere_coeff, name); 

        pointCloudViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 1, name);
        pointCloudViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.val[2], color.val[1], color.val[0], name);
        pointCloudViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, name); 
}


void updateLineCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer>& pointCloudViewer, Eigen::VectorXf& point1, Eigen::VectorXf& point2, cv::Scalar color, int idx=0) {

    pcl::ModelCoefficients cylinder_coeff; 
    cylinder_coeff.values.resize(7);

    cylinder_coeff.values[0] = point1.x()/1000.;
    cylinder_coeff.values[1] = point1.y()/1000.;
    cylinder_coeff.values[2] = point1.z()/1000.;

    cylinder_coeff.values[3] = (point2.x()-point1.x())/1000.;
    cylinder_coeff.values[4] = (point2.y()-point1.y())/1000.;
    cylinder_coeff.values[5] = (point2.z()-point1.z())/1000.;

    cylinder_coeff.values[6] = 0.0001;

    pointCloudViewer->addCylinder(cylinder_coeff, std::string("cylinder")+std::to_string(idx)); 
    pointCloudViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 3, std::string("cylinder")+std::to_string(idx));
    pointCloudViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 20, std::string("cylinder")+std::to_string(idx));
    pointCloudViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.val[2]/255.,  color.val[1]/255., color.val[0]/255., std::string("cylinder")+std::to_string(idx));

}

void getDistMat(cv::Mat& imgDepth, cv::Mat& distMat, Eigen::MatrixXf& points, Eigen::VectorXf& parameter) {

	distMat = cv::Mat::zeros(imgDepth.size(), CV_32FC1);
	Eigen::MatrixXf dist = ((points*parameter).array()-1)/parameter.norm();

	for (int i=0; i<imgDepth.rows; i++) {
		for (int j=0;j<imgDepth.cols; j++) {
			int index = i*imgDepth.cols+j;
			distMat.at<float>(i,j)= dist(index);
		}
	}
}

void removePlane(cv::Mat& distMat, cv::Mat& filtered, int method, double distMax, double distMin=9999.9) {

	cv::Mat tempDistMat;
	filtered = 255*cv::Mat::ones(distMat.size(), CV_8UC1);
	distMat.copyTo(tempDistMat);
	// plane에 해당하는 pixel을 제거해준다.

	tempDistMat.setTo(0, distMat>distMax);
	filtered.setTo(0, distMat>distMax);

	if (distMin<distMax) {
		tempDistMat.setTo(0, distMat<distMin);
		filtered.setTo(0, distMat<distMin);
	}

	if (method==ADAPTIVE) { // method==ADAPTIVE
		plaincode::getScaledImage(tempDistMat, tempDistMat, 0);
		cv::Mat preFiltered;
		cv::Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15,15));
		cv::adaptiveThreshold(tempDistMat, filtered, 255, CV_ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, plaincode::th::thSize*2+3, plaincode::th::thC);

	}
}

void project(cv::Mat& src , cv::Mat& dst, Eigen::MatrixXf& points, Eigen::VectorXf& parameter, Eigen::MatrixXf& K) {

  // mask==1일때의 index를 받아와야 한다.
  dst = cv::Mat::zeros(src.size(), CV_8UC1);
  std::vector<cv::Point> locations;
  if (cv::countNonZero(src)==0) return; 
  cv::findNonZero(src, locations);

  Eigen::MatrixXf sampledPoints(locations.size(), 3);
  std::vector<int> eachId(locations.size());

  for (std::size_t j=0; j<locations.size(); j++) {
    int index = locations[j].x+locations[j].y*src.cols;
    sampledPoints.row(j) = points.row(index);
  }

  Eigen::MatrixXf distM = (sampledPoints*parameter).array()-1;
  distM = distM.array()/parameter.norm();
  sampledPoints-=(distM*parameter.transpose()/parameter.norm());
  sampledPoints = sampledPoints.cwiseQuotient(sampledPoints.col(2).replicate(1,3));

  Eigen::MatrixXf prIdx = sampledPoints*K.transpose();

  for (int j=0; j<prIdx.rows(); j++) {
    int col = prIdx(j,0);
    int row = prIdx(j,1);
    if (col>=0 && col<src.cols && row>=0 && row<src.rows) {
      int index = row*src.cols+col;
      dst.data[index]=255;
    }
  }
}


void project(cv::Mat& src , cv::Mat& dst, Eigen::MatrixXf& points, Eigen::VectorXf& parameter, Eigen::MatrixXf& K, cv::Mat& ref) {

  // mask==1일때의 index를 받아와야 한다.
  dst = cv::Mat::zeros(src.size(), CV_8UC1);
  std::vector<cv::Point> locations;
  if (cv::countNonZero(src)==0) return; 
  cv::findNonZero(src, locations);
  std::vector<int> originIdx;

  Eigen::MatrixXf sampledPoints(locations.size(), 3);
  std::vector<int> eachId(locations.size());

  for (std::size_t j=0; j<locations.size(); j++) {
    int index = locations[j].x+locations[j].y*src.cols;
    sampledPoints.row(j) = points.row(index);
    originIdx.push_back(index);
  }

  Eigen::MatrixXf distM = (sampledPoints*parameter).array()-1;
  distM = distM.array()/parameter.norm();
  sampledPoints-=(distM*parameter.transpose()/parameter.norm());
  sampledPoints = sampledPoints.cwiseQuotient(sampledPoints.col(2).replicate(1,3));

  Eigen::MatrixXf prIdx = sampledPoints*K.transpose();

  for (int j=0; j<prIdx.rows(); j++) {
    int col = prIdx(j,0);
    int row = prIdx(j,1);
    int index = row*src.cols+col;
    if (col>=0 && col<src.cols && row>=0 && row<src.rows && ref.data[index]) {
      dst.data[originIdx[j]]=255;
    }
  }
}


void getCandidate(cv::Mat& distMat, cv::Mat& filtered, cv::Mat& mask, Eigen::MatrixXf& points, Eigen::VectorXf& parameter, Eigen::MatrixXf& K, double th=-20.) {

  removePlane(distMat, filtered, plaincode::CONSTANT, -20);
  filtered.setTo(0, mask);

  // projection
  cv::Mat prPlane;
  plaincode::project(filtered, prPlane, points, parameter, K);

  cv::Mat btwPlane;
  plaincode::removePlane(distMat, btwPlane, plaincode::CONSTANT, 0, -20);

  cv::Mat btwPrPlane;
  plaincode::project(btwPlane, btwPrPlane, points, parameter, K, prPlane);
  filtered.setTo(255, btwPrPlane);
}

void crossBilateralFilter(cv::Mat& imgRGB, cv::Mat& imgDepth) {

	cv::Mat resultDepth;
	cv::Mat imgGray, scaledDepth;

	scaledDepth = imgDepth.clone();

	// Convert a cropped RGB image into a gray image.
	cv::cvtColor(imgRGB, imgGray, cv::COLOR_RGB2GRAY);

	// Crop a depth image and normalize each pixel value that ranges from 0 to 255.
	double min, max;
	cv::minMaxLoc(scaledDepth, &min, &max);

	scaledDepth.convertTo(scaledDepth, 0, 255.0/max, 0);

	bool* noiseMask = new bool [scaledDepth.size().height*scaledDepth.size().width];

	for (std::size_t i=0; i<(std::size_t)(scaledDepth.size().height*scaledDepth.size().width); i++) {
		noiseMask[i]=0;
		if (scaledDepth.data[i]==0)
			noiseMask[i]=1;
	}

	int num_scales = 3;
	double sigma_s[3] = {12, 5, 8};
	double sigma_r[3] = {0.2, 0.08, 0.02};

	cbf::cbf(scaledDepth.size().height, scaledDepth.size().width, scaledDepth.data, imgGray.data, noiseMask, scaledDepth.data, num_scales, sigma_s, sigma_r);

	//		scaledDepth -> resultDepth로 변환해주어야 한다.
	scaledDepth.convertTo(scaledDepth, 2, max/255., 0);

	imgDepth = scaledDepth.clone();	
	cv::medianBlur(imgDepth, imgDepth, 5); 


}

/*void crossBilateralFilter(cv::Mat& imgRGB, cv::Mat& imgDepth) {

	cv::Mat resultDepth;
	cv::Mat imgGray, scaledDepth;

	scaledDepth = imgDepth.clone();

	// Convert a cropped RGB image into a gray image.
	cv::cvtColor(imgRGB, imgGray, cv::COLOR_RGB2GRAY);

	// Crop a depth image and normalize each pixel value that ranges from 0 to 255.
	double min, max;
	cv::minMaxLoc(scaledDepth, &min, &max);

	scaledDepth.convertTo(scaledDepth, 0, 255.0/max, 0);

	bool* noiseMask = new bool [scaledDepth.size().height*scaledDepth.size().width];

	for (std::size_t i=0; i<(std::size_t)(scaledDepth.size().height*scaledDepth.size().width); i++) {
		noiseMask[i]=0;
		if (scaledDepth.data[i]==0) noiseMask[i]=1;
	}

	int num_scales = 3;
	double sigma_s[3] = {12, 5, 8};
	double sigma_r[3] = {0.2, 0.08, 0.02};

	cbf::cbf(scaledDepth.size().height, scaledDepth.size().width, scaledDepth.data, imgGray.data, noiseMask, scaledDepth.data, num_scales, sigma_s, sigma_r);

	//		scaledDepth -> resultDepth로 변환해주어야 한다.
	scaledDepth.convertTo(scaledDepth, 2, max/255., 0);
	imgDepth = scaledDepth.clone();
}
*/

void tuneThreshold (bool tuneMode, cv::Mat& dist, cv::Mat& mask) {

	cv::Mat distMat = cv::Mat::zeros(dist.size(), dist.type());
	dist.copyTo(distMat, mask);

	if (tuneMode) {
		cv::namedWindow("Threshold Parameter");
		cv::createTrackbar("distAboveTable", "Threshold Parameter", &plaincode::th::distAbove, 100);
		cv::createTrackbar("thresholdSize", "Threshold Parameter", &plaincode::th::thSize, 225);
		cv::createTrackbar("thresholdC", "Threshold Parameter", &plaincode::th::thC, 125);
	}
	else {
		cv::destroyWindow("Threshold Parameter");

	}
}

void setCameraMat(float fx, float fy, float cx, float cy, Eigen::MatrixXf& K, Eigen::MatrixXf& Kinv) {
	K = Eigen::MatrixXf::Zero(3,3);
	Kinv = Eigen::MatrixXf::Zero(3,3);
	K(0,0)=fx;
	K(1,1)=fy;
	K(0,2)=cx;
	K(1,2)=cy;
	K(2,2)=1.f;

    Kinv = K.inverse();
}

}
