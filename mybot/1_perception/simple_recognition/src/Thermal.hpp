#include<iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace plaincode {

int findBiggestContour(std::vector<std::vector<cv::Point>>& contours) {
	int idx=-1;
	int numPoint=0;
	for (std::size_t i=0; i<contours.size(); i++) {
		if (contours[i].size()>numPoint) {
			numPoint=contours[i].size();
			idx=i;
		}
	}
	return idx;
}

float distance(cv::Point a, cv::Point b) {
	float d = std::sqrt(std::fabs(std::pow(a.x-b.x,2)+std::pow(a.y-b.y,2)));
	return d;
}

float angle(cv::Point s, cv::Point f, cv::Point e) {
	float l1 = distance(f,s);
	float l2 = distance(f,e);
	float dot=(s.x-f.x)*(e.x-f.x)+(s.y-f.y)*(e.y-f.y);
	float angle=std::acos(dot/(l1*l2));
	angle=angle*180/M_PI;
	return angle;
}


void findPoints(cv::Mat& _mask, std::vector<std::vector<cv::Point>>& contours, std::vector<cv::Point>& points, cv::Mat& result) {
	std::vector<cv::Vec4i> hierarchy_color;
	cv::Mat mask = _mask.clone();
	cv::findContours(mask, contours, hierarchy_color, CV_RETR_TREE, CV_CLOCKWISE, cv::Point(0,0));

	result = cv::Mat::zeros(mask.size(), CV_8UC3);
	points.clear();

	std::vector<std::vector<cv::Point>> hullP(contours.size());
	std::vector<std::vector<int>> hullI(contours.size());
	std::vector<std::vector<cv::Vec4i>> defects_color(contours.size());
	cv::Rect maxRect;

	cv::Point startPt, endPt, farPt;

	int idx=plaincode::findBiggestContour(contours);
	if (idx!=-1) {

		cv::drawContours(result, contours, idx, cv::Scalar(0,255,255), 1);
		maxRect = cv::boundingRect(contours[idx]);
		cv::convexHull(contours[idx], hullP[idx], false, true);
		cv::convexHull(contours[idx], hullI[idx], false, false);

		if (contours[idx].size()>3) {
			cv::convexityDefects(contours[idx], hullI[idx], defects_color[idx]);

			int startIdx, endIdx, farIdx;
			float depthVal;

			for (std::size_t i=0; i<defects_color[idx].size(); i++) {

				startIdx = defects_color[idx][i][0];	startPt = cv::Point(contours[idx][startIdx]);
				endIdx = defects_color[idx][i][1];		endPt = cv::Point(contours[idx][endIdx]);
				farIdx = defects_color[idx][i][2]; 		farPt = cv::Point(contours[idx][farIdx]);
				depthVal = defects_color[idx][i][3]/256.;


				if (depthVal>10 && depthVal<80) {
					if (points.size()==0) {
						points.push_back(startPt);
						points.push_back(endPt);
					}
					else {
						points.back() = startPt;
						points.push_back(endPt);
					}
					cv::line(result, startPt, farPt, cv::Scalar(0,255,0), 2);
					cv::line(result, endPt, farPt, cv::Scalar(0,255,0), 2);
				}
			}
			for (std::size_t i=0; i<points.size(); i++) {
				cv::circle(result, points[i], 3, cv::Scalar(255,0,0), 2);
			}
		}
	}


}


void randomSamplePts(cv::Mat& mColorPts, cv::Mat& mHeatPts, cv::Mat& mColorSamples, cv::Mat& mHeatSamples) {
	int numPoints = mColorPts.rows;
	int* index = new int [numPoints];
	std::vector<int> randNumber;
	int k=4;
	for (int i=0;i<numPoints;i++) index[i]=i;
	do {
		int idx = rand()%numPoints;
		mColorSamples.push_back(mColorPts.row(index[idx]));
		mHeatSamples.push_back(mHeatPts.row(index[idx]));
		index[idx]=index[--numPoints];
	} while(--k);
}



void randperm(std::vector<int>& randNumber, int N, int k) {
	int* index = new int [N];
	randNumber.clear();
	for (int i=0; i<N; i++) index[i]=i;
	do {
		int idx = rand()%N;
		randNumber.push_back(index[idx]);
		index[idx]=index[--N];
	} while(--k);
	delete index;
}


void randomSample(cv::Mat& mColorPoints, cv::Mat& mHeatPoints, std::vector<int>& finalInlierIdx, cv::Mat& globalTransmat) {

	 double errTh=3000.0;
	double inlierRatio = 0.0;
	double bestInlierRatio = 0.7;
	int numData = mColorPoints.rows;
	int maxIter = 1000;
	cv::Mat finalTransmat;
	finalInlierIdx.clear();

	/*  - localTransmat:  각 iteration마다의 transmat
		- globalTransmat: 현재까지 가장 적합한 transmat
		- finalTransmat:  최종 transmat                */

	std::vector<int> bestInlierIdx;

	for (int iter=0; iter<maxIter; iter++) {

		cv::Mat localTransmat;
		cv::Mat mColorSamples, mHeatSamples;
		std::vector<int> inlierIdx; // bestInlierIdx가 매번 이터레이션 마다 초기화가 된다.
		inlierIdx.clear();

		if (iter>0) {
			randomSamplePts(mColorPoints, mHeatPoints, mColorSamples, mHeatSamples);
			localTransmat = ((mColorSamples.t()*mColorSamples))*(mHeatSamples.t()*mColorSamples).inv();
		}
		else localTransmat = globalTransmat.clone();

		cv::Mat pw, pwsum;

		cv::pow((mColorPoints -mHeatPoints*localTransmat.t()), 2, pw);
		cv::reduce(pw.t(), pw, 0, CV_REDUCE_SUM, pw.type());
		cv::sqrt(pw, pw);

		for (int i=0; i<numData; i++)
			if (pw.at<float>(0,i)<errTh) inlierIdx.push_back(i);

		if (inlierIdx.size()/(float)numData>=inlierRatio) { // 현재 inlier의 비율이 기존의 inlier비율보다 클 경우,
			inlierRatio = inlierIdx.size()/(float)numData;
			bestInlierIdx.clear();
			bestInlierIdx=inlierIdx;
			globalTransmat = localTransmat.clone();

			if (inlierRatio>=bestInlierRatio) {
				finalInlierIdx.clear();
				finalInlierIdx = bestInlierIdx;
				finalTransmat = globalTransmat.clone();
				break;
			}
		}

		if (iter==maxIter-1) { // 마지막까지 iteration을 돌았는데, 아직 안끝난 경우.
			finalInlierIdx.clear();
			finalInlierIdx = bestInlierIdx;
			finalTransmat = globalTransmat.clone();
		}
	}
}

void deleteOutlier(cv::Mat& mColorPoints, cv::Mat& mHeatPoints, cv::Mat& pColorPoints, cv::Mat& pHeatPoints, std::vector<int>& inlierIdx) {

	cv::Mat tempColorPts, tempHeatPts;

	for (std::size_t i=0; i<inlierIdx.size(); i++) {
		tempColorPts.push_back(mColorPoints.row(i));
		tempHeatPts.push_back(mHeatPoints.row(i));
	}

	tempColorPts.copyTo(mColorPoints);
	tempHeatPts.copyTo(mHeatPoints);
}

void getTransformed(cv::Mat& RGB, cv::Mat& Depth, cv::Mat& Thermal, cv::Mat& extracted, cv::Mat& transmat) {
	extracted = cv::Mat::zeros(RGB.size(), CV_16UC1);

	// 이 부분을 수정해야함.
	cv::Mat pts_th_e;

	for (int i=0; i<Thermal.rows; i++) {
		for (int j=0; j<Thermal.cols; j++) {
			// Depth를 구한다.
			float z = (float)(Depth.at<unsigned short>(i, j));
			cv::Mat sample_th = (cv::Mat_<float>(1,3)<<(float)j*z, (float)i*z, z);
			pts_th_e.push_back(sample_th);
		}
	}
	cv::Mat pts_rgb_e = pts_th_e*transmat.t();

	//pts_rgb_e를 각각의 z로 나누어주자.
	for (int i=0; i<pts_rgb_e.rows; i++) {

		int pCol = (int)(pts_th_e.at<float>(i,0)/pts_th_e.at<float>(i,2));
		int pRow = (int)(pts_th_e.at<float>(i,1)/pts_th_e.at<float>(i,2));

		int col = (int)(pts_rgb_e.at<float>(i,0)/pts_rgb_e.at<float>(i,2));
		int row = (int)(pts_rgb_e.at<float>(i,1)/pts_rgb_e.at<float>(i,2));

		if (col>=0 && col<Thermal.cols && row>=0 && row<Thermal.rows)
			extracted.at<unsigned short>(row,col) = Thermal.at<unsigned short>(pRow, pCol);
	}
}

}

