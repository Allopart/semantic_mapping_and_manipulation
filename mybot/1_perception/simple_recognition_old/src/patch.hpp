#pragma once

#include<iostream>

namespace plaincode {

class Patch {
public:

public:
	Patch(std::string _rootDir, std::vector<std::string> _subDir) {
		rootDir = _rootDir;
		imageDir = "image";
		rawDir = "unsorted";
		modelDir = "model";
		resultDir = "result";

		for (std::size_t i=0; i<_subDir.size(); i++) {
			subDir.push_back(_subDir[i]);
		}

		generateDirectory();
	}

	void generateDirectory() {
		boost::filesystem::create_directory(rootDir);
		boost::filesystem::create_directory(rootDir/imageDir);
		boost::filesystem::create_directory(rootDir/modelDir);
		boost::filesystem::create_directory(rootDir/rawDir);
		boost::filesystem::create_directory(rootDir/resultDir);

		// directory 생성
		for (std::size_t i=0;i<subDir.size(); i++) {
			boost::filesystem::create_directory(rootDir/imageDir/subDir[i]);
			boost::filesystem::create_directory(rootDir/resultDir/subDir[i]);
		}
	}

	int rename(boost::filesystem::path& currentDir, std::string ext=".jpg") {

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

	static void gray2Contours(cv::Mat& gray, std::vector<std::vector<cv::Point>>& contours) {
		cv::Mat mask;
//		cv::threshold(gray, mask, 100, 255, CV_THRESH_BINARY);
		cv::adaptiveThreshold(gray, mask, 255, CV_ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 55, 10);
		cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	}

	static int findRectangle(std::vector<std::vector<cv::Point>> contours, std::vector<std::vector<cv::Point2f>>& corner) {

		int idx=0, maxIdx=0, maxArea=0, area;
		std::vector<cv::Point> approx;

		for (std::size_t i=0; i<contours.size(); i++) {
			cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);
			area = cv::contourArea(contours[i]);
			if (std::fabs(area) < 50 || !cv::isContourConvex(approx))
				continue;

			if (approx.size() == 4) {

				std::vector<cv::Point2f> sampleCorner;
				for (std::size_t j=0; j<approx.size(); j++)
					sampleCorner.push_back(cv::Point2f(approx[j].x, approx[j].y));
				corner.push_back(sampleCorner);

				if (area>maxArea) {
					maxIdx=idx++;
					maxArea=area;
				}
			}
		}
		return maxIdx;
	}

	static void sortCorners(std::vector<cv::Point2f>& srcCorner)
	{
		cv::Point2f center(0,0);
		std::vector<cv::Point2f> top, bot;

		for (std::size_t i=0; i<srcCorner.size(); i++) {
		    center.x+=(srcCorner[i].x/4.0);
		    center.y+=(srcCorner[i].y/4.0);
		}

		for (std::size_t i = 0; i < srcCorner.size(); i++)
		{
			if (srcCorner[i].y < center.y)
				top.push_back(srcCorner[i]);
			else bot.push_back(srcCorner[i]);
		}

		cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
		cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
		cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
		cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];

		srcCorner[0]=tl;
		srcCorner[1]=tr;
		srcCorner[2]=br;
		srcCorner[3]=bl;
	}

	static void sortCorners(std::vector<std::vector<cv::Point2f>>& corner)
	{
		for (std::size_t i=0; i<corner.size(); i++) {
			sortCorners(corner[i]);
		}
	}

	void getCorners(cv::Mat& mask, std::vector<std::vector<cv::Point2f>>& corner) {
		std::vector<std::vector<cv::Point>> contours; //
		cv::findContours(mask, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
		findRectangle(contours, corner);
		sortCorners(corner);
	}

	void displayCorner(cv::Mat& RGB, std::vector<std::vector<cv::Point2f>>& corner) {
		for (std::size_t i=0; i<corner.size(); i++) {
			for (std::size_t j=0; j<corner[i].size(); j++) {
				cv::circle(RGB, cv::Point(corner[i][j].x, corner[i][j].y), 2, cv::Scalar(0,0,200), 4);
			}
		}
	}

	std::string extractFilename(boost::filesystem::path& currentDir, std::string ext) {
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

	std::string extractFilename(boost::filesystem::path& currentDir, std::string ext, std::string filename) {
		// filename이 없을 경우, 찾아준다.
		std::string path = currentDir.string() + "/" + filename + ext;
		return path;
	}

	static bool extractImage(cv::Mat& src, cv::Mat& rotated, std::vector<cv::Point2f>& srcCorner, std::vector<cv::Point2f>& refPt) {
		cv::Mat extracted = cv::Mat(cv::Size(64,64), 0);

		cv::Mat transmat = cv::getPerspectiveTransform(srcCorner, refPt);
		cv::warpPerspective(src, extracted, transmat, extracted.size());

		rotated = extracted(cv::Rect(16,16,32,32));
		return true;
	}

	void imageSave(std::string path, cv::Mat& image) {
		cv::imwrite(path, image);
	}

	void savePattern() {
		auto saveDir = rootDir/modelDir;
		for (std::size_t i=0; i<subDir.size(); i++) {
			int count=0;
			std::string ext(".jpg");
			boost::filesystem::path imgPath = rootDir/imageDir/subDir[i];
			rename(imgPath);
			cv::Mat sum = cv::Mat::zeros(cv::Size(32,32), CV_32FC1);
			for (boost::filesystem::directory_iterator iter(imgPath); iter!=boost::filesystem::directory_iterator(); iter++) {
				// image를 불러오고, 평균값을 구하여 저장.
				cv::Mat temp;
				cv::Mat selected = cv::imread(iter->path().c_str(), 0);
				selected.convertTo(temp, CV_32FC1);
				cv::add(sum, temp, sum);
				count++;
			}
			sum/=(float)count;
			sum.convertTo(sum, CV_8UC1);
			std::string imagefilePath = extractFilename(saveDir, ext, subDir[i].c_str());
			cv::imwrite(imagefilePath, sum);
		}
	}

	void classify(cv::Mat& rotated, int& label, std::vector<cv::Point2f>& srcCorner) {

		label=0;
		int direction=0;
		double minSum = 999999.;
		auto loadDir = rootDir/modelDir;
		for (std::size_t i=0; i<subDir.size(); i++) {
			std::string filename = subDir[i].string()+".jpg";
			auto file = rootDir/modelDir/filename;
			cv::Mat reference = cv::imread(file.c_str(), 0);

			cv::Point center(16,16);
			float angle=0;
			for (auto j=0; j<4; j++) {
				cv::Mat rotReference, diff;
				rotation(reference, rotReference, angle);

				// calibrated와 rotReference의 차이를 구한다. 그리고, 전체 합을 구한다.
				cv::absdiff(rotated, rotReference, diff);
				double sum = cv::sum(diff)[0]/diff.total();
				if (sum<minSum) {
					minSum=sum;
					label=i;
					direction=j;
				}
				angle-=90.;
			}
		}

		if (minSum>100) label=-1;
		int refIdx=0;
		std::vector<cv::Point2f> tempCorner(4);
		for (auto i=0;i<4;i++) {
			tempCorner[i] = srcCorner[(refIdx+direction)%4];
			refIdx++;
		}
		srcCorner = tempCorner;
		rotation(rotated, rotated, direction*90);
	}

	void rotation(cv::Mat& src, cv::Mat& rotated, double angle) {
		cv::Point center(src.cols/2, src.rows/2);
		cv::Mat rotMat = cv::getRotationMatrix2D(center, angle, 1.0);
		cv::warpAffine(src, rotated, rotMat, src.size());
	}

	void setAbsolutePt(std::vector<std::vector<cv::Point2f>>& _absolutePt) {
		absolutePt.clear();
		absolutePt = _absolutePt;
	}

	void getPerspectiveTransform(std::vector<cv::Point2f>& srcCorner, std::vector<cv::Point2f>& dstCorner, cv::Mat& transmat) {
		int numData = srcCorner.size();
		if (numData==4) {
			transmat = cv::getPerspectiveTransform(srcCorner, dstCorner);
		}
		else {
			cv::Mat srcMat = cv::Mat::ones(cv::Size(3, numData), CV_32FC1);
			cv::Mat dstMat = cv::Mat::ones(cv::Size(3, numData), CV_32FC1);

			cv::Mat invSrcMat(numData, 3, CV_32FC1);

			for (auto i=0; i<numData; i++) {
				srcMat.at<float>(0,i) = srcCorner[i].x;
				srcMat.at<float>(1,i) = srcCorner[i].y;
				dstMat.at<float>(0,i) = dstCorner[i].x;
				dstMat.at<float>(1,i) = dstCorner[i].y;
			}

			cv::invert(srcMat, invSrcMat, cv::DECOMP_SVD);
			transmat = invSrcMat*dstMat;
		}
	}

	void putText(cv::Mat& image, cv::Point2f& ltCorner, std::string text, cv::Scalar& bgColor, double alpha=0.7) {
		int baseline=0;
		cv::Size textSize = cv::getTextSize(text, cv::FONT_ITALIC, 0.5, 1, &baseline);
		int topLeft_x = std::min((int)ltCorner.x, image.cols-textSize.width);
		int topLeft_y = std::min((int)ltCorner.y, image.rows-textSize.height);
		if (topLeft_y-textSize.height<0) topLeft_y=textSize.height;

		cv::Mat roi = image(cv::Rect(topLeft_x, topLeft_y-textSize.height, textSize.width, textSize.height));
		cv::Mat overlaped(roi.size(), CV_8UC3, bgColor);
		cv::addWeighted(overlaped, alpha, roi, 1.0-alpha, 0.0, roi);
		cv::putText(image, text, cv::Point(topLeft_x, topLeft_y), cv::FONT_ITALIC, 0.5, cv::Scalar(255,255,255), 1, 1);
	}

public:
	boost::filesystem::path rootDir;
	boost::filesystem::path imageDir;
	boost::filesystem::path featureDir;
	boost::filesystem::path modelDir;
	boost::filesystem::path rawDir;
	boost::filesystem::path resultDir;
	std::vector<boost::filesystem::path> subDir;

	std::vector<std::vector<cv::Point2f>> absolutePt;
};

}
