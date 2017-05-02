#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

namespace plaincode {
// HSV color를 10개로 제한. 0~9
class Color {
public:
  Color();
	Color(std::string _colorFile);

  void setFile(std::string);

	void display(cv::Mat& RGB);
	void modifyColor(int keyCode);
	void addColor();
  void deleteColor(); 
	void save();
	void showColorTable();
	void RGB2Mask(cv::Mat& RGB, cv::Mat& Mask, int idxColor);
	void HSV2Mask(cv::Mat& HSV, cv::Mat& Mask, int idxColor);
  static void RGB2Mask(cv::Mat& RGB, cv::Mat& Mask, cv::Scalar_<int>& hsvMin, cv::Scalar_<int>& hsvMax); 
  static void HSV2Mask(cv::Mat& HSV, cv::Mat& Mask, cv::Scalar_<int>& hsvMin, cv::Scalar_<int>& hsvMax); 

  static void RGB2MaskAll(cv::Mat& RGB, std::vector<cv::Mat>& Masks, std::vector<cv::Scalar_<int>>& hsvMin, std::vector<cv::Scalar_<int>>& hsvMax, std::vector<int>& isObject);
  void calcHueHist(cv::Mat& src, cv::Mat& dst, const cv::Mat& mask);
	void calcBackProject(cv::Mat& src, cv::Mat&_h_hist, cv::Mat& dst);
  void tune(cv::Mat& RGB);
  
  std::vector<int> isObject; 
  std::vector<std::string> colorName; 
	std::vector<bool> isDisplayed;
	std::string colorFile;
	std::fstream colorStream;
	std::vector<cv::Scalar_<int>> hsvMin, hsvMax;
};
}
