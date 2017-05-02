#include "Color.hpp"


plaincode::Color::Color(){}

plaincode::Color::Color(std::string _colorFile):colorFile(_colorFile), colorStream(colorFile.c_str(), std::ios::in)  {
	std::string _colorName;
	cv::Scalar_<int> _hsvMin, _hsvMax;
  int _isObject; 

	if (!colorStream.is_open()) {
		std::ofstream tempStream(colorFile);
		tempStream.close();
    colorStream.open(colorFile.c_str(), std::ios::in); 
	}

	colorStream >> _colorName;

	while (!colorStream.eof()) {

		colorStream >> _colorName >> _hsvMin.val[0] >> _hsvMin.val[1] >> _hsvMin.val[2] >> _isObject;
		colorStream >> _colorName >> _hsvMax.val[0] >> _hsvMax.val[1] >> _hsvMax.val[2] >> _isObject; 

		isDisplayed.push_back(false);
		colorName.push_back(_colorName);
		hsvMin.push_back(_hsvMin);
		hsvMax.push_back(_hsvMax);
    isObject.push_back(_isObject); 
	}

	colorStream.close();
	showColorTable();
}

void plaincode::Color::setFile(std::string _colorFile){

  int _isObject;

  colorFile = _colorFile;
  colorStream.open(colorFile.c_str(), std::ios::in); 
	std::string _colorName;
	cv::Scalar_<int> _hsvMin, _hsvMax;

	if (!colorStream.is_open()) {
		std::ofstream tempStream(colorFile);
		tempStream.close();
		colorStream.open(colorFile.c_str(), std::ios::in); 
	}

	colorStream >> _colorName;

	while (colorStream >> _colorName) {

		colorStream >> _hsvMin.val[0] >> _hsvMin.val[1] >> _hsvMin.val[2] >> _isObject;
		colorStream >> _colorName >> _hsvMax.val[0] >> _hsvMax.val[1] >> _hsvMax.val[2] >> _isObject;

		isDisplayed.push_back(false);
		colorName.push_back(_colorName);
		hsvMin.push_back(_hsvMin);
		hsvMax.push_back(_hsvMax);
    isObject.push_back(_isObject); 
	}

	colorStream.close();
}
void plaincode::Color::display(cv::Mat& RGB) {
			cv::Mat HSV, Mask;
			cv::cvtColor(RGB, HSV, cv::COLOR_BGR2HSV);

			for (std::size_t i=0; i<isDisplayed.size(); i++) {
				if (isDisplayed[i]) {
					HSV2Mask(HSV, Mask, i);
					std::string displayName = colorName[i]+"_filtered";
					cv::imshow(displayName.c_str(), Mask);
          cv::moveWindow(displayName.c_str(), 100, 400); 
				}
			}

		}

void plaincode::Color::modifyColor(int keyCode) {
			int numColor = isDisplayed.size();

			if (keyCode<numColor && keyCode>=0 && keyCode<=9) {
				if (!isDisplayed[keyCode]) {
					cv::namedWindow(colorName[keyCode], 1);
					cv::createTrackbar("Hue(Min)", colorName[keyCode], &hsvMin[keyCode].val[0], 179);
					cv::createTrackbar("Hue(Max)", colorName[keyCode], &hsvMax[keyCode].val[0], 179);
					cv::createTrackbar("Sat(Min)", colorName[keyCode], &hsvMin[keyCode].val[1], 255);
					cv::createTrackbar("Sat(Max)", colorName[keyCode], &hsvMax[keyCode].val[1], 255);
					cv::createTrackbar("Val(Min)", colorName[keyCode], &hsvMin[keyCode].val[2], 255);
					cv::createTrackbar("Val(Max)", colorName[keyCode], &hsvMax[keyCode].val[2], 255);
          cv::createTrackbar("isObject", colorName[keyCode], &isObject[keyCode], 1); 
          cv::moveWindow(colorName[keyCode], 400, 100); 
				}
				else {
					// save
					save();
					cv::destroyWindow(colorName[keyCode]);
					cv::destroyWindow(colorName[keyCode]+"_filtered");
				}
				isDisplayed[keyCode]=!isDisplayed[keyCode];
			}
      else std::cout << "Out of color index!" << std::endl;
}

void plaincode::Color::addColor() { // 'a'를 눌렀을  때 호출.
        std::cout << "Type color's name:" << std::endl;
        std::string _colorName;
        std::cin >> _colorName;
        isDisplayed.push_back(false);
        colorName.push_back(_colorName);
        hsvMin.push_back(cv::Scalar_<int>(0,0,0));
        hsvMax.push_back(cv::Scalar_<int>(179,255,255));
        isObject.push_back(1); 
        save();
}

void plaincode::Color::deleteColor() {
        for (std::size_t i=0; i<isDisplayed.size(); i++) {
                cv::destroyWindow(colorName[i]); 
                cv::destroyWindow(colorName[i]+"_filtered"); 
        }

        for (std::size_t i=0; i<isDisplayed.size();) {
                if (isDisplayed[i]) {
                        hsvMin.erase(hsvMin.begin()+i);
                        hsvMax.erase(hsvMax.begin()+i); 
                        colorName.erase(colorName.begin()+i);
                        isObject.erase(isObject.begin()+i);      
                        isDisplayed.erase(isDisplayed.begin()+i);
                }
                else i++;
        }
        save();
}

void plaincode::Color::save() {
        colorStream.open(colorFile, std::ios::out);
        colorStream << "ColorTable";
        for (std::size_t i=0; i<isDisplayed.size(); i++) {
                colorStream << std::endl;
                colorStream << colorName[i] << "\t" << hsvMin[i].val[0] << "\t" << hsvMin[i].val[1] << "\t" << hsvMin[i].val[2] << "\t" << isObject[i] << std::endl;
                colorStream << colorName[i] << "\t" << hsvMax[i].val[0] << "\t" << hsvMax[i].val[1] << "\t" << hsvMax[i].val[2] << "\t" << isObject[i]; 
        }
        colorStream.close();
}

void plaincode::Color::showColorTable() {
        std::cout << "Current color table" << std::endl;
        std::cout << "-------------------" << std::endl;

        for (std::size_t i=0; i<isDisplayed.size(); i++) {
                std::cout << i << ". " << colorName[i] << std::endl;
        }
        std::cout << "-------------------" << std::endl;
}



void plaincode::Color::RGB2Mask(cv::Mat& RGB, cv::Mat& Mask, cv::Scalar_<int>& hsvMin, cv::Scalar_<int>& hsvMax) {

        cv::Mat HSV;
        cv::cvtColor(RGB, HSV, cv::COLOR_BGR2HSV);
        HSV2Mask(HSV, Mask, hsvMin, hsvMax);
}

//void plaincode::Color::RGB2Mask(cv::Mat& RGB, cv::Mat& Mask, int idxColor) {
//        cv::Mat HSV;
//        cv::cvtColor(RGB, HSV, cv::COLOR_BGR2HSV);
//        HSV2Mask(HSV, Mask, idxColor);
//}

void plaincode::Color::HSV2Mask(cv::Mat& HSV, cv::Mat& Mask, cv::Scalar_<int>& hsvMin, cv::Scalar_<int>& hsvMax) {
        if (hsvMin.val[0]>hsvMax.val[0]) {
                cv::Scalar_<int> temp_min=hsvMin, temp_max=hsvMax;
                cv::Mat HSV1, HSV2;
                temp_min.val[0]=0;
                temp_max.val[0]=179;

                cv::inRange(HSV, cv::Scalar(temp_min.val[0], temp_min.val[1], temp_min.val[2]), cv::Scalar(hsvMax.val[0], hsvMax.val[1], hsvMax.val[2]), HSV1);
                cv::inRange(HSV, cv::Scalar(hsvMin.val[0], hsvMin.val[1], hsvMin.val[2]), cv::Scalar(temp_max.val[0], temp_max[1], temp_max[2]), HSV2);

                Mask = HSV1|HSV2;
        }
        else {
                cv::inRange(HSV, cv::Scalar(hsvMin.val[0],hsvMin.val[1], hsvMin.val[2]), cv::Scalar(hsvMax.val[0],hsvMax.val[1], hsvMax.val[2]), Mask);
        }
}

void plaincode::Color::RGB2Mask(cv::Mat& RGB, cv::Mat& Mask, int idxColor) {
			cv::Mat HSV;
			cv::cvtColor(RGB, HSV, cv::COLOR_BGR2HSV);
			HSV2Mask(HSV, Mask, idxColor);
		}

void plaincode::Color::HSV2Mask(cv::Mat& HSV, cv::Mat& Mask, int idxColor) {
  if (idxColor>=hsvMin.size()) {

      ROS_ERROR("Out of index in palette."); 
      ROS_ERROR("Check out your palette file."); 
      exit(0); 
  }
	if (hsvMin[idxColor].val[0]>hsvMax[idxColor].val[0]) {
		cv::Scalar_<int> temp_min=hsvMin[idxColor], temp_max=hsvMax[idxColor];
		cv::Mat HSV1, HSV2;
		temp_min.val[0]=0;
		temp_max.val[0]=179;

		cv::inRange(HSV, cv::Scalar(temp_min.val[0], temp_min.val[1], temp_min.val[2]), cv::Scalar(hsvMax[idxColor].val[0], hsvMax[idxColor].val[1], hsvMax[idxColor].val[2]), HSV1);
		cv::inRange(HSV, cv::Scalar(hsvMin[idxColor].val[0], hsvMin[idxColor].val[1], hsvMin[idxColor].val[2]), cv::Scalar(temp_max.val[0], temp_max[1], temp_max[2]), HSV2);

		Mask = HSV1|HSV2;
	}
	else {
		cv::inRange(HSV, cv::Scalar(hsvMin[idxColor].val[0],hsvMin[idxColor].val[1], hsvMin[idxColor].val[2]), cv::Scalar(hsvMax[idxColor].val[0],hsvMax[idxColor].val[1], hsvMax[idxColor].val[2]), Mask);
	}
}



void plaincode::Color::RGB2MaskAll(cv::Mat& RGB, std::vector<cv::Mat>& Masks, std::vector<cv::Scalar_<int>>& hsvMin, std::vector<cv::Scalar_<int>>& hsvMax, std::vector<int>& isObject) {

  if (hsvMin.size()==0) {
    std::cout << "The palette list is empty." << std::endl; 
    std::cout << "Add the \" palette \"" << std::endl; 
  }

  Masks.clear();
  cv::Mat HSV;
  cv::cvtColor(RGB, HSV, cv::COLOR_BGR2HSV);

  for (std::size_t i=0; i<isObject.size(); i++) {
    if (isObject[i]==0) continue;

    if (hsvMin[i].val[0]>hsvMax[i].val[0]) {
      cv::Scalar_<int> temp_min=hsvMin[i], temp_max=hsvMax[i];
      cv::Mat HSV1, HSV2;
      temp_min.val[0]=0;
      temp_max.val[0]=179;

      cv::inRange(HSV, cv::Scalar(temp_min.val[0], temp_min.val[1], temp_min.val[2]), cv::Scalar(hsvMax[i].val[0], hsvMax[i].val[1], hsvMax[i].val[2]), HSV1);
      cv::inRange(HSV, cv::Scalar(hsvMin[i].val[0], hsvMin[i].val[1], hsvMin[i].val[2]), cv::Scalar(temp_max.val[0], temp_max[1], temp_max[2]), HSV2);

      Masks.push_back(HSV1|HSV2);
    }
    else {
      cv::Mat temp;
      cv::inRange(HSV, cv::Scalar(hsvMin[i].val[0],hsvMin[i].val[1], hsvMin[i].val[2]), cv::Scalar(hsvMax[i].val[0],hsvMax[i].val[1], hsvMax[i].val[2]), temp);
      Masks.push_back(temp);
    }

    cv::medianBlur(Masks.back(), Masks.back(), 3);
  }
}

void plaincode::Color::calcHueHist(cv::Mat& src, cv::Mat& h_hist, const cv::Mat& mask) {

	cv::Mat hsv;
	std::vector<cv::Mat> splited;
	int histSize=180;
	float range[] = {0,(float)histSize};
	const float* ranges = {range};
	cv::cvtColor(src, hsv, CV_BGR2HSV);
	cv::split(hsv, splited);
	cv::calcHist(&splited[0], 1, 0, mask, h_hist, 1, &histSize, &ranges, true, false);
	cv::normalize(h_hist, h_hist, 0, 255, cv::NORM_MINMAX, -1, cv::Mat());
//	cv::calcBackProject(&splited[0], 1, 0, h_hist, dst, &ranges, 1, true);
}

void plaincode::Color::calcBackProject(cv::Mat& src, cv::Mat& h_hist, cv::Mat& dst) {
	cv::Mat hsv;
	std::vector<cv::Mat> splited;
	int histSize=180;
	float range[] = {0,(float)histSize};
	const float* ranges = {range};
	cv::cvtColor(src, hsv, CV_BGR2HSV);
	cv::split(hsv, splited);
	cv::calcBackProject(&splited[0], 1, 0, h_hist, dst, &ranges, 1, true);
}

void plaincode::Color::tune(cv::Mat& RGB) {
    char key;

    // Check white balance and off
    while(true) {
        cv::imshow("RGB", RGB);
        key=cv::waitKey(1);
        if (key=='q') {
            //rgbd_sensor.disableAutoFunction();
            cv::destroyAllWindows();
            break;
        }
   }

    while(true) {
        cv::imshow("originRGB", RGB);
        cv::imshow("RGB", RGB);

        display(RGB);

        key = cv::waitKey(1);

        if (key-'0'>=0 && key-'0'<=9) {
           modifyColor(key-'0');
        }
        else if (key=='a') {
            addColor();
        }
        else if (key=='q') {
            break;
        }
    }
}

