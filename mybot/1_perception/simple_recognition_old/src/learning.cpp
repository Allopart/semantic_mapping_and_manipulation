// STL Header
#include <iostream>
#include <numeric>
#include <memory>
#include "cbf.h"

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

// Eigen Header
#include <eigen3/Eigen/Dense>

// plaincode header
#include "Core.hpp"
#include "Color.hpp"
#include "Object.hpp"
#include "patch.hpp"

// Boost Header
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

lua_State *lua;
plaincode::Object patched_things;
plaincode::Object colored_things;
plaincode::Object other_things; 
plaincode::Color color;
std::vector<std::string> nameTags;
std::vector<cv::Scalar> allLut; 
std::vector<plaincode::Blob> objects; 

void training() {
	// training mode
	cv::Mat trainingMat; 
	cv::Mat labelMat; 
	
	for (std::size_t i=0; i<patched_things.subDir.size(); i++) {
		boost::filesystem::path imgDir = patched_things.rootDir/patched_things.imageDir/patched_things.subDir[i];
		boost::filesystem::path featDir = patched_things.rootDir/patched_things.featureDir/patched_things.subDir[i];
		patched_things.rename(imgDir);
	}

	patched_things.lua_saveModel(lua); 


	for (std::size_t i=0; i<colored_things.subDir.size(); i++) {
		boost::filesystem::path imgDir = colored_things.rootDir/colored_things.imageDir/colored_things.subDir[i];
		boost::filesystem::path featDir = colored_things.rootDir/colored_things.featureDir/colored_things.subDir[i];
		colored_things.rename(imgDir);
	}
	colored_things.lua_saveModel(lua); 

	// object training
	for (std::size_t i=0; i<other_things.subDir.size(); i++) {
			boost::filesystem::path imgDir = other_things.rootDir/other_things.imageDir/other_things.subDir[i];
			boost::filesystem::path featDir = other_things.rootDir/other_things.featureDir/other_things.subDir[i];
			other_things.rename(imgDir);
		}
		// lua_parameter
	other_things.lua_saveModel(lua);
	std::cout << "AFTER savemodel" << std::endl; 

}


int main(int argc, char** argv){
  ros::init(argc, argv, "learning");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

	lua = luaL_newstate();
	luaL_openlibs(lua);

  plaincode::th::distAbove=0; // 실제 값은 -100
  plaincode::th::thSize=100;
  plaincode::th::thC=20;
  cv::Rect imgRect(20,20,280,200); 

  std::string lua_files, object_files, color_file, setup_files; 
  nh.getParam("lua_files", lua_files);
	nh.getParam("setup_files", setup_files); 

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
	for (std::size_t i=0; i<patched_things.subDir.size(); i++)
		patched_things.rename(patched_things.rootDir/patched_things.resultDir/patched_things.subDir[i], ext);
    lua_pushstring(lua, patched_things.rootDir.c_str()); lua_setglobal(lua, "class");
  luaL_dofile(lua, std::string(lua_files + "/4_loadModel.lua").c_str());

	for (std::size_t i=0; i<colored_things.subDir.size(); i++)
		colored_things.rename(colored_things.rootDir/colored_things.resultDir/colored_things.subDir[i], ext);
    lua_pushstring(lua, colored_things.rootDir.c_str()); lua_setglobal(lua, "class");
  luaL_dofile(lua, std::string(lua_files + "/4_loadModel.lua").c_str());

	for (std::size_t i=0; i<other_things.subDir.size(); i++)
		other_things.rename(other_things.rootDir/other_things.resultDir/other_things.subDir[i], ext);
    lua_pushstring(lua, other_things.rootDir.c_str()); lua_setglobal(lua, "class");
  luaL_dofile(lua, std::string(lua_files + "/4_loadModel.lua").c_str());
	
	training(); 

	lua_close(lua);
  return 0;
}
