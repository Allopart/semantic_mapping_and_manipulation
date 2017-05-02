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
plaincode::Object objects; 
plaincode::Color color;

void training() {
	// training mode
	cv::Mat trainingMat; 
	cv::Mat labelMat; 

	// object training
	for (std::size_t i=0; i<objects.subDir.size(); i++) {
      plaincode::rename(objects.rootDir/objects.imageDir/objects.subDir[i]); 
		}
		// lua_parameter
  objects.lua_saveModel(lua);
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
  if (setup_files[0]=='~') setup_files.replace(0,1,getenv("HOME")); 

  luaL_dofile(lua, std::string(lua_files + "/0_parameter.lua").c_str());

  // other_things setting // 
	objects.setup(lua, lua_files, setup_files, imgRect); 

	boost::filesystem::path objectSaveDir =  objects.rootDir/objects.rawDir; 
	std::string ext = "_result.jpg";

  for (std::size_t i=0; i<objects.subDir.size(); i++)
          plaincode::rename(objects.rootDir/objects.resultDir/objects.subDir[i], ext); 
  lua_pushstring(lua, objects.rootDir.c_str()); lua_setglobal(lua, "class");
  luaL_dofile(lua, std::string(lua_files + "/4_loadModel.lua").c_str());
	
	training(); 

	lua_close(lua);
  return 0;
}

