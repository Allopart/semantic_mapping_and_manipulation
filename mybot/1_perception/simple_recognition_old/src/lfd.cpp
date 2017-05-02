// stl header
#include <iostream>
#include <numeric>
#include <memory>
#include "cbf.h"

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

// eigen header
#include <eigen3/Eigen/Dense>

// plaincode header
#include "Core.hpp"
#include "Color.hpp"
#include "Object.hpp"
#include "patch.hpp"

// boost header
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

lua_State *lua;
boost::filesystem::path demonstrationDir; 
int numAct; 
int numObj; 

void lfd(std::string _luaDir) {
        // training mode
        std::cout << "train trajectory" << std::endl; 
        std::fstream fileTrajectory; 
        int iter_count=0;  
     
        // load text file

        lua_newtable(lua); 
        for (boost::filesystem::directory_iterator iter(demonstrationDir); 
            iter!=boost::filesystem::directory_iterator(); iter++, iter_count++) {
            
            std::vector<std::vector<double>> sequences; 
          
            std::vector<double> sequence(numObj+3);
            std::string filename = (iter->path()/"Trajectory.txt").string();
            fileTrajectory.open(filename.c_str(), std::ios::in); 

            while(!fileTrajectory.eof()){
                    for (std::size_t i=0; i<numObj+3; i++) 
                            fileTrajectory >> sequence[i]; 
                    sequences.push_back(sequence); 
            }

            fileTrajectory.close(); 

            lua_pushnumber(lua, iter_count+1); 
            lua_newtable(lua); 
            for (std::size_t i=0; i<sequences.size()-1; i++) {
                lua_pushnumber(lua, i+1); 
                lua_newtable(lua); 
                for (std::size_t j=0; j<numObj+3; j++) {
                    lua_pushnumber(lua, j+1); 
                    lua_pushnumber(lua, sequences[i][j]); 
                    lua_settable(lua, -3); 
                }
                lua_settable(lua, -3); 
            }
            lua_settable(lua, -3); 
            std::cout << "sequences size = " << sequences.size()-1 << std::endl; 
        }

        lua_setglobal(lua, "trajectories"); 
        boost::filesystem::path luaDir = _luaDir; 
        std::cout << (luaDir/"train.lua").c_str() << std::endl; 
        luaL_dofile(lua, (luaDir/"train.lua").c_str()); 

        std::cout << "after savemodel" << std::endl; 
}

int main(int argc, char** argv){
        ros::init(argc, argv, "lfd");
        ros::NodeHandle nh("~");
        ros::NodeHandle node_handle;

        demonstrationDir = boost::filesystem::path("/home/yongho/Demonstration"); 

        lua = luaL_newstate();
        luaL_openlibs(lua);

        std::string luaDir; 
        nh.getParam("lua_files", luaDir);

//        numAct = 5;
        numObj = 6; 
        lfd(luaDir); 

        lua_close(lua);
        return 0;
}
