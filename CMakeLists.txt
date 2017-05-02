cmake_minimum_required(VERSION 2.8.3)
project(semantic_mapping)

set(PCL_DIR "/usr/share/pcl-1.7/") 

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_visualization
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  PCL
  message_filters
  tf
  sensor_msgs
  message_generation
  agile_grasp
	geometry_msgs
	diagnostic_msgs
	trajectory_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost 1.54 COMPONENTS program_options REQUIRED)
find_package(octomap REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LabelledPointCloud2.msg
	PanTilt.msg
	BoundingBox_data.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
	diagnostic_msgs
	trajectory_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES table_cleaner 
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy std_msgs message_runtime tf message_runtime agile_grasp geometry_msgs moveit_core  moveit_ros_planning_interface diagnostic_msgs trajectory_msgs 
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(semantic_mapping
#   src/${PROJECT_NAME}/semantic_mapping.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(semantic_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(semantic_mapping_node src/semantic_mapping_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(semantic_mapping_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(semantic_mapping_node
#   ${catkin_LIBRARIES}
# )




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS semantic_mapping semantic_mapping_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_semantic_mapping.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#add_definitions(-std=c++11)


add_executable( mask_data_2_labpcl src/mask_data_2_labpcl.cpp)
target_link_libraries(mask_data_2_labpcl ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES})

add_executable( select_object_to_grasp src/select_object_to_grasp.cpp)
target_link_libraries(select_object_to_grasp ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})


add_executable( ICP_registration src/ICP_registration.cpp)
target_link_libraries(ICP_registration ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable( move_arm_to_grasp src/move_arm_to_grasp.cpp)
target_link_libraries(move_arm_to_grasp ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable( BB_for_reg_pcl src/BB_for_reg_pcl.cpp)
target_link_libraries(BB_for_reg_pcl ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})


## SEGMENTATION ALGORITHMS

add_executable( class_labpcl_seg_all src/seg_algorithms/class_labpcl_seg_all.cpp)
target_link_libraries(class_labpcl_seg_all ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable( class_labpcl_seg_h src/seg_algorithms/class_labpcl_seg_h.cpp)
target_link_libraries(class_labpcl_seg_h ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable( class_labpcl_seg_v src/seg_algorithms/class_labpcl_seg_v.cpp)
target_link_libraries(class_labpcl_seg_v ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable( class_labpcl_seg_outlier src/seg_algorithms/class_labpcl_seg_outlier.cpp)
target_link_libraries(class_labpcl_seg_outlier ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable( class_labpcl_seg_cylinder src/seg_algorithms/class_labpcl_seg_cylinder.cpp)
target_link_libraries(class_labpcl_seg_cylinder ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable( class_labpcl_seg_sphere src/seg_algorithms/class_labpcl_seg_sphere.cpp)
target_link_libraries(class_labpcl_seg_sphere ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable( class_labpcl_seg_region src/seg_algorithms/class_labpcl_seg_region.cpp)
target_link_libraries(class_labpcl_seg_region ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable( class_labpcl_seg_nothing src/seg_algorithms/class_labpcl_seg_nothing.cpp)
target_link_libraries(class_labpcl_seg_nothing ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

## MAYBE USEFULL

add_executable( class_labpcl_seg src/maybe_useful/class_labpcl_seg.cpp)
target_link_libraries(class_labpcl_seg ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable( test_gripper src/maybe_useful/test_gripper.cpp)
target_link_libraries(test_gripper ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable( mask_data_2_pcl src/maybe_useful/mask_data_2_pcl.cpp)
target_link_libraries(mask_data_2_pcl ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES})

add_executable( class_pcl_seg src/maybe_useful/class_pcl_seg.cpp)
target_link_libraries(class_pcl_seg ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable( mask_pcl_segm src/maybe_useful/mask_pcl_segm.cpp)
target_link_libraries(mask_pcl_segm ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable( crop_img_depth src/maybe_useful/crop_img_depth.cpp)
target_link_libraries(crop_img_depth ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES})

add_executable( cylindrical_segm src/maybe_useful/cylindrical_segm.cpp)
target_link_libraries(cylindrical_segm ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES})

#add_executable( my_eucl src/my_eucl.cpp)
#target_link_libraries(my_eucl ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES})


#add_executable( sem_img src/nothing/sem_img.cpp)
#target_link_libraries(sem_img ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

#add_executable( region_growth src/region_growth.cpp)
#target_link_libraries(region_growth ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES})

#add_executable( rgb_region_growth src/rgb_region_growth.cpp)
#target_link_libraries(rgb_region_growth ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES})

#add_executable( eucl_clustering src/eucl_clustering.cpp)
#target_link_libraries(eucl_clustering ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES})

