set(MOVEIT_LIB_NAME moveit_depth_image_octomap_updater)

add_library(${MOVEIT_LIB_NAME}_core
  src/depth_image_octomap_updater.cpp
  )

target_link_libraries(${MOVEIT_LIB_NAME}_core moveit_lazy_free_space_updater moveit_mesh_filter moveit_occupancy_map_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(${MOVEIT_LIB_NAME}_core sensor_msgs_generate_cpp)

add_library(${MOVEIT_LIB_NAME} src/updater_plugin.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}_core ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
