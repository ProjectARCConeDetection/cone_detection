cmake_minimum_required(VERSION 2.8.3)
project(cone_detection)

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
	roscpp
	rospy
	geometry_msgs
	nav_msgs
	sensor_msgs
 	pcl_conversions
  	pcl_ros
  	OpenCV  REQUIRED
  	eigen_catkin
	catkin_simple REQUIRED
)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
	src/grid_mapper.cpp 
	src/image_handler.cpp
    src/laser_detection.cpp
    src/tools.cpp
)

cs_add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${PROJECT_NAME})

cs_install()
cs_export()




