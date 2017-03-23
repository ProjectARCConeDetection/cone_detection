cmake_minimum_required(VERSION 2.8.3)
project(cone_detection)


find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
	sensor_msgs
	roscpp
 	pcl_conversions
  	pcl_ros
	catkin_simple REQUIRED
)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
    src/laser_detection.cpp
)

cs_add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${PROJECT_NAME})

cs_install()
cs_export()




