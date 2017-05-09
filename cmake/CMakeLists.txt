cmake_minimum_required(VERSION 2.8.3)
project(cone_detection)

set (OpenCV_DIR /home/sele/opencv/release)
set(OpenCV_FOUND 1)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	geometry_msgs
	nav_msgs
	sensor_msgs
 	pcl_conversions
  	pcl_ros
  	OpenCV REQUIRED
  	cv_bridge
	genmsg
    message_generation
)

add_message_files(
  FILES
  Label.msg
  Path.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS 
	message_runtime
	sensor_msgs
  std_msgs
)

#include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

#add_library(${PROJECT_NAME}
#	src/grid_mapper.cpp 
#	src/image_handler.cpp
#    src/laser_detection.cpp
#)

#add_executable(main src/main.cpp)
#target_link_libraries(main ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES})

#add_executable(testing_transformation test/transformation.cpp)
#target_link_libraries(testing_transformation ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBS})

