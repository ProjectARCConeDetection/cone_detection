#ifndef CONE_DETECTION_GPS_HANDLER_HPP
#define CONE_DETECTION_GPS_HANDLER_HPP

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

namespace cone_detection{

class GPSHandler{
public:
	GPSHandler();
	~GPSHandler();
	std::vector<double> get2DPosition();
	void set2DPosition(std::vector<double> position);

private:
	std::vector<double> position_;
};
}
#endif