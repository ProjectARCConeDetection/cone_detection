#include <cone_detection/gps_handler.hpp>

namespace cone_detection{

GPSHandler::GPSHandler(){
	//Assumption: 2D environment.
	position_.push_back(0.0);
	position_.push_back(0.0);
}

GPSHandler::~GPSHandler(){}

std::vector<double> GPSHandler::get2DPosition(){
	return position_;
}

void GPSHandler::set2DPosition(std::vector<double> position){
	position_[0] = position[0];
	position_[1] = position[1];
}
}