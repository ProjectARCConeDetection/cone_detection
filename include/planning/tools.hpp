#ifndef TOOLS_PLANNING_HPP
#define TOOLS_PLANNING_HPP

#include <arpa/inet.h>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <vector>

struct AckermannControl{
	double steering_angle;
	double velocity;
};

struct Control{
	double distance_interpolation;
	double k1_lad_s;
	double k2_lad_s;
	double upperbound_lad_s;
	double lowerbound_lad_s;
	double k1_lad_v;
	double k2_lad_v;
	double distance_wheel_axis;
	double max_absolute_velocity;
	double max_lateral_acceleration;
	double max_steering_angle;
};

namespace path{
	double distanceBetween(int base_index,int target_index,std::vector<Eigen::Vector2d> positions);
	double distanceToIndex(int target_index,std::vector<Eigen::Vector2d> positions);
	int indexOfDistanceFront(double max_distance,std::vector<Eigen::Vector2d> positions);
}//namespace path.

class VCUInterface{
public:
	VCUInterface();
	~VCUInterface();
	void init();
	void send_msg(std::string symbol, double msg, bool requirement);
	void send_msg(std::string symbol, double msg, bool requirement,
				  double max_value, double min_value, double shift);

private:
	//Network.
	struct sockaddr_in si_me_, si_other_, si_VCU_;
	int sock_;
	socklen_t slen_;
	//Useful functions.
	void printError(std::string error);
};

#endif