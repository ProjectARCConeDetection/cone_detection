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

#define buflen_ 512 

struct AckermannControl{
	double steering_angle;
	double velocity;
	//Help function.
	void print();
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

struct Planning{
	double distance_cone;
	double number_points_x_axis;
	double searching_length;
	double searching_width;
	double resolution;
};

namespace path{
	double distanceBetween(int base_index,int target_index,std::vector<Eigen::Vector2d> positions);
	double distanceToIndex(int target_index,std::vector<Eigen::Vector2d> positions, int current_array_index);
	int indexOfDistanceFront(double max_distance,std::vector<Eigen::Vector2d> positions, Eigen::Vector2d position);
	int currentArray(std::vector<Eigen::Vector2d> positions, Eigen::Vector2d position);
}//namespace path.

class VCUInterface{
public:
	VCUInterface();
	~VCUInterface();
	void init(bool use_vcu);
	double recv_velocity();
	void send_msg(std::string symbol, double msg, bool requirement);
	void send_msg(std::string symbol, double msg, bool requirement,
				  double max_value, double min_value, double shift);

private:
	//Network.
	struct sockaddr_in si_me_, si_other_, si_VCU_;
	int sock_;
	socklen_t slen_;
	//Use vcu.
	bool use_vcu_;
	//Useful functions.
	void printError(std::string error);
};

#endif