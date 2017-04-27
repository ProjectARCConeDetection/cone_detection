#ifndef TOOLS_PLANNING_HPP
#define TOOLS_PLANNING_HPP

#include <eigen3/Eigen/Eigen>
#include <iostream>
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
};

namespace path{
	double distanceBetween(int base_index,int target_index,std::vector<Eigen::Vector3d> positions);
	double distanceToIndex(int target_index,std::vector<Eigen::Vector3d> positions);
	int indexOfDistanceFront(double max_distance,std::vector<Eigen::Vector3d> positions);
}//namespace path.

#endif