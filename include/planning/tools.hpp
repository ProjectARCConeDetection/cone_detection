#ifndef TOOLS_PLANNING_HPP
#define TOOLS_PLANNING_HPP

#include <cone_detection/tools.hpp>

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

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

#endif