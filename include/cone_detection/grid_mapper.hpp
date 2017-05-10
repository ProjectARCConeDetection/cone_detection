#ifndef CONE_DETECTION_GRID_MAPPER_HPP
#define CONE_DETECTION_GRID_MAPPER_HPP

#include <cone_detection/tools.hpp>

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <math.h> 
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

class GridMapper{
public:
	GridMapper();
	~GridMapper();
	void init(Detection detection);
	void updateConeMap(Eigen::Vector2d cone_position);
	Eigen::Vector2d convertLocalToGlobal(Candidate cone);
	std::vector< std::vector<int> > getConeMap();
	nav_msgs::OccupancyGrid getOccupancyGridMap();
	Pose getPose();
	geometry_msgs::Point getPoseMsg();
	void setPose(Pose pose);
private:
	//Current pose.
	Pose pose_;
	//Grid map.
	std::vector< std::vector<int> > cone_map_;
	//Parameter.
	Detection detection_;
	//Helper function.
	Eigen::Vector2d findGridElement(double x, double y);
	void initConeMap();
	void validConeArea(Eigen::Vector2d cone_index);
};
#endif