#ifndef CONE_DETECTION_GRID_MAPPER_HPP
#define CONE_DETECTION_GRID_MAPPER_HPP

#include <cone_detection/tools.hpp>

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <math.h> 
#include <vector>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

class GridMapper{
public:
	GridMapper();
	~GridMapper();
	void updateConeMap(Eigen::Vector3d local);
	nav_msgs::OccupancyGrid getOccupancyGridMap();
	void initConeMap();
	void setGridHeight(double height);
	void setGridResolution(double resolution);
	void setGridWidth(double width);
	void setPose(Pose pose);
private:
	//Current pose.
	Pose pose_;
	//Grid map.
	std::vector< std::vector<int> > cone_map_;
	double height_;
	double resolution_;
	double width_;
	std::vector<int> findNextGridElement(double x, double y);
};
#endif