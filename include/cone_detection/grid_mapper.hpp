#ifndef CONE_DETECTION_GRID_MAPPER_HPP
#define CONE_DETECTION_GRID_MAPPER_HPP

#include <iostream>
#include <math.h> 
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

namespace cone_detection{

class GridMapper{
public:
	GridMapper();
	~GridMapper();
	void updateConeMap(std::vector<double> rel_cone_pose,
					   std::vector<double> global_pose);
	nav_msgs::OccupancyGrid getOccupancyGridMap();
	void initConeMap();
	void setGridHeight(double height);
	void setGridResolution(double resolution);
	void setGridWidth(double width);

private:
	std::vector< std::vector<int> > cone_map_;
	double height_;
	double resolution_;
	double width_;
	std::vector<int> findNextGridElement(double x, double y);
};
}
#endif