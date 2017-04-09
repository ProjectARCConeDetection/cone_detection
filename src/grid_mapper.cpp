#include <cone_detection/grid_mapper.hpp>

namespace cone_detection{

GridMapper::GridMapper(){}
	
GridMapper::~GridMapper(){}

void GridMapper::updateConeMap(std::vector<double> rel_cone_pose,
					   		   std::vector<double> global_pose){
	double x_local = rel_cone_pose[0];
	double y_local = rel_cone_pose[1];
	double x_global = global_pose[0];
	double y_global = global_pose[1];
	double x = x_global + x_local;
	double y = y_global + y_local;
	std::vector<int> grid_indizes = findNextGridElement(x,y);
	// Updating cone map.
	cone_map_[grid_indizes[0]][grid_indizes[1]] = 1;
}

nav_msgs::OccupancyGrid GridMapper::getOccupancyGridMap(){
	nav_msgs::OccupancyGrid grid;
	int x_steps = height_/resolution_;
	int y_steps = width_/resolution_;
	grid.info.height = x_steps;
	grid.info.resolution = resolution_;
  	grid.info.width = y_steps;
  	geometry_msgs::Pose pose;
  	pose.position.x = height_/2;
  	pose.position.y = 0;
 	pose.position.z = 0;
  	pose.orientation.x = 0;
  	pose.orientation.y = 0;
  	pose.orientation.z = 0;
  	pose.orientation.w = 1;
  	grid.info.origin = pose;
	for(int i=0;i<x_steps;++i)
		for (int j=0;j<y_steps; ++j)
			grid.data.push_back(cone_map_[i][j]);
    return grid;
}

std::vector<int> GridMapper::findNextGridElement(double x, double y){
	int x_index = x/resolution_;
	if(fmod(x,resolution_) > 0.5) x_index++;
	int y_index = y/resolution_;
	if(fmod(y,resolution_) > 0.5) y_index++;
	std::vector<int> indizes;
	indizes[0] = x_index; 
	indizes[1] = y_index;
	return indizes;
}
	
void GridMapper::initConeMap(){
	int x_steps = height_/resolution_;
	int y_steps = width_/resolution_;
	for(int i=0;i<x_steps;++i){
		std::vector<int> temp;
		for (int j=0;j<y_steps; ++j)
			temp.push_back(0);
		cone_map_.push_back(temp);
	}
}

void GridMapper::setGridHeight(double height){
	height_ = height;}
void GridMapper::setGridResolution(double resolution){
	resolution_ = resolution;}
void GridMapper::setGridWidth(double width){
	width_ = width;}
}