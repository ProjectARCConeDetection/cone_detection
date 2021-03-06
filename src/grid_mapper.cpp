#include <cone_detection/grid_mapper.hpp>

GridMapper::GridMapper(){}
	
GridMapper::~GridMapper(){}

void GridMapper::init(Detection detection){
	//Set grid parameter.
	detection_ = detection;
	//Init cone map.
	initConeMap();
}

void GridMapper::updateConeMap(Eigen::Vector2d cone_position){
	//Get cone position.
 	double x = cone_position(0);
	double y = cone_position(1);
	Eigen::Vector2d cone_indizes = findGridElement(x,y);
	std::cout << "Index: " << cone_indizes(0) << " " << cone_indizes(1) << std::endl;
	// Updating cone map.
	cone_map_[cone_indizes(0)][cone_indizes(1)] = 1;
}

Eigen::Vector2d GridMapper::convertLocalToGlobal(Candidate cone){
	Eigen::Vector2d local(cone.x, cone.y);
	//Transform local to global vector.
	Eigen::Vector2d delta_global = pose_.localToGlobal(local);
	//Find global position.
	double x = pose_.position(0) + delta_global(0);
	double y = pose_.position(1) + delta_global(1);
	Eigen::Vector2d global(x,y);
	return global;
}

std::vector< std::vector<int> > GridMapper::getConeMap(){return cone_map_;}

Pose GridMapper::getPose(){return pose_;}

nav_msgs::OccupancyGrid GridMapper::getOccupancyGridMap(){
	nav_msgs::OccupancyGrid grid;
	int x_steps = detection_.searching_length/detection_.searching_resolution;
	int y_steps = detection_.searching_width/detection_.searching_resolution;
	grid.info.height = x_steps;
	grid.info.resolution = detection_.searching_resolution;
  	grid.info.width = y_steps;
  	geometry_msgs::Pose pose;
  	pose.position.x = 0;
  	pose.position.y = 0;
 	pose.position.z = 0;
  	pose.orientation.x = 0;
  	pose.orientation.y = 0;
  	pose.orientation.z = 0;
  	pose.orientation.w = 1;
  	grid.info.origin = pose;
  	for(int i=0;i<x_steps;++i)
		for (int j=0;j<y_steps; j++)
			grid.data.push_back(cone_map_[i][j]);
    return grid;
}

Eigen::Vector2d GridMapper::findGridElement(double x, double y){
	int x_index = x/detection_.searching_resolution;
	int y_index = (y+detection_.searching_width/2)/detection_.searching_resolution;
	Eigen::Vector2d indizes(x_index, y_index);
	return indizes;
}
	
void GridMapper::initConeMap(){
	int x_steps = detection_.searching_length/detection_.searching_resolution;
	int y_steps = detection_.searching_width/detection_.searching_resolution;
	for(int i=0;i<x_steps;++i){
		std::vector<int> temp;
		for (int j=0;j<y_steps; ++j)
			temp.push_back(0);
		cone_map_.push_back(temp);
	}
}

void GridMapper::setPose(Pose pose){pose_ = pose;}

// void GridMapper::validConeArea(Eigen::Vector2d cone_index){
// 	bool another_cone = false;
// 	//Convert cone area to index.
// 	int area_index_x = detection_.cone_area_x/detection_.searching_resolution;
// 	int area_index_y = detection_.cone_area_y/detection_.searching_resolution;
// 	//Check if cone already exists.
// 	int x = std::max(cone_index(0)-area_index_x,0.0);
// 	int x_upper_bound = std::min(detection_.searching_length/detection_.searching_resolution-1, (double)x+2*area_index_x);
// 	for(;x<x_upper_bound;x++) {
// 		int y = std::max(cone_index(1)-area_index_y,0.0);
// 		int y_upper_bound = std::min(detection_.searching_width/detection_.searching_resolution-1, (double)y+2*area_index_y);
// 		for(; y<y_upper_bound; y++) {
// 			if(cone_map_[x][y]==1) 
// 				return;
// 	}}
// 	//Fill cone area iff cone is not in map.
// 	if(!another_cone) cone_map_[cone_index(0)][cone_index(1)] = 1;	
// }