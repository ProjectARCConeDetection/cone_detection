#include <cone_detection/grid_mapper.hpp>

GridMapper::GridMapper(){}
	
GridMapper::~GridMapper(){}

void GridMapper::init(Detection detection){
	//Init pose.
	pose_.position = Eigen::Vector2d(-2.2,0);
	pose_.orientation = Eigen::Vector4d(0,0,0,1);
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
	// Updating cone map.
	validConeArea(cone_indizes);
	//if(cone_indizes(1) >= 0) validConeArea(cone_indizes);
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

geometry_msgs::Pose GridMapper::getPoseMsg(){
	geometry_msgs::Pose pose_msg;
	pose_msg.position.x = pose_.position(0);
	pose_msg.position.y = pose_.position(1);
	pose_msg.position.z = 0.0;
	pose_msg.orientation.x = pose_.orientation(0);
	pose_msg.orientation.y = pose_.orientation(1);
	pose_msg.orientation.z = pose_.orientation(2);
	pose_msg.orientation.w = pose_.orientation(3);
	return pose_msg;
}

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
		for (int j=0;j<y_steps; ++j)
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
		for (int j=-y_steps/2;j<y_steps/2; ++j)
			temp.push_back(0);
		cone_map_.push_back(temp);
	}
}

void GridMapper::validConeArea(Eigen::Vector2d cone_index){
	//Convert cone area to index.
	int area_index_x = detection_.cone_area_x/detection_.searching_resolution;
	int area_index_y = detection_.cone_area_y/detection_.searching_resolution;
	//Check if cone already exists.
	int x = std::max(cone_index(0)-area_index_x,0.0);
	int x_upper_bound = std::min(detection_.searching_length/detection_.searching_resolution-1, (double)x+2*area_index_x);
	for(;x<x_upper_bound;x++) {
		int y = std::max(cone_index(1)-area_index_y,0.0);
		int y_upper_bound = std::min(detection_.searching_width/detection_.searching_resolution-1, (double)y+2*area_index_y);
		for(; y<y_upper_bound; y++) {
			if(cone_map_[x][y]==1) {
				return;
			}
	}}
	//Fill cone area iff cone is not in map.
	cone_map_[cone_index(0)][cone_index(1)] = 1;	
}

void GridMapper::setPose(Pose pose){pose_ = pose;}
