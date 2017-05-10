#include <cone_detection/grid_mapper.hpp>

GridMapper::GridMapper(){}
	
GridMapper::~GridMapper(){}

void GridMapper::init(Detection detection){
	//Init pose.
	pose_.position = Eigen::Vector2d(0,0);
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
	if(cone_indizes(1) >= 0) validConeArea(cone_indizes);
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

geometry_msgs::Point GridMapper::getPoseMsg(){
	geometry_msgs::Point point_msg;
	point_msg.x = pose_.position(0);
	point_msg.y = pose_.position(1);
	return point_msg;
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
	int area_index = detection_.cone_area/detection_.searching_resolution;
	//Check if cone already exists.
	for(int x = cone_index(0)-area_index; x<cone_index(0)+area_index; ++x)
		for(int y = cone_index(1)-area_index; y<cone_index(1)+area_index; ++y)
			if((bool)cone_map_[x][y]) return;
	//Fill cone area iff cone is not in map.
	cone_map_[cone_index(0)][cone_index(1)] = 1;		
}

void GridMapper::setPose(Pose pose){pose_ = pose;}
