#include <planning/planner.hpp>

void Planner::init(Planning planning) {
	planning_ = planning;
}

void Planner::gridAnalyser(const nav_msgs::OccupancyGrid grid){
	double height = grid.info.height;
	double width = grid.info.width;
	planning_.resolution = grid.info.resolution;
	for(int i=0; i<(height*width); i++) {
		if (grid.data[i]==1) update_conePosition((int)floor(i/width), fmod(i,width));
	}
}

void Planner::update_conePosition(int x_cell_index, int y_cell_index){
	Eigen::Vector2d position_cone;
	position_cone[0] = x_cell_index*planning_.resolution + planning_.resolution/2;
	position_cone[1] = y_cell_index*planning_.resolution + planning_.resolution/2 - planning_.searching_width/2;
	bool included = false;
	for(int i=0; i<position_cones_.size(); i++) {
		if((position_cones_[i][0]==position_cone[0]) && (position_cones_[i][1]==position_cone[1])) included=true;
	}
	if(!included) {
		position_cones_.push_back(position_cone);
		updatePositionVector();
	}
}

void Planner::updatePositionVector(){
	int size =  position_cones_.size();
	Eigen::Vector2d new_cone = position_cones_[size-1];
	bool between_cones = false;
	if(size==1) {
		updateStartPath();
		return;
	}
	for(int i=0;i<size-2;i++) {
		double delta_x_distance = position_cones_[i+1][0] - position_cones_[i][0];
		double delta_y_distance = position_cones_[i+1][1] - position_cones_[i][1];
		double delta_x_new_cone = new_cone[0]- position_cones_[i][0];
		double delta_y_new_cone = new_cone[1]- position_cones_[i][1];
		if((delta_x_new_cone<delta_x_distance)) {
			between_cones = true;
			for(int j=2;j<=size-i-1;j++) {
				position_cones_[size-j+1] = position_cones_[size-j];
			}
			position_cones_[i+1] = new_cone;
			for(int i=0; i<position_cones_.size();i++) {
			}
		break;
	}}
	if(between_cones) updateWholePath();
	else updateEndPath();
}

void Planner::updateStartPath(){
	Eigen::Vector2d temp = position_cones_[0];
	position_cones_[0][0] = 0;
	position_cones_[0][1] = 0;
	position_cones_.push_back(temp);
	std::vector<Eigen::Vector2d> new_global_points = PathPlanner(position_cones_[0], position_cones_[1], 0);
	for(int i=0; i<new_global_points.size();i++) {
		global_path_.push_back(new_global_points[i]);
	}
}

void Planner::updateWholePath(){
	global_path_.clear();
	edge_points_.clear();
	for(int i=0; i<position_cones_.size()-1;i++) {
		std::vector<Eigen::Vector2d> new_global_points = PathPlanner(position_cones_[i], position_cones_[i+1], i);
		for(int j=0; j<new_global_points.size();j++) {
			global_path_.push_back(new_global_points[j]);
}}}

void Planner::updateEndPath(){
	int size = position_cones_.size();
	std::vector<Eigen::Vector2d> new_global_points = PathPlanner(position_cones_[size-2], position_cones_[size-1], size-2);
	for(int i=0; i<new_global_points.size();i++) {
		global_path_.push_back(new_global_points[i]);
	}
	for(int i=0; i<position_cones_.size();i++) {
}}

std::vector<Eigen::Vector2d> Planner::PathPlanner(Eigen::Vector2d position_cone_1, Eigen::Vector2d position_cone_2, int index_position_cone_1){
	double angle = LocalOrientationAngle(position_cone_1, position_cone_2);
	double length = LocalLengthXAxis(position_cone_1, position_cone_2);
	std::vector<Eigen::Vector2d> new_global_cosine_points = GlobalControlPoints(CosinePlanner(length, position_cone_1, index_position_cone_1), angle, position_cone_1);
	edge_points_.push_back(new_global_cosine_points[0]);
	edge_points_.push_back(new_global_cosine_points[new_global_cosine_points.size()-1]);
	std::vector<Eigen::Vector2d> new_global_points;
	if(edge_points_.size()>2) {
		Eigen::Vector2d edge_point_cone_1 = edge_points_[edge_points_.size()-3];
		Eigen::Vector2d edge_point_cone_2 = edge_points_[edge_points_.size()-2];
		double distance_cone1_edge1 = (edge_point_cone_1 - position_cones_[index_position_cone_1-1]).norm();
		double distance_cone1_edge2 = (edge_point_cone_2 - position_cones_[index_position_cone_1-1]).norm();
		std::vector<Eigen::Vector2d> new_global_filler_points;
		if(distance_cone1_edge1 < distance_cone1_edge2) {
			new_global_filler_points = CircleFiller(position_cone_1, edge_point_cone_1, edge_point_cone_2);
			for(int i=0; i<new_global_filler_points.size(); i++) {
				new_global_points.push_back(new_global_filler_points[i]);
			}
			for(int i=0; i<new_global_cosine_points.size(); i++) {
				new_global_points.push_back(new_global_cosine_points[i]);
		}}
		else {
			double remove_distance = removePoints(index_position_cone_1);
			double length2 = LocalLengthXAxis(position_cone_1, position_cones_[index_position_cone_1-1]);
			int number_points_remove1 = planning_.number_points_x_axis/length2*remove_distance;
			int number_points_remove2 = planning_.number_points_x_axis/length*remove_distance+1;
			for(int i=-number_points_remove1;i<0;i++) {
				global_path_[global_path_.size()+i] = new_global_cosine_points[number_points_remove2+number_points_remove1+i];
			}
			for(int i=(number_points_remove1+number_points_remove2);i<new_global_cosine_points.size();i++) {
				new_global_points.push_back(new_global_cosine_points[i]);
	}}}
	else {
		for(int i=0;i<new_global_cosine_points.size();i++) {
			new_global_points.push_back(new_global_cosine_points[i]);					
	}}
	return new_global_points;
}

double Planner::LocalOrientationAngle(Eigen::Vector2d position_cone_1, Eigen::Vector2d position_cone_2){
	double delta_y = position_cone_2[1]-position_cone_1[1];
	double delta_x = position_cone_2[0]-position_cone_1[0];
	double angle = atan2(delta_y,delta_x);
	return angle;
}

double Planner::LocalLengthXAxis(Eigen::Vector2d position_cone_1, Eigen::Vector2d position_cone_2){
	double length = (position_cone_2 - position_cone_1).norm();
	return length;
}

std::vector<Eigen::Vector2d> Planner::CosinePlanner(double length, Eigen::Vector2d position_cone_1, int index_position_cone_1){
	std::vector<Eigen::Vector2d> controller_points;
	Eigen::Vector2d controller_point;
	bool shifted = false;
	if(edge_points_.size()>0) {
		if(edge_points_[2*index_position_cone_1-1][1] - position_cone_1[1]>0) {
			shifted = 0;
		}
		else shifted = 1;
	}
	double delta_x = length/planning_.number_points_x_axis;
	for(int i=0; i<(planning_.number_points_x_axis+1); i++) {
		controller_point[0] = delta_x*i;
		controller_point[1] = planning_.distance_cone*cos((controller_point[0]/length)*M_PI+shifted*M_PI);
		controller_points.push_back(controller_point);
	}
	return controller_points;
}

std::vector<Eigen::Vector2d> Planner::GlobalControlPoints(std::vector<Eigen::Vector2d> local_points, double angle, Eigen::Vector2d position_cone_1){
	Eigen::Matrix2d Rotation_Matrix;
  	Rotation_Matrix <<  cos(angle), -sin(angle),
                        sin(angle), cos(angle);
    std::vector<Eigen::Vector2d> global_controller_points;
    Eigen::Vector2d global_controller_point;
    for(int i=0; i< local_points.size(); i++) {
    	global_controller_point = Rotation_Matrix*local_points[i];
    	global_controller_point = global_controller_point + position_cone_1;
    	global_controller_points.push_back(global_controller_point);
    }
    return global_controller_points;
}

std::vector<Eigen::Vector2d> Planner::CircleFiller(Eigen::Vector2d position_cone_1, Eigen::Vector2d edge_point_cone_1, Eigen::Vector2d edge_point_cone_2){
	double angle_x_edge1 = atan2((edge_point_cone_1[1]-position_cone_1[1]), (edge_point_cone_1[0]-position_cone_1[0]));
	double angle_x_edge2 = atan2((edge_point_cone_2[1]-position_cone_1[1]), (edge_point_cone_2[0]-position_cone_1[0]));
	double radian_step = M_PI/200;
	double delta_angle = fabs(angle_x_edge2 - angle_x_edge1);
	std::vector<Eigen::Vector2d> circle_points;
	Eigen::Vector2d temp;
	int sign = -1;
	if(angle_x_edge2>angle_x_edge1) sign=1; 
	for(int i=0; i<(delta_angle/radian_step);i++) {
		temp[0] = cos(sign*i*radian_step+angle_x_edge1)*planning_.distance_cone;
		temp[1] = sin(sign*i*radian_step+angle_x_edge1)*planning_.distance_cone;
		temp = temp + position_cone_1;
		circle_points.push_back(temp);
	}
	return circle_points;
}

double Planner::removePoints(int index_position_cone_1){
	double angle_cone1 = atan2(position_cones_[index_position_cone_1+1][1]-position_cones_[index_position_cone_1][1], position_cones_[index_position_cone_1+1][0]-position_cones_[index_position_cone_1][0]);
	double angle_cone3 = atan2(position_cones_[index_position_cone_1-1][1]-position_cones_[index_position_cone_1][1], position_cones_[index_position_cone_1-1][0]-position_cones_[index_position_cone_1][0]);
	double angle_cone1_cone3;
	if(angle_cone1*angle_cone3>0) angle_cone1_cone3=fabs(angle_cone3-angle_cone1);
	else angle_cone1_cone3=fabs(angle_cone3+angle_cone1);
	return planning_.distance_cone/(tan(angle_cone1_cone3/2));
}

std_msgs::Float32MultiArray Planner::getGlobalPathMsg(){
	std_msgs::Float32MultiArray global_path_array;
	for(int i=0;i<global_path_.size();i++) {
		global_path_array.data.push_back(global_path_[i][0]);
		global_path_array.data.push_back(global_path_[i][1]);
	}
	return global_path_array;
}

std::vector<Eigen::Vector2d> Planner::getGlobalPath(){
	return global_path_;
}