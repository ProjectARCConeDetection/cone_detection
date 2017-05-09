#include <planning/pure_pursuit.hpp>
#include <planning/tools.hpp>
#include <planning/planner.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cone_detection/Path.h>

//Publisher.& Subscriber
ros::Subscriber mode_sub;
ros::Subscriber gridmap_sub;
ros::Publisher path_pub;
//Init classes.
PurePursuit pure_pursuit;
Planner planner;
//Decleration of functions.
void modeCallback(const std_msgs::Bool::ConstPtr& msg);
void gridmapCallback(const nav_msgs::OccupancyGrid grid);
void gettingParameter(ros::NodeHandle* node, Control* control, Planning* planning);

int main(int argc, char** argv){
	ros::init(argc, argv, "path_planner");
	ros::NodeHandle node;
	//Getting parameter.
	Control control;
	Planning planning;
	gettingParameter(&node,&control,&planning);
	//Init classes.
	pure_pursuit.init(control);
	planner.init(planning);
	//Init pubs & subs.
	path_pub = node.advertise<std_msgs::Float32MultiArray>("/path", 10);
	mode_sub = node.subscribe("/mode", 1, modeCallback);
	gridmap_sub = node.subscribe("/cones_grid", 1, gridmapCallback);
  	//Spinning.
  	ros::spin();
	return 0;
}

void modeCallback(const std_msgs::Bool::ConstPtr& msg){
	//Start autonomous mode if true.
	pure_pursuit.startAutonomousMode(msg->data);
}

void gridmapCallback(const nav_msgs::OccupancyGrid grid) {
	planner.gridAnalyser(grid);
	std_msgs::Float32MultiArray global_path = planner.getGlobalPath();
	path_pub.publish(global_path);
}

void gettingParameter(ros::NodeHandle* node, Control* control, Planning* planning){
	//Get control parameter.
	node->getParam("/control/distance_interpolation", control->distance_interpolation);
	node->getParam("/control/K1_LAD_S", control->k1_lad_s);
	node->getParam("/control/K2_LAD_S", control->k2_lad_s);
	node->getParam("/control/K1_LAD_V", control->k1_lad_v);
	node->getParam("/control/K2_LAD_V", control->k2_lad_v);
	node->getParam("/control/upperbound_lad_s", control->upperbound_lad_s);
	node->getParam("/control/lowerbound_lad_s", control->lowerbound_lad_s);
	node->getParam("/control/max_absolute_velocity", control->max_absolute_velocity);
	node->getParam("/control/max_lateral_acceleration", control->max_lateral_acceleration);
	node->getParam("/control/max_steering_angle", control->max_steering_angle);
	node->getParam("/path_planning/distance_cone", planning->distance_cone);
	node->getParam("/path_planning/number_points_x_axis", planning->number_points_x_axis);
	node->getParam("/detection/searching_length", planning->searching_length);
	node->getParam("/detection/searching_width", planning->searching_width);
}
