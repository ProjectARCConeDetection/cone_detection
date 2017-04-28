#include <planning/pure_pursuit.hpp>
#include <planning/tools.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

//Publisher.
ros::Subscriber mode_sub;
//Init classes.
PurePursuit pure_pursuit;
//Decleration of functions.
void modeCallback(const std_msgs::Bool::ConstPtr& msg);
void gettingParameter(ros::NodeHandle* node, Control* control);

int main(int argc, char** argv){
	ros::init(argc, argv, "path_planner");
	ros::NodeHandle node;
	//Getting parameter.
	Control control;
	gettingParameter(&node,&control);
	//Init classes.
	pure_pursuit.init(control);
	//Init pubs & subs.
	mode_sub = node.subscribe("/mode", 1, modeCallback);
  	//Spinning.
  	ros::spin();
	return 0;
}

void modeCallback(const std_msgs::Bool::ConstPtr& msg){
	//Start autonomous mode if true.
	pure_pursuit.startAutonomousMode(msg->data);
}

void gettingParameter(ros::NodeHandle* node, Control* control){
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
}
