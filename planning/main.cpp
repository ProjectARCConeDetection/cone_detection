#include <planning/pure_pursuit.hpp>
#include <planning/tools.hpp>
#include <planning/planner.hpp>

#include <cone_detection/tools.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cone_detection/Path.h>

//Publisher.& Subscriber.
ros::Publisher controls_pub;
ros::Publisher path_pub;
ros::Subscriber mode_sub;
ros::Subscriber gridmap_sub;
ros::Subscriber pose_sub;
//Init classes.
PurePursuit pure_pursuit;
Planner planner;
VCUInterface vcu;
//Parameter.
Control control;
Erod erod;
Planning planning;
//Decleration of functions.
void modeCallback(const std_msgs::Bool::ConstPtr& msg);
void gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid);
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
void gettingParameter(ros::NodeHandle* node, Control* control, Erod* erod, Planning* planning);

int main(int argc, char** argv){
	ros::init(argc, argv, "path_planner");
	ros::NodeHandle node;
	//Getting parameter.
	gettingParameter(&node,&control,&erod,&planning);
	bool use_vcu = (strlen(*(argv + 1)) == 5) ? false : true;
	//Init classes.
	pure_pursuit.init(control,erod);
	planner.init(planning);
	vcu.init(use_vcu);
	//Init pubs & subs.
	controls_pub = node.advertise<std_msgs::Float32MultiArray>("/stellgroessen", 1);
	path_pub = node.advertise<std_msgs::Float32MultiArray>("/path", 10);
	mode_sub = node.subscribe("/mode", 1, modeCallback);
	gridmap_sub = node.subscribe("/cones_grid", 1, gridmapCallback);
	pose_sub = node.subscribe("/car_pose", 1, poseCallback);
  	//Spinning.
  	ros::Rate rate(10);
  	while(ros::ok()){
  		ros::spinOnce();
		pure_pursuit.setVelocity(vcu.recv_velocity());
		rate.sleep();
  	}
	return 0;
}

void modeCallback(const std_msgs::Bool::ConstPtr& msg){
	vcu.send_msg("cc", 5.0, msg->data);
	ros::Duration(0.5).sleep();
	vcu.send_msg("am", 1.0, msg->data);
}

void gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid){
	//Path planning.
	planner.gridAnalyser(*grid);
	//Calculate controls.
	std::vector<Eigen::Vector2d> path = planner.getGlobalPath();
	if(path.size() > 0){
		AckermannControl stellgroessen = pure_pursuit.calculateControls(path);
		//Send controls to VCU.
		vcu.send_msg("vs",stellgroessen.velocity, true, control.max_absolute_velocity, -100, 0);
		vcu.send_msg("ss",stellgroessen.steering_angle/180.0*M_PI, true, 
				  	control.max_steering_angle, -control.max_steering_angle, 0);
	}
	//Visualisation.
	std_msgs::Float32MultiArray global_path_msg = planner.getGlobalPathMsg();
	path_pub.publish(global_path_msg);
	std_msgs::Float32MultiArray control_msg = pure_pursuit.getControlsMsg();
	controls_pub.publish(control_msg);
}

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg){
	Pose pose;
	pose.position = Eigen::Vector2d(msg->position.x, msg->position.y);
	pose.orientation = Eigen::Vector4d(msg->orientation.x, msg->orientation.y, 
									   msg->orientation.z, msg->orientation.w);
	pure_pursuit.setPose(pose);
}

void gettingParameter(ros::NodeHandle* node, Control* control, Erod* erod, Planning* planning){
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
	//Get erod parameter.
	node->getParam("/erod/length_laser_to_VI", erod->length_laser_to_VI);
	node->getParam("/erod/distance_wheel_axis", erod->distance_wheel_axis);
	node->getParam("/erod/height_laser", erod->height_laser);
	//Get planning parameter.
	node->getParam("/path_planning/distance_cone", planning->distance_cone);
	node->getParam("/path_planning/number_points_x_axis", planning->number_points_x_axis);
	node->getParam("/detection/searching_length", planning->searching_length);
	node->getParam("/detection/searching_width", planning->searching_width);
}
