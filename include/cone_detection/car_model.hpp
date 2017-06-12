#include <cone_detection/tools.hpp>

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>


class CarModel{
public:
	CarModel();
	~CarModel();
	void init(Erod erod);	
	void updateModel();
	geometry_msgs::Pose getPoseMsg();
	geometry_msgs::TwistStamped getTwistMsg();
	void setSteeringAngle(double steering_angle);
	void setRearLeftWheelVel(double vel);
	void setRearRightWheelVel(double vel);
private:
	//Velocity.
	Eigen::Vector2d local_velocity_;
	//Pose.
	Pose car_pose_;
	//Current measurements.
	double steering_angle_;
	double velocity_left_;
	double velocity_right_;
	double last_update_time_;
	//Parameter.
	float distance_rear_front_axis_;
	float width_axis_;
};