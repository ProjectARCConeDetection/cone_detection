#include <cone_detection/tools.hpp>

#include <eigen3/Eigen/Eigen>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>


class CarModel{
public:
	CarModel();
	~CarModel();
	void init(Erod erod);	
	void updateModel();
	geometry_msgs::TwistStamped getTwistMsg();
	Eigen::Vector3d getVelocity();
	void setSteeringAngle(double steering_angle);
	void setRearLeftWheelVel(double vel);
	void setRearRightWheelVel(double vel);
	void setTimeStamp(ros::Time timestamp);
private:
	//Velocity.
	Eigen::Vector2d local_velocity_;
	Eigen::Vector3d tilted_velocity_;
	//Pose.
	Eigen::Vector2d position_;
	double orientation_;
	//Current measurements.
	double steering_angle_;
	double velocity_left_;
	double velocity_right_;
	ros::Time timestamp_;
	double time_delta_;
	//Parameter.
	float distance_rear_front_axis_;
	float width_axis_;
};