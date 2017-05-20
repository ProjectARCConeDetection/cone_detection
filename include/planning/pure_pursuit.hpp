#ifndef PURE_PURSUIT_PLANNING_HPP
#define PURE_PURSUIT_PLANNING_HPP

#include <cone_detection/tools.hpp>
#include <planning/tools.hpp>

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

class PurePursuit{
public:
	PurePursuit();
	~PurePursuit();
	void init(Control control, Erod erod);
	AckermannControl calculateControls(std::vector<Eigen::Vector2d> path);
	std_msgs::Float32MultiArray getControlsMsg();
	void setPose(Pose pose);
	void setVelocity(double velocity);
private:
	//Current controls.
	AckermannControl should_controls_;
	//Reference path.
	std::vector<Eigen::Vector2d> path_;
	//Car position, orientation and velocity.
	Pose pose_;
	double velocity_;
	//Control parameter.
	Control control_;
	Erod erod_;
	//Helper functions.
	double calculateSteering();
	double calculateVel();
	double curveRadius();
};

#endif