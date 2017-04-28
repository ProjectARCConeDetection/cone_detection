#ifndef PURE_PURSUIT_PLANNING_HPP
#define PURE_PURSUIT_PLANNING_HPP

#include <cone_detection/tools.hpp>
#include <planning/tools.hpp>

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <vector>

class PurePursuit{
public:
	PurePursuit();
	~PurePursuit();
	void init(Control control);
	void calculateControls(Pose pose, double velocity);
	void startAutonomousMode(bool mode);
private:
	//Current controls.
	AckermannControl should_controls_;
	//Reference path.
	std::vector<Eigen::Vector2d> path_;
	//Control parameter.
	Control control_;
	//VCU Interface.
	VCUInterface vcu_;
	//Helper functions.
	double calculateSteering(Pose pose, double velocity);
	double calculateVel(Pose pose, double velocity);
	double curveRadius();
};

#endif