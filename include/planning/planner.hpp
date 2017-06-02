#ifndef PLANNER_PLANNING_HPP
#define PLANNER_PLANNING_HPP

#include <arpa/inet.h>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <vector>
#include <cmath>
#include <planning/tools.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32MultiArray.h>

class Planner{
public:
	void init(Planning planning);
	void gridAnalyser(const nav_msgs::OccupancyGrid grid);
	void update_conePosition(int x_cell_index, int y_cell_index);
	double LocalOrientationAngle(Eigen::Vector2d position_cone_1, Eigen::Vector2d position_cone_2);
	void updatePositionVector();
	void updateStartPath();
	void updateWholePath();
	void updateEndPath();
	std::vector<Eigen::Vector2d> PathPlanner(Eigen::Vector2d position_cone_1, Eigen::Vector2d position_cone_2, int index_position_cone_1);
	std::vector<Eigen::Vector2d> startPath(Eigen::Vector2d position_cone_2, double angle, double length);
	double LocalLengthXAxis(Eigen::Vector2d position_cone_1, Eigen::Vector2d position_cone_2);
	std::vector<Eigen::Vector2d> CosinePlanner(double length, Eigen::Vector2d position_cone_1, int index_position_cone_1);
	std::vector<Eigen::Vector2d> GlobalControlPoints(std::vector<Eigen::Vector2d> local_points, double angle, Eigen::Vector2d position_cone_1);
	std::vector<Eigen::Vector2d> CircleFiller(Eigen::Vector2d position_cone_1, Eigen::Vector2d edge_point_cone_1, Eigen::Vector2d edge_point_cone_2);
	double removePoints(int index_position_cone_1);
	std_msgs::Float32MultiArray getGlobalPathMsg();
	std::vector<Eigen::Vector2d> getGlobalPath();

private:
	Planning planning_;
	std::vector<Eigen::Vector2d> position_cones_;
	std::vector<Eigen::Vector2d> global_path_;
	std::vector<Eigen::Vector2d> edge_points_;
};


#endif