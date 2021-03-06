#ifndef TOOLS_CONE_DETECTION_HPP
#define TOOLS_CONE_DETECTION_HPP

#include "opencv2/core/core.hpp"

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <math.h>
#include <vector>

struct Candidate{
	double x;
	double y;
	double z;
	int index;
	//Functions.
	void print();
};

struct Cam{
	cv::Mat intrisicMat;
    cv::Mat rVec;
    cv::Mat tVec;
    cv::Mat distCoeffs; 
    int image_height;
    int image_width;
    //Functions.
    void initCamMatrices();
};

struct Cone{
	double height_meter;
	double height_pixel;
	double width_pixel;
};

struct Detection{
	double cone_area_x;
	double cone_area_y;
	double intensity_threshold;
	double searching_length;
	double searching_resolution;
	double searching_width;
};

struct Erod{
	double distance_wheel_axis;
	double length_laser_to_VI;
	double height_laser;
	double width_wheel_axis;
};

struct Pose{
	//Position.
	Eigen::Vector2d position;
	//Orientation.
	double orientation;
	//Functions.
	Eigen::Vector2d globalToLocal(const Eigen::Vector2d global);
	Eigen::Vector2d localToGlobal(const Eigen::Vector2d local);
	void print();
};

namespace transforms{
	Eigen::Matrix2d getRotationMatrix(double angle);
}//namespace transforms.
namespace quat {
	Eigen::Vector4d multQuaternion(Eigen::Vector4d q1,Eigen::Vector4d q2);
	Eigen::Vector4d inverseQuaternion(Eigen::Vector4d quat);
	Eigen::Vector4d diffQuaternion(Eigen::Vector4d base_quat, Eigen::Vector4d target_quat);
}//namespace quat.
namespace vector{
	int getSameIndex(std::vector <Candidate> xyz_index_vector,int index);
}//namespace vector.

#endif