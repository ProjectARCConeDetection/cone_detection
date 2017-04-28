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
	double cone_area;
	double intensity_threshold;
	double searching_length;
	double searching_resolution;
	double searching_width;
};

struct Erod{
	double distance_wheel_axis;
	double length_laser_to_VI;
	double height_laser;
};

struct Pose{
	//Position.
	Eigen::Vector2d position;
	//Orientation.
	Eigen::Vector4d orientation;
	//Functions.
	Eigen::Vector3d euler();
	Eigen::Vector2d globalToLocal(Eigen::Vector2d global);
	Eigen::Vector2d localToGlobal(const Eigen::Vector2d local);
	void print();
};

namespace transforms{
	Eigen::Vector2d to2D(const Eigen::Vector3d input);
	Eigen::Vector3d to3D(const Eigen::Vector2d input);
	Eigen::Matrix3d getRotationMatrix(Eigen::Vector3d euler);
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