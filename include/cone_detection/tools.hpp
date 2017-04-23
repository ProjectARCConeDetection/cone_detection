#ifndef TOOLS_CONE_DETECTION_HPP
#define TOOLS_CONE_DETECTION_HPP

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

struct Pose{
	//Position.
	Eigen::Vector3d position;
	//Orientation.
	Eigen::Vector4d orientation;
	//Functions.
	Eigen::Vector3d euler();
	Eigen::Matrix3d getRotationMatrix();
	Eigen::Vector3d localToGlobal(const Eigen::Vector3d local);
	void print();
};

namespace quat {
	Eigen::Vector4d multQuaternion(Eigen::Vector4d q1,Eigen::Vector4d q2);
	Eigen::Vector4d inverseQuaternion(Eigen::Vector4d quat);
	Eigen::Vector4d diffQuaternion(Eigen::Vector4d base_quat, Eigen::Vector4d target_quat);
}//namespace quat.
namespace vector{
	int getSameIndex(std::vector <Candidate> xyz_index_vector,int index);
}

#endif