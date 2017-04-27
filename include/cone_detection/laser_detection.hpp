#ifndef CONE_DETECTION_LASER_DETECTION_HPP
#define CONE_DETECTION_LASER_DETECTION_HPP

#include <cone_detection/tools.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <iostream>
#include <vector>

#include <sensor_msgs/PointCloud2.h>

class LaserDetection{
public:
	LaserDetection();
	~LaserDetection();
	void init(Cone cone, Detection detection, Erod erod);
	void coneMarker(const sensor_msgs::PointCloud2& msg);
	pcl::PointCloud<pcl::PointXYZI> getLabeledCloud();
	std::vector <Candidate> getXYZIndexVector();
	sensor_msgs::PointCloud2 cloudToMsg(const pcl::PointCloud<pcl::PointXYZI> cloud);
	std::vector <Candidate> cloudToVectors(const pcl::PointCloud<pcl::PointXYZI> cloud);
	pcl::PointCloud<pcl::PointXYZI> msgToCloud(const sensor_msgs::PointCloud2& msg);

private:
	//Labeled cloud.
	pcl::PointCloud<pcl::PointXYZI> label_cloud_;
	//Candidate vector.
	std::vector <Candidate> xyz_index_vector_;
	//Candidate index.
	int index_;
	//Parameter.
	Cone cone_;
	Detection detection_;
	Erod erod_;
};

#endif