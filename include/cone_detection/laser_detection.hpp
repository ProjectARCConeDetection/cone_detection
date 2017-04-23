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
	void coneMarker(const sensor_msgs::PointCloud2& msg);
	pcl::PointCloud<pcl::PointXYZI> getLabeledCloud();
	std::vector <Candidate> getXYZIndexVector();
	sensor_msgs::PointCloud2 cloudToMsg(const pcl::PointCloud<pcl::PointXYZI> cloud);
	std::vector <Candidate> cloudToVectors(const pcl::PointCloud<pcl::PointXYZI> cloud);
	pcl::PointCloud<pcl::PointXYZI> msgToCloud(const sensor_msgs::PointCloud2& msg);
	void setIntensityThreshold(double intensity_threshold);
	void setLaserHeight(double laser_height);
	void setLengthToVI(double length_to_VI);
	void setObjectHeight(double object_height);
	void setSearchingWidth(double searching_width);

private:
	pcl::PointCloud<pcl::PointXYZI> label_cloud_;
	std::vector <Candidate> xyz_index_vector_;
	double intensity_threshold_;
	double laser_height_;
	double length_to_VI_;
	double object_height_;
	double searching_width_;
	int index_;
};

#endif