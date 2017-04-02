#include <cone_detection/laser_detection.hpp>

namespace cone_detection{

LaserDetection::LaserDetection(){
	//Initialise candidate index.
	index_ = 0;
}

LaserDetection::~LaserDetection(){}

sensor_msgs::PointCloud2 LaserDetection::cloudToMsg(const pcl::PointCloud<pcl::PointXYZI> cloud){
	sensor_msgs::PointCloud2 msg;
	pcl::PCLPointCloud2 temp_pcl22;
  	pcl::toPCLPointCloud2(cloud, temp_pcl22);
  	pcl_conversions::fromPCL(temp_pcl22, msg);
  	return msg;
}

std::vector < std::vector<double> > LaserDetection::cloudToVectors(const pcl::PointCloud<pcl::PointXYZI> cloud){
	std::vector<double> x,y,z,index;
	for(int i=0; i<cloud.size(); ++i){
		x.push_back(cloud.points[i].x);
		y.push_back(cloud.points[i].y);
		z.push_back(cloud.points[i].z);
		index.push_back(index_);
		index_ ++;
	}
	std::vector < std::vector<double> > vectors;
	vectors.push_back(x);
	vectors.push_back(y);
	vectors.push_back(z);
	vectors.push_back(index);
	return vectors;
}

//Getting point cloud including only cone candidates.
pcl::PointCloud<pcl::PointXYZI> LaserDetection::coneMarker(const pcl::PointCloud<pcl::PointXYZI> cloud){
	//Transforming into x,y,z vectors.
	std::vector<double> x,y,z, intensity;
	for(int i = 0; i<cloud.size(); ++i){
		x.push_back(cloud.points[i].x);
		y.push_back(cloud.points[i].y);
		z.push_back(cloud.points[i].z);
		intensity.push_back(cloud.points[i].intensity);
	}
	//Select candidate points.
	pcl::PointCloud<pcl::PointXYZI> label_cloud;
	for(int i = 0; i<x.size(); ++i){
		//Cones has fixed size and laser a fixed position, so that cones have to 
		//be in a predefined z-Range .
		if(z[i] <= -(laser_height_-object_height_) && z[i] > -laser_height_){
		//Cones located in limited lane, y constraint.
		if(y[i] <= searching_width_ && y[i] >= -searching_width_){
		//Cones only in front of car, visual detection lonely possible here.
		if(x[i] > 0.0){
		//Due to its white color the cones refelcted beams should have high intensity.
		if(intensity[i]>intensity_threshold_){
			label_cloud.push_back(cloud.points[i]);
		}}}}
	}
	return label_cloud;
}

pcl::PointCloud<pcl::PointXYZI> LaserDetection::msgToCloud(const sensor_msgs::PointCloud2& msg){
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::PCLPointCloud2 temp_pcl2;
  	pcl_conversions::toPCL(msg, temp_pcl2);
  	pcl::fromPCLPointCloud2(temp_pcl2, cloud);
  	return cloud;
}

void LaserDetection::setIntensityThreshold(double intensity_threshold){
	intensity_threshold_ = intensity_threshold;}
void LaserDetection::setLaserHeight(double laser_height){
	laser_height_ = laser_height;}
void LaserDetection::setObjectHeight(double object_height){
	object_height_ = object_height;}
void LaserDetection::setSearchingWidth(double searching_width){
	searching_width_ = searching_width;}
}