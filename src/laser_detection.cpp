#include <cone_detection/laser_detection.hpp>

LaserDetection::LaserDetection(){
	//Initialise candidate index.
	index_ = 0;
}

LaserDetection::~LaserDetection(){}

//Getting point cloud including only cone candidates.
void LaserDetection::coneMarker(const sensor_msgs::PointCloud2& msg){
	//Clear vectors.
	label_cloud_.clear();
	xyz_index_vector_.clear();
	//Transform msg to xyzi cloud.
	pcl::PointCloud<pcl::PointXYZI> cloud;
	cloud = msgToCloud(msg);
	//Transforming into x,y,z vectors.
	std::vector<double> x,y,z, intensity;
	for(int i = 0; i<cloud.size(); ++i){
		x.push_back(cloud.points[i].x);
		y.push_back(cloud.points[i].y);
		z.push_back(cloud.points[i].z);
		intensity.push_back(cloud.points[i].intensity);
	}
	//Select candidate points.
	for(int i = 0; i<x.size(); ++i){
		//Cones has fixed size and laser a fixed position, so that cones have to 
		//be in a predefined z-Range .
		if(z[i] <= -(laser_height_-object_height_) && z[i] > -laser_height_){
		//Cones located in limited lane, y constraint.
		if(y[i] <= searching_width_ && y[i] >= -searching_width_){
		//Cones only in front of car, visual detection lonely possible here.
		if(x[i] > length_to_VI_){
		//Due to its white color the cones refelcted beams should have high intensity.
		if(intensity[i]>intensity_threshold_){
			label_cloud_.push_back(cloud.points[i]);
		}}}}
	}
	//Transfrom to vector.
	xyz_index_vector_ = cloudToVectors(label_cloud_); 
}

pcl::PointCloud<pcl::PointXYZI> LaserDetection::getLabeledCloud(){
	return label_cloud_;}

std::vector <Candidate> LaserDetection::getXYZIndexVector(){
	return xyz_index_vector_;}

sensor_msgs::PointCloud2 LaserDetection::cloudToMsg(const pcl::PointCloud<pcl::PointXYZI> cloud){
	sensor_msgs::PointCloud2 msg;
	pcl::PCLPointCloud2 temp_pcl22;
  	pcl::toPCLPointCloud2(cloud, temp_pcl22);
  	pcl_conversions::fromPCL(temp_pcl22, msg);
  	msg.header.stamp = ros::Time::now();
  	return msg;
}

std::vector <Candidate> LaserDetection::cloudToVectors(const pcl::PointCloud<pcl::PointXYZI> cloud){
	std::vector<Candidate> xyz_index_vector;
	for(int i=0; i<cloud.size(); ++i){
		Candidate temp;
		temp.x = cloud.points[i].x;
		temp.y = cloud.points[i].y;
		temp.z = cloud.points[i].z;
		temp.index = index_;
		index_ ++;
		xyz_index_vector.push_back(temp);
	}
	return xyz_index_vector;
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
void LaserDetection::setLengthToVI(double length_to_VI){
	length_to_VI_ = length_to_VI;}
void LaserDetection::setObjectHeight(double object_height){
	object_height_ = object_height;}
void LaserDetection::setSearchingWidth(double searching_width){
	searching_width_ = searching_width;}
