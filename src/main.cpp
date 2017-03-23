#include <cone_detection/laser_detection.hpp>

#include <sensor_msgs/PointCloud2.h>

//Constants.
double INTENSITY_THRESHOLD;
double LASER_HEIGHT;
double OBJECT_HEIGHT;
double SEARCHING_WIDTH;
//Publisher and Subscriber.
ros::Publisher labeled_cloud_pub;
ros::Subscriber cloud_sub;
//Init laser recognition class.
cone_detection::LaserDetection Cone_Detector;
//Decleration of functions.
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void initConeDetection(ros::NodeHandle* node);
void initLaserDetection();

int main(int argc, char** argv){
	ros::init(argc, argv, "cone_detection");
	ros::NodeHandle node;
	//Getting constants and init subcribers and publishers.
	initConeDetection(&node);
	//Init Laser Detection.
	initLaserDetection();
  	//Spinning.
  	ros::spin();
	return 0;
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
	//Transform msg to xyzi cloud.
	pcl::PointCloud<pcl::PointXYZI> sensor_points;
	sensor_points = Cone_Detector.msgToCloud(*msg);
	//Labeling points.
	pcl::PointCloud<pcl::PointXYZI> labeled_points;
	labeled_points = Cone_Detector.coneMarker(sensor_points);
	//Visualisation.
	sensor_msgs::PointCloud2 labeled_msg;
	labeled_msg = Cone_Detector.cloudToMsg(labeled_points);
	labeled_msg.header.stamp = ros::Time::now();
  	labeled_msg.header.frame_id = msg->header.frame_id;
	labeled_cloud_pub.publish(labeled_msg);
}

void initConeDetection(ros::NodeHandle* node){
	//Getting constants.
	node->getParam("/object/height", OBJECT_HEIGHT);
	node->getParam("/laser/intensity_threshold", INTENSITY_THRESHOLD);
	node->getParam("/laser/height", LASER_HEIGHT);
	node->getParam("/laser/searching_width", SEARCHING_WIDTH);
	//Subscribing raw sensor cloud and publishing label cloud.
	labeled_cloud_pub = node->advertise<sensor_msgs::PointCloud2>("/labeled_points", 10);
	cloud_sub = node->subscribe("/velodyne_points", 10, cloudCallback);
}

void initLaserDetection(){
	Cone_Detector.setIntensityThreshold(INTENSITY_THRESHOLD);
	Cone_Detector.setLaserHeight(LASER_HEIGHT);
	Cone_Detector.setObjectHeight(OBJECT_HEIGHT);
	Cone_Detector.setSearchingWidth(SEARCHING_WIDTH);
}