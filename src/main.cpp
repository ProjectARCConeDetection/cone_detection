#include <cone_detection/image_handler.hpp>
#include <cone_detection/laser_detection.hpp>

//Constants.
double INTENSITY_THRESHOLD;
double LASER_HEIGHT;
double OBJECT_HEIGHT;
double OBJECT_WIDTH;
double SEARCHING_WIDTH;
//Publisher and Subscriber.
ros::Publisher candidate_image_pub;
ros::Publisher labeled_cloud_pub;
ros::Subscriber cloud_sub;
ros::Subscriber raw_image_sub;
//Init laser recognition class.
cone_detection::LaserDetection cone_detector;
//Init image handler class.
cone_detection::ImageHandler image_handler;
//Decleration of functions.
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
void initConeDetection(ros::NodeHandle* node);
void initImageHandler();
void initLaserDetection();


int main(int argc, char** argv){
	ros::init(argc, argv, "cone_detection");
	ros::NodeHandle node;
	//Getting constants and init subcribers and publishers.
	initConeDetection(&node);
	//Init Laser Detection.
	initLaserDetection();
	//Init image handler.
	initImageHandler();
  	//Spinning.
  	ros::spin();
	return 0;
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
	//Transform msg to xyzi cloud.
	pcl::PointCloud<pcl::PointXYZI> sensor_points;
	sensor_points = cone_detector.msgToCloud(*msg);
	//Labeling points.
	pcl::PointCloud<pcl::PointXYZI> labeled_points;
	labeled_points = cone_detector.coneMarker(sensor_points);
	//Combining image with laser points.
	if (labeled_points.size() > 0){
		cone_detector.cloudToVectors(labeled_points);
		std::vector < std::vector<double> > xyz_index_vector(cone_detector.cloudToVectors(labeled_points));
		image_handler.newPointVector(xyz_index_vector);
		image_handler.transformPointToPixel();
	}
	//Visualisation.
	//Rviz ->labeled point cloud.
	sensor_msgs::PointCloud2 labeled_msg;
	labeled_msg = cone_detector.cloudToMsg(labeled_points);
	labeled_msg.header.stamp = ros::Time::now();
  	labeled_msg.header.frame_id = msg->header.frame_id;
	labeled_cloud_pub.publish(labeled_msg);
	//Image View -> Candidates.
	if(image_handler.getImagePtr() != NULL){
		image_handler.showCandidates();
		candidate_image_pub.publish(image_handler.getImagePtr()->toImageMsg());
	}
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image_handler.setImgPtr(cv_ptr);
}

void initConeDetection(ros::NodeHandle* node){
	//Getting constants.
	node->getParam("/object/height", OBJECT_HEIGHT);
	node->getParam("/object/width", OBJECT_WIDTH);
	node->getParam("/laser/intensity_threshold", INTENSITY_THRESHOLD);
	node->getParam("/laser/height", LASER_HEIGHT);
	node->getParam("/laser/searching_width", SEARCHING_WIDTH);
	//Subscribing raw sensor cloud and publishing label cloud.
	labeled_cloud_pub = node->advertise<sensor_msgs::PointCloud2>("/labeled_points", 10);
	candidate_image_pub = node->advertise<sensor_msgs::Image>("/candidate_img", 10);
	cloud_sub = node->subscribe("/velodyne_points", 10, cloudCallback);
	raw_image_sub = node->subscribe("/usb_cam/image_raw", 10, imageCallback);
}

void initImageHandler(){
	image_handler.setObjectHeight(OBJECT_HEIGHT);
	image_handler.setObjectWidth(OBJECT_WIDTH);
}

void initLaserDetection(){
	cone_detector.setIntensityThreshold(INTENSITY_THRESHOLD);
	cone_detector.setLaserHeight(LASER_HEIGHT);
	cone_detector.setObjectHeight(OBJECT_HEIGHT);
	cone_detector.setSearchingWidth(SEARCHING_WIDTH);
}