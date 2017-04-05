#include <cone_detection/image_handler.hpp>
#include <cone_detection/laser_detection.hpp>

//Publisher and Subscriber.
ros::Publisher candidate_images_pub;
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
void initImageHandler(ros::NodeHandle* node);
void initLaserDetection(ros::NodeHandle* node);

int main(int argc, char** argv){
	ros::init(argc, argv, "cone_detection");
	ros::NodeHandle node;
	//Getting constants and init subcribers and publishers.
	initConeDetection(&node);
	//Init Laser Detection.
	initLaserDetection(&node);
	//Init image handler.
	initImageHandler(&node);
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
	if (labeled_points.size() > 0 && image_handler.getImagePtr() != NULL){
		std::vector < std::vector<double> > xyz_index_vector;
		xyz_index_vector = cone_detector.cloudToVectors(labeled_points); 
		image_handler.newPointVector(xyz_index_vector);
		image_handler.transformPointToPixel();
		xyz_index_vector.clear();
	}
	//Getting cropped candidate images.
	if(image_handler.getImagePtr() != NULL){
		std::vector<sensor_msgs::Image::ConstPtr> candidates = image_handler.getCandidateImages();
		for (int i=0; i < candidates.size(); i++) candidate_images_pub.publish(candidates[i]);
	}
	//Visualisation.
	//Rviz ->labeled point cloud.
	sensor_msgs::PointCloud2 labeled_msg;
	labeled_msg = cone_detector.cloudToMsg(labeled_points);
	labeled_msg.header.stamp = ros::Time::now();
  	labeled_msg.header.frame_id = msg->header.frame_id;
	labeled_cloud_pub.publish(labeled_msg);
	labeled_points.clear();
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image_handler.setImgPtr(cv_ptr);
}

void initConeDetection(ros::NodeHandle* node){
	//Subscribing raw sensor cloud and publishing label cloud.
	candidate_images_pub = node->advertise<sensor_msgs::Image>("/candidates", 10);
	labeled_cloud_pub = node->advertise<sensor_msgs::PointCloud2>("/labeled_points", 10);
	cloud_sub = node->subscribe("/velodyne_points", 10, cloudCallback);
	raw_image_sub = node->subscribe("/usb_cam/image_raw", 10, imageCallback);
}

void initImageHandler(ros::NodeHandle* node){
	//Getting constants.
	int image_height, image_width;
	double object_height_pixel, object_width_pixel;
	node->getParam("/cam/image_height", image_height);
	node->getParam("/cam/image_width", image_width);
	node->getParam("/object/height_pixel", object_height_pixel);
	node->getParam("/object/width_pixel", object_width_pixel);
	image_handler.setObjectConstants(object_height_pixel, object_width_pixel);
	image_handler.setImageConstants(image_height, image_width);
}

void initLaserDetection(ros::NodeHandle* node){
	double intensity_threshold, laser_height, length_to_VI, object_height_meter, searching_width;
	node->getParam("/laser/intensity_threshold", intensity_threshold);
	node->getParam("/laser/height", laser_height);
	node->getParam("/object/height_meter", object_height_meter);
	node->getParam("/laser/length_to_VI", length_to_VI);
	node->getParam("/laser/searching_width", searching_width);
	cone_detector.setIntensityThreshold(intensity_threshold);
	cone_detector.setLaserHeight(laser_height);
	cone_detector.setLengthToVI(length_to_VI);
	cone_detector.setObjectHeight(object_height_meter);
	cone_detector.setSearchingWidth(searching_width);
}