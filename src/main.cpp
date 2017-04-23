#include <cone_detection/grid_mapper.hpp>
#include <cone_detection/image_handler.hpp>
#include <cone_detection/laser_detection.hpp>
#include <cone_detection/tools.hpp>

#include <ros/ros.h>
#include <cone_detection/Label.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

//Publisher.
ros::Publisher candidates_pub;
ros::Publisher cone_grid_pub;
ros::Publisher labeled_cloud_pub;
ros::Subscriber cloud_sub;
ros::Subscriber raw_image_sub;
ros::Subscriber cones_sub;
ros::Subscriber rovio_sub;
//Init classes.
LaserDetection cone_detector;
ImageHandler image_handler;
GridMapper grid_mapper;
//Decleration of functions.
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void conesCallback(const cone_detection::Label::ConstPtr& msg);
void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
void rovioCallback(const nav_msgs::Odometry::ConstPtr& msg);
void publishCandidates(std::vector <Candidate> xyz_index_vector,
					   std::vector<cv::Mat> candidates, std::vector<int> candidate_indizes);
void initConeDetection(ros::NodeHandle* node);
void initGridMapper(ros::NodeHandle* node);
void initImageHandler(ros::NodeHandle* node);
void initLaserDetection(ros::NodeHandle* node);

int main(int argc, char** argv){
	ros::init(argc, argv, "cone_detection");
	ros::NodeHandle node;
	//Getting constants and init subcribers and publishers.
	initConeDetection(&node);
	initImageHandler(&node);
	initGridMapper(&node);
	initLaserDetection(&node);
  	//Spinning.
  	ros::spin();
	return 0;
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
	//Detect candidates.
	cone_detector.coneMarker(*msg);
	std::vector <Candidate> xyz_index_vector = cone_detector.getXYZIndexVector();
	//Getting cropped candidate images.
	if(image_handler.getImagePtr()!=NULL){
		image_handler.croppCandidates(xyz_index_vector);
		std::vector<cv::Mat> candidates = image_handler.getCandidateVector();
    	std::vector<int> candidate_indizes = image_handler.getCandidateIndexVector();
		//Publish Candidates.
		publishCandidates(xyz_index_vector, candidates, candidate_indizes);
		//Clear vectors.
		candidates.clear();
		candidate_indizes.clear();
	}
	//Visualisation.
	//Rviz ->labeled point cloud.
	sensor_msgs::PointCloud2 labeled_msg;
	labeled_msg = cone_detector.cloudToMsg(cone_detector.getLabeledCloud());
	labeled_msg.header.frame_id = msg->header.frame_id;
	labeled_cloud_pub.publish(labeled_msg);
	//Grid map visualisation.
	nav_msgs::OccupancyGrid cone_grid = grid_mapper.getOccupancyGridMap();
	cone_grid_pub.publish(cone_grid);
	//Clear vectors.
	xyz_index_vector.clear();
}

void conesCallback(const cone_detection::Label::ConstPtr& msg){
	//Getting coordinates.
	Eigen::Vector3d local(msg->x, msg->y,0);
	//Updating grid map. 
	grid_mapper.updateConeMap(local);
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image_handler.setImgPtr(cv_ptr);
}

void rovioCallback(const nav_msgs::Odometry::ConstPtr& msg){
	//Get rovio pose.
	Pose rovio_pose;
	geometry_msgs::Pose temp = msg->pose.pose;
	Eigen::Vector3d temp_position = Eigen::Vector3d(temp.position.x, temp.position.y, 0);
	Eigen::Vector4d temp_orientation = Eigen::Vector4d(temp.orientation.x, temp.orientation.y, 
											 		   temp.orientation.z, temp.orientation.w);
	//Transfrom orientation.
	Eigen::Vector4d init_quat(-0.7818,0.0086,-0.0001,-0.6275);
	Eigen::Vector4d init_quat_soll(0,0,0,1);	
	Eigen::Vector4d quat_init_trafo = quat::diffQuaternion(init_quat, init_quat_soll);
  	rovio_pose.orientation = quat::diffQuaternion(quat_init_trafo, temp_orientation);
  	//Transform position.
  	temp_position = Eigen::Vector3d(-temp_position(1), temp_position(0), temp_position(2));
  	Eigen::Vector3d trans_vi_laser(-2.3,0,0);
	temp_position += trans_vi_laser;
	rovio_pose.position = temp_position;
  	//Set pose.
  	grid_mapper.setPose(rovio_pose);
}

void publishCandidates(std::vector <Candidate> xyz_index_vector,
					   std::vector<cv::Mat> candidates, std::vector<int> candidate_indizes){
	//Comparing xyz_index_points with candidate_indizes to get only the points
	//in the image (no error while cropping).
	for(int i=0;i<candidates.size();++i){
		int current_index = candidate_indizes[i];
		int xyz_index = vector::getSameIndex(xyz_index_vector, current_index);
		if(xyz_index > xyz_index_vector.size() || xyz_index < 0) continue;
		// Create and publish candidate.
		cone_detection::Label label_msg;
		label_msg.image = image_handler.getSensorMsg(candidates[i]);
		label_msg.x = xyz_index_vector[xyz_index].x;
		label_msg.y = xyz_index_vector[xyz_index].y;
		label_msg.label = false;
		candidates_pub.publish(label_msg);
	}
}

void initConeDetection(ros::NodeHandle* node){
	//Subscribing raw sensor cloud and publishing label cloud.
	candidates_pub = node->advertise<cone_detection::Label>("/candidates", 10);
	cone_grid_pub = node->advertise<nav_msgs::OccupancyGrid>("/cones_grid", 10);
	labeled_cloud_pub = node->advertise<sensor_msgs::PointCloud2>("/labeled_points", 10);
	cloud_sub = node->subscribe("/velodyne_points", 10, cloudCallback);
	raw_image_sub = node->subscribe("/usb_cam/image_raw", 10, imageCallback);
	cones_sub = node->subscribe("/cones", 10, conesCallback);
	rovio_sub = node->subscribe("/rovio/odometry", 10, rovioCallback);
}

void initGridMapper(ros::NodeHandle* node){
	double height, resolution, width;
	node->getParam("/laser/searching_length", height);
	node->getParam("/laser/searching_resolution", resolution);
	node->getParam("/laser/searching_width", width);
	grid_mapper.setGridHeight(height);
	grid_mapper.setGridResolution(resolution);
	grid_mapper.setGridWidth(width);
	grid_mapper.initConeMap();
}

void initImageHandler(ros::NodeHandle* node){
	int image_height, image_width;
	double object_height_pixel, object_width_pixel;
	std::string candidate_path;
	node->getParam("/cam/image_height", image_height);
	node->getParam("/cam/image_width", image_width);
	node->getParam("/object/height_pixel", object_height_pixel);
	node->getParam("/object/width_pixel", object_width_pixel);
	node->getParam("/candidate_path", candidate_path);
	image_handler.setCandidatePath(candidate_path);
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
