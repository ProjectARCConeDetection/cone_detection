#include <cone_detection/car_model.hpp>
#include <cone_detection/grid_mapper.hpp>
#include <cone_detection/image_handler.hpp>
#include <cone_detection/laser_detection.hpp>
#include <cone_detection/tools.hpp>

#include <ros/ros.h>
#include <cone_detection/Label.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

//Publisher.
ros::Publisher candidates_pub;
ros::Publisher car_model_pub;
ros::Publisher car_model_pose;
ros::Publisher cone_grid_pub;
ros::Publisher labeled_cloud_pub;
ros::Publisher position_pub;
ros::Subscriber cloud_sub;
ros::Subscriber cones_sub;
ros::Subscriber raw_image_sub;
ros::Subscriber steering_sub;
ros::Subscriber wheel_left_sub;
ros::Subscriber wheel_right_sub;
//Init classes.
CarModel car_model;
LaserDetection cone_detector;
ImageHandler image_handler;
GridMapper grid_mapper;
int counter=0;
//Decleration of functions.
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void conesCallback(const cone_detection::Label::ConstPtr& msg);
void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
void steeringCallback(const std_msgs::Float64::ConstPtr& msg);
void wheelLeftCallback(const std_msgs::Float64::ConstPtr& msg);
void wheelRightCallback(const std_msgs::Float64::ConstPtr& msg);
void publishCandidates(std::vector <Candidate> xyz_index_vector,
					   std::vector<cv::Mat> candidates, std::vector<int> candidate_indizes);
void gettingParameter(ros::NodeHandle* node, std::string* candidate_path,
					  Cone* cone, Detection* detection, Erod* erod);

int main(int argc, char** argv){
	ros::init(argc, argv, "cone_detector");
	ros::NodeHandle node;
	//Getting parameter.
	std::string candidate_path;
	Cone cone; Detection detection; Erod erod;
	gettingParameter(&node,&candidate_path,&cone,&detection,&erod);
	//Init classes.
	car_model.init(erod);
	grid_mapper.init(detection);
	image_handler.init(candidate_path, cone, detection);
	cone_detector.init(cone, detection, erod);
	//Init pubs & subs.
	car_model_pub = node.advertise<geometry_msgs::TwistStamped>("/car_model_velocity", 1);
	candidates_pub = node.advertise<cone_detection::Label>("/candidates", 10);
	cone_grid_pub = node.advertise<nav_msgs::OccupancyGrid>("/cones_grid", 10);
	labeled_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/labeled_points", 10);
	position_pub = node.advertise<geometry_msgs::Pose>("/car_pose", 10);
	cloud_sub = node.subscribe("/velodyne_points", 10, cloudCallback);
	raw_image_sub = node.subscribe("/usb_cam/image_raw", 10, imageCallback);
	cones_sub = node.subscribe("/cones", 100, conesCallback);
	steering_sub = node.subscribe("/state_steering_angle", 1, steeringCallback);
	wheel_left_sub = node.subscribe("/wheel_rear_left", 1, wheelLeftCallback);
	wheel_right_sub = node.subscribe("/wheel_rear_right", 1, wheelRightCallback);
  	//Spinning.
  	ros::Rate rate(10);
  	while(ros::ok()){
  		ros::spinOnce();
  		rate.sleep();
  	}
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
	//Update cone visualisation.
    image_handler.showCones(grid_mapper.getPose());
	//Clear vectors.
	xyz_index_vector.clear();
}

void conesCallback(const cone_detection::Label::ConstPtr& msg){
	//Getting coordinates.
	Eigen::Vector2d cone_position(msg->x, msg->y);
	//Updating grid map. 
	grid_mapper.updateConeMap(cone_position);
	image_handler.setConePosition(cone_position);
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image_handler.setImgPtr(cv_ptr);
}

void steeringCallback(const std_msgs::Float64::ConstPtr& msg){
	car_model.setSteeringAngle(msg->data);
	//Update grid pose.
	geometry_msgs::Pose car_pose_msg = car_model.getPoseMsg();
	Pose car_model_pose; 
	car_model_pose.position = Eigen::Vector2d(car_pose_msg.position.x, car_pose_msg.position.y);
	car_model_pose.orientation = car_pose_msg.orientation.w; 
	grid_mapper.setPose(car_model_pose);
	//Velocity and Pose update.
  	car_model_pub.publish(car_model.getTwistMsg());
  	position_pub.publish(car_pose_msg);
}

void wheelLeftCallback(const std_msgs::Float64::ConstPtr& msg){
	car_model.setRearLeftWheelVel(msg->data);
}
void wheelRightCallback(const std_msgs::Float64::ConstPtr& msg){
	car_model.setRearRightWheelVel(msg->data);
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
		Eigen::Vector2d global_position = grid_mapper.convertLocalToGlobal(xyz_index_vector[xyz_index]);
		label_msg.x = global_position(0);
		label_msg.y = global_position(1);
		label_msg.label = false;
		candidates_pub.publish(label_msg);
	}
}

void gettingParameter(ros::NodeHandle* node, std::string* candidate_path,
					  Cone* cone, Detection* detection, Erod* erod){
	//Get candidate path.
	node->getParam("/candidate_path", *candidate_path);
	//Get cone parameter.
	node->getParam("/cone/height_meter", cone->height_meter);
	node->getParam("/cone/height_pixel", cone->height_pixel);
	node->getParam("/cone/width_pixel", cone->width_pixel);
	//Get erod parameter.
	node->getParam("/erod/length_laser_to_VI", erod->length_laser_to_VI);
	node->getParam("/erod/distance_wheel_axis", erod->distance_wheel_axis);
	node->getParam("/erod/height_laser", erod->height_laser);
	node->getParam("/erod/width_wheel_axis", erod->width_wheel_axis);
	//Get detection parameter.
	node->getParam("/detection/cone_area_x", detection->cone_area_x);
	node->getParam("/detection/cone_area_y", detection->cone_area_y);
	node->getParam("/detection/searching_length", detection->searching_length);
	node->getParam("/detection/searching_resolution", detection->searching_resolution);
	node->getParam("/detection/searching_width", detection->searching_width);	
	node->getParam("/detection/intensity_threshold", detection->intensity_threshold);
}
