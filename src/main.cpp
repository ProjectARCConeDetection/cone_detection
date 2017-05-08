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
ros::Publisher position_pub;
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
	grid_mapper.init(detection);
	image_handler.init(candidate_path, cone, detection);
	cone_detector.init(cone, detection, erod);
	//Init pubs & subs.
	candidates_pub = node.advertise<cone_detection::Label>("/candidates", 10);
	cone_grid_pub = node.advertise<nav_msgs::OccupancyGrid>("/cones_grid", 10);
	labeled_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/labeled_points", 10);
	position_pub = node.advertise<geometry_msgs::Point>("/car_position", 10);
	cloud_sub = node.subscribe("/velodyne_points", 10, cloudCallback);
	raw_image_sub = node.subscribe("/usb_cam/image_raw", 10, imageCallback);
	cones_sub = node.subscribe("/cones", 10, conesCallback);
	rovio_sub = node.subscribe("/rovio/odometry", 10, rovioCallback);
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
	//Pose visualisation.
	geometry_msgs::Point position = grid_mapper.getPoseMsg();
	position_pub.publish(position);
	//Update cone visualisation.
    // image_handler.showCones(grid_mapper.getConeMap(), grid_mapper.getPose());
	//Clear vectors.
	xyz_index_vector.clear();
}

void conesCallback(const cone_detection::Label::ConstPtr& msg){
	//Getting coordinates.
	Eigen::Vector2d cone_position(msg->x, msg->y);
	//Updating grid map. 
	grid_mapper.updateConeMap(cone_position);
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
	Eigen::Vector3d temp_position = Eigen::Vector3d(temp.position.x, temp.position.y, temp.position.z);
	Eigen::Vector4d temp_orientation = Eigen::Vector4d(temp.orientation.x, temp.orientation.y, 
											 		   temp.orientation.z, temp.orientation.w);
	//Transfrom orientation.
	Eigen::Vector4d init_quat(-0.7818,0.0086,-0.0001,-0.6275);
	Eigen::Vector4d init_quat_soll(0,0,0,1);	
	Eigen::Vector4d quat_init_trafo = quat::diffQuaternion(init_quat_soll, init_quat);
  	rovio_pose.orientation = quat::diffQuaternion(temp_orientation, quat_init_trafo);
  	//Transform position.
  	temp_position = Eigen::Vector3d(-temp_position(1), temp_position(0), temp_position(2));
  	temp_position(0) *= cos(11.0/180.0*M_PI);
  	temp_position(1) *= sin(11.0/180.0*M_PI);
 	Eigen::Vector3d trans_vi_laser(-2.3,0,0);
	temp_position += trans_vi_laser;
	rovio_pose.position = transforms::to2D(temp_position);
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
	//Get detection parameter.
	node->getParam("/detection/cone_area", detection->cone_area);
	node->getParam("/detection/searching_length", detection->searching_length);
	node->getParam("/detection/searching_resolution", detection->searching_resolution);
	node->getParam("/detection/searching_width", detection->searching_width);	
	node->getParam("/detection/intensity_threshold", detection->intensity_threshold);
}
