#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

//Constants.
const double OBJECT_WIDTH = 50;
const double OBJECT_HEIGHT = 50;
//Publisher and subscriber.
ros::Publisher label_image_pub;
ros::Subscriber image_sub;
//Declaration of functions.
void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

class ImageHandler{
public:
    ImageHandler();
    ~ImageHandler();
    void add3DPoint(cv::Point3d point);
    cv::Point get2DPoint(int index);
    void transformPointToPixel();
private:
    std::vector<cv::Point2d> imagePoints_;
    std::vector<cv::Point3d> objectPoints_;
    cv::Mat intrisicMat_;
    cv::Mat rVec_;
    cv::Mat tVec_;
    cv::Mat distCoeffs_;      
};

ImageHandler::ImageHandler() : intrisicMat_(3,3,cv::DataType<double>::type), 
                               rVec_(3,1,cv::DataType<double>::type), 
                               tVec_(3,1,cv::DataType<double>::type), 
                               distCoeffs_(5,1,cv::DataType<double>::type){
 // Intrisic matrix.
    intrisicMat_.at<double>(0, 0) = 621.701971;
    intrisicMat_.at<double>(1, 0) = 0;
    intrisicMat_.at<double>(2, 0) = 325.157695;
    intrisicMat_.at<double>(0, 1) = 0;
    intrisicMat_.at<double>(1, 1) = 621.971316;
    intrisicMat_.at<double>(2, 1) = 233.170431;
    intrisicMat_.at<double>(0, 2) = 0;
    intrisicMat_.at<double>(1, 2) = 0;
    intrisicMat_.at<double>(2, 2) = 1;
    // Rotation vector.
    rVec_.at<double>(0) = 0;
    rVec_.at<double>(1) = 0;
    rVec_.at<double>(2) = 0;
    // Translation vector.
    tVec_.at<double>(0) = 0;
    tVec_.at<double>(1) = 0;
    tVec_.at<double>(2) = 0;
    // Distortion vector.
    distCoeffs_.at<double>(0) = 0.079264;
    distCoeffs_.at<double>(1) = -0.149967;
    distCoeffs_.at<double>(2) = 0.000716;
    distCoeffs_.at<double>(3) = 0.001749;
    distCoeffs_.at<double>(4) = 0;
}

ImageHandler::~ImageHandler(){}

void ImageHandler::add3DPoint(cv::Point3d point){
    objectPoints_.push_back(point);
}

cv::Point ImageHandler::get2DPoint(int index){
    return imagePoints_[index];
}

void ImageHandler::transformPointToPixel(){

    //Projecting.
    cv::projectPoints(objectPoints_, rVec_, tVec_, intrisicMat_, distCoeffs_, imagePoints_);
    //Printing result.
    std::cout << "Input: " << objectPoints_ << std::endl;
    std::cout << "Output: " << imagePoints_ << std::endl;
}
//Init ImageHandler Object.
ImageHandler image_handler;

int main(int argc, char** argv){
    //Init ros and node.
    ros::init(argc, argv, "transform_test");
    ros::NodeHandle node;
    label_image_pub = node.advertise<sensor_msgs::Image>("/labeled_image", 10);
    image_sub = node.subscribe("/usb_cam/image_raw", 10, imageCallback);
    // Init 3D point.
    cv::Point3d point(0,0,5);
    image_handler.add3DPoint(point);
    //Transform to 2D cam point.
    image_handler.transformPointToPixel();

    //Spinning.
    ros::spin();
	return 0;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    // Draws the rect in the original image and show it.
    cv::Point object_point = image_handler.get2DPoint(0);
    cv::Point pt1(object_point.x-OBJECT_WIDTH/2+320, object_point.y+OBJECT_HEIGHT/2+240);
    cv::Point pt2(pt1.x+OBJECT_WIDTH,pt1.y-OBJECT_HEIGHT);
    cv::rectangle(cv_ptr->image, pt1, pt2, CV_RGB(255,0,0), 1);
    // Update GUI Window.
    cv::imshow("Image window", cv_ptr->image);
    cv::waitKey(3);
    // Output modified video stream.
    label_image_pub.publish(cv_ptr->toImageMsg());
}
