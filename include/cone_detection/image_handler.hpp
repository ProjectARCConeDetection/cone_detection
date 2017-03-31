#ifndef CONE_DETECTION_IMAGE_HANDLER_HPP
#define CONE_DETECTION_IMAGE_HANDLER_HPP

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

namespace cone_detection{

class ImageHandler{
public:
    ImageHandler();
    ~ImageHandler();
    void newPointVector(std::vector < std::vector<double> > xyz_index_vector);
    cv_bridge::CvImagePtr getImagePtr();
    void setImgPtr(cv_bridge::CvImagePtr cv_ptr);
    void setObjectHeight(double height);
    void setObjectWidth(double width);
    void showCandidates();
    void transformPointToPixel();
private:
	cv_bridge::CvImagePtr cv_ptr_;   
    std::vector<cv::Point2d> imagePoints_;
    std::vector<cv::Point3d> objectPoints_;
    cv::Mat intrisicMat_;
    cv::Mat rVec_;
    cv::Mat tVec_;
    cv::Mat distCoeffs_; 
    double object_height_;
    double object_width_;  
};
} //namespace cone_detection.

#endif