#ifndef CONE_DETECTION_IMAGE_HANDLER_HPP
#define CONE_DETECTION_IMAGE_HANDLER_HPP

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace cone_detection{

class ImageHandler{
public:
    ImageHandler();
    ~ImageHandler();
    void getCandidateImages();
    void newPointVector(std::vector < std::vector<double> > xyz_index_vector);
    cv_bridge::CvImagePtr getImagePtr();
    void setImgPtr(cv_bridge::CvImagePtr cv_ptr);
    void setImageConstants(int height, int width);
    void setObjectConstants(double height, double width);
    void transformPointToPixel();
private:
	cv_bridge::CvImagePtr cv_ptr_;   
    std::vector<cv::Point2d> imagePoints_;
    std::vector<cv::Point3d> objectPoints_;
    std::vector<int> index_vector_;
    // Cam constants.
    cv::Mat intrisicMat_;
    cv::Mat rVec_;
    cv::Mat tVec_;
    cv::Mat distCoeffs_; 
    cv::Mat projectionMat_;
    int image_height_;
    int image_width_;
    // Object constants.
    double object_height_;
    double object_width_; 

    std::string numberToString(int number);
};
} //namespace cone_detection.

#endif