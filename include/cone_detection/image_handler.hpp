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
    void croppCandidates(std::vector < std::vector<double> > xyz_index_vector);
    void transformPointToPixel();
    std::vector<cv::Mat> getCandidateVector();
    std::vector<int> getCandidateIndexVector();
    cv_bridge::CvImagePtr getImagePtr();
    sensor_msgs::Image::ConstPtr getSensorMsg(const cv::Mat base_image);
    cv::Mat croppImage(cv::Mat src, int x_start, int y_start);
    cv::Mat rotateImage(double angle);
    void showCandidates(cv::Mat src, int x_start, int y_start);
    void setCandidatePath(std::string candidate_path);
    void setImgPtr(cv_bridge::CvImagePtr cv_ptr);
    void setImageConstants(int height, int width);
    void setObjectConstants(double height, double width);
private:
	cv_bridge::CvImagePtr cv_ptr_;   
    std::vector<cv::Point2d> image_points_;
    std::vector<cv::Point3d> object_points_;
    std::vector<cv::Mat> candidates_;
    std::vector<int> candidate_indizes_;
    // Cam constants.
    cv::Mat intrisicMat_;
    cv::Mat rVec_;
    cv::Mat tVec_;
    cv::Mat distCoeffs_; 
    int image_height_;
    int image_width_;
    // Object constants.
    double object_height_;
    double object_width_; 
    // File path.
    std::string candidate_path_;

    std::string numberToString(int number);
};
} //namespace cone_detection.

#endif
