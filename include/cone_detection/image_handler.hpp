#ifndef CONE_DETECTION_IMAGE_HANDLER_HPP
#define CONE_DETECTION_IMAGE_HANDLER_HPP

#include <cone_detection/laser_detection.hpp>
#include <cone_detection/tools.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <sstream>
#include <string>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

class ImageHandler{
public:
    ImageHandler();
    ~ImageHandler();
    void init(std::string candidate_path, Cone cone);
    void croppCandidates(std::vector <Candidate> xyz_index_vector);
    void transformPointToPixel();
    std::vector<cv::Mat> getCandidateVector();
    std::vector<int> getCandidateIndexVector();
    cv_bridge::CvImagePtr getImagePtr();
    sensor_msgs::Image getSensorMsg(const cv::Mat base_image);
    cv::Mat croppImage(cv::Mat src, int x_start, int y_start);
    cv::Mat rotateImage(double angle);
    void showCandidates(cv::Mat src);
    void showCandidates(cv::Mat src, int x_start, int y_start);
    void setImgPtr(cv_bridge::CvImagePtr cv_ptr);
private:
	cv_bridge::CvImagePtr cv_ptr_;   
    std::vector<cv::Point2d> image_points_;
    std::vector<cv::Point3d> object_points_;
    std::vector<cv::Mat> candidates_;
    std::vector<int> candidate_indizes_;
    //Parameter.
    Cam cam_;
    Cone cone_;
    // File path.
    std::string candidate_path_;
    //Helper functions.
    std::string numberToString(int number);
};

#endif
