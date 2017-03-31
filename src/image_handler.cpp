#include <cone_detection/image_handler.hpp>

namespace cone_detection{

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
    tVec_.at<double>(0) = M_PI/2;
    tVec_.at<double>(1) = 0;
    tVec_.at<double>(2) = M_PI/2;
    // Distortion vector.
    distCoeffs_.at<double>(0) = 0.079264;
    distCoeffs_.at<double>(1) = -0.149967;
    distCoeffs_.at<double>(2) = 0.000716;
    distCoeffs_.at<double>(3) = 0.001749;
    distCoeffs_.at<double>(4) = 0;
}

ImageHandler::~ImageHandler(){}

cv_bridge::CvImagePtr ImageHandler::getImagePtr(){
    return cv_ptr_;
}

void ImageHandler::newPointVector(std::vector < std::vector<double> > xyz_index_vector){
    //Clear object vector.
    objectPoints_.clear();
    //Push back vector points.
    for(int i=0; i<xyz_index_vector.size(); ++i){
        cv::Point3d point;
        point.x = xyz_index_vector[i][0];
        point.y = xyz_index_vector[i][1];
        point.z = xyz_index_vector[i][2];
        objectPoints_.push_back(point);
    }
}

void ImageHandler::setImgPtr(cv_bridge::CvImagePtr cv_ptr){
    cv_ptr_ = cv_ptr;
}

void ImageHandler::setObjectHeight(double height){
    object_height_ = height;
}

void ImageHandler::setObjectWidth(double width){
    object_width_ = width;
}

void ImageHandler::showCandidates(){
    if(imagePoints_.size() > 0){
        // Draws the rect in the original image and show it.
        for(int i=0; i<imagePoints_.size();++i){
            cv::Point point = imagePoints_[i];
            cv::Point pt1(point.x-object_width_/2+320, point.y+object_height_/2+240);
            cv::Point pt2(pt1.x+object_width_,pt1.y-object_height_);
            cv::rectangle(cv_ptr_->image, pt1, pt2, CV_RGB(255,0,0), 1);
        }
    // Update GUI Window.
    cv::imshow("Candidates", cv_ptr_->image);
    cv::waitKey(3);
    }
}

void ImageHandler::transformPointToPixel(){
    //Projecting.
    if(objectPoints_.size() > 0)
        cv::projectPoints(objectPoints_, rVec_, tVec_, intrisicMat_, distCoeffs_, imagePoints_);
}

} //namespace cone_detection.