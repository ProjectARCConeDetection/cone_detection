#include <cone_detection/image_handler.hpp>

namespace cone_detection{

ImageHandler::ImageHandler() : intrisicMat_(3,3,cv::DataType<double>::type), 
                               rVec_(3,1,cv::DataType<double>::type), 
                               tVec_(3,1,cv::DataType<double>::type), 
                               distCoeffs_(5,1,cv::DataType<double>::type), 
                               projectionMat_(3,4,cv::DataType<double>::type){
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
    // Rotation vector (laser coord system to open cv coord system).
    rVec_.at<double>(0) = -90;
    rVec_.at<double>(1) = 90;
    rVec_.at<double>(2) = 180;
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
    // Projection Matrix.
    projectionMat_.at<double>(0,0) = 627.292786;
    projectionMat_.at<double>(1,0) = 0;
    projectionMat_.at<double>(2,0) = 0;
    projectionMat_.at<double>(0,1) = 0;
    projectionMat_.at<double>(1,1) = 627.968628;
    projectionMat_.at<double>(2,1) = 0;
    projectionMat_.at<double>(0,2) = 325.534629;
    projectionMat_.at<double>(1,2) = 232.928839;
    projectionMat_.at<double>(2,2) = 1;
    projectionMat_.at<double>(0,3) = 0;
    projectionMat_.at<double>(1,3) = 0;
    projectionMat_.at<double>(2,3) = 0;
}

ImageHandler::~ImageHandler(){}

sensor_msgs::Image::ConstPtr ImageHandler::convertCVToSensorMsg(const cv::Mat image){
    cv::Mat temp_image = cv_ptr_->image;
    cv_bridge::CvImagePtr temp_ptr = cv_ptr_;
    temp_ptr->image = image;
    sensor_msgs::Image::ConstPtr image_ptr = temp_ptr->toImageMsg();
    cv_ptr_->image = temp_image;
    return image_ptr;
}

cv::Mat ImageHandler::croppCandidates(cv::Mat src, int x_start, int y_start, std::string name){
    cv::Mat cropped(src, cv::Rect(abs(x_start), abs(y_start),object_width_,object_height_));
    cv::imwrite(name, cropped);
    return cropped;
}

std::vector<sensor_msgs::Image::ConstPtr> ImageHandler::getCandidateImages(){
    //Rotate Image.
    cv::Mat dst = rotateImage(180);
    // Create vector of candidates.
    std::vector<sensor_msgs::Image::ConstPtr> candidates;
    // Marking candidates.
    if(imagePoints_.size() > 0){
        // Draws the rect in the original image and show it.
        for(int i=0; i<imagePoints_.size();++i){
            cv::Point point = imagePoints_[i];
            int x_start = abs(point.x - object_width_/2); 
            int y_start = abs(point.y - object_height_/2);
            if(x_start < image_width_-object_width_ && x_start > 0 && y_start < image_height_-object_height_ && y_start > 0){ 
                std::string name = "/home/sele/candidates/" + numberToString(index_vector_[i]) + ".jpg";
                candidates.push_back(convertCVToSensorMsg(croppCandidates(dst, x_start, y_start, name)));
                showCandidates(dst, x_start, y_start);
            }
        }
    }
    return candidates;
}

cv_bridge::CvImagePtr ImageHandler::getImagePtr(){
    return cv_ptr_;
}

void ImageHandler::newPointVector(std::vector < std::vector<double> > xyz_index_vector){
    //Clear object vector.
    objectPoints_.clear();
    index_vector_.clear();
    //Push back vector points.
    for(int i=0; i<xyz_index_vector[0].size(); ++i){
        cv::Point3d point;
        point.x = xyz_index_vector[0][i];
        point.y = xyz_index_vector[1][i];
        point.z = xyz_index_vector[2][i];
        objectPoints_.push_back(point);
        int index = xyz_index_vector[3][i];
        index_vector_.push_back(index);
    }
}

std::string ImageHandler::numberToString(int number){
    std::stringstream ss;
    ss << number;
    return ss.str();
}

cv::Mat ImageHandler::rotateImage(double angle){
    cv::Mat dst;
    cv::Mat src = cv_ptr_->image;
    cv::Point2f pt(image_width_/2., image_height_/2.);
    cv::Mat rot = cv::getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, rot, cv::Size(image_width_, image_height_));
    return dst;
}

void ImageHandler::setImgPtr(cv_bridge::CvImagePtr cv_ptr){
    cv_ptr_ = cv_ptr;
}

void ImageHandler::setObjectConstants(double height, double width){
    object_height_ = height;
    object_width_ = width;
}

void ImageHandler::setImageConstants(int height, int width){
    image_height_ = height;
    image_width_ = width;
}

void ImageHandler::showCandidates(cv::Mat src, int x_start, int y_start){
    cv::Mat src_copy = src.clone();
    cv::Point pt1(x_start, y_start);
    cv::Point pt2(x_start+object_width_,y_start+object_height_);
    cv::rectangle(src_copy, pt1, pt2, CV_RGB(255,0,0), 1);
    cv::imshow("Candidates", src_copy);
    cv::waitKey(6);
}

void ImageHandler::transformPointToPixel(){
    //Projecting.
    if(objectPoints_.size() > 0){
        cv::projectPoints(objectPoints_, rVec_, tVec_, intrisicMat_, distCoeffs_, imagePoints_);

        for(int j=0; j<objectPoints_.size();++j)
            std::cout << objectPoints_[j] << " transformed to " << imagePoints_[j] << std::endl;
    }
}

} //namespace cone_detection.