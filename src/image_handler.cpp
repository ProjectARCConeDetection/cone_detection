#include <cone_detection/image_handler.hpp>

ImageHandler::ImageHandler() : intrisicMat_(3,3,cv::DataType<double>::type), 
                               rVec_(3,1,cv::DataType<double>::type), 
                               tVec_(3,1,cv::DataType<double>::type), 
                               distCoeffs_(5,1,cv::DataType<double>::type){
    // Intrisic matrix.
    intrisicMat_.at<double>(0, 0) = 621.701971;
    intrisicMat_.at<double>(0, 1) = 0;
    intrisicMat_.at<double>(0, 2) = 325.157695;
    intrisicMat_.at<double>(1, 0) = 0;
    intrisicMat_.at<double>(1, 1) = 621.971316;
    intrisicMat_.at<double>(1, 2) = 233.170431;
    intrisicMat_.at<double>(2, 0) = 0;
    intrisicMat_.at<double>(2, 1) = 0;
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

void ImageHandler::croppCandidates(std::vector <Candidate> xyz_index_vector){
    //Clear vectors.
    object_points_.clear();
    image_points_.clear();
    candidates_.clear();
    candidate_indizes_.clear();
    //Push back vector points (laser coords -> cam coords).
    for(int i=0; i<xyz_index_vector.size(); ++i){
        cv::Point3d point;
        point.z = xyz_index_vector[i].x;
        point.x = -xyz_index_vector[i].y;
        point.y = -xyz_index_vector[i].z;
        object_points_.push_back(point);
    }
    //Transform to pixel points.
    transformPointToPixel();
    //Rotate Image.
    cv::Mat dst = rotateImage(180);
    // Marking candidates.
    if(image_points_.size() > 0){
        // Draws the rect in the original image and show it.
        for(int i=0; i<image_points_.size();++i){
            cv::Point point = image_points_[i];
            // int x_start = point.x - object_width_/2;
            // int y_start = point.y - object_height_/2;
            int x_start = point.x;
            int y_start = point.y + object_height_/2;
            if(x_start < image_width_-object_width_ && x_start > 0 && y_start < image_height_-object_height_ && y_start > 0){ 
                cv::Mat cropped = croppImage(dst, x_start, y_start);
                //Push back vectors.
                candidates_.push_back(cropped);
                candidate_indizes_.push_back(xyz_index_vector[i].index);
                //Save and show candidate.
                std::string name = candidate_path_ + numberToString(xyz_index_vector[i].index) + ".jpg";
                cv::imwrite(name, cropped);
                showCandidates(dst, x_start, y_start);
            }
        }
    }
    else
        showCandidates(dst);
}

void ImageHandler::showCandidates(cv::Mat src){
    cv::Mat src_copy = src.clone();
    cv::imshow("Candidates", src_copy);
    cv::waitKey(6);
}

void ImageHandler::showCandidates(cv::Mat src, int x_start, int y_start){
    cv::Mat src_copy = src.clone();
    cv::Point pt1(x_start, y_start);
    cv::Point pt2(x_start+object_width_,y_start-object_height_);
    cv::rectangle(src_copy, pt1, pt2, CV_RGB(255,0,0), 1);
    cv::imshow("Candidates", src_copy);
    cv::waitKey(10);
}

void ImageHandler::transformPointToPixel(){
    //Projecting.
    if(object_points_.size() > 0){
        cv::projectPoints(object_points_, rVec_, tVec_, intrisicMat_, distCoeffs_, image_points_);
    }
}

std::vector<cv::Mat> ImageHandler::getCandidateVector(){
    return candidates_;}

std::vector<int> ImageHandler::getCandidateIndexVector(){
    return candidate_indizes_;}

cv_bridge::CvImagePtr ImageHandler::getImagePtr(){
    return cv_ptr_;}

sensor_msgs::Image ImageHandler::getSensorMsg(const cv::Mat base_image){
    cv::Mat temp_image = cv_ptr_->image;
    cv_bridge::CvImagePtr temp_ptr = cv_ptr_;
    temp_ptr->image = base_image;
    sensor_msgs::Image image_ptr = *temp_ptr->toImageMsg();
    cv_ptr_->image = temp_image;
    return image_ptr;
}

std::string ImageHandler::numberToString(int number){
    std::stringstream ss;
    ss << number;
    return ss.str();
}

cv::Mat ImageHandler::croppImage(cv::Mat src, int x_start, int y_start){
    cv::Mat cropped(src, cv::Rect(abs(x_start), abs(y_start-object_height_),object_width_,object_height_));
    return cropped;
}

cv::Mat ImageHandler::rotateImage(double angle){
    cv::Mat dst;
    cv::Mat src = cv_ptr_->image;
    cv::Point2f pt(image_width_/2., image_height_/2.);
    cv::Mat rot = cv::getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, rot, cv::Size(image_width_, image_height_));
    return dst;
}

void ImageHandler::setCandidatePath(std::string candidate_path){
    candidate_path_ = candidate_path;}

void ImageHandler::setImgPtr(cv_bridge::CvImagePtr cv_ptr){
    cv_ptr_ = cv_ptr;}

void ImageHandler::setObjectConstants(double height, double width){
    object_height_ = height;
    object_width_ = width;
}

void ImageHandler::setImageConstants(int height, int width){
    image_height_ = height;
    image_width_ = width;
}
