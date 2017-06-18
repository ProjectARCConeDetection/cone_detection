#include <cone_detection/image_handler.hpp>

ImageHandler::ImageHandler(){}

ImageHandler::~ImageHandler(){}

void ImageHandler::init(std::string candidate_path, Cone cone, Detection detection){
    //Init camera matrices.
    cam_.initCamMatrices();
    //Getting parameter.
    cone_ = cone;
    detection_ = detection;
    candidate_path_ = candidate_path;
}

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
    cv::Mat src_copy = dst.clone();
    if(image_points_.size() > 0){
        // Draws the rect in the original image and show it.
        for(int i=0; i<image_points_.size();++i){
            cv::Point point = image_points_[i];
            int x_start = point.x - cone_.width_pixel;
            int y_start = point.y + cone_.height_pixel;
            if(x_start < cam_.image_width-cone_.width_pixel && x_start > 0 
              && y_start < cam_.image_height-cone_.height_pixel && y_start > 0){ 
                cv::Mat cropped = croppImage(dst, x_start, y_start);
                //Push back vectors.
                candidates_.push_back(cropped);
                candidate_indizes_.push_back(xyz_index_vector[i].index);
                //Save and show candidate.
                std::string name = candidate_path_ + "candidates/" + numberToString(xyz_index_vector[i].index) + ".jpg";
                cv::imwrite(name, cropped);
                cv::Point pt1(x_start, y_start);
                cv::Point pt2(x_start+cone_.width_pixel,y_start-cone_.height_pixel);
                cv::rectangle(src_copy, pt1, pt2, CV_RGB(255,0,0), 1);
            }
        }
    }
    cv::imshow("candidates", src_copy);
    cv::waitKey(10);   
}

void ImageHandler::showCones(Pose pose){
    //Rotate image.
    cv::Mat dst = rotateImage(180);
    //Convert to global and show boundary boxes.
    std::vector<Eigen::Vector2d> cones;
    for(int i=0; i<cones_global_poses_.size(); ++i)
        cones.push_back(pose.globalToLocal(cones_global_poses_[i]));
    //Search for cones in front of car.
    std::vector<Eigen::Vector2d> cones_in_front;
    for(int i=0; i<cones.size(); ++i)
        if(cones[i](0) > 0) cones_in_front.push_back(cones[i]);
    //Show only image iff no cones in front.
    cv::Mat src_copy = dst.clone();
    if(cones_in_front.size() == 0){
        cv::imshow("cones", src_copy);
        cv::waitKey(5);  
        return;
    }
    //Convert to pixel.
    object_points_.clear();
    image_points_.clear();
    for(int i=0; i<cones_in_front.size(); ++i){
        cv::Point3d point;
        point.z = cones_in_front[i](0);
        point.x = -cones_in_front[i](1);
        point.y = - (-1.33);
        object_points_.push_back(point);
    }
    transformPointToPixel();
    if(image_points_.size() > 0){
        // Draws the rect in the original image and show it.
        for(int i=0; i<image_points_.size();++i){
            cv::Point point = image_points_[i];
            int x_start = point.x - cone_.width_pixel;
            int y_start = point.y + cone_.height_pixel;
            if(x_start < cam_.image_width-cone_.width_pixel && x_start > 0 
              && y_start < cam_.image_height-cone_.height_pixel && y_start > 0){
                cv::Point pt1(x_start, y_start-cone_.height_pixel);
                cv::Point pt2(x_start+cone_.width_pixel, y_start);
                cv::rectangle(src_copy, pt1, pt2, CV_RGB(255,0,0), 1);
            }
        }
    }
    cv::imshow("cones", src_copy);
    cv::waitKey(5);
}

void ImageHandler::transformPointToPixel(){
    //Projecting.
    if(object_points_.size() > 0){
        cv::projectPoints(object_points_, cam_.rVec, cam_.tVec, cam_.intrisicMat, cam_.distCoeffs, image_points_);
    }
}

std::vector<cv::Mat> ImageHandler::getCandidateVector(){return candidates_;}

std::vector<int> ImageHandler::getCandidateIndexVector(){return candidate_indizes_;}

cv_bridge::CvImagePtr ImageHandler::getImagePtr(){return cv_ptr_;}

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
    cv::Rect rect(abs(x_start), abs(y_start-cone_.height_pixel),
                  cone_.width_pixel,cone_.height_pixel);
    cv::Mat cropped(src, rect);
    return cropped;
}

cv::Mat ImageHandler::rotateImage(double angle){
    cv::Mat dst;
    cv::Mat src = cv_ptr_->image;
    cv::Point2f pt(cam_.image_width/2., cam_.image_height/2.);
    cv::Mat rot = cv::getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, rot, cv::Size(cam_.image_width, cam_.image_height));
    return dst;
}

void ImageHandler::setConePosition(Eigen::Vector2d cone_position){
    cones_global_poses_.push_back(cone_position);
}

void ImageHandler::setImgPtr(cv_bridge::CvImagePtr cv_ptr){cv_ptr_ = cv_ptr;}
