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
    if(image_points_.size() > 0){
        // Draws the rect in the original image and show it.
        for(int i=0; i<image_points_.size();++i){
            cv::Point point = image_points_[i];
            int x_start = point.x;
            int y_start = point.y + cone_.height_pixel/2;
            if(x_start < cam_.image_width-cone_.width_pixel && x_start > 0 
              && y_start < cam_.image_height-cone_.height_pixel && y_start > 0){ 
                cv::Mat cropped = croppImage(dst, x_start, y_start);
                //Push back vectors.
                candidates_.push_back(cropped);
                candidate_indizes_.push_back(xyz_index_vector[i].index);
                //Save and show candidate.
                std::string name = candidate_path_ + "candidates/" + numberToString(xyz_index_vector[i].index) + ".jpg";
                cv::imwrite(name, cropped);
                showCandidates(dst, x_start, y_start, "candidates");
            }
        }
    }
    else
        showCandidates(dst, "candidates");
}

void ImageHandler::showCandidates(cv::Mat src, std::string name){
    cv::Mat src_copy = src.clone();
    cv::imshow(name, src_copy);
    cv::waitKey(6);
}

void ImageHandler::showCandidates(cv::Mat src, int x_start, int y_start, std::string name){
    cv::Mat src_copy = src.clone();
    cv::Point pt1(x_start, y_start);
    cv::Point pt2(x_start+cone_.width_pixel,y_start-cone_.height_pixel);
    cv::rectangle(src_copy, pt1, pt2, CV_RGB(255,0,0), 1);
    cv::imshow(name, src_copy);
    cv::waitKey(10);
}

void ImageHandler::showCones(std::vector< std::vector<int> > cone_map, Pose pose){
    //Rotate image.
    cv::Mat dst = rotateImage(180);
    //Get all cones in cone map.
    std::vector<Eigen::Vector2d> cones;
    int x_steps = detection_.searching_length/detection_.searching_resolution;
    int y_steps = detection_.searching_width/detection_.searching_resolution;
    for(int x=0; x<x_steps; ++x)
        for (int y=0; y<y_steps; ++y)
            if(cone_map[x][y] == 1){
                double x_pose = x*detection_.searching_resolution;
                double y_pose = y*detection_.searching_resolution;
                cones.push_back(Eigen::Vector2d(x_pose, y_pose));
            }
    //Convert to global and show boundary boxes.
    for(int i=0; i<cones.size(); ++i)
        cones[i] = pose.globalToLocal(cones[i]);
    //Search for cones in front of car.
    std::vector<Eigen::Vector2d> cones_in_front;
    for(int i=0; i<cones.size(); ++i){
        if(cones[i](0) > 0) cones_in_front.push_back(cones[i]);
        std::cout << "Position: " << cones[i](0) << ", " << cones[i](1) << std::endl;
    }
    //Show only image iff no cones in front.
    if(cones_in_front.size() == 0){
        showCandidates(dst, "cones");
        return;
    }
    //Convert to pixel.
    object_points_.clear();
    image_points_.clear();
    for(int i=0; i<cones_in_front.size(); ++i){
        cv::Point3d point;
        point.z = cones_in_front[i](0);
        point.x = -cones_in_front[i](1);
        point.y = - (-1.4);
        object_points_.push_back(point);
    }
    transformPointToPixel();
    if(image_points_.size() > 0){
        // Draws the rect in the original image and show it.
        for(int i=0; i<image_points_.size();++i){
            cv::Point point = image_points_[i];
            int x_start = point.x;
            int y_start = point.y + cone_.height_pixel/2;
            if(x_start < cam_.image_width-cone_.width_pixel && x_start > 0 
              && y_start < cam_.image_height-cone_.height_pixel && y_start > 0){ 
                showCandidates(dst, x_start, y_start, "cones");
            }
        }
    }
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

void ImageHandler::setImgPtr(cv_bridge::CvImagePtr cv_ptr){cv_ptr_ = cv_ptr;}
