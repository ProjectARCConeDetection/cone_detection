#include "cone_detection/car_model.hpp"

CarModel::CarModel(){}

CarModel::~CarModel(){}

void CarModel::init(Erod erod){
    //Init current measurements.
    steering_angle_ = 0.0;
    velocity_left_ = 0.0;
    velocity_right_ = 0.0;
    //Init position.
    car_pose_.position = Eigen::Vector2d(0.0,0.0);
    car_pose_.orientation = 0;
    //Init timestamp.
    last_update_time_ = ros::Time::now().toSec();
    //Init parameter.
    distance_rear_front_axis_ = erod.distance_wheel_axis;
    width_axis_ = erod.width_wheel_axis; 
}    

void CarModel::updateModel(){
    //Geometric calculations: Equal angular velocities and current center of rotation on
    //horizontal line from rear axle.
    if(fabs(steering_angle_) <= 0.01) {
        local_velocity_(0) = (velocity_right_ + velocity_left_ ) / 2;
        local_velocity_(1) = 0; 
    }
    else { 
        double v_rear = (velocity_right_ + velocity_left_ ) / 2;
        double beta = M_PI / 2 - fabs(steering_angle_);
        double r1 = distance_rear_front_axis_ * tan(beta);
        double r2 = distance_rear_front_axis_ / cos(beta);
        double w = v_rear / r1;
        double v_front = w * r2;
        local_velocity_(0) = cos(steering_angle_) * v_front;
        local_velocity_(1) = sin (steering_angle_) * v_front;
    }
    //Time update.
    double time_delta = ros::Time::now().toSec() - last_update_time_;
    last_update_time_ = ros::Time::now().toSec();
    //Position update.
    Eigen::Vector2d last_position = car_pose_.position;
    car_pose_.position += time_delta*local_velocity_;
    //Orientation update.
    // double delta_x = fabs(car_pose_.position(0) - last_position(0));
    // double delta_y = fabs(car_pose_.position(1) - last_position(1));
    // std::cout << "dx: " << delta_x << " and dy: " << delta_y << std::endl; 
    // car_pose_.orientation = atan2(delta_y,delta_x);
    double angle_vel = local_velocity_(1)/1.33;
    car_pose_.orientation += angle_vel*time_delta;
}

geometry_msgs::Pose CarModel::getPoseMsg(){
    geometry_msgs::Pose pose;
    pose.position.x = car_pose_.position(0);
    pose.position.y = car_pose_.position(1);
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = car_pose_.orientation;
    return pose;
}

geometry_msgs::TwistStamped CarModel::getTwistMsg(){
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.twist.linear.x = local_velocity_(0);
    twist.twist.linear.y  = local_velocity_(1);
    twist.twist.linear.z  = 0.0;
    return twist;
}

void CarModel::setSteeringAngle(double steering_angle){
    steering_angle_ = steering_angle; 
    updateModel();
}

void CarModel::setRearLeftWheelVel(double vel){velocity_left_ = vel;}

void CarModel::setRearRightWheelVel(double vel){velocity_right_ = vel;}