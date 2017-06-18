#include "cone_detection/car_model.hpp"

CarModel::CarModel(){}

CarModel::~CarModel(){}

void CarModel::init(Erod erod){
    //Init current measurements.

}    

void CarModel::updateModel(){
    //Geometric calculations: Equal angular velocities and current center of rotation on
    //horizontal line from rear axle.

}

geometry_msgs::Pose CarModel::getPoseMsg(){
    geometry_msgs::Pose pose;
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