#include "planning/car_model.hpp"

CarModel::CarModel(){}

CarModel::~CarModel(){}

void CarModel::init(Erod erod){
    //Init current measurements.
    steering_angle_ = 0.0;
    velocity_left_ = 0.0;
    velocity_right_ = 0.0;
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
    //Rotate due to tilted VI Sensor.
    float tilting_angle = 11.0;
    tilted_velocity_(0) = local_velocity_(1);
    tilted_velocity_(1) = sin(tilting_angle/180*M_PI)*local_velocity_(0);
    tilted_velocity_(2) = cos(tilting_angle/180*M_PI)*local_velocity_(0); 
}

geometry_msgs::TwistStamped CarModel::getTwistMsg(){
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = timestamp_;
    twist.twist.linear.x = tilted_velocity_(0);
    twist.twist.linear.y  = tilted_velocity_(1);
    twist.twist.linear.z  = tilted_velocity_(2);
    return twist;
}

Eigen::Vector3d CarModel::getVelocity(){return tilted_velocity_;}

void CarModel::setSteeringAngle(double steering_angle){
    steering_angle_ = steering_angle; 
    updateModel();
}

void CarModel::setRearLeftWheelVel(double vel){velocity_left_ = vel;}

void CarModel::setRearRightWheelVel(double vel){velocity_right_ = vel;}

void CarModel::setTimeStamp(ros::Time timestamp){timestamp_ = timestamp;}