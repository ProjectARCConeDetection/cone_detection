#include <planning/tools.hpp>

void AckermannControl::print(){
    std::cout << "------------Stellgroessen -----------------" << std::endl;
    std::cout << "steering: " << steering_angle << " velocity " << velocity << std::endl;
 }


namespace path{

double distanceBetween(int base_index,int target_index,std::vector<Eigen::Vector2d> positions){
    double distance = 0.0;
    for (int i=base_index; i<=target_index; ++i){
      if(i>(int)positions.size()-1 || i<0) break;
      distance += (positions[i] - positions[i+1]).norm();
    }
    return distance;
}

double distanceToIndex(int target_index,std::vector<Eigen::Vector2d> positions, int current_array_index){
    double distance = 0.0;
    for (int i=current_array_index; i<target_index; ++i){
      if(i>(int)positions.size()-1 || i<0) break;
      distance += (positions[i] - positions[i+1]).norm();
    }
    return distance;
}

int indexOfDistanceFront(double max_distance,std::vector<Eigen::Vector2d> positions, Eigen::Vector2d position){
    double distance = 0.0;
    int index = currentArray(positions, position);
    while((distance < max_distance) && (index < positions.size()-2)){
        distance += distanceBetween(index,index+1,positions);
        index++;
    }
    return index+1;
}
int currentArray(std::vector<Eigen::Vector2d> positions, Eigen::Vector2d position) {
    double min_distance=50;
    double current_array_index;
    for(int i=0; i<positions.size(); i++) {
        double check_distance = (positions[i]-position).norm();
        if(check_distance<min_distance) {
            min_distance = check_distance;
            current_array_index = i;
    }}
    return current_array_index;
}
}//namespace path.

//Car Model.
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

//VCU Interface.
VCUInterface::VCUInterface(){}

VCUInterface::~VCUInterface(){
    //Back to manuell mode.
    send_msg("am", 0.0, true);
}

void VCUInterface::init(bool use_vcu, CarModel* car_model){
    //Init car model.
    car_model_ = car_model;
    //Use vcu.
    use_vcu_ = use_vcu;
    if(!use_vcu_) return;
    //Getting parameters.
    memset((char *) &si_me_, 0, sizeof(si_me_));
    si_me_.sin_family = AF_INET;
    si_me_.sin_port = htons(8010);
    si_me_.sin_addr.s_addr = htonl(INADDR_ANY);
    si_VCU_.sin_family = AF_INET;
    si_VCU_.sin_port = htons(8001);
    si_VCU_.sin_addr.s_addr = inet_addr("10.0.0.8");
    slen_ = sizeof(si_other_);
    //Set up vcu udp binding (socket + binding).
    if ((sock_=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) printError("socket");
    if(bind(sock_, (struct sockaddr*)&si_me_, sizeof(si_me_)) == -1) printError("binding");
}

void VCUInterface::recv_car_model(){
    if(!use_vcu_) return;
    //Receiving.
    int recv_len;
    char buffer_in[buflen_];
    if ((recv_len = recvfrom(sock_, buffer_in, buflen_, 0, (struct sockaddr *) &si_other_, &slen_)) == -1) 
       printError("receiving");
    //Convert msg to string.
    std::string msg;
    for (int i = 0; i < recv_len; ++i) msg += buffer_in[i];
    //Get kind of msg and value.
    std::string kind = msg.substr(0,2);
    std::string value_string(msg, 3, msg.length()-1);
    const char *buffer = value_string.c_str();
    double value = atof(buffer);
    //Answers.
    if(kind == "rr") car_model_->setRearRightWheelVel(value);
    else if(kind == "rl") car_model_->setRearLeftWheelVel(value);
    else if(kind == "si"){
        value = (value-1000)*M_PI/180.0;
        car_model_->setSteeringAngle(value);
    }
}

void VCUInterface::send_msg(std::string symbol, double msg, bool requirement){
    //Convert msg to string.
    std::ostringstream stream;
    stream << msg;
    std::string value_string = stream.str();
    //Create char array.
    std::string sending = symbol + ":" + value_string;
    const char *buffer_out = sending.c_str();
    //Sending to VCU.
    if(requirement && use_vcu_)
       if (sendto(sock_, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_VCU_, slen_) == -1) 
          printError("sending " + symbol);
}

void VCUInterface::send_msg(std::string symbol, double msg, bool requirement, 
                            double max_value, double min_value, double shift){
    //Check requirements.
    if(msg>max_value) msg = max_value;
    if(msg<min_value) msg = min_value;
    //Shift value.
    msg += shift;
    //Convert msg to string.
    std::ostringstream stream;
    stream << msg;
    std::string value_string = stream.str();
    //Create char array.
    std::string sending = symbol + ":" + value_string;
    const char *buffer_out = sending.c_str();
    //Sending to VCU.
    if(requirement && use_vcu_)
       if (sendto(sock_, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_VCU_, slen_) == -1) 
          printError("sending " + symbol);
}

void VCUInterface::printError(std::string error){
    std::cout << "ARC Interface: Error with " + error << std::endl;
}

