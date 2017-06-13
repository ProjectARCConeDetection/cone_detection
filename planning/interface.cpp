#include <planning/tools.hpp>

#include <cone_detection/tools.hpp>

#include <arpa/inet.h>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>

//Publisher.& Subscriber.
ros::Publisher steering_angle_pub;
ros::Publisher velocity_left_pub;
ros::Publisher velocity_right_pub;
ros::Subscriber mode_sub;
ros::Subscriber stellgroessen_sub;
//Parameter.
Control control;
//Network.
struct sockaddr_in si_me_, si_other_, si_VCU_;
int sock_;
socklen_t slen_;
double first_steering_should=0;
double second_steering_should=0;
double third_steering_should=0;
//Decleration of functions.
void modeCallback(const std_msgs::Bool::ConstPtr& msg);
void stellgroessenCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void recv_msgs();
void send_msg(std::string symbol, double msg, bool requirement);
void send_msg(std::string symbol, double msg, bool requirement, 
			  double max_value, double min_value, double shift);
void gettingParameter(ros::NodeHandle* node, Control* control);
void printError(std::string error);

int main(int argc, char** argv){
	ros::init(argc, argv, "vcu_interface");
	ros::NodeHandle node;
	//Getting parameter.
	gettingParameter(&node,&control);
	//Init publisher and subscriber.
	steering_angle_pub = node.advertise<std_msgs::Float64>("/state_steering_angle", 10);
	velocity_left_pub = node.advertise<std_msgs::Float64>("/wheel_rear_left", 10);
	velocity_right_pub = node.advertise<std_msgs::Float64>("/wheel_rear_right", 10);
	mode_sub = node.subscribe("/mode", 1, modeCallback);
	stellgroessen_sub = node.subscribe("/stellgroessen", 1, stellgroessenCallback);
	//Init vcu.
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
  	//Spinning.
  	// ros::Rate rate(10);
  	while(ros::ok()){
  		ros::spinOnce();
  		recv_msgs();
		// rate.sleep();
  	}
  	//Back to manuell mode.
    send_msg("am", 0.0, true);
	return 0;
}

void modeCallback(const std_msgs::Bool::ConstPtr& msg){
	send_msg("cc", 5.0, msg->data);
	ros::Duration(0.5).sleep();
	send_msg("am", 1.0, msg->data);
}

void stellgroessenCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	send_msg("vs",msg->data[1], true, control.max_absolute_velocity, -100, 0);
    third_steering_should = second_steering_should;
    second_steering_should = first_steering_should;
    first_steering_should = msg->data[0];
    double new_steering_should = (first_steering_should+second_steering_should+third_steering_should)/3;
	send_msg("ss",first_steering_should*180/M_PI, true, 
				  	control.max_steering_angle, -control.max_steering_angle, 0);
}

void recv_msgs(){
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
    float value = atof(buffer);
    std_msgs::Float64 value_msg;
    value_msg.data = value;
    //Answers.
    if(kind == "rr"){  
        velocity_right_pub.publish(value_msg);

    }
    else if(kind == "rl") velocity_left_pub.publish(value_msg);
    else if(kind == "si"){
        value = (value-1000)*M_PI/180.0;
        value_msg.data = value;
        steering_angle_pub.publish(value_msg);
    }
}

void send_msg(std::string symbol, double msg, bool requirement){
    //Convert msg to string.
    std::ostringstream stream;
    stream << msg;
    std::string value_string = stream.str();
    //Create char array.
    std::string sending = symbol + ":" + value_string;
    const char *buffer_out = sending.c_str();
    //Sending to VCU.
    if(requirement)
       if (sendto(sock_, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_VCU_, slen_) == -1) 
          printError("sending " + symbol);
}

void send_msg(std::string symbol, double msg, bool requirement, 
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
    if(requirement)
       if (sendto(sock_, buffer_out, sizeof(buffer_out), 0, (struct sockaddr*) &si_VCU_, slen_) == -1) 
          printError("sending " + symbol);
}

void gettingParameter(ros::NodeHandle* node, Control* control){
	//Get control parameter.
	node->getParam("/control/distance_interpolation", control->distance_interpolation);
	node->getParam("/control/K1_LAD_S", control->k1_lad_s);
	node->getParam("/control/K2_LAD_S", control->k2_lad_s);
	node->getParam("/control/K1_LAD_V", control->k1_lad_v);
	node->getParam("/control/K2_LAD_V", control->k2_lad_v);
	node->getParam("/control/upperbound_lad_s", control->upperbound_lad_s);
	node->getParam("/control/lowerbound_lad_s", control->lowerbound_lad_s);
	node->getParam("/control/max_absolute_velocity", control->max_absolute_velocity);
	node->getParam("/control/max_lateral_acceleration", control->max_lateral_acceleration);
	node->getParam("/control/max_steering_angle", control->max_steering_angle);
}

void printError(std::string error){
    std::cout << "ARC Interface: Error with " + error << std::endl;
}



