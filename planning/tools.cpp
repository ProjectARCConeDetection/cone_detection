#include <planning/tools.hpp>

void AckermannControl::print(){
    std::cout << "------------Stellgroessen -----------------" << std::endl;
    std::cout << "steering: " << steering_angle << " velocity " << velocity << std::endl;
 }


namespace path{

double distanceBetween(int base_index,int target_index,std::vector<Eigen::Vector2d> positions){
    double distance = 0.0;
    for (int i=base_index; i<target_index; ++i){
      if(i>(int)positions.size()-1 || i<0) break;
      distance += (positions[i] - positions[i+1]).norm();
    }
    return distance;
}

double distanceToIndex(int target_index,std::vector<Eigen::Vector2d> positions){
    double distance = 0.0;
    for (int i=0; i<target_index; ++i){
      if(i>(int)positions.size()-1 || i<0) break;
      distance += (positions[i] - positions[i+1]).norm();
    }
    return distance;
}

int indexOfDistanceFront(double max_distance,std::vector<Eigen::Vector2d> positions){
    double distance = 0.0;
    int index = 0;
    while((distance < max_distance) && (index < positions.size()-2)){
      distance += distanceBetween(index+1,index,positions);
      index++;
    }
    return index+1;
}
}//namespace path.

VCUInterface::VCUInterface(){}

VCUInterface::~VCUInterface(){
    //Back to manuell mode.
    send_msg("am", 0.0, true);
}

void VCUInterface::init(bool use_vcu){
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

double VCUInterface::recv_velocity(){
    if(!use_vcu_) return -10.0;
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
    double velocity;
    if(kind == "rr") velocity = value;
    else if(kind == "rl") velocity = value;
    else velocity = -10.0;
    return velocity;
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
