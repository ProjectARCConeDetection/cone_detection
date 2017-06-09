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

