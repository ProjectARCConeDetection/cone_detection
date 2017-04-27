#include <planning/tools.hpp>

namespace path{

double distanceBetween(int base_index,int target_index,std::vector<Eigen::Vector3d> positions){
    double distance = 0.0;
    for (int i=base_index; i<target_index; ++i){
      if(i>(int)positions.size()-1 || i<0) break;
      distance += (positions[i] - positions[i+1]).norm();
    }
    return distance;
}

double distanceToIndex(int target_index,std::vector<Eigen::Vector3d> positions){
    double distance = 0.0;
    for (int i=0; i<target_index; ++i){
      if(i>(int)positions.size()-1 || i<0) break;
      distance += (positions[i] - positions[i+1]).norm();
    }
    return distance;
}

int indexOfDistanceFront(double max_distance,std::vector<Eigen::Vector3d> positions){
    double distance = 0.0;
    int index = 0;
    while((distance < max_distance) && (index < positions.size()-2)){
      distance += distanceBetween(index+1,index,positions);
      index++;
    }
    return index+1;
}

}//namespace path.