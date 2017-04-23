#include <cone_detection/tools.hpp>

void Candidate::print(){
    std::cout << "---------------------" << std::endl;
    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
    std::cout << "index: " << index << std::endl;
}

Eigen::Vector3d Pose::euler(){
    Eigen::Vector3d euler;
    Eigen::Vector4d quat = orientation;
    double ysqr = quat(1) * quat(1);
    // roll (x-axis rotation)
    double t0 = +2.0 * (quat(3) * quat(0) + quat(1) * quat(2));
    double t1 = +1.0 - 2.0 * (quat(0) * quat(0) + ysqr);
    euler(0) = std::atan2(t0, t1);
    // pitch (y-axis rotation)
    double t2 = +2.0 * (quat(3) * quat(1) - quat(2) * quat(0));
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    euler(1) = std::asin(t2);
    // yaw (z-axis rotation)
    double t3 = +2.0 * (quat(3) * quat(2) + quat(0) * quat(1));
    double t4 = +1.0 - 2.0 * (ysqr + quat(2) * quat(2));  
    euler(2) = std::atan2(t3, t4);
    return euler;
}

Eigen::Matrix3d Pose::getRotationMatrix(){
    Eigen::Vector3d angles = euler();   
    double roll = angles(0);
    double pitch = angles(1);
    double yaw = angles(2);
    //Rotation matrix.
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix<<   cos(yaw)*cos(pitch),    cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll),   cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll) ,
            sin(yaw)*cos(pitch),    sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll),   sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll),
            -sin(pitch),        cos(pitch)*sin(roll),                       cos(pitch)*cos(roll);
    return rotation_matrix;
}

Eigen::Vector3d Pose::localToGlobal(const Eigen::Vector3d local){
    Eigen::Matrix3d R = getRotationMatrix();
    Eigen::Vector3d global = R*local;
    return global;
}

void Pose::print(){
    std::cout << "---------------------------" << std::endl;
    std::cout << "Position: " << position(0) << ", " << position(1) << ", " 
              << position(2) << std::endl; 
    std::cout << "Orientation: " << orientation(0) << ", " << orientation(1) << ", " 
              << orientation(2) << ", " << orientation(3) << std::endl; 
}

namespace quat{

Eigen::Vector4d multQuaternion(Eigen::Vector4d q1,Eigen::Vector4d q2){
    Eigen::Vector3d axes1(q1(0), q1(1), q1(2));
    Eigen::Vector3d axes2(q2(0), q2(1), q2(2));
    //Hamiltonian product to get new quaternion.
    Eigen::Vector4d new_quat;
    new_quat(3) = q1(3)*q2(3) + axes1.dot(axes2);
    Eigen::Vector3d new_axes = q1(3)*axes2 + q2(3)*axes1 + axes1.cross(axes2);
    new_quat(0) = new_axes(0);
    new_quat(1) = new_axes(1);
    new_quat(2) = new_axes(2);
    new_quat = new_quat/new_quat.norm();
    return new_quat;
}

Eigen::Vector4d inverseQuaternion(Eigen::Vector4d quat){
    Eigen::Vector4d inverse_quat;
    inverse_quat(0) = -quat(0);
    inverse_quat(1) = -quat(1);
    inverse_quat(2) = -quat(2);
    inverse_quat(3) = quat(3);
    return inverse_quat;
}

Eigen::Vector4d diffQuaternion(Eigen::Vector4d base_quat, Eigen::Vector4d target_quat){
    //Normalization.
    base_quat = base_quat/base_quat.norm();
    target_quat = target_quat/target_quat.norm();
    //Getting difference of unit quaternion.
    Eigen::Vector4d diff_quat = multQuaternion(base_quat, inverseQuaternion(target_quat));
    diff_quat.normalize();
    return diff_quat;
}
}//namespace quat.

namespace vector{

int getSameIndex(std::vector <Candidate> xyz_index_vector,int index){
    for(int i=0;i<xyz_index_vector.size();++i){
        if(index == xyz_index_vector[i].index) return i;
    }
    return -1;
}
}//namespace vector.