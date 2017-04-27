#include <cone_detection/tools.hpp>

void Candidate::print(){
    std::cout << "---------------------" << std::endl;
    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
    std::cout << "index: " << index << std::endl;
}

void Cam::initCamMatrices(){
    // Definition.
    intrisicMat = cv::Mat(3,3,cv::DataType<double>::type);
    rVec = cv::Mat(3,1,cv::DataType<double>::type);
    tVec = cv::Mat(3,1,cv::DataType<double>::type);
    distCoeffs = cv::Mat(5,1,cv::DataType<double>::type);
    // Intrisic matrix.
    intrisicMat.at<double>(0, 0) = 621.701971;
    intrisicMat.at<double>(0, 1) = 0;
    intrisicMat.at<double>(0, 2) = 325.157695;
    intrisicMat.at<double>(1, 0) = 0;
    intrisicMat.at<double>(1, 1) = 621.971316;
    intrisicMat.at<double>(1, 2) = 233.170431;
    intrisicMat.at<double>(2, 0) = 0;
    intrisicMat.at<double>(2, 1) = 0;
    intrisicMat.at<double>(2, 2) = 1;
    // Rotation vector.
    rVec.at<double>(0) = 0;
    rVec.at<double>(1) = 0;
    rVec.at<double>(2) = 0;
    // Translation vector.
    tVec.at<double>(0) = 0;
    tVec.at<double>(1) = 0;
    tVec.at<double>(2) = 0;
    // Distortion vector.
    distCoeffs.at<double>(0) = 0.079264;
    distCoeffs.at<double>(1) = -0.149967;
    distCoeffs.at<double>(2) = 0.000716;
    distCoeffs.at<double>(3) = 0.001749;
    distCoeffs.at<double>(4) = 0;
    // Image width and height.
    image_height = 480;
    image_width = 640;
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

Eigen::Vector3d Pose::globalToLocal(Eigen::Vector3d global){
  //Translatation
  Eigen::Vector3d temp = global - position;
  //Rotation
  Eigen::Matrix3d R = transforms::getRotationMatrix(euler());
  Eigen::Matrix3d T = R.transpose();
  Eigen::Vector3d local = T*temp;
  //Change of coordinate frame.
  //Eigen::Vector3d local_rotated;
  //local_rotated(0) = -local(1);
  //local_rotated(1) = local(0);
  //local_rotated(2) = local(2);
  //return local_rotated;
  return local;
}

Eigen::Vector3d Pose::localToGlobal(const Eigen::Vector3d local){
    Eigen::Matrix3d R = transforms::getRotationMatrix(euler());
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

namespace transforms{

Eigen::Matrix3d getRotationMatrix(Eigen::Vector3d euler){ 
    double roll = euler(0);
    double pitch = euler(1);
    double yaw = euler(2);
    //Rotation matrix.
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix<<   cos(yaw)*cos(pitch),    cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll),   cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll) ,
            sin(yaw)*cos(pitch),    sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll),   sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll),
            -sin(pitch),        cos(pitch)*sin(roll),                       cos(pitch)*cos(roll);
    return rotation_matrix;
}
}//namespace transforms.

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