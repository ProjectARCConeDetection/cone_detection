#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv){
    // Read 3D points
    std::vector<cv::Point3d> objectPoints;
    cv::Point3d pointing(5,5,1);
    objectPoints.push_back(pointing);
    std::vector<cv::Point2d> imagePoints;

    cv::Mat intrisicMat(3, 3, cv::DataType<double>::type); // Intrisic matrix
    intrisicMat.at<double>(0, 0) = 619.197508;
    intrisicMat.at<double>(1, 0) = 0;
    intrisicMat.at<double>(2, 0) = 313.097177;

    intrisicMat.at<double>(0, 1) = 0;
    intrisicMat.at<double>(1, 1) = 620.856958;
    intrisicMat.at<double>(2, 1) = 230.861780;

    intrisicMat.at<double>(0, 2) = 0;
    intrisicMat.at<double>(1, 2) = 0;
    intrisicMat.at<double>(2, 2) = 1;

    cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
    rVec.at<double>(0) = 0;
    rVec.at<double>(1) = 0;
    rVec.at<double>(2) = 0;

    cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
    tVec.at<double>(0) = 0;
    tVec.at<double>(1) = 0;
    tVec.at<double>(2) = 0;

    cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);   // Distortion vector
    distCoeffs.at<double>(0) = 0.190819;
    distCoeffs.at<double>(1) = -0.440311;
    distCoeffs.at<double>(2) = -0.003138;
    distCoeffs.at<double>(3) = -0.008376;
    distCoeffs.at<double>(4) = 0;

    cv::projectPoints(objectPoints, rVec, tVec, intrisicMat, distCoeffs, imagePoints);

    std::cout << "Input: " << objectPoints << std::endl;
    std::cout << "Output: " << imagePoints << std::endl;

 	//Rect rect = boundingRect(v[idx]);
	// Point pt1, pt2;
	// pt1.x = rect.x;
	// pt1.y = rect.y;
	// pt2.x = rect.x + rect.width;
	// pt2.y = rect.y + rect.height;
	// // Draws the rect in the original image and show it
	// rectangle(originalimage, pt1, pt2, CV_RGB(255,0,0), 1);
	//http://stackoverflow.com/questions/3669611/bounding-box-using-opencv.

	return 0;
}