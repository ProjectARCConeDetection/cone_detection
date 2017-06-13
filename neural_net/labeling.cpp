#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

#include <ncurses.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

//Compile: g++ labeling.cpp -o labeling `pkg-config --cflags --libs opencv` -lncurses



int main(int argc, char** argv){

int index = 0;
int input;
int safety = 0;
int counter_cones = 0;
int counter_notcones = 0;
std::string path;
bool ok=true;
cv::namedWindow( "Display", cv::WINDOW_AUTOSIZE );

std::ofstream file;
file.open("/home/arcsystem/cones/candidates/temp2/labels.csv");

cv::Mat img;
cv::Mat dst;

initscr();
refresh();

while(ok) {
	std::stringstream ss;
    ss << index;
    path = "/home/arcsystem/cones/candidates/temp2/" + ss.str() + ".jpg";
    cv::Size size(800,1200);
    img = cv::imread(path);
    if(img.data) {

    	resize(img, dst, size);
    	cv::imshow( "Display", dst);
    	cv::waitKey(5);
    	cbreak();
    	input = getch();
    	nocbreak();
    	if(input==49) {   //ASCII 1 = 49
    		std::string text = ss.str()+",1\n";
    		file<<text;
    		std::cout<<std::endl<<index<<":  Cone"<<std::endl;
    		safety=0;
    		counter_cones++;
    	}
    	else if(input==48) {
    		std::string text = ss.str()+",0\n";
    		file<<text;
    		std::cout<<std::endl<<index<<":  Not a Cone"<<std::endl;
    		safety=0;
    		counter_notcones++;
    	}
    	else if(input==53) {
    		ok=false;
     		std::cout<<std::endl<<"Break"<<std::endl;
    	}

    }

    else {
    	safety++;
    	if(safety>=5000) {
    		break;
    	}
    }


    index++;

	
}

endwin();
std::cout<<std::endl<<"Numbers Cones: "<<counter_cones<<std::endl<<"Numbers Not a Cone: "<<counter_notcones;
file.close();



	return 0;
}

