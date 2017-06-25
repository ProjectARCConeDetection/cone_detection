# cone_detection

**Authors:** [Nico Messikommer](), [Simon Schaefer]()

**Current version:** 1.0.0 

Cone detection algorithm for an autonomous car using a LiDAR sensor and a colour camera. By evaluating simple constraints, the LiDAR detection algorithm preselects cone candidates in the 3 dimensional space. The candidates are projected into the image plane of the colour camera and an image candidate is cropped out. A convolutional neural networks classifies the image candidates as cone or not a cone. With the fusion of the precise position estimation of the LiDAR sensor and the high classification accuracy of a neural network, a reliable cone detection algorithm was implemented. Furthermore, a path planning algorithm generates a path around the detected cones. The final system detects cones even at higher velocity and has the potential to drive fully autonomous around the cones.

## Required Sotfware
- OpenCV (http://opencv.org)
- Tensorflow (https://github.com/tensorflow/tensorflow)
- ROS (http://wiki.ros.org)

### Usage 
1. Adapt parameter in cfg/parameter.yaml
2. roslaunch cone_detection cone_detection.launch
