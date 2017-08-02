#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_aruco/Markers.h" 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

class ArKalman{
  ros::NodeHandle n;
  ros::Subscriber aruco_sub;
  ros::Publisher kalman_pub;
  cv::KalmanFilter ArKF;
  
public:
  ArKalam(){

}











};
