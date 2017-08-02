#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_aruco/Markers.h" 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
class ArKalman
{
  ros::NodeHandle n;
  ros::Subscriber aruco_sub;
  ros::Publisher kalman_pub;
  cv::KalmanFilter ArKF;
   
public:
  ArKalman()
  {
    aruco_sub = n.subscribe("/ros_aruco/markers", 1000, &ArKalman::arucoCallback, this);
    //this argument is to designate this objects member callback function as the target callback function.
    //in other cases you may define to objects, and want to make the 1st object use the callback function of the second object. 
    //this will hapend in main Listener listener_1 listener_2
    //ros::Subscriber sub = n.subscribe("chatter", 1000, &Listener::callback, &listener_2);
    
    kalman_pub = n.advertise<ros_aruco::Markers>(ros::this_node::getName()+ "/markers",1);
    int nStates = 18;            // the number of states
    int nMeasurements = 6;       // the number of measured states
    int nInputs = 0;             // the number of action control
    double dt = 1/29;           // time between measurements (1/FPS)

    initKalmanFilter(ArKF, nStates, nMeasurements, nInputs, dt);    // init function
  }
  
  ~ArKalman() // may be this is needed for the dynamic allocation to deallocate the space. Search more.
  {
  }

  void arucoCallback(const ros_aruco::MarkersConstPtr& msg)
  {
    ros_aruco::MarkersPtr msg_pub(new ros_aruco::Markers);
    //to be dynamically allocated, because we don't know the size of it in the compile time.
    msg_pub->header = msg->header;
    msg_pub->count = msg->count;
    msg_pub->ids.resize(msg->count);
    msg_pub->T.resize(3*msg->count);
    msg_pub->R.resize(3*msg->count);
    msg_pub->px.resize(2*msg->count);

    if(msg->count == 1)
     //because if it equals zero, we will access somewhere that exceeds the array dimention.
     //this proggy is made to use only one marker
    {
      // Get the measured translation
      cv::Mat translation_measured(3, 1, CV_64F);
      translation_measured.at<double>(0) = msg->T[0];
      translation_measured.at<double>(1) = msg->T[1];
      translation_measured.at<double>(2) = msg->T[2];

      // Get the measured rotation, it is in rodrigues forumula
      cv::Mat rot_rodrigues_measured(3, 1, CV_64F);
      rot_rodrigues_measured.at<double>(0) = msg->R[0];
      rot_rodrigues_measured.at<double>(1) = msg->R[1];
      rot_rodrigues_measured.at<double>(2) = msg->R[2];

      cv::Mat rotation_measured(3, 3, CV_64F);
      
      //get the rotation matrix representation
      cv::Rodrigues(rot_rodrigues_measured, rotation_measured);
      
      // fill the measurements vector
      cv::Mat measurements(6, 1, CV_64F);
      //what happens here
      fillMeasurements(measurements, translation_measured, rotation_measured);

      // Instantiate estimated translation and rotation
      cv::Mat translation_estimated(3, 1, CV_64F);
      cv::Mat rotation_estimated(3, 3, CV_64F);
      // update the Kalman filter with good measurements
      updateKalmanFilter(ArKF, measurements,
                    translation_estimated, rotation_estimated);

      msg_pub->T[0] = translation_estimated.at<double>(0);
      msg_pub->T[1] = translation_estimated.at<double>(1);
      msg_pub->T[2] = translation_estimated.at<double>(2);

      cv::Mat rot_rodrigues_estimated(3, 1, CV_64F);
      cv::Rodrigues(rotation_estimated, rot_rodrigues_estimated);      

      msg_pub->R[0] = rot_rodrigues_estimated.at<double>(0);
      msg_pub->R[1] = rot_rodrigues_estimated.at<double>(1);
      msg_pub->R[2] = rot_rodrigues_estimated.at<double>(2);

      msg_pub->ids[0] = msg->ids[0];
      
    }

    kalman_pub.publish(msg_pub);
  }

  void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
  {
    KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
    //set ideneity sets a scaled identity matrix, the scalar is the 2nd field
    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-4));   // set measurement noise
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance
                   /* DYNAMIC MODEL */
    //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]
    // position
    KF.transitionMatrix.at<double>(0,3) = dt;
    KF.transitionMatrix.at<double>(1,4) = dt;
    KF.transitionMatrix.at<double>(2,5) = dt;
    KF.transitionMatrix.at<double>(3,6) = dt;
    KF.transitionMatrix.at<double>(4,7) = dt;
    KF.transitionMatrix.at<double>(5,8) = dt;
    KF.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);
    // orientation
    KF.transitionMatrix.at<double>(9,12) = dt;
    KF.transitionMatrix.at<double>(10,13) = dt;
    KF.transitionMatrix.at<double>(11,14) = dt;
    KF.transitionMatrix.at<double>(12,15) = dt;
    KF.transitionMatrix.at<double>(13,16) = dt;
    KF.transitionMatrix.at<double>(14,17) = dt;cv::Scalar::all
    KF.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);
         /* MEASUREMENT MODEL */
    //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
    KF.measurementMatrix.at<double>(0,0) = 1;  // x
    KF.measurementMatrix.at<double>(1,1) = 1;  // y
    KF.measurementMatrix.at<double>(2,2) = 1;  // z
    KF.measurementMatrix.at<double>(3,9) = 1;  // roll
    KF.measurementMatrix.at<double>(4,10) = 1; // pitch
    KF.measurementMatrix.at<double>(5,11) = 1; // yaw
  }  

  void fillMeasurements( cv::Mat &measurements,
                     const cv::Mat &translation_measured, const cv::Mat &rotation_measuredcv::Scalar::all)
  {   // Here happens the 2nd conversion (from rot_matrix to euler angle)
      // May be because we need 3 angles ??
      // Convert rotation matrix to euler angles
      cv::Mat measured_eulers(3, 1, CV_64F);
      measured_eulers = rot2euler(rotation_measured);
      // Set measurement to predict
      measurements.at<double>(0) = translation_measured.at<double>(0); // x
      measurements.at<double>(1) = translation_measured.at<double>(1); // y
      measurements.at<double>(2) = translation_measured.at<double>(2); // z
      // Those are the three angles
      measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
      measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
      measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw
  }

  void updateKalmanFilter( cv::KalmanFilter &KF, cv::Mat &measurement,
                       cv::Mat &translation_estimated, cv::Mat &rotation_estimated )
  {
      // First predict, to update the internal statePre variable
      // It takes no arguments as the control input is zero
      cv::Mat prediction = KF.predict();
    
      // The "correct" phase that is going to use the predicted value and our measurement
      cv::Mat estimated = KF.correct(measurement);
      // Estimated translation
      translation_estimated.at<double>(0) = estimated.at<double>(0);
      translation_estimated.at<double>(1) = estimated.at<double>(1);
      translation_estimated.at<double>(2) = estimated.at<double>(2);
      // Estimated euler angles
      cv::Mat eulers_estimated(3, 1, CV_64F);
      eulers_estimated.at<double>(0) = estimated.at<double>(9);
      eulers_estimated.at<double>(1) = estimated.at<double>(10);
      eulers_estimated.at<double>(2) = estimated.at<double>(11);
      // Convert estimated quaternion to rotation matrix
      rotation_estimated = euler2rot(eulers_estimated);
  }

  cv::Mat rot2euler(const cv::Mat & rotationMatrix)
  {
    cv::Mat euler(3,1,CV_64F);
  
    double m00 = rotationMatrix.at<double>(0,0);
    double m02 = rotationMatrix.at<double>(0,2);
    double m10 = rotationMatrix.at<double>(1,0);
    double m11 = rotationMatrix.at<double>(1,1);
    double m12 = rotationMatrix.at<double>(1,2);
    double m20 = rotationMatrix.at<double>(2,0);
    double m22 = rotationMatrix.at<double>(2,2);
  cv::Scalar::all
    double x, y, z;
  
    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
      x = 0;
      y = CV_PI/2;
      z = atan2(m02,m22);
    }
    else if (m10 < -0.998) { // singularity at south pole
      x = 0;
      y = -CV_PI/2;
      z = atan2(m02,m22);
    }
    else
    {
      x = atan2(-m12,m11);
      y = asin(m10);
      z = atan2(-m20,m00);
    }
  
    euler.at<double>(0) = x;
    euler.at<double>(1) = y;
    euler.at<double>(2) = z;
  
    return euler;
  }

  cv::Mat euler2rot(const cv::Mat & euler)
  {
    cv::Mat rotationMatrix(3,3,CV_64F);
  
    double x = euler.at<double>(0);
    double y = euler.at<double>(1);
    double z = euler.at<double>(2);
  
    // Assuming the angles are in radians.
    double ch = cos(z);
    double sh = sin(z);
    double ca = cos(y);
    double sa = sin(y);
    double cb = cos(x);
    double sb = sin(x);
  
    double m00, m01, m02, m10, m11, m12, m20, m21, m22;
  
    m00 = ch * ca;
    m01 = sh*sb - ch*sa*cb;
    m02 = ch*sa*sb + sh*cb;
    m10 = sa;
    m11 = ca*cb;
    m12 = -ca*sb;
    m20 = -sh*ca;
    m21 = sh*sa*cb + ch*sb;
    m22 = -sh*sa*sb + ch*cb;
  
    rotationMatrix.at<double>(0,0) = m00;
    rotationMatrix.at<double>(0,1) = m01;
    rotationMatrix.at<double>(0,2) = m02;
    rotationMatrix.at<double>(1,0) = m10;
    rotationMatrix.at<double>(1,1) = m11;
    rotationMatrix.at<double>(1,2) = m12;
    rotationMatrix.at<double>(2,0) = m20;
    rotationMatrix.at<double>(2,1) = m21;
    rotationMatrix.at<double>(2,2) = m22;
  
    return rotationMatrix;
  }

};

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "ar_kalman");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  ArKalman arkalman;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */



  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
