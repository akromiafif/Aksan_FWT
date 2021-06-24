#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <image_transport/image_transport.h> 
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h> 
#include <mavros_msgs/Waypoint.h> 
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/VFR_HUD.h> 
#include <sensor_msgs/NavSatFix.h> 
#include <std_msgs/Float64.h>
#include <math.h>

using namespace cv;
using namespace ros;
using namespace std;
using namespace mavros_msgs;
using namespace sensor_msgs;

Waypoint wp_target; //Drop-zone waypoint
Waypoint wp_hw; //Headwind determination waypoint

float BH = 0; //Drop-zone Bearing
float H_cur = 0; //Heading
float Alt_cur = 0; //Altitude
float x = 0; //Pixel x coordinate
float y = 0; //Pixel y coordinate
float xm = 0; //x axis pixel displacement
float ym = 0; //y axis pixel displacement
float lat_ref = 0; //Reference latitude
float long_ref = 0; //Reference longitude
float d = 0; //Ground plane displacement
float R = 0; //Earth's radius
float B = 0; //Fixed bearing
float pi = 3.14159265359;

//Current flight controller variables (airspeed, heading, etc)
VFR_HUD HV;
void vfr_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg) {
  HV = *msg;
}

//Obtain current GPS location
NavSatFix GPS;
void nsf_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  GPS = *msg;
}

//Obtain relative altitude
std_msgs::Float64 Alt;
void alt_cb(const std_msgs::Float64::ConstPtr& msg) {
  Alt = *msg;
}

//Image callback - Begin object recognition and tracking
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    NodeHandle n;

    Mat BGR = cv_bridge::toCvShare(msg, "bgr8")->image; //Convert image from msg to BGR format
    Mat orig_image = BGR.clone(); //Make copy of image for processing
    
    medianBlur(BGR, BGR, 3); //Introduce blur for image flitering and converting
    
    // Convert input image to HSV
    Mat HSV;
    cvtColor(BGR, HSV, COLOR_BGR2HSV); //Convert BGR image to HSV image
    
    // Threshold the HSV image, keep only the red pixels
    Mat lower_hue;
    Mat upper_hue;
    inRange(HSV, Scalar(0, 100, 100), Scalar(10, 255, 255), lower_hue); //Lower hue threshold range
    inRange(HSV, Scalar(165, 100, 100), Scalar(179, 255, 255), upper_hue); //Upper hue threshold range
    
    // Combine the above two images
    Mat hue_image;
    addWeighted(lower_hue, 1.0, upper_hue, 1.0, 0.0, hue_image); //Combine hue images for object recognition
    
    // Introduce interference
    GaussianBlur(hue_image, hue_image, Size(9, 9), 2, 2); //Introduce noise
    
    // Hough transform to detect circles
    vector<Vec3f> circles; //Detected circles array
    HoughCircles(hue_image, circles, CV_HOUGH_GRADIENT, 1, hue_image.rows/8, 100, 25, 0, 0); //HOUGH CIRCLE TRANSFORNATION
    imshow("Original", orig_image);

    // Highlight detected object
    for(size_t cur = 0; cur < circles.size(); ++cur) {
      Point center(circles[cur][0], circles[cur][1]); //Define centre point of detected circle
      int radius = circles[cur][2]; //Define radius of detected circle
      
      circle(orig_image, center, radius, Scalar(239, 152, 38), 2); //Overlay detected cricle outline onto origional image

      imshow("Vision Output", orig_image); //Display circle image overlay
      ROS_INFO("Size: (W) %i x (H) %i", orig_image.cols, orig_image.rows); //Define image size
      waitKey(30); //allow for display of image for given milliseconds (Image overlay refreshrate) 
    }

  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_listener"); //Define node name
  ros::NodeHandle nh;

  cv::namedWindow("Vision Output", WINDOW_AUTOSIZE); //Create viewable window - Image overlay output
  cv::startWindowThread(); //Begin window view and image display
  image_transport::ImageTransport it(nh); //Define source of image
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback); //Image subscriber
  
  //image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::spin();
}