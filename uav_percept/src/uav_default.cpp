//Vision-Based Autonomous Aircraft Payload Delivery System
//James Sewell
//MEng (Mechatronics)
//Nelson Mandela University
/////////////////////////////////////////////////
//////////////IMAGE PROCESSING NODE//////////////
/////////////////////////////////////////////////


#include <ros/ros.h> //ROS Header
#include <std_msgs/String.h> //String Header
#include <image_transport/image_transport.h> //Image Transporter Header
#include <opencv2/highgui/highgui.hpp> //OPENCV Header
#include <cv_bridge/cv_bridge.h> //OPENCV Bridge Header
#include <mavros_msgs/Waypoint.h> //Waypoint Header
#include <mavros_msgs/WaypointPush.h> //Upload waypoints to FCU
#include <mavros_msgs/VFR_HUD.h> //Telemetry Header
#include <sensor_msgs/NavSatFix.h> //GPS Header
#include <std_msgs/Float64.h> //Altitude header (realtive to takeoff)
#include <math.h> //Maths header


//Namespaces - for neater code
using namespace cv;
using namespace ros;
using namespace std;
using namespace mavros_msgs;
using namespace sensor_msgs;

Waypoint wp_target; //Drop-zone waypoint
Waypoint wp_hw; //Headwind determination waypoint

//Variables
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

    //VFR_HUD Subscriber
    Subscriber vfr_sub = n.subscribe<mavros_msgs::VFR_HUD>("mavros/vfr_hud", 10, vfr_cb);
    
    //GPS info Subscriber
    Subscriber nsf_sub = n.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, nsf_cb); 

    //Altitude subscriber
    Subscriber alt_sub = n.subscribe<std_msgs::Float64>("mavros/global_position/rel_alt", 10, alt_cb); 

    //Waypoint upload header
    ServiceClient waypush_client = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push"); 
    lat_ref = GPS.latitude * (pi/180); //Reference latitude (current latitude of aircraft)
    long_ref = GPS.longitude * (pi/180); //Reference longitude (current longitude of aircraft)
    H_cur = HV.heading; //Current heading
    Alt_cur = Alt.data; //Current altitude
    Rate rate(30.0); //Transmission rate of imagery

    Mat BGR = cv_bridge::toCvShare(msg, "bgr8")->image; //Convert image from msg to BGR format
    Mat orig_image = BGR.clone(); //Make copy of image for processing
    medianBlur(BGR, BGR, 7); //Introduce blur for image flitering and converting

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
    HoughCircles(hue_image, circles, CV_HOUGH_GRADIENT, 1, hue_image.rows/8,100, 25, 0, 0); //HOUGH CIRCLE TRANSFORNATION
    
    // Highlight detected object
    for(size_t cur = 0; cur < circles.size(); ++cur) {
      Point center(circles[cur][0], circles[cur][1]); //Define centre point of detected circle
      int radius = circles[cur][2]; //Define radius of detected circle
      circle(orig_image, center, radius, Scalar(239, 152, 38), 2); //Overlay detected cricle outline onto origional image
      ROS_INFO("Image Centre: %f , %f", circles[cur][0], circles[cur][1]); //Display circle centre coordinates
    }

    imshow("Lower hue", lower_hue); //Display lower hue image
    imshow("Upper hue", upper_hue); //Display upper hue image
    imshow("Vision Output", orig_image); //Display circle image overlay
    ROS_INFO("Size: (W) %i x (H) %i", orig_image.cols, orig_image.rows); //Define image size
    waitKey(10); //allow for display of image for given milliseconds (Image overlay refreshrate)

    //Target Locating
    //Check if any circles were detected
    if(circles.size() != 0) {
      WaypointPush waypush;
      //Target bearing
      BH = 0;
      x = circles[0][0] - (orig_image.cols / 2); //x pixel coordinate
      y = circles[0][1] - (orig_image.rows / 2); //y pixel coordinate
      xm = (abs(x)/(orig_image.cols/2)) * (Alt_cur*tan(pi/6)); //x displacement in meters
      ym = (abs(y)/(orig_image.rows/2)) * (Alt_cur*tan(pi/8)); //y displacement in meters

      /* Region designation - (0,0) in top-left corner of 2
      2|1
      ---
      3|4
      */
      
      //Determine bearing of target based on image segment

      //region 2 and 3
      if(circles[0][0] >= 0 && circles[0][0] <= (orig_image.cols / 2))  {
        if(circles[0][1] >= 0 && circles[0][1] <= (orig_image.rows /2)) { //region 2
          BH = H_cur + atan(abs(x)/abs(y));
        } else { //region 3
          BH = H_cur + (180 - atan(abs(x)/abs(y)));
        }
      } else { //region 1 and 4
        if(circles[0][1] > (orig_image.rows / 2)) { //region 4
          BH = H_cur + 180 + atan(abs(x)/abs(y));
        } else { //region 1
          BH = H_cur - atan(abs(x)/abs(y));
        }
      }

      //Bearing regression (0 - 359)
      if(BH >= 360) {
        BH = BH - 360;
      } else if(BH < 0) {
        BH = BH + 360;
      }

      GPS.position_covariance_type = 3; //Type of GPS frame reference (3 - Covariance known)
      d = sqrt(pow(abs(xm), 2) + pow(abs(ym), 2)) / 1000; //magnitude of vector to target location in km
      R = 6378.1; //Radius of the Earth (km)
      B = BH * pi/180; //Bearing upward (0 degrees -> radians)
      wp_target.frame = 3; // mavros_msgs::Waypoint::FRAME_GLOBAL;
      wp_target.command = 16; //MAV_CMD Waypoint type
      wp_target.is_current = false;
      wp_target.autocontinue = true;
      wp_target.param1 = 0;
      wp_target.param2 = 2;
      wp_target.param3 = 0;
      wp_target.param4 = 0;
      wp_target.z_alt = 50.0;

      //Deduce target's location based on current location
      wp_target.x_lat = (asin(sin(lat_ref)*cos(d/R) + cos(lat_ref)*sin(d/R)*cos(B))) * (180/pi); //Target Latitude
      wp_target.y_long = (long_ref + atan2(sin(B)*sin(d/R)*cos(lat_ref), cos(d/R) - sin(lat_ref)*sin(wp_target.x_lat))) * (180/pi); //Target Longitude
      
      //Display target details deduced
      ROS_INFO("Lat: %f", wp_target.x_lat); //Target latitude
      ROS_INFO("Long: %f", wp_target.y_long); //Target Longitude
      ROS_INFO("LatRef: %f", lat_ref); //Reference latitude
      ROS_INFO("LongRef: %f", long_ref); //Reference longitude
      ROS_INFO("xm: %f", xm); //X axis displacement (km)
      ROS_INFO("ym: %f", ym); //Y axis displacment (km)
      ROS_INFO("displacement: %f", d); //Total displacment (km)
      ROS_INFO("bearing: %f", BH); //Bearing
      ROS_INFO("altitude: %f", Alt_cur);
      
      //Update target waypoint parameters
      wp_hw.frame = 3; // mavros_msgs::Waypoint::FRAME_GLOBAL;
      wp_hw.command = 18;
      wp_hw.is_current = false;
      wp_hw.autocontinue = true;
      wp_hw.param1 = 3;
      wp_hw.param2 = 0;
      wp_hw.param3 = 30;
      wp_hw.param4 = 1;
      wp_hw.z_alt = 50.0;
      wp_hw.x_lat = wp_target.x_lat;
      wp_hw.y_long = wp_target.y_long;

      //Transmit updated target location parameters to FCU
      bool targetSent = false;
      waypush.request.start_index = 15; //Update waypoint 15
      ros::Time last_request = ros::Time::now();
      while (targetSent != true) {
        if (targetSent != true && (ros::Time::now() - last_request > ros::Duration(0.2))) {
          waypush.request.waypoints.push_back(wp_target); //send target waypoint to the FCU
          if (waypush_client.call(waypush)) {
            targetSent = true;
          }

          last_request = ros::Time::now();
        }

        ros::spinOnce();
        rate.sleep();
      }

      //Transmit updated headwind determination location parameters to FCU
      targetSent = false;
      waypush.request.start_index = 11; //Update waypoint 11
      last_request = ros::Time::now();

      while (targetSent != true) {
        if (targetSent != true && (ros::Time::now() - last_request >ros::Duration(0.2))) {
          waypush.request.waypoints.push_back(wp_hw); //send target waypoint to the FCU
          if (waypush_client.call(waypush)) {
            targetSent = true;
          }

          last_request = ros::Time::now();
        }

        ros::spinOnce();
        rate.sleep();
      }
    }
  }

  //End "if circles detected" loop - End target location identification
  //Throw error if image publisher fails
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


//Initial loop
int main(int argc, char **argv)  {
  init(argc, argv, "image_listener"); //Define node name
  NodeHandle nh;
  namedWindow("Vision Output", WINDOW_AUTOSIZE); //Create viewable window - Image overlay output
  namedWindow("Lower hue", WINDOW_AUTOSIZE); //Create viewable window - Lower hue range
  namedWindow("Upper hue", WINDOW_AUTOSIZE); //Create viewable window - Upper hue range
  startWindowThread(); //Begin window view and image display
  image_transport::ImageTransport it(nh); //Define source of image
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback); //Image subscriber
  spin();
}