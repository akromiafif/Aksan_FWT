#include <ros/ros.h>

// For OpenCV purpose
#include <image_transport/image_transport.h> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// For Telemetry data
#include <mavros_msgs/Waypoint.h> 
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/VFR_HUD.h> 
#include <sensor_msgs/NavSatFix.h> 
#include <std_msgs/Float64.h>
#include <math.h>
#include <std_msgs/String.h> 

//Custom message
#include <uav_commander/lap_info.h>
#include <uav_commander/impro_info.h>

using namespace cv;
using namespace ros;
using namespace std;
using namespace mavros_msgs;
using namespace sensor_msgs;

namespace uav_percept {
  // OpenCV Window Name for imshow
  static const std::string OPENCV_WINDOW = "Vision Detected";

  // Topics
  static const std::string IMAGE_TOPIC = "/camera/image";


  class UAVPercept {
    public: 
      image_transport::Subscriber itSubscriber;
      ros::Subscriber vfrHUDSubscriber;
      ros::Subscriber nsfSubscriber;
      ros::Subscriber altSubscriber;
      ros::Subscriber lapInfoSubscriber;
      ros::Subscriber improInfoSubscriber;
      
      // Save Lat Long variable
      std::ofstream latLongAircraft;

      // Save current aircraft lat, long coordinate
      std::ofstream latLongDroppingZone;

      std::ofstream testWrite;

      // Save video variable
      cv::VideoWriter writer;

      // lat long counter
      int counter;

      //Variables
      float bearingDropZone = 0; //Drop-zone Bearing
      float headingCurrent = 0; //Heading
      float altCurrent = 0; //Altitude
      float x = 0; //Pixel x coordinate
      float y = 0; //Pixel y coordinate
      float xm = 0; //x axis pixel displacement
      float ym = 0; //y axis pixel displacement
      float latRef = 0; //Reference latitude
      float longRef = 0; //Reference longitude
      float d = 0; //Ground plane displacement
      float R = 0; //Earth's radius
      float bearing = 0; //Fixed bearing
      float pi = 3.14159265359;

      //Variables from Callback
      mavros_msgs::VFR_HUD vfrHUD;
      sensor_msgs::NavSatFix GPS;
      std_msgs::Float64 Alt;
      uav_commander::lap_info lapInfo;
      uav_commander::impro_info improInfo;
      
    public:
      UAVPercept(ros::NodeHandle* node);
      ~UAVPercept();

      void improCB(const sensor_msgs::ImageConstPtr& msg);
      void vfrCB(const mavros_msgs::VFR_HUD::ConstPtr& msg);
      void nsfCB(const sensor_msgs::NavSatFix::ConstPtr& msg);
      void altCB(const std_msgs::Float64::ConstPtr& msg);
      void lapInfoCB(const uav_commander::lap_info::ConstPtr& msg);
      void improInfoCB(const uav_commander::impro_info::ConstPtr& msg);
  };
}