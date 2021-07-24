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

namespace uav_percept {
  class UAVPercept {
    public: 
      image_transport::Subscriber itSubscriber;
      Subscriber vfrHUDSubscriber;
      Subscriber nsfSubscriber;
      Subscriber altSubscriber;

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
      
    public:
      UAVPercept(ros::NodeHandle node);
      ~UAVPercept();

      void improCB(const sensor_msgs::ImageConstPtr& msg);
      void vfrCB(const mavros_msgs::VFR_HUD::ConstPtr& msg);
      void nsfCB(const sensor_msgs::NavSatFix::ConstPtr& msg);
      void altCB(const std_msgs::Float64::ConstPtr& msg);
  };
}