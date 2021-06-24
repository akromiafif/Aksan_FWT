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
      
    public:
      UAVPercept(ros::NodeHandle node);
      ~UAVPercept();

      void improCB(const sensor_msgs::ImageConstPtr& msg);
  };
}