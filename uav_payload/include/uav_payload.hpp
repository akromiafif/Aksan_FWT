#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdlib.h>     

// For dropping payload
#include <mavros_msgs/CommandLong.h>
#include <sensor_msgs/NavSatFix.h> 

// Lap info and when impro is activated
#include <uav_commander/lap_info.h>
#include <uav_commander/impro_info.h>
#include <uav_percept/coordinate_payload.h>

using namespace std;
using namespace ros;

namespace uav_payload {
  class UAVPayload {
    public:
      // Service for call mavlink function
      ros::ServiceClient commandClient;

      // Subscriber for receiving gps data
      ros::Subscriber nsfSubscriber;
      
      // Subscriber for lap and impro status
      ros::Subscriber lapInfoSubscriber;
      ros::Subscriber improInfoSubscriber;
      ros::Subscriber coordinatePayloadSubscriber;

      // Variables for callbacks
      sensor_msgs::NavSatFix GPS;
      uav_commander::lap_info LapInfo;
      uav_commander::impro_info ImproInfo;
      uav_percept::coordinate_payload coordinatePayload;

    public:
      UAVPayload(ros::NodeHandle* node);
      ~UAVPayload();

      // Callback function for receiving gps data
      void nsfCB(const sensor_msgs::NavSatFix::ConstPtr& msg);

      // Callback function for lap and impro status
      void lapInfoCB(const uav_commander::lap_info::ConstPtr& msg);
      void improInfoCB(const uav_commander::impro_info::ConstPtr& msg);
      void dropZoneCB(const uav_percept::coordinate_payload::ConstPtr& msg);

      // Payload calculation function
      bool isDropInRange(double lat1, double lon1, double lat2, double lon2, int range);
      void doServoMove(int channel, int pwm);
      void doDropPayload();
  };
}