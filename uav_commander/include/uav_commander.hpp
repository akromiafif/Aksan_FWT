//ROS and Other Libraries
#include <ros/ros.h> //ROS Header
#include <math.h> //Maths header
#include <geometry_msgs/Twist.h> //Velocity
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

//MAVROS Message Types
#include <mavros_msgs/OverrideRCIn.h> //Payload release
#include <mavros_msgs/SetMode.h> //Define aircraft flightmode
#include <mavros_msgs/State.h> //Current autopilot state
#include <mavros_msgs/VFR_HUD.h> //Telemetry
#include <mavros_msgs/Waypoint.h> //Waypoint header
#include <mavros_msgs/WaypointClear.h> //Clear waypoints header
#include <mavros_msgs/WaypointList.h> //List current waypoints
#include <mavros_msgs/WaypointReached.h> //Confirmation on reaching waypoint

//MAVROS Service Types
#include <mavros_msgs/CommandBool.h> //Arming statement
#include <mavros_msgs/CommandTOL.h> //Takeoff and Landing
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/WaypointPull.h> //Request waypoints from FCU
#include <mavros_msgs/WaypointPush.h> //Send waypoints to FCU
#include <mavros_msgs/WaypointSetCurrent.h> //Define a waypoint for the aircraft to fly to (Provide index)


namespace uav_commander {
  class UAVCommander {
    public:
      ros::Publisher localPosPublisher;
      ros::Publisher localVelPublisher;
      ros::Publisher lapInfoPublisher;
      ros::Publisher improInfoPublisher;

      ros::Subscriber stateSubscriber;
      ros::Subscriber waypointReachSubscriber;
      ros::Subscriber vfrSubscriber;
      
      ros::ServiceClient armingClient;
      ros::ServiceClient landClient;
      ros::ServiceClient setModeClient;
      ros::ServiceClient takeoffClient;
      ros::ServiceClient commandClient;
      ros::ServiceClient waypointClearClient;

      mavros_msgs::State currStateGlobal;
      mavros_msgs::WaypointReached WayReached;
      mavros_msgs::VFR_HUD vfrHUD;
      nav_msgs::Odometry currPoseGlobal;

      std_msgs::Bool lapOne;
      std_msgs::Bool lapTwo;
      std_msgs::Bool lapThree;

      std_msgs::Bool improEnabled;
      
    public:
      UAVCommander(ros::NodeHandle* node);
      ~UAVCommander();

      // Callback function
      void stateCB(const mavros_msgs::State::ConstPtr& msg);
      void waypointReachedCB(const mavros_msgs::WaypointReached::ConstPtr& msg);
      void vfrCB(const mavros_msgs::VFR_HUD::ConstPtr& msg);

      void infoWayReached();
      void isImproEnabled();

      void setArm();
      void setAutoMissionMode();
      void setAirspeed(float airspeed);

      void dropPayload();
  };
}