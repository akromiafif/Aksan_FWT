#include <uav_commander.hpp>
#include <iostream>
#include <string>
#include <fstream>

namespace uav_commander {
  UAVCommander::UAVCommander(ros::NodeHandle node) {
    //Publishers (Transmits information to FCU)
    localPosPublisher = node.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    stateSubscriber = node.subscribe<mavros_msgs::State>("/mavros/state", 10, &UAVCommander::stateCB, this);
    waypointReachSubscriber = node.subscribe<mavros_msgs::WaypointReached>("mavros/mission/reached", 10, &UAVCommander::waypointReachedCB, this); // Listens for whether the aircraft reached the desired waypoint

    //Services and Clients (Allows for functions to be defined and called, returning a booleean result as to the clients success at executing the service
    //Two members: request and response
    armingClient = node.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    landClient = node.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    setModeClient = node.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    takeoffClient = node.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    commandClient = node.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
  }

  UAVCommander::~UAVCommander(){}

  void UAVCommander::stateCB(const mavros_msgs::State::ConstPtr& msg) {
    currStateGlobal = *msg;
  }

  void UAVCommander::waypointReachedCB(const mavros_msgs::WaypointReached::ConstPtr& msg) {
    WayReached = *msg;
  }

  void UAVCommander::setTakeOff(int takeoffAlt) {
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = takeoffAlt;

    if (takeoffClient.call(srv_takeoff)) {
      sleep(2);
      ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
    } else {
      ROS_ERROR("Failed Takeoff");
    }
  }

  void UAVCommander::setAutoLandMode(std::string mode) {
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = mode.c_str();
    if(setModeClient.call(srv_setMode)){
      ROS_INFO("SetMode to AUTO LAND success");
    }else{
      ROS_ERROR("Failed SetMode");
    }
  }

  void UAVCommander::setArm() {
    // arming
    ROS_INFO("Arming drone");
    mavros_msgs::CommandBool arm_request;
    arm_request.request.value = true;

    while (!currStateGlobal.armed && !arm_request.response.success && ros::ok()) {
      ros::Duration(.1).sleep();
      armingClient.call(arm_request);
    }

    if(arm_request.response.success) {
      ROS_INFO("Arming Successful");	
    } else {
      ROS_INFO("Arming failed with %d", arm_request.response.success);
    }
  }

  void UAVCommander::loadMission() {
    std::string const HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";

    system("rosrun mavros mavwp load ~/Aksan_Dev/catkin_gana_raksaka/src/uav_commander/missions/missionwp_husein.txt");
    ROS_INFO("Mission WayPoint LOADED!");
    
    int counter = 1;
    std::string myText;
    std::ifstream MyReadFile(HOME + "/Aksan_Dev/catkin_gana_raksaka/src/uav_commander/missions/missionwp_husein.txt");
    while (getline (MyReadFile, myText)) {
      ROS_INFO("%d %s", counter, myText.c_str());
      counter++;
    }

    MyReadFile.close();
    system("rosrun mavros mavwp show");
  }

  void UAVCommander::setAutoMissionMode() {
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    std::string mode = "AUTO.MISSION";
    srv_setMode.request.custom_mode = mode.c_str();
    if(setModeClient.call(srv_setMode)){
      ROS_INFO("SetMode Success");
    }else{
      ROS_ERROR("Failed SetMode");
    }
  }

  void UAVCommander::infoWayReached() {
    //Communication Transmission Rate (>>2Hz)
    ros::Rate rate(20.0); //rate defines the frequency (rate is an attribute of the ROS::Rate topic)
    
    while (ros::ok) {
      ROS_INFO("WP: reached %d", WayReached.wp_seq);

      ros::spinOnce();
      rate.sleep();
    }
  }
}