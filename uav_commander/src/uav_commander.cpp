#include <uav_commander.hpp>
#include <iostream>
#include <string>
#include <fstream>




namespace uav_commander {
  UAVCommander::UAVCommander(ros::NodeHandle node) {
    //Publishers (Transmits information to FCU)
    localPosPublisher = node.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    stateSubscriber = node.subscribe<mavros_msgs::State>("/mavros/state", 10, &UAVCommander::stateCB, this);
    waypointReachSubscriber = node.subscribe<mavros_msgs::WaypointReached>("/mavros/mission/reached", 10, &UAVCommander::waypointReachedCB, this); // Listens for whether the aircraft reached the desired waypoint
    vfrSubscriber = node.subscribe<mavros_msgs::VFR_HUD>("/mavros/vfr_hud", 10, &UAVCommander::vfrCB, this); //Listens for aircraft variables

    //Services and Clients (Allows for functions to be defined and called, returning a booleean result as to the clients success at executing the service
    //Two members: request and response
    armingClient = node.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    landClient = node.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    setModeClient = node.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    takeoffClient = node.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    commandClient = node.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    ROS_INFO("UAVCommander Initialized");
  }

  UAVCommander::~UAVCommander(){}

  //Current flight controller status
  void UAVCommander::stateCB(const mavros_msgs::State::ConstPtr& msg) {
    currStateGlobal = *msg;
  }

  //Current waypoint reached status
  void UAVCommander::waypointReachedCB(const mavros_msgs::WaypointReached::ConstPtr& msg) {
    WayReached = *msg;
  }

  //Current flight controller variables (airspeed, heading, etc)
  void UAVCommander::vfrCB(const mavros_msgs::VFR_HUD::ConstPtr& msg) {
    vfrHUD = *msg;
  }

  void UAVCommander::setArm() {
    ROS_INFO(" ===== Arming UAV ===== ");
    mavros_msgs::CommandBool arm_request;
    arm_request.request.value = true;

    while (!currStateGlobal.armed && !arm_request.response.success && ros::ok()) {
      ros::Duration(.1).sleep();
      armingClient.call(arm_request);
    }

    if(arm_request.response.success) {
      ROS_INFO("Arming Successfull");	
    } else {
      ROS_INFO("Arming failed with %d", arm_request.response.success);
    }
    ROS_INFO(" ===== Arming Completed ===== ");

    ros::Rate rate(20.0);
    ros::spinOnce();
    rate.sleep();
  }

  void UAVCommander::setAutoMissionMode() {
    ROS_INFO(" ===== Auto Mission UAV ===== ");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    std::string mode = "AUTO.MISSION";
    srv_setMode.request.custom_mode = mode.c_str();

    if(setModeClient.call(srv_setMode)){
      ROS_INFO("SetMode to Auto Mission Mode Successfull");
    }else{
      ROS_ERROR("Failed SetMode to Auto Mission Mode");
    }
    ROS_INFO(" ===== Auto Mission Completed ===== ");

      
    ros::Rate rate(20.0);
    ros::spinOnce();
    rate.sleep();
  }

  void UAVCommander::infoWayReached() {
    //Communication Transmission Rate (>>2Hz)
    ros::Rate rate(20.0); //rate defines the frequency (rate is an attribute of the ROS::Rate topic)
    
    while (ros::ok) {
      ROS_INFO("alt: %f", vfrHUD.altitude);
      ROS_INFO("heading: %d", vfrHUD.heading);
      ROS_INFO("WP: reached %d", WayReached.wp_seq);
      if (WayReached.wp_seq == 1) {
        ROS_INFO("alt: %f", vfrHUD.altitude);
        ROS_INFO("heading: %d", vfrHUD.heading);
        ROS_INFO("WP: reached %d", WayReached.wp_seq);
      } else if (WayReached.wp_seq == 3) {
        ROS_INFO("WP: reached %d", WayReached.wp_seq);
      }

      ros::spinOnce();
      rate.sleep();
    }
  }





















  // void UAVCommander::loadMission() {
  //   std::string const HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";

  //   system("rosrun mavros mavwp load ~/Aksan_Dev/catkin_gana_raksaka/src/uav_commander/missions/missionwp_husein.txt");
  //   ROS_INFO("Mission WayPoint LOADED!");
    
  //   int counter = 1;
  //   std::string myText;
  //   std::ifstream MyReadFile(HOME + "/Aksan_Dev/catkin_gana_raksaka/src/uav_commander/missions/missionwp_husein.txt");
  //   while (getline (MyReadFile, myText)) {
  //     ROS_INFO("%d %s", counter, myText.c_str());
  //     counter++;
  //   }

  //   MyReadFile.close();
  //   system("rosrun mavros mavwp show");
  // }

  // void UAVCommander::setTakeOff(int takeoffAlt) {
  //   mavros_msgs::CommandTOL srv_takeoff;
  //   srv_takeoff.request.altitude = takeoffAlt;

  //   if (takeoffClient.call(srv_takeoff)) {
  //     sleep(2);
  //     ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
  //   } else {
  //     ROS_ERROR("Failed Takeoff");
  //   }
  // }

  // void UAVCommander::setAutoLandMode(std::string mode) {
  //   mavros_msgs::SetMode srv_setMode;
  //   srv_setMode.request.base_mode = 0;
  //   srv_setMode.request.custom_mode = mode.c_str();
  //   if(setModeClient.call(srv_setMode)){
  //     ROS_INFO("SetMode to AUTO LAND success");
  //   }else{
  //     ROS_ERROR("Failed SetMode");
  //   }
  // }
}