#include <uav_commander.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include <uav_commander/lap_info.h>
#include <uav_commander/impro_info.h>


namespace uav_commander {
  UAVCommander::UAVCommander(ros::NodeHandle* node) {
    ROS_INFO("======= UAVCommander Initialize =======" );

    //Publishers (Transmits information to FCU)
    localPosPublisher = node->advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);
    localVelPublisher = node->advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1000);
    lapInfoPublisher = node->advertise<uav_commander::lap_info>("/lap_info", 1000);
    improInfoPublisher = node->advertise<uav_commander::impro_info>("/impro_info", 1000);

    //Subscribers (Listens for information from FCU)
    stateSubscriber = node->subscribe<mavros_msgs::State>("/mavros/state", 1000, &UAVCommander::stateCB, this);
    waypointReachSubscriber = node->subscribe<mavros_msgs::WaypointReached>("/mavros/mission/reached", 1000, &UAVCommander::waypointReachedCB, this); // Listens for whether the aircraft reached the desired waypoint
    vfrSubscriber = node->subscribe<mavros_msgs::VFR_HUD>("/mavros/vfr_hud", 1000, &UAVCommander::vfrCB, this); //Listens for aircraft variables

    //Services and Clients (Allows for functions to be defined and called, returning a booleean result as to the clients success at executing the service
    //Two members: request and response
    armingClient = node->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    landClient = node->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    setModeClient = node->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    takeoffClient = node->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    commandClient = node->serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    ROS_INFO("======= UAVCommander Initialize Completed =======" );
    ROS_INFO("                                                 " );
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


  // Get the data from aircraft
  void UAVCommander::infoWayReached() {
    uav_commander::lap_info lapInfo;

      if (WayReached.wp_seq == 3) {
        lapOne.data = true;
      } else {
        lapOne.data = false;
      }

      if (WayReached.wp_seq == 7) {
        lapTwo.data = true;
      } else {
        lapTwo.data = false;
      }

      if (WayReached.wp_seq == 11) {
        lapThree.data = true;
      } else {
        lapThree.data = false;
      }

      lapInfo.lap_one = lapOne;
      lapInfo.lap_two = lapTwo;
      lapInfo.lap_three = lapThree;

      lapInfoPublisher.publish(lapInfo);
  }

  // Check when is impro enabled or not
  void UAVCommander::isImproEnabled() {
    uav_commander::impro_info improInfo;

    if (currStateGlobal.armed) {
      if (WayReached.wp_seq == 1 || WayReached.wp_seq == 5 || WayReached.wp_seq == 9) {
        improEnabled.data = true;
        improInfo.impro_enabled = improEnabled;
      }

      if (WayReached.wp_seq == 2 || WayReached.wp_seq == 6 || WayReached.wp_seq == 10) {
        improEnabled.data = false;
        improInfo.impro_enabled = improEnabled;
      }

      improInfoPublisher.publish(improInfo);
    }
  }




  //Set aircraft to AUTO mode
  void UAVCommander::setAutoMissionMode() {
    ROS_INFO(" ===== Auto Mission Mode ===== ");

    mavros_msgs::SetMode srv_setMode;
    mavros_msgs::SetMode::Request request;
    mavros_msgs::SetMode::Response response;

    request.base_mode = 0;
    std::string mode = "AUTO.MISSION";
    request.custom_mode = mode.c_str();

    bool success = setModeClient.call(request, response);

    if(success){
      ROS_INFO("Auto Mission ON");
    } else{ 
      ROS_ERROR("Failed to Auto Mission");
    }
    ROS_INFO(" ===== Auto Mission Completed ===== ");

      
    // ros::Rate rate(20.0);
    // ros::spinOnce();
    // rate.sleep();

    ROS_INFO(" ===== Auto Mission Mode Completed ===== ");
    ROS_INFO("                                         ");
  }

  //Define flight linear velocity
  void UAVCommander::setAirspeed(float airspeed) {
    ROS_INFO(" ===== Set Airspeed ===== ");

    geometry_msgs::Twist velocity;
    velocity.linear.x = airspeed;
    localVelPublisher.publish(velocity);

    // ros::Rate rate(20.0);
    // ros::spinOnce();
    // rate.sleep();
  
    ROS_INFO(" ===== Set Airspeed Completed ===== ");
  }

  //Arming the aircraft
  void UAVCommander::setArm() {
    ROS_INFO(" ===== Arming UAV ===== ");

    mavros_msgs::CommandBool srv_arm;
    mavros_msgs::CommandBool::Request request;
    mavros_msgs::CommandBool::Response response;

    request.value = true;
    bool success = false;

    while (!currStateGlobal.armed && !response.success && ros::ok()) {
      ros::Duration(.1).sleep();

      success = armingClient.call(request, response);
    }

    if(success) {
      ROS_INFO("Arming ON");	
    } else {
      ROS_INFO("Arming failed with %d", response.success);
    }

    // ros::Rate rate(20.0);
    // ros::spinOnce();
    // rate.sleep();

    ROS_INFO(" ===== Arming Completed ===== ");
    ROS_INFO("                              ");
  }











  

  





























  // void UAVCommander::initializeManualMode() {
  //   ROS_INFO(" ===== Initialize Manual Mode ===== ");
  //   ros::Rate rate(20.0); 
  //   ros::Time lastRequest = ros::Time::now();
  //   mavros_msgs::SetMode srv_setMode;
  //   mavros_msgs::CommandBool srv_motorCMD;

  //   std::string mode = "MANUAL";
  //   srv_setMode.request.custom_mode = mode.c_str();
  //   srv_motorCMD.request.value = false;

  //   while (currStateGlobal.mode != "MANUAL" || currStateGlobal.armed) { //Disarming and MANUAL mode transmission loop 
  //     if (currStateGlobal.mode != "MANUAL" && (ros::Time::now() - lastRequest > ros::Duration(0.2))) { //Delay by 'X' seconds to prevent bottleneck
      
  //       if (setModeClient.call(srv_setMode) && srv_setMode.response.mode_sent) { //Call service client
  //         ROS_INFO("Manual Flight Mode"); //Confirmation
  //       }

  //       lastRequest = ros::Time::now(); //Update timer
  //     } else {
  //       if (currStateGlobal.armed && (ros::Time::now() - lastRequest > ros::Duration(0.2))) { //Delay by 'X' seconds to prevent bottleneck
      
  //         if (armingClient.call(srv_motorCMD) && srv_motorCMD.response.success) { //Call service client
  //           ROS_INFO("Motors Disarmed"); //Confirmation
  //         }
  //         lastRequest = ros::Time::now(); //Update timer
  //       }
  //     }

  //     ros::spinOnce();
  //     rate.sleep();
  //   }
  //   ROS_INFO(" ===== Initialize Manual Mode Completed ===== ");
  // }


  // /*************Flight Mode Selection************/
  // //Defines desired aircraft flight mode and arms motors
  // void UAVCommander::setFlightMode() {
  //   ROS_INFO(" ===== Set Flight Mode ===== ");

  //   ros::Rate rate(20.0); 
  //   ros::Time lastRequest = ros::Time::now();
  //   mavros_msgs::SetMode srv_setMode;
  //   mavros_msgs::CommandBool srv_motorCMD;

  //   srv_setMode.request.custom_mode = "AUTO"; //Assign the desired aircraft flight mode
  //   srv_motorCMD.request.value = true; //Arm the aircraft (Service-Clientmemeber)
  //   lastRequest = ros::Time::now();

  //   //Confirm that flight mode was successfully entered
  //   while(currStateGlobal.mode != "AUTO" || !currStateGlobal.armed) { //Arming and AUTO Flight Mode Transmission Loop{
  //     if(!currStateGlobal.armed && (ros::Time::now() - lastRequest > ros::Duration(2.0))) { //Delay by 'X' seconds to prevent bottleneck    
  //       if(armingClient.call(srv_motorCMD) && srv_motorCMD.response.success) {
  //         ROS_INFO("Aircraft Armed");
  //       }
  //       lastRequest = ros::Time::now(); //Update current timer time
  //     }

  //     if(!currStateGlobal.armed && (ros::Time::now() - lastRequest > ros::Duration(2.0))) {
  //       if(currStateGlobal.mode != "AUTO" && (ros::Time::now() - lastRequest > ros::Duration(2.0))) { //Delay by 'X' seconds to prevent bottleneck
        
  //         if(setModeClient.call(srv_setMode) && srv_setMode.response.mode_sent) {
  //           ROS_INFO("AUTO Flight Mode");
  //         }
  //         lastRequest = ros::Time::now();
  //       }
  //     }

  //     ros::spinOnce();
  //     rate.sleep();
  //   }

  //   ROS_INFO(" ===== Set Flight Mode Completed ===== ");
  // }

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