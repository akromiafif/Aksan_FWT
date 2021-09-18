#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <ctime>

// import header file
#include <uav_payload.hpp>

namespace uav_payload {
  UAVPayload::UAVPayload(ros::NodeHandle* node) {
    //  GPS Subscriber
    nsfSubscriber = node->subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1000, &UAVPayload::nsfCB, this);

    // DropZone coordinate
    coordinatePayloadSubscriber = node->subscribe<uav_percept::coordinate_payload>("/coordinate_payload", 1000, &UAVPayload::dropZoneCB, this);

    //LapInfo Subscriber
    lapInfoSubscriber = node->subscribe<uav_commander::lap_info>("/lap_info", 1000, &UAVPayload::lapInfoCB, this);

    //improInfo Subscriber
    improInfoSubscriber = node->subscribe<uav_commander::impro_info>("/impro_info", 1000, &UAVPayload::improInfoCB, this);

    commandClient = node->serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
  }

  UAVPayload::~UAVPayload() {}

  void UAVPayload::nsfCB(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    GPS = *msg;
  }

  void UAVPayload::lapInfoCB(const uav_commander::lap_info::ConstPtr& msg) {
    LapInfo = *msg;
  }

  void UAVPayload::improInfoCB(const uav_commander::impro_info::ConstPtr& msg) {
    ImproInfo = *msg;
  }

  void UAVPayload::dropZoneCB(const uav_percept::coordinate_payload::ConstPtr& msg) {
    coordinatePayload = *msg;
  }

  bool UAVPayload::isDropInRange(double lat1, double lon1, double lat2, double lon2, int range) {
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;

    // convert to radians
    lat1 = (lat1) * M_PI / 180.0;
    lat2 = (lat2) * M_PI / 180.0;

    // apply formulae
    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double rad = 6371;
    double c = 2 * asin(sqrt(a));
    double distance = rad * c * 1000;

    if (distance < range) {
      return true;
    }

    return false;
  }

  void UAVPayload::doServoMove(int channel, int pwm) {
    ros::Rate rate(30.0);

    mavros_msgs::CommandLong srv;
    srv.request.command = 183;
    srv.request.param1 = channel;
    srv.request.param2 = pwm;
    bool succeed = commandClient.call(srv);

    if (succeed) {
      ROS_INFO("Servo successfully to move");
    } else {
      ROS_INFO("Servo failed to move");
    }

    ros::spinOnce();
		rate.sleep();
  }

  void UAVPayload::doDropPayload() {
    bool isImproEnabled = ImproInfo.impro_enabled.data;
    bool isLapTwo = LapInfo.lap_two.data;
    bool isLapThree = LapInfo.lap_three.data;

    if (isImproEnabled) {
      if (isLapTwo) {
        float coordinateLat = coordinatePayload.lat_drop.data;
        float coordinateLon = coordinatePayload.long_drop.data;

        if (coordinateLat == 0.0 && coordinateLon == 0.0) {
          srand(time(0));
          if ( (rand() % 1000 + 1) > 500) {
            ROS_INFO("Random %d: ", rand() % 1000 + 1);
            sleep(5);
            doServoMove(8, 1250);
            ROS_INFO("PDM Open");
            sleep(3);
            doServoMove(8, 2200);
            ROS_INFO("PDM Close");
          }

        } else {
          bool isInRange = isDropInRange(GPS.latitude, GPS.longitude, coordinatePayload.lat_drop.data, coordinatePayload.long_drop.data, 17.09);

          if (isInRange) {
            doServoMove(8, 1250);
            ROS_INFO("PDM Open");
            sleep(3);
            doServoMove(8, 2200);
            ROS_INFO("PDM Close");
          }
        }
      }

      if (isLapThree) {
        float coordinateLat = coordinatePayload.lat_drop.data;
        float coordinateLon = coordinatePayload.long_drop.data;

        if (coordinateLat == 0.0 && coordinateLon == 0.0) {
          srand(time(0));
          if ( (rand() % 1000 + 1) > 500) {
            ROS_INFO("Random %d: ", rand() % 1000 + 1);
            sleep(5);
            doServoMove(9, 1250);
            ROS_INFO("PDM Open");
            sleep(3);
            doServoMove(9, 2200);
            ROS_INFO("PDM Close");
          }

        } else {
          bool isInRange = isDropInRange(GPS.latitude, GPS.longitude, coordinatePayload.lat_drop.data, coordinatePayload.long_drop.data, 17.09);

          if (isInRange) {
            doServoMove(9, 1250);
            ROS_INFO("PDM Open");
            sleep(3);
            doServoMove(9, 2200);
            ROS_INFO("PDM Close");
          }
        }
      }
    }
  }
}