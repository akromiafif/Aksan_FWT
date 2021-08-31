#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <cmath>

// import header file
#include <uav_payload.hpp>

namespace uav_payload {
  UAVPayload::UAVPayload(ros::NodeHandle* node) {
    //GPS Subscriber
    nsfSubscriber = node->subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1000, &UAVPayload::nsfCB, this);
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
      ROS_INFO("Payload Dropped");
    } else {
      ROS_INFO("Payload not dropped");
    }

    ros::spinOnce();
		rate.sleep();
  }

  void UAVPayload::doDropPayload() {
    bool isImproEnabled = ImproInfo.impro_enabled.data;
    bool isLapTwo = LapInfo.lap_two.data;

    if (isImproEnabled) {
      if (isLapTwo) {
        bool isInRange = isDropInRange(GPS.latitude, GPS.longitude, 40.1, 40.2, 11);
      }
    }
  }
}