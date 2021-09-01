#include "uav_payload.cpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "uav_payload_node");
  ros::NodeHandle uav_payload_node;
  ros::Rate rate(30.0);

  uav_payload::UAVPayload uavPayload(&uav_payload_node);

  while(uav_payload_node.ok()) {
    uavPayload.doDropPayload();
    ros::spinOnce();
		rate.sleep();
  }

  ros::spin();
	return 0;
}
