#include "uav_payload.cpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "uav_payload_node");
  ros::NodeHandle uav_payload_node;

  uav_payload::UAVPayload uavPayload(&uav_payload_node);

  ros::spin();
	return 0;
}
