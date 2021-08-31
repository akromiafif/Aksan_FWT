#include "uav_percept.cpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "uav_percept_node");
  ros::NodeHandle uav_percept_node;

  uav_percept::UAVPercept uavPercept(&uav_percept_node);

  ros::spin();
	return 0;
}
