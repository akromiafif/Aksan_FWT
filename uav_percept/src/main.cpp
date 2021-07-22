#include "uav_percept.cpp"



int main(int argc, char** argv) {
	ros::init(argc, argv, "uav_percept_node");
  ros::NodeHandle uav_percept_node;
	ros::Rate rate(20.0);

  uav_percept::UAVPercept uavCommander(uav_percept_node);

  ros::spin();

	return 0;
}
