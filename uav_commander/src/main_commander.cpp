#include "uav_commander.cpp"



int main(int argc, char** argv) {
	ros::init(argc, argv, "uav_commander_node");
	ros::NodeHandle uav_commander_node;
	ros::Rate rate(20.0);

	uav_commander::UAVCommander uavCommander(uav_commander_node);
	uavCommander.setAutoMissionMode();
	uavCommander.setArm();
	
	ROS_INFO("========== READING WAYPOINT ==========");
	uavCommander.infoWayReached();

	ros::spin();
	return 0;
}