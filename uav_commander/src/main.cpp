#include "uav_commander.cpp"


int main(int argc, char** argv) {
	ros::init(argc, argv, "auto_mission_node");
	ros::NodeHandle auto_mission_node;
	ros::Rate rate(20.0);

	uav_commander::UAVCommander uavCommander(auto_mission_node);
	// uavCommander.loadMission();
	uavCommander.setAutoMissionMode();
	uavCommander.setArm();
	ROS_INFO("Start executing all mission");
	
	ros::spinOnce();
	rate.sleep();

	ROS_INFO("========== READING WAYPOINT ==========");
	uavCommander.infoWayReached();

	return 0;
}