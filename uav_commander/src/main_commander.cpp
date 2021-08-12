#include "uav_commander.cpp"



int main(int argc, char** argv) {
	ros::init(argc, argv, "uav_commander_node");
	ros::NodeHandle uav_commander_node;
	ros::Rate rate(30.0);

	//Initialize all publisher and subscriber in constructor
	uav_commander::UAVCommander uavCommander(&uav_commander_node);

	//Wait for connection to be established between MAVROS and autopilot
	while (ros::ok() && !uavCommander.currStateGlobal.connected) {
		ros::spinOnce();
		rate.sleep();
	}

	uavCommander.setAutoMissionMode();
	uavCommander.setAirspeed(20.0);
	uavCommander.setArm();
	
	ROS_INFO("========== STATUS ==========");

	if (uavCommander.currStateGlobal.armed) {
		ROS_INFO("Zaenab in action");
		ROS_INFO("Impro is set");
	} else {
		ROS_INFO("Zaenab not running");
	}

	while (uav_commander_node.ok()) {
		uavCommander.infoWayReached();
		uavCommander.isImproEnabled();

		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO("========== STATUS ==========");

	ros::spin();
	return 0;
}