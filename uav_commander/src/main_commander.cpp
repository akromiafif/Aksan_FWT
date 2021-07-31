#include "uav_commander.cpp"



int main(int argc, char** argv) {
	ros::init(argc, argv, "uav_commander_node");
	ros::NodeHandle uav_commander_node;
	ros::Rate rate(20.0);

	//Initialize all publisher and subscriber in constructor
	uav_commander::UAVCommander uavCommander(uav_commander_node);

	//Wait for connection to be established between MAVROS and autopilot
	while (ros::ok() && !uavCommander.currStateGlobal.connected) {
		ros::spinOnce();
		rate.sleep();
	}

	// //Ensures the aircraft is disarmed and set to manual mode initially
	// uavCommander.initializeManualMode();

	// //Define flight velocity
	// uavCommander.setAirspeed(20.0);

	// //Defines desired aircraft flight mode and arms motors
	// uavCommander.setFlightMode();

	uavCommander.setAutoMissionMode();
	uavCommander.setAirspeed(20.0);
	uavCommander.setArm();
	
	ROS_INFO("========== READING WAYPOINT ==========");
	uavCommander.infoWayReached();

	ros::spin();
	return 0;
}