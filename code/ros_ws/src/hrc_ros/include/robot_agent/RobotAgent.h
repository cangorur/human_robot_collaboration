/*
 *  RobotAgent.cpp
 *
 *  Created on: 19.04.2018
 *      Author: Orhan Can Görür
 *      Email: orhan-can.goeruer@dai-labor.de
 */

#ifndef HRC_ROS_SRC_ROBOTAGENT_H
#define HRC_ROS_SRC_ROBOTAGENT_H

#include <ros/ros.h>
// include <package_name/service_type_name.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <hrc_ros/InformRobotToTaskMang.h>
#include <hrc_ros/RobotUpdateMsg.h>
#include <hrc_ros/ResetRobotROS.h>

#include "simple_web_socket/server_ws.hpp"

using namespace std;
typedef SimpleWeb::SocketServer<SimpleWeb::WS> WsServer;

class RobotAgent {
public:
	RobotAgent();
	virtual ~RobotAgent();

private:
	/**
	 * Initialize all the ROS services/clients/topics and parameters.
	 * @todo in the future can be a ROS service initialized by the task manager
	 */
	void initialize();

	/**
	 * Control service to reset robot actions/position (task manager calls during the operation).
	 * @param req ResetRobotROS Request object
	 * @param res ResetRobotROS Response object
	 * @return True if initialization was successful
	 */
	bool resetScenario(hrc_ros::ResetRobotROSRequest &req, hrc_ros::ResetRobotROSResponse &res);

	/**
	 * Opens up the websocket server (despotMDPHuman connects to it and provides human action decision and the current state)
	 * The socket information is defined under the variables below. Port = 9090
	 */
	void update();


private:

	/*
	 * /task_manager/robot_status_update is a rosservice by task manager agent to get information on
	 * the new robot state, action, time of exection, rewards collected and the belief state etc.
	 */
	ros::ServiceClient callTaskMang_inform;


	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient pointToObj;
	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient grasp;
	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient cancelAction;
	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient planForGrasp;

	/*
	 * Advertised service. See the method: bool resetScenario
	 */
	ros::ServiceServer reset_scenario;

	/// Ros time instance to get the ROS up time. This is used as an information of when the robot acted, how long it took etc.
	ros::Time action_taken_time;

	/*
	* Simple Web Socket server variables
	*/
	WsServer server;
	///WebSocket (WS)-server at port 9090 using 1 thread
	int port = 8080;

	/// Flags to control the information communication to observation and task manager agents
	bool action_info_received = false;
	/// Flags to control the information communication to observation and task manager agents
	bool newstate_info_received = false;
	/// Flags to control the information communication to observation and task manager agents
	bool initial_state_received = true;

	/// Variables that hold the information of robot action, states, rewards, belief etc. to be sent to the other agents
	string robot_action_taken = "";
	/// Variables that hold the information of robot action, states, rewards, belief etc. to be sent to the other agents
	string robot_belief_state = "";
	/// Variables that hold the information of robot action, states, rewards, belief etc. to be sent to the other agents
	string robot_real_state = "";
	/// Variables that hold the information of robot action, states, rewards, belief etc. to be sent to the other agents
	string robot_immediate_reward = "";
	/// Variables that hold the information of robot action, states, rewards, belief etc. to be sent to the other agents
	string robot_total_disc_reward = "";

};

#endif /* HRC_ROS_SRC_ROBOTAGENT_H */
