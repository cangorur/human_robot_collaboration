/*
 *  HumanAgent.cpp
 *
 *  Created on: 19.04.2018
 *      Author: Orhan Can Görür
 *      Email: orhan-can.goeruer@dai-labor.de
 */

#ifndef HRC_ROS_SRC_HUMANAGENT_H_
#define HRC_ROS_SRC_HUMANAGENT_H_

#include <ros/ros.h>
// include <package_name/service_type_name.h>
#include <std_srvs/Trigger.h>
#include <hrc_ros/InformHumanToTaskMang.h>
#include <hrc_ros/InformHumanAction.h>
#include <hrc_ros/InformHumanState.h>
#include <hrc_ros/ResetHumanROS.h>

#include "simple_web_socket/server_ws.hpp"


class HumanAgent {
public:
	HumanAgent();
	virtual ~HumanAgent();

private:
	/**
	 * Initialize all the ROS services/clients/topics and parameters.
	 * @todo in the future can be a ROS service initialized by the task manager
	 */
	void initialize();
	
	/**
	 * Control service to reset human actions/position (task manager calls during the operation).
	 * @param req ResetHumanROS Request object
	 * @param res ResetHumanROS Response object
	 * @return True if initialization was successful
	 */
	bool resetScenario(hrc_ros::ResetHumanROSRequest &req, hrc_ros::ResetHumanROSResponse &res);

	/**
	 * Opens up the websocket server (despotMDPHuman connects to it and provides human action decision and the current state)
	 * The socket information is defined under the variables below. Port = 9090
	 */
	void update();
	
	
private:
	
	/*
	 * rosservice call /observation_agent/inform_human_action is a rosservice by observation agent to get information on
	 * the human action selected (action flow: despotMDP (thru websocked) --> Human Agent (executed) -->  Observation Agent (inform))
	 */
	ros::ServiceClient callObs_inform_action;

	/*
	 * /observation_agent/inform_new_human_state is a rosservice by observation agent to get information on
	 * the new human state (state flow: despotMDP (thru websocked) --> Human Agent (inform) -->  Observation Agent (inform))
	 */
	ros::ServiceClient callObs_inform_newState;

	/*
	 * /task_manager/human_status_update is a rosservice by task manager agent to get information on
	 * the new human state, action, time of exection etc.
	 */
	ros::ServiceClient callTaskMang_inform;

	/*
	 * ROS services opened by MORSE to control human actions
	 */
	ros::ServiceClient stayIdle;
	ros::ServiceClient walkAway;
	ros::ServiceClient lookAround;
	ros::ServiceClient warnRobot;
	ros::ServiceClient attemptGrasp;
	ros::ServiceClient grasp;

	/*
	 * Advertised service. See the method: bool resetScenario
	 */
	ros::ServiceServer reset_scenario;
	
	/// Ros time instance to get the ROS up time. This is used as an information of when the human acted, how long it took etc.
	ros::Time action_taken_time;

	/*
	* Simple Web Socket server variables
	*/
	typedef SimpleWeb::SocketServer<SimpleWeb::WS> WsServer;
	WsServer server;
	///WebSocket (WS)-server at port 9090 using 1 thread
	server.config.port = 9090;

	/// Flags to control the information communication to observation and task manager agents
	bool action_info_received = false;
	bool newstate_info_received = false;
	bool initial_state_received = true;

	/// Variables that hold the information of human action and states to be sent to the other agents
	string human_action_taken = "";
	string human_belief_state = "";
	string human_real_state = "";

	/// Task counter: Used to regulate human actions (failing, known as attempting_grasp, more often as more tasks assigned)
	int task_number_counter = 0;


};

#endif /* HRC_ROS_SRC_HUMANAGENT_H_ */
