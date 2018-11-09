/*
 *  HumanAgent.cpp
 *
 *  Created on: 19.04.2018
 *      Author: Orhan Can Görür
 *      Email: orhan-can.goeruer@dai-labor.de
 */

#ifndef HRC_ROS_SRC_HUMANAGENT_H
#define HRC_ROS_SRC_HUMANAGENT_H

#include <ros/ros.h>
#include <std_msgs/String.h>
// include <package_name/service_type_name.h>
#include <std_srvs/Trigger.h>
#include <hrc_ros/InformHumanToTaskMang.h>
#include <hrc_ros/InformHumanAction.h>
#include <hrc_ros/DrawNewStateMC.h>
#include <hrc_ros/InitiateScenario.h>
#include <hrc_ros/InformHumanState.h>
#include <hrc_ros/ResetHumanROS.h>

#include "simple_web_socket/server_ws.hpp"

using namespace std;
typedef SimpleWeb::SocketServer<SimpleWeb::WS> WsServer;


class HumanAgent {
public:
	HumanAgent();
	virtual ~HumanAgent();

        //TODO: use only enums
        enum STATE{
            UNKNOWN_STATE = -1,
            TASKHUMAN,
            GLOBAL_SUCCESS,
            GLOBAL_FAIL,
            FAILED_TO_GRASP,
            NO_ATTENTION,
            EVALUATING,
            TIRED,
            RECOVERY,
            ROBOT_INTERFERED,
            WARN,
            ROBOT_IS_WARNED,
            TASK_ROBOT
        };

        enum ACTION{
            UNKNOWN_ACTION = -1,
            GRASP,
            LOOK_AROUND,
            IDLE,
            WALK_AWAY,
            WARN_ROBOT
        };

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
	 * Work principle:
	 * 		1. It first receives the state information from DESPOT (line 206 else condition). Does nothing but inform obs agent
	 * 		2. Then, it receives the action information from DESPOT (line 112, first if condition)
	 *				2.1  It draws a state from human_mc_sampler in case the action is grasp (to decide the grasp will succeed or not).
	 *  			2.2  If it is not grasp, human acts as informed and draws a state. It waits for the robot to act (in human_mc_sampler) before
	 *		3. After a state is drawn following the robot's act, the new action and state info is sent to the task manager (line 222 if action_received)
	 *		4. In the same condition finally the new state information is sent to DESPOT and we iterate back to step 1.
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


	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient stayIdle;
	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient walkAway;
	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient lookAround;
	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient warnRobot;
	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient attemptGrasp;
	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient grasp;

	///
  ros::ServiceClient humanStateSamplerClient;
  ros::ServiceClient requestGraspingOutcome;

	ros::ServiceClient taskManagerNewScenarioClient;

	/*
	 * Advertised service. See the method: bool resetScenario
	 */
	ros::ServiceServer reset_scenario;

	/// Ros time instance to get the ROS up time. This is used as an information of when the human acted, how long it took etc.
	ros::Time action_taken_time;

private:
	//readability purpose
    HumanAgent::STATE stringToStateInt(string state);
    HumanAgent::ACTION stringToActionInt(string action);
    std::string intToAction(const int taken_action);

    map<string, int> stateMap;
    vector<string> states;

    /*
	* Simple Web Socket server variable
	*/
	WsServer server;
	///WebSocket (WS)-server at port 9090 using 1 thread
	int port = 9090;

	/// Flags to control the information communication to observation and task manager agents
	bool action_info_received = false;
	/// Flags to control the information communication to observation and task manager agents
	bool newstate_info_received = false;
	/// Flags to control the information communication to observation and task manager agents
	bool initial_state_received = true;
	/// Flags to control the information communication to observation and task manager agents
	bool terminal_state_reached = false;

	/// Variables that hold the information of human action and states to be sent to the other agents
	string human_action_taken = "";
	/// Variables that hold the information of human action and states to be sent to the other agents
	string human_belief_state = "";
	/// Variables that hold the information of human action and states to be sent to the other agents
	string human_real_state = "";

	/// Task counter: Used to regulate human actions (failing, known as attempting_grasp, more often as more tasks assigned)
	int task_number_counter = 0;


};

#endif /* HRC_ROS_SRC_HUMANAGENT_H */
