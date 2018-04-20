/*
 *  ObservationAgent.cpp
 *
 *  Created on: 19.04.2018
 *      Author: Orhan Can Görür
 *      Email: orhan-can.goeruer@dai-labor.de
 */

#ifndef HRC_ROS_SRC_OBSERVATIONAGENT_H
#define HRC_ROS_SRC_OBSERVATIONAGENT_H

#include <ros/ros.h>
// include <package_name/service_type_name.h>
#include <std_srvs/Trigger.h>
#include <hrc_ros/TraySensor.h>
#include <hrc_ros/InformHumanAction.h>
#include <hrc_ros/InformHumanState.h>
#include <hrc_ros/InformObsToTaskMang.h>
#include <hrc_ros/ResetObsROS.h>

#include "simple_web_socket/client_ws.hpp"

using namespace std;
typedef SimpleWeb::SocketClient<SimpleWeb::WS> WsClient;


class ObservationAgent {
public:
	ObservationAgent();
	virtual ~ObservationAgent();

private:
	
	/**
	 * Initialize all the ROS services/clients/topics and parameters.
	 * @todo in the future can be a ROS service initialized by the task manager
	 */
	void initialize();
	
	/**
	 * Control service to reset observation agent (task manager calls during the operation).
	 * @param req ResetHumanROS Request object
	 * @param res ResetHumanROS Response object
	 * @return True if initialization was successful
	 */
	bool resetScenario(hrc_ros::ResetObsROSRequest &req, hrc_ros::ResetObsROSResponse &res);

	/**
	 * Timer event created to track the human task duration
	 */
	void HumanTaskTimer(const ros::TimerEvent&);

	/**
	 * Callback function to catch the updated tray sensor information
	 */
	void ReceiveTraySensors(const hrc_ros::TraySensor &msg);

	/// ---- Operation based functions. Mostly for reactive robot's rule based decisions --- ///

	// ************** WEB CLIENTS TO COMMUNICATE WITH DESPOT ******************** //
	/**
	 * This advertised rosservice is called "/observation_agent/inform_human_action". It is called by human_agent after it receives the action
	 * information from Despot MDP, which is then executed and gathered by MORSE (this is hard coded. If human action of grasp executed, 
	 * human grasps and once successful it is directly provided back to the human agent as "human grasped"). This then triggers the rest of mapping 
	 * operations as implemented with below functions. Finally, this method opens a web client to despot MDP or POMDP to send the information for 
	 * decision-making process
	 * @todo (HAR system to be integrated to here) Normally the human observations shouldnt come from the human agent itself. It is sort of human 
	 * is informing the action taken directly back to the system. Ideal would be to detect the human actions through actual observation process 
	 * (e.g. input from a camera) and should be processed here to recognize.
	 */
	bool action_to_obs_Map(hrc_ros::InformHumanAction::Request &req, hrc_ros::InformHumanAction::Response &res);

	/**
	 * This advertised rosservice is called "/observation_agent/inform_new_human_state". It is called by human_agent after it receives the human state
	 * information from despot MDP human decision-making process. Then the (actual) state information is mapped to the robot states with the functions below
	 * and provided to the despot MDP / POMDP robot (thru web client) as the current state information. In MDP it is used as the decision-making for the next action, 
	 * in POMDP it is used for calculating the immediate rewards.
	 */
	bool humanSt_to_robotSt_Map(hrc_ros::InformHumanState::Request &req, hrc_ros::InformHumanState::Response &res);
	// ********************************** //

	/**
	 * Counting the human states of being tired, notattending, failed etc. 
	 * This is for reactive robot's rule based decision-making mechanism
	 */
	void humanStCounter(string humanState);

	/**
	 * This function gets the actual human state and maps them to the POMDP robot states
	 * E.g. FailedGrasp state of human may correspond to MayNotBeCapable or NeedsHelp. The goal is to calcuate how accurate the robot
	 * estimates the human beliefs correctly. The mapping is for now handled manually (rule based). 
	 * @todo The detection of human needs help is mapped completely rule based which may not be the case. So, in real user studies this sort of 
	 * mapping is useless. Even in the simulation since the human model does not have a direct state of 'human needs help' so understanding it is 
	 * manually implemented for now.
	 */
	string getRealRbtStPOMDP(string humanState);

	/**
	 * This function gets the human observation names and maps them into logic numbers (binaries, e.g. 100001).
	 * Refer to the google doc for the map: https://docs.google.com/spreadsheets/d/1jDDyNXrNnYsDy5L82CDVipNZQwEvev44tx2FqHkm2wE/edit#gid=2060522298
	 * @todo: the observable names of ov and oir to be updated. Currently !OV = Looking Around and OIR = Human is detected
	 * @todo: Simplify the observation combinations if the reactive model responds almost better than the proactive one!
	 */
	string MapObservablesToObservations(bool ov, bool oir, bool a0, bool ipd, bool a4, bool a2, bool upd);

	/**
	 * This function maps the binary observable numbers to the POMDP model observations. POMDP model
	 * holds observations in numbers (designed by pomdpx modelling template) and they are manually numbered. This way POMDP understands
	 * what observations human has emitted and makes a decision accordingly. The observation information is sent to the despot POMDP executor.
	 * Refer to the google doc for the mapping: https://docs.google.com/spreadsheets/d/1jDDyNXrNnYsDy5L82CDVipNZQwEvev44tx2FqHkm2wE/edit#gid=2060522298
	 */
	string MapObservationsToPOMDP(string observable);

	/**
	 * This function gets the actual human state and maps them to the MDP robot states. The same as in getRealRbtStPOMDP. It is to compare the human state 
	 * estimation accuracy (although it is rule based in MDP) of the MDP robot.  
	 */
	string getRealRbtStMDP(string humanState);

	/**
	 * This function maps the human observations to the robot MDP states. For MDP to run, it needs the actual state information (unlike POMDP observations).
	 * So each MDP state is rule-based implemented according to the human observations. E.g. if human is not around (not OIR), then human needs help.
	 */
	string MapObservationsToMDP(string observation);
	
private:

	/*
	 * Calling the task manager service for observation status update
	 * /task_manager/observation_update is a rosservice by task manager agent to get information on
	 * the new human state, action, time of exection etc.
	 */
	ros::ServiceClient ObsUpdater;

	/*
	 * ROS services opened by MORSE to control human actions
	 */
	ros::ServiceClient is_ov; // ACTION: LOOK AROUND
	ros::ServiceClient is_oir; // SITUATION: IS HUMAN DETECTED?
	//ros::ServiceClient is_ho  = nh.serviceClient<std_srvs::Trigger>("/human/is_ho");
	ros::ServiceClient is_a0;  // ACTION: GRASP ATTEMPT 
	ros::ServiceClient is_a2;  // ACTION: IDLE
	ros::ServiceClient is_a4;  // ACTION: WARN THE ROBOT

	/*
	 * Advertised service. See their methods for the functionality of the services
	 */
	// ros::ServiceServer server = node_handle.advertiseService(service_name, pointer_to_callback_function);
	ros::ServiceServer action_server;
	ros::ServiceServer new_state__server;
	ros::ServiceServer reset_scenario;
	
	/// ROS subscribers (traySensor_subs) subscribes to the proximity sensors on the trays in the MORSE env.
	ros::Subscriber traySensor_subs;

	/// Ros time instance to record the time when the tray sensor message has been changed. 
	/// This is triggered when a package is put or removed from the tray, means success or failure or a new task in the scenario
	ros::Time tray_msg_stamp;

	/// A ROS timer to track human task !
	ros::Timer task_timer;

	/// Flags to control the information communication and variables for task and human-robot states tracking 
	bool prevStIsInitSt = true; 
	bool preventOneLoop = true; 
	bool humanLoop = false;
	int FailedStCounter = 0;
	int EvaluateStCounter = 0;
	int TiredStCounter = 0;
	int NoAttentionCounter = 0;

	int humanidle_counter = 0;
	int humanfail_counter = 0;

	bool ipd_sensor = false;	// inspected product detector sensor
	bool upd_sensor = false;	// uninspected product detector sensor

	bool humanAttempted = false; // for reactive model to state human attempted to grasp (take action) and track if success occured afterwards. Otherwise take over!
	int human_task_time = 0; // this counts every second that human spends when the task is assigned to him
	bool humanTrustsRobot;

	/// Variables that hold the information of human and robot action and states to be processed here and informed to the task manager
	string humanType; // beginner, expert
	string humanMood; // for beginner: stubborn, distracted, thinker, tired; for expert: normal, tired
	string robotType; // either proactive or reactive
	string whoIsAssigned;
	string whoSucceeded = "";
	string real_robot_state_name = "";
	string prev_robot_observation_pomdp = ""; // only for proactive model
	string prev_observables = "0";
	string prev_robot_state = ""; // for both reactive and proactive
	string prev_real_robot_state = ""; // for both reactive and proactive
	string prev_human_state = "";

	/// Task counter: Used to regulate human actions (failing, known as attempting_grasp, more often as more tasks assigned)
	int task_number_counter = 0;
};

#endif /* HRC_ROS_SRC_OBSERVATIONAGENT_H */
