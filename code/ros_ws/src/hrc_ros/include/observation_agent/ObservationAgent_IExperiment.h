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
#include <hrc_ros/InformObsToTaskMangIE.h>
#include <hrc_ros/ResetObsROS.h>
#include <hrc_ros/InformTrayUpdate.h>
#include <hrc_ros/InformActionRecognized.h>
#include <hrc_ros/SuccessStatusObserved.h>
#include <hrc_ros/HeadGestureMsg.h>
#include <hrc_ros/ObjectGraspColourMsg.h>

#include <hrc_ros/RequestSuccessCriteria.h>
#include "std_msgs/String.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

#include "simple_web_socket/client_ws.hpp"

#include <helper_functions/Json_parser.h>

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
	 * This Function is called at the beginning of each task.
	 * Main functions are
	 * 1) Reset global variables ( statistics , counters, start and reset timers
	 * 2) Read in TaskConfiguration -> new task rules needed to calculate success and failure
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
	 * Timer event to track the subtask duration. Used to calculate discounted rewards
	 */
	void SubTaskTimer(const ros::TimerEvent&);

	/**
	 * Timer event created to trigger a decision even if no action has been received for a long time
	 */
	void DecisionTimer(const ros::TimerEvent&);


	/**
	 * Callback function to catch the updated tray sensor information
	 */
	void ReceiveTraySensors(const hrc_ros::TraySensor &msg);

	// ---- Operation based functions. Mostly for reactive robot's rule based decisions --- //

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

    /** ################  functions for recognition update in Interaction Experiment
	 *
	 *
	 */
	//void IEaction_recognized_update_to_obs_map(hrc_ros::TrayUpdateCamera &msg);

	bool IEaction_to_obs_Map();

	//void IEtray_update_to_obs_map(const hrc_ros::TrayUpdateCamera &msg);
	bool IE_receive_tray_update(hrc_ros::InformTrayUpdate::Request &req,hrc_ros::InformTrayUpdate::Response &res);


	/**
	 * This advertised rosservice is called "/observation_agent/request_success_criteria". It is called by dobot_worker_agent, if it is commanded to grasp and needs to know the current
	 * success criteria. The success criteria is use to let dobot know where to put the current object.
	 * The success criteria will be returned by the service handler afterwards.
	 *
	 */
	bool IE_request_success_criteria(hrc_ros::RequestSuccessCriteria::Request &req, hrc_ros::RequestSuccessCriteria::Response &res);



	bool IE_receive_actionrecognition_update(hrc_ros::InformActionRecognized::Request &req, hrc_ros::InformActionRecognized::Response &res);

	// subscriber callback that receives the head gesture
	void ReceiveHeadGesture(const hrc_ros::HeadGestureMsg &msg);

	// subscriber callback that receives the color of the object taken by the human
	void ReceiveObjectTracked(const hrc_ros::ObjectGraspColourMsg &msg);

	/**  // TODO delete once new function works
	 * This advertised rosservice is called "/observation_agent/inform_new_human_state". It is called by human_agent after it receives the human state
	 * information from despot MDP human decision-making process. Then the (actual) state information is mapped to the robot states with the functions below
	 * and provided to the despot MDP / POMDP robot (thru web client) as the current state information. In MDP it is used as the decision-making for the next action,
	 * in POMDP it is used for calculating the immediate rewards.
	 */
	bool humanSt_to_robotSt_Map(hrc_ros::InformHumanState::Request &req, hrc_ros::InformHumanState::Response &res);


	/**
	 * This advertised rosservice is called "/observation_agent/inform_new_human_state". It is called by human_agent after it receives the human state
	 * information from despot MDP human decision-making process. Then the (actual) state information is mapped to the robot states with the functions below
	 * and provided to the despot MDP / POMDP robot (thru web client) as the current state information. In MDP it is used as the decision-making for the next action,
	 * in POMDP it is used for calculating the immediate rewards.
	 *
	 * So far this whole behaviour is simulated in the tes_decision_trigger node
	 *
	 * Note: the real_human_state_observed actually denotes the human's state in the robot POMDP state space
	 */
    bool IE_humanSt_to_robotSt_Map(string real_human_state_observed);



	// ********************************** //

	/**
	 * Counting the human states of being tired, notattending, failed etc.
	 * This is for reactive robot's rule based decision-making mechanism
	 */
	void humanStCounter(string humanState);


	/**
	 * This function checks if a global success is reached, compiles the tray_status message for the task manager and publishes it.
	 * It is only called if the robot is grasping an object and informing a grasping_status.
	 */
	void inform_trayupdate_to_taskmanager(void);


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
	 * This function gets the observable in the DESPOT format [0-11] and calculates an immediate and discounted reward for the interaction experiment case.
	 * It also checks if robot has succeeded in subtask or failed due to a disrupt, which are not informed by the camera agent yet also being rewarded.
	 * NOT: The immediate_reward and discounted_reward are passed by reference
	 */
	void calculate_reward_IE(string obs, float &immediate_reward_out, float &discounted_reward_out);

	/**
	 * This function gets the human observation names and maps them into logic numbers (binaries, e.g. 100001).
	 * Refer to the google doc for the map: https://docs.google.com/spreadsheets/d/1jDDyNXrNnYsDy5L82CDVipNZQwEvev44tx2FqHkm2wE/edit#gid=2060522298
	 * ## subtask_status mapping:
	 *    ## 0 = ongoing
	 *    ## 1 = subtask_success
	 *    ## 3 = subtask_failure
	 * @todo: the observable names of ov and oir to be updated. Currently !OV = Looking Around and OIR = Human is detected
	 * @todo: Simplify the observation combinations if the reactive model responds almost better than the proactive one!
	 */
	string MapObservablesToObservations(bool ov, bool oir, bool a0, bool ipd, bool a4, bool a2, bool upd, int subtask_status);

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


	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient is_ov; // ACTION: LOOK AROUND
	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient is_oir; // SITUATION: IS HUMAN DETECTED?

	//ros::ServiceClient is_ho  = nh.serviceClient<std_srvs::Trigger>("/human/is_ho");

	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient is_a0;  // ACTION: GRASP ATTEMPT
	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient is_a2;  // ACTION: IDLE
	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient is_a4;  // ACTION: WARN THE ROBOT
	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient is_a0_failed;  // ACTION: GRASP ATTEMPTED BUT FAILED


	/// Advertised service. See their methods for the functionality of the services

	// ros::ServiceServer server = node_handle.advertiseService(service_name, pointer_to_callback_function);
    /// Advertised service. See their methods for the functionality of the services

	/// Advertised service. See their methods for the functionality of the services
	ros::ServiceServer IErequest_successcriteria_server ;
	/// Advertised service. See their methods for the functionality of the services
	ros::ServiceServer IEaction_recognition_server ;
	/// Advertised service. See their methods for the functionality of the services
	ros::ServiceServer IEtray_update_server;
	/// Advertised service. See their methods for the functionality of the services
	ros::ServiceServer new_state__server;
	/// Advertised service. See their methods for the functionality of the services
	ros::ServiceServer reset_scenario;


	ros::ServiceServer IE_new_state__server;

	/// ROS subscribers (traySensor_subs) subscribes to the proximity sensors on the trays in the MORSE env.
	ros::Subscriber traySensor_subs;
	/// Subscribes to the HeadGesture topic, this indicates the observable O4
	ros::Subscriber headGesture_subs;
	/// Subscribes to the HeadGesture topic, this indicates the observable O4
	ros::Subscriber objectTrack_subs;
	//// Ros publisher - is published after the tray status has bee received | bublishes the successs or fail status or a subtask
	ros::Publisher 	traySensor_success_pub;
	//// Ros publisher - is published after the tray status has bee received | used to inform human about the tray update, e.g. by sound ...
	ros::Publisher 	traySensor_notifyToHuman_pub;

	/// publishes the observation update -> this info is used for logging and analysis on system level
	ros::Publisher ObsUpdaterPub;


	/// Ros time instance to record the time when the tray sensor message has been changed.
	/// This is triggered when a package is put or removed from the tray, means success or failure or a new task in the scenario
	ros::Time tray_msg_stamp;

	/// A ROS timer to track the time of a subtask
	ros::Timer subtask_timer;
	int subtask_timer_tick =0; // counts the ticks of the subtask_timer -> used for discounted reward calculation
	int before_subtask_reset_tick = 0;

	/// A ROS timer to track human task !
	ros::Timer task_timer;
	/// A Timer that will trigger a decision if no action has been recognised for a certain amount of time
	ros::Timer decision_timer;

	/// Flags to control the information communication and variables for task and human-robot states tracking
	bool prevStIsInitSt = true;
	/// Flags to control the information communication and variables for task and human-robot states tracking
	bool preventOneLoop = true;
	/// Flags to control the information communication and variables for task and human-robot states tracking
	bool humanLoop = false;
	/// Flags to control the information communication and variables for task and human-robot states tracking
	int FailedStCounter = 0;
	/// Flags to control the information communication and variables for task and human-robot states tracking
	int EvaluateStCounter = 0;
	/// Flags to control the information communication and variables for task and human-robot states tracking
	int TiredStCounter = 0;
	/// Flags to control the information communication and variables for task and human-robot states tracking
	int NoAttentionCounter = 0;
	/// Flags to control the information communication and variables for task and human-robot states tracking
	int humanidle_counter = 0;
	/// Flags to control the information communication and variables for task and human-robot states tracking
	int humanfail_counter = 0;
	/// inspected product detector sensor
	bool ipd_sensor = false;
	/// uninspected product detector sensor
	bool upd_sensor = false;
    /// for reactive model to state human attempted to grasp (take action) and track if success occured afterwards. Otherwise take over!
	bool humanAttempted = false;
	/// this counts every second that human spends when the task is assigned to him
	int human_task_time = 0;
	/// Boolean to hold if human trusts the robot. This is inherently known to observation agent from the human model run by task manager
	/// In our applications, for now, it is always kept TRUE
	bool humanTrustsRobot;
	/// Flag to check if new observations should be ignored or a new decision to be triggered
	bool allowDecisionMaking = true;
	/// Robot succeeded in placing a package
	bool isRobotSucceed = false;
	/// Robot succeeded in placing a package
	bool isRobotFailed = false;

	// Variables that hold the information of human and robot action and states to be processed here and informed to the task manager
	/// beginner, expert: For now by default it is beginner. Information comes from task manager. This can be modified under configs/scenario_config.json
	string humanType;
	/// for beginner: stubborn, distracted, thinker, tired. Information comes from task manager. This can be modified under configs/scenario_config.json
	string humanMood;
	/// either proactive or reactive. Information comes from task manager. This can be modified under configs/scenario_config.json
	string robotType;
	/// holds the information of who is asigned with the task
	string whoIsAssigned;
	string whoSucceeded = "";
	string real_robot_state_name = "";
	/// To keep track on the prev and current observations. Used to decide human's actual state. only for proactive model
	string prev_robot_observation_pomdp = "";
	string prev_observables = "0";
	/// Previous robot state for reactive
	string prev_robot_state = "";
	/// Previous real robot state (the state the robot should have estimated to be in). Only for proactive robot.
	string prev_real_robot_state = "";
	/// Previous actual human state
	string prev_human_state = "";

	/// Task counter: Used to regulate human actions (failing, known as attempting_grasp, more often as more tasks assigned)
	int task_number_counter = 0;

	// global variables to store the last recognized human action and other observables
	// name in code  | observable number according to https://docs.google.com/spreadsheets/d/1gJoA5ltNewCgFDSOcUGdoqZcWzdyu6Id3xDJE6V_nDg/edit#gid=1272592695
	bool o4_ov		= true;				// O_4  Human is not looking around
	bool o3_oir		= false;				// O_3  Human is detected
	bool o5_a0		= false;				// O_5  grasping attempt
	bool o1_ipd		= false;				// O_1  task successs (processed product detected)
	bool o6_a4		= false;				// O_6  warning received
	bool o7_a2		= false; 				// O_7  Idle
	bool o2_upd		= false; 				// O_2	failure
	int  int_subtask_status = 0; 			// 0 = ongoing | 1 = success | 2 = fail  this is used to issue the subtask observables
	bool notO4_human_looking_around = false;

	// former values to calculate if update happened -> only update will trigger despot decision making
	bool o6_former = true;
	bool o7_former = true;
	bool o5_former = true;
	bool o3_former = true;
	bool o4_former = true;
	ros::Time former_time_stamp;


	// global variables to store the latest tray status and the latest recognized human action
	int tray_object_combination = 0;
		/*# R_tray: 	1 = Red object in Red tray
		# 				2 = Green object in Red tray
		#				4 = Blue  object in Red tray
		# G_tray: 		8 = Red object in Green tray
		#		...
		#		...
		# B_tray: 		64 = Red object in Blue tray
		#		...
		#		...
		*/

	int current_object = 0; 				// 1 = Red | 2 = Green | 3 = Blue
	int robot_current_object = 0;
	int graspped_object_data = 0;
	int prev_object = 4;


	// global variables to store the scenario data -> will be used to elaborate success and failure
	int true_tray_object_combination = 0;

	success_combo success_criteria_read;

	// counter that counts the subtasks that have been executed.
	/// Note: counter starts at 1!!!

	int task_counter = 0; // will be incremented to 1 when experiment starts
	int subtask_counter = 1;

	/// times and metrics for experiment
	ros::Time subtask_start_time;
	ros::Duration subtask_duration;
	double task_time_sumOfSubtasks_sec = 0.0; // seconds precision, this variable holds the sum of all measured subtask durations of a task in seconds precision



	// property tree to hold the testscenario information
	boost::property_tree::ptree testscenario_pt;
	global_task_config global_task_configuration_read;
	int current_subtask_quantity = 0;

	// ############# Variables used for experiment statistics and the calculation thereof ########
	// variables to calculate rewards in the interaction experiment
	float immediate_reward_IE = 0.0;
	float discounted_reward_IE = 0.0;
	float discount_factor = 0.98;
	double percentage_successful_subtasks = 0.0;
	// counters that count the successes and failures during a task -> used to determine global_success and global_fail
	int successful_subtasks = 0;
	int failed_subtasks 	= 0;
	int warnings_received_task = 0;
	int warnings_received_subtask = 0;
	double subtask_time_seconds = 0.0;
	string who_succeeded = "NOBODY"; // NOBODY, ROBOT, HUMAN
	int successful_tasks_cnt = 0;
	int failed_tasks_cnt = 0;
	double percentage_successful_tasks = 0.0;

	// global variables for interaction experiment
	bool experiment_started = false; //variable that is set true, once the experiment started. Some data will only be evaluated aftewards (e.g. tray updates and actions)



};

#endif /* HRC_ROS_SRC_OBSERVATIONAGENT_H */
