/*
 * TaskPlanner.cpp
 *
 *  Created on: 25.08.2017
 *  Modified on: 19.04.2018
 *      Author: Orhan Can Görür
 *      Email: orhan-can.goeruer@dai-labor.de
 */

#ifndef HRC_ROS_SRC_TASKMANAGER_H
#define HRC_ROS_SRC_TASKMANAGER_H

#include <ros/ros.h>
#include <hrc_ros/InitiateScenario.h>
#include <hrc_ros/InformHumanToTaskMang.h>
#include <hrc_ros/InformObsToTaskMang.h>
#include <hrc_ros/InformRobotToTaskMang.h>
#include <std_srvs/Trigger.h>

#include <hrc_ros/TaskState.h>
#include <hrc_ros/ObsUpdateMsg.h>
#include <hrc_ros/HumanUpdateMsg.h>
#include <hrc_ros/RobotUpdateMsg.h>
#include <hrc_ros/ResetHumanROS.h>
#include <hrc_ros/ResetObsROS.h>
#include <hrc_ros/ResetRobotROS.h>
#include <hrc_ros/MoveNewPackage.h>
#include <hrc_ros/SelectPolicy.h>
#include <hrc_ros/PolicySelectorBPR.h>
#include <hrc_ros/SuccessStatusObserved.h>

#include <hrc_ros/TraySensor.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include "std_msgs/String.h"
#include <helper_functions/Json_parser.h>


class TaskManager {
public:
	TaskManager();
	virtual ~TaskManager();

private:

	/**
	 * Initialize service and topic handlers.
	 * @param req Request object
	 * @param res Response object
	 * @return True if initialization was successful
	 */
	void initialize();

	//void taskStateUpdateEvent(const ros::TimerEvent &e);

	// ================Advertised Services=======================
	/**
	 * Rosservice is called /task_manager_IE/new_scenario_request
	 * This service is manually called from the terminal to initiate a new scenario: human, robot and obs agents are reset,
	 * a package arrives conveyor runs and stops in the middle of the human and the robot. After a task has been executed the task manager
	 * node will call this function again. A whole task set (consisting of multiple individual placing tasks) is implemented here. 
	 */
	bool initiateScenario(hrc_ros::InitiateScenarioRequest &req, hrc_ros::InitiateScenarioResponse &res);

	/**
	 * Rosservice is called /task_manager_IE/reset_task
	 * This service is manually called from the terminal to reset a scenario: human, robot and obs agents are reset,
	 * a package arrives conveyor runs and stops in the middle of the human and the robot.
	 */
	bool ResetTask(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

	/**
	 * Not implemented currently. In this application we only change the location of one package. No new generation.
	 * That is why when a new task is received the previous package processed is reput on the conveyor as a reprocess
	 * @todo callout the package generation service: it is initiated by package_generator.py
	 */
	bool packageGenerator();

	/**
	 * Rosservice call: /task_manager/human_status_update
	 * It is called by the human_agent to provide last information
	 */
	bool HumanStatusUpdater(hrc_ros::InformHumanToTaskMangRequest &req, hrc_ros::InformHumanToTaskMangResponse &res);

	/**
	 * Rosservice call: /task_manager/observation_update
	 * It is called by the observation_agent to provide last information
	 */
	bool ObsUpdater(hrc_ros::InformObsToTaskMangRequest &req, hrc_ros::InformObsToTaskMangResponse &res);

	/**
	 * Rosservice call: /task_manager/robot_status_update
	 * It is called by the robot_agent to provide last information
	 */
	bool RobotStatusUpdater(hrc_ros::InformRobotToTaskMangRequest &req, hrc_ros::InformRobotToTaskMangResponse &res);

	/// =======================================

	///================Rostopic Callbacks========================
	/**
	 * Calculates the time a task takes
	 */
	void TaskFinishTimer(const ros::TimerEvent&);

	/**
	 * It receives the tray sensor information, a proximity sensor placed on the trays. It is received from MORSE which is
	 * implemented as a rostopic to constantly send the sensory info. It is pushed only when the value changes.
	 */
	void ReceiveTraySensors(const hrc_ros::TraySensor &msg);
	// =======================================

	void ReceiveTraySuccessStatus(const hrc_ros::SuccessStatusObserved &msg);
	// =======================================

	/// =========== Other Methods============
	/**
	 * This function starts a new task if current one has finished,
	 * and the human, the robot and the observation agents have obs_has_informed
	 * their status.
	 */
	void CheckToStartNewTask(void);
  /// =======================================

private:


	/// Below are the client objects for the ros services called
	ros::ServiceClient humanReset;
	/// Below are the client objects for the ros services called
	ros::ServiceClient robotReset;
	/// Below are the client objects for the ros services called
	ros::ServiceClient humanROSReset;
	/// Below are the client objects for the ros services called
	ros::ServiceClient obsROSReset;
	/// Below are the client objects for the ros services called
	ros::ServiceClient robotROSReset;
	/// Below are the client objects for the ros services called
	ros::ServiceClient conveyorPrinterOnOff;
	/// Below are the client objects for the ros services called
	ros::ServiceClient conveyorAssembly1OnOff;
	/// Below are the client objects for the ros services called
	ros::ServiceClient conveyorAssembly2OnOff;
	/// Below are the client objects for the ros services called
	ros::ServiceClient moveNewPackage;
	/// Below is the client object for the policy evaluator service call
	ros::ServiceClient policyRetrieve;
	/// Contextual-MAB Client
	ros::ServiceClient cmab_call;
	/// Bayesian Policy Selector Client
	ros::ServiceClient bpr_call;



	/// Below are the service objects for the advertised services. See "initiateScenario" above for the functionality
	ros::ServiceServer scenarioRequestService;
	/// Below are the service objects for the advertised services. See "HumanStatusUpdater" above for the functionality
	ros::ServiceServer HumanUpdateService;
	/// Below are the service objects for the advertised services. See "ObsUpdater" above for the functionality
	ros::ServiceServer ObsUpdateService;
	/// Below are the service objects for the advertised services. See "RobotStatusUpdater" above for the functionality
	ros::ServiceServer RobotUpdateService;
	/// Below are the service objects for the advertised services. See "ResetTask" above for the functionality
  ros::ServiceServer resetTaskService;

	/*
	 * Task status update timer
	 */
	ros::Timer taskFinishTimer;

	/*
	 * TASK STATE IS PUBLISHED FROM: /task_manager/task_status --> echo this topic to visualize the info or use rosbag to save
	 * Task State: human states + actions, robot state + actions + rewards and general info are published as a ROS topic
	 */
	ros::Publisher taskStatusPublisher;

	ros::Subscriber traySensor_subs; // SUBSCRIBE: TRAY SENSORS

	ros::Subscriber trayObservation_success_status_subs; // subscribe to the success or fail update by observation agent

	/// this holds every iteration the state the robot should estimate correctly (for accuracy check)
	std::string realRobotState = "";
	/// beginner, expert: For now by default it is beginner. Information comes from task manager. This can be modified under configs/scenario_config.json
	std::string human_expertise = "";
	/// distracted, tired, nontired. Information comes from task manager. This can be modified under configs/scenario_config.json
	std::string human_mood = "";
	/// collaborative, noncollaborative. Defines if the human is ok with collaborating with the robot. Set under configs/scenario_config.json
	std::string human_collaborativeness = "";
	/// In our applications, for now, it is always kept TRUE
	std::string human_trust = "";
	/// human_type = human_expertise + "_" + human_mood + "_" + human_collaborativeness + ".POMDPx" // OR in evaluating mode changing every iteration
	std::string human_type = "";

	/// proactive, reactive. Can be set under configs/sceneario_config
	std::string robot_AItype = "";
	/// beginner, collaborative, noncollaborative, distracted, tired, nontired. Select a robot model designed for a specific human type. Set under configs/scenario_config.json
	std::string robot_forHuman = "";
	/// robot_model = robot_AItype + "_" + robot_forHuman + "".pomdx"
	std::string robot_model = "";
	/// holds the human type belief distr of the robot
	std::vector<float> robot_belief;


	/// Counts the sub_task within task(set). Each subtask is an individual placing tasks
	/// Note: The counter starts at 1!!! 
	int subtask_counter = 1; 
	/// Counts the task number that should currently be executed | note: the first task is taks 1!!! 
	
	int task_counter = 1; 
	/// Counts the steps taken by either human or robot within one task (each action decision is a step)
	int step_counter = 0;
	/// The time a task takes. This is counted by the ros timer event, increased in every second
	int task_time = 0;
	/// The number (also as ID) of the task assigned.
	int task_number = 0;
	// The number of the warnings from human
	int warning_number = 0;
	// The number of times robot interfered state occured
	int robot_interfered_number = 0;

	/// Flags to see if all agents informed their status (awareness) to start a new task afterwards
	bool human_has_informed = false;
	bool robot_has_informed = false;
	bool task_has_finished = false;
	bool task_stuck_flag = false; // This is to save the system if a task is stuck (despot models are not informed somehow)


	int subtask_number_current_task = 100; 
	boost::property_tree::ptree testscenario_pt;
	task_set current_task_set;
	global_task_config current_global_task_config; 
	int task_quantity_scenario = 5; 

	// variables for statistic 
	int task_fail_statisctics = 0; 
	int task_success_statistics = 0;
	int subtask_success_statistics = 0; 
	int subtask_fail_statistics	   = 0; 
	std::string final_state_statistics = "uninitialized"; 
};

#endif /* HRC_ROS_SRC_TASKMANAGER_H */
