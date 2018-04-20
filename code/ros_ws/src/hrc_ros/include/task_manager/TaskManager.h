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

#include <hrc_ros/TraySensor.h>

#include <boost/property_tree/json_parser.hpp>


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
	 * Rosservice is called /task_manager/new_scenario_request
	 * This service is manually called from the terminal to initiate a new scenario: human, robot and obs agents are reset,
	 * a package arrives conveyor runs and stops in the middle of the human and the robot. 
	 */
	bool initiateScenario(hrc_ros::InitiateScenarioRequest &req, hrc_ros::InitiateScenarioResponse &res);

	/**
	 * Rosservice is called /task_manager/reset_task
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
	
	/// this holds every iteration the state the robot should estimate correctly (for accuracy check)
	std::string realRobotState = "";
	/// beginner, expert: For now by default it is beginner. Information comes from task manager. This can be modified under configs/scenario_config.json 
	std::string human_expert = "";
	/// for beginner: stubborn, distracted, thinker, tired. Information comes from task manager. This can be modified under configs/scenario_config.json
	std::string human_mood = "";
	/// In our applications, for now, it is always kept TRUE
	std::string human_trust = "";
	
	/// Counts the steps taken by either human or robot within one task (each action decision is a step)
	int step_counter = 0;
	/// The time a task takes. This is counted by the ros timer event, increased in every second
	int task_time = 0;
	/// The number (also as ID) of the task assigned.
	int task_number = 0;

};

#endif /* HRC_ROS_SRC_TASKMANAGER_H */
