/*
 * TaskPlanner.cpp
 *
 *  Created on: 25.08.2017
 *      Author: Orhan Can Görür
 *      Email: orhan-can.goeruer@dai-labor.de
 */
#ifndef HRC_ROS_SRC_HRCTASKMANAGER_H_
#define HRC_ROS_SRC_HRCTASKMANAGER_H_

#include <ros/ros.h>
#include <hrc_ros/InitiateScenario.h>
#include <hrc_ros/InformHumanToTaskMang.h>
#include <hrc_ros/InformObsToTaskMang.h>
#include <hrc_ros/InformRobotToTaskMang.h>
#include <std_srvs/Trigger.h>

#include <hrc_ros/TraySensor.h>

#include <boost/property_tree/json_parser.hpp>


class HRCTaskManager {
public:
	HRCTaskManager();
	virtual ~HRCTaskManager();

private:
	/**
	 * Initialize service handler.
	 * @param req Request object
	 * @param res Response object
	 * @return True if initialization was successful
	 */
	void initialize();
	
	//void taskStateUpdateEvent(const ros::TimerEvent &e);
	
	bool initiateScenario(hrc_ros::InitiateScenarioRequest &req, hrc_ros::InitiateScenarioResponse &res);
	bool ResetTask(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
	
	bool packageGenerator();
	
	bool HumanStatusUpdater(hrc_ros::InformHumanToTaskMangRequest &req, hrc_ros::InformHumanToTaskMangResponse &res);
	bool ObsUpdater(hrc_ros::InformObsToTaskMangRequest &req, hrc_ros::InformObsToTaskMangResponse &res);
	bool RobotStatusUpdater(hrc_ros::InformRobotToTaskMangRequest &req, hrc_ros::InformRobotToTaskMangResponse &res);
	
	void TaskFinishTimer(const ros::TimerEvent&);
	void ReceiveTraySensors(const hrc_ros::TraySensor &msg);
	/**
	 * Get current storage state from the storage management. Takes a json ptree and returns it back with storage info
	 * @return Storage state
	 */
	//TODO: how to return a json object !
	//boost::property_tree::ptree GetStateOfStorages(boost::property_tree::ptree message_pt);
	
	
	/**
	 * Below are the callbacks for the subcribed topics
	void receiveRobot1Battery(const std_msgs::Float32& msg);
	*/
	
private:

	/* 
	 * Subscriber object for several rostopics
	ros::Subscriber storageUpdateSub;
	*/
	
	
	/*
	 * Below are the client objects for the ros services called
	*/
	ros::ServiceClient humanReset;
	ros::ServiceClient robotReset;
	ros::ServiceClient humanROSReset;
	ros::ServiceClient obsROSReset;
	ros::ServiceClient robotROSReset;
	ros::ServiceClient conveyorOnOff;
	ros::ServiceClient moveNewPackage;
	
	/*
	 * Below are the service objects for the advertised services
	 */
	ros::ServiceServer scenarioRequestService;
	
	ros::ServiceServer HumanUpdateService;
	ros::ServiceServer ObsUpdateService;
	ros::ServiceServer RobotUpdateService;
	ros::ServiceServer resetTaskService;
	
	/*
	 * Task status update timer
	 */
	ros::Timer taskFinishTimer;
	
	ros::Publisher taskStatusPublisher;
	
	ros::Subscriber traySensor_subs; // SUBSCRIBE: TRAY SENSORS
	
	std::string realRobotState = ""; // this holds every iteration the state the robot should estimate correctly (for accuracy check)
	std::string human_expert = "";
	std::string human_mood = "";
	std::string human_trust = "";
	
	int step_counter = 0;
	int task_time = 0; // this is counted by the ros timer event, increased in every second
	int task_number = 0;
	/*
	* Several variables
	double robotBatteries_arr[8] = {100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0}; //for 8 different robots
	*/

};

#endif /* HRC_ROS_SRC_HRCTASKMANAGER_H_ */
