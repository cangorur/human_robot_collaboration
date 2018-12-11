/*
 * TestAgent.h
 *
 *  Created on: 04.12.2018
 * 
 *  Author: Elia Kargruber
 *      
 */



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
#include <hrc_ros/InformTrayUpdate.h> 
#include <hrc_ros/InformActionRecognized.h>

#include <hrc_ros/TraySensor.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include "std_msgs/String.h"
#include <helper_functions/Json_parser.h>


class TestAgent {
public:
	TestAgent();
	virtual ~TestAgent();


public: 

	void ReceiveSuccessStatusObserved(const hrc_ros::SuccessStatusObserved &msg);

	bool issue_tray_update(int current_object_int, int current_tray_int);
	bool issue_action_update(std::string action,int human_detected, int human_looking_around);


	// global variables that need to be accessible

	bool success_received = false;
	std::string subtask_success_status = "uninitialized";
	int task_cnt_received = 0; 
	int subtask_cnt_received = 0; 
	std::string Despot_global_task_status = "uninitilized";


private:

	/**
	 * Initialize service and topic handlers.
	 * @param req Request object
	 * @param res Response object
	 * @return True if initialization was successful
	 */
	void initialize();






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


	void ReceiveTaskManageTaskStatus(const hrc_ros::TaskState &msg);
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

	 ros::Subscriber sucessStatus_sub;
	 ros::Subscriber task_manag_task_status_sub;
	 hrc_ros::InformTrayUpdate tray_srv;
	 hrc_ros::InformActionRecognized action_srv;
	 ros::ServiceClient tray_client;
	 ros::ServiceClient action_client;

	
	  

	//


	/// Below are the service objects for the advertised services. See "initiateScenario" above for the functionality
//	ros::ServiceServer scenarioRequestService;
	/// Below are the service objects for the advertised services. See "HumanStatusUpdater" above for the 
//	ros::Publisher taskStatusPublisher;


};


