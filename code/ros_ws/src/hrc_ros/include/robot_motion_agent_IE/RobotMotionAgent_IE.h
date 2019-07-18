/*
 *  RobotMotionAgent.cpp
 *
 *  Created on: 17.01.2019
 *      Author: Elia Jona Kargruber
 *      Email: kargruber@campus.tu-berlin.de
 */

#ifndef HRC_ROS_SRC_ROBOTPLANNER_H
#define HRC_ROS_SRC_ROBOTPLANNER_H

#include <ros/ros.h>
// include <package_name/service_type_name.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <hrc_ros/InformRobotToTaskMang.h>
#include <hrc_ros/RobotUpdateMsg.h>
#include <hrc_ros/ResetRobotROS.h>
#include <std_msgs/Bool.h>

// dobot specific services
#include <hrc_ros/ContPickAndPlace.h>
#include <hrc_ros/InitDobotArmApp.h>
#include <hrc_ros/SetQueuedCmdStopExec.h>
#include <hrc_ros/SetQueuedCmdForceStopExec.h>
#include <hrc_ros/OneTimePickAndPlace.h>
#include <hrc_ros/SetPTPCoordinateParams.h>  //service to set speed and velocities of dobot
#include <hrc_ros/SetQueuedCmdStartExec.h>
#include <hrc_ros/SetPTPCmd.h> // for the gotoPointApp service
#include <hrc_ros/SetEndEffectorSuctionCup.h>
#include <hrc_ros/SetQueuedCmdClear.h>
#include <hrc_ros/SimplePickAndPlace.h>
#include <hrc_ros/InOprConveyorControl.h>

// websocket and stream includes
#include "simple_web_socket/server_ws.hpp"
#include <iostream>
#include <fstream>

using namespace std;
typedef SimpleWeb::SocketServer<SimpleWeb::WS> WsServer;

class RobotMotionAgent {
public:
	RobotMotionAgent();
	virtual ~RobotMotionAgent();

	// file and filename to store the results from the pomdp evaluation
	std::ofstream test_result_file;
	string filename_robot_pomdp_eval;

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

	bool executeDobotMotionTest( std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

	// TODO check if still needed -> remove otherwise
	/*static void dobot_pointing_thWorker();
	static void dobot_planning_thWorker() ;
	static void dobot_warning_thWorker() ;
	static void dobot_grasping_thWorker() ;

	std::thread pointing_thread;
	std::thread planninging_thread;
	std::thread warning_thread;
	std::thread grasping_thread();
	*/


private:

// **** publishers to publish the DOBOT command to the worker nodes ****
	ros::Publisher dobot_grasp_pub;
	ros::Publisher dobot_cancel_pub;
	ros::Publisher dobot_plan_pub;
	ros::Publisher dobot_idle_pub;
	ros::Publisher dobot_point_pub;

// **** publisher to publish the action the robot took -> this is recorded

	ros::Publisher robot_action_pub;
	/*
	 * /task_manager/robot_status_update is a rosservice by task manager agent to get information on
	 * the new robot state, action, time of exection, rewards collected and the belief state etc.
	 */
	ros::ServiceClient callTaskMang_inform;



// **** services to command the DOBOT API ****
	/*
	 * /dobot_arm_app/simplePickAndPlace is a service to command DOBOT to pick an object and put it directly into a tray.
	 *  No detour for the colour sensor is made.
	 */
	ros::ServiceClient Dobot_SimplePickAndPlace;

	/*
	 * /dobot_arm_app/setPTPCoordinateParamsApp is a service to set velocities and accelerations of the DOBOT
	 * Note: the include is called SetPTPCoordinateParams.h
	 */
	ros::ServiceClient Dobot_SetPTPCoordinateParams;

	/*
	 * /dobot_arm_app/init service initiates the Dobot arm app
	 */
	ros::ServiceClient Dobot_InitDobotArmApp;

	/*
	 * /dobot_arm_app/setQueuedCmdForceStopExecApp  forces the Dobot to stop immediately (stop all actions)
	 */
	ros::ServiceClient Dobot_SetQueuedCmdForceStopExec;

	/*
	 * /dobot_arm_app/setQueuedCmdStopExecApp service initiates stops the execution of the command queue without interrupting the currently executed action
	 */
	ros::ServiceClient Dobot_SetQueuedCmdStopExec;

	/*
	 * /dobot_arm_app/setQueuedCmdStartExecApp restarts the execution of DOBOT commands. This has to be called after the execution has been stopped in order to continue normal DOBOT operation.
	 */
	ros::ServiceClient Dobot_SetQueuedCmdStartExec;

	/*
	 * /dobot_arm_app/setQueuedCmdClearApp  clears the command queue and deletes all commands. Afterwards you will have a clean DOBOT command queue.
	 */
	ros::ServiceClient Dobot_SetQueuedCmdClear;


	/*
	 * /dobot_arm_app/setEndEffectorSuctionCupApp   let's you enable or disable DOBOT's suctionCup | set the config flag to 1 in order to be able to modify it.
	 */
	ros::ServiceClient Dobot_SetEndEffectorSuctionCup;


	/*
	 * /dobot_arm_app/gotoPointApp commands DOBOT to go to a specified point
	 * Note: the include is called SetPTPCmd.h
	 */
	ros::ServiceClient Dobot_gotoPoint;

	/*
	 * /dobot_arm_app/contPickAndPlace  is a service that will command DOBOT to continously pick objects and place them in the containers.
	 * The service is mainly used for demonstration purposes.
	 *
	 */
	ros::ServiceClient Dobot_ContPickAndPlace;

	/*
	 * /dobot_arm_app/oneTimePickAndPlace   is a service that will command DOBOT grasp an object once and put it in a specified container.
	 * A detour to the colour sensor is made. The service is mainly used for demonstration purposes.
	 *
	 */
	ros::ServiceClient Dobot_oneTimePickAndPlace ;



	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient pointToObj;
	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient grasp;
	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient cancelAction;
	/// ROS services opened by MORSE to control human actions
	ros::ServiceClient planForGrasp;


	/*
	 * /robot_motion_agent/motionTest is a rosservice by task manager agent to get information on
	 *
	 */
	ros::ServiceServer DoDobotMotionTestService;


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
	/// Robot type variable: reactive or proactive
	string robotType = "";
	/// a flag for stating if it is the initial state from DESPOT
	bool init_state = false;

};

#endif /* HRC_ROS_SRC_ROBOTAGENT_H */
