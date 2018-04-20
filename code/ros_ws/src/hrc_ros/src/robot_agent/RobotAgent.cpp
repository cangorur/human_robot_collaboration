/*
 *  Created on: 19.04.2018
 *      Author: Orhan Can Görür
 *      Email: orhan-can.goeruer@dai-labor.de
 */

#include <ros/ros.h>

#include <stdlib.h> 				// for rand() and RAND_MAX
#include <string>
#include <robot_agent/RobotAgent.h>

RobotAgent::RobotAgent() {
	ros::NodeHandle nh("~");
	initialize();
}

RobotAgent::~RobotAgent() {
}

void RobotAgent::initialize() {
	ros::NodeHandle nh("~");

	/*
	 * Initializing ros services
	 */
	callTaskMang_inform = nh.serviceClient<hrc_ros::InformRobotToTaskMang>("/task_manager/robot_status_update");  // Client for the task manager service for the status update

	/// ROS services by MORSE to call for the robot actions	
	pointToObj = nh.serviceClient<std_srvs::Trigger>("/robot/point_to_obj");
	grasp = nh.serviceClient<std_srvs::Trigger>("/robot/grasp");
	cancelAction = nh.serviceClient<std_srvs::Trigger>("/robot/cancel_action");
	planForGrasp = nh.serviceClient<std_srvs::Trigger>("/robot/planning_for_grasping");

	reset_scenario = nh.advertiseService("/robot_agent/reset", &RobotAgent::resetScenario, this);

	ROS_INFO("Robot agent is created !");
	update();
}

bool RobotAgent::resetScenario(hrc_ros::ResetRobotROSRequest &req,
		hrc_ros::ResetRobotROSResponse &res) {
	robot_action_taken = "";
	robot_belief_state = "";
	robot_real_state = "";
	robot_immediate_reward = "";
	robot_total_disc_reward = "";
	initial_state_received = true;
	newstate_info_received = false;
	action_info_received = false;
	
	ROS_INFO("ROBOT ROS: Reset!");
	res.success = true;
	return true;
}

void RobotAgent::update() {
	
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	srand (time(NULL));
	//WebSocket (WS)-server at port 8080 using 1 thread
	server.config.port = port;

	auto& echo=server.endpoint["^/?$"];
	
	echo.on_open=[](shared_ptr<WsServer::Connection> connection) {
	//cout << "Server: Opened connection " << (size_t)connection.get() << endl;
		ros::spinOnce(); // TODO: intended to catch the reset service call
	};
	
	echo.on_message=[&](shared_ptr<WsServer::Connection> connection, shared_ptr<WsServer::Message> message) {
		ros::spinOnce(); // TODO: intended to catch the reset service call
		string message_receive=message->string();
		
		cout << "Robot ROS Server: Message received: \"" << message_receive << "\"" << endl;
		
		string message_send = "None";
		auto send_stream=make_shared<WsServer::SendStream>();
		*send_stream << message_send;
		server.send(connection, send_stream, [](const SimpleWeb::error_code& ec){
			if(ec) {
				cout << "Server: Error sending message. " <<
						"Error: " << ec << ", error message: " << ec.message() << endl;
			}
		});
			//cout << "Server: Sending message \"" << message_send <<  "\" to MDP"<< endl;
		
		// interprete action and state
		std::size_t index = message_receive.find(",");
		string robot_action = message_receive.substr(0, index);
		
		string remainder = message_receive.substr(index+1);
		index = remainder.find(",");
		string robot_state = remainder.substr(0, index);
		
		remainder = remainder.substr(index+1);
		index = remainder.find(",");
		string immediate_reward = remainder.substr(0, index);
		
		remainder = remainder.substr(index+1);
		index = remainder.find(",");
		string total_disc_reward = remainder.substr(0, index);
		
		// EVERY MESSAGE EITHER IS A ROBOT ACTION BELIEF UPDATE OR ROBOT REWARD UPDATE
		// ROBOT BELIEF UPDATE RECEIVED. INFORMING ABOUT BOTH ACTION AND THE BELIEF STATE (either initial or belief state)
			
		if (robot_action != "-1"){
		
			std_srvs::Trigger::Request req;
			std_srvs::Trigger::Response resp;

			action_taken_time = ros::Time::now();

			bool success;
			if (robot_action == "0") {	// idle
				//Putting the actual name of the action to inform the task manager
				robot_action = "idle";
				success = true;
				ROS_INFO_STREAM("Current action is robot staying idle...");
			}
			else if (robot_action == "1") {	// grasp
				robot_action = "grasp";
				success = grasp.call(req, resp);
				ROS_INFO_STREAM("Current action is robot grasping...");
			}
			else if (robot_action == "2") {	// cancel all actions
				robot_action = "cancel all actions";
				success = cancelAction.call(req, resp);
				ROS_INFO_STREAM("Current action is robot canceling all actions...");
			}
			else if (robot_action == "3") {	// point to object
				robot_action = "point to remind";
				success = pointToObj.call(req, resp);
				ROS_INFO_STREAM("Current action is robot pointing to object...");
			}
			else if (robot_action == "4"){
				robot_action = "planning for grasping";
				ROS_INFO_STREAM("Current action is robot planning for grasping...");
				success = planForGrasp.call(req, resp);
				success = true;
			}
			else {
				success = true;
				//ROS_INFO_STREAM("Current action is robot doing nothing...");
			}
			ROS_INFO("ROBOT ROS: robot in belief state: %s took action: %s", robot_state.c_str(), robot_action.c_str());
			
			// Global variables are set below to be sent to task manager
			robot_action_taken = robot_action;
			robot_belief_state = robot_state;
			action_info_received = true; // raise the action received flag			
		}
		
		else{
			ROS_INFO("ROBOT ROS: robot received (prev step) reward: %s from the real state: %s | Total Disc.Reward: %s", 
										immediate_reward.c_str(), robot_state.c_str(), total_disc_reward.c_str());
			// global variables are assigned ! These are for sending the status later to the task manager
			robot_immediate_reward = immediate_reward;
			robot_total_disc_reward = total_disc_reward;
			robot_real_state = robot_state;
			newstate_info_received = true;
		}
		
		// First action and belief state is received, then the real state (the real of the belief). After both, inform the task mang. 
		if ((action_info_received && newstate_info_received) || initial_state_received){
			
			hrc_ros::RobotUpdateMsg update_msg;
			
			update_msg.stamp_robot_update = ros::Time::now();
			update_msg.action_taken_time = action_taken_time;
			update_msg.robot_action_taken = robot_action_taken; // took action in the belief state
			update_msg.robot_belief_state = robot_belief_state; // belief state
			
			if (initial_state_received){
				update_msg.robot_real_state = robot_belief_state; // in initial state robot mdp/pomdp sends the init state directly
				initial_state_received = false;
			}else{
				update_msg.robot_real_state = robot_real_state; // new state
				action_info_received = false;
				newstate_info_received = false;
			}
			
			update_msg.immediate_reward = robot_immediate_reward; // this reward is received from the real state (prev action taken)
			update_msg.total_disc_reward = robot_total_disc_reward;
			
			hrc_ros::InformRobotToTaskMang::Request robotUpdateReq;
			hrc_ros::InformRobotToTaskMang::Response robotUpdateRes;
			
			robotUpdateReq.robot_update = update_msg;
			callTaskMang_inform.call(robotUpdateReq, robotUpdateRes);
		}
	
	};
	//See RFC 6455 7.4.1. for status codes
	
	//See http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html, Error Codes for error code meanings
	echo.on_error=[](shared_ptr<WsServer::Connection> connection, const SimpleWeb::error_code& ec) {
		ros::spinOnce(); // TODO: intended to catch the reset service call
		cout << "Server: Error in connection " << (size_t)connection.get() << ". " << 
				"Error: " << ec << ", error message: " << ec.message() << endl;
	};

	server.start();
}