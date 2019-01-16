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
cout << "in the constructor" << endl; 
test_result_file << "\nThe file was closed in the destructor\n"; 
test_result_file.close();
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
	planForGrasp = nh.serviceClient<std_srvs::Trigger>("/robot/planning_for_motion");

	reset_scenario = nh.advertiseService("/robot_agent/reset", &RobotAgent::resetScenario, this);
	
	// opening results file with current date and time 
	time_t now = time(0);
	struct tm * now_tm = localtime(&now); 
	char date_buffer [80];
    strftime (date_buffer,80,"%Y-%m-%d_%R",now_tm); 
	filename_robot_pomdp_eval = string("/home/elia/master_thesis/catkin_ws/src/hrc_industry/code/results/POMDP_IE_tests/pomdp_robotagent_file_") + date_buffer + string(".csv");


	test_result_file.open (filename_robot_pomdp_eval,std::ios_base::app); 
	test_result_file << string("robot_action_taken") + "," + string("robot_belief_state") + "," + string("robot_real_state") + "," + string("robot_immediate_reward") + "," + string("robot_total_disc_reward") +  "\n";
	test_result_file.close();			


	ROS_INFO("[ROBOT AGENT] for POMDP evaluation created !");
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

	ROS_INFO("[ROBOT AGENT] Reset!");
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
		string message_received=message->string();

		ROS_INFO("[ROBOT AGENT] Server: Message received: %s", message_received.c_str());

		string message_send = "None";
		auto send_stream=make_shared<WsServer::SendStream>();
		*send_stream << message_send;
		server.send(connection, send_stream, [](const SimpleWeb::error_code& ec){
			if(ec) {
				cout << "[ROBOT AGENT] Server: Error sending message. " <<
						"Error: " << ec << ", error message: " << ec.message() << endl;
			}
		});
			//cout << "Server: Sending message \"" << message_send <<  "\" to MDP"<< endl;

		// interprete action and state
		std::size_t index = message_received.find(",");
		string robot_action = message_received.substr(0, index);

		string remainder = message_received.substr(index+1);
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
				// Global variables are set below to be sent to task manager
				robot_action_taken = "idle";
				success = true;
				ROS_INFO_STREAM("[ROBOT AGENT] Current action is robot staying idle...");
			}
			else if (robot_action == "1") {	// grasp
				// Global variables are set below to be sent to task manager
				robot_action_taken = "grasp";
				success = grasp.call(req, resp);
				ROS_INFO_STREAM("[ROBOT AGENT] Current action is robot grasping...");
			}
			else if (robot_action == "2") {	// cancel all actions
				// Global variables are set below to be sent to task manager
				robot_action_taken = "cancel all actions";
				success = cancelAction.call(req, resp);

				std_srvs::SetBool is_warned;
				is_warned.request.data = true;
				ros::ServiceClient informHuman = nh.serviceClient<std_srvs::SetBool>("/human_mc_sampler/robot_is_warned");
				informHuman.call(is_warned);
				ROS_INFO_STREAM("[ROBOT AGENT] Current action is robot canceling all actions...");
			}
			else if (robot_action == "3") {	// point to object
				// Global variables are set below to be sent to task manager
				robot_action_taken = "point to remind";
				success = pointToObj.call(req, resp);
				ROS_INFO_STREAM("[ROBOT AGENT] Current action is robot pointing to object...");
			}
			else if (robot_action == "4"){
				// Global variables are set below to be sent to task manager
				robot_action_taken = "planning for grasping";
				ROS_INFO_STREAM("[ROBOT AGENT] Current action is robot planning for grasping...");
				success = planForGrasp.call(req, resp);
				success = true;
			}
			else {
				success = true;
				//ROS_INFO_STREAM("Current action is robot doing nothing...");
			}
			ROS_WARN("[ROBOT AGENT] robot in belief state: %s took action: %s", robot_state.c_str(), robot_action_taken.c_str());

			// inform human about robots action via parameter
			// this is needed to let human know how robot acted. maybe placed in observation agent?
			ros::param::set("/current_robot_action", stoi(robot_action));
			int terminal_state = -1;
	    ros::param::get("/human_observable_state", terminal_state);
	    if(terminal_state != 1 && terminal_state != 2 && robot_action_taken == "grasp")
			// this means the robot has interfered by grasping (yet no success or fail has happened)
	    {
	        ROS_INFO("[ROBOT AGENT]: #### Robot action was : %s", robot_action_taken.c_str());
	        ros::param::set("/human_observable_state", 8);
	    }

			// Global variables are set below to be sent to task manager
			robot_belief_state = robot_state;
			action_info_received = true; // raise the action received flag
		}
		else{
			ROS_INFO("[ROBOT AGENT] Robot received (prev step) reward: %s from the real state: %s | Total Disc.Reward: %s",
										immediate_reward.c_str(), robot_state.c_str(), total_disc_reward.c_str());
			// global variables are assigned ! These are for sending the status later to the task manager
			robot_immediate_reward = immediate_reward;
			robot_total_disc_reward = total_disc_reward;
			robot_real_state = robot_state;
			newstate_info_received = true;
		}

		bool init_state = false;
		ros::param::get("/robot_initial_state", init_state);

		// First action and belief state is received, then the real state (the real of the belief). After both, inform the task mang.
		if ((action_info_received && newstate_info_received) || init_state){

			hrc_ros::RobotUpdateMsg update_msg;

			update_msg.stamp_robot_update = ros::Time::now();
			update_msg.action_taken_time = action_taken_time;
			update_msg.robot_action_taken = robot_action_taken; // took action in the belief state
			update_msg.robot_belief_state = robot_belief_state; // belief state

			// if (initial_state_received){
			if (init_state){
				update_msg.robot_real_state = robot_belief_state; // in initial state robot mdp/pomdp sends the init state directly
				ros::param::set("/robot_initial_state", false);
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

			test_result_file.open (filename_robot_pomdp_eval,std::ios_base::app); 
			test_result_file << update_msg.robot_action_taken + "," + update_msg.robot_belief_state + "," + update_msg.robot_real_state + "," + update_msg.immediate_reward + "," + update_msg.total_disc_reward + ","  + "\n";
			test_result_file.close();			
			
			callTaskMang_inform.call(robotUpdateReq, robotUpdateRes);

		}

	};
	//See RFC 6455 7.4.1. for status codes

	//See http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html, Error Codes for error code meanings
	echo.on_error=[](shared_ptr<WsServer::Connection> connection, const SimpleWeb::error_code& ec) {
		ros::spinOnce(); // TODO: intended to catch the reset service call
		cout << "[ROBOT AGENT] Server: Error in connection " << (size_t)connection.get() << ". " <<
				"Error: " << ec << ", error message: " << ec.message() << endl;
	};

	server.start();
}
