// created by Qichao Xu in 23.Juli.2017

#include <ros/ros.h>

// include <package_name/service_type_name.h>
#include <std_srvs/Trigger.h>
#include <hrc_ros/InformHumanToTaskMang.h>
#include <hrc_ros/InformHumanAction.h>
#include <hrc_ros/InformHumanState.h>
#include <hrc_ros/ResetHumanROS.h>

#include <stdlib.h> 				// for rand() and RAND_MAX
#include <string>
#include <thread>

#include "simple_web_socket/server_ws.hpp"

using namespace std;
typedef SimpleWeb::SocketServer<SimpleWeb::WS> WsServer;

bool action_info_received = false;
bool newstate_info_received = false;
bool initial_state_received = true;

ros::ServiceClient call_obsNode_action;
ros::ServiceClient call_obsNode_newState;
ros::ServiceClient call_taskMang;

ros::ServiceServer reset_scenario;

string human_action_taken = "";
ros::Time action_taken_time;
string human_belief_state = "";
string human_real_state = "";

int task_number_counter = 0;

bool resetScenario(hrc_ros::ResetHumanROSRequest &req,
		hrc_ros::ResetHumanROSResponse &res) {
	human_action_taken = "";
	human_belief_state = "";
	human_real_state = "";
	initial_state_received = true;
	ROS_INFO("HUMAN ROS: Reset!");
	
	res.success = true;
	return true;
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "hrc_human");
	
	ros::NodeHandle nh;
	
	call_obsNode_action = nh.serviceClient<hrc_ros::InformHumanAction>("/hrc_obs/inform_human_action");
	call_obsNode_newState = nh.serviceClient<hrc_ros::InformHumanState>("/hrc_obs/inform_new_human_state");
	call_taskMang = nh.serviceClient<hrc_ros::InformHumanToTaskMang>("/hrc_task_manager/human_status_update");
	reset_scenario = nh.advertiseService("/hrc_human/reset", &resetScenario);
	// loop at 2Hz until the node is shut down
	// ros::Rate rate(0.2); // No need for this when running a thread as a service
	
	srand (time(NULL));
	
	//WebSocket (WS)-server at port 9090 using 1 thread
	WsServer server;
	server.config.port = 9090;
	
	auto& echo=server.endpoint["^/?$"];
	
	echo.on_open=[](shared_ptr<WsServer::Connection> connection) {
	//cout << "Server: Opened connection " << (size_t)connection.get() << endl;
	};
	
	echo.on_message=[&](shared_ptr<WsServer::Connection> connection, shared_ptr<WsServer::Message> message) {
	
		ros::spinOnce(); // TODO: intended to catch the reset service call
		
		string message_receive=message->string();
		
		cout << "Human ROS Server: Message received: \"" << message_receive << "\"" << endl;
		
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
		string human_action = message_receive.substr(0, index);
		string human_state = message_receive.substr(index+1);
		
	
		/* call action service*/
		// EVERY MESSAGE IS EITHER A HUMAN ACTION BELIEF UPDATE OR A HUMAN REAL STATE UPDATE
		// HUMAN ACTION UPDATE RECEIVED. INFORMING ABOUT BOTH ACTION AND THE STATE (either initial or belief state) 
		if (human_action != "-1"){

			ros::ServiceClient stayIdle = nh.serviceClient<std_srvs::Trigger>("/human/stay_idle");
			ros::ServiceClient walkAway = nh.serviceClient<std_srvs::Trigger>("/human/walk_away");
			ros::ServiceClient lookAround = nh.serviceClient<std_srvs::Trigger>("/human/look_around");
			ros::ServiceClient warnRobot = nh.serviceClient<std_srvs::Trigger>("/human/warn_robot");
			ros::ServiceClient attemptGrasp = nh.serviceClient<std_srvs::Trigger>("/human/attempt_grasp");
			ros::ServiceClient grasp = nh.serviceClient<std_srvs::Trigger>("/human/grasp");
		
			std_srvs::Trigger::Request req;
			std_srvs::Trigger::Response resp;
			
			action_taken_time = ros::Time::now();
			
			// TODO: For the expert case fix the probabilities below
			bool success = true; // TODO: this is not used!
			if (human_action == "0") {	// grasp or grasp attempt
				if (task_number_counter > 30){
					if (rand() % 10 < 7){ // fixed to 80% chance of failure
						attemptGrasp.call(req, resp);
						ROS_INFO_STREAM("Current action is human attemping grasping...");
					} else {
						grasp.call(req, resp);
						ROS_INFO_STREAM("Current action is human grasping...");
					}
				} else if (task_number_counter > 20 && task_number_counter <= 30){
					if (rand() % 20 < 12){ // fixed to 60% chance of failure
						attemptGrasp.call(req, resp);
						ROS_INFO_STREAM("Current action is human attemping grasping...");
					} else {
						grasp.call(req, resp);
						ROS_INFO_STREAM("Current action is human grasping...");
					}
				} else if (rand() % 40 < task_number_counter) {	// gradually decrease the amount of successful grasp as the task number increases
					attemptGrasp.call(req, resp);
					ROS_INFO_STREAM("Current action is human attemping grasping...");
				} else {
					grasp.call(req, resp);
					ROS_INFO_STREAM("Current action is human grasping...");
				}
				
				// change the name of the action to its originals so that task manager recognizes
				human_action = "grasp";
			}
			else if (human_action == "1") {	// look around
				ROS_INFO_STREAM("Current action is human looking around...");
				lookAround.call(req, resp);
				human_action = "looking around";
			}
			else if (human_action == "2") {	// idle
				ROS_INFO_STREAM("Current action is human staying idle...");
				stayIdle.call(req, resp);
				human_action = "idle";
			}
			else if (human_action == "3") {	// walk away
				ROS_INFO_STREAM("Current action is human walking away...");
				walkAway.call(req, resp);
				human_action = "walking away";
			}
			else if (human_action == "4") {	// warn robot
				ROS_INFO_STREAM("Current action is human warning robot...");
				warnRobot.call(req, resp);
				human_action = "warn the robot";
			}
			else {
				success = true;
				//ROS_INFO_STREAM("Current action is human ERROR !!!");
			}
			ROS_INFO("HUMAN ROS: human in belief state: %s and took action: %s", human_state.c_str(), human_action.c_str());
			
			/* THIS IS TO LET HRC_OBS KNOW THAT HUMAN ACTED!*/		
			hrc_ros::InformHumanAction::Request req_action;
			hrc_ros::InformHumanAction::Response res_action;
			
			req_action.human_action = human_action;
			call_obsNode_action.call(req_action, res_action); // response is a boolean about the status of the service call
			
			// Below the global variables are assigned ! These are for sending the status later to the task manager
			human_action_taken = human_action;
			human_belief_state = human_state;
			action_info_received = true; // raise the action received flag
			newstate_info_received = false;
		
		}
		// HUMAN STATE UPDATE RECEIVED. INFORMING ABOUT THE NEW HUMAN STATE RECEIVED. 
		// THIS IS FOLLOWED BY THE ACTION SELECTION THEREFORE THIS NEW STATE IS THE REAL STATE OF THE NEXT STEP
		else{
			ROS_INFO("HUMAN ROS: Real state informed after the action is: %s", human_state.c_str());
			hrc_ros::InformHumanState::Request req_state;
			hrc_ros::InformHumanState::Response res_state;
			req_state.new_human_state = human_state; // 
			call_obsNode_newState.call(req_state, res_state); // response is a boolean about the status of the service call
			
			// global variables are assigned ! These are for sending the status later to the task manager
			human_real_state = human_state;
			
			newstate_info_received = true;
		}
		
		// TODO: currently it is sent every time human action info is received. So, send one in every two calls
		// First real state is received, then the action and the belief on this state. After both are received then inform task mang.
		if (action_info_received){
			// ############ SENDING HUMAN UPDATE TO TASK MANAGER ############
			action_info_received = false;
			
			hrc_ros::InformHumanToTaskMang::Request reqForUpdate;
			hrc_ros::InformHumanToTaskMang::Response resForUpdate;
			
			hrc_ros::HumanUpdateMsg update_msg;
			
			update_msg.stamp_human_update = ros::Time::now();
			update_msg.action_taken_time = action_taken_time;
			update_msg.human_belief_state = human_belief_state; // belief state
			update_msg.human_action_taken = human_action_taken; // took action in the belief state
			
			if (initial_state_received){
				update_msg.human_real_state = human_belief_state; // in initial state human mdp sends the init state directly
				initial_state_received = false;
			}else{
				update_msg.human_real_state = human_real_state; // new state
			}
			
			reqForUpdate.human_update = update_msg;
			
			call_taskMang.call(reqForUpdate, resForUpdate);
			task_number_counter = resForUpdate.task_number;
		}
	
	};
	
	server.start();

}
