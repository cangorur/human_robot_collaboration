// created by Qichao Xu in 23.Juli.2017

#include <ros/ros.h>

// include <package_name/service_type_name.h>
#include <std_srvs/Trigger.h>
#include <turtlesim/Spawn.h>

#include <stdlib.h> 				// for rand() and RAND_MAX
#include <string>

#include "simple_web_socket/client_ws.hpp"

using namespace std;
typedef SimpleWeb::SocketClient<SimpleWeb::WS> WsClient;


bool prevStIsInitSt = true; 
bool preventOneLoop = true; 
bool humanLoop = false;
int StupidStCounter = 0;
string prevHuSt = "";

int humanStCounter(string humanState)
{
	if (humanState == "2" || humanState == "3" || humanState == "4")
		StupidStCounter++;
	return StupidStCounter;
}

string getRbtSt(string humanState)
{
	int currStupidStCounter;
	string currRbtSt = "";

	if (prevHuSt==humanState)
		return currRbtSt = "3";

	currStupidStCounter = humanStCounter(humanState);

	if (currStupidStCounter >= 2)
		return currRbtSt = "3";

	if (humanState == "0")
		currRbtSt = "0";
	else if (humanState == "1")
		currRbtSt = "1";
	else if (humanState == "2" && prevStIsInitSt == true)
		currRbtSt = "2";
	else if (humanState == "2" && prevStIsInitSt == false)
		currRbtSt = "3";
	else if (humanState == "3" && prevStIsInitSt == true)
		currRbtSt = "4";
	else if (humanState == "3" && prevStIsInitSt == false)
		currRbtSt = "5";
	else if (humanState == "4" && prevStIsInitSt == true) 
		currRbtSt = "2";	
	else if (humanState == "4" && prevStIsInitSt == false) 
		currRbtSt = "3";
	else if (humanState == "5" && prevStIsInitSt == true)
		currRbtSt = "2";
	else if (humanState == "5" && prevStIsInitSt == false)
		currRbtSt = "3";
	else if (humanState == "6")
		currRbtSt = "3";
	else
		currRbtSt = "6";

	if (!preventOneLoop){
		prevStIsInitSt = false;
	}

	prevHuSt = humanState;

	preventOneLoop = false;

	return currRbtSt;
}


bool obs_state_Map(turtlesim::Spawn::Request &req, turtlesim::Spawn::Response &resp) {
	
	string humanState = req.name;
  string robot_state = getRbtSt(humanState);

	//WebSocket (WS)-client at port 7070 using 1 thread
  WsClient client("localhost:7070");

  client.on_open=[&]() {

    string robot_observation = "";
		
		ros::NodeHandle nh;

		/*
		O-0 #  object not visible, object not in range, no grasp attempt, has no object, received no warning, not idling
    O-1 #  object     visible, object not in range, no grasp attempt, has no object, received no warning, not idling
    O-2 #  object not visible, object     in range, no grasp attempt, has no object, received no warning, not idling
    O-3 #  object     visible, object     in range, no grasp attempt, has no object, received no warning, not idling
    O-7 #  object     visible, object     in range,    grasp attempt, has no object, received no warning, not idling
    O-10 # object not visible, object     in range, no grasp attempt, has    object, received no warning, not idling
    O-11 # object     visible, object     in range, no grasp attempt, has    object, received no warning, not idling
    O-16 # object not visible, object not in range, no grasp attempt, has no object, received    warning, not idling
    O-17 # object     visible, object not in range, no grasp attempt, has no object, received    warning, not idling
    O-18 # object not visible, object     in range, no grasp attempt, has no object, received    warning, not idling
    O-19 # object     visible, object     in range, no grasp attempt, has no object, received    warning, not idling
    O-32 # object not visible, object not in range, no grasp attempt, has no object, received no warning,     idling
    O-33 # object     visible, object not in range, no grasp attempt, has no object, received no warning,     idling
    O-34 # object not visible, object     in range, no grasp attempt, has no object, received no warning,     idling
    O-35 # object     visible, object     in range, no grasp attempt, has no object, received no warning,     idling
		*/

    // ask for the observation
		ros::ServiceClient is_ov  = nh.serviceClient<std_srvs::Trigger>("/human/is_ov");
		ros::ServiceClient is_oir = nh.serviceClient<std_srvs::Trigger>("/human/is_oir");
		ros::ServiceClient is_ho  = nh.serviceClient<std_srvs::Trigger>("/human/is_ho");
		ros::ServiceClient is_a0  = nh.serviceClient<std_srvs::Trigger>("/human/is_a0");
		ros::ServiceClient is_a2  = nh.serviceClient<std_srvs::Trigger>("/human/is_a2");
		ros::ServiceClient is_a4  = nh.serviceClient<std_srvs::Trigger>("/human/is_a4");

		std_srvs::Trigger::Request req1;
		std_srvs::Trigger::Response resp1;
		is_ov.call(req1, resp1);
		bool ov = resp1.success;	// object is visiable

		std_srvs::Trigger::Request req2;
		std_srvs::Trigger::Response resp2;
		is_oir.call(req2, resp2);
		bool oir = resp2.success;	// object is in range

		std_srvs::Trigger::Request req3;
		std_srvs::Trigger::Response resp3;
		is_ho.call(req3, resp3);
		bool ho = resp3.success;	// have object

		std_srvs::Trigger::Request req4;
		std_srvs::Trigger::Response resp4;
		is_a0.call(req4, resp4);
		bool a0 = resp4.success;	// action is attemp grasp

		std_srvs::Trigger::Request req5;
		std_srvs::Trigger::Response resp5;
		is_a2.call(req5, resp5);
		bool a2 = resp5.success;	// action is staying idle

		std_srvs::Trigger::Request req6;
		std_srvs::Trigger::Response resp6;
		is_a4.call(req6, resp6);
		bool a4 = resp6.success;	// action is warn robot

    cout << "observatioin: " << ov << " " << oir << " " << a0 << " " << ho << " " << a4 << " " << a2 << endl;

		if (not ov && not oir && not ho && not a0 && not a2 && not a4) {
			robot_observation = "0";
		}
		else if (    ov && not oir && not ho && not a0 && not a2 && not a4) {
			robot_observation = "1";
		}
		else if (not ov &&     oir && not ho && not a0 && not a2 && not a4) {
			robot_observation = "2";
		}
		else if (    ov &&     oir && not ho && not a0 && not a2 && not a4) {
			robot_observation = "3";
		}
		else if (    ov &&     oir && not ho &&     a0 && not a2 && not a4) {
			robot_observation = "4";
		}
		else if (not ov &&     oir &&     ho && not a0 && not a2 && not a4) {
			robot_observation = "5";
		}
		else if (    ov &&     oir &&     ho && not a0 && not a2 && not a4) {
			robot_observation = "6";
		}
		else if (not ov && not oir && not ho && not a0 && not a2 &&     a4) {
			robot_observation = "7";
		}
		else if (    ov && not oir && not ho && not a0 && not a2 &&     a4) {
			robot_observation = "8";
		}
		else if (not ov &&     oir && not ho && not a0 && not a2 &&     a4) {
			robot_observation = "9";
		}
		else if (    ov &&     oir && not ho && not a0 && not a2 &&     a4) {
			robot_observation = "10";
		}
		else if (not ov && not oir && not ho && not a0 &&     a2 && not a4) {
			robot_observation = "11";
		}
		else if (    ov && not oir && not ho && not a0 &&     a2 && not a4) {
			robot_observation = "12";
		}
		else if (not ov &&     oir && not ho && not a0 &&     a2 && not a4) {
			robot_observation = "13";
		}
		else if (    ov &&     oir && not ho && not a0 &&     a2 && not a4) {
			robot_observation = "14";
		}
		else {
			robot_observation = "15";
		}
		
		string message = robot_observation + "," + robot_state;

    cout << "obs Client: Sending message: \"" << message << "\"" << endl;

    auto send_stream=make_shared<WsClient::SendStream>();
    *send_stream << message;
    client.send(send_stream);
  };

  client.on_message=[&client](shared_ptr<WsClient::Message> message) {
    //cout << "Client: Sending close connection" << endl;
    client.send_close(1000);
  };
  
  client.start();

	return true;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "hrc_obs");

	ros::NodeHandle nh;
	
	// loop at 2Hz until the node is shut down
	ros::Rate rate(0.2);

	// ros::ServiceServer server = node_handle.advertiseService(service_name, pointer_to_callback_function);
	ros::ServiceServer server = nh.advertiseService("/human/state", &obs_state_Map);
	ros::spin();
}