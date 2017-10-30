// created by Qichao Xu in 23.Juli.2017
// update and stable version by Orhan Can Görür 25.09.2017

#include <ros/ros.h>

// include <package_name/service_type_name.h>
#include <std_srvs/Trigger.h>

#include <hrc_ros/TraySensor.h>
#include <hrc_ros/InformHumanAction.h>
#include <hrc_ros/InformHumanState.h>
#include <hrc_ros/InformObsToTaskMang.h>
#include <hrc_ros/ResetObsROS.h>

#include <hrc_ros/ObsUpdateMsg.h>

#include <stdlib.h> 				// for rand() and RAND_MAX
#include <string>

#include "simple_web_socket/client_ws.hpp"

using namespace std;
typedef SimpleWeb::SocketClient<SimpleWeb::WS> WsClient;


bool prevStIsInitSt = true; 
bool preventOneLoop = true; 
bool humanLoop = false;
int FailedStCounter = 0;
int EvaluateStCounter = 0;
int TiredStCounter = 0;
int NoAttentionCounter = 0;

int humanidle_counter = 0;
int humanfail_counter = 0;

bool ipd_sensor = false;	// inspected product detector sensor
bool upd_sensor = false;	// uninspected product detector sensor
string whoSucceeded = "";
ros::Time tray_msg_stamp;

ros::ServiceClient is_ov; // ACTION: LOOK AROUND
ros::ServiceClient is_oir; // SITUATION: IS HUMAN DETECTED?
//ros::ServiceClient is_ho  = nh.serviceClient<std_srvs::Trigger>("/human/is_ho");
ros::ServiceClient is_a0;  // ACTION: GRASP ATTEMPT 
ros::ServiceClient is_a2;  // ACTION: IDLE
ros::ServiceClient is_a4;  // ACTION: WARN THE ROBOT
ros::Subscriber traySensor_subs; // SUBSCRIBE: TRAY SENSORS

ros::ServiceClient ObsUpdater; // Calling the task manager service for observation status update
string real_robot_state_name = "";

string prev_robot_observation_pomdp = ""; // only for proactive model
string prev_observables = "0";
string prev_robot_state = ""; // for both reactive and proactive
string prev_real_robot_state = ""; // for both reactive and proactive
string prev_human_state = "";
bool humanAttempted = false; // for reactive model to state human attempted to grasp (take action) and track if success occured afterwards. Otherwise take over!
int human_task_time = 0; // this counts every second that human spends when the task is assigned to him

bool humanTrustsRobot;
string humanType; // beginner, expert
string humanMood; // for beginner: stubborn, distracted, thinker, tired; for expert: normal, tired
string robotType; // either proactive or reactive
string whoIsAssigned;

void humanStCounter(string humanState){
	
	if (humanState == "FailedToGrasp")
		FailedStCounter++;
	if (humanState == "Evaluating")
		EvaluateStCounter++;
	if (humanState == "Tired")
		TiredStCounter++;
	if (humanState == "NoAttention")
		NoAttentionCounter++;
	if (humanState == "TaskHuman" || humanState == "TaskRobot"){
		FailedStCounter = 0;
		EvaluateStCounter = 0;
		TiredStCounter = 0;
		NoAttentionCounter = 0;
	}
}

string getRealRbtStPOMDP(string humanState)
{
	/*
	 * TaskHuman
	 * MayNotBeCapable
	 * MayBeTired
	 * NoFocus
	 * NeedsToBeReminded
	 * NeedsHelp
	 * NoNeedHelp
	 * WarningReceived
	 * GlobalSuccess
	 * GlobalFail
	 * TaskRobot
	 */
	
	string currRbtSt = "0";
	humanStCounter(humanState);

	if (humanState == "TaskHuman"){
		real_robot_state_name = "TaskHuman";
	} else if (humanState == "GlobalSuccess"){
		real_robot_state_name = "GlobalSuccess";
	} else if (humanState == "GlobalFail"){
		real_robot_state_name = "GlobalFail";
	} else if (humanState == "Evaluating"){
		if (humanMood == "tired"){
			real_robot_state_name = "MayBeTired";
		} else if (EvaluateStCounter <= 2){
			// any prev estimation (either taskhuman, nofocus, may not be capable, may be tired)
			real_robot_state_name = (whoIsAssigned == "human") ? "TaskHuman": "TaskRobot"; // seriously evaluating :)
		} else if (EvaluateStCounter > 2){
			if (FailedStCounter > 0){
				if (prev_real_robot_state != "MayNotBeCapable" || prev_real_robot_state != "NeedsHelp"){
					real_robot_state_name = "MayNotBeCapable";
				} else if (humanTrustsRobot){
					real_robot_state_name = "NeedsHelp";
				} else {
					real_robot_state_name = "MayBeTired";
				}
			}
			if (NoAttentionCounter > 0){
				if (prev_real_robot_state != "NoFocus" || prev_real_robot_state != "NeedsToBeReminded"){
					real_robot_state_name = "NoFocus";
				} else if (prev_real_robot_state == "NoFocus"){
					real_robot_state_name = "NeedsToBeReminded";
				} else if (humanTrustsRobot && FailedStCounter > 0){
					real_robot_state_name = "NeedsHelp";
				} else {
					real_robot_state_name = "MayBeTired";
				}
			}
			if (FailedStCounter == 0 && NoAttentionCounter == 0){
				real_robot_state_name = (whoIsAssigned == "human") ? "TaskHuman": "TaskRobot";
			}
		}
	} else if (humanState == "FailedToGrasp"){
		if (FailedStCounter == 1){
			if (humanTrustsRobot && EvaluateStCounter > 1){ // already one spent before failing. thinking more than twice and already failed once
				real_robot_state_name = "NeedsHelp";
			} else { // evaluated less than 2 iteration, might still be not capable
				real_robot_state_name = "MayNotBeCapable";
			}
		} else if (FailedStCounter >= 2){
			if (humanTrustsRobot && (prev_human_state != "FailedToGrasp")){ // means not constantly trying and trusting the human
				real_robot_state_name = "NeedsHelp";
			} else {
				real_robot_state_name = "NoNeedHelp"; //TODO: may think of more about this
			}
		}
	} else if (humanState == "NoAttention"){
		if (humanMood == "tired"){
			real_robot_state_name = "MayBeTired";
		} else if (NoAttentionCounter == 1){
			real_robot_state_name = "NoFocus";			
		} else if (NoAttentionCounter == 1 && EvaluateStCounter > 1){
			real_robot_state_name = "NeedsToBeReminded";
		} else if (NoAttentionCounter >= 2){
			real_robot_state_name = "NeedsToBeReminded";
		}
	} else if (humanState == "Tired"){
		if (TiredStCounter == 1){
			real_robot_state_name = "MayBeTired";
			if (humanTrustsRobot && (NoAttentionCounter > 1 || EvaluateStCounter > 2))
				real_robot_state_name = "NeedsHelp";
		} else { // tired state multiple times
			if (humanTrustsRobot){
				real_robot_state_name = "NeedsHelp";
			} else {
				real_robot_state_name = "NoNeedHelp";
			}
		}
	} else if (humanState == "Recovery"){
		real_robot_state_name = "NeedsHelp";
	} else if (humanState == "WarningTheRobot"){
		real_robot_state_name = "WarningReceived";
	} else if (humanState == "RobotIsWarned"){
		real_robot_state_name = "NoNeedHelp";
	} else if (humanState == "TaskRobot"){
		real_robot_state_name = "TaskRobot";
	} 
	prev_real_robot_state = real_robot_state_name;
	
	//Returning with the state number for the despot planner !!
	if (real_robot_state_name == "TaskHuman")
		currRbtSt = "0";
	else if (real_robot_state_name == "MayNotBeCapable")
		currRbtSt = "1";
	else if (real_robot_state_name == "MayBeTired")
		currRbtSt = "2";
	else if (real_robot_state_name == "NoFocus")
		currRbtSt = "3";
	else if (real_robot_state_name == "NeedsToBeReminded")
		currRbtSt = "4";
	else if (real_robot_state_name == "NeedsHelp")
		currRbtSt = "5";
	else if (real_robot_state_name == "NoNeedHelp")
		currRbtSt = "6";
	else if (real_robot_state_name == "WarningReceived")
		currRbtSt = "7";
	else if (real_robot_state_name == "GlobalSuccess")
		currRbtSt = "8";
	else if (real_robot_state_name == "GlobalFail")
		currRbtSt = "9";
	else if (real_robot_state_name == "TaskRobot")
		currRbtSt = "10";
	
	return currRbtSt;
}

string MapObservationsToPOMDP(string observable){
		
		
		string robot_observation = "";
		
		if (observable == "0" || observable == "1") {
			robot_observation = "0";
		}
		else if (observable == "2") {
			robot_observation = "1";
		}
		else if (observable == "3") {
			robot_observation = "2";
		}
		else if (observable == "6") {
			robot_observation = "3";
		}
		else if	(observable == "7") {
			robot_observation = "4";
		}
		else if (observable == "8" || observable == "9" || observable == "10" || observable == "11" || observable == "14" || observable == "15") {
			robot_observation = "5";
		}
		else if (observable == "16" || observable == "17" || observable == "18" || observable == "19" || observable == "27") {
			robot_observation = "6";
		}
		else if (observable == "32" || observable == "33") {
			robot_observation = "7";
		}
		else if (observable == "34") {
			robot_observation = "8";
		}
		else if (observable == "35") {
			robot_observation = "9";
		}
		else if (observable == "40" || observable == "42" || observable == "43") {
			robot_observation = "10";
		}
		else if (observable == "64" || observable == "65" || observable == "66" || observable == "67" || observable == "70" || observable == "71") {
			robot_observation = "11";
		}
		else if (observable == "80" || observable == "81" || observable == "82" || observable == "83" || observable == "84" || observable == "85") {
			robot_observation = "11";
		}
		else if (observable == "86" || observable == "87" || observable == "96" || observable == "98" ||observable == "99" ||observable == "103") {
			robot_observation = "11";
		}else{
			robot_observation = prev_robot_observation_pomdp;
		}
		
		prev_robot_observation_pomdp = robot_observation;
		return robot_observation;
}

//TODO: the observable names of ov and oir update. !OV = Looking Around and OIR = Human is detected
//TODO: Simplify the observation combinations if the reactive model responds almost better than the proactive one!
string MapObservablesToObservations(bool ov, bool oir, bool a0, bool ipd, bool a4, bool a2, bool upd){
	string robot_observation = prev_observables;	
	if (not ov && not oir && not a0 && not ipd && not a4 && not a2 && not upd) {
		robot_observation = "0";	
	}
	else if (ov && not oir && not a0 && not ipd && not a4 && not a2 && not upd) {
		robot_observation = "1";
	}
	else if (not ov && oir && not a0 && not ipd && not a4 && not a2 && not upd) {
		robot_observation = "2";
	}
	else if (ov && oir && not a0 && not ipd && not a4 && not a2 && not upd) {
		robot_observation = "3";
	}
	else if (not ov && oir && a0 && not ipd && not a4 && not a2 && not upd) {
		robot_observation = "6";
	}
	else if (ov && oir && a0 && not ipd && not a4 && not a2 && not upd) {
		robot_observation = "7";
	}
	else if (not ov && not oir && not a0 && ipd && not a4 && not a2 && not upd) {
		robot_observation = "8";
	}
	else if (ov && not oir && not a0 && ipd && not a4 && not a2 && not upd) {
		robot_observation = "9";
	}
	else if (not ov && oir && not a0 && ipd && not a4 && not a2 && not upd) {
		robot_observation = "10";
	}
	else if (ov && oir && not a0 && ipd && not a4 && not a2 && not upd) {
		robot_observation = "11";
	}
	else if (not ov && oir && a0 && ipd && not a4 && not a2 && not upd) {
		robot_observation = "14";
	}
	else if (ov && oir && a0 && ipd && not a4 && not a2 && not upd) {
		robot_observation = "15";
	}
	else if (not ov && not oir && not a0 && not ipd && a4 && not a2 && not upd) {
		robot_observation = "16";
	}
	else if (not ov && oir && not a0 && not ipd && a4 && not a2 && not upd) {
		robot_observation = "18";
	}
	else if (ov && oir && not a0 && not ipd && a4 && not a2 && not upd) {
		robot_observation = "19";
	}
	else if (ov && oir && not a0 && ipd && a4 && not a2 && not upd) {
		robot_observation = "27";
	}
	else if (not ov && not oir && not a0 && not ipd && not a4 && a2 && not upd) {
		robot_observation = "32";
	}
	else if (not ov && oir && not a0 && not ipd && not a4 && a2 && not upd) {
		robot_observation = "34";
	}
	else if (ov && oir && not a0 && not ipd && not a4 && a2 && not upd) {
		robot_observation = "35";
	}
	else if (not ov && not oir && not a0 && ipd && not a4 && a2 && not upd) {
		robot_observation = "40";
	}
	else if (not ov && oir && not a0 && ipd && not a4 && a2 && not upd) {
		robot_observation = "42";
	}
	else if (ov && oir && not a0 && ipd && not a4 && a2 && not upd) {
		robot_observation = "43";
	}
	else if (not ov && oir && a0 && ipd && not a4 && a2 && not upd) {
		robot_observation = "46";
	}
	else if (ov && oir && a0 && ipd && not a4 && a2 && not upd) {
		robot_observation = "47";
	}
	else if (not ov && not oir && not a0 && not ipd && not a4 && not a2 && upd) {
		robot_observation = "64";
	}
	else if (not ov && oir && not a0 && not ipd && not a4 && not a2 && upd) {
		robot_observation = "66";
	}
	else if (ov && oir && not a0 && not ipd && not a4 && not a2 && upd) {
		robot_observation = "67";
	}
	else if (not ov && oir && a0 && not ipd && not a4 && not a2 && upd) {
		robot_observation = "70";
	}
	else if (ov && oir && a0 && not ipd && not a4 && not a2 && upd) {
		robot_observation = "71";
	}
	else if (not ov && not oir && not a0 && not ipd && a4 && not a2 && upd) {
		robot_observation = "80";
	}
	else if (not ov && oir && not a0 && not ipd && a4 && not a2 && upd) {
		robot_observation = "82";
	}
	else if (ov && oir && not a0 && not ipd && a4 && not a2 && upd) {
		robot_observation = "83";
	}
	else if (not ov && oir && a0 && not ipd && a4 && not a2 && upd) {
		robot_observation = "86";
	}
	else if (ov && oir && a0 && not ipd && a4 && not a2 && upd) {
		robot_observation = "87";
	}
	else if (not ov && not oir && not a0 && not ipd && not a4 && a2 && upd) {
		robot_observation = "96";
	}
	else if (not ov && oir && not a0 && not ipd && not a4 && a2 && upd) {
		robot_observation = "98";
	}
	else if (ov && oir && not a0 && not ipd && not a4 && a2 && upd) {
		robot_observation = "99";
	}
	else if (not ov && oir && a0 && not ipd && not a4 && a2 && upd) {
		robot_observation = "102";
	}
	else if (ov && oir && a0 && not ipd && not a4 && a2 && upd) {
		robot_observation = "103";
	}
	else if (upd == 1){
		robot_observation = "64";
	}
	else if (ipd == 1){
		robot_observation = "8";
	}
	prev_observables = robot_observation;
	
	return robot_observation;
}

string getRealRbtStMDP(string humanState)
{
	// The real full names of the states
	string currRbtSt = "0";

	humanStCounter(humanState);
	// In REACTIVE MODEL ROBOT IS NOT ABLE TO REASON ABOUT EVALUATING AND NO ATTENTION STATES
	if (humanState == "TaskHuman" || humanState == "Evaluating" || humanState == "NoAttention"){
		currRbtSt = "0";
		real_robot_state_name = "TaskHuman";
	} else if (humanState == "GlobalSuccess"){
		currRbtSt = "1";
		real_robot_state_name = "GlobalSuccess";
	} else if (humanState == "GlobalFail"){
		currRbtSt = "5";
		real_robot_state_name = "GlobalFail";
	} else if (humanState == "FailedToGrasp" && FailedStCounter >= 1){ // after a fail it already indicates the human needs help
		if(humanTrustsRobot){ // human should trust the robot.
			currRbtSt = "2";
			real_robot_state_name = "HumanNeedsHelp";
		} else {
			currRbtSt = "0";
			real_robot_state_name = "TaskHuman";
		}
	} else if (humanState == "Tired" && TiredStCounter >= 1){ // after one iteration in tired state it already indicates the human needs help
		if(humanTrustsRobot){ // human should trust the robot.
			currRbtSt = "2";
			real_robot_state_name = "HumanNeedsHelp";
		} else{
			currRbtSt = "0";
			real_robot_state_name = "TaskHuman";
		}
	} else if (humanState == "Recovery"){
		currRbtSt = "2";
		real_robot_state_name = "HumanNeedsHelp";
	} else if (humanState == "WarningTheRobot"){
		currRbtSt = "3";
		real_robot_state_name = "WarningReceived";
	} else if (humanState == "RobotIsWarned"){
		currRbtSt = "3";
		real_robot_state_name = "TaskHuman";
	} 
	else if (humanState == "TaskRobot"){
		currRbtSt = "4";
		real_robot_state_name = "TaskRobot";
	}
	return currRbtSt;
}

string MapObservationsToMDP(string observation) {
	/*
	 * Here are the Reactive robot states:
	 * TaskHuman
	   GlobalSuccess
	   HumanNeedsHelp
	   WarningReceived
	   TaskRobot
	   GlobalFail
	 */
	string robot_state = "";
	if ((prev_robot_state != "HumanNeedsHelp") && human_task_time >= 15 && (not ipd_sensor && not upd_sensor)){ // if human attempted to grasp in previous action yet still no success
		ROS_INFO("OBSERVATION ROS: Timeout! Task will be taken over by the ROBOT!");
		robot_state = "HumanNeedsHelp";
	} else if ((prev_robot_state != "HumanNeedsHelp") && humanAttempted && (not ipd_sensor && not upd_sensor)) {
		humanAttempted = false;
		robot_state = "HumanNeedsHelp";
		humanfail_counter = 0; // TODO: this is not being used anywhere but still here
	} else {
		
		if (observation == "0") {
			if (prev_robot_state == "TaskHuman")
				robot_state = "HumanNeedsHelp"; // whoever is assigned for the first state
			else if (prev_robot_state == "TaskRobot")
				robot_state = "TaskRobot"; // whoever is assigned for the first state
		} else if ((observation == "2" || observation == "3")) {
			robot_state = prev_robot_state; // This is either task assigned to human or task assigned to the robot
		} else if (observation == "6" || observation == "7") { // human attempted to grasp
			humanAttempted = true;
			robot_state = "TaskHuman";
			humanfail_counter ++; // TODO: i am increasing this but for now has nothing to do about the state decisions		
		} else if (observation == "8" || observation == "9" || observation == "10" || observation == "11") {
			robot_state = "GlobalSuccess";
		} else if (observation == "12" || observation == "13" || observation == "14" || observation == "15") {
			robot_state = "GlobalSuccess";
		} else if (observation == "40" || observation == "41" || observation == "42" || observation == "43") {
			robot_state = "GlobalSuccess";
		} else if (observation == "44" || observation == "45" || observation == "46" || observation == "47") {
			robot_state = "GlobalSuccess";
		} else if (observation == "34" || observation == "35") {
			robot_state = prev_robot_state; // whoever is assigned for the first state
			humanidle_counter ++; // idle counter is not used anywhere currently
		} else if (observation == "32" || observation == "33") { // human is not detected
			robot_state = "HumanNeedsHelp";
			humanidle_counter = 0;
		} else if (observation == "16" || observation == "17" || observation == "18" || observation == "19" || observation == "27") {
			robot_state = "WarningReceived";
		} else if (observation == "64" || observation == "65" || observation == "66" || observation == "67") {
			robot_state = "GlobalFail";
		} else if (observation == "68" || observation == "69" || observation == "70" || observation == "71") {
			robot_state = "GlobalFail";
		} else if (observation == "80" || observation == "81" || observation == "82" || observation == "83") {
			robot_state = "GlobalFail";
		} else if (observation == "84" || observation == "85" || observation == "86" || observation == "87") {
			robot_state = "GlobalFail";
		} else if (observation == "96" || observation == "97" || observation == "98" || observation == "99") {
			robot_state = "GlobalFail";
		} else if (observation == "100" || observation == "101" || observation == "102" || observation == "103") {
			robot_state = "GlobalFail";
		}
	}
	
	prev_robot_state = robot_state;
	if (robot_state == "TaskHuman"){
		robot_state = "0";
	}
	else if (robot_state == "GlobalSuccess"){
		robot_state = "1";
	}
	else if (robot_state == "HumanNeedsHelp"){
		robot_state = "2";
	}
	else if (robot_state == "WarningReceived"){
		robot_state = "3";
	}
	else if (robot_state == "TaskRobot"){
		robot_state = "4";
	}
	else if (robot_state == "GlobalFail"){
		robot_state = "5";
	}
	
	return robot_state;
}

bool resetScenario(hrc_ros::ResetObsROSRequest &req,
		hrc_ros::ResetObsROSResponse &res) {
	if (req.assignedTo == "human"){
		prev_robot_state = "TaskHuman";
		prev_human_state = "TaskHuman";
		real_robot_state_name = "TaskHuman";
		whoIsAssigned = "human";
	}else if (req.assignedTo == "robot"){
		prev_robot_state = "TaskRobot";
		prev_human_state = "TaskRobot";
		real_robot_state_name = "TaskRobot";
		whoIsAssigned = "robot";
	}
	
	
	humanTrustsRobot = (req.humanTrustsRobot == "YES") ? true : false;
	humanType = req.humanType;
	humanMood = req.humanMood;
	robotType = req.robotType;
	
	prev_real_robot_state = real_robot_state_name;
	prev_robot_observation_pomdp = "";
	prev_observables = "0";
	
	human_task_time = 0; // counting the time spent when a task is assigned to the human
	humanAttempted = false;
	
	FailedStCounter = 0;
	EvaluateStCounter = 0;
	TiredStCounter = 0;
	NoAttentionCounter = 0;

	humanidle_counter = 0;
	humanfail_counter = 0;

	ipd_sensor = false;	// inspected product detector sensor
	upd_sensor = false;	// uninspected product detector sensor
	ROS_INFO("OBSERVATION ROS: Reset!");
	res.success = true;
	return true;
}

// HUMAN ACTION IS RECEIVED !!!
bool action_to_obs_Map(hrc_ros::InformHumanAction::Request &req, 
		hrc_ros::InformHumanAction::Response &res) {
	
	//WebSocket (WS)-client at port 7070 using 1 thread
	WsClient client("localhost:7070");
	
	client.on_open=[&]() {
		
		std_srvs::Trigger::Request req1;
		std_srvs::Trigger::Response resp1;
		is_ov.call(req1, resp1);
		bool ov = resp1.success;	// object is visiable
	
		std_srvs::Trigger::Request req2;
		std_srvs::Trigger::Response resp2;
		is_oir.call(req2, resp2);
		bool oir = resp2.success;	// object is in range
	
		bool ipd = ipd_sensor;	// inspected product is detected // SITUATION: INSPECTED PRODUCT DETECTED? (SUCCESS)
		bool upd = upd_sensor;	// uninspected product is detected  // SITUATION: UNINSPECTED PRODUT DETECTED? (FAIL)
		/*		
		std_srvs::Trigger::Request req3;
		std_srvs::Trigger::Response resp3;
		is_ho.call(req3, resp3);
		bool ho = resp3.success;	// have object
		*/
		std_srvs::Trigger::Request req5;
		std_srvs::Trigger::Response resp5;
		is_a0.call(req5, resp5);
		bool a0 = resp5.success;	// action is attemp grasp
	
		std_srvs::Trigger::Request req6;
		std_srvs::Trigger::Response resp6;
		is_a2.call(req6, resp6);
		bool a2 = resp6.success;	// action is staying idle
	
		std_srvs::Trigger::Request req7;
		std_srvs::Trigger::Response resp7;
		is_a4.call(req7, resp7);
		bool a4 = resp7.success;	// action is warn robot
			
		// ===== ADDING A NOISE TO THE OBSERVATIONS ========
		bool ov_noisy = ov;
		bool oir_noisy = oir;
		bool a0_noisy = a0;
		bool ipd_noisy = ipd;
		bool a4_noisy = a4;
		bool a2_noisy = a2;
		bool upd_noisy = upd;
		
		int r = rand() % 10;
		int m = rand() % 2; // mixed or missed
		if ((a0 || a4) && r == 0){ // if one of this is one then 10% noise
			if (m == 0){ // 5 % chance of mixing up a0 and a4
				a0_noisy = not a0;
				a4_noisy = not a4;
			} else if (m == 1){ // 5 % chance of missing the gesture and assuming idle (a2)
				a0_noisy = false;
				a4_noisy = false;
				a2_noisy = true;
			}
		} else if (((not ov) || a2) && r == 0){ // if either looking around (not ov) or idle is detected, 10 % chance of mixing them up
			ov_noisy = not ov;
			a2_noisy = not a2;
		}
		// ===================================================
		
		string robot_observation_real = "", observation = "", robot_observation_noisy = "";
		robot_observation_real = MapObservablesToObservations(ov,oir,a0,ipd,a4,a2,upd);
		robot_observation_noisy = MapObservablesToObservations(ov_noisy,oir_noisy,a0_noisy,ipd_noisy,a4_noisy,a2_noisy,upd_noisy);
		
		if (robotType == "reactive"){
			observation = MapObservationsToMDP(robot_observation_noisy); // Get correspending state for reactive ROBOT wrt observations to state mapping
		} else if (robotType == "proactive"){
			observation = MapObservationsToPOMDP(robot_observation_noisy); // Get correspending observation for proactive ROBOT wrt observables received
		}
		
		ROS_INFO("OBSERVATION ROS: real_observable: %s, noisy_observable: %s, mapped observation: %s", 
				robot_observation_real.c_str(), robot_observation_noisy.c_str(), observation.c_str());
		string message = observation + ",-1"; // It is only sending observed_state, real state is send in another iteration (when it is provided)

		// ############ SENDING OBSERVATIONS TO TASK MANAGER ############
		
		hrc_ros::ObsUpdateMsg obs_update;
		
		hrc_ros::InformObsToTaskMang::Request reqForUpdate;
		hrc_ros::InformObsToTaskMang::Response resForUpdate;
		
		obs_update.stamp_obs_update = ros::Time::now();
		
		std::vector<uint8_t> real_obs_received;
		real_obs_received.push_back(not ov);
		real_obs_received.push_back(oir);
		real_obs_received.push_back(a0);
		real_obs_received.push_back(ipd);
		real_obs_received.push_back(a4);
		real_obs_received.push_back(a2);
		real_obs_received.push_back(upd);

		obs_update.real_obs_received = real_obs_received;

		
		std::vector<uint8_t> obs_with_noise;
		obs_with_noise.push_back(not ov_noisy);
		obs_with_noise.push_back(oir_noisy);
		obs_with_noise.push_back(a0_noisy);
		obs_with_noise.push_back(ipd_noisy);
		obs_with_noise.push_back(a4_noisy);
		obs_with_noise.push_back(a2_noisy);
		obs_with_noise.push_back(upd_noisy);
		
		obs_update.obs_with_noise = obs_with_noise;
		obs_update.who_succeeded = whoSucceeded;
		
		reqForUpdate.obs_update = obs_update;
		ObsUpdater.call(reqForUpdate, resForUpdate);
		
		// ###########################################################################################
		
		ROS_INFO("OBSERVATION Client: Sending to the robot planner:: OBSERVATION= %s", message.c_str());
		auto send_stream=make_shared<WsClient::SendStream>();
		*send_stream << message;
		client.send(send_stream);
	};
	
	client.on_message=[&client](shared_ptr<WsClient::Message> message) {
		client.send_close(1000);
	};
	
	client.start();
	
	return true;
}

bool humanSt_to_robotSt_Map(hrc_ros::InformHumanState::Request &req, 
		hrc_ros::InformHumanState::Response &res) {
	
	//WebSocket (WS)-client at port 7070 using 1 thread
	WsClient client("localhost:7070");
	
	client.on_open=[&]() {
		string realRbtSt_code = "";
		if (robotType == "reactive"){
			realRbtSt_code = getRealRbtStMDP(req.new_human_state); // this is the mapped human state to the robot's. Robot should estimate it correctly
		} else if (robotType == "proactive"){
			realRbtSt_code = getRealRbtStPOMDP(req.new_human_state); //For POMDP model
		}

		string message = "-1," + realRbtSt_code;
				
		ROS_INFO("OBSERVATION Client: Sending to the robot planner:: REAL STATE WAS = %s", message.c_str());
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


// ********* CALLBACKS *********** //
void ReceiveTraySensors(const hrc_ros::TraySensor &msg){
	tray_msg_stamp = msg.stamp;
	
	if (msg.tray_id == "tray_unprocessed"){
		if (msg.occupied)
			upd_sensor = true;
		else
			upd_sensor = false;
	}
	if (msg.tray_id == "tray_human"){
		if (msg.occupied){
			ipd_sensor = true;
			whoSucceeded = "human";
		}else{
			ipd_sensor = false;
			whoSucceeded = "";
		}
	}
	if (msg.tray_id == "tray_robot"){
		if (msg.occupied){
			ipd_sensor = true;
			whoSucceeded = "robot";
		}else{
			ipd_sensor = false;
			whoSucceeded = "";
		}
	}
}
void HumanTaskTimer(const ros::TimerEvent&){
	human_task_time += 1; // increase one in every second
}

// ********************************** //

int main(int argc, char **argv) {
	ros::init(argc, argv, "hrc_obs");

	ros::NodeHandle nh;
	
	// loop at 2Hz until the node is shut down
	ros::Rate rate(0.2);

	// ros::ServiceServer server = node_handle.advertiseService(service_name, pointer_to_callback_function);
	ros::ServiceServer action_server = nh.advertiseService("/hrc_obs/inform_human_action", &action_to_obs_Map);
	ros::ServiceServer new_state__server = nh.advertiseService("/hrc_obs/inform_new_human_state", &humanSt_to_robotSt_Map);
	
	ros::ServiceServer reset_scenario = nh.advertiseService("/hrc_obs/reset", &resetScenario);
	
	// ask for the observation
	is_ov  = nh.serviceClient<std_srvs::Trigger>("/human/is_ov");  // ACTION: LOOK AROUND
	is_oir = nh.serviceClient<std_srvs::Trigger>("/human/is_oir"); // SITUATION: IS HUMAN DETECTED?
	//ros::ServiceClient is_ho  = nh.serviceClient<std_srvs::Trigger>("/human/is_ho");
	is_a0  = nh.serviceClient<std_srvs::Trigger>("/human/is_a0");  // ACTION: GRASP ATTEMPT 
	is_a2  = nh.serviceClient<std_srvs::Trigger>("/human/is_a2");  // ACTION: IDLE
	is_a4  = nh.serviceClient<std_srvs::Trigger>("/human/is_a4");  // ACTION: WARN THE ROBOT
	
	traySensor_subs = nh.subscribe("/production_line/tray_sensors", 1000, &ReceiveTraySensors);
	
	ObsUpdater = nh.serviceClient<hrc_ros::InformObsToTaskMang>("/hrc_task_manager/observation_update");  // Client for the task manager service for the status update
	
	ros::Timer timer1 = nh.createTimer(ros::Duration(1.0), HumanTaskTimer);
	
	ros::spin();
}
