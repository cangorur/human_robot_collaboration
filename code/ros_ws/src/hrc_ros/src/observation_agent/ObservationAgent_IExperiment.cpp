/*
 *  Created on: 19.04.2018
 *      Author: Orhan Can Görür
 *      Email: orhan-can.goeruer@dai-labor.de
 */

#include <ros/ros.h>

#include <stdlib.h> 				// for rand() and RAND_MAX
#include <string>
#include <thread>

#include <observation_agent/ObservationAgent_IExperiment.h>

using namespace std;

ObservationAgent::ObservationAgent() {
	ros::NodeHandle nh("~");
	initialize();
}

ObservationAgent::~ObservationAgent() {
}

void ObservationAgent::initialize(){
	ros::NodeHandle nh("~");

	/*
	 * Initializing ros services
	 */
	ObsUpdater = nh.serviceClient<hrc_ros::InformObsToTaskMang>("/task_manager/observation_update");  // Client for the task manager service for the status update

	/// ROS services by MORSE to call for the human actions
	is_ov  = nh.serviceClient<std_srvs::Trigger>("/human/is_ov");  // ACTION: LOOK AROUND
	is_oir = nh.serviceClient<std_srvs::Trigger>("/human/is_oir"); // SITUATION: IS HUMAN DETECTED?
	//ros::ServiceClient is_ho  = nh.serviceClient<std_srvs::Trigger>("/human/is_ho");
	is_a0  = nh.serviceClient<std_srvs::Trigger>("/human/is_a0");  // ACTION: GRASP ATTEMPT
	is_a2  = nh.serviceClient<std_srvs::Trigger>("/human/is_a2");  // ACTION: IDLE
	is_a4  = nh.serviceClient<std_srvs::Trigger>("/human/is_a4");  // ACTION: WARN THE ROBOT
	// This information below for recording human observable history. Processed and saved under TaskManager
	is_a0_failed  = nh.serviceClient<std_srvs::Trigger>("/human/is_a0_failed");  // ACTION: GRASP ATTEMPTED BUT FAILED

	/*
	 *Ros Publishers and Subscriber initialization 
	 * 
	 */
	traySensor_success_pub = nh.advertise<std_msgs::String>("pub_observedsuccess_status_update_IE", 1000); 
															 
	 
	/*
	 * ROS Services initialization
	 */
	action_server = nh.advertiseService("/observation_agent/inform_human_action", &ObservationAgent::action_to_obs_Map, this);
	IEaction_recognition_server = nh.advertiseService("/observation_agent/inform_action_recognized", &ObservationAgent::IE_receive_actionrecognition_update, this);
	IEtray_update_server = nh.advertiseService("/observation_agent/inform_tray_update", &ObservationAgent::IE_receive_tray_update, this);
	//new_state__server = nh.advertiseService("/observation_agent/inform_new_human_state", &ObservationAgent::humanSt_to_robotSt_Map, this);
	IE_new_state__server = nh.advertiseService("/observation_agent/inform_new_human_state", &ObservationAgent::IE_humanSt_to_robotSt_Map, this);
	reset_scenario = nh.advertiseService("/observation_agent/reset", &ObservationAgent::resetScenario, this);

	/*
	 * A ROS topic for a subscription to tray proximity sensors (detecting packages in the trays)
	 */
	//traySensor_subs = nh.subscribe("/production_line/tray_sensors", 1000, &ObservationAgent::ReceiveTraySensors, this);

	/*
	 * A ROS topic for a subscription to the 9D tray_detection, that indicates which object is placed in which container
	 */  // TODO remove if not needed 
	//trayDetection_subs = nh.subscribe("/camera_agent/tray_detection9D", 1000, &ObservationAgent::ReceiveTrayDetection9D, this);

	/// A ROS timer for the duration of a task assigned to the human
	task_timer = nh.createTimer(ros::Duration(1.0), &ObservationAgent::HumanTaskTimer, this);


	/// ROS rate to control the loop frequency
	ros::Rate rate(0.2);

	ROS_INFO("Observation Agent is created !");
	ros::spin();
}

bool ObservationAgent::resetScenario(hrc_ros::ResetObsROSRequest &req,
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

    // TODO inform robot type and human type -> remove human type 
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

// TODO: Change for baby example -> how to map new observables to this one ? 
/* 
OBSERVABLES
ov = object visible 
oir = object in range
ho = has object
a0 = grasp attempt 
ipd = inpsected product detected (ipd box sensor)
a4 = warn the robot
a2 = Idle
upd =  uninspected product detected (upd box sensor) 

*/

void ObservationAgent::HumanTaskTimer(const ros::TimerEvent&){
	human_task_time += 1; // increase one in every second
}

// ********* WEB CLIENTS TO COMMUNICATE WITH DESPOT *********** //
// HUMAN ACTION IS RECEIVED !!!
bool ObservationAgent::action_to_obs_Map(hrc_ros::InformHumanAction::Request &req, // TODO check if InformHumanAction can still be used
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

		std_srvs::Trigger::Request req5_1;
		std_srvs::Trigger::Response resp5_1;
		is_a0_failed.call(req5_1, resp5_1);
		bool a0_failed = resp5_1.success;	// attemped to grasp but failed

		std_srvs::Trigger::Request req6;
		std_srvs::Trigger::Response resp6;
		is_a2.call(req6, resp6);
		bool a2 = resp6.success;	// action is staying idle

		std_srvs::Trigger::Request req7;
		std_srvs::Trigger::Response resp7;
		is_a4.call(req7, resp7);
		bool a4 = resp7.success;	// action is warn robot

		// ===== ADDING A NOISE TO THE OBSERVATIONS ========
		// TODO: update for toy example for the HW setup exp
		bool ov_noisy = ov;
		bool oir_noisy = oir;
		bool a0_noisy = a0;
		bool ipd_noisy = ipd; // success: O1
		bool a4_noisy = a4;
		bool a2_noisy = a2; // human idle: O7
		bool upd_noisy = upd; // failure: O2

		int r = rand() % 20;
		int m = rand() % 2; // confused or missed
		if ((a0 || a4) && r == 0){ // if one of this is one then 10% noise
			if (m == 0){ // 2.5 % chance of confusing a0 and a4
				a0_noisy = not a0;
				a4_noisy = not a4;
			} else if (m == 1){ // 2.5 % chance of missing the gesture and assuming idle (a2)
				a0_noisy = false;
				a4_noisy = false;
				a2_noisy = true;
			}
		} else if (((not ov) || a2) && r == 0){ // if either looking around (not ov) or idle is detected, 10 % chance of confusing them
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

		// Real observation is for the task manager to record statistics on running human models
		std::vector<uint8_t> real_obs_received;
		real_obs_received.push_back(not ov);
		real_obs_received.push_back(oir);
		real_obs_received.push_back(a0);
		real_obs_received.push_back(ipd);
		real_obs_received.push_back(a4);
		real_obs_received.push_back(a2);
		real_obs_received.push_back(upd);
		real_obs_received.push_back(a0_failed); // This information for recording human observable history. Processed and saved under TaskManager


		obs_update.real_obs_received = real_obs_received;

		// Noisy observation is for the robot
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

// TODO: change here, that only the communication to the despot is triggered
// ********* WEB CLIENTS TO COMMUNICATE WITH DESPOT *********** //
// HUMAN ACTION IS RECEIVED in Experiment setup !!!
bool ObservationAgent::IEaction_to_obs_Map(void) {

	ROS_INFO("In IEaction_to_obs_Map");
	//WebSocket (WS)-client at port 7070 using 1 thread
	WsClient client("localhost:7070");

	client.on_open=[&]() {

	ROS_INFO("client opened");

		//std_srvs::Trigger::Request req1;
		//std_srvs::Trigger::Response resp1;
		//is_ov.call(req1, resp1);
		//bool ov = resp1.success;	// object is visiable

		//std_srvs::Trigger::Request req2;
		//std_srvs::Trigger::Response resp2;
		//is_oir.call(req2, resp2);
		//bool oir = resp2.success;	// object is in range

		//bool ipd = ipd_sensor;	// inspected product is detected // SITUATION: INSPECTED PRODUCT DETECTED? (SUCCESS)
		//bool upd = upd_sensor;	// uninspected product is detected  // SITUATION: UNINSPECTED PRODUT DETECTED? (FAIL)
		/*
		std_srvs::Trigger::Request req3;
		std_srvs::Trigger::Response resp3;
		is_ho.call(req3, resp3);
		bool ho = resp3.success;	// have object
		*/
/*		std_srvs::Trigger::Request req5;
		std_srvs::Trigger::Response resp5;
		is_a0.call(req5, resp5);
		bool a0 = resp5.success;	// action is attemp grasp

		std_srvs::Trigger::Request req5_1;
		std_srvs::Trigger::Response resp5_1;
		is_a0_failed.call(req5_1, resp5_1);
		bool a0_failed = resp5_1.success;	// attemped to grasp but failed

		std_srvs::Trigger::Request req6;
		std_srvs::Trigger::Response resp6;
		is_a2.call(req6, resp6);
		bool a2 = resp6.success;	// action is staying idle

		std_srvs::Trigger::Request req7;
		std_srvs::Trigger::Response resp7;
		is_a4.call(req7, resp7);
		bool a4 = resp7.success;	// action is warn robot

		*/

		// ===== ADDING A NOISE TO THE OBSERVATIONS ========
		// TODO: update for toy example for the HW setup exp


		bool o4_ov_noisy =  o4_ov;
		bool o3_oir_noisy = o3_oir;
		bool o5_a0_noisy = o5_a0;
		bool o1_ipd_noisy = o1_ipd; // success: O1
		bool o6_a4_noisy = o6_a4;
		bool o7_a2_noisy = o7_a2; // human idle: O7
		bool o2_upd_noisy = o2_upd; // failure: O2

		int r = rand() % 20;
		int m = rand() % 2; // confused or missed
		if ((o5_a0 || o6_a4) && r == 0){ // if one of this is one then 10% noise
			if (m == 0){ // 2.5 % chance of confusing a0 and a4
				o5_a0_noisy = not o5_a0;
				o6_a4_noisy = not o6_a4;
			} else if (m == 1){ // 2.5 % chance of missing the gesture and assuming idle (a2)
				o5_a0_noisy = false;
				o6_a4_noisy = false;
				o7_a2_noisy = true;
			}
		} else if (((not o4_ov) || o7_a2) && r == 0){ // if either looking around (not o4_ov) or idle is detected, 10 % chance of confusing them
			o4_ov_noisy = not o4_ov;
			o7_a2_noisy = not o7_a2;
		}
		// ===================================================

		string robot_observation_real = "", observation = "", robot_observation_noisy = "";
		robot_observation_real = MapObservablesToObservations(o4_ov,o3_oir,o5_a0,o1_ipd,o6_a4,o7_a2,o2_upd);
		robot_observation_noisy = MapObservablesToObservations(o4_ov_noisy,o3_oir_noisy,o5_a0_noisy,o1_ipd_noisy,o6_a4_noisy,o7_a2_noisy,o2_upd_noisy);

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

		// Real observation is for the task manager to record statistics on running human models
		std::vector<uint8_t> real_obs_received;
		real_obs_received.push_back(not o4_ov);
		real_obs_received.push_back(o3_oir);
		real_obs_received.push_back(o5_a0);
		real_obs_received.push_back(o1_ipd);
		real_obs_received.push_back(o6_a4);
		real_obs_received.push_back(o7_a2);
		real_obs_received.push_back(o2_upd);
		// TODO how to gain grasp attempt failed ?? 
		//real_obs_received.push_back(o5_a0_failed); // This information for recording human observable history. Processed and saved under TaskManager


		obs_update.real_obs_received = real_obs_received;

		// Noisy observation is for the robot
		std::vector<uint8_t> obs_with_noise;
		obs_with_noise.push_back(not o4_ov_noisy);
		obs_with_noise.push_back(o3_oir_noisy);
		obs_with_noise.push_back(o5_a0_noisy);
		obs_with_noise.push_back(o1_ipd_noisy);
		obs_with_noise.push_back(o6_a4_noisy);
		obs_with_noise.push_back(o7_a2_noisy);
		obs_with_noise.push_back(o2_upd_noisy);

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


bool ObservationAgent::humanSt_to_robotSt_Map(hrc_ros::InformHumanState::Request &req,
		hrc_ros::InformHumanState::Response &res) {

	//WebSocket (WS)-client at port 7070 using 1 thread
	WsClient client("localhost:7070");

	client.on_open=[&]() {
		string realRbtSt_code = "";
		if (robotType == "reactive"){
			realRbtSt_code = getRealRbtStMDP(req.new_human_state); // this is the mapped human state to the robot's. Robot should estimate it correctly
		} else if (robotType == "proactive"){
			// TODO: update for toy example for the HW setup exp
			realRbtSt_code = getRealRbtStPOMDP(req.new_human_state); //For POMDP model
		}

		
		string message = "-1," + realRbtSt_code;

		ROS_INFO("OBSERVATION ROS: << robot_Type:  %s",robotType.c_str());

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
// ********************************** //

bool ObservationAgent::IE_humanSt_to_robotSt_Map(hrc_ros::InformHumanState::Request &req,
		hrc_ros::InformHumanState::Response &res) {

	//WebSocket (WS)-client at port 7070 using 1 thread
	WsClient client("localhost:7070");

	client.on_open=[&]() {
		string realRbtSt_code = "";
		if (robotType == "reactive"){
			realRbtSt_code = getRealRbtStMDP(req.new_human_state); // this is the mapped human state to the robot's. Robot should estimate it correctly
		} else if (robotType == "proactive"){
			// TODO: update for toy example for the HW setup exp
			realRbtSt_code = getRealRbtStPOMDP(req.new_human_state); //For POMDP model
		}

		
		string message = "-1," + realRbtSt_code;

		ROS_INFO("OBSERVATION ROS: << robot_Type:  %s",robotType.c_str());

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



// *** Service handler that receives a tray update message and calculates the observables success and failure 
//void ObservationAgent::IEtray_update_to_obs_map(const hrc_ros::TrayUpdateCamera &msg){
bool ObservationAgent::IE_receive_tray_update(hrc_ros::InformTrayUpdate::Request &req,hrc_ros::InformTrayUpdate::Response &res){
	
	
  /*o6_a4						// O_6  warning received
	o7_a2						// O_7  Idle
	o4_ov  						// O_4  Human is not looking around  
	o3_oir						// O_3  Human is detected 
	o5_a0						// O_5  grasping attempt
	*/

	current_object = req.current_object;  // or recalculate from tray_object_combination
	tray_object_combination = req.tray_obj_combination; 

	// mapping tray status to observables ( o1 = success | o2 = failure )
	

	if (true_tray_object_combination == tray_object_combination){
		o1_ipd = true; //  O_1  task successs (processed product detected)
		o2_upd = false; // O_2	failure 
	} else {
		o1_ipd = false; //  O_1  task successs (processed product detected)
		o2_upd = true; // O_2	failure 
	}
	
	ROS_INFO("OBSERVATION ROS: ## TrayUpdate_Camera  RECEIVED ##");
	ROS_INFO(" Tray object combination is %d",req.tray_obj_combination);
	ROS_INFO("********\n\n\n");

	//tray_msg_stamp = msg.stamp;

	// get the rule for the single task at hand by browsing the current task rule sets // TODO test this and implement task_rules updater 
	//current_task_rule = task_rules[msg.current_object] // TODO task_rules should be a global variable filled once 

	/*if (msg.tray_obj_combination == current_task_rule) { 		// success in single task => ipd_sensor = true 
		//ipd_sensor = true; 
		//upd_sensor = false;
		ipd_O1	= true;				// O_1  task successs (processed product detected)
		upd_O2	= false; 				// O_2	failure 

	} else if (msg.tray_obj_combination != current_task_rule) { // failure in single task => upd_sensor = true 
		//ipd_sensor = false; 
		//upd_sensor = true; 
		ipd_O1	= false;				
		upd_O2	= true; 				
	*/

	std_msgs::String success_status_msg;

    success_status_msg.data = string("success");
	traySensor_success_pub.publish(success_status_msg);
	ros::spinOnce();
	
	// trigger decision !!!! 
	// TODO change this to the actual trigger function 
	bool mapping_success = ObservationAgent::IEaction_to_obs_Map();

	//TODO broadcast the tray_update here -> success or failure   



	

   /* int r = rand() % 3;
	if(r== 1){
		return true; 
	} else {
		return false; 
	} */

	ROS_INFO("Mapping success: %d" ,mapping_success);
	res.success = true;
	return true; 
}


// *** Service handler that receives a classified action and calculates the observables 
bool ObservationAgent::IE_receive_actionrecognition_update(hrc_ros::InformActionRecognized::Request &req, hrc_ros::InformActionRecognized::Response &res){
	
	
	if (req.action == "warning"){
		o6_a4 = true;
	} else{
		o6_a4 = false; 
	} 

	if (req.action == "idle"){
		o7_a2 = true; 
	} else {
		o7_a2 = false; 
	}
  
	if (req.action == "grasping"){ // O_5  grasping attempt
		o5_a0 = true; 
	} else {
		o5_a0 = false;
	}

	 o3_oir = req.human_detected;            // O_3  Human is detected 
	 o4_ov  = not(req.human_looking_around);  // O_4  Human is not looking around  

	ROS_INFO("\n\nOBSERVATION ROS: ## ActionRecognition update received  RECEIVED ##");
	ROS_INFO(" Action %s",req.action.c_str());
	ROS_INFO("Human detected =  %d", o3_oir);
	ROS_INFO("Human looking around = %d", o4_ov);
	ROS_INFO("********\n\n\n");

	//tray_msg_stamp = msg.stamp;

	// get the rule for the single task at hand by browsing the current task rule sets // TODO test this and implement task_rules updater 
	//current_task_rule = task_rules[msg.current_object] // TODO task_rules should be a global variable filled once 

	/*if (msg.tray_obj_combination == current_task_rule) { 		// success in single task => ipd_sensor = true 
		//ipd_sensor = true; 
		//upd_sensor = false;
		ipd_O1	= true;				// O_1  task successs (processed product detected)
		upd_O2	= false; 				// O_2	failure 

	} else if (msg.tray_obj_combination != current_task_rule) { // failure in single task => upd_sensor = true 
		//ipd_sensor = false; 
		//upd_sensor = true; 
		ipd_O1	= false;				
		upd_O2	= true; 				
	*/

	
	
	// trigger decision !!!! 
	// TODO change this to the actual trigger function 
	bool mapping_success = ObservationAgent::IEaction_to_obs_Map();
   /* int r = rand() % 3;
	if(r== 1){
		return true; 
	} else {
		return false; 
	} */

	ROS_INFO("Mapping success: %d" ,mapping_success);
	res.success = true;
	return true; 
}


// *** Service handler that receives a human action update message (once it has been recognized)  and calculates the observables success and failure 
/*void ObservationAgent::IEaction_recognized_update_to_obs_map(){ // hrc_ros::TraySensor_IE_9D &msg
	tray_msg_stamp = msg.stamp;

	//ov_O4	= false;				// O_4  Human is not looking around   => TODO supply, once head gesture recognition is ready 
	oir_O3	= msg.O3;				// O_3  Human is detected 
	a0_O5	= msg.O5;				// O_5  grasping attempt
	ipd_O1	= msg.O1;				// O_1  task successs (processed product detected)
	a4_O6	= msg.O6;				// O_6  warning received
	a2_O7	= msg.O7; 				// O_7  Idle

	
	
	// trigger decision !!!! 
	// TODO change this to the actual trigger function 
	//bool ObservationAgent::IEaction_to_obs_Map(hrc_ros::InformHumanAction::Request &req,hrc_ros::InformHumanAction::Response &res) {
}
*/


// TODO  delete

// ********* CALLBACKS *********** //
/*void ObservationAgent::ReceiveTraySensors(const hrc_ros::TraySensor &msg){
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
*/
// ********************************** //



// ********* CALLBACKS *********** //

// TODO  test with new topic once it is present 
//        change message type !!! 

// upd = failure in task step
// ipd = success in task step 
/*void ObservationAgent::ReceiveTrayDetection9D(const hrc_ros::TraySensor &msg){
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
*/
// ********************************** //



// ********* MAIN OPERATIONS: MAPPING OF HUMAN OBSERVATIONS AND STATES TO ROBOT OBSERVATIONS AND STATES FOR DESPOT *********** //
void ObservationAgent::humanStCounter(string humanState){

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

string ObservationAgent::getRealRbtStPOMDP(string humanState)
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

	// LEAVE THESE AS IT IS
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

string ObservationAgent::MapObservationsToPOMDP(string observable){


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
string ObservationAgent::MapObservablesToObservations(bool ov, bool oir, bool a0, bool ipd, bool a4, bool a2, bool upd){
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

string ObservationAgent::getRealRbtStMDP(string humanState)
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

string ObservationAgent::MapObservationsToMDP(string observation) {
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
			else
				robot_state = "HumanNeedsHelp";
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
// ********************************** //
