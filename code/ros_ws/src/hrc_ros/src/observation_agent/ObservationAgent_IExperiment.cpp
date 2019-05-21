/*
 *  Created on: 19.04.2018
 *      Author: Orhan Can Görür
 *      Email: orhan-can.goeruer@dai-labor.de
 *  Modified on: 27.03.2019
 * 			Author: Elia Kargruber and Orhan Can Görür
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <stdlib.h> 				// for rand() and RAND_MAX
#include <string>
#include <thread>

#include <observation_agent/ObservationAgent_IExperiment.h>
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"


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
	 *Ros Publishers and Subscriber initialization
	 *
	 */
	traySensor_success_pub = nh.advertise<hrc_ros::SuccessStatusObserved>("/observation_agent/observedsuccess_status", 1);
	traySensor_notifyToHuman_pub = nh.advertise<hrc_ros::SuccessStatusObserved>("/observation_agent/tray_status_NotifyToHuman",1);
	//subscriber for the headGesture
	headGesture_subs = nh.subscribe("/headTrackingAgent/head_gesture_pub/", 1 , &ObservationAgent::ReceiveHeadGesture,this);
	ObsUpdaterPub = nh.advertise<hrc_ros::ObsUpdateMsgIE>("/observation_agent/observation_update",1);

	/*
	 * ROS Services initialization
	 */
	IEaction_recognition_server = nh.advertiseService("/observation_agent/inform_action_recognized", &ObservationAgent::IE_receive_actionrecognition_update, this);
	IEtray_update_server = nh.advertiseService("/observation_agent/inform_tray_update", &ObservationAgent::IE_receive_tray_update, this);
	reset_scenario = nh.advertiseService("/observation_agent/reset", &ObservationAgent::resetScenario, this);
	IErequest_successcriteria_server = nh.advertiseService("/observation_agent/request_success_criteria", &ObservationAgent::IE_request_success_criteria, this);
		ObsUpdater = nh.serviceClient<hrc_ros::InformObsToTaskMangIE>("/task_manager/observation_update");  // Client for the task manager service for the status update

	/*
	 * Initializing global variables
	 */
	successful_subtasks = 0;
	failed_subtasks = 0;

	// ######### Reset variables for staticstics ################
	// statistics that are sent to taskmanager
	failed_subtasks = 0;
	successful_subtasks = 0;
	subtask_time_seconds = 0.0;
	task_time_sumOfSubtasks_sec = 0.0; // task_combined_subtask_time_seconds in message
	percentage_successful_subtasks = 0.0;
	who_succeeded = "NOBODY";
	warnings_received_task = 0;
	warnings_received_subtask =0; 
	successful_tasks_cnt = 0;
	failed_tasks_cnt = 0;
	percentage_successful_tasks = 0.0;


	// other variables
	o1_ipd = false; //  O_1  task successs (processed product detected)
	o2_upd = false; // O_2	failure
	immediate_reward_IE = 0.0;
	discounted_reward_IE = 0.0;
  subtask_timer_tick = 0;


	/// A ROS timer for the duration of a task assigned to the human
	task_timer = nh.createTimer(ros::Duration(1.0), &ObservationAgent::HumanTaskTimer, this);
	decision_timer = nh.createTimer(ros::Duration(10.0),&ObservationAgent::DecisionTimer,this);
	decision_timer.stop();

	/// subtask_timer measuring how long human // robot takes to finish a subtask -> used for reward calculation
	subtask_timer = nh.createTimer(ros::Duration(1.0),&ObservationAgent::SubTaskTimer,this);
	subtask_timer.stop();

	/// making sure that experiment_started is set to false -> tray_status and observations are not considered until the task manager initiates the experiment
	experiment_started = false;


	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
	ros::console::notifyLoggerLevelsChanged();
	ROS_INFO("[Observation Agent] is created !");
	ros::spin();
}


// called at beginning of new task by task manager
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

	// get task_counter & reset subtask_counter
	task_counter = req.task_cnt;
	subtask_counter = 1;

	// ######### Reset variables for staticstics ################
	// statistics that are sent to taskmanager
	failed_subtasks = 0;
	successful_subtasks = 0;
	subtask_time_seconds = 0.0;
	task_time_sumOfSubtasks_sec = 0.0; // task_combined_subtask_time_seconds in message
	percentage_successful_subtasks = 0.0;
	who_succeeded = "NOBODY";
	warnings_received_task = 0;
	warnings_received_subtask = 0; 
	// other variables
	o1_ipd = false; //  O_1  task successs (processed product detected)
	o2_upd = false; // O_2	failure
	immediate_reward_IE = 0.0;
	discounted_reward_IE = 0.0;
  subtask_timer_tick = 0;


	//cout << endl << endl << " ###################  task_counter received: =  " << task_counter << "  subtask_counter  = " << subtask_counter <<  endl << endl;

	// Read in the task_scenario -> this info is used to elaborate subtask success or failure

        ROS_INFO("[Observation_Manager]: Parsing task scenario definition file @initialisation");

        // Call the parse function to parse the scenario definition file
            string pkg_path1 = ros::package::getPath("hrc_ros");
            std::ifstream jsonFiletask(pkg_path1 + "/../../../configs/IE_task_config.json");

            try {
                boost::property_tree::read_json(jsonFiletask, testscenario_pt);
            } catch(boost::property_tree::json_parser::json_parser_error &e) {
                ROS_FATAL("Cannot parse the message to Json. Error: %s", e.what());
                res.success = false;
                return false;
            }

	// get global task_configuration setting (criterias for global success and failure) once per task
	// get task related number of subtask
	global_task_configuration_read = read_global_task_config(testscenario_pt);
	stringstream s_stream;
	s_stream << task_counter;
	string task_str = s_stream.str();
	current_subtask_quantity = get_subtask_quantity(task_str,testscenario_pt);

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

	// start timers
	decision_timer.stop();
	decision_timer.setPeriod(ros::Duration(global_task_configuration_read.decision_timer_periode));
	decision_timer.start();

	//cout << "Subtask Counter started now " << endl;
	subtask_start_time = ros::Time::now();
	task_time_sumOfSubtasks_sec = 0.0;
	subtask_timer.stop();
	subtask_timer.start();

	ipd_sensor = false;	// inspected product detector sensor
	upd_sensor = false;	// uninspected product detector sensor
	ROS_INFO("OBSERVATION ROS: Reset!");
	res.success = true;

	/// set experiment_started to true -> indicates that task_manager started experiment => afterwards the tray_status and observations are considered and decisions are triggered
	if (not experiment_started) {
		experiment_started = true;
		ROS_WARN("OBSERVATION ROS:  Experiment started -> accepting actions & tray updates now!!!  \n\n ");
	}

	return true;
}


// TODO change this to subtask_time and use it for the calculation of rewards
void ObservationAgent::HumanTaskTimer(const ros::TimerEvent&){
	human_task_time += 1; // increase one in every second
}


void ObservationAgent::DecisionTimer(const ros::TimerEvent &event){

	 if (experiment_started) {
	 // Note the duration printout is wrong for the first time when the timer has been restarted
	 double seconds_real = ( ((event.current_real - event.last_real).toNSec() ) / double(1000000000.0) );
	 ROS_WARN("XXXXXX  DecisionTimer Event because Timeout reached after   : %f  seconds \n\n" , seconds_real);

	 // trigger a decision
	 // bool mapping_success = ObservationAgent::IEaction_to_obs_Map();
	 hrc_ros::InformActionRecognized::Request req;
	 hrc_ros::InformActionRecognized::Response res;
	 req.human_detected = false;
	 req.stamp = ros::Time::now();
	 // req.human_looking_around = true;
	 ObservationAgent::IE_receive_actionrecognition_update(req, res);
	 } else { ROS_WARN("\nOBSERVATION ROS:  Decission timer is running but experiment not started yet -> no decision will be taken \n                  to start the Experiment, call:	 rosservice call /task_manager_IE/new_scenario_request  \n\n"); }

}
void ObservationAgent::SubTaskTimer(const ros::TimerEvent&){

	subtask_timer_tick ++;
	//cout << "subtask_timer_tick " << subtask_timer_tick << endl;

}


// ********* WEB CLIENTS TO COMMUNICATE WITH DESPOT *********** //
// HUMAN ACTION IS RECEIVED in Experiment setup !!!
bool ObservationAgent::IEaction_to_obs_Map(void) {

	ROS_INFO("In IEaction_to_obs_Map");
	//WebSocket (WS)-client at port 7070 using 1 thread
	WsClient client("localhost:7070");

	client.on_open=[&]() {

		string robot_observation_real = "", observation = "", robot_observation_noisy = "";
		robot_observation_real = MapObservablesToObservations(o4_ov,o3_oir,o5_a0,o1_ipd,o6_a4,o7_a2,o2_upd,int_subtask_status);

		// track number of warnings received for statistics
		if ( o6_a4 == true){
			warnings_received_task ++;
			warnings_received_subtask ++;
		}

		if (robotType == "reactive"){
			observation = MapObservationsToMDP(robot_observation_real); // Get correspending state for reactive ROBOT wrt observations to state mapping
		} else if (robotType == "proactive"){
			observation = MapObservationsToPOMDP(robot_observation_real); // Get correspending observation for proactive ROBOT wrt observables received
		}

		ROS_INFO("[OBSERVATION AGENT]: Sending to DESPOT: real_observable: %s, mapped observation: %s",
				robot_observation_real.c_str(), observation.c_str());
		string message = observation + ",-1"; // It is only sending observed_state, real state is send in another iteration (when it is provided)

		// ############ Calculating reward for interaction experiment ###################
	  calculate_reward_IE(observation, immediate_reward_IE, discounted_reward_IE);
		//cout << "immediate_reward " <<  immediate_reward_IE << "   discounted_reward "<< discounted_reward_IE << endl;
		string reward_ = std::to_string(immediate_reward_IE);
		ros::param::set("/robot_immediate_reward", reward_);
		reward_= std::to_string(discounted_reward_IE);
		ros::param::set("/robot_total_disc_reward", reward_);

		// ############ SENDING OBSERVATIONS TO TASK MANAGER ############

		hrc_ros::ObsUpdateMsgIE obs_update;

		hrc_ros::InformObsToTaskMangIE::Request reqForUpdate;
		hrc_ros::InformObsToTaskMangIE::Response resForUpdate;

		obs_update.stamp_obs_update = ros::Time::now();

		// ********* NOTE Real observations and noisy observations are the same in IE. The received ones are
		// Real observation is for the task manager to record statistics on running human models
		std::vector<uint8_t> real_obs_received;
		real_obs_received.push_back( uint8_t(not o4_ov) ); // O_4  Human is not looking around
		real_obs_received.push_back( uint8_t(o3_oir) 		); // O_3  Human is detected
		real_obs_received.push_back( uint8_t(o5_a0) 		); // O_5  grasping attempt
 		real_obs_received.push_back( uint8_t(o1_ipd) 		); // O_1  task successs (processed product detected)
		real_obs_received.push_back( uint8_t(o6_a4)     ); // O_6  warning received
		real_obs_received.push_back( uint8_t(o7_a2)			); // O_7  Idle
		real_obs_received.push_back( uint8_t(o2_upd)		); // O_2	failure
		real_obs_received.push_back( uint8_t(((char)int_subtask_status)) ); // subtask-status --- 0 = ongoing | 1 = success | 3 = fail
		// TODO how to gain grasp attempt failed ??
		//real_obs_received.push_back(o5_a0_failed); // This information for recording human observable history. Processed and saved under TaskManager


		obs_update.real_obs_received = real_obs_received;
		obs_update.mapped_observation_pomdp = std::stoi(observation);
		obs_update.mapped_observation_raw   = std::stoi(robot_observation_real);
		// TODO TEST this as now it is sent by robot agent


		// assign obs_update message to InformObsToTaskMang service
		reqForUpdate.obs_update = obs_update;
		ObsUpdater.call(reqForUpdate, resForUpdate);

		// also publish the message for logging
		ObsUpdaterPub.publish(obs_update);

		// ###########################################################################################

		ROS_WARN("\n\n OBSERVATION Client: TRIGGER -- Sending to the robot planner:: OBSERVATION= %s \n\n", message.c_str());
		auto send_stream=make_shared<WsClient::SendStream>();
		*send_stream << message;
		client.send(send_stream);
	};

	client.on_message=[&client](shared_ptr<WsClient::Message> message) {
		client.send_close(1000);
	};

	client.start();

	// Delay next decision making if human is grasping
	/*if (o5_a0 == true){
		ros::Duration(1).sleep();
		ROS_WARN("\nhuman grasping -> delay next decision for 4s \n");
	} */

	return true;
}



// map the real_state to DESPOT format and send it to DESPOT, real-state is not used by DESPOT but remains for synchronisation purposes
bool ObservationAgent::IE_humanSt_to_robotSt_Map(string real_human_state_observed) {

	//WebSocket (WS)-client at port 7070 using 1 thread
	WsClient client("localhost:7070");

	client.on_open=[&]() {
		string realRbtSt_code = "";
		// TODO: remove if reactive model is not used anymore
		if (robotType == "reactive"){
			realRbtSt_code = getRealRbtStMDP(real_human_state_observed); // this is the mapped human state to the robot's. Robot should estimate it correctly
		} else if (robotType == "proactive"){
			// TODO: update for toy example for the HW setup exp
			realRbtSt_code = getRealRbtStPOMDP(real_human_state_observed); //For POMDP model
		}


		string message = "-1," + realRbtSt_code;

		ROS_INFO("OBSERVATION ROS: << robot_Type:  %s",robotType.c_str());
		// TODO remove
		ROS_INFO("observed state received was  = %s", real_human_state_observed.c_str());

		ROS_INFO("OBSERVATION Client: Sending to the robot planner:: REAL STATE WAS = %s", message.c_str());
		auto send_stream=make_shared<WsClient::SendStream>();
		*send_stream << message;
		client.send(send_stream);
	};

	client.on_message=[&client](shared_ptr<WsClient::Message> message) {
	////cout << "Client: Sending close connection" << endl;
	client.send_close(1000);
	};

	client.start();

	return true;
}


// *** Callback function for the /headGestureAgent/head_gesture_pub/ topic"
void ObservationAgent::ReceiveHeadGesture(const hrc_ros::HeadGestureMsg &msg){
    // TOCONSIDER currently the head_gesture is only updated for the decision making if a new observation is received. In case it should always be up-to date, independent of a observation update then: o4_ov should be used instead
		notO4_human_looking_around = msg.humanLookingAround;

		////cout << "\n\n received Head Gesture  | HumanLookingAround =   " << notO4_human_looking_around << endl;

}


void ObservationAgent::calculate_reward_IE(string obs , float &immediate_reward_out, float &discounted_reward_out){

	int obs_int = stoi(obs);
	float tmp_immediate_rew = 0.0;
	float tmp_discounted_rew = discounted_reward_out;

	int discount_tick = subtask_timer_tick; // it is either subtask_counter_tick or the last_tick before reset in subtask_success or subtask_fail_case
	//cout << "calculating_reward :: obs_int= " << obs_int <<" Raw string  " << obs << endl;

	// TODO check if other rewards need to be given as well
	switch (obs_int){

		case 0:

			break;

		case 1:

			break;

		case 2:

			break;

		case 3:

			break;

		case 4:

			break;

		case 5:

			break;

		case 6: // warning
		tmp_immediate_rew  = -3;
			break;

		case 7:

			break;

		case 8:

			break;

		case 9:

			break;

		case 10:

			break;

		case 11:

			break;

		case 12:  // subtask_success
			tmp_immediate_rew  = 6;
			discount_tick = before_subtask_reset_tick;
			break;

		case 13:  // subtask_fail
			tmp_immediate_rew  = -6;
			discount_tick = before_subtask_reset_tick;
			break;
	}

	// TODO: check if this rewarding mechanism is ok for the cases when robot succeeds or somehow failed in placement
	if (isRobotSucceed){ // this variable set after robot's successfull grasp action, which is not informed within obs received
		tmp_immediate_rew += 6;
		isRobotSucceed = false; // it is processed so being removed
	} else if (isRobotFailed){
		tmp_immediate_rew -= 6;
		isRobotFailed = false; // it is processed so being removed
	}

	//tmp_discounted_rew += pow(discount_factor,(subtask_counter -1) ) * tmp_immediate_rew;  // former reward calculation -> discounting per subtask step
	//cout << " Calculating reward : tick_is   " << discount_tick << endl;
	tmp_discounted_rew += pow(discount_factor,(discount_tick) ) * tmp_immediate_rew;

	// returns by reference
	immediate_reward_out  = tmp_immediate_rew;
	discounted_reward_out = tmp_discounted_rew;
}


// *** Service handler that returns the current success_criteria - this is used by DOBOT to place an object
bool ObservationAgent::IE_request_success_criteria(hrc_ros::RequestSuccessCriteria::Request &req, hrc_ros::RequestSuccessCriteria::Response &res){

		// ## get strings for success_criteria request
		stringstream ss_task_counter;
		stringstream ss_subtask_counter;

		ss_task_counter << task_counter;
		ss_subtask_counter << subtask_counter;

		current_object = req.current_object;
		string task_str = ss_task_counter.str();
		string subtask_str = ss_subtask_counter.str();
		string object_str  = object_int_to_str(current_object);

		// ## determine subtask success state
		try {
		success_criteria_read = get_success_criteria(task_str,subtask_str,object_str,testscenario_pt);
		} catch(boost::property_tree::json_parser::json_parser_error &e) {
				ROS_FATAL("Cannot parse the message to Json. Error: %s", e.what());
				return false;
			}

  		res.stamp = ros::Time::now();
		res.tray   = success_criteria_read.tray;


}


// *** Service handler that receives a tray update message and calculates the observables success and failure
bool ObservationAgent::IE_receive_tray_update(hrc_ros::InformTrayUpdate::Request &req,hrc_ros::InformTrayUpdate::Response &res){

	//Observables calculated in this function are:  (other observables are calculated in action_update_received callback)
	// o1_ipd   O_1  task successs (processed product detected)
  // o2_upd = O_2	failure

	if (experiment_started) {
		// Reset decision timer ( timer only triggers if system is stuck )
		decision_timer.stop();
		decision_timer.start();
		subtask_timer.stop();
		before_subtask_reset_tick = subtask_timer_tick;
		subtask_timer_tick = 0;
		ros::Time subtask_stop_time = ros::Time::now();
		subtask_duration = subtask_stop_time - subtask_start_time;
		task_time_sumOfSubtasks_sec +=  subtask_duration.toNSec() * std::pow(10.0, (-9.0) );
		////cout << " last subtask took "  <<  subtask_duration.toNSec() * (std::pow(10.0, (-9.0) ) ) <<  "  Combined task time in sec: " << task_time_sumOfSubtasks_sec << "     before_subtask_reset_tick     " << before_subtask_reset_tick << endl;

		// start timers and counters for new subtask
		subtask_start_time = ros::Time::now();
		subtask_timer.start();


		// #### mapping tray status to observables ( o1 = success | o2 = failure ) ####
		current_object = req.current_object;
		string task_success_state = "ongoing";
		string subtask_success_state = "ongoing";
		int_subtask_status = 0; // 0 = ongoing | 1 = success | 3 = fail
		hrc_ros::SuccessStatusObserved success_status_msg;


		// ## get strings for success_criteria request
		stringstream ss_task_counter;
		stringstream ss_subtask_counter;

		ss_task_counter << task_counter;
		ss_subtask_counter << subtask_counter;

		string task_str = ss_task_counter.str();
		string subtask_str = ss_subtask_counter.str();
		string object_str  = object_int_to_str(current_object);

		// ## determine subtask success state
		try {
		success_criteria_read = get_success_criteria(task_str,subtask_str,object_str,testscenario_pt);
		} catch(boost::property_tree::json_parser::json_parser_error &e) {
				ROS_FATAL("Cannot parse the message to Json. Error: %s", e.what());
				res.success = false;
				return false;
			}


		int success_tray_read = success_criteria_read.tray;
		if (req.current_tray == success_tray_read){

			o1_ipd = false; //  O_1  task successs (processed product detected)
			o2_upd = false; // O_2	failure

			subtask_success_state = "success";
			int_subtask_status = 1;
			successful_subtasks += 1;

		} else {
			o1_ipd = false; //  O_1  task successs (processed product detected)
			o2_upd = false; // O_2	failure

			subtask_success_state = "fail";
			int_subtask_status = 3;
			failed_subtasks += 1;
		}


		// ## determine global success state -> it is only set once all subtasks are finished
		if ( subtask_counter >= current_subtask_quantity ) {  // all subtasks done

			// reset experiment started -> will be set again when new experiment is started
			experiment_started = false;

			ROS_WARN("Global Success state is calculated");
			if (successful_subtasks >= global_task_configuration_read.global_success_assert ){
				task_success_state = "success"; // global success
				successful_tasks_cnt ++;
				o1_ipd = true; //  O_1  task successs (processed product detected)
				o2_upd = false; // O_2	failure
				decision_timer.stop();
				subtask_timer.stop();
				subtask_timer_tick = 0;
				//TODO remove
				//cout <<  " Global Success | successful_subtasks = " << successful_subtasks << "  global_success_criteria = " << global_task_configuration_read.global_success_assert << endl;
			} else if ( failed_subtasks >= global_task_configuration_read.global_fail_assert){
				task_success_state = "fail"; // global fail
				failed_tasks_cnt ++;
				o1_ipd = false; //  O_1  task successs (processed product detected)
				o2_upd = true; // O_2	failure
				decision_timer.stop();
				subtask_timer.stop();
				subtask_timer_tick = 0;
				// TODO remove
				//cout <<  " Global Fail | failed_subtasks = " << failed_subtasks << "  global_fail_criteria = " << global_task_configuration_read.global_fail_assert << endl;
			}

		}

		// TODO add task time to success_status_msg and publish by task manager
		// ## Compose success_status_msg
		success_status_msg.stamp = ros::Time::now();
		success_status_msg.subtask_success_status = subtask_success_state;
		success_status_msg.task_success_status = task_success_state;

		// ## fields for debugging
		success_status_msg.current_object = current_object;
		success_status_msg.current_tray = req.current_tray;
		success_status_msg.success_tray = success_tray_read;
		success_status_msg.task_counter = task_counter;
		success_status_msg.subtask_counter = subtask_counter;


		// ## fields for statistics
		success_status_msg.failed_subtasks = failed_subtasks;
		success_status_msg.successful_subtasks = successful_subtasks;
			subtask_time_seconds = subtask_duration.toNSec() * (std::pow(10.0, (-9.0) ) );
		success_status_msg.subtask_time_seconds = subtask_time_seconds;
		success_status_msg.task_combined_subtask_time_seconds = task_time_sumOfSubtasks_sec;
			percentage_successful_subtasks = ( double(successful_subtasks) / double(subtask_counter) ) * double(100.0) ;
		success_status_msg.percentage_successful_subtasks = percentage_successful_subtasks;
		success_status_msg.who_succeeded = "HUMAN";
		success_status_msg.task_warnings_received = warnings_received_task;
		success_status_msg.subtask_warnings_received =	warnings_received_subtask;
		success_status_msg.successful_tasks_cnt = successful_tasks_cnt;
		success_status_msg.failed_tasks_cnt     = failed_tasks_cnt;
			percentage_successful_tasks = (double(successful_tasks_cnt) / double(successful_tasks_cnt + failed_tasks_cnt) ) * double(100.0);
		success_status_msg.percentage_successful_tasks = percentage_successful_tasks;

		// ## Notify Human about the successful detection of a tray update, e.g. by sound 
		traySensor_notifyToHuman_pub.publish(success_status_msg);

		// ## Trigger a decision
		bool mapping_success = ObservationAgent::IEaction_to_obs_Map();
		int_subtask_status = 0; // reset to ongoing

		float final_immediate_reward_IE = 0; 
		float final_discounted_reward_IE = 0; 
		final_immediate_reward_IE = immediate_reward_IE;
		final_discounted_reward_IE = discounted_reward_IE;
		// ############ if final state is reached it should also be informed to the POMDP -> the pomdp will terminate afterwards
		if (task_success_state.compare("success") ==0 ){
			//cout << endl << endl << "GlobalSuccess will be sent to POMDP -> it will terminate afterwards" << endl;
			IE_humanSt_to_robotSt_Map("GlobalSuccess");
			calculate_reward_IE("12",final_immediate_reward_IE,final_discounted_reward_IE);
		} else if (task_success_state.compare("fail")==0) {
			//cout << endl << endl << "GlobalFail will be sent to POMDP -> it will terminate afterwards" << endl;
			IE_humanSt_to_robotSt_Map("GlobalFail");
			calculate_reward_IE("13",final_immediate_reward_IE,final_discounted_reward_IE);
		}

		success_status_msg.task_finished_immediate_reward =  final_immediate_reward_IE;
		success_status_msg.task_finished_discounted_reward = final_discounted_reward_IE;
		// ########################  Before the tray status was published here -> after decision making ########
		// ############ Publish success_status_msg (mainly used by task manager to check if a new task should be started)
		traySensor_success_pub.publish(success_status_msg);
		
		// ## Human can also be notified after a decision is taken, in case the decision making blocks for too long 
		//traySensor_notifyToHuman_pub.publish(success_status_msg);

		// ## increment subtask counter
		subtask_counter += 1;


		// ### Reset variables that are relevant for 1 subtask only 

		warnings_received_subtask = 0; 


		ROS_INFO("\n\n");
		ROS_WARN("OBSERVATION ROS: #### TrayUpdate_Camera  RECEIVED #### \n\n");
		//ROS_INFO(" Tray object combination is %d",req.tray_obj_combination);
		ROS_INFO(" Current_object %d",req.current_object);
		ROS_INFO(" Current_tray %d",req.current_tray);
		ROS_INFO("subtask_success_state = %s",subtask_success_state.c_str());
		ROS_INFO("task_success_state = %s \n ",task_success_state.c_str());
		ROS_INFO("Failed_subtasks = %d  ",failed_subtasks);
		ROS_INFO("successful_subtasks = %d  ",successful_subtasks);
		ROS_INFO("********\n\n\n");


		//cout << endl << " ###############  Current task counters and success states ######################## " << endl << endl;
		//cout << "task_counter = " << task_counter << "  subtask_counter = " << subtask_counter << " subtask_success_state = " << subtask_success_state << endl << endl;

		ROS_INFO("Mapping success: %d \n \n \n" ,mapping_success);
		res.success = true;

	} else { ROS_WARN("\nOBSERVATION ROS:  Tray_update received but experiment not started yet -> tray_update is dismissed \n                  to start the Experiment, call:	 rosservice call /task_manager_IE/new_scenario_request  \n\n"); }  // if experiment is not running yet
	return true;
}


// *** Service handler that receives a classified action and calculates the observables and triggers the new decision-making process
bool ObservationAgent::IE_receive_actionrecognition_update(hrc_ros::InformActionRecognized::Request &req, hrc_ros::InformActionRecognized::Response &res){

	bool informTrayupdateTaskmanager = false;

	if (experiment_started) {

		// Reset decision timer ( timer only triggers if system is stuck )
		decision_timer.stop();
		decision_timer.start();

		string observation_mapped = "TaskHuman"; // only WarningReceived, GlobalSuccess, and GlobalFail are relevant

		if (req.action == "warning"){		//O6
			o6_a4 = true;
			observation_mapped = "WarningTheRobot";
		} else{
			o6_a4 = false;
		}

		if (req.action == "idle"){			// O7
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
		o4_ov  = not(notO4_human_looking_around);  // O_4  Human is not looking around (=> global variable received by head_gesture sub) | o4_ov is only updated for decision making when new observation is detected

		ROS_INFO("[OBSERVATION AGENT] ##### ActionRecognition update received  RECEIVED #####");
		ROS_INFO(" Action %s     			| warning = O6 | Idle = O7",req.action.c_str());
		ROS_INFO("Human detected (O3) =  %d", o3_oir);
		ROS_INFO("Human NOT looking around (!O4) = %d", o4_ov);
		ROS_INFO("observation_mapped =  %s",observation_mapped.c_str());
		ROS_INFO("********\n\n\n");


		ROS_INFO("XXXXXX  Time_passed_since_new_action   : %f \n\n" , (req.stamp - former_time_stamp).toSec() );

		// ==== CHECKING THE ROBOT ACTION BEFORE TRIGERRING NEW DECISION ====
		// TODO: Work in progress for syncing of decision-making
		// Rules:
		// *  if robot has received a warning no matter what --> allow new decision AND increase subtask count
		// * 	if robot is grasping and no warnings received --> ignore new obs
		// *  if robot has succeeded in grasp, increase the subtask_count
		// *
		allowDecisionMaking = true;
		int dobot_grasp_state = -2; // -2= initial -1= grasp_planning | 0=ongoing | 1= grasping finished successfully 3=warning received | 4=timeout or other error | 5=empty conveyor | 6 = warning in progress
		ros::param::get("/robot_grasping_state", dobot_grasp_state);
		////cout << "grasp_state: " << dobot_grasp_state << endl;
		// TODO: WARNING WONT BE RECEIVED AT ALL AS LONG AS WE WONT LET DECISION-MAKING DECIDE.
		// see the warning_received_flag set in DobotWorkerNode which is set after cancellCallback. That is why I added o6_a4 check as well
		if (dobot_grasp_state == 3 || o6_a4) { // warning received during grasp
			//subtask_counter += 1; // TODO: should we assume this as a subtask failure?
			//cout << "warning received during grasp" << endl;
			ros::param::set("/robot_grasping_state",-2); // reset to -2 if grasp final state received
			allowDecisionMaking = true;
		}/*else if (dobot_grasp_state == -1){ // grasp planning is ongoing
			//cout << "robot is planning for grasp, core is busy ..." << endl;
			allowDecisionMaking = false; } */
		else if (dobot_grasp_state == 6) { //cancel action is ongoing
			allowDecisionMaking = false;
		}
		else if(dobot_grasp_state == 0){ // robot's grasping and placing is in progress
			//cout << "robot grasping in progress" << endl;
			allowDecisionMaking = false;
		} else if(dobot_grasp_state == 1){ // robot succeeded -> grasp success
			//cout << "robot grasped successfully" << endl;
			ros::param::set("/robot_grasping_state",-2); // reset to -2 if grasp final state received
			isRobotSucceed = true;
			successful_subtasks ++;
			informTrayupdateTaskmanager = true; // will inform success after decision making
		} else if(dobot_grasp_state == 4) { // timeout or other error
			// grasp_error_cnt ++; TODO: what did you want to do with this?
			ros::param::set("/robot_grasping_state",-2); // reset to -2 if grasp final state received
			isRobotFailed = true;
			failed_subtasks ++;
			informTrayupdateTaskmanager = true;
			//cout << "a problem with robot led to a subtask failure" << endl;
		}

		// informTrayUpdate to taskmanager if a robot tray update occured, this will also calculate global success/fail and will send it to despot
		if(informTrayupdateTaskmanager){
			inform_trayupdate_to_taskmanager();
		}

		// TODO -> check if each warning leads to a negative reward in real setup. If not the allowDecisionMaking && might be changed to ||
		// THIS CHECK OF RECEIVED THE SAME OBS IS TO PREVENT DECISION UPDATE WITH HAR AGENT UPDATE
		if ( allowDecisionMaking && ( (o3_former != o3_oir) || (o4_former != o4_ov) || (o5_former != o5_a0) || (o6_former != o6_a4) || (o7_former != o7_a2)  || ((req.stamp - former_time_stamp) >= ros::Duration(global_task_configuration_read.sameaction_timeout)) ) ) {

			former_time_stamp = req.stamp;
			// send real_state to DESPOT -> this is not used anymore but remains for synchronisation purposes
			IE_humanSt_to_robotSt_Map(observation_mapped);

			// trigger DESPOT Decision
			bool mapping_success = ObservationAgent::IEaction_to_obs_Map();
			ROS_WARN("[OBSERVATION_AGENT] Issuing a despot decision making: %d, time passed since prev action: %f" ,mapping_success, (req.stamp - former_time_stamp).toSec());

		} else
			ROS_WARN("[OBSERVATION_AGENT] SKIPPING despot decision making this round");


		o6_former = o6_a4; 	// Assign former values -> used to check if update ocurred
		o7_former = o7_a2;
		o5_former = o5_a0;
		o3_former = o3_oir;
		o4_former = o4_ov;


		res.success = true;
	} else { ROS_WARN("[OBSERVATION AGENT]  Action received but experiment not started yet -> action is dismissed \n                  to start the Experiment, call:	 rosservice call /task_manager_IE/new_scenario_request  \n\n"); }

	return true;
}

// Tray update if robot took over the task
void ObservationAgent::inform_trayupdate_to_taskmanager(void){
	hrc_ros::SuccessStatusObserved success_status_msg;
	string task_success_state = "ongoing";
	string subtask_success_state = "ongoing";

	// calculate time of last subtask and restart/reset timers
	decision_timer.stop();
	decision_timer.start();
	subtask_timer.stop();
	before_subtask_reset_tick = subtask_timer_tick;
	subtask_timer_tick = 0;
	ros::Time subtask_stop_time = ros::Time::now();
	subtask_duration = subtask_stop_time - subtask_start_time;
	task_time_sumOfSubtasks_sec +=  subtask_duration.toNSec() * std::pow(10.0, (-9.0) );
	////cout << " last subtask took "  <<  subtask_duration.toNSec() * (std::pow(10.0, (-9.0) ) ) <<  "  Combined task time in sec: " << task_time_sumOfSubtasks_sec << "     before_subtask_reset_tick     " << before_subtask_reset_tick << endl;

	// start timers and counters for new subtask
	subtask_start_time = ros::Time::now();
	subtask_timer.start();

	// determine global success state
	// ## determine global success state -> it is only set once all subtasks are finished
	if ( subtask_counter >= current_subtask_quantity ) {  // all subtasks done

		ROS_WARN("Global Success state is calculated");
		if (successful_subtasks >= global_task_configuration_read.global_success_assert ){
			task_success_state = "success"; // global success
			successful_tasks_cnt ++;
			o1_ipd = true; //  O_1  task successs (processed product detected)
			o2_upd = false; // O_2	failure
			decision_timer.stop();
			subtask_timer.stop();
			subtask_timer_tick = 0;
			//TODO remove
			//cout <<  " Global Success | successful_subtasks = " << successful_subtasks << "  global_success_criteria = " << global_task_configuration_read.global_success_assert << endl;
		} else if ( failed_subtasks >= global_task_configuration_read.global_fail_assert){
			task_success_state = "fail"; // global fail
			failed_tasks_cnt ++;
			o1_ipd = false; //  O_1  task successs (processed product detected)
			o2_upd = true; // O_2	failure
			decision_timer.stop();
			subtask_timer.stop();
			subtask_timer_tick = 0;
			// TODO remove
			//cout <<  " Global Fail | failed_subtasks = " << failed_subtasks << "  global_fail_criteria = " << global_task_configuration_read.global_fail_assert << endl;
		}

	}

	// compose message

		// TODO add task time to success_status_msg and publish by task manager
		// ## Compose success_status_msg
		success_status_msg.stamp = ros::Time::now();
		success_status_msg.subtask_success_status = "ROBOT SUCCEEDED";
		success_status_msg.task_success_status = task_success_state;

		// ## fields for debugging
		success_status_msg.current_object = current_object;
		success_status_msg.current_tray = 0;
		success_status_msg.success_tray = success_criteria_read.tray;
		success_status_msg.task_counter = task_counter;
		success_status_msg.subtask_counter = subtask_counter;


		// ## fields for statistics
		success_status_msg.failed_subtasks = failed_subtasks;
		success_status_msg.successful_subtasks = successful_subtasks;
			subtask_time_seconds = subtask_duration.toNSec() * (std::pow(10.0, (-9.0) ) );
		success_status_msg.subtask_time_seconds = subtask_time_seconds;
		success_status_msg.task_combined_subtask_time_seconds = task_time_sumOfSubtasks_sec;
			percentage_successful_subtasks = ( double(successful_subtasks) / double(subtask_counter) ) * double(100.0) ;
		success_status_msg.percentage_successful_subtasks = percentage_successful_subtasks;
		success_status_msg.who_succeeded = "ROBOT";
		success_status_msg.task_warnings_received = warnings_received_task;
		success_status_msg.successful_tasks_cnt = successful_tasks_cnt;
		success_status_msg.failed_tasks_cnt     = failed_tasks_cnt;
			percentage_successful_tasks = (double(successful_tasks_cnt) / double(successful_tasks_cnt + failed_tasks_cnt) ) * double(100.0);
		success_status_msg.percentage_successful_tasks = percentage_successful_tasks;





	// if global state reached send it to despot as well
	// ############ if final state is reached it should also be informed to the POMDP -> the pomdp will terminate afterwards
		if (task_success_state.compare("success") ==0 ){
			//cout << endl << endl << "GlobalSuccess will be sent to POMDP -> it will terminate afterwards" << endl;
			IE_humanSt_to_robotSt_Map("GlobalSuccess");
			ObservationAgent::IEaction_to_obs_Map(); // trigger decision -> this will terminate despot
		} else if (task_success_state.compare("fail")==0) {
			//cout << endl << endl << "GlobalFail will be sent to POMDP -> it will terminate afterwards" << endl;
			IE_humanSt_to_robotSt_Map("GlobalFail");
			ObservationAgent::IEaction_to_obs_Map(); // trigger decision -> this will terminate despot
		}


	// ############ Publish success_status_msg (mainly used by task manager to check if a new task should be started)
	traySensor_success_pub.publish(success_status_msg);

	// ## increment subtask counter
	subtask_counter += 1;

	ROS_INFO("\n\n");
	ROS_WARN("OBSERVATION ROS: #### ROBOT TRAY-UPDATE  RECEIVED #### \n\n");
	ROS_INFO("subtask_success_state = %s",subtask_success_state.c_str());
	ROS_INFO("task_success_state = %s \n ",task_success_state.c_str());
	ROS_INFO("Failed_subtasks = %d  ",failed_subtasks);
	ROS_INFO("successful_subtasks = %d  ",successful_subtasks);
	ROS_INFO("********\n\n\n");

	//cout << endl << " ###############  Current task counters and success states ######################## " << endl << endl;
	//cout << "task_counter = " << task_counter << "  subtask_counter = " << subtask_counter << " subtask_success_state = " << subtask_success_state << endl << endl;

	ros::Duration(4).sleep();

}





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
		else if (observable == "23"){   // subtask_fail
			robot_observation = "13";
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
		else if (observable == "39"){   // subtask_success
			robot_observation = "12";
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
string ObservationAgent::MapObservablesToObservations(bool ov, bool oir, bool a0, bool ipd, bool a4, bool a2, bool upd, int subtask_status){
	string robot_observation = prev_observables;
	if ( (subtask_status == 1) && not ipd && not upd  ) {  // subtask success | most observables not inspected for that
		robot_observation = "39";
	}
	else if ((subtask_status == 3) && not ipd && not upd ) { // subtask fail |  most observables not inspected for that
		robot_observation = "23";
	}
	else if (not ov && not oir && not a0 && not ipd && not a4 && not a2 && not upd) {
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
