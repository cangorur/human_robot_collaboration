/*
 *  Created on: 19.04.2018
 *      Author: Orhan Can Görür
 *      Email: orhan-can.goeruer@dai-labor.de
 */

#include <ros/ros.h>
#include <ros/package.h>

// include <package_name/service_type_name.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#define GetCurrentDir getcwd
#include <iostream>

#include <task_manager/TaskManager.h>

#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"

using namespace std;


TaskManager::TaskManager() {
	ros::NodeHandle nh("~");
	initialize();
}

TaskManager::~TaskManager() {
}


void TaskManager::initialize(){
	ros::NodeHandle nh("~");

	//here set the parameter of conveyor belt to "init" so it automatically starts
	
	/*
	 * Initializing ros services to reset human and the robot
	 */
	humanReset = nh.serviceClient<std_srvs::Trigger>("/human/reset");
	robotReset = nh.serviceClient<std_srvs::Trigger>("/robot/reset");
	conveyorPrinterOnOff = nh.serviceClient<std_srvs::Trigger>("/conveyor/printer_part/switch_on_off");
    conveyorAssembly1OnOff = nh.serviceClient<std_srvs::Trigger>("/conveyor/assembly_part1/switch_on_off");
    conveyorAssembly2OnOff = nh.serviceClient<std_srvs::Trigger>("/conveyor/assembly_part2/switch_on_off");
	moveNewPackage = nh.serviceClient<hrc_ros::MoveNewPackage>("/package_manipulator/move_new_package");
	
	humanROSReset = nh.serviceClient<hrc_ros::ResetHumanROS>("/human_agent/reset");
	obsROSReset = nh.serviceClient<hrc_ros::ResetObsROS>("/observation_agent/reset");
	robotROSReset = nh.serviceClient<hrc_ros::ResetRobotROS>("/robot_agent/reset");
	
	/*
	 * Initializing advertised ros services 
	 */
	scenarioRequestService = nh.advertiseService("new_scenario_request", &TaskManager::initiateScenario, this);
	HumanUpdateService = nh.advertiseService("human_status_update", &TaskManager::HumanStatusUpdater,this);
	ObsUpdateService = nh.advertiseService("observation_update", &TaskManager::ObsUpdater, this);
	RobotUpdateService = nh.advertiseService("robot_status_update", &TaskManager::RobotStatusUpdater, this);
	resetTaskService = nh.advertiseService("reset_task", &TaskManager::ResetTask, this);
	
	/// Task State: human states actions, robot state actions rewards and general info are published as a ROS topic
	taskStatusPublisher = nh.advertise<hrc_ros::TaskState>("task_status", 1);
	
	traySensor_subs = nh.subscribe("/production_line/tray_sensors", 1000, &TaskManager::ReceiveTraySensors, this);
	/*
	 * Timer initialization 
	 */
	taskFinishTimer = nh.createTimer(ros::Duration(1.0), &TaskManager::TaskFinishTimer, this);
	//initialize random seed for rand()
	srand(time(NULL));
	/*
	 * Below are the subscriptions to a ros topic
	robot1_battery = n.subscribe("/robot_1/battery", 1000, &TaskManager::receiveRobot1Battery, this);
	robot2_battery = n.subscribe("/robot_2/battery", 1000, &TaskManager::receiveRobot2Battery, this);
	*/
	ROS_INFO("Task Manager is created !");
}

//================Advertised Services=======================
bool TaskManager::initiateScenario(hrc_ros::InitiateScenarioRequest &req,
		hrc_ros::InitiateScenarioResponse &res) {
	
	string pkg_path = ros::package::getPath("hrc_ros");
	boost::property_tree::ptree config_pt; // json ptree object
	std::ifstream jsonFile(pkg_path + "/../../../configs/scenario_config.json");

	try {
		boost::property_tree::read_json(jsonFile, config_pt);
	} catch(boost::property_tree::json_parser::json_parser_error &e) {
		ROS_FATAL("Cannot parse the message to Json. Error: %s", e.what());
		res.success = false;
		return false;
	}
	
	int r;
	bool humanTrustsRobot;
	if (human_expert == "" || (human_expert != config_pt.get<string>("human.type.expertise"))){ // if it is still the same human
		r = rand() % 3;
		humanTrustsRobot = (r == 0 || r == 1) ? true : false; 
		human_trust = (humanTrustsRobot) ? "YES" : "NO"; // global
	}
	
	string task_assigned = config_pt.get<string>("task.assignment");
	human_expert = config_pt.get<string>("human.type.expertise"); // global
	human_mood = config_pt.get<string>("human.type.mood"); // global
	string robot_type = config_pt.get<string>("robot.type");

	// ========== PACKAGE GENERATOR ================
	// Calling the right function (rosservice) to respond to the request
	//int package_amounts = config_pt.get<int>("package_pool.amount.light");
	// ========== PACKAGE GENERATOR ================
	
	// =========== HUMAN MOOD ASSIGNMENT =====================
	r = (rand() % 50) + 1; // from 1 to 50
	if (task_number != 0){
		if (task_number <= 10){
			human_mood = (r <= task_number + 2) ? "thinker" : human_mood; // gradually increasing from 6 % to 25%
		} else if (task_number > 10 && task_number <= 15){
			human_mood = (r <= task_number + 2) ? "thinker" : human_mood; // gradually increasing from 26 % to 30%
			human_mood = ((r > 15) && (r <= 6 + task_number)) ? "tired" : human_mood; // gradually increasing from 4 % to 12%
		} else if (task_number > 15 && task_number <= 20){
			if (task_number <= 17){
				human_mood = (r <= task_number) ? "thinker" : human_mood; // gradually increasing from 30 % to 35%
			} else {
				human_mood = (r <= 17) ? "thinker" : human_mood; // fixed to 35%
			}
			human_mood = ((r > 17) && (r <= 8 + task_number)) ? "tired" : human_mood; // gradually increasing from 14 % to 22%
			human_mood = ((r > 40) && (r <= 42)) ? "distracted" : human_mood; // fixed to 6%
		} else if (task_number > 20 && task_number <= 30){
			human_mood = (r <= 15) ? "thinker" : human_mood; // fixed to 30%
			human_mood = ((r > 15) && (r <= (10 + task_number))) ? "tired" : human_mood; // gradually increasing from 32 % to 40%
			human_mood = (r > 44) ? "distracted" : human_mood; // fixed to 12%
		} else if (task_number > 30 && task_number <= 40){
			human_mood = (r <= 15) ? "thinker" : human_mood; // fixed to 30%
			human_mood = (r > 15 && r <= 40) ? "tired" : human_mood; // fixed to 50%
			human_mood = (r > 40 && r <= 50) ? "distracted" : human_mood; // fixed to 20%
		}
	}

	// ========================================================
	
	ROS_INFO("[TASK_MANAGER]: Human type is: %s and %s!", human_expert.c_str(), human_mood.c_str());
	ROS_INFO("[TASK_MANAGER]: Robot type is: %s!", robot_type.c_str());
	ROS_INFO("[TASK_MANAGER]: Task is assigned to: %s", task_assigned.c_str());
	ROS_INFO("[TASK_MANAGER]: Does HUMAN TRUST ROBOT?: %s", human_trust.c_str());
	
	// ========== RESET HUMAN AND ROBOT=============
	// We reset the human and robot locations and planners
	std_srvs::Trigger::Request req1;
	std_srvs::Trigger::Response res1;
	humanReset.call(req1, res1);
	robotReset.call(req1, res1);
	// =============================================
	
	// ==== Moving a New Package ====
	// TODO: fix here so that it always goes to the beginning of the conveyor belt
	// TODO: add conveyor run and stop when the package arrives between the human and the robot
	hrc_ros::MoveNewPackage::Request req_ForPkg;
	hrc_ros::MoveNewPackage::Response res_ForPkg;
	req_ForPkg.package_id = "package1";
	req_ForPkg.x = 5.5;
	req_ForPkg.y = -2.1;
	req_ForPkg.z = 0.8;
	moveNewPackage.call(req_ForPkg, res_ForPkg);
	// run the conveyor until pkg arrives between the human and the robot
	std_srvs::Trigger::Request req_conveyor;
	std_srvs::Trigger::Response res_conveyor;
	conveyorPrinterOnOff.call(req_conveyor, res_conveyor); // SWITCH ON
	conveyorAssembly1OnOff.call(req_conveyor, res_conveyor); // SWITCH ON
	conveyorAssembly2OnOff.call(req_conveyor, res_conveyor); // SWITCH ON
	ros::Duration(3.5).sleep(); // sleep for half a second
	conveyorPrinterOnOff.call(req_conveyor, res_conveyor); // SWITCH OFF
	conveyorAssembly1OnOff.call(req_conveyor, res_conveyor); // SWITCH OFF
	conveyorAssembly2OnOff.call(req_conveyor, res_conveyor); // SWITCH OFF
	// assure the package is in between human and the robot
	req_ForPkg.package_id = "package1";
	req_ForPkg.x = 7.7;
	req_ForPkg.y = -2.1;
	req_ForPkg.z = 0.8;
	moveNewPackage.call(req_ForPkg, res_ForPkg);
	// ===============================
	
	// ========== RESET ROS AGENTS (HUMAN, OBSERVATION, ROBOT) =============
	hrc_ros::ResetHumanROS::Request req_human;
	hrc_ros::ResetHumanROS::Response res_human;
	hrc_ros::ResetObsROS::Request req_obs;
	hrc_ros::ResetObsROS::Response res_obs;
	hrc_ros::ResetRobotROS::Request req_robot;
	hrc_ros::ResetRobotROS::Response res_robot;
	// TODO: information about the human's initial state, human type and random human trust. This will be preknown to the obs as it informs the actual states to the robot
	req_human.assignedTo = task_assigned;
	req_obs.assignedTo = task_assigned;
	req_obs.humanTrustsRobot = human_trust;
	req_obs.humanType = human_expert;
	req_obs.humanMood = human_mood;
	req_robot.assignedTo = task_assigned;
	req_obs.robotType = robot_type;
	//humanROSReset.call(req_human, res_human);
	obsROSReset.call(req_obs, res_obs);
	//robotROSReset.call(req_robot, res_robot);
	// =====================================================================
	
	// ==== Planners reset =====
	//TODO: first close the system console then open it !!
	//TODO: add the other human types below
	string mdp_human_shell;
	if (human_mood == "new_normal"){ // new_normal is to test the new human model created (v2)
		mdp_human_shell = "gnome-terminal -e 'sh -c \"" + pkg_path + "/model_scripts/MDP_human_newNormal.sh " + pkg_path + "\"'";
	}else if (human_mood == "stubborn" && human_expert == "beginner"){
		mdp_human_shell = "gnome-terminal -e 'sh -c \"" + pkg_path + "/model_scripts/MDP_human_stubborn.sh " + pkg_path + "\"'";
	}else if (human_mood == "thinker" && human_expert == "beginner"){
		mdp_human_shell = "gnome-terminal -e 'sh -c \"" + pkg_path + "/model_scripts/MDP_human_thinker.sh " + pkg_path + "\"'";
	}else if (human_mood == "distracted" && human_expert == "beginner"){
		mdp_human_shell = "gnome-terminal -e 'sh -c \"" + pkg_path + "/model_scripts/MDP_human_distracted.sh " + pkg_path + "\"'";
	}else if (human_mood == "tired"){
		mdp_human_shell = "gnome-terminal -e 'sh -c \"" + pkg_path + "/model_scripts/MDP_human_tired.sh " + pkg_path + "\"'";
	}else if (human_expert == "expert"){
		mdp_human_shell = "gnome-terminal -e 'sh -c \"" + pkg_path + "/model_scripts/MDP_human_expert.sh " + pkg_path + "\"'";
	}
	const char * c_mdp_human_shell = mdp_human_shell.c_str();
	//cout << "pid of the processes" << getpid();
	cout << "MDP Human shell script path" << mdp_human_shell << endl;
	system(c_mdp_human_shell);

	
	string robot_shell;
	if (robot_type == "proactive"){
		robot_shell = "gnome-terminal -e 'sh -c \"" + pkg_path + "/model_scripts/POMDP_robot_proactive.sh " + pkg_path + "\"'";
	}else if (robot_type == "reactive"){
		//TODO: add the reactive model call here
		robot_shell = "gnome-terminal -e 'sh -c \"" + pkg_path + "/model_scripts/MDP_robot_reactive.sh " + pkg_path + "\"'";
	}else{
		//TODO: add the reactive model call here
		robot_shell = "gnome-terminal -e 'sh -c \"" + pkg_path + "/model_scripts/POMDP_robot.sh " + pkg_path + "\"'";
	}
	const char * c_robot_shell = robot_shell.c_str();
	system(c_robot_shell);
	cout << "Robot shell script path: " << robot_shell << endl;
	// ==== Planners reset =====
	
	// =======================
	step_counter = 0;
	task_time = 0;
	task_number += 1;
	ROS_INFO("[TASK MANAGER] Task is initiated!");
	
	// ======= Informing about the init ===========
	hrc_ros::TaskState taskState_msg;
	taskState_msg.task_id = task_number;
	taskState_msg.who_reports = "MANAGER";
	taskState_msg.update_received_time = ros::Time::now();
	taskState_msg.human_expertise = human_expert;
	taskState_msg.human_mood = human_mood;
	taskState_msg.human_trust = human_trust;
	taskState_msg.task_status = "START";
	taskStatusPublisher.publish(taskState_msg);
	// ============================================
	//TODO: there will be a scenario global object holding the max. number of packages to be generated, may be the last task info.
	//TODO: here there needs to be a global object that holds both the state and info. State is sth updated every iteration
	//that, is whenever the human makes a decision then the robot makes a decision, the trays detect a package etc. Every State 
	//object is renewed and recorded under task_info whenever a package falls into any conveyor. These updates will be done through 
	//callbacks by calling the taskStateUpdateEvent below under the callbacks. This intiate service only sets those variables.
	//TODO:Package generator will trigger the package manipulator whenever a tray reads a value. The generator will in the mean time
	//count how much package has been generated until reaching the final value. The generator will first remove 
	//that package from the tray then generate a new one at the beginning of the conveyor. The first triggers will be done here.
	//TODO: Initiate the conveyor belt --> /conveyor/switch_on_off

	//hrc_ros::TaskState state;
	//hrc_ros::TaskInfo task_info;
			
	res.success = true;
	return true;
}
bool TaskManager::ResetTask(std_srvs::TriggerRequest &req,
		std_srvs::TriggerResponse &res) {
	task_number -= 1;
	
	hrc_ros::InitiateScenario::Request req_init;
	hrc_ros::InitiateScenario::Response res_init;
	initiateScenario(req_init, res_init);
	
	res.success = true;
	return true;
}

bool TaskManager::HumanStatusUpdater(hrc_ros::InformHumanToTaskMangRequest &req, hrc_ros::InformHumanToTaskMangResponse &res){
	
	hrc_ros::TaskState taskState_msg;
	
	taskState_msg.task_id = task_number;
	taskState_msg.step_count = step_counter;
	taskState_msg.who_reports = "HUMAN";
	taskState_msg.human_expertise = human_expert;
	taskState_msg.human_mood = human_mood;
	taskState_msg.human_trust = human_trust;
		
	
	// TODO: update below for the new taskState msg format
	
	taskState_msg.update_received_time = req.human_update.stamp_human_update;
	taskState_msg.action_taken_time = req.human_update.action_taken_time;
	taskState_msg.taken_action = req.human_update.human_action_taken;
	taskState_msg.belief_state = req.human_update.human_belief_state; 
	taskState_msg.real_state = req.human_update.human_real_state;
	if (req.human_update.human_belief_state == req.human_update.human_real_state)
			taskState_msg.isEstimationCorrect = true;
	else
		taskState_msg.isEstimationCorrect = false;
	
	// INformation below are for observation and robot status updates. Otherwise leave empty
	/*taskState_msg.obs_received = "NONE";
	taskState_msg.task_status = "ONGOING";
	taskState_msg.who_succeeded = "NONE";
	
	taskState_msg.immediate_reward = 0.0;
	taskState_msg.total_disc_reward = 0.0;*/
	
	taskStatusPublisher.publish(taskState_msg);
	
	step_counter += 1;
	res.task_number = task_number; // informing human about the ID (task number of the current task)
	res.success = true;
	return res.success;
}
//TODO: TASK STATE MSG STRUCTURE HAS BEEN CHANGED
bool TaskManager::ObsUpdater(hrc_ros::InformObsToTaskMangRequest &req, hrc_ros::InformObsToTaskMangResponse &res){
	
	hrc_ros::TaskState taskState_msg;
	
	taskState_msg.task_id = task_number;
	taskState_msg.step_count = step_counter;
	taskState_msg.who_reports = "OBSERVATION";
	
	taskState_msg.update_received_time = req.obs_update.stamp_obs_update;
	std::vector<uint8_t> real_obs_msg = req.obs_update.real_obs_received;
	string real_observation_array = "LA (not OV): " + to_string(real_obs_msg[0]) + " || OIR (Det.): " + to_string(real_obs_msg[1]) + 
			" || GRASP: " + to_string(real_obs_msg[2]) + " || IPD: " + to_string(real_obs_msg[3]) + " || WARN: " + to_string(real_obs_msg[4]) +
			" || IDLE: " + to_string(real_obs_msg[5]) + " || UPD: " + to_string(real_obs_msg[6]);
	ROS_INFO("[TASK_MANAGER]: Real Observables: %s", real_observation_array.c_str());
	taskState_msg.real_obs_received = real_observation_array;
	
	std::vector<uint8_t> noisy_obs_msg = req.obs_update.obs_with_noise;
	string observation_with_noise_array = "LA (not OV): " + to_string(noisy_obs_msg[0]) + " || OIR (Det.): " + to_string(noisy_obs_msg[1]) + 
			" || GRASP: " + to_string(noisy_obs_msg[2]) + " || IPD: " + to_string(noisy_obs_msg[3]) + " || WARN: " + to_string(noisy_obs_msg[4]) +
			" || IDLE: " + to_string(noisy_obs_msg[5]) + " || UPD: " + to_string(noisy_obs_msg[6]);
	ROS_INFO("[TASK_MANAGER]: Noisy Observables: %s", observation_with_noise_array.c_str());	
	taskState_msg.obs_with_noise = observation_with_noise_array;
	
	if (real_obs_msg[3]){
		//TODO: terminate the task and assign a new one ! --> HOW TO? Calling own service?
		taskState_msg.task_status = "SUCCESS";
		taskState_msg.who_succeeded = req.obs_update.who_succeeded;
	}else if (real_obs_msg[6]){
		//TODO: terminate the task and assign a new one ! --> HOW TO? Calling own service?
		taskState_msg.task_status = "FAIL";
	}else if (not(real_obs_msg[6] || real_obs_msg[3])){
		taskState_msg.task_status = "ONGOING";
	}
	
	taskStatusPublisher.publish(taskState_msg);

	res.success = true;
	return res.success;
}

bool TaskManager::RobotStatusUpdater(hrc_ros::InformRobotToTaskMangRequest &req, hrc_ros::InformRobotToTaskMangResponse &res){
	
	hrc_ros::TaskState taskState_msg;
	
	taskState_msg.task_id = task_number;	
	taskState_msg.step_count = step_counter;
	taskState_msg.who_reports = "ROBOT";
	
	taskState_msg.update_received_time = req.robot_update.stamp_robot_update;
	taskState_msg.action_taken_time = req.robot_update.action_taken_time;
	taskState_msg.taken_action = req.robot_update.robot_action_taken;
	taskState_msg.belief_state = req.robot_update.robot_belief_state; 
	taskState_msg.real_state = req.robot_update.robot_real_state;
	
	if (req.robot_update.robot_belief_state == req.robot_update.robot_real_state)
		taskState_msg.isEstimationCorrect = true;
	else
		taskState_msg.isEstimationCorrect = false;
	
	// INformation below are for observation and robot status updates. Otherwise leave empty
	/*taskState_msg.obs_received = "NONE";
	taskState_msg.task_status = "ONGOING";
	taskState_msg.who_succeeded = "NONE";
	*/
	
	taskState_msg.immediate_reward = req.robot_update.immediate_reward;
	taskState_msg.total_disc_reward = req.robot_update.total_disc_reward;
	
	taskStatusPublisher.publish(taskState_msg);
	
	
	res.success = true;
	return res.success;
}
	
bool TaskManager::packageGenerator(){
	//TODO: callout the package generation service: it is initiated by package_generator.py
	return true;
}

//=================ROS Timer ===============================
//TODO: I dont know how to use this below --> I THINK THIS CHECKS REGULARLY THE STATE OF THE TASK. USELESS
//TODO: I think I would call it when the task is finalized. Then i need one more to initiate the task --> timer
//TODO: How this function is being called --> who initates?
/*void TaskManager::taskStateUpdateEvent(const ros::TimerEvent &e) {
	hrc_ros::TaskState state;
	state.stamp = ros::Time::now(); // This is all I need !!!
	// here put the information of the task status
	//state.registered_robots = registeredRobots.size();

	// TODO: PUBLISH STATES ALSO SAVE IT AS A ROSBAG FILE
	// statusUpdatePub.publish(state);
}*/

//================rostopic callbacks========================
void TaskManager::TaskFinishTimer(const ros::TimerEvent&){
	task_time += 1; // increase one in every second
	if (task_time == 40){ // after 35 seconds the limit has been reached and run the conveyor belt
		std_srvs::Trigger::Request req;
		std_srvs::Trigger::Response res;
		conveyorPrinterOnOff.call(req, res); // SWITCH ON
		conveyorAssembly1OnOff.call(req, res); // SWITCH ON
		conveyorAssembly2OnOff.call(req, res); // TODO: define a conveyor belt status flag (global)
		ros::Duration(4).sleep(); // sleep for half a second
		conveyorPrinterOnOff.call(req, res); // SWITCH ON
		conveyorAssembly1OnOff.call(req, res); // SWITCH ON
		conveyorAssembly2OnOff.call(req, res);
		task_time = 0;
	}
}

void TaskManager::ReceiveTraySensors(const hrc_ros::TraySensor &msg){
	
	ros::Time tray_msg_stamp = msg.stamp;
	string whoSucceeded = "";
	string taskStatus = "ONGOING";
	
	if (msg.tray_id == "tray_unprocessed"){
		if (msg.occupied)
			taskStatus = "FAIL";
		else
			taskStatus = "ONGOING";
	}
	if (msg.tray_id == "tray_human"){
		if (msg.occupied){
			taskStatus = "SUCCESS";
			whoSucceeded = "human";
		}else{
			taskStatus = "ONGOING";
			whoSucceeded = "";
		}
	}
	if (msg.tray_id == "tray_robot"){
		if (msg.occupied){
			taskStatus = "SUCCESS";
			whoSucceeded = "robot";
		}else{
			taskStatus = "ONGOING";
			whoSucceeded = "";
		}
	}
	
	if (msg.occupied){
		
		hrc_ros::TaskState taskState_msg;
		
		taskState_msg.task_id = task_number;
		taskState_msg.step_count = step_counter;
		taskState_msg.who_reports = "SENSORS";
		taskState_msg.update_received_time = ros::Time::now();
		
		taskState_msg.task_status = taskStatus;
		taskState_msg.who_succeeded = whoSucceeded;
			
		taskStatusPublisher.publish(taskState_msg);
		
		/*hrc_ros::InitiateScenario::Request req;
		hrc_ros::InitiateScenario::Response res;
		initiateScenario(req, res);
		*/
	}
}
