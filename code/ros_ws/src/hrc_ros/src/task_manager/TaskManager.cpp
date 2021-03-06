/*
 *  Created on: 19.04.2018
 *  Modified on: 10.08.2018
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
    objReset = nh.serviceClient<std_srvs::Trigger>("/human/obj_reset");
    robotReset = nh.serviceClient<std_srvs::Trigger>("/robot/reset");
    conveyorPrinterOnOff = nh.serviceClient<std_srvs::Trigger>("/conveyor/printer_part/switch_on_off");
    conveyorAssembly1OnOff = nh.serviceClient<std_srvs::Trigger>("/conveyor/assembly_part1/switch_on_off");
    conveyorAssembly2OnOff = nh.serviceClient<std_srvs::Trigger>("/conveyor/assembly_part2/switch_on_off");
    moveNewPackage = nh.serviceClient<hrc_ros::MoveNewPackage>("/package_manipulator/move_new_package");

    humanROSReset = nh.serviceClient<hrc_ros::ResetHumanROS>("/human_agent/reset");
    obsROSReset = nh.serviceClient<hrc_ros::ResetObsROS>("/observation_agent/reset");
    robotROSReset = nh.serviceClient<hrc_ros::ResetRobotROS>("/robot_agent/reset");

    policyRetrieve = nh.serviceClient<std_srvs::Trigger>("/policy_evaluator/retrieve_policies");

    // Initialize Contextual MAB service and if you want to use it
    cmab_call = nh.serviceClient<hrc_ros::SelectPolicy>("/cmab/select_policy");
    // Client for Bayesian Policy Selector service
    bpr_call = nh.serviceClient<hrc_ros::PolicySelectorBPR>("/policy_selector_bpr/select_policy");


    ros::param::set("/task_count", task_number);
    ros::param::set("/robot_interfered_count", 0);
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

    ROS_INFO("[TASK_MANAGER]: Initiating started");
    ros::Duration(3, 0).sleep();

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

    /* TODO: Below was for randomly choosing human trust. Now it is covered in the human models
    int r;
    bool humanTrustsRobot;
    if (human_expertise == "" || (human_expertise != config_pt.get<string>("human.type.expertise"))){ // if it is still the same human
        r = rand() % 3;
        humanTrustsRobot = (r == 0 || r == 1) ? true : false;
        human_trust = (humanTrustsRobot) ? "YES" : "NO"; // global
    }
    */
    string task_assigned = config_pt.get<string>("task.assignment");

    human_expertise = config_pt.get<string>("human.type.expertise"); // global
    human_mood = config_pt.get<string>("human.type.mood"); // global
    human_collaborativeness = config_pt.get<string>("human.type.collaborativeness"); // global
    robot_AItype = config_pt.get<string>("robot.type.AItype"); // global
    robot_forHuman = config_pt.get<string>("robot.type.forHumanType"); // global

    bool useCMAB = config_pt.get<bool>("operation_modes.useCMAB");
    bool useBPR = config_pt.get<bool>("operation_modes.useBPR");
    bool useEvaluator = config_pt.get<bool>("operation_modes.useEvaluator.active");
    bool useRandom = config_pt.get<bool>("operation_modes.useRandom");
    int interactionSampleNumber = config_pt.get<int>("operation_modes.useEvaluator.interactionNumber"); // # of interactions for each human-robot model
    bool useTransitionFunction = config_pt.get<bool>("operation_modes.useTransitionFunction");
    std::vector<string> robot_policies;
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, config_pt.get_child("evaluation_models.policies"))
        {
            robot_policies.push_back(v.second.get_value<string>());
        }
    ros::param::set("/cmab_flag", useCMAB);
    ros::param::set("/bpr_flag", useBPR);
    ros::param::set("/evaluator_flag", useEvaluator);
    ros::param::set("/dynamic_transition_flag", useTransitionFunction);
    ros::param::set("/interaction_sample_amount", interactionSampleNumber);

    // ========== PACKAGE GENERATOR ================
    // Calling the right function (rosservice) to respond to the request
    //int package_amounts = config_pt.get<int>("package_pool.amount.light");
    // ========== PACKAGE GENERATOR ================

    // =========== OLD HUMAN MOOD ASSIGNMENT =====================
    // Used to change the human mood as below.
    // r = (rand() % 50) + 1; // from 1 to 50
    // if (task_number != 0){
    //     if (task_number <= 10){
    //         human_mood = (r <= task_number + 2) ? "thinker" : human_mood; // gradually increasing from 6 % to 25%
    //     } else if (task_number > 10 && task_number <= 15){
    //         human_mood = (r <= task_number + 2) ? "thinker" : human_mood; // gradually increasing from 26 % to 30%
    //         human_mood = ((r > 15) && (r <= 6 + task_number)) ? "tired" : human_mood; // gradually increasing from 4 % to 12%
    //     } else if (task_number > 15 && task_number <= 20){
    //         if (task_number <= 17){
    //             human_mood = (r <= task_number) ? "thinker" : human_mood; // gradually increasing from 30 % to 35%
    //         } else {
    //             human_mood = (r <= 17) ? "thinker" : human_mood; // fixed to 35%
    //         }
    //         human_mood = ((r > 17) && (r <= 8 + task_number)) ? "tired" : human_mood; // gradually increasing from 14 % to 22%
    //         human_mood = ((r > 40) && (r <= 42)) ? "distracted" : human_mood; // fixed to 6%
    //     } else if (task_number > 20 && task_number <= 30){
    //         human_mood = (r <= 15) ? "thinker" : human_mood; // fixed to 30%
    //         human_mood = ((r > 15) && (r <= (10 + task_number))) ? "tired" : human_mood; // gradually increasing from 32 % to 40%
    //         human_mood = (r > 44) ? "distracted" : human_mood; // fixed to 12%
    //     } else if (task_number > 30 && task_number <= 40){
    //         human_mood = (r <= 15) ? "thinker" : human_mood; // fixed to 30%
    //         human_mood = (r > 15 && r <= 40) ? "tired" : human_mood; // fixed to 50%
    //         human_mood = (r > 40 && r <= 50) ? "distracted" : human_mood; // fixed to 20%
    //     }
    // }

    // ========================================================

    // ========== RESET HUMAN AND ROBOT=============
    // We reset the morse / simulator human and robots.
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

    // ==== Timers set =====
    step_counter = 0;
    task_time = 0;
    // Allowing to manually update the task_number
    ros::param::get("/task_count", task_number);
    task_number += 1;
    //if (((task_number-1) % 300) == 0)// NOTE: this reset at every 300th task is for testing purposes
    //    task_number = 1;
    ros::param::set("/task_count", task_number);
    // TODO Whether reset warnings_count is required?
    warning_number = 0;
    // =======================

    // ================== START HUMAN AND ROBOT POLICIES ===================
    // === HUMAN: Setting up the selected human policy ===
    human_type = "";
    //  Start the mdp human with the given type (either from config file as default or rosparam: manual set or set by policy retrieve)
    string mdp_human_shell = "gnome-terminal --geometry=80x24+10+10 -e 'sh -c \"" + pkg_path + "/model_scripts/MDP_human_general.sh " + pkg_path + " ";

    if (useEvaluator){ // in the evaluator mode call the policy_evaluator node to retrieve human-robot poliy pairs (as rosparams)
        std_srvs::Trigger::Request req_polRetrieve;
        std_srvs::Trigger::Response res_polRetrieve;
        policyRetrieve.call(req_polRetrieve, res_polRetrieve);
        if (!res_polRetrieve.success){ // if policy retrieve returns false, means all combinations have been tested
            res.success = false;
            return false;
        }
        if(ros::param::has("/human_type")){
            ros::param::get("/human_type",human_type);
        }
        mdp_human_shell = mdp_human_shell + "/Evaluate/";
    } else {
        // This overwrites the human type with manual entry from config.json file.
        human_type = human_expertise + "_" + human_mood + "_" + human_collaborativeness + ".POMDPx";
    }

    mdp_human_shell = mdp_human_shell + human_type + "\"'";
    const char * c_mdp_human_shell = mdp_human_shell.c_str();
    //cout << "pid of the processes" << getpid();
    cout << "MDP Human shell script path: " << mdp_human_shell << endl;
    system(c_mdp_human_shell);

    //NOTE: human mood and expertise is used under mc_sampler and observation agent (transition_function)
    //NOTE: THAT human_type = human_expertise + human_mood + human_collaborativeness
    ros::param::set("/human_type", human_type);
    std::size_t index = human_type.find("_");
  	if (human_type.substr(0, index) == "beginner" || human_type.substr(0, index) == "expert"){
        human_expertise = human_type.substr(0, index);
        human_mood = human_type.substr(index+1); //mood + collaborativeness + POMDPx string
        std::size_t index2 = human_mood.find("_");
        human_collaborativeness = human_mood.substr(index2+1); // collaborativeness + POMPDx string
        human_mood = human_mood.substr(0, index2); // only mood now
        std::size_t index3 = human_collaborativeness.find(".");
        human_collaborativeness = human_collaborativeness.substr(0, index3);
  	} else if (human_type.substr(0, index) == "distracted"){
        human_mood = "distracted";
        human_expertise = "undefined";
        human_collaborativeness = human_type.substr(index+1); // collaborativeness + POMPDx string
        std::size_t index2 = human_collaborativeness.find(".");
        human_collaborativeness = human_collaborativeness.substr(0, index2);
    } else {
        human_mood = "default";
    }
    ros::param::set("/human_mood", human_mood);
    ros::param::set("/human_expertise", human_expertise);
    ros::param::set("/human_collaborativeness", human_collaborativeness);

    // TODO: See if we need terminal_state and initial_state informers
    ros::param::set("/human_observable_state", -1); // Either GlobalSuccess or GlobalFail --> to inform human agent
    ros::param::set("/human_initial_state", true);

    // === ROBOT: Setting up the selected robot policy ===
    robot_model = "";
    // Start the robot shell with the given type (either from config file as default or rosparam: manual set or set by policy retrieve)
    string robot_shell = "gnome-terminal --geometry=80x24+10+10 -e 'sh -c \"" + pkg_path + "/model_scripts/POMDP_robot_general.sh " + pkg_path + " ";

    if(useEvaluator){ // robot_model is selected by the policy_evaluator node
        if(ros::param::has("/robot_model")){
            ros::param::get("/robot_model",robot_model);
            if (robot_model == "reactive.pomdpx"){
                // Selecting the MDP despote exe
                robot_shell = "gnome-terminal --geometry=80x24+10+10 -e 'sh -c \"" + pkg_path + "/model_scripts/MDP_robot_reactive.sh " + pkg_path + " ";
                robot_AItype = "reactive";
            }
            robot_AItype = "proactive";
            // robot_shell = robot_shell + "/Evaluate/" + robot_model + "\"'";
        }
    }
    if(useCMAB){ // If the use of CMAB is enabled for the policy selection, we overwrite the manually selected robot policy
        //ros::param::get("/robot_model",robot_model);
        hrc_ros::SelectPolicy::Request req_cmab;
        hrc_ros::SelectPolicy::Response res_cmab;
        cmab_call.call(req_cmab,res_cmab);
        robot_model=res_cmab.robot_model;
        if (robot_model == "reactive.pomdpx"){
          robot_AItype = "reactive";
          robot_shell = "gnome-terminal --geometry=80x24+10+10 -e 'sh -c \"" + pkg_path + "/model_scripts/MDP_robot_reactive.sh " + pkg_path + " ";
        }
        robot_shell = robot_shell + robot_model + "\"'";
    }else if(useBPR){ // If the use of BPR is enabled for the policy selection, we overwrite the manually selected robot policy
        hrc_ros::PolicySelectorBPR::Request req_bpr;
        hrc_ros::PolicySelectorBPR::Response res_bpr;
        bpr_call.call(req_bpr, res_bpr);
        robot_belief = res_bpr.belief;
        robot_model = robot_policies[res_bpr.policy_id];
        if (robot_model == "reactive.pomdpx"){
          robot_AItype = "reactive";
          robot_shell = "gnome-terminal --geometry=80x24+10+10 -e 'sh -c \"" + pkg_path + "/model_scripts/MDP_robot_reactive.sh " + pkg_path + " ";
        }
        robot_shell = robot_shell + "/Evaluate/" + robot_model + "\"'";
    }else if(useRandom){
        int r = (rand() % 14); // from 0 to 13
        robot_AItype = "proactive";
        if (r == 0){
          robot_model = "base.pomdpx";
        }else if (r == 13){
          robot_model = "reactive.pomdpx";
          robot_AItype = "reactive";
          robot_shell = "gnome-terminal --geometry=80x24+10+10 -e 'sh -c \"" + pkg_path + "/model_scripts/MDP_robot_reactive.sh " + pkg_path + " ";
        }else{
          robot_model = "policy" + to_string(r) + ".pomdpx";
        }
        robot_shell = robot_shell + "/Evaluate/" + robot_model + "\"'";
    }
    else if(!useEvaluator){ // take the robot model specified under scenario_config.json file
        robot_model = robot_AItype + "_" + robot_forHuman + ".pomdpx";
        if (robot_AItype == "proactive"){
            robot_shell = robot_shell + robot_model + "\"'";
        }else if (robot_AItype == "reactive"){
            robot_shell = "gnome-terminal --geometry=80x24+10+10 -e 'sh -c \"" + pkg_path + "/model_scripts/MDP_robot_reactive.sh " + pkg_path + " ";
            robot_shell = robot_shell + robot_model + "\"'";
        }else{
            //if nothing specified call the default pomdp robot model
            robot_model = "proactive.pomdpx";
            robot_shell = robot_shell + robot_model + "\"'";
        }
    }else{
        robot_shell = robot_shell + "/Evaluate/" + robot_model + "\"'";
    }

    const char * c_robot_shell = robot_shell.c_str();
    system(c_robot_shell);
    cout << "Robot shell script path: " << robot_shell << endl;

    // Setting robot infor and new scenario information parameters
    ros::param::set("/robot_initial_state", true);
    ros::param::set("/current_robot_action", -1);
    // =====================================================================

    // ========== RESET ROS AGENTS (HUMAN, OBSERVATION, ROBOT) =============
    // TODO: Human and Robot ROS reseting does not work due to the reset service. It cant be caught cause of the set websocket thread.
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
    req_obs.humanType = human_type;
    req_obs.humanMood = human_mood;
    req_robot.assignedTo = task_assigned;
    req_obs.robotType = robot_AItype;
//    humanROSReset.call(req_human, res_human);
    obsROSReset.call(req_obs, res_obs);
    //robotROSReset.call(req_robot, res_robot);
    // =====================================================================

    ROS_INFO("[TASK MANAGER] ======= CURRENT TASK INFORMATION ======");
    ROS_INFO("[TASK MANAGER] Task is initiated!");
    ROS_INFO("[TASK_MANAGER]: Task #%d is assigned to: %s", task_number, task_assigned.c_str());
    ROS_INFO("[TASK_MANAGER]: Evaluator mode is on?: %d", useEvaluator);
    ROS_INFO("[TASK_MANAGER]: HRI interaction # for each policy pair: %d", interactionSampleNumber);
    ROS_INFO("[TASK MANAGER] ======= HUMAN INFORMATION ======");
    ROS_INFO("[TASK_MANAGER]: Dynamic human state transition is on?: %d", useTransitionFunction);
    ROS_INFO("[TASK_MANAGER]: HUMAN TYPE is: %s", human_type.c_str());
    ROS_INFO("[TASK MANAGER] ======= ROBOT INFORMATION ======");
    ROS_INFO("[TASK_MANAGER]: CMAB Policy selector use is on?: %d", useCMAB);
    ROS_INFO("[TASK_MANAGER]: BPR Policy selector use is on?: %d", useBPR);
    ROS_INFO("[TASK_MANAGER]: Random selection use is on?: %d", useRandom);
    ROS_INFO("[TASK_MANAGER]: ROBOT MODEL is: %s", robot_model.c_str());

    // ======= Informing about the init ===========
    hrc_ros::TaskState taskState_msg;
    taskState_msg.task_id = task_number;
    taskState_msg.who_reports = "MANAGER";
    taskState_msg.update_received_time = ros::Time::now();
    taskState_msg.human_model = human_type;
    taskState_msg.task_status = "START";
    taskState_msg.warnings_count = warning_number;
    taskState_msg.robot_model = robot_model;

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
    ros::param::set("/task_count", task_number);

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
    taskState_msg.human_model = human_type;

    // TODO: update below for the new taskState msg format

    taskState_msg.update_received_time = req.human_update.stamp_human_update;
    taskState_msg.action_taken_time = req.human_update.action_taken_time;
    // If the action taken is warn_robot, increment the warning_number
    if (req.human_update.human_action_taken == "4")
        warning_number += 1;
    taskState_msg.taken_action = req.human_update.human_action_taken;
    taskState_msg.warnings_count = warning_number;
    // If the human state is RobotInterfered, increment the robot_interfered_number
    if (req.human_update.human_real_state == "RobotInterfered")
        robot_interfered_number += 1;
    taskState_msg.belief_state = req.human_update.human_belief_state;
    taskState_msg.real_state = req.human_update.human_real_state;
    if (req.human_update.human_belief_state == req.human_update.human_real_state)
        taskState_msg.isEstimationCorrect = true;
    else
        taskState_msg.isEstimationCorrect = false;

    // Setting counter parameters for task, warning and robot interfered
    ros::param::set("/warning_count", warning_number);
    ros::param::set("/robot_interfered_count", robot_interfered_number);

    // Information below are for observation and robot status updates. Otherwise leave empty
    /*taskState_msg.obs_received = "NONE";
        taskState_msg.task_status = "ONGOING";
        taskState_msg.who_succeeded = "NONE";

        taskState_msg.immediate_reward = 0.0;
        taskState_msg.total_disc_reward = 0.0;*/

    taskStatusPublisher.publish(taskState_msg);

    if (req.human_update.human_real_state == "GlobalSuccess" || req.human_update.human_real_state == "GlobalFail"){
        // checked when the task is finished to start a new one (all agents need to inform)
        human_has_informed = true;
    }

    step_counter += 1;
    /*if (step_counter > 20){
      // TODO: a bug in the system that sometimes human wont receive success. That causes to iterate forever.
      // This is a temporary solution to hard stop the task on the human's side.
      ros::param::set("/human_observable_state", 1);
      task_stuck_flag = true;
    }*/
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

    // Observation array =                     [LA = Is human Looking Around?,
    //                                          H.DET = Is human detected?,
    //                                          GRASP = Has human graspped?,
    //                                          SUCCESS = Has task reached to success?,
    //                                          WARN = Has human warned?,
    //                                          IDLE = Is human idling?,
    //                                          FAIL = Has the task failed?]


    string real_observation_array = "LA: " + to_string(real_obs_msg[0]) + " || H.DET: " + to_string(real_obs_msg[1]) +
            " || GRASP: " + to_string(real_obs_msg[2]) + " || SUCCESS: " + to_string(real_obs_msg[3]) + " || WARN: " + to_string(real_obs_msg[4]) +
            " || IDLE: " + to_string(real_obs_msg[5]) + " || FAIL: " + to_string(real_obs_msg[6]);
    ROS_INFO("[TASK_MANAGER]: Real Observables: %s", real_observation_array.c_str());
    taskState_msg.real_obs_received = real_observation_array;

    std::vector<uint8_t> noisy_obs_msg = req.obs_update.obs_with_noise;
    string observation_with_noise_array = "LA: " + to_string(noisy_obs_msg[0]) + " || H.DET: " + to_string(noisy_obs_msg[1]) +
            " || GRASP: " + to_string(noisy_obs_msg[2]) + " || SUCCESS: " + to_string(noisy_obs_msg[3]) + " || WARN: " + to_string(noisy_obs_msg[4]) +
            " || IDLE: " + to_string(noisy_obs_msg[5]) + " || FAIL: " + to_string(noisy_obs_msg[6]);
    ROS_INFO("[TASK_MANAGER]: Noisy Observables: %s", observation_with_noise_array.c_str());
    taskState_msg.obs_with_noise = observation_with_noise_array;

    // Human observables are the observation vector only related to the human actions and situation:
    // human_obs as vector<bool> =                  [Human Detected?,
    //                                               Human is Looking around?,
    //                                               Human has graspped successfully?, // if this and the next one both are false then human didnt grasp
    //                                               Human has failed in grasping?,
    //                                               Human is warning?,
    //                                               Human is staying idle?]
    bool human_grasp_success = false;
    if (real_obs_msg[2] && !real_obs_msg[7]) // real_obs_msg[7] holds the grasp failed info
        human_grasp_success = true;
    std::vector<uint8_t> human_obs;
    human_obs.push_back(real_obs_msg[1]); human_obs.push_back(real_obs_msg[0]); human_obs.push_back(human_grasp_success);
    human_obs.push_back(real_obs_msg[7]); human_obs.push_back(real_obs_msg[4]); human_obs.push_back(real_obs_msg[5]);
    taskState_msg.human_observables = human_obs;

    if (real_obs_msg[3]){
        //TODO: terminate the task and assign a new one ! --> HOW TO? Calling own service?
        //this is done after receiving traysensor data directly by calling own service
        taskState_msg.task_status = "SUCCESS";
        taskState_msg.who_succeeded = req.obs_update.who_succeeded;

    }else if (real_obs_msg[6]){
        //TODO: terminate the task and assign a new one ! --> HOW TO? Calling own service?
        //this is done after receiving traysensor data directly by calling own service
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
    taskState_msg.robot_model = robot_model;
    taskState_msg.immediate_reward = req.robot_update.immediate_reward;
    taskState_msg.total_disc_reward = req.robot_update.total_disc_reward;
    taskState_msg.robot_belief = robot_belief;

    taskStatusPublisher.publish(taskState_msg);

    if (req.robot_update.robot_real_state == "GlobalSuccess" || req.robot_update.robot_real_state == "GlobalFail"){
        // checked when the task is finished to start a new one (all agents need to inform)
        robot_has_informed = true;
    }

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
    if (task_time == 45){ // after 50 seconds the limit has been reached and run the conveyor belt
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;
        conveyorPrinterOnOff.call(req, res); // SWITCH ON
        conveyorAssembly1OnOff.call(req, res); // SWITCH ON
        conveyorAssembly2OnOff.call(req, res); // TODO: define a conveyor belt status flag (global)
        ros::Duration(7).sleep(); // sleep until task reaches to a failure or a success
        conveyorPrinterOnOff.call(req, res); // SWITCH ON
        conveyorAssembly1OnOff.call(req, res); // SWITCH ON
        conveyorAssembly2OnOff.call(req, res);
    } else if (task_time == 80){ // if the pkg fell down somewhere else, this directly terminates
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;
        std_srvs::Trigger::Request req1;
        std_srvs::Trigger::Response res1;
        objReset.call(req1, res1);
        conveyorPrinterOnOff.call(req, res); // SWITCH ON
        conveyorAssembly1OnOff.call(req, res); // SWITCH ON
        conveyorAssembly2OnOff.call(req, res); // TODO: define a conveyor belt status flag (global)
        ros::Duration(7).sleep(); // sleep until task reaches to a failure or a success
        conveyorPrinterOnOff.call(req, res); // SWITCH ON
        conveyorAssembly1OnOff.call(req, res); // SWITCH ON
        conveyorAssembly2OnOff.call(req, res);
        task_time = 0;
    }
    // Check if the task is accomplished (all the agents should acknowledge the success or fail)
    CheckToStartNewTask();
}

void TaskManager::ReceiveTraySensors(const hrc_ros::TraySensor &msg){

    ros::Time tray_msg_stamp = msg.stamp;
    string whoSucceeded = "";
    string taskStatus = "ONGOING";

    if (msg.tray_id == "tray_unprocessed"){
        if (msg.occupied){
            taskStatus = "FAIL";
            // This flag is for letting taskmanager know that sensors informed
            task_has_finished = true;
        } else
            taskStatus = "ONGOING";
    }
    if (msg.tray_id == "tray_human"){
        if (msg.occupied){
            taskStatus = "SUCCESS";
            whoSucceeded = "human";
            task_has_finished = true;
        }else{
            taskStatus = "ONGOING";
            whoSucceeded = "";
        }
    }
    if (msg.tray_id == "tray_robot"){
        if (msg.occupied){
            taskStatus = "SUCCESS";
            whoSucceeded = "robot";
            task_has_finished = true;
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
        // TODO: see if we need to set those parameters below, or if we can set them somewhere else
        bool init_state = false;
        ros::param::get("/human_initial_state", init_state);
        if(!init_state && (taskStatus == "FAIL" || taskStatus == "SUCCESS"))
        {
            int terminal_state = (taskStatus == "FAIL") ? 2: 1;
            ros::param::set("/human_observable_state", terminal_state);
        }
    }
}

void TaskManager::CheckToStartNewTask(void){

    if ((task_has_finished && human_has_informed && robot_has_informed) || task_stuck_flag){
        // Task status informers reset
        task_has_finished = false;
        human_has_informed = false;
        robot_has_informed = false;
        task_stuck_flag = false;
        task_time = 0;

        hrc_ros::InitiateScenario::Request req_init;
        hrc_ros::InitiateScenario::Response res_init;
        initiateScenario(req_init, res_init);
    }

}
