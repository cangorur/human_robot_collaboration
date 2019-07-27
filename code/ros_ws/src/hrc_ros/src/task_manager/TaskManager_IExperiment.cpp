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

#include <task_manager/TaskManager_IExperiment.h>

#include "boost/property_tree/ptree.hpp"



using namespace std;


TaskManager::TaskManager() {
    ros::NodeHandle nh("~");
    initialize();
}

TaskManager::~TaskManager() {
}


void TaskManager::initialize(){
    ros::NodeHandle nh("~");

    /*
    * Initializing counters for the task and subtask control
    */
    task_number = 0; // incremented before experiment starts
    subtask_counter = 1;

    //here set the parameter of conveyor belt to "init" so it automatically starts

    /*
     * Initializing ros services to reset human and the robot and dobot
     */
    dobotWorkerReset = nh.serviceClient<std_srvs::Trigger>("/dobot_worker/reset");
    humanReset = nh.serviceClient<std_srvs::Trigger>("/human/reset");
    robotReset = nh.serviceClient<std_srvs::Trigger>("/robot/reset");
    conveyorPrinterOnOff = nh.serviceClient<std_srvs::Trigger>("/conveyor/printer_part/switch_on_off");
    conveyorAssembly1OnOff = nh.serviceClient<std_srvs::Trigger>("/conveyor/assembly_part1/switch_on_off");
    conveyorAssembly2OnOff = nh.serviceClient<std_srvs::Trigger>("/conveyor/assembly_part2/switch_on_off");
    moveNewPackage = nh.serviceClient<hrc_ros::MoveNewPackage>("/package_manipulator/move_new_package");

    humanROSReset = nh.serviceClient<hrc_ros::ResetHumanROS>("/human_agent/reset");
    obsROSReset = nh.serviceClient<hrc_ros::ResetObsROS>("/observation_agent/reset");
    robotROSReset = nh.serviceClient<hrc_ros::ResetRobotROS>("/robot_agent/reset");

    policyRetrieve = nh.serviceClient<std_srvs::Trigger>("/policy_evaluator/retrieve_policies");

    display_task_rule_client = nh.serviceClient<hrc_ros::DisplayTaskRule>("/rule_monitor/display_task_rule");
    display_score_client = nh.serviceClient<hrc_ros::DisplayScoring>("/rule_monitor/display_scoring");
    distract_human_client = nh.serviceClient<std_srvs::Trigger>("/rule_monitor/distract_participants");

    // Initialize Contextual MAB service and if you want to use it
    cmab_call = nh.serviceClient<hrc_ros::SelectPolicy>("/cmab/select_policy");
    // Client for Bayesian Policy Selector service
    bpr_call = nh.serviceClient<hrc_ros::PolicySelectorBPR>("/policy_selector_bpr/select_policy");

    // clearing all the ros parameters
    ros::param::set("/task_count", task_number);
    ros::param::set("/robot_interfered_count", 0);
    ros::param::set("/robot_immediate_reward", "");
    ros::param::set("/robot_total_disc_reward", "");
    ros::param::set("/robot_grasping_state", -2);
    /*
         * Initializing advertised ros services
         */
    scenarioRequestService = nh.advertiseService("new_scenario_request", &TaskManager::initiateScenario, this);
    // HumanUpdateService = nh.advertiseService("human_status_update", &TaskManager::HumanStatusUpdater,this);
    ObsUpdateService = nh.advertiseService("/task_manager/observation_update", &TaskManager::ObsUpdater, this);
    RobotUpdateService = nh.advertiseService("/task_manager/robot_status_update", &TaskManager::RobotStatusUpdater, this);
    resetTaskService = nh.advertiseService("reset_task", &TaskManager::ResetTask, this);
    setTaskNumberService = nh.advertiseService("/task_manager_IE/setTaskNumber", &TaskManager::setTaskNumber,this);

    /// Task State: human states actions, robot state actions rewards and general info are published as a ROS topic
    taskStatusPublisher = nh.advertise<hrc_ros::TaskStateIE>("/task_manager/task_status", 1);

    // TODO delete if not needed in IE anymore
    //traySensor_subs = nh.subscribe("/production_line/tray_sensors", 1000, &TaskManager::ReceiveTraySensors, this);

    trayObservation_success_status_subs = nh.subscribe("/observation_agent/observedsuccess_status", 1000, &TaskManager::ReceiveTraySuccessStatus, this);

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

    // distracting of human -> not used so far
    //std_srvs::Trigger trigger_service;
    //distract_human_client.call(trigger_service.request,trigger_service.response);

    ROS_INFO("Task Manager is created !");
}

bool TaskManager::setTaskNumber(hrc_ros::SetTaskNumberRequest &req, hrc_ros::SetTaskNumberResponse &res){

    // kill all open despots
    string kill_pomdpx_str = "killall despot_pomdpx";
    const char * c_kill_pomdpx = kill_pomdpx_str.c_str();
    system(c_kill_pomdpx);

    task_number = req.task_number -1;
    ros::param::set("/task_count", task_number);

    //cout << "task_number " << task_number << "  ( is -1 since it will be increased when scenario is started" << endl;

    //hrc_ros::InitiateScenario::Request req_init;
    //hrc_ros::InitiateScenario::Response res_init;
    //initiateScenario(req_init, res_init);

    res.success = true;
    return true;

}

// Call this function once per every task !!!
//================Advertised Services=======================
bool TaskManager::initiateScenario(hrc_ros::InitiateScenarioRequest &req,
                                   hrc_ros::InitiateScenarioResponse &res) {
    /*
    * read in task scenario definitions at start (task_number = 1)
    */

        ROS_INFO("[TASK_MANAGER]: Parsing task scenario definition file @initialisation");

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

        //cout << "# task_counter: " << task_number << endl << endl;

    subtask_counter = 1;
    //task_counter += 1;

    // ==== Timers set =====
    step_counter = 0;
    task_time = 0;
    // Allowing to manually update the task_number
    ros::param::get("/task_count", task_number);
    task_number += 1;
    ros::param::set("/task_count", task_number);
    // TODO Whether reset warnings_count is required?
    warning_number = 0;
    // =======================

    // ####################  sending current task rules to the observation agent #####################

    string task_counter_str;
    stringstream ss;
    ss << task_number;
    string task_str = ss.str();


    current_global_task_config = read_global_task_config(testscenario_pt);
    current_task_set = read_task_set(task_str,testscenario_pt);
    subtask_number_current_task = current_task_set.subtask_quantity; // this var is used to check if new tasks should be started


    ROS_INFO("[TASK_MANAGER]: Initiating started");


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

    // ========================================================

    // ========== RESET DOBOT AGENTS =============
    std_srvs::Trigger::Request req1;
    std_srvs::Trigger::Response res1;
    //humanReset.call(req1, res1);
    //robotReset.call(req1, res1);
    // Reset dobot_worker_node -> will ensure that counters ... are initial
    dobotWorkerReset.call(req1,res1);

    // =============================================




/*
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
    ////cout << "pid of the processes" << getpid();
    //cout << "MDP Human shell script path: " << mdp_human_shell << endl;
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

    */

    // === ROBOT: Setting up the selected robot policy ===
    robot_model = "";
    // Start the robot shell with the given type (either from config file as default or rosparam: manual set or set by policy retrieve)

    //## use this, if the POMDP should show up in own terminal
    //string robot_shell = "gnome-terminal --geometry=80x24+10+10 -e 'sh -c \"" + pkg_path + "/model_scripts/POMDP_robot_general.sh " + pkg_path + " ";

    string robot_shell =  pkg_path + "/model_scripts/POMDP_robot_general.sh " + pkg_path + " ";


    if(useEvaluator){ // robot_model is selected by the policy_evaluator node
        if(ros::param::has("/robot_model")){
            ros::param::get("/robot_model",robot_model);
            if (robot_model == "reactive.pomdpx"){
                // Selecting the MDP despote exe
                //robot_shell = pkg_path + "/model_scripts/MDP_robot_reactive.sh " + pkg_path + " ";
                robot_shell = pkg_path + "/model_scripts/python_robot_reactive.sh " + pkg_path + "/src/robot_motion_agent_IE";
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
          //robot_shell = pkg_path + "/model_scripts/MDP_robot_reactive.sh " + pkg_path + " ";
          robot_shell = pkg_path + "/model_scripts/python_robot_reactive.sh " + pkg_path + "/src/robot_motion_agent_IE";
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
          //robot_shell = pkg_path + "/model_scripts/MDP_robot_reactive.sh " + pkg_path + " ";
          robot_shell = pkg_path + "/model_scripts/python_robot_reactive.sh " + pkg_path + "/src/robot_motion_agent_IE";
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
          robot_shell = pkg_path + "/model_scripts/python_robot_reactive.sh " + pkg_path + "/src/robot_motion_agent_IE";
          //robot_shell = pkg_path + "/model_scripts/MDP_robot_reactive.sh " + pkg_path + " ";
          //robot_shell = robot_shell + "reactive.pomdpx" + " &";
        }else{
          robot_model = "policy" + to_string(r) + ".pomdpx";
        }
        robot_shell = robot_shell + "/Evaluate/" + robot_model + "\"'";
    }
    else if(!useEvaluator){ // take the robot model specified under scenario_config.json file
        if (robot_AItype == "proactive"){
      	    // ## if POMDP should show up in own terminal
      	    //robot_shell = robot_shell + robot_model + "\"'";
            robot_model = robot_AItype + "_" + robot_forHuman + ".pomdpx";
            robot_shell = robot_shell + robot_model + " &";
        }else if (robot_AItype == "reactive"){
            robot_model = "reactive.pomdpx";
            robot_shell = "rosrun hrc_ros reactive_robot_DM_agent.py";
            //robot_shell = pkg_path + "/model_scripts/python_robot_reactive.sh " + pkg_path + "/src/robot_motion_agent_IE";
            //robot_shell = pkg_path + "/model_scripts/MDP_robot_reactive.sh " + pkg_path + " ";
            //robot_shell = robot_shell + "reactive.pomdpx" + " &";
        }else{
            //if nothing specified call the default pomdp robot model
            robot_model = "proactive.pomdpx";
	   // ## if robot_shell should show up in own terminal
	   //robot_shell = robot_shell + robot_model + "\"'";
	   robot_shell = robot_shell + robot_model   + " &";
        }
    }else{
        robot_shell = robot_shell + "/Evaluate/" + robot_model + "\"'";
    }
    ROS_INFO("[TASK MANAGER] Robot Shell: %s", robot_shell.c_str());
    // kill all open despots before starting a new one
    string kill_pomdpx_str = "killall despot_pomdpx";
    const char * c_kill_pomdpx = kill_pomdpx_str.c_str();
    system(c_kill_pomdpx);

    //############  Issuing to display taskrules before experiment is started
    hrc_ros::DisplayTaskRule::Request   req_task_rule_display;
    hrc_ros::DisplayTaskRule::Response  resp_task_rule_display;

    req_task_rule_display.task_counter = task_number;
    display_task_rule_client.call(req_task_rule_display, resp_task_rule_display);


    //############  Launching new DESPOT #####################################
    const char * c_robot_shell = robot_shell.c_str();
    // for the reactive robot, run the websocket node separately as its blocking the process 
    if (robot_AItype == "proactive")
      system(c_robot_shell);

    //cout << "Robot shell script path: " << robot_shell << endl;

    // Setting robot infor and new scenario information parameters
    ros::param::set("/robot_initial_state", true);
    ros::param::set("/current_robot_action", -1);
    // =====================================================================

    // ========== RESET ROS AGENTS (HUMAN, OBSERVATION, ROBOT) =============
    // TODO: Human and Robot ROS reseting does not work due to the reset service. It cant be caught cause of the set websocket thread.
   // hrc_ros::ResetHumanROS::Request req_human;
   // hrc_ros::ResetHumanROS::Response res_human;
    hrc_ros::ResetObsROS::Request req_obs;
    hrc_ros::ResetObsROS::Response res_obs;
    hrc_ros::ResetRobotROS::Request req_robot;
    hrc_ros::ResetRobotROS::Response res_robot;
    // TODO: information about the human's initial state, human type and random human trust. This will be preknown to the obs as it informs the actual states to the robot
    //req_human.assignedTo = task_assigned;
    req_obs.assignedTo = task_assigned;
    //req_obs.humanTrustsRobot = human_trust;
    //req_obs.humanType = human_type;
    //req_obs.humanMood = human_mood;
    req_robot.assignedTo = task_assigned;
    req_robot.robotType = robot_AItype;
    req_obs.robotType = robot_AItype;
    req_obs.task_cnt = task_number;
//    humanROSReset.call(req_human, res_human);
    obsROSReset.call(req_obs, res_obs);
    //robotROSReset.call(req_robot, res_robot);
    // =====================================================================

    ROS_INFO("[TASK MANAGER] ======= CURRENT TASK INFORMATION ======");
    ROS_INFO("[TASK MANAGER] Task is initiated!");
    ROS_INFO("[TASK_MANAGER]: Task #%d is assigned to: %s", task_number, task_assigned.c_str());
    ROS_INFO("[TASK MANAGER]: Task counter is: %d  | subtask_counter is:  %d", task_number, subtask_counter);
    ROS_INFO("[TASK_MANAGER]: Evaluator mode is on?: %d", useEvaluator);
    ROS_INFO("[TASK_MANAGER]: HRI interaction # for each policy pair: %d", interactionSampleNumber);
    ROS_INFO("[TASK MANAGER] ======= HUMAN INFORMATION ======");
    //ROS_INFO("[TASK_MANAGER]: Dynamic human state transition is on?: %d", useTransitionFunction);
    //ROS_INFO("[TASK_MANAGER]: HUMAN TYPE is: %s", human_type.c_str());
    ROS_INFO("[TASK MANAGER] ======= ROBOT INFORMATION ======");
    ROS_INFO("[TASK_MANAGER]: CMAB Policy selector use is on?: %d", useCMAB);
    ROS_INFO("[TASK_MANAGER]: BPR Policy selector use is on?: %d", useBPR);
    ROS_INFO("[TASK_MANAGER]: Random selection use is on?: %d", useRandom);
    ROS_INFO("[TASK_MANAGER]: ROBOT MODEL is: %s", robot_model.c_str());

    // ======= Informing about the init ===========
    hrc_ros::TaskStateIE taskState_msg;
    taskState_msg.task_id = task_number;
    taskState_msg.who_reports = "MANAGER";
    taskState_msg.update_received_time = ros::Time::now();
    //taskState_msg.human_model = human_type;
    taskState_msg.task_status = "START";
    //taskState_msg.warnings_count = warning_number;
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
/*
bool TaskManager::HumanStatusUpdater(hrc_ros::InformHumanToTaskMangRequest &req, hrc_ros::InformHumanToTaskMangResponse &res){

    hrc_ros::TaskStateIE taskState_msg;

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
    //taskState_msg.obs_received = "NONE";
    //    taskState_msg.task_status = "ONGOING";
    //    taskState_msg.who_succeeded = "NONE";

    //    taskState_msg.immediate_reward = 0.0;
    //    taskState_msg.total_disc_reward = 0.0;

    taskStatusPublisher.publish(taskState_msg);

    if (req.human_update.human_real_state == "GlobalSuccess" || req.human_update.human_real_state == "GlobalFail"){
        // checked when the task is finished to start a new one (all agents need to inform)
        human_has_informed = true;
    }

    step_counter += 1;
    //if (step_counter > 20){
      // TODO: a bug in the system that sometimes human wont receive success. That causes to iterate forever.
      // This is a temporary solution to hard stop the task on the human's side.
    //  ros::param::set("/human_observable_state", 1);
    //  task_stuck_flag = true;
    //}
    res.task_number = task_number; // informing human about the ID (task number of the current task)
    res.success = true;
    return res.success;
}*/

// TODO throw from observation agent
//TODO: TASK STATE MSG STRUCTURE HAS BEEN CHANGED
bool TaskManager::ObsUpdater(hrc_ros::InformObsToTaskMangIERequest &req, hrc_ros::InformObsToTaskMangIEResponse &res){

    step_counter += 1;

    hrc_ros::TaskStateIE taskState_msg;

    // Additional fields for interaction experiment & assigning global variables
        global_obs_mapped_observation_pomdp = req.obs_update.mapped_observation_pomdp;
    taskState_msg.mapped_observation_pomdp = global_obs_mapped_observation_pomdp;
        global_obs_mapped_observation_raw = req.obs_update.mapped_observation_raw;
    taskState_msg.mapped_observation_raw   = global_obs_mapped_observation_raw;

    taskState_msg.task_id = task_number;
    taskState_msg.step_count = step_counter;
    taskState_msg.who_reports = "OBSERVATION";

    taskState_msg.update_received_time = req.obs_update.stamp_obs_update;
        global_obs_real_obs_received = req.obs_update.real_obs_received;
    std::vector<uint8_t> real_obs_msg = global_obs_real_obs_received;

    // Observation array =                     [LA = Is human Looking Around?,
    //                                          H.DET = Is human detected?,
    //                                          GRASP = Has human graspped?,
    //                                          SUCCESS = Has task reached to success?,
    //                                          WARN = Has human warned?,
    //                                          IDLE = Is human idling?,
    //                                          FAIL = Has the task failed?
    //                                          SUBTASKST. = Is subtask successful 1 | failed 3 | ongoing 0 ]


    string real_observation_array = "LA: " + to_string(real_obs_msg[0]) + " || H.DET: " + to_string(real_obs_msg[1]) +
            " || GRASP: " + to_string(real_obs_msg[2]) + " || SUCCESS: " + to_string(real_obs_msg[3]) + " || WARN: " + to_string(real_obs_msg[4]) +
            " || IDLE: " + to_string(real_obs_msg[5]) + " || FAIL: " + to_string(real_obs_msg[6]) + " || SUBTASKST: " + to_string(real_obs_msg[6]) ;
    ROS_INFO("[TASK_MANAGER]: Real Observables: %s", real_observation_array.c_str());
    taskState_msg.real_obs_received = real_observation_array;
    taskState_msg.real_obs_received_array = real_obs_msg;

    // Human observables are the observation vector only related to the human actions and situation:
    // human_obs as vector<bool> =                  [Human Detected?,
    //                                               Human is Looking around?,
    //                                               Human has graspped successfully?, // if this and the next one both are false then human didnt grasp
    //                                               Human has failed in grasping?,
    //                                               Human is warning?,
    //                                               Human is staying idle?]

    // TODO CAN decide what to do with the human_state. Especially grasp_success is not present anymore
    bool human_grasp_success = false;
    if (real_obs_msg[2] && !real_obs_msg[7]) // real_obs_msg[7] holds the grasp failed info
        human_grasp_success = true;
    std::vector<uint8_t> human_obs;
    human_obs.push_back(real_obs_msg[1]); human_obs.push_back(real_obs_msg[0]); human_obs.push_back(human_grasp_success);
    human_obs.push_back(real_obs_msg[7]); human_obs.push_back(real_obs_msg[4]); human_obs.push_back(real_obs_msg[5]);
    taskState_msg.human_observables = human_obs;

/*
    // TODO remove and send from observation agent
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
*/



    taskStatusPublisher.publish(taskState_msg);

    res.success = true;
    return res.success;

}

bool TaskManager::RobotStatusUpdater(hrc_ros::InformRobotToTaskMangRequest &req, hrc_ros::InformRobotToTaskMangResponse &res){

    hrc_ros::TaskStateIE taskState_msg;

    taskState_msg.task_id = task_number;
    taskState_msg.step_count = step_counter;
    taskState_msg.who_reports = "ROBOT";

    taskState_msg.update_received_time = req.robot_update.stamp_robot_update;
    taskState_msg.action_taken_time = req.robot_update.action_taken_time;
    taskState_msg.taken_action = req.robot_update.robot_action_taken;
    taskState_msg.belief_state = req.robot_update.robot_belief_state;
    taskState_msg.real_state = req.robot_update.robot_real_state;


    // TODO remove after debugging
    //cout << "#RobotStatusUpdater:   Robot_real_state =  " <<  req.robot_update.robot_real_state  << " Robot_belief_state: " << req.robot_update.robot_belief_state << endl;
    if (req.robot_update.robot_belief_state == req.robot_update.robot_real_state)
        taskState_msg.isEstimationCorrect = true;
    else
        taskState_msg.isEstimationCorrect = false;

    // INformation below are for observation and robot status updates. Otherwise leave empty
    /*taskState_msg.obs_received = "NONE";
        taskState_msg.task_status = "ONGOING";
        taskState_msg.who_succeeded = "NONE";
        */

    // TODO Elia - introduce global var
    taskState_msg.robot_model = robot_model;
        global_immediate_reward = req.robot_update.immediate_reward;
    taskState_msg.immediate_reward = global_immediate_reward;
        global_total_disc_reward = req.robot_update.total_disc_reward;
    taskState_msg.total_disc_reward = global_total_disc_reward;
    taskState_msg.robot_belief = robot_belief;


    taskStatusPublisher.publish(taskState_msg);

    // TODO CAN check
    // TODO double check which robot state should be elaborated (real_state or belief_state [ real state is only delayed version of belief_state currently])  (was robot_real_state before) -> how to get the real state -> currently it equals  the detected state
    if (req.robot_update.robot_real_state == "GlobalSuccess" || req.robot_update.robot_real_state == "GlobalFail"){
        // checked when the task is finished to start a new one (all agents need to inform)
        robot_has_informed = true;
    }

    CheckToStartNewTask();

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
        task_time = 0;
    }
    // Check if the task is accomplished (all the agents should acknowledge the success or fail)
    //CheckToStartNewTask();
}


// Observation agent informs about trayupdate and task und subtask statistics
void TaskManager::ReceiveTraySuccessStatus(const hrc_ros::SuccessStatusObserved &msg){

    string subtaskStatus = "ONGOING";

    task_has_finished = false;

    if( ( msg.subtask_success_status.compare("success") == 0 ) ) {
        subtaskStatus = "SUCCESS";
    } else {
        subtaskStatus = "FAIL";
    }

    if( (msg.task_success_status.compare("success") == 0) ) {
        task_has_finished = true;
        task_success_statistics += 1;
        //subtask_success_statistics = msg.successful_subtasks;
        //final_state_statistics = "GlobalSuccess";
        // TODO remove
        //cout << endl << endl << " Global success received " << endl;
    } else if ( (msg.task_success_status.compare("fail") == 0) ) {
        task_has_finished = true;
        task_fail_statisctics += 1;
        //subtask_fail_statistics = msg.failed_subtasks;
        //final_state_statistics = "GlobalFail";
        //cout << endl << endl << " Global fail received " << endl;
    }



    global_stat_tray_update_stamp = msg.stamp;
    global_stat_subtask_success_status = msg.subtask_success_status;
    global_stat_task_success_status = msg.task_success_status;

    //## Debug values | also for statistics
    // uint32 current_object
    // uint32 current_tray
    // uint32 success_tray
    global_stat_task_counter = msg.task_counter;
    global_stat_subtask_counter = msg.subtask_counter;

    //## values mainly used for statistics
    global_stat_failed_subtasks = msg.failed_subtasks;
    global_stat_successful_subtasks = msg.successful_subtasks;
    global_stat_subtask_time_seconds = msg.subtask_time_seconds;
    global_stat_task_combined_subtask_time_seconds = msg.task_combined_subtask_time_seconds;
    global_stat_percentage_successful_subtasks = msg.percentage_successful_subtasks;
    global_stat_who_succeeded = msg.who_succeeded;
    global_stat_task_warnings_received = msg.task_warnings_received;
    global_stat_successful_tasks_cnt = msg.successful_tasks_cnt;
    global_stat_failed_tasks_cnt = msg.failed_tasks_cnt;
    global_stat_percentage_successful_tasks = msg.percentage_successful_tasks;

    // TODO remove this publishing part if the success statistics should only be published after a task is done
    hrc_ros::TaskStateIE taskState_msg;

    taskState_msg.task_id = task_number;
    taskState_msg.step_count = step_counter;
    taskState_msg.update_received_time = ros::Time::now();
    taskState_msg.who_reports = "SENSOR"; // was Observation-Tray-update before.

    taskState_msg.task_duration = msg.task_combined_subtask_time_seconds;
    taskState_msg.tray_update_received_time = global_stat_tray_update_stamp;
    taskState_msg.subtask_status = global_stat_subtask_success_status;
    taskState_msg.subtask_id = global_stat_subtask_counter;
    taskState_msg.failed_subtasks = global_stat_failed_subtasks;
    taskState_msg.successful_subtasks = global_stat_successful_subtasks;
    taskState_msg.subtask_duration = global_stat_subtask_time_seconds;
    taskState_msg.percentage_successful_subtasks = global_stat_percentage_successful_subtasks;
    taskState_msg.who_succeeded_subtask =  global_stat_who_succeeded;
    // taskState_msg.task_warnings_received = global_stat_task_warnings_received;
    taskState_msg.warnings_count_subtask = msg.subtask_warnings_received;
    taskState_msg.successful_tasks_cnt = global_stat_successful_tasks_cnt;
    taskState_msg.failed_tasks_cnt = global_stat_failed_tasks_cnt;
    taskState_msg.percentage_successful_tasks = global_stat_percentage_successful_tasks;

    // add statistics rewards (informed by robot agent)
    // taskState_msg.immediate_reward  = global_immediate_reward;
    taskState_msg.total_disc_reward = global_total_disc_reward;


    taskStatusPublisher.publish(taskState_msg);


    subtask_counter += 1;
    // TODO remove after debugging
    ROS_INFO("\n\n ++ Tray success status received ++");

    CheckToStartNewTask();
/*
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
*/

}



// TODO get task_has_finished from ovservation agent => Test this!!
void TaskManager::CheckToStartNewTask(void){

    //cout << "#CheckToStartNewTask:   task_has_finished: " << task_has_finished << "   robot_has_informed:   " << robot_has_informed << endl;
    //cout << "#CheckToStartNewTask:   task_counter: " << task_number << "   subtask_counter:  " << subtask_counter << endl;
    // TODO: check if task_has_finished && robot_hs_informed is stil relevant
    if ( (task_has_finished && robot_has_informed ) || task_stuck_flag || (subtask_counter > subtask_number_current_task)){


        // TODO remove printout
        //cout << endl << " -> -> Initiation will be triggered (in if) " << endl << " Task_stuck_flag : " << task_stuck_flag << "subtask_counter : " << subtask_counter << " subtask_number_current_task : " << subtask_number_current_task << endl;
        //cout << " task_has_finished = " << task_has_finished << " robot_has_informed : = " << robot_has_informed << endl << endl << endl;
        // Task status informers reset
        task_has_finished = false;
        human_has_informed = false;
        robot_has_informed = false;
        task_stuck_flag = false;

        // Publish final statistics with task_status topic
        hrc_ros::TaskStateIE taskState_msg;

        taskState_msg.task_id = task_number;
        taskState_msg.step_count = step_counter;
        taskState_msg.who_reports = "MANAGER-TASK-DONE"; // was MANAGER before

        // TODO which timestamp and how to get those
        /*taskState_msg.update_received_time = req.robot_update.stamp_robot_update;
        taskState_msg.action_taken_time = req.robot_update.action_taken_time;
        taskState_msg.taken_action = req.robot_update.robot_action_taken;
        taskState_msg.belief_state = req.robot_update.robot_belief_state;
        taskState_msg.real_state = req.robot_update.robot_real_state;
        */

        /*# below is only for observations
        string  real_obs_received
        uint8[] real_obs_received_array
        bool[] human_observables # this is for creating training set for type estimation
        string task_status # success, fail or ongoing
        # this var is doubel TODO decide which one to remove
        string who_succeeded # if the task has been succeeded who did then
        #ADDED for interaction experiment
        uint32 mapped_observation_pomdp
        uint32 mapped_observation_raw


        # below are specific to robot status update
        string robot_model
        string immediate_reward
        string total_disc_reward
        float32[] robot_belief

        */

        // #######  publish final statistics before starting new task -> those are received by tray_update ###########
        taskState_msg.update_received_time = ros::Time::now();
        taskState_msg.tray_update_received_time = global_stat_tray_update_stamp;
        taskState_msg.task_status = global_stat_task_success_status;
        taskState_msg.subtask_id = global_stat_subtask_counter;
        taskState_msg.failed_subtasks = global_stat_failed_subtasks;
        taskState_msg.successful_subtasks = global_stat_successful_subtasks;
        taskState_msg.task_duration = global_stat_task_combined_subtask_time_seconds;
        taskState_msg.percentage_successful_subtasks = global_stat_percentage_successful_subtasks;
        taskState_msg.warnings_count_task = global_stat_task_warnings_received;
        taskState_msg.successful_tasks_cnt = global_stat_successful_tasks_cnt;
        taskState_msg.failed_tasks_cnt = global_stat_failed_tasks_cnt;
        taskState_msg.percentage_successful_tasks = global_stat_percentage_successful_tasks;

        // add statistics rewards (informed by robot agent)
        // taskState_msg.immediate_reward  = global_immediate_reward; //TODO: dont think we need this
        taskState_msg.total_disc_reward = global_total_disc_reward;



        // Publish
        taskStatusPublisher.publish(taskState_msg);


        // trigger displaying of score
        hrc_ros::DisplayScoring::Request  score_display_req;
        hrc_ros::DisplayScoring::Response score_display_resp;

        score_display_req.task_duration = global_stat_task_combined_subtask_time_seconds;
        score_display_req.percentage_successful_subtasks = global_stat_percentage_successful_subtasks;
        std::string::size_type sz;
        score_display_req.reward_scoring_task = global_total_disc_reward;
        display_score_client.call(score_display_req,score_display_resp);



        // ###### Reset all subtask relevant variables
        // TODO : save all relevant statistics to file -> afterwards savely reset all variables
        //std::ofstream StatisticFile;
        //StatisticFile.open("StatisticsTest.csv");
        //StatisticFile << "Task,Successfull_Subtasks,FailedSubtask,FinalState" << endl;
        //StatisticFile << task_counter << "," << subtask_success_statistics << "," << subtask_fail_statistics << "," << final_state_statistics;
        //StatisticFile.close();
        subtask_success_statistics = 0;
        subtask_fail_statistics    = 0;
        // TODO remove after statistics test -> not used anymore final_state_statistics     = "Reset";



        // increment subtask_counter -> task_counter is incremented when new scanario is triggered !!!
        subtask_counter = 1;



        if (task_number + 1 > (current_global_task_config.task_max) ){
            //cout << endl << endl << " All tests are done - :-) " << endl << endl;


        } else {
            //cout << "#CheckToStartNewTask:   task_counter:   " << task_number << "     => call initiateScenario" << endl;

            // TODO remove if manual staring of scenario is ok
            hrc_ros::InitiateScenario::Request req_init;
            hrc_ros::InitiateScenario::Response res_init;
            //initiateScenario(req_init, res_init); // uncomment if next task should be started automatically
            ROS_WARN("Task is done - please start a new one by calling   /task_manager_IE/new_scenario_request   service  \n");

        }


    }

}
