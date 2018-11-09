/*
 *  Created on: 19.04.2018
 *      Author: Orhan Can Görür
 *      Email: orhan-can.goeruer@dai-labor.de
 */

#include <ros/ros.h>

#include <stdlib.h> 				// for rand() and RAND_MAX
#include <string>
#include <thread>

#include <human_agent/HumanAgent.h>

HumanAgent::HumanAgent() {
    ros::NodeHandle nh("~");
    initialize();
}

HumanAgent::~HumanAgent() {
}


void HumanAgent::initialize(){
    ros::NodeHandle nh("~");

    //init statemap
    states = {"TaskHuman", "GlobalSuccess", "GlobalFail", "FailedToGrasp", "NoAttention", "Evaluating",
              "Tired", "Recovery", "RobotInterfered", "WarningTheRobot", "RobotIsWarned"};
    for(int s = 0; s < states.size();s++)
    {
        stateMap.insert(pair<string, int>(states[s], s));
    }

    //here set the parameter of conveyor belt to "init" so it automatically starts

    /*
     * Initializing ros services
     */
    callObs_inform_action = nh.serviceClient<hrc_ros::InformHumanAction>("/observation_agent/inform_human_action");
    callObs_inform_newState = nh.serviceClient<hrc_ros::InformHumanState>("/observation_agent/inform_new_human_state");
    callTaskMang_inform = nh.serviceClient<hrc_ros::InformHumanToTaskMang>("/task_manager/human_status_update");

    /// ROS services by MORSE to call for the human actions
    stayIdle = nh.serviceClient<std_srvs::Trigger>("/human/stay_idle");
    walkAway = nh.serviceClient<std_srvs::Trigger>("/human/walk_away");
    lookAround = nh.serviceClient<std_srvs::Trigger>("/human/look_around");
    warnRobot = nh.serviceClient<std_srvs::Trigger>("/human/warn_robot");
    attemptGrasp = nh.serviceClient<std_srvs::Trigger>("/human/attempt_grasp");
    grasp = nh.serviceClient<std_srvs::Trigger>("/human/grasp");

    reset_scenario = nh.advertiseService("/human_agent/reset", &HumanAgent::resetScenario, this);
//    reset_scenario = nh.advertiseService("/human/reset", &HumanAgent::resetScenario, this);

    humanStateSamplerClient = nh.serviceClient<hrc_ros::DrawNewStateMC>("/human_mc_sampler/human_mc_sampler");
    requestGraspingOutcome = nh.serviceClient<hrc_ros::DrawNewStateMC>("/human_mc_sampler/grasping_outcome");

    // loop at 2Hz until the node is shut down
    // ros::Rate rate(0.2); // No need for this when running a thread as a service

    ROS_INFO("[HUMAN AGENT] Human Agent is created !");
    update();
}

bool HumanAgent::resetScenario(hrc_ros::ResetHumanROSRequest &req,
                               hrc_ros::ResetHumanROSResponse &res) {
    human_action_taken = "";
    human_belief_state = "";
    human_real_state = "";
    initial_state_received = true;
    newstate_info_received = false;
    action_info_received = false;
    terminal_state_reached = false;

    ROS_INFO("[HUMAN AGENT] Reset!");

    res.success = true;
    return true;
}

void HumanAgent::update() {
    // More information about how this func is designed is under HumanAgent.h
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    srand (time(NULL));

    auto& echo=server.endpoint["^/?$"];
    /// Setting the config port number. Port is configured under HumanAgent.h
    server.config.port = port;

    echo.on_open=[](shared_ptr<WsServer::Connection> connection) {
        //cout << "Server: Opened connection " << (size_t)connection.get() << endl;
        //send new state on open
    };

    echo.on_message=[&](shared_ptr<WsServer::Connection> connection, shared_ptr<WsServer::Message> message) {

        // ROS_INFO("##########################human agent receives message###################################");
        ros::spinOnce(); // TODO: intended to catch the reset service call

        string message_received = message->string();

        // interprete action and state
        std::size_t index = message_received.find(",");
        string human_action = message_received.substr(0, index);
        string human_state = message_received.substr(index+1);

        string sampled_state = "unknown";
        string sampled_state_toSend = "-1";

        // Check if a new task has started (human DESPOT reinitiated)
        // We are checking the initial state thru rosparam set by task manager
        // TODO: this is because human ROS reset service is not working with websocket
        ros::param::get("/human_initial_state", initial_state_received);
        if (initial_state_received){
            human_real_state = human_state; // assign the initial state
        }

        // EVERY MESSAGE IS EITHER A HUMAN ACTION BELIEF UPDATE OR A HUMAN REAL STATE UPDATE

        // HUMAN ACTION UPDATE RECEIVED. INFORMING ABOUT BOTH ACTION AND THE STATE (either initial or belief state)
        /* call action services*/
        if (stringToActionInt(human_action) != ACTION::UNKNOWN_ACTION){

            std_srvs::Trigger::Request req;
            std_srvs::Trigger::Response resp;

            action_taken_time = ros::Time::now();

            //calling human_sampler node
            hrc_ros::DrawNewStateMC draw_srv;
            draw_srv.request.human_action = stringToActionInt(human_action);
            draw_srv.request.current_human_state = stringToStateInt(human_state);

            //let the new_state be the previous state at first
            sampled_state = human_state;
            sampled_state_toSend = to_string(stringToStateInt(human_state));

            // TODO: For the expert case fix the probabilities below
            bool success = true; // TODO: this is not used!
            if (stringToActionInt(human_action) == ACTION::GRASP){

                //requesting grasping outcome
                requestGraspingOutcome.waitForExistence();
                if (!requestGraspingOutcome.call(draw_srv))
                {
                    ROS_ERROR("[HUMAN AGENT] Failed to call service human_mc_sampler");
                }
                sampled_state = states.at(draw_srv.response.new_state);
                sampled_state_toSend = to_string(draw_srv.response.new_state);

                if(sampled_state == "FailedToGrasp") {	// grasp or grasp attempt
                    attemptGrasp.call(req, resp);
                    ROS_INFO_STREAM("[HUMAN AGENT] Current action is human attemping grasping...");
                }
                else if(sampled_state == "GlobalSuccess") {
                    grasp.call(req, resp);
                    ROS_INFO("[HUMAN AGENT] Current action is human grasping...");
                }
                else
                {
                    // Could also iterate on evaluating. very very less likely though
                    attemptGrasp.call(req, resp);
                    ROS_INFO_STREAM("[HUMAN AGENT] Current action is human attemping grasping...");
                    ROS_INFO("[HUMAN AGENT] After action was grasp, state was drawn other than success or FailedToGrasp");
                }
            }
            else if (stringToActionInt(human_action) == ACTION::LOOK_AROUND) {
                ROS_INFO_STREAM("[HUMAN AGENT] Current action is human looking around...");
                lookAround.call(req, resp);
            }
            else if (stringToActionInt(human_action) == ACTION::IDLE) {
                ROS_INFO_STREAM("[HUMAN AGENT] Current action is human staying idle...");
                stayIdle.call(req, resp);
            }
            else if (stringToActionInt(human_action) == ACTION::WALK_AWAY) {
                ROS_INFO_STREAM("[HUMAN AGENT] Current action is human walking away...");
                walkAway.call(req, resp);
            }
            else if (stringToActionInt(human_action) == ACTION::WARN_ROBOT) {
                if (initial_state_received || human_state == "TaskHuman" || human_state == "Evaluating"){
                    human_action = "2"; // assign idle action to wrongly executed warn action
                    ROS_INFO_STREAM("[HUMAN AGENT] Current action is human staying idle...");
                    stayIdle.call(req, resp);
                }else{
                    ROS_INFO_STREAM("[HUMAN AGENT] Current action is human warning robot...");
                    warnRobot.call(req, resp);
                }
            }
            else {
                success = true;
                //ROS_INFO_STREAM("[HUMAN AGENT] [HUMAN AGENT] Current action is human ERROR !!!");
            }

            /* THIS IS TO LET OBS_AGENT KNOW THAT HUMAN ACTED!*/
            hrc_ros::InformHumanAction::Request req_action;
            hrc_ros::InformHumanAction::Response res_action;

            req_action.human_action = human_action;
            callObs_inform_action.call(req_action, res_action); // response is a boolean about the status of the service call

            // ####### here sampler waits till robot acted and reacts accordingly ###########
            // NOTE: if the action is grasp, since it was sampled already human_mc_sampler ignores the request)
            draw_srv.request.human_action = stringToActionInt(human_action);
            draw_srv.request.current_human_state = stringToStateInt(sampled_state);//

            humanStateSamplerClient.waitForExistence();
            if (!humanStateSamplerClient.call(draw_srv))
            {
                ROS_ERROR("[HUMAN AGENT] Failed to call service human_mc_sampler");
            }

            sampled_state = states.at(draw_srv.response.new_state);
            sampled_state_toSend = to_string(draw_srv.response.new_state);

            // Below the global variables are set! These are for sending the status later to the task manager
            human_action_taken = human_action;
            human_belief_state = human_state;
            action_info_received = true; // raise the action received flag (NOT USED ANYMORE)

            // ############ SENDING THE NEW STATE DRAWN ABOVE TO DESPOT ############
            // send back new message to despot human
            // ROS_ERROR("sending state %s to despot", sampled_state_toSend.c_str());
            auto send_stream=make_shared<WsServer::SendStream>();
            *send_stream << sampled_state_toSend;
            server.send(connection, send_stream, [](const SimpleWeb::error_code& ec){
                if(ec) {
                    cout << "[HUMAN AGENT] Server: Error sending message. " <<
                            "Error: " << ec << ", error message: " << ec.message() << endl;
                }
            });
        }

        // HUMAN STATE UPDATE IS RECEIVED. INFORMING ABOUT THE NEW HUMAN STATE RECEIVED.
        // THIS IS FOLLOWED BY THE ACTION SELECTION THEREFORE THIS NEW STATE IS THE REAL STATE OF THE NEXT STEP
        else {
            ROS_INFO("[HUMAN AGENT] Real state informed after the action is: %s", human_action_taken.c_str());
            // ROS_INFO("HUMAN ROS: Real state informed after the action is: %s", human_state.c_str());
            hrc_ros::InformHumanState::Request req_state;
            hrc_ros::InformHumanState::Response res_state;
            req_state.new_human_state = human_state; //
            callObs_inform_newState.call(req_state, res_state); // response is a boolean about the status of the service call

            // global variables are assigned ! human_real_state holds the actual state human is in (equals to the sampled_state)
            human_real_state = human_state;
            newstate_info_received = true; // raise the flag for task manager informing below

            ROS_INFO("[HUMAN AGENT] Message received: %s", message_received.c_str());
            if (human_state == "GlobalSuccess" || human_state == "GlobalFail"){
                // This flag will directly end the iteration by sending all info to TaskManager ()
                terminal_state_reached = true;
            }

            // ############ SENDING ACKNOWLEDGEMENT ############
            // send back acknowledgement to despot human. As the despot client waits for new state
            // this ack msg returns -1 to inform the client to ignore the msg content
            auto send_stream=make_shared<WsServer::SendStream>();
            *send_stream << "-1";
            server.send(connection, send_stream, [](const SimpleWeb::error_code& ec){
                if(ec) {
                    cout << "[HUMAN AGENT] Server: Error sending message. " <<
                            "Error: " << ec << ", error message: " << ec.message() << endl;
                }
            });
        }

        if ((action_info_received && newstate_info_received) || terminal_state_reached || initial_state_received){
            // ############ SENDING HUMAN UPDATE TO TASK MANAGER ############
            // task manager is informed after state information is received and the human acted on it
            // or is informed when a terminal state is reached
            hrc_ros::InformHumanToTaskMang::Request reqForUpdate;
            hrc_ros::InformHumanToTaskMang::Response resForUpdate;

            hrc_ros::HumanUpdateMsg update_msg;

            update_msg.stamp_human_update = ros::Time::now();
            update_msg.action_taken_time = action_taken_time;
            update_msg.human_belief_state = human_belief_state; // belief state
            update_msg.human_action_taken = human_action_taken; // took action in the belief state

            // This is to inform the initial state
            if (initial_state_received){
                update_msg.human_real_state = human_belief_state; // in initial state human mdp sends the init state directly
                initial_state_received = false;
                ros::param::set("/human_initial_state", false);
            }else{
                update_msg.human_real_state = human_real_state; // new state
            }

            reqForUpdate.human_update = update_msg;

            callTaskMang_inform.call(reqForUpdate, resForUpdate);
            task_number_counter = resForUpdate.task_number;

            action_info_received = false; // lower the action received flag
            newstate_info_received = false; // lower the state received flag
            terminal_state_reached = false; // lower the terminal state reached flag
        }

        /*
        // call new scene when despot is informing about new state i.e. human_action is -1
        if(stringToActionInt(human_action) == ACTION::UNKNOWN_ACTION && (human_state == "GlobalSuccess" || human_state == "GlobalFail"))
        {
                ROS_INFO("[HUMAN AGENT]: %s detected: requesting new scene!", human_state.c_str());
                hrc_ros::InitiateScenario::Request req;
                hrc_ros::InitiateScenario::Response res;
                new_scenario_client.call(req, res);
        }
        */
    };

    server.start();
}


HumanAgent::STATE HumanAgent::stringToStateInt(string state)
{
    int s = stateMap[state];
    switch(s)
    {
        case HumanAgent::STATE::TASKHUMAN:       return HumanAgent::STATE::TASKHUMAN;
        case HumanAgent::STATE::GLOBAL_SUCCESS:  return HumanAgent::STATE::GLOBAL_SUCCESS;
        case HumanAgent::STATE::GLOBAL_FAIL:     return HumanAgent::STATE::GLOBAL_FAIL;
        case HumanAgent::STATE::FAILED_TO_GRASP: return HumanAgent::STATE::FAILED_TO_GRASP;
        case HumanAgent::STATE::NO_ATTENTION:    return HumanAgent::STATE::NO_ATTENTION;
        case HumanAgent::STATE::EVALUATING:      return HumanAgent::STATE::EVALUATING;
        case HumanAgent::STATE::TIRED:           return HumanAgent::STATE::TIRED;
        case HumanAgent::STATE::RECOVERY:        return HumanAgent::STATE::RECOVERY;
        case HumanAgent::STATE::ROBOT_INTERFERED:return HumanAgent::STATE::ROBOT_INTERFERED;
        case HumanAgent::STATE::WARN:            return HumanAgent::STATE::WARN;
        case HumanAgent::STATE::ROBOT_IS_WARNED: return HumanAgent::STATE::ROBOT_IS_WARNED;
        case HumanAgent::STATE::TASK_ROBOT:      return HumanAgent::STATE::TASK_ROBOT;
        default:                                 return HumanAgent::STATE::UNKNOWN_STATE;
    }
}

HumanAgent::ACTION HumanAgent::stringToActionInt(string action)
{
    int a = stoi(action);
    switch(a)
    {
        case HumanAgent::ACTION::GRASP:         return HumanAgent::ACTION::GRASP;
        case HumanAgent::ACTION::LOOK_AROUND:   return HumanAgent::ACTION::LOOK_AROUND;
        case HumanAgent::ACTION::IDLE:          return HumanAgent::ACTION::IDLE;
        case HumanAgent::ACTION::WALK_AWAY:     return HumanAgent::ACTION::WALK_AWAY;
        case HumanAgent::ACTION::WARN_ROBOT:    return HumanAgent::ACTION::WARN_ROBOT;
        default:                                return HumanAgent::ACTION::UNKNOWN_ACTION;
    }
}

std::string HumanAgent::intToAction(const int taken_action)
{
    switch(taken_action)
    {
        case 0: return "grasp";
        case 1: return "look_around";
        case 2: return "idle";
        case 3: return "walk_away";
        case 4: return "warn_robot";
        default: return "unknown";
    }
}
