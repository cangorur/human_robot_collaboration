/*
 * experimentMonitorNode.cpp
 *
 *  Created on: 18.04.2019
 *      Author: Elia Kargruber
 */

#include <ros/ros.h>
#include<stdlib.h>
#include <ros/package.h>

#include <string>


#include<ros/callback_queue.h>
#include<std_msgs/Bool.h>
#include <hrc_ros/DisplayTaskRule.h> 
#include <hrc_ros/DisplayScoring.h>  
#include <hrc_ros/TaskStateIE.h>

// generic service includes 
#include <std_srvs/Trigger.h>

// includes for Json parsing
//#include <helper_functions/Json_parser.h>
//#include <vector>


using namespace std;

//namespace pt = boost::property_tree; 
//using boost::property_tree::ptree;

// All of this could go into a header 
	
ros::ServiceServer display_task_rules;
ros::ServiceServer display_scoring_server; 
ros::ServiceServer distract_participants_server; 

ros::Subscriber taskStatus_subs;

// define colours for coloured prints 
const std::string red("\033[1;31m");
const std::string green("\033[1;32m");
const std::string blue("\033[1;34m");
const std::string normal("\033[1;37m");	

// variables and structs to parse Json 
//task_set current_task_set;
//success_combo success_criteria_read;
//global_task_config global_task_config_read; 

// Fixed sequence arrays to display subtask_type 3 - the number indicate the sequence of the objects 
vector<int> red_subtask_order_vect;   // 1,4,7,10
vector<int> green_subtask_order_vect; // 2,6,8
vector<int> blue_subtask_order_vect;  // 3,5,9 

// variables to print statistics 

// DONE 
string DONE_task_status					= " ";
int Global_subtask_id					= 0; 
int DONE_failed_subtasks                = 0; 
int DONE_successful_subtasks			= 0; 
double DONE_task_duration                  = 0.0;
double DONE_percentage_successful_subtasks = 0.0; 
int DONE_warnings_count					= 0; 
int DONE_successful_tasks				= 0; 
int DONE_failed_tasks					= 0; 
double DONE_percentage_successful_tasks	= 0.0; 
string Global_total_disc_reward 			= ""; 

// ROBOT 
string ROBOT_immediate_reward				= ""; 
string ROBOT_total_dics_reward				= ""; 
std::vector<float> ROBOT_robot_belief			; 
int Global_task_id							= 0;
int Global_subtask_number					= 0; 
string ROBOT_taken_action					= " ";
string ROBOT_belief_state					= " ";
string ROBOT_real_state					= " ";

// OBSERVATION 
int OBSERVATION_mapped_observation_pomdp	= 0; 
string OBSERVATION_real_obs					= ""; 


// SENSOR 
// Global_task_id
int Global_step_count								= 0; 
string SENSOR_subtask_status							= " ";
//int Global_subtask_id								= 0; 
int SENSOR_failed_subtasks						= 0; 
int SENSOR_successful_subtasks					= 0; 
double SENSOR_subtask_duration					= 0.0; 
double Global_percentage_successful_subtasks	= 0.0; 
string SENSOR_who_succeeded						= " ";
int Global_successful_tasks_cnt					= 0; 	
int Global_failed_tasks_cnt						= 0; 
double Global_percentage_successful_tasks		= 0.0; 
string Global_immediate_reward					= " "; 
//double Global_total_disc_reward					= 0.0; 




//#################### function declarations ##################### 
std::string mapintToString_colour(int int_value, std::string & print_string);
bool display_task_rules_server(hrc_ros::DisplayTaskRuleRequest &req,hrc_ros::DisplayTaskRuleResponse &res);
void clear_screen(void);
std::string mapintToEnumeration(int number);
void update_statistics_display(void); 
void receive_task_status(const hrc_ros::TaskStateIE &msg);

// ########## Function implementations ##########################

std::string mapintToString_colour(int int_value, std::string & print_string){

	 std::string return_string = normal;  

	if (int_value == 1) {// red
		return_string = red;
		print_string = " red ";  
	} else if (int_value ==2){ // green
		return_string = green; 
		print_string = " green ";
	} else if (int_value == 3){
		return_string = blue;
		print_string = " blue ";
	}
	 
	return return_string; 
}

std::string mapintToEnumeration(int number){
	string enum_string = "";
	if (number == 1){
		enum_string =" 1st ";
	} else if ( number == 2){
		enum_string = " 2nd ";
	} else if ( number == 3){
		enum_string = " 3rd ";
	} else if ( number == 4){
		enum_string = " 4th ";
	}

	return enum_string; 
}

/*bool display_task_rules_server(hrc_ros::DisplayTaskRuleRequest &req,hrc_ros::DisplayTaskRuleResponse &res){

	int task_counter = req.task_counter; 


	clear_screen(); 


	//### parse task_rule file 

	// Call the parse function to parse the scenario definition file 
	boost::property_tree::ptree testscenario_pt;
	string pkg_path1 = ros::package::getPath("hrc_ros");
	std::ifstream jsonFiletask(pkg_path1 + "/../../../configs/IE_task_config.json");

	try {
		boost::property_tree::read_json(jsonFiletask, testscenario_pt);
	} catch(boost::property_tree::json_parser::json_parser_error &e) {
		ROS_FATAL("Cannot parse the message to Json. Error: %s", e.what());
		res.success = false;
		return false;
	}


    stringstream ss;
    ss << task_counter;
    string task_str = ss.str(); 

	current_task_set = read_task_set(task_str,testscenario_pt);
	int subtask_number_current_task = current_task_set.subtask_quantity; 


	// read in the order of coloured blocks and the associated subtasks for displaying of task_type 3 (for each colour the subtaks# of the objects is shown) 
	BOOST_FOREACH (boost::property_tree::ptree::value_type &child_object, testscenario_pt.get_child("config.red_subtasks_order")) {
		red_subtask_order_vect.push_back( stoi(child_object.second.data()) ); 
	}
	BOOST_FOREACH (boost::property_tree::ptree::value_type &child_object, testscenario_pt.get_child("config.green_subtasks_order")) {
		green_subtask_order_vect.push_back( stoi(child_object.second.data()) ); 
	}
	BOOST_FOREACH (boost::property_tree::ptree::value_type &child_object, testscenario_pt.get_child("config.blue_subtasks_order")) {
		blue_subtask_order_vect.push_back( stoi(child_object.second.data()) ); 
	}

	
	if (current_task_set.all_set == true){ // rules are the same for all subtasks 
		int subtask_counter = 1; 
		cout << endl << endl << endl << normal << "Task #: " << task_counter << "		Subtasks: " << current_task_set.subtask_quantity << endl <<"------------------------------------------------------------------" << endl << endl << endl << endl; 
		cout << "For all subtasks : "<< endl << "-----------------"<< endl << endl << endl;  
		for ( int obj = 1; obj <=3; obj++){
			// ## get strings for success_criteria request
			stringstream ss_task_counter;  
			stringstream ss_subtask_counter; 

			ss_task_counter << task_counter;
			ss_subtask_counter << subtask_counter;
			
			
			string subtask_str = ss_subtask_counter.str();
			string object_str  = object_int_to_str(obj);

			// ## determine subtask success state 
			try {
			success_criteria_read = get_success_criteria(task_str,subtask_str,object_str,testscenario_pt);
			} catch(boost::property_tree::json_parser::json_parser_error &e) {
				ROS_FATAL("Cannot parse the message to Json. Error: %s", e.what());
				return false;
			}

			string obj_print_string; 
			string container_print_string; 
			const std::string obj_colour = mapintToString_colour(obj,obj_print_string); 
			const std::string container_colour = mapintToString_colour(success_criteria_read.tray, container_print_string );

			// printing the rule with colourful printout	
			cout << "			Place " << obj_colour << obj_print_string << normal << " object into " << container_colour << container_print_string << normal << " container " << endl << endl; 
		} 

		// wait for certain time, then clear the screen and return true to start the experiment
		ros::Duration(15).sleep();
		clear_screen(); 

	} else { // different subtask rules - the different types are differentiated with subtask_type read from task_config json

		string subtask_type = get_subtask_type(task_str,testscenario_pt);

		
		if (subtask_type.compare("2_same_tray_order") ==0) { // regardless of colour the object always goes to same tray - sequence is reapeated after 3 subtasks 

			cout << endl << endl << endl << normal << "Task #: " << task_counter << "		Subtasks: " << current_task_set.subtask_quantity << endl <<"------------------------------------------------------------------" << endl << endl << endl << endl; 
			cout << "Repeat following sequence : "<< endl << "-----------------"<< endl << endl << endl; 

			
			// increment trough first 3 subtasks - regardless of colour - the following rules will be repetition of first 3 rules 
			for (int rule_i =1; rule_i<= 3; rule_i ++) {
				// static task_str - increment subtask_str - colour/object alwas 1=red 
				stringstream ss_subtask_counter;
				ss_subtask_counter << rule_i; 

				string subtask_str = ss_subtask_counter.str();
			 	string object_str  = object_int_to_str(1);

				// ## determine subtask success state 
				try {
				success_criteria_read = get_success_criteria(task_str,subtask_str,object_str,testscenario_pt);
				} catch(boost::property_tree::json_parser::json_parser_error &e) {
					ROS_FATAL("Cannot parse the message to Json. Error: %s", e.what());
					return false;
				}

				string container_print_string; 
				const std::string container_colour = mapintToString_colour(success_criteria_read.tray, container_print_string );

				// ################## Print the rules ##################################################### 
				if (rule_i == 1) {
					cout << "			Place 1st object into " << container_colour << container_print_string << normal << " container " << endl << endl; 
				} else if (rule_i ==2){
					cout << "			Place 2nd object into " << container_colour << container_print_string << normal << " container " << endl << endl; 
				} else if (rule_i ==3){
					cout << "			Place 3rd object into " << container_colour << container_print_string << normal << " container " << endl << endl;
					cout << endl << endl << "			REPEAT Sequence  " << endl << endl; 
				}
			}

			
		} else if (subtask_type.compare("3_mixed_fixed_sequence") == 0 ) { // most complex scenario - for each colour and occurence a single rule is given (4 red, 3green, 3 blue)

			cout << endl << endl << endl << normal << "Task #: " << task_counter << "		Subtasks: " << current_task_set.subtask_quantity << endl <<"------------------------------------------------------------------" << endl << endl << endl << endl; 
			cout << "Execute following sequence : "<< endl << "-----------------"<< endl << endl << endl; 

			// printing rules for red objects - loop trough the occurence index of object and retrieve and print the success criteria for this subtask 
			for (int red_i =1; red_i <=4; red_i ++){
				// static task_str - increment subtask_str => always assign occurence index of subtask_order_vect - colour/object alwas 1=red 
				
				string obj_print_string; 
				string container_print_string;
				string order_string = mapintToEnumeration(red_i);
				int obj = 1; // 1 = red 
				string object_str  = object_int_to_str(obj);
				
				stringstream ss_subtask_counter; 
				ss_subtask_counter << red_subtask_order_vect.at(red_i -1); // assign occurence index = subtask where object with colour occurs  //red_subtasks[red_i -1];
				string subtask_str = ss_subtask_counter.str(); 
				

				// ## determine subtask success state 
				try {
				success_criteria_read = get_success_criteria(task_str,subtask_str,object_str,testscenario_pt);
				} catch(boost::property_tree::json_parser::json_parser_error &e) {
					ROS_FATAL("Cannot parse the message to Json. Error: %s", e.what());
					return false;
				}

				const std::string obj_colour = mapintToString_colour(obj,obj_print_string); 
				const std::string container_colour = mapintToString_colour(success_criteria_read.tray, container_print_string );
				
				if (red_i == 1){
				cout << "			Place" << order_string << obj_colour << obj_print_string << normal << " object into " << container_colour << container_print_string << normal << " container " << endl;
				} else {
				cout << "			     " << order_string << obj_colour << obj_print_string << normal << " object into " << container_colour << container_print_string << normal << " container " << endl;	
				}
			}

			cout << endl << endl; 

			// printing rules for green objects - loop trough the occurence index of object and retrieve and print the success criteria for this subtask
			for (int green_i =1; green_i <=3; green_i ++){

				// static task_str - increment subtask_str => always assign occurence index of subtask_order_vect - colour/object alwas 2=green 
				string obj_print_string; 
				string container_print_string;
				string order_string = mapintToEnumeration(green_i);
				int obj = 2; // 2= green 
				string object_str  = object_int_to_str(obj);
				
				stringstream ss_subtask_counter; 
				ss_subtask_counter << green_subtask_order_vect.at(green_i -1); // assign occurence index = subtask where object with colour occurs  //green_subtasks[green_i -1];
				string subtask_str = ss_subtask_counter.str(); 
				

				// ## determine subtask success state 
				try {
				success_criteria_read = get_success_criteria(task_str,subtask_str,object_str,testscenario_pt);
				} catch(boost::property_tree::json_parser::json_parser_error &e) {
					ROS_FATAL("Cannot parse the message to Json. Error: %s", e.what());
					return false;
				}

				const std::string obj_colour = mapintToString_colour(obj,obj_print_string); 
				const std::string container_colour = mapintToString_colour(success_criteria_read.tray, container_print_string );
				
				if (green_i == 1){
				cout << "			Place" << order_string << obj_colour << obj_print_string << normal << " object into " << container_colour << container_print_string << normal << " container " << endl;
				} else {
				cout << "			     " << order_string << obj_colour << obj_print_string << normal << " object into " << container_colour << container_print_string << normal << " container " << endl;	
				}
			}

			cout << endl << endl;

			// printing rules for blue objects - loop trough the occurence index of object and retrieve and print the success criteria for this subtask
			for (int blue_i =1; blue_i <=3; blue_i ++){
				// static task_str - increment subtask_str => always assign occurence index of subtask_order_vect - colour/object alwas 3=blue 

				string obj_print_string; 
				string container_print_string;
				string order_string = mapintToEnumeration(blue_i);
				int obj = 3; // 3= blue 
				string object_str  = object_int_to_str(obj);
				
				stringstream ss_subtask_counter; 
				ss_subtask_counter << blue_subtask_order_vect.at(blue_i -1); // assign occurence index = subtask where object with colour occurs  //  blue_subtasks[blue_i -1];
				string subtask_str = ss_subtask_counter.str(); 
				

				// ## determine subtask success state 
				try {
				success_criteria_read = get_success_criteria(task_str,subtask_str,object_str,testscenario_pt);
				} catch(boost::property_tree::json_parser::json_parser_error &e) {
					ROS_FATAL("Cannot parse the message to Json. Error: %s", e.what());
					return false;
				}

				const std::string obj_colour = mapintToString_colour(obj,obj_print_string); 
				const std::string container_colour = mapintToString_colour(success_criteria_read.tray, container_print_string );
				
				if (blue_i == 1){
				cout << "			Place" << order_string << obj_colour << obj_print_string << normal << " object into " << container_colour << container_print_string << normal << " container " << endl;
				} else {
				cout << "			     " << order_string << obj_colour << obj_print_string << normal << " object into " << container_colour << container_print_string << normal << " container " << endl;	
				}
			}

			ros::Duration(15).sleep(); 
		}
		

		
		ros::Duration(15).sleep();
		clear_screen();
	}

   res.success = true;
   return true;
}

*/

	void clear_screen() {
    // CSI[2J clears screen, CSI[H moves the cursor to top-left corner
    std::cout << "\x1B[2J\x1B[H" << std::flush;
}



// ###################

bool display_scoring(hrc_ros::DisplayScoring::Request &req, hrc_ros::DisplayScoring::Response &res ){

	clear_screen(); 

	cout << blue << "############################################" << endl; 

	cout << normal << endl << endl << "     You scored "  << red << req.reward_scoring_task  << normal << "  points" << endl << endl; 

	cout << normal << "     Task duration: " << blue << req.task_duration << normal << " seconds" << endl << endl; 

	cout << normal << "     Correct subtasks: " << green << req.percentage_successful_subtasks << " %" << endl << endl;  


	cout << blue << "############################################" << endl; 

	res.success=true;
	return true; 
}


bool distract_participants(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

	clear_screen(); 
	cout << endl << endl << blue << "###########################################################" << endl << endl; 
	cout << red << "Tonight: " << normal << " Light " << blue << "rain showers and a " << green << " moderate breeze." << endl << endl; 
	cout << red << "Tomorrow: " << " Sunny intervals " << normal << "and a" << green << " gentle breeze. " << endl << endl;
	cout << red << "Weekend: "  << blue << "Partly cloudy" << normal << " and " << green << "light winds." << endl << endl; 
	cout << endl << endl << blue << "###########################################################" << endl << endl;

	res.success=true; 	
	return true; 
}

void receive_task_status(const hrc_ros::TaskStateIE &msg){

	if ( (msg.who_reports.compare("SENSOR") == 0) ) { // tray_update received 

		Global_task_id = msg.task_id;
		Global_step_count = msg.step_count;
		SENSOR_subtask_status = msg.subtask_status;
		Global_subtask_id = msg.subtask_id;
		SENSOR_failed_subtasks = msg.failed_subtasks;
		SENSOR_successful_subtasks = msg.successful_subtasks;
		SENSOR_subtask_duration = msg.subtask_duration;
		Global_percentage_successful_subtasks = msg.percentage_successful_subtasks;
		SENSOR_who_succeeded = msg.who_succeeded_subtask;
		
		Global_successful_tasks_cnt = msg.successful_tasks_cnt;
		Global_failed_tasks_cnt = msg.failed_tasks_cnt;
		Global_percentage_successful_tasks = msg.percentage_successful_tasks;

		
		//Global_immediate_reward = msg.immediate_reward;
		Global_total_disc_reward = msg.total_disc_reward;

	} else if ( msg.who_reports.compare("OBSERVATION") ==0 ){ // observations by observation agent right before sent to the DESPOT for decision making 

		OBSERVATION_mapped_observation_pomdp 			= msg.mapped_observation_pomdp; 
		OBSERVATION_real_obs							= msg.real_obs_received; 
														 

	} else if ( ( msg.who_reports.compare("ROBOT") == 0) ) { // robot beliefs 

		ROBOT_immediate_reward = msg.immediate_reward;
		ROBOT_total_dics_reward = msg.total_disc_reward; 
		ROBOT_robot_belief = msg.robot_belief;
		Global_task_id = msg.task_id;
		Global_subtask_number = msg.step_count;
		ROBOT_taken_action = msg.taken_action;
		ROBOT_belief_state = msg.belief_state;
		ROBOT_real_state = msg.real_state;

	} else if ( (msg.who_reports.compare("MANAGER-TASK-DONE"))){ // task is done final statistics 
		
        DONE_task_status = msg.task_status; 				
        Global_subtask_id =  msg.subtask_id; 					
        DONE_failed_subtasks = msg.failed_subtasks; 			
        DONE_successful_subtasks = msg.successful_subtasks; 		
        DONE_task_duration = msg.task_duration; 				
        DONE_percentage_successful_subtasks = msg.percentage_successful_subtasks; 
        DONE_warnings_count = msg.warnings_count_task; 
        DONE_successful_tasks = msg.successful_tasks_cnt; 
        DONE_failed_tasks = msg.failed_tasks_cnt; 
        DONE_percentage_successful_tasks =  msg.percentage_successful_tasks; 
        Global_total_disc_reward = msg.total_disc_reward; 

	}

	update_statistics_display(); 
}

void update_statistics_display(void){
	clear_screen(); 

	cout << normal << " Gloabal: " << endl << "	Task_cnt: " << red << Global_task_id << normal <<  "	Subatask_cnt: " << red << Global_subtask_id << endl << endl; 
	// OBSERVATION 
	cout << blue << "############ Observation ################" << endl << endl; 
	cout << normal << "observation_pomdp: " << blue << OBSERVATION_mapped_observation_pomdp << normal << endl << "real_obs: " << blue << OBSERVATION_real_obs << endl << endl; 

	// TRAY 
	cout << green << "############ Tray Update ################" << endl << endl ; 
	
	cout << normal << "Successful subtasks: " << green << SENSOR_successful_subtasks << " = " << Global_percentage_successful_subtasks << " % " << normal << "  failed_subtasks: " << green << SENSOR_failed_subtasks << endl; 
	cout << normal << "Discounted_reward: " << green << ROBOT_total_dics_reward << normal << "  scored by: " << green << SENSOR_who_succeeded << endl ; 
	cout << normal << "Duration: " << green << SENSOR_subtask_duration << normal << " seconds " << endl;  
	cout << normal << "Subtask_status: " << green << SENSOR_subtask_status << endl << endl; 
	// ## optional 	
	//string SENSOR_subtask_status							= " ";
	//string SENSOR_who_succeeded						= " ";
	//int Global_successful_tasks_cnt					= 0; 	
	//int Global_failed_tasks_cnt						= 0; 
	//double Global_percentage_successful_tasks		= 0.0; 


	// ROBOT 
	cout << blue << "############# ROBOT #######################" << endl << endl; 
	
	cout << normal << "action taken: " << blue << ROBOT_taken_action << normal << " in belief state: "<< blue  << ROBOT_belief_state << endl << endl;  
	//cout << normal << "belief state: " << blue << ROBOT_belief_state << endl << endl; 
	//cout << normal << "robot belief: " << blue << ROBOT_robot_belief.at(0) << ROBOT_robot_belief.at(1) << ROBOT_robot_belief.at(2) << endl << endl; 
	//cout << normal << "real state: "   << blue << ROBOT_real_state << endl << endl; 


	// Task is done 
	cout << blue << "############ Task Done ####################" << endl << endl; 
	
	cout << normal << " Task status: " << blue << DONE_task_status << normal << "  Duration: " << blue << DONE_task_duration << endl; 
	cout << normal << " Total reward: " << blue << Global_total_disc_reward << normal << "Successful task: " << blue << DONE_successful_tasks << " = " << DONE_percentage_successful_tasks << "% " << endl; 
	cout << normal << " Warnings received: " << blue << DONE_warnings_count << endl << endl; 

	//// could maybe be used 
	//int DONE_failed_subtasks                = 0; 
	//int DONE_successful_subtasks			= 0; 
	//double DONE_task_duration                  = 0.0;
	//double DONE_percentage_successful_subtasks = 0.0; 
	//int DONE_warnings_count					= 0; 
	//int DONE_failed_tasks					= 0; 
	//double DONE_percentage_successful_tasks	= 0.0; 
	//string Global_total_disc_reward 			= ""; 

}


int main(int argc, char **argv) {

	ros::init(argc, argv, "experiment_monitor_Node");
	ros::NodeHandle nh;
	ros::CallbackQueue my_queue; 
	// declare object here is class is use 
	//RobotMotionAgent robot_motion_agent;
	clear_screen();
	//cout << "                    ############# Task Rules #############                   " << endl;
		
	
	ros::AsyncSpinner spinner(3 /*number of threads*/, &my_queue /* spinner exclusively for my_queue */); 
	
	// bind the queue to the node handle
	nh.setCallbackQueue( &my_queue );

 	// rosparameters 

	taskStatus_subs = nh.subscribe("/task_manager/task_status", 1 , receive_task_status);

	/*"SENSOR" when tray_update received 
	"MANAGER-TASK-DONE" after a task is done 
	"OBSERVATION"  observations by observation agent right before sent to the DESPOT for decision making 
	"ROBOT"  robot beliefs
*/


	// Services 
	//display_task_rules = nh.advertiseService("/rule_monitor/display_task_rule", display_task_rules_server);
	//display_scoring_server = nh.advertiseService("/rule_monitor/display_scoring", display_scoring);
	//distract_participants_server = nh.advertiseService("/rule_monitor/distract_participants",distract_participants);
	

	// starting spinners with multiple threads 
	spinner.start();
	ros::waitForShutdown();
}




