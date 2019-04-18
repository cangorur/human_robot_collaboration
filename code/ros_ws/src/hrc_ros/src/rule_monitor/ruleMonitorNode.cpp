/*
 * ruleMonitorNode.cpp
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

// generic service includes 
#include <std_srvs/Trigger.h>

// includes for Json parsing
#include <helper_functions/Json_parser.h>



using namespace std;


// All of this could go into a header 
	
ros::ServiceServer display_task_rules; 

// define colours for coloured prints 
const std::string red("\033[1;31m");
const std::string green("\033[1;32m");
const std::string blue("\033[1;34m");
const std::string normal("\033[1;37m");	

// variables and structs to parse Json 
task_set current_task_set;
success_combo success_criteria_read;



//#################### function declarations ##################### 
std::string mapintToString_colour(int int_value, std::string & print_string);
bool display_task_rules_server(hrc_ros::DisplayTaskRuleRequest &req,hrc_ros::DisplayTaskRuleResponse &res);
void clear_screen(void);

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

bool display_task_rules_server(hrc_ros::DisplayTaskRuleRequest &req,hrc_ros::DisplayTaskRuleResponse &res){

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
			
			cout << "			Place " << obj_colour << obj_print_string << normal << " object into " << container_colour << container_print_string << normal << " container " << endl << endl; 
		} 

		// wait for certain time, then clear the screen and return true to start the experiment
		ros::Duration(15).sleep();
		clear_screen();  
	} else { // different subtask rules 
		  cout << endl << endl << endl << normal << "Task #: " << task_counter << "		Subtasks: " << current_task_set.subtask_quantity << endl <<"------------------------------------------------------------------" << endl << endl << endl << endl; 
		  cout << " different subtasks not supported yet -> available soon :-) " << endl; 
		  // wait for certain time, then clear the screen and return true to start the experiment
		  ros::Duration(5).sleep();
		  clear_screen();
	}




   res.success = true;
   return true;
}

	void clear_screen() {
    // CSI[2J clears screen, CSI[H moves the cursor to top-left corner
    std::cout << "\x1B[2J\x1B[H" << std::flush;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "rule_monitor_Node");
	ros::NodeHandle nh;
	ros::CallbackQueue my_queue; 
	// declare object here is class is use 
	//RobotMotionAgent robot_motion_agent;
	clear_screen();
	cout << "                    ############# Task Rules #############                   " << endl;
		
	
	ros::AsyncSpinner spinner(3 /*number of threads*/, &my_queue /* spinner exclusively for my_queue */); 
	
	// bind the queue to the node handle
	nh.setCallbackQueue( &my_queue );

 	// rosparameters 



	// Services 
	display_task_rules = nh.advertiseService("/rule_monitor/display_task_rule", display_task_rules_server);
	

	// starting spinners with multiple threads 
	spinner.start();
	ros::waitForShutdown();
}




