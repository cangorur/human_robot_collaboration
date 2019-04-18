/*
 * ruleMonitorNode.cpp
 *
 *  Created on: 18.04.2019
 *      Author: Elia Kargruber
 */

#include <ros/ros.h>
#include<stdlib.h>


#include<ros/callback_queue.h>
#include<std_msgs/Bool.h>
#include <hrc_ros/DisplayTaskRule.h> 

// generic service includes 
#include <std_srvs/Trigger.h>



using namespace std;


// All of this could go into a header 
	
ros::ServiceServer display_task_rules; 

// define colours for coloured prints 
const std::string red("\033[1;31m");
const std::string green("\033[1;32m");
const std::string blue("\033[1;34m");
const std::string normal("\033[1;37m");	

//#################### function declarations ##################### 
bool display_task_rules_server(hrc_ros::DisplayTaskRuleRequest &req,hrc_ros::DisplayTaskRuleResponse &res);
void clear_screen(void);

// ########## Function implementations ##########################
bool display_task_rules_server(hrc_ros::DisplayTaskRuleRequest &req,hrc_ros::DisplayTaskRuleResponse &res){

	clear_screen(); 
	cout << normal << "This is a task_rule " << endl << endl; 
	cout << red   << " red text" << endl; 
	cout << green << "green text" << endl; 
	cout << blue  << "blue text" << endl; 
	cout << blue << "blue tesxt" << normal << "normal text" << red << " red text" << green << " green text " << endl; 


	//### parse task_rule file 



	// wait for certain time, then clear the screen and return true to start the experiment
	ros::Duration(5).sleep();
	clear_screen();  



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




