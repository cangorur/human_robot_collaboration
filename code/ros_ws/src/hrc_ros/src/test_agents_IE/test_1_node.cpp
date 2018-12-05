/*
 *  TaskManagerNode.cpp
 *
 *  Created on: 02.09.2017
 *  Modified on: 19.04.2018
 *  	Author: Orhan Can Görür
 */

#include <ros/ros.h>
#include <test_agents_IE/TestAgent.h>

using namespace std; 

int main(int argc, char **argv) {

	ros::init(argc, argv, "test_2_synchronisation_task_observation");
	ros::NodeHandle nh;

	ROS_INFO("Test2 is setup and ready!\n\n");

	TestAgent test_agent;

	ros::Duration(1.0).sleep(); 

	int test_object =  0; 
	int test_tray   =  0; 
	int correct_tray = 0;
	string test_outcome = "failure";



		// #######  Test step 1 ###########
		// task 1  | subtask 1 
		cout << endl << endl << "########## Test step 1 ########"<< endl; 
		test_object = 2;
		test_tray   = 3;
		correct_tray = 3; 
		test_outcome = "success";
		test_agent.issue_tray_update(test_object,test_tray);
		ros::spinOnce();
		//ros::Duration(1.0).sleep();
		ros::spinOnce();
		
		for (int i = 0; i<= 10; i ++) {
			if (test_agent.success_received == false) {
			cout << " success_received still " << test_agent.success_received;
			ros::spinOnce(); 
			ros::Duration(0.1).sleep(); 
			} else { break; }
			
		}

		cout << endl << endl << "   ## Assert Test ##" << endl; 
		if ( (test_agent.subtask_success_status).compare(test_outcome) == 0){
			cout << " Test successfull     _ SUCCESS" << endl;
		} else {
			cout << " Test Failed		   _ FAILURE" << endl; 
		}

		test_agent.success_received = false; 



		// #######  Test step 2 ###########
		// task 1  | subtask 2 
		cout << endl << endl << "########## Test step 2 ########"<< endl; 
		test_object = 1;
		test_tray   = 4;
		correct_tray = 4; 
		test_outcome = "success";
		test_agent.issue_tray_update(test_object,test_tray);
		ros::spinOnce();
		//ros::Duration(1.0).sleep();
		ros::spinOnce();
		
		for (int i = 0; i<= 10; i ++) {
			if (test_agent.success_received == false) {
			cout << " success_received still " << test_agent.success_received;
			ros::spinOnce(); 
			ros::Duration(0.1).sleep(); 
			} else { break; }
			
		}

		cout << endl << endl << "   ## Assert Test ##" << endl; 
		if ( (test_agent.subtask_success_status).compare(test_outcome) == 0){
			cout << " Test successfull     _ SUCCESS" << endl;
		} else {
			cout << " Test Failed		   _ FAILURE" << endl; 
		}

		test_agent.success_received = false; 


	
ros::spin();
}
