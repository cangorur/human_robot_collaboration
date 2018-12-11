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

	ros::init(argc, argv, "test_1_synchro_task_observation");
	ros::NodeHandle nh;

	ROS_INFO("Test1 is setup and ready!\n\n");

	TestAgent test_agent;

	ros::Duration(1.0).sleep(); 

	int test_object =  0; 
	int test_tray   =  0; 
	int correct_tray = 0;
	int current_task_no = 0; 
	int current_subtask_no = 0; 
	string test_outcome = "failure";



		// #######  Test step 1 ###########
		// task 1  | subtask 1
		cout << endl << endl << "########## Test step 1 ########"<< endl; 
		test_object = 2;  // test_object that is published by the observationagent 	
		test_tray   = 3;  // test_tray is published as the tray the observationagent has observed 
		correct_tray = 3; // this is the correct tray according to the IE_task_config.json file 

		// #### expected results:  
		test_outcome = "success";
		current_task_no = 1; 
		current_subtask_no = 1; 

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

		// assert if test success status is as expected 
		cout << endl << endl << "   ## Assert Test ##" << endl; 
		if ( (test_agent.subtask_success_status).compare(test_outcome) == 0){
			cout << " Test successfull     _ SUCCESS" << endl;
		} else {
			cout << " Test Failed		   _ FAILURE" << endl; 
		}
		// assert if task counters are in sync 
		if (test_agent.task_cnt_received == current_task_no && test_agent.subtask_cnt_received == current_subtask_no){
			cout << " Task and subtask counters correct     _ IN SYNC" << endl; 
		} else {
			cout << " Task and subtask out of sync 			_ OUT OF SYNC ERROR" << endl;
			cout << " reveived: task_cnt = " << test_agent.task_cnt_received << "  subtask_cnt = " << test_agent.subtask_cnt_received << endl; 
			cout << "should be: task_cnt = " << current_task_no << "  subtask_cnt = " << current_subtask_no << endl;  
		}

		test_agent.success_received = false; 



		// #######  Test step 2 ###########
		// task 1  | subtask 2  
		cout << endl << endl << "########## Test step 2 ########"<< endl; 
		test_object = 1;
		test_tray   = 4;
		correct_tray = 4;

		// expected results: 
		test_outcome = "success";
		current_task_no = 1; 
		current_subtask_no = 2;

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
		// assert if test success status is as expected 
		if ( (test_agent.subtask_success_status).compare(test_outcome) == 0){
			cout << " Test successfull     _ SUCCESS" << endl;
		} else {
			cout << " Test Failed		   _ FAILURE" << endl; 
		}
		
		// assert if task counters are in sync | test_agent.task_cnt_received are issued by the observation agent -> they should be as expected 
		if (test_agent.task_cnt_received == current_task_no && test_agent.subtask_cnt_received == current_subtask_no){
			cout << " Task and subtask counters correct     _ IN SYNC" << endl; 
		} else {
			cout << " Task and subtask out of sync 			_ OUT OF SYNC ERROR" << endl;
			cout << " reveived: task_cnt = " << test_agent.task_cnt_received << "  subtask_cnt = " << test_agent.subtask_cnt_received << endl; 
			cout << "should be: task_cnt = " << current_task_no << "  subtask_cnt = " << current_subtask_no << endl;  
		}

		test_agent.success_received = false; 


		ros::Duration(6.0).sleep(); 


		// #######  Test step 3 ###########
		//task 2  | subtask 1 
		cout << endl << endl << "########## Test step 3 ########"<< endl; 
		test_object = 1;
		test_tray   = 10;
		correct_tray = 10; 
		

		// expected results:
		test_outcome = "success";
		current_task_no = 2; 
		current_subtask_no = 1; 
		
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

		// assert if test success status is as expected
		if ( (test_agent.subtask_success_status).compare(test_outcome) == 0){
			cout << " Test successfull     _ SUCCESS" << endl;
		} else {
			cout << " Test Failed		   _ FAILURE" << endl; 
		}

		// assert if task counters are in sync 
		if (test_agent.task_cnt_received == current_task_no && test_agent.subtask_cnt_received == current_subtask_no){
			cout << " Task and subtask counters correct     _ IN SYNC" << endl; 
		} else {
			cout << " Task and subtask out of sync 			_ OUT OF SYNC ERROR" << endl;
			cout << " reveived: task_cnt = " << test_agent.task_cnt_received << "  subtask_cnt = " << test_agent.subtask_cnt_received << endl; 
			cout << "should be: task_cnt = " << current_task_no << "  subtask_cnt = " << current_subtask_no << endl;  
		}

		test_agent.success_received = false; 



		// #######  Test step 4 ###########
		//task 2  | subtask 2 
		cout << endl << endl << "########## Test step 4 ########"<< endl; 
		test_object = 3;
		test_tray   = 15;
		correct_tray = 15; 
		

		// expected results:
		test_outcome = "success";
		current_task_no = 2; 
		current_subtask_no = 2; 
		
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

		// assert if test success status is as expected
		if ( (test_agent.subtask_success_status).compare(test_outcome) == 0){
			cout << " Test successfull     _ SUCCESS" << endl;
		} else {
			cout << " Test Failed		   _ FAILURE" << endl; 
		}

		// assert if task counters are in sync 
		if (test_agent.task_cnt_received == current_task_no && test_agent.subtask_cnt_received == current_subtask_no){
			cout << " Task and subtask counters correct     _ IN SYNC" << endl; 
		} else {
			cout << " Task and subtask out of sync 			_ OUT OF SYNC ERROR" << endl;
			cout << " reveived: task_cnt = " << test_agent.task_cnt_received << "  subtask_cnt = " << test_agent.subtask_cnt_received << endl; 
			cout << "should be: task_cnt = " << current_task_no << "  subtask_cnt = " << current_subtask_no << endl;  
		}

		test_agent.success_received = false; 


		// #######  Test step 5 ###########
		//task 2  | subtask 3 
		cout << endl << endl << "########## Test step 5 ########"<< endl; 
		test_object = 2;
		test_tray   = 20;
		correct_tray = 20; 
		

		// expected results:
		test_outcome = "success";
		current_task_no = 2; 
		current_subtask_no = 3; 
		
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

		// assert if test success status is as expected
		if ( (test_agent.subtask_success_status).compare(test_outcome) == 0){
			cout << " Test successfull     _ SUCCESS" << endl;
		} else {
			cout << " Test Failed		   _ FAILURE" << endl; 
		}

		// assert if task counters are in sync 
		if (test_agent.task_cnt_received == current_task_no && test_agent.subtask_cnt_received == current_subtask_no){
			cout << " Task and subtask counters correct     _ IN SYNC" << endl; 
		} else {
			cout << " Task and subtask out of sync 			_ OUT OF SYNC ERROR" << endl;
			cout << " reveived: task_cnt = " << test_agent.task_cnt_received << "  subtask_cnt = " << test_agent.subtask_cnt_received << endl; 
			cout << "should be: task_cnt = " << current_task_no << "  subtask_cnt = " << current_subtask_no << endl;  
		}

		test_agent.success_received = false; 


		ros::Duration(6.0).sleep(); 


		// #######  Test step 6 ###########
		//task 3  | subtask 1 
		cout << endl << endl << "########## Test step 6 ########"<< endl; 
		test_object = 1;
		test_tray   = 11;
		correct_tray = 11; 
		

		// expected results:
		test_outcome = "success";
		current_task_no = 3; 
		current_subtask_no = 1; 
		
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

		// assert if test success status is as expected
		if ( (test_agent.subtask_success_status).compare(test_outcome) == 0){
			cout << " Test successfull     _ SUCCESS" << endl;
		} else {
			cout << " Test Failed		   _ FAILURE" << endl; 
		}

		// assert if task counters are in sync 
		if (test_agent.task_cnt_received == current_task_no && test_agent.subtask_cnt_received == current_subtask_no){
			cout << " Task and subtask counters correct     _ IN SYNC" << endl; 
		} else {
			cout << " Task and subtask out of sync 			_ OUT OF SYNC ERROR" << endl;
			cout << " reveived: task_cnt = " << test_agent.task_cnt_received << "  subtask_cnt = " << test_agent.subtask_cnt_received << endl; 
			cout << "should be: task_cnt = " << current_task_no << "  subtask_cnt = " << current_subtask_no << endl;  
		}

		test_agent.success_received = false; 


	
ros::spin();
}
