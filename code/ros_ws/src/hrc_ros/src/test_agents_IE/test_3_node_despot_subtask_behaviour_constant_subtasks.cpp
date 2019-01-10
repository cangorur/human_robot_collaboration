/*
 *  TaskManagerNode.cpp
 *
 *  Created on: 02.09.2017
 *  Modified on: 19.04.2018
 *  	Author: Orhan Can Görür
 */

#include <ros/ros.h>
#include <test_agents_IE/TestAgent.h>
#include <iostream> 
#include <signal.h>

using namespace std; 

int main(int argc, char **argv) {

	ros::init(argc, argv, "test_2_despot_subtask_behaviour");
	ros::NodeHandle nh;

	
	ROS_INFO("Test2 is setup and ready!\n\n");

	TestAgent test_agent;

	//signal(SIGINT, test_agent.mySigintHandler);  // manual sigint handler for correct shutdown of node

	ros::Duration(1.0).sleep(); 

	int test_object =  0; 
	int test_tray   =  0; 
	int correct_tray = 0;
	int current_task_no = 0; 
	int current_subtask_no = 0;
	// observations 
	string obs_info = "Observation 35 | Despot observation 9";  
	string action = "idle";
	int human_detected = 1;  
	int human_looking_around = 0;
	string test_outcome = "failure";


		
			// #######  Test step 1 ###########
			// task 1  | subtask 1
			cout << endl << endl << "########## Test step 1 ########"<< endl; 
			test_object = 2;  // test_object that is published by the observationagent 	
			test_tray   = 2;  // test_tray is published as the tray the observationagent has observed 
			correct_tray = 2; // this is the correct tray according to the IE_task_config.json file 


			// ####  Observations 
			obs_info = "Observation 35 | Despot observation 9"; 
			action = "idle";
			human_detected = 1;  
			human_looking_around = 0;

			// ~~~~~~~~~~~~~~~~~~~  Issue first observations ~~~~~~~~~~~~~~~~~~~~~~~~~~
			test_agent.issue_action_update(action,human_detected,human_looking_around);

			cout << endl << endl << " Issued action update " << endl; 
			cout << " ~~ " << obs_info << " ~~ " << endl; 
			cout << "		# action = " << action << endl; 
			cout << "		# human_detected = " << human_detected << endl;  
			cout << "		# human_looking_around = " << human_looking_around << endl;    
			
			
			ros::spinOnce();
			cout << endl << " Press any key to continue " << endl;
			cin.get();

		
		// #### expected results:  
		test_outcome = "success";
		current_task_no = 1; 
		current_subtask_no = 1; 

		test_agent.issue_tray_update(test_object,test_tray);
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
		


			// issue second observation -> the same again -> this will trigger robot agent to inform about the current robot state 
			test_agent.issue_action_update(action,human_detected,human_looking_around);

			cout << endl << endl << " Issued action update " << endl; 
			cout << " ~~ " << obs_info << " ~~ " << endl; 
			cout << "		# action = " << action << endl; 
			cout << "		# human_detected = " << human_detected << endl;  
			cout << "		# human_looking_around = " << human_looking_around << endl;    
			
			
			for ( int i = 0; i <= 30; i++){
				ros::spinOnce(); 
				ros::Duration(0.1).sleep(); 
				ros::spinOnce();
			}

			test_agent.success_received = false; 
			ros::spinOnce();

			if ( test_agent.Despot_global_task_status.compare("ongoing") == 0 ) {
				
			}

			cout << " TaskManager_status (received from Despot): " << test_agent.Despot_global_task_status << endl;
			cout << "1st subtask success sent -> check the status of despot " << endl << endl; 
			cout << "Press any key to continue"  << endl;  
			cin.get(); 
	



		// #######  Test step 2 ###########
		// task 1  | subtask 2  
		cout << endl << endl << "########## Test step 2 ########"<< endl; 
		test_object = 1;
		test_tray   = 1;
		correct_tray = 1;

		// #### Observations 
		obs_info = "Observation 35 | Despot observation 9"; 
		action = "idle";
		human_detected = 1;  
		human_looking_around = 0;

		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;     
		
		cout << endl << " Press any key to continue " << endl; 
		cin.get();



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


		// issue second observation -> the same again -> this will trigger robot agent to inform about the current robot state 
		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;    
		
	


		for ( int i = 0; i <= 30; i++){
			ros::spinOnce(); 
			ros::Duration(0.1).sleep(); 
			ros::spinOnce();
		}

		test_agent.success_received = false; 
		ros::spinOnce(); 

		cout << " TaskManager_status (received from Despot): " << test_agent.Despot_global_task_status << endl;
		cout << endl << endl << " 2nd subtask success issued -> please check the state of the Despot " << endl << endl; 
		cout << "Press any key to continue"  << endl;  
		cin.get(); 



		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// #######  Test step 3 ###########
		// task 1  | subtask 3  
		cout << endl << endl << "########## Test step 3 ########"<< endl; 
		test_object  = 3;
		test_tray    = 3;
		correct_tray = 3;

		// #### Observations 
		obs_info = "Observation 35 | Despot observation 9"; 
		action = "idle";
		human_detected = 1;  
		human_looking_around = 0;


		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;      
		
		cout << endl << " Press any key to continue " << endl; 
		cin.get();



		// expected results: 
		test_outcome = "success";
		current_task_no = 1; 
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
		
		// assert if task counters are in sync | test_agent.task_cnt_received are issued by the observation agent -> they should be as expected 
		if (test_agent.task_cnt_received == current_task_no && test_agent.subtask_cnt_received == current_subtask_no){
			cout << " Task and subtask counters correct     _ IN SYNC" << endl; 
		} else {
			cout << " Task and subtask out of sync 			_ OUT OF SYNC ERROR" << endl;
			cout << " reveived: task_cnt = " << test_agent.task_cnt_received << "  subtask_cnt = " << test_agent.subtask_cnt_received << endl; 
			cout << "should be: task_cnt = " << current_task_no << "  subtask_cnt = " << current_subtask_no << endl;  
		}

		obs_info = "Observation 35 | Despot observation 9"; 
		action = "idle";
		human_detected = 1;  
		human_looking_around = 0;
		// issue second observation -> the same again -> this will trigger robot agent to inform about the current robot state 
		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;    
		
		cout << endl << " Press any key to continue " << endl; 
		cin.get();

		for ( int i = 0; i <= 30; i++){
			ros::spinOnce(); 
			ros::Duration(0.1).sleep(); 
			ros::spinOnce();
		}

		test_agent.success_received = false; 
		ros::spinOnce(); 

		cout << " TaskManager_status (received from Despot): " << test_agent.Despot_global_task_status << endl;
		cout << endl << endl << " 3rd subtask success issued -> please check the state of the Despot " << endl << endl; 
		cout << "Press any key to continue"  << endl;  
		cin.get(); 


		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// #######  Test step 4 ###########
		// task 1  | subtask 4  
		cout << endl << endl << "########## Test step 4 ########"<< endl; 
		test_object =  2 ;
		test_tray   =  3 ; 
		correct_tray = 2 ;

		// #### Observations 
		obs_info = "Observation 35 | Despot observation 9"; 
		action = "idle";
		human_detected = 1;  
		human_looking_around = 0;

		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;   
		
		cout << endl << " Press any key to continue " << endl; 
		cin.get();


		// expected results: 
		test_outcome = "fail";
		current_task_no = 1; 
		current_subtask_no = 4;

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

		// issue second observation -> the same again -> this will trigger robot agent to inform about the current robot state 
		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;    
		
		cout << endl << " Press any key to continue " << endl; 
		cin.get();

		for ( int i = 0; i <= 30; i++){
			ros::spinOnce(); 
			ros::Duration(0.1).sleep(); 
			ros::spinOnce();
		}

		test_agent.success_received = false; 
		ros::spinOnce(); 

		cout << " TaskManager_status (received from Despot): " << test_agent.Despot_global_task_status << endl;
		cout << endl << endl << " 4th subtask success issued -> please check the state of the Despot -> should stop and be in global success " << endl << endl; 
		cout << "Press any key to continue"  << endl;  
		cin.get(); 


		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// #######  Test step 5 ###########
		// task 2  | subtask 1  
		cout << endl << endl << "########## Test step 5 ########"<< endl; 
		test_object   = 3;
		test_tray     = 1;
		correct_tray  = 1;

		// #### Observations 
		obs_info = "Observation 35 | Despot observation 9"; 
		action = "idle";
		human_detected = 1;  
		human_looking_around = 0;

		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;   
		
		cout << endl << " Press any key to continue " << endl; 
		cin.get();


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
		
		// assert if task counters are in sync | test_agent.task_cnt_received are issued by the observation agent -> they should be as expected 
		if (test_agent.task_cnt_received == current_task_no && test_agent.subtask_cnt_received == current_subtask_no){
			cout << " Task and subtask counters correct     _ IN SYNC" << endl; 
		} else {
			cout << " Task and subtask out of sync 			_ OUT OF SYNC ERROR" << endl;
			cout << " reveived: task_cnt = " << test_agent.task_cnt_received << "  subtask_cnt = " << test_agent.subtask_cnt_received << endl; 
			cout << "should be: task_cnt = " << current_task_no << "  subtask_cnt = " << current_subtask_no << endl;  
		}

		// issue second observation -> the same again -> this will trigger robot agent to inform about the current robot state 
		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;    
		
		cout << endl << " Press any key to continue " << endl; 
		cin.get();

		for ( int i = 0; i <= 30; i++){
			ros::spinOnce(); 
			ros::Duration(0.1).sleep(); 
			ros::spinOnce();
		}

		test_agent.success_received = false; 
		ros::spinOnce(); 

		cout << " TaskManager_status (received from Despot): " << test_agent.Despot_global_task_status << endl;
		cout << endl << endl << " 5th subtask success issued -> please check the state of the Despot  " << endl << endl; 
		cout << "Press any key to continue"  << endl;  
		cin.get(); 


		cout << " The end for positive test " << endl; 

		cin.get(); 



		// #######  Test step 6 ###########
		// task 2  | subtask 2
		cout << endl << endl << "########## Test step 6 ########"<< endl; 
		test_object =  2;  // test_object that is published by the observationagent 	
		test_tray   =  3;  // test_tray is published as the tray the observationagent has observed 
		correct_tray = 2; // this is the correct tray according to the IE_task_config.json file 


		// #### Observations 
		obs_info = "Observation 7 | Despot observation 4";
		action = "grasping";
		human_detected = 1;  
		human_looking_around = 0;

		// ~~~~~~~~~~~~~~~~~~~  Issue first observations ~~~~~~~~~~~~~~~~~~~~~~~~~~
		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;    
		
		 
		ros::spinOnce();
		cout << endl << " Press any key to continue " << endl;
		cin.get();

		
		// #### expected results:  
		test_outcome = "fail";
		current_task_no = 2; 
		current_subtask_no = 1; 

		test_agent.issue_tray_update(test_object,test_tray);
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
		


		// issue second observation -> the same again -> this will trigger robot agent to inform about the current robot state 
		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;    
		
		
		for ( int i = 0; i <= 30; i++){
			ros::spinOnce(); 
			ros::Duration(0.1).sleep(); 
			ros::spinOnce();
		}

		test_agent.success_received = false; 
		ros::spinOnce();

		if ( test_agent.Despot_global_task_status.compare("ongoing") == 0 ) {
			 
		}

		cout << " TaskManager_status (received from Despot): " << test_agent.Despot_global_task_status << endl;
		cout << "1st subtask fail sent -> check the status of despot " << endl << endl; 
		cout << "Press any key to continue"  << endl;  
		cin.get(); 



		// #######  Test step 7 ###########
		// task 2  | subtask 3  
		cout << endl << endl << "########## Test step 7 ########"<< endl; 
		test_object  = 1;
		test_tray    = 1;
		correct_tray = 3;

		// #### Observations 
		obs_info = "Observation 7 | Despot observation 4"; 
		action = "grasping";
		human_detected = 1;  
		human_looking_around = 0;

		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;     
		
		cout << endl << " Press any key to continue " << endl; 
		cin.get();



		// expected results: 
		test_outcome = "fail";
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
		
		// assert if task counters are in sync | test_agent.task_cnt_received are issued by the observation agent -> they should be as expected 
		if (test_agent.task_cnt_received == current_task_no && test_agent.subtask_cnt_received == current_subtask_no){
			cout << " Task and subtask counters correct     _ IN SYNC" << endl; 
		} else {
			cout << " Task and subtask out of sync 			_ OUT OF SYNC ERROR" << endl;
			cout << " reveived: task_cnt = " << test_agent.task_cnt_received << "  subtask_cnt = " << test_agent.subtask_cnt_received << endl; 
			cout << "should be: task_cnt = " << current_task_no << "  subtask_cnt = " << current_subtask_no << endl;  
		}


		// issue second observation -> the same again -> this will trigger robot agent to inform about the current robot state 
		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;    
		
	


		for ( int i = 0; i <= 30; i++){
			ros::spinOnce(); 
			ros::Duration(0.1).sleep(); 
			ros::spinOnce();
		}

		test_agent.success_received = false; 
		ros::spinOnce(); 

		cout << " TaskManager_status (received from Despot): " << test_agent.Despot_global_task_status << endl;
		cout << endl << endl << " 2nd subtask fail issued -> please check the state of the Despot " << endl << endl; 
		cout << "Press any key to continue"  << endl;  
		cin.get(); 



		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// #######  Test step 8 ###########
		// task 2  | subtask 4  
		cout << endl << endl << "########## Test step 8 ########"<< endl; 
		test_object  = 2;
		test_tray    = 1;
		correct_tray = 2;

		// #### Observations 
		obs_info = "Observation 7 | Despot observation 4"; 
		action = "grasping";
		human_detected = 1;  
		human_looking_around = 0;

		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;      
		
		cout << endl << " Press any key to continue " << endl; 
		cin.get();



		// expected results: 
		test_outcome = "fail";
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
		
		// assert if task counters are in sync | test_agent.task_cnt_received are issued by the observation agent -> they should be as expected 
		if (test_agent.task_cnt_received == current_task_no && test_agent.subtask_cnt_received == current_subtask_no){
			cout << " Task and subtask counters correct     _ IN SYNC" << endl; 
		} else {
			cout << " Task and subtask out of sync 			_ OUT OF SYNC ERROR" << endl;
			cout << " reveived: task_cnt = " << test_agent.task_cnt_received << "  subtask_cnt = " << test_agent.subtask_cnt_received << endl; 
			cout << "should be: task_cnt = " << current_task_no << "  subtask_cnt = " << current_subtask_no << endl;  
		}

		// issue second observation -> the same again -> this will trigger robot agent to inform about the current robot state 
		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;    
		
		cout << endl << " Press any key to continue " << endl; 
		cin.get();

		for ( int i = 0; i <= 30; i++){
			ros::spinOnce(); 
			ros::Duration(0.1).sleep(); 
			ros::spinOnce();
		}

		test_agent.success_received = false; 
		ros::spinOnce(); 

		cout << " TaskManager_status (received from Despot): " << test_agent.Despot_global_task_status << endl;
		cout << endl << endl << " 3rd subtask fail issued -> please check the state of the Despot -> should stop and be in global fail " << endl << endl; 
		cout << "Press any key to continue"  << endl;  
		cin.get(); 


		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// #######  Test step 9 ###########
		// task 2  | subtask 4  
		cout << endl << endl << "########## Test step 9 ########"<< endl; 
		test_object = 2;
		test_tray   = 1000;
		correct_tray = 3;

		// #### Observations 
		obs_info = "Observation 7 | Despot observation 4"; 
		action = "grasping";
		human_detected = 1;  
		human_looking_around = 0;

		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;   
		
		cout << endl << " Press any key to continue " << endl; 
		cin.get();


		// expected results: 
		test_outcome = "fail";
		current_task_no = 2; 
		current_subtask_no = 4;

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

		// issue second observation -> the same again -> this will trigger robot agent to inform about the current robot state 
		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;    
		
		cout << endl << " Press any key to continue " << endl; 
		cin.get();

		for ( int i = 0; i <= 30; i++){
			ros::spinOnce(); 
			ros::Duration(0.1).sleep(); 
			ros::spinOnce();
		}

		test_agent.success_received = false; 
		ros::spinOnce(); 

		cout << " TaskManager_status (received from Despot): " << test_agent.Despot_global_task_status << endl;
		cout << endl << endl << " 4th subtask fail issued -> please check the state of the Despot " << endl << endl; 
		cout << "Press any key to continue"  << endl;  
		cin.get(); 


		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// #######  Test step 10 ###########
		// task 2  | subtask 5  
		cout << endl << endl << "########## Test step 10 ########"<< endl; 
		obs_info = "Observation 7 | Despot observation 4"; 
		test_object = 2;
		test_tray   = 1000;
		correct_tray = 3;

		// #### Observations 
		obs_info = "Observation 6 | Despot observation 3"; 
		action = "grasping";
		human_detected = 1;  
		human_looking_around = 0;

		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;   
		
		cout << endl << " Press any key to continue " << endl; 
		cin.get();


		// expected results: 
		test_outcome = "fail";
		current_task_no = 2; 
		current_subtask_no = 5;

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

		// issue second observation -> the same again -> this will trigger robot agent to inform about the current robot state 
		test_agent.issue_action_update(action,human_detected,human_looking_around);

		cout << endl << endl << " Issued action update " << endl; 
		cout << " ~~ " << obs_info << " ~~ " << endl; 
		cout << "		# action = " << action << endl; 
		cout << "		# human_detected = " << human_detected << endl;  
		cout << "		# human_looking_around = " << human_looking_around << endl;    
		
		cout << endl << " Press any key to continue " << endl; 
		cin.get();

		for ( int i = 0; i <= 30; i++){
			ros::spinOnce(); 
			ros::Duration(0.1).sleep(); 
			ros::spinOnce();
		}

		test_agent.success_received = false; 
		ros::spinOnce(); 

		cout << " TaskManager_status (received from Despot): " << test_agent.Despot_global_task_status << endl;
		cout << endl << endl << " 5th subtask fail issued -> please check the state of the Despot -> should stop and be in global fail " << endl << endl; 
		cout << "Press any key to continue"  << endl;  
		cin.get(); 


		cout << " The end for fail test " << endl; 

		cin.get(); 



	
	
ros::spin();
}
