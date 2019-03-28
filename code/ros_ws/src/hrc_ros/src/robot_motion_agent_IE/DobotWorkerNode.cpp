/*
 * DobotWorkerNode.cpp
 *
 *  Created on: 20.02.2019
 *      Author: Elia Kargruber
 */

#include <ros/ros.h>
//#include <robot_motion_agent_IE/RobotMotionAgent_IE.h>
#include<stdlib.h>


#include<ros/callback_queue.h>
#include<std_msgs/Bool.h>
#include<hrc_ros/RequestSuccessCriteria.h> 
#include <hrc_ros/ObjectGraspColourMsg.h> 

// generic service includes 
#include <std_srvs/Trigger.h>

// dobot specific services 
#include <hrc_ros/ContPickAndPlace.h>
#include <hrc_ros/InitDobotArmApp.h>
#include <hrc_ros/SetQueuedCmdStopExec.h>
#include <hrc_ros/SetQueuedCmdForceStopExec.h>
#include <hrc_ros/OneTimePickAndPlace.h>
#include <hrc_ros/SetPTPCoordinateParams.h>  //service to set speed and velocities of dobot 
#include <hrc_ros/SetQueuedCmdStartExec.h>
#include <hrc_ros/SetPTPCmd.h> // for the gotoPointApp service 
#include <hrc_ros/SetEndEffectorSuctionCup.h>
#include <hrc_ros/SetQueuedCmdClear.h>
#include <hrc_ros/SimplePickAndPlace.h>
#include <hrc_ros/InOprConveyorControl.h> 
#include <hrc_ros/GetQueuedCmdCurrentIndex.h> 
#include <hrc_ros/PublishActionRecognisedMsg.h>

using namespace std;


// All of this could go into a header 
				
ros::ServiceClient Dobot_SimplePickAndPlace; 				 
ros::ServiceClient Dobot_SetPTPCoordinateParams;
ros::ServiceClient Dobot_InitDobotArmApp;
ros::ServiceClient Dobot_SetQueuedCmdForceStopExec;
ros::ServiceClient Dobot_SetQueuedCmdStopExec;
ros::ServiceClient Dobot_SetQueuedCmdStartExec;
ros::ServiceClient Dobot_SetQueuedCmdClear;
ros::ServiceClient Dobot_SetEndEffectorSuctionCup;
ros::ServiceClient Dobot_gotoPoint;
ros::ServiceClient Dobot_ContPickAndPlace;
ros::ServiceClient Dobot_oneTimePickAndPlace;
ros::ServiceClient request_success_criteria;
ros::ServiceClient enableConveyor; 
ros::ServiceClient Dobot_getQueueIndex;

ros::ServiceServer calibrate_scenario;
ros::ServiceServer reset_scenario; 

// ############ Dobot API tipps ################
// The goto point uses a PTP movement schema there are the following ptpModes available 
// 0 = Door shape or Jump mode  - relatively slow 
// 1 = moveJ -> move all joints independently -> this is the fastest movement type 
// 2 = moveL -> move on a straigth line -> this is quite slow 

// Variables that switch between fullDobot setup and a wait only version 
bool no_Dobot_flag = false; // used for debugging without dobot (true= only wait | false= call dobot service ) || is set by ros_param 
int wait_time = 20; // time dobot should wait in noDobot mode

// switch between different expression -> can be set dynamically by rosparameter
int dobot_expression_version = 2;   // values can be: 1= version 1 | 2= version 2 

// ########### flags for dobot action status communication between callbacks ###############
bool warning_received_flag = false; // flag that indicates that a warning has been received, this is relevant for the grasping action, it is set to false at the beginning of the grasp and checked wether it is set true during the grasp
bool grasp_is_planned_flag = false; // set true after a grasp has been planned  and set false in grasCallback after successful grasp
bool planning_in_progress  = false; // set true during a planning of a path and set false when planning is done -> subsequent calls of planCallback will skip planning while another planning is in progress 


// Struct to hold the package drop place 
struct package_drop_loc {
  double x;
  double y;
  double z; 
} ;

package_drop_loc red_package_drop_loc; 
package_drop_loc green_package_drop_loc;
package_drop_loc blue_package_drop_loc;
int red_placed_cnt = 0; 
int green_placed_cnt = 0; 
int blue_placed_cnt = 0; 
int object_to_grasp_colour = 4;// 1=red|2=green|3=blue|4= reference => no object visible on conveyor belt

// ########## locations #######################
// idle positon
float x_idle = 197.13; //195.06; // above the IR sensor was x=215, y=45, z=30; 
float y_idle =  -0.82; 		   //-4.15; 
float z_idle =  35.22;

// graspplanning position -> robot will move towards the object but stop above it 
float x_planning_v1 = 282;
float y_planning_v1 =  40;
float z_planning_v1 =  12 + 60; 

// graspplanning position Version2 -> less invasive -> robot will move towards the object but stop above the IR sensor -> human can still grasp
float x_planning_v2 = 205;
float y_planning_v2 =  45;
float z_planning_v2 =  55; 


// grasp pickup place 
float x_grasp_pick = 282; // // before x=278, y=50, z= 12 
float y_grasp_pick =  40; 
float z_grasp_pick =  12;

// attention postion for rearing up and pointing
float x_attention = 201; 
float y_attention =   0; 
float z_attention = 170;

// attention postion for rearing up and pointing
float x_point_briefly = 218; 
float y_point_briefly =  16; 
float z_point_briefly = 158;

//########### times and flags for physical hrc 
int planning_time = 6; // time the robot waits above the object to simulate the grasp path planning 
int warning_time = 8; // Time the cancel action will block other actions from being executed -> should be bigger than the planning time! 
bool grasp_in_progress = false; // used to block pointing while grasp is in progress 
bool point_in_progress = false; // used to wait in grasping until pointing is done 
bool interrupt_immediately_flag = false; // is set true, then dobot will cancel immediately if warning is recognized - even before DESPOT decided to cancel 

//#################### function declarations ##################### 
void planCallback(const std_msgs::Bool::ConstPtr& msg);
void pointCallback(const std_msgs::Bool::ConstPtr& msg);
void graspCallback(const std_msgs::Bool::ConstPtr& msg);
void cancelCallback(const std_msgs::Bool::ConstPtr& msg);
void idleCallback(const std_msgs::Bool::ConstPtr& msg); 
void resumeQueueCallback(const std_msgs::Bool::ConstPtr&msg);
void returnHomeCallback(const std_msgs::Bool::ConstPtr&msg);
void receiveObjectToGraspCallback(const hrc_ros::ObjectGraspColourMsg &msg );
void gotoCallibrationCallback(const std_msgs::Bool::ConstPtr&msg);
bool calibrateScenario(std_srvs::TriggerRequest &req,std_srvs::TriggerResponse &res);
bool resetScenario(std_srvs::TriggerRequest &req,std_srvs::TriggerResponse &res);
void init_drop_locations(void);


// ########## Function implementations ##########################
void pointCallback(const std_msgs::Bool::ConstPtr& msg) {
		cout << " In Pointing thread"; 
		ros::param::get("/noDobot", no_Dobot_flag);	
		ros::param::get("/dobot_expression_version", dobot_expression_version); 
		hrc_ros::SetPTPCmd::Request			gotoStart_req;
		hrc_ros::SetPTPCmd::Response		gotoStart_resp;
		point_in_progress = true;  
		if (no_Dobot_flag == false){
			cout << " - calling services" << endl; 
		
			if(dobot_expression_version ==1){// ****************** execute pointing action V1 
					// Point move version 1:: Go up and down 
						if (grasp_in_progress == false && planning_in_progress == false ){ // skip if grasping or planning is in progress 
							// Point position 
							gotoStart_req.ptpMode = 1; // MoveJ move all joints independently -> max speed 
							gotoStart_req.x = 204;   
							gotoStart_req.y = 0; 
							gotoStart_req.z = 100;
							Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);

							// reset grasp_is_planned flag -> planning has to be done again  
							grasp_is_planned_flag = false; 

							ros::Duration(1.0).sleep();

							if(grasp_in_progress == false && planning_in_progress == false){
								// Point position -> go a bit lower  
								gotoStart_req.ptpMode = 1; // MoveJ move all joints independently -> max speed 
								gotoStart_req.x = 204;   
								gotoStart_req.y = 0; 
								gotoStart_req.z = 70;
								Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);
							}

							ros::Duration(0.1).sleep();

							if(grasp_in_progress == false && planning_in_progress == false ) {
							
								// Point position 
								gotoStart_req.ptpMode = 1; // MoveJ move all joints independently -> max speed 
								gotoStart_req.x = 204;   
								gotoStart_req.y = 0; 
								gotoStart_req.z = 100;
								Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);
							} 
							

							ros::Duration(1.0).sleep();  

							if(grasp_in_progress == false && planning_in_progress == false ) {
								// Point position -> go a bit lower  
								gotoStart_req.ptpMode = 1; // MoveJ move all joints independently -> max speed 
								gotoStart_req.x = 204;   
								gotoStart_req.y = 0; 
								gotoStart_req.z = 70;
								Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);
							}
							
							ros::Duration(0.1).sleep(); 

							if(grasp_in_progress == false && planning_in_progress == false ) {
								// Point position 
								gotoStart_req.ptpMode = 1; // MoveJ move all joints independently -> max speed 
								gotoStart_req.x = 204;   
								gotoStart_req.y = 0; 
								gotoStart_req.z = 100;
								Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);
							} 
							
							ros::Duration(1.0).sleep();
							
							if(grasp_in_progress == false && planning_in_progress == false ){ 
								// Standard position for the dobot arm 
								gotoStart_req.ptpMode = 1; // MoveJ move all joints independently -> max speed 
								gotoStart_req.x = x_idle;   
								gotoStart_req.y = y_idle; 
								gotoStart_req.z = z_idle;  
								Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);

							}
						}else { cout << " grasp in progress -> pointing skipped " << endl; }
						// ############ End of point version 1 #######################################
			} else if (dobot_expression_version ==2){      //****************** execute pointing action V2 
								
								
							// Getting the attention of the human by rearing up 
							if (grasp_in_progress == false && planning_in_progress == false ){ // skip if grasping or planning is in progress 
								// Point position 
								gotoStart_req.ptpMode = 1; // MoveJ move all joints independently -> max speed 
								gotoStart_req.x = x_attention; 
								gotoStart_req.y = y_attention; 
								gotoStart_req.z = z_attention; 
								Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);

								// reset grasp_is_planned flag -> planning has to be done again  
								grasp_is_planned_flag = false; 
								
								ros::Duration(1.0).sleep();

								// waiving towards the package -> pointing briefly 
								if(grasp_in_progress == false && planning_in_progress == false){
									gotoStart_req.ptpMode = 1; // MoveJ move all joints independently -> max speed 
									gotoStart_req.x = x_point_briefly;    
									gotoStart_req.y = y_point_briefly; 
									gotoStart_req.z = z_point_briefly; 
									Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);
								}

								ros::Duration(0.3).sleep();

								// going back to rear up position 
								if(grasp_in_progress == false && planning_in_progress == false ) {
								
									// Point position 
									gotoStart_req.ptpMode = 1; // MoveJ move all joints independently -> max speed 
									gotoStart_req.x = x_attention; 
									gotoStart_req.y = y_attention; 
									gotoStart_req.z = z_attention; 
									Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);
								} 
								ros::Duration(0.2).sleep();
								
							}else { cout << " grasp in progress -> pointing skipped " << endl; }
						// ############ End of point version 2 #######################################			
			}

		} else { // only wait - do not call dobot services  
		  cout << " ~ sleeping " << endl; 
		  ros::Duration(wait_time).sleep();
		}
	ros::param::get("/noDobot", no_Dobot_flag);	
	point_in_progress = false; 
	cout << " => finished pointing thread" << endl;
	
}



/*
*
*Executes a grasping action. The following steps are done: 
* 1. check if grasping path has been planned, if not call planning function
* 2. Request success criteria from observation_agent(the current_object_colour is transmitted, which is received by action_tracking node by publisher)
* 3. Calculate drop positions for the current_object based on success criteria (map tray to postions)  
* 4. Check the current CMD_queue_index before starting grasp, call the DOBOT API grasp routine (dobot_app layer service), 
* 5. check if warning_is_received while grasp is in progress, check if grasp finished by double checking if CMD_queu_index increased as expected
* 6. set grasp_state parameter => indicates if grasp is 0=ongoing | 1=finished successfully 3=warning received | 4=timeout or other error -> used by observation agent to block decision triggering while grasp is in progress  
* 7. reset grasp_is_planned_flag to false => next time planning is triggered it will be executed again (flag is used for planninCallback control) 
*/
void graspCallback(const std_msgs::Bool::ConstPtr& msg) {
		cout << endl <<" => In grasping thread " << endl; 
		int grasping_state = 0; // 0=ongoing | 1= grasping finished successfully 3=warning received | 4=timeout or other error | 5=empty conveyor 
		bool grasp_done_in_time = false;
		bool conveyor_empty_flag = false; 
		grasp_in_progress = true; 

		ros::param::get("/noDobot", no_Dobot_flag);
		if (no_Dobot_flag == false ) { // call dobot api services if available otherwise wait only 

			//set grasping state parameter before grasping => always 0
			ros::param::set("/robot_grasping_state", grasping_state);

			while(point_in_progress == true ) { cout << "point in grogress = " << point_in_progress << endl;  } // wait until pointing action is preemted -> triggering grasp will cancel pointing action

			// conveyor empty flag that will skip grasping and planning 
			if (object_to_grasp_colour == 4){
				conveyor_empty_flag = true; 
				cout << "conveyor is empty " <<endl; 
			}


			// 1. check if grasp has been planned, if not plan grasp path first
			while((grasp_is_planned_flag == false) && (warning_received_flag == false) && (conveyor_empty_flag==false)) {  
				std_msgs::Bool::ConstPtr msg; 
				cout << "planning is issued    conveyor_empty_flag = " << conveyor_empty_flag << endl; 
				planCallback(msg); // call planningCallback to do planning
			}


			if (conveyor_empty_flag == false){
				// drive to planning position (v1) directly above the object if it has not been reached yet 
				hrc_ros::SetPTPCmd::Request	 ensure_plan_loc_req; 
				hrc_ros::SetPTPCmd::Response ensure_plan_loc_resp; 
				ensure_plan_loc_req.ptpMode = 1; // moveJ 
				ensure_plan_loc_req.x = x_planning_v1; 
				ensure_plan_loc_req.y = y_planning_v1; 
				ensure_plan_loc_req.z = z_planning_v1;
				Dobot_gotoPoint.call(ensure_plan_loc_req,ensure_plan_loc_resp); 
				ros::Duration(0.2).sleep();
			}

			

			cout << " - calling services" << endl; 
			hrc_ros::SimplePickAndPlace::Request spp_request;
			hrc_ros::SimplePickAndPlace::Response spp_resp; 

			hrc_ros::RequestSuccessCriteria::Request success_req; 
			hrc_ros::RequestSuccessCriteria::Response success_resp;

			// 2. request success criteria 
			if (object_to_grasp_colour < 4) {// if colour is red, green or blue 
				success_req.current_object = object_to_grasp_colour; // object_colour is received by action recognition, object on conveyor in front of IR sensor is published
				 
			} else { // default is 1=red 
				success_req.current_object = 1; 
			}
			cout << "Grasping now -> will use object colour " << success_req.current_object << endl; 

			

			// get success tray from observation agent 
			request_success_criteria.call(success_req, success_resp);
			
			int tray   = success_resp.tray;
			
			// assign default tray as red tray in case wrong tray information received 
			if (tray >4 || tray < 1){
				tray = 1; 
				cout << " received tray info out of range -> will set to red =1 now" << endl; 
			}
			cout << "Requested succes_criteria from observation_agent -  object: " << success_req.current_object << "  tray: " << tray << endl;  

			// 3. map tray to placement positions here
			if(tray == 1){ // red container
				double x_incr = 0.0; 
				double y_incr = 0.0; 
				// calculate offset for placement based on first object
				if (red_placed_cnt == 1){ // second placement 
					x_incr = 0.0; 
					y_incr = 35.0;  
				} else if (red_placed_cnt == 2) {
					x_incr = 0.0; 
					y_incr = 70.0; 
				} else if (red_placed_cnt == 3){
					x_incr = 35.0; 
					y_incr = 35.0;
				} else if (red_placed_cnt == 4){
					x_incr = 35.0;
					y_incr = 70.0; 
				}

				spp_request.placeX = red_package_drop_loc.x + x_incr;
				spp_request.placeY = red_package_drop_loc.y + y_incr; 
				spp_request.placeZ = red_package_drop_loc.z; 
				red_placed_cnt +=1;

			} else if (tray == 2){ // green container

				double x_incr = 0.0; 
				double y_incr = 0.0; 
				// calculate offset for placement based on first object
				if (green_placed_cnt == 1){ // second placement 
					x_incr = 0.0; 
					y_incr = 35.0;  
				} else if (green_placed_cnt == 2) {
					x_incr = 0.0; 
					y_incr = 70.0; 
				} else if (green_placed_cnt == 3){
					x_incr = 35.0; 
					y_incr = 0.0;
				} else if (green_placed_cnt == 4){
					x_incr = 35.0;
					y_incr = 35.0; 
				}

				spp_request.placeX = green_package_drop_loc.x + x_incr;
				spp_request.placeY = green_package_drop_loc.y + y_incr; 
				spp_request.placeZ = green_package_drop_loc.z; 
				green_placed_cnt +=1;

			} else if (tray == 3){ // blue container 

				double x_incr = 0.0; 
				double y_incr = 0.0; 
				// calculate offset for placement based on first object
				if (blue_placed_cnt == 1){ // second placement 
					x_incr = 0.0; 
					y_incr = 35.0;  
				} else if (blue_placed_cnt == 2) {
					x_incr = 0.0; 
					y_incr = 70.0; 
				} else if (blue_placed_cnt == 3){
					x_incr = 35.0; 
					y_incr = 0.0;
				} else if (blue_placed_cnt == 4){
					x_incr = 35.0;
					y_incr = 35.0; 
				}

				spp_request.placeX = blue_package_drop_loc.x + x_incr;
				spp_request.placeY = blue_package_drop_loc.y + y_incr; 
				spp_request.placeZ = blue_package_drop_loc.z;
				blue_placed_cnt +=1; 
			}
		

			spp_request.pickX = x_grasp_pick; 
			spp_request.pickY = y_grasp_pick;  
			spp_request.pickZ = z_grasp_pick;  
			spp_request.isLocConfigEnabled = true; // override default drop locations
			

			cout << "place_x : " << spp_request.placeX << "place_y " << spp_request.placeY << "place_z " << spp_request.placeZ << endl; 

		
			// 4. get Dobots CommanQueuIndex before calling the grasp service, service is comprised of 5 dobot API services, so when the grasp is finished the index should have incremented by 5 (4 would be when dobot drops object)
			hrc_ros::GetQueuedCmdCurrentIndex::Request cmd_index_req; 
			hrc_ros::GetQueuedCmdCurrentIndex::Response cmd_index_resp; 
			Dobot_getQueueIndex.call(cmd_index_req,cmd_index_resp);  
			int before_grasp_index =  cmd_index_resp.queuedCmdIndex; 
			int after_grasp_index = before_grasp_index + 4 ;


			ros::Time stop_grasp_time;
			ros::Duration grasp_duration; 
			ros::Duration drop_duration; 
			ros::Time start_grasp_time; 
			ros::Time drop_time; 
			if (warning_received_flag == false && (conveyor_empty_flag == false) ){ // skip grasping if warning has been received or conveyor is empty 
				Dobot_SimplePickAndPlace.call(spp_request,spp_resp);
			
			//set grasping state parameter before grasping => always 0
			ros::param::set("/robot_grasping_state", grasping_state);
			
			// 5. check if grasping is done in time and without interruption of warning 
			int grasp_time = 0;
			start_grasp_time = ros::Time::now(); 
			drop_time =ros::Time::now(); 


			while(grasp_time < 30 ){ // the time is only accurate if services are not available
				grasp_done_in_time = false; 
				Dobot_getQueueIndex.call(cmd_index_req,cmd_index_resp);
				int current_queue_index = cmd_index_resp.queuedCmdIndex;
				cout << "current_queue_index: " << current_queue_index << "   grasp_time(1/100sec): " << grasp_time << endl;
				if (current_queue_index >= after_grasp_index){
					grasp_done_in_time = true; 
					break;
				}
				// TODO might be removed - used to measure timing behaviour of robot 
				if (current_queue_index >= before_grasp_index +4){
					drop_time = ros::Time::now(); 
				}

					ros::Duration(0.01).sleep();   
					grasp_time ++; 
				}
				stop_grasp_time = ros::Time::now();
				grasp_duration = stop_grasp_time - start_grasp_time;
				drop_duration = drop_time - start_grasp_time;   
			} else { cout << "warning received or empty conveyor belt  ->   skipping grasp" << endl; }

			// 6. set grasping_state via parameter
			cout << "-----" << endl << "grasp_state:   " << endl; 
			if(warning_received_flag == true ){
				grasping_state = 3; // grasping interrupted, since warning received 
				cout << "warning_received_during_grasp   grasp_state= " << grasping_state << "  took  : " << grasp_duration << " Time till droping the object  " << drop_duration << endl;
			} else if ( (warning_received_flag == false) && (grasp_done_in_time == true) ){
				grasping_state = 1; // grasping successfully finished in time 
				cout << "grasp_done_in_time = " << grasp_done_in_time << "  grasp_state = " << grasping_state  << "  took  : " << grasp_duration << " Time till droping the object  " << drop_duration << endl;
			} else if ( conveyor_empty_flag == true ) { // timeout or other error grasping_state = 4 

				grasping_state = 5; 
				cout << "grasp not executed -> conveyor is empty     grasp_state = " << grasping_state << endl; 	
			} else {
				
				grasping_state = 4; 
				cout << "Grasp_not_successfull - is dobot running?   grasp_state=  " << grasping_state << endl; 
			}

			
			

			ros::param::set("/robot_grasping_state", grasping_state);	// set grasping_state parameter -> checked by observation agent to block decision making while grasping

			// insert wait time if after_grasp_index is only 4 bigger than before_grasp_index 
			ros::Duration(5).sleep();
			 
			// TODO remove when not testing with setup 
			//ros::Duration(10).sleep(); 
		} else { // only wait - do not call dobot services 
		  cout << " ~ sleeping " << endl;
		  ros::Duration(wait_time).sleep();
		}
		
		// 7. reset grasp_planned_flag -> when the next planning is triggered it will actually be executed (as log as it is true planning is always skipped)
		grasp_is_planned_flag = false; 
		grasp_in_progress = false; 
		ros::param::get("/noDobot", no_Dobot_flag);	
		cout << " <= finished grasping thread" << endl;
		
}

void cancelCallback(const std_msgs::Bool::ConstPtr& msg) {
		cout << endl << " => In cancel thread " << endl;
		ros::param::get("/noDobot", no_Dobot_flag);	
		warning_received_flag = true; 
		
		// reset all flags 
		grasp_is_planned_flag = false; 
		bool planning_in_progress  = false;
		bool grasp_in_progress = false; 
		bool point_in_progress = false; 

		if (no_Dobot_flag == false){ // call dobot api service
			cout << " - calling services" << endl;  
			hrc_ros::SetQueuedCmdForceStopExec::Request 	forceStopQueue_req; 
			hrc_ros::SetQueuedCmdForceStopExec::Response  forceSTopQueue_resp; 

			hrc_ros::SetQueuedCmdClear::Request		   	clearQueue_req;
			hrc_ros::SetQueuedCmdClear::Response		   	clearQueue_resp;

			hrc_ros::SetQueuedCmdStartExec::Request     	startQueue_req;
			hrc_ros::SetQueuedCmdStartExec::Response     	startQueue_resp;

			hrc_ros::SetEndEffectorSuctionCup::Request  suctionCup_req;
			hrc_ros::SetEndEffectorSuctionCup::Response suctionCup_resp;
			suctionCup_req.suck = 0; 
			suctionCup_req.enableCtrl = 1;


			hrc_ros::SetPTPCmd::Request				   	gotoStart_req;
			hrc_ros::SetPTPCmd::Response				   	gotoStart_resp; 

			// Standard position for the dobot arm 
			gotoStart_req.ptpMode = 1; //moveJ - move joints independently -> fast movement
			gotoStart_req.x = x_idle;   
			gotoStart_req.y = y_idle;  
			gotoStart_req.z = z_idle; 

			Dobot_SetQueuedCmdForceStopExec.call(forceStopQueue_req,forceSTopQueue_resp);
			Dobot_SetQueuedCmdClear.call(clearQueue_req, clearQueue_resp);
			Dobot_SetQueuedCmdStartExec.call(startQueue_req, startQueue_resp);
			Dobot_SetEndEffectorSuctionCup.call(suctionCup_req,suctionCup_resp); // turn off suctionCup -> Dobot will drop the object 
			Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);
		
		} else { 
			cout << " ~ sleeping " << endl;
			ros::Duration(wait_time).sleep(); 
		}
		
		ros::Duration(warning_time).sleep(); 
		warning_received_flag = false; // reset warning received flag -> new commands can be issued again 
		cout << " <= finished cancel action" << endl; 
		ros::param::get("/noDobot", no_Dobot_flag);	
}


/*
*
* Simulates the planning phase of a grasping path. Will either be called by Dobot after deciding either for planning action or for grasping action but the path is not planned yet. 
* 1. check if grasp has already been planned(grasp_is_planned_flag) or planning is currently in progress(planning_in_progress)
* 2. Move Dobot to a grasping position and stop there. Depending on the dobot_expression_version flag the position is either 1: directly above the object or 2: above the IR sensor => less intrusive
     skip further planning by setting planning_in_progress = true 
* 3. Wait for a specified time (planning_time), this simulates the planning of the path
* 4. Set grasp_is_planned_flag = true (planning will be skipped if function called again) |  and planning_in_progress=false (used to skip planning, if planning is already in progress) 
*
* NOTE: flags are set as follows: 
*       - unplanned path 			=> grasp_is_planned_flag = false  | planning_in_progress = false   | planCallback will plan, if called
*       - path planning in progress => grasp_is_planned_flag = false  | planning_in_progress = true    | planCallback will skip planning if called 
*		- path planning done    	=> grasp_is_planned_flag = true   | planning_in_progress = false   | planCallback will skip planning if called
*		- after grasp is done   	=> grasp_is_planned_flag = false  | planning_in_progress = false   | planCallback will plan, if called | the grasp_is_planned_flag is reset by graspCallback after grasp is completely done
*/
void planCallback(const std_msgs::Bool::ConstPtr& msg) {
	  ros::param::get("/noDobot", no_Dobot_flag);	
		ros::param::get("/dobot_expression_version", dobot_expression_version);
	  cout << endl << " => In planning thread" << endl;
	  bool conveyor_empty = false;
	   

	  // check if conveyor is empty 
	  if (object_to_grasp_colour == 4){
		  conveyor_empty = true; 
	  }

	  while (point_in_progress == true){ } // wait until pointing is done 

	  cout << "grasp_is_planned = " << grasp_is_planned_flag << "  planning_in_progress = " << planning_in_progress << "  no_Dobot_flag = " << no_Dobot_flag << endl; 
	  // 1. check if grasp has already been planned(grasp_is_planned_flag) or planning is currently in progress(planning_in_progress)
	  if (grasp_is_planned_flag == false && (planning_in_progress==false) && (conveyor_empty == false) )  { // flag reset to false after each grasp
	  	  planning_in_progress = true;  
		if (no_Dobot_flag == false  ){ // call dobot api service

			if (warning_received_flag == false ){ // skip planning if warning has been received 
				cout << "  -> calling goto service " << endl; 			
				
				hrc_ros::SetPTPCmd::Request				  gotoPlanning_req;
				hrc_ros::SetPTPCmd::Response				gotoPlanning_resp; 

				if (dobot_expression_version == 1){ //########### execute planning movement V1 -> directly above the object and wait 
					// 2. move to a position directly above the object and stop there -> this indicates that robot needs to plan the path 
					gotoPlanning_req.ptpMode = 1; // MoveJ -> move joint independently -> maximum speed 
					gotoPlanning_req.x = x_planning_v1; 
					gotoPlanning_req.y = y_planning_v1; 
					gotoPlanning_req.z = z_planning_v1; 
				} else if (dobot_expression_version ==2){ // ###### execute planning movement V2 -> move above the IR sensor and wait
					gotoPlanning_req.ptpMode = 1; // MoveJ -> move joint independently -> maximum speed 
					gotoPlanning_req.x = x_planning_v2; 
					gotoPlanning_req.y = y_planning_v2; 
					gotoPlanning_req.z = z_planning_v2; 
				}

				Dobot_gotoPoint.call(gotoPlanning_req,gotoPlanning_resp);
			}
	  	} else { 
			cout << " ~ sleeping " << endl;
			ros::Duration(wait_time).sleep(); 
	  	}

	  // 3. Wait for a specified time (planning_time), this simulates the planning of the path
	  ros::Duration(planning_time).sleep();
	  
	  // 4. Set grasp_is_planned_flag to true and planning_in_progress to false 
	  ros::param::get("/noDobot", no_Dobot_flag);
	  if(warning_received_flag == false){ // if warning received then set the flags accordingly 
	  	grasp_is_planned_flag = true;
	  	planning_in_progress = false; 
	  } else {
		grasp_is_planned_flag = false; 
		planning_in_progress = false; 
	  }
	  cout << " <= finished planning thread" << endl; 
	  

	} else if (conveyor_empty == true) {
		cout << " conveyor is empty -> planning skipped " << endl; 
	} else { 
		cout << " planning finished or currently in progress " << endl; 
	}
}


void idleCallback(const std_msgs::Bool::ConstPtr& msg) {
	  ros::param::get("/noDobot", no_Dobot_flag);	
	  cout << " In IDLE thread" << endl;
	  grasp_is_planned_flag = false; 
	   
	  while(point_in_progress == true){ cout << " point_in_progress " << point_in_progress; } // wait until pointing is done 
	  
	  cout << endl << " grasp_in_progress flag " << grasp_in_progress << endl; 
	  if ((grasp_in_progress==false) )  { // skip if grasp is in progress 
	  
		if (no_Dobot_flag == false  ){ // call dobot api service

			if (warning_received_flag == false ){ // skip idle if warning has been received 
				cout << "  -> calling goto service " << endl; 			
				// 2. move to a position directly above the object and stop there -> this indicates that robot needs to plan the path 
				hrc_ros::SetPTPCmd::Request				   	gotoPlanning_req;
				hrc_ros::SetPTPCmd::Response				gotoPlanning_resp; 

				gotoPlanning_req.ptpMode = 1; // MoveJ -> move joint independently -> maximum speed 
				gotoPlanning_req.x = x_idle; 
				gotoPlanning_req.y = y_idle; 
				gotoPlanning_req.z = z_idle; 

				Dobot_gotoPoint.call(gotoPlanning_req,gotoPlanning_resp);

				// reset grasp_is_planned -> it will have to be replanned 
				grasp_is_planned_flag = false; 
			}
	  	} else { 
			cout << " ~ sleeping " << endl;
			ros::Duration(wait_time).sleep(); 
	  	}
	}


	  cout << " <= finished Idle thread" << endl; 
	  ros::param::get("/noDobot", no_Dobot_flag); 
}

void resumeQueueCallback(const std_msgs::Bool::ConstPtr&msg)
{
		cout << " In resumeQueue thread";
		ros::param::get("/noDobot", no_Dobot_flag);	
		if (no_Dobot_flag == false) { // call full dobot api service 
			cout << " - calling services" << endl; 
			hrc_ros::SetQueuedCmdClearRequest		   	clearQueue_req;
			hrc_ros::SetQueuedCmdClearResponse		   	clearQueue_resp;

			hrc_ros::SetQueuedCmdStartExecRequest     	startQueue_req;
			hrc_ros::SetQueuedCmdStartExecResponse     	startQueue_resp;


			Dobot_SetQueuedCmdClear.call(clearQueue_req, clearQueue_resp);
			Dobot_SetQueuedCmdStartExec.call(startQueue_req, startQueue_resp);
	
		} else { // wait only 
			cout << " ~ sleeping " << endl;
			ros::Duration(wait_time).sleep();
		} 

	cout << " finished resume thread" << endl;
	ros::param::get("/noDobot", no_Dobot_flag);
}

void returnHomeCallback(const std_msgs::Bool::ConstPtr&msg)
{
		cout << " In returnHome thread";
		ros::param::get("/noDobot", no_Dobot_flag);	
		if (no_Dobot_flag == false) { // call dobot api service 
			cout << " - calling services" << endl; 
			hrc_ros::SetPTPCmdRequest		gotoStart_req;
			hrc_ros::SetPTPCmdRequest		gotoStart_resp; 

			// Standard position for the dobot arm 
			gotoStart_req.x = x_idle;   
			gotoStart_req.y = y_idle; 
			gotoStart_req.z = z_idle; 

			Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);
		} else { // dobot not present - wait only 
			cout << " ~ sleeping " << endl;
			ros::Duration(wait_time).sleep();
		}

		cout << " finished returnHome thread " << endl; 
		ros::param::get("/noDobot", no_Dobot_flag);
}

void receiveObjectToGraspCallback(const hrc_ros::ObjectGraspColourMsg &msg ){
 
			object_to_grasp_colour = msg.object_colour; 
			//cout << "object_to_grasp  " << object_to_grasp_colour << endl; 

}

// use to receive action by action recognition and issue cancel action faster by circumventing DESPOT 
void actionRecognizedCallback(const hrc_ros::PublishActionRecognisedMsg &msg){

	if (msg.action.compare("warning") == 0 && (interrupt_immediately_flag == true) ){
		ROS_WARN("\n Warning received & interrupt_immediately_flag is set - will cancel \n"); 
		std_msgs::Bool::ConstPtr msg;
		cancelCallback(msg);
	}

}

void gotoCallibrationCallback(const std_msgs::Bool::ConstPtr&msg)
{
	cout << " In gotoCallibration thread";
	ros::param::get("/noDobot", no_Dobot_flag);	
	if (no_Dobot_flag==false) {
		cout << " - calling services" << endl; 
		hrc_ros::SetPTPCmdRequest goto_request; 
		hrc_ros::SetPTPCmdResponse goto_resp;
		goto_request.x = 60;   
		goto_request.y = 250; 
		goto_request.z = -45; 
		Dobot_gotoPoint.call(goto_request,goto_resp);
	} else { // dobot not present - wait only 
		cout << " ~ sleeping " << endl;
		ros::Duration(wait_time).sleep();
	}
	cout << "finished gotoCallibration thread" << endl; 
	ros::param::get("/noDobot", no_Dobot_flag);
}

// called to drive Dobot to a position where the containers can be calibrated. Containers will be visible. 
bool calibrateScenario(std_srvs::TriggerRequest &req,std_srvs::TriggerResponse &res){



  cout << "goto clibration position " << endl; 
	// goto calibration position: 
	hrc_ros::SetPTPCmdRequest goto_request; 
	hrc_ros::SetPTPCmdResponse goto_resp;
	goto_request.x = 60;   
	goto_request.y = 250; 
	goto_request.z = -45; 
	Dobot_gotoPoint.call(goto_request,goto_resp);

	cout << "Wait there for 60 seconds before returning to IDLE position" << endl; 
	
	cout << "Make sure conveyor is enabled in the meantime" << endl; 
	// enable conveyor belt 
	hrc_ros::InOprConveyorControlRequest enableConv_request; 
	hrc_ros::InOprConveyorControlResponse enableConv_resp;
	enableConv_request.isEnabled = true; 
	enableConv_request.speed = 25; 
	enableConveyor.call(enableConv_request,enableConv_resp); 

	sleep(20); 

  cout << "goto IDLE position" << endl; 
	// goto IDLE position: 
	goto_request.x = x_idle; //215;   
	goto_request.y = y_idle;//45; 
	goto_request.z = z_idle; //30; 
	Dobot_gotoPoint.call(goto_request,goto_resp);

  // returns 
	res.success = true;
	return true;
}

bool resetScenario(std_srvs::TriggerRequest &req,std_srvs::TriggerResponse &res) {

	//ros::param::get("/...",variable );
	//TODO Package location could also be retriefed by rosparameter
	red_package_drop_loc.x = 160.0; 
	red_package_drop_loc.y = -285.0;
	red_package_drop_loc.z = -39.0; 

	// green drop place 
	green_package_drop_loc.x =  70.0; 
	green_package_drop_loc.y = -290.0;
	green_package_drop_loc.z = -39.0; 

	blue_package_drop_loc.x =  -30.0; 
	blue_package_drop_loc.y = -300.0;
	blue_package_drop_loc.z =  -39.0;

	// reset placed counters 
	red_placed_cnt = 0; 
	green_placed_cnt = 0; 
	blue_placed_cnt = 0;

	// reset flags 
	bool warning_received_flag = false;
	bool grasp_is_planned_flag = false;
	bool planning_in_progress  = false;
	bool grasp_in_progress = false; 
	bool point_in_progress = false; 



	// additional task relevant resets 
	object_to_grasp_colour = 4;

	// move dobot to idle position 
	std_msgs::Bool::ConstPtr msg;
	idleCallback(msg); 

	ROS_WARN("[DOBOT WORKER] Reset!");

	res.success = true;
	return true;
}








void init_drop_locations(void) {

	//ros::param::get("/...",variable );
	//TODO Package location could also be retriefed by rosparameter
	red_package_drop_loc.x = 160.0; 
	red_package_drop_loc.y = -285.0;
	red_package_drop_loc.z = -39.0; 

	// green drop place 
	green_package_drop_loc.x =  70.0; 
	green_package_drop_loc.y = -290.0;
	green_package_drop_loc.z = -39.0; 

	blue_package_drop_loc.x =  -30.0; 
	blue_package_drop_loc.y = -300.0;
	blue_package_drop_loc.z =  -39.0; 

	// reset placed counters 
	red_placed_cnt = 0; 
	green_placed_cnt = 0; 
	blue_placed_cnt = 0;

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "dobot_worker_node");
	ros::NodeHandle nh;
	ros::CallbackQueue my_queue; 
	// declare object here is class is use 
	//RobotMotionAgent robot_motion_agent;

	init_drop_locations(); 
  	ROS_INFO("Robot Motion Agent multi is ready - Dobot can be controlled now !");

	

	ros::AsyncSpinner spinner(12 /*number of threads*/, &my_queue /* spinner exclusively for my_queue */); 
	
	// bind the queue to the node handle
	nh.setCallbackQueue( &my_queue );

 
	ros::param::get("/noDobot", no_Dobot_flag);
	ros::param::get("/dobot_expression_version", dobot_expression_version);

	cout << " no_Dobot_flag = " << no_Dobot_flag << endl << " You can set it by calling rosparam set /noDobot [true/false]  -- will be picked up by node" << endl; 
	

	// Subscribers should go here single subscriber for each action
	ros::Subscriber dobot_grasp_sub = nh.subscribe("/robot_motion_agent/dobot_grasp",1, graspCallback);  
  ros::Subscriber dobot_cancel_sub = nh.subscribe("/robot_motion_agent/dobot_cancel",1, cancelCallback); 
	ros::Subscriber dobot_plan_sub = nh.subscribe("/robot_motion_agent/dobot_plan",1, planCallback);  
	ros::Subscriber dobot_idle_sub = nh.subscribe("/robot_motion_agent/dobot_idle",1, idleCallback); 
	ros::Subscriber dobot_point_sub = nh.subscribe("/robot_motion_agent/dobot_point",1, pointCallback); 
	ros::Subscriber dobot_resume_sub = nh.subscribe("/robot_motion_agent/dobot_resume",1, resumeQueueCallback); 
	ros::Subscriber dobot_gohome_sub = nh.subscribe("/robot_motion_agent/dobot_gohome",1, returnHomeCallback); 
	ros::Subscriber dobot_calibration_sub = nh.subscribe("/robot_motion_agent/dobot_calibration",1, gotoCallibrationCallback); 
	// Other subscribers 
	ros::Subscriber ObjectToGrasp_sub = nh.subscribe("/object_tracking/object_tograsp_colour",1,receiveObjectToGraspCallback) ;
	ros::Subscriber Action_recognised_sub = nh.subscribe("/action_recognition/publish_action_recognized",1,actionRecognizedCallback);

	// Services 
	calibrate_scenario = nh.advertiseService("/dobot_worker/calibrate", calibrateScenario);
	reset_scenario = nh.advertiseService("/dobot_worker/reset", resetScenario);
	Dobot_SimplePickAndPlace          = nh.serviceClient<hrc_ros::SimplePickAndPlace>("/dobot_arm_app/simplePickAndPlace");
	Dobot_SetPTPCoordinateParams 			= nh.serviceClient<hrc_ros::SetPTPCoordinateParams>("/dobot_arm_app/setPTPCoordinateParamsApp");  // not used so far
  Dobot_InitDobotArmApp 		 				= nh.serviceClient<hrc_ros::InitDobotArmApp>("/dobot_arm_app/init"); 															// not used so far
	Dobot_SetQueuedCmdForceStopExec		= nh.serviceClient<hrc_ros::SetQueuedCmdForceStopExec  >("/dobot_arm_app/setQueuedCmdForceStopExecApp");
	Dobot_SetQueuedCmdStopExec  			= nh.serviceClient<hrc_ros::SetQueuedCmdStopExec>("/dobot_arm_app/setQueuedCmdStopExecApp");			// not used so far
	Dobot_SetQueuedCmdStartExec 			= nh.serviceClient<hrc_ros::SetQueuedCmdStartExec>("/dobot_arm_app/setQueuedCmdStartExecApp");
	Dobot_SetQueuedCmdClear 					= nh.serviceClient<hrc_ros::SetQueuedCmdClear>("/dobot_arm_app/setQueuedCmdClearApp");
	Dobot_SetEndEffectorSuctionCup 		= nh.serviceClient<hrc_ros::SetEndEffectorSuctionCup>("/dobot_arm_app/setEndEffectorSuctionCupApp");
	Dobot_gotoPoint 									= nh.serviceClient<hrc_ros::SetPTPCmd>("/dobot_arm_app/gotoPointApp");
	Dobot_ContPickAndPlace 						= nh.serviceClient<hrc_ros::ContPickAndPlace>("/dobot_arm_app/contPickAndPlace");									// not used so far
	Dobot_oneTimePickAndPlace 				= nh.serviceClient<hrc_ros::OneTimePickAndPlace>("/dobot_arm_app/oneTimePickAndPlace");						// not used so far
	request_success_criteria				= nh.serviceClient<hrc_ros::RequestSuccessCriteria>("/observation_agent/request_success_criteria");
	enableConveyor							= nh.serviceClient<hrc_ros::InOprConveyorControl>("/conveyor_control_app/inOprConveyorControl"); 
	Dobot_getQueueIndex						= nh.serviceClient<hrc_ros::GetQueuedCmdCurrentIndex>("/dobot_arm_app/getQueuedCmdCurrentIndexApp");
	

	// starting spinners with multiple threads 
	spinner.start();
	ros::waitForShutdown();
}



