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


// Variables that switch between fullDobot setup and a wait only version 
bool no_Dobot_flag = false; // used for debugging without dobot (true= only wait | false= call dobot service ) || is set by ros_param 
int wait_time = 20; // time dobot should wait in noDobot mode
bool warning_received_flag = false; // flag that indicates that a warning has been received, this is relevant for the grasping action, it is set to false at the beginning of the grasp and checked wether it is set true during the grasp

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

void pointCallback(const std_msgs::Bool::ConstPtr& msg) {
		cout << " In Pointing thread"; 
		ros::param::get("/noDobot", no_Dobot_flag);	
		hrc_ros::SetPTPCmd::Request		gotoStart_req;
		hrc_ros::SetPTPCmd::Response		gotoStart_resp; 
		if (no_Dobot_flag == false){
			cout << " - calling services" << endl; 
		
		// Point move version 1:: Go up and down 

			// Point position 
			gotoStart_req.x = 219;   
			gotoStart_req.y = 40; 
			gotoStart_req.z = 94;
			Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);

			// Standard position for the dobot arm 
			gotoStart_req.x = 215;   
			gotoStart_req.y = 45; 
			gotoStart_req.z = 30; 
			Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);
		
			// Point position 
			gotoStart_req.x = 219;   
			gotoStart_req.y = 40; 
			gotoStart_req.z = 94;
			Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);

			// Standard position for the dobot arm 
			gotoStart_req.x = 215;   
			gotoStart_req.y = 45; 
			gotoStart_req.z = 30; 
			Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);

		cout << " Will sleep for 3 seconds and do second pointing afterwards " << endl; 
		sleep(3);
		// Point move version 2 :: pointing 	
		
		
			// Point position 
			gotoStart_req.x = 230;   
			gotoStart_req.y = 50; 
			gotoStart_req.z = 40;
			Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);

			// Move a bit further to make point obvious 
			gotoStart_req.x = 230;   
			gotoStart_req.y = 48; 
			gotoStart_req.z = 42;
			Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);

			// Standard position for the dobot arm 
			gotoStart_req.x = 215;   
			gotoStart_req.y = 45; 
			gotoStart_req.z = 30; 
			Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);
		} else { // only wait - do not call dobot services  
		  cout << " ~ sleeping " << endl; 
		  sleep(wait_time);
		}
	ros::param::get("/noDobot", no_Dobot_flag);	
	cout << " => finished pointing thread" << endl;
	

}

void graspCallback(const std_msgs::Bool::ConstPtr& msg) {
		cout << " In grasping thread "; 
		int grasping_state = 0; // 0=ongoing | 1= grasping finished successfully 3=warning received | 4=timeout or other error 
		bool grasp_done_in_time = false;
		warning_received_flag = false; // set at beginning, if it changes during the grasping action a warning occured 

		ros::param::get("/noDobot", no_Dobot_flag);
		if (no_Dobot_flag == false ) { // call dobot api services 
			cout << " - calling services" << endl; 
			hrc_ros::SimplePickAndPlace::Request spp_request;
			hrc_ros::SimplePickAndPlace::Response spp_resp; 

			hrc_ros::RequestSuccessCriteria::Request success_req; 
			hrc_ros::RequestSuccessCriteria::Response success_resp;

			if (object_to_grasp_colour < 4) {// if colour is red, green or blue 
				success_req.current_object = object_to_grasp_colour; // object_colour is received by action recognition, object on conveyor in front of IR sensor is published
				 
			} else { // default is 1=red 
				success_req.current_object = 1; 
			}
			cout << "Grasping now -> will use object colour " << success_req.current_object << endl; 

			// get success tray from observation agent 
			request_success_criteria.call(success_req, success_resp);
			
			int tray   = success_resp.tray;

			cout << "Requested succes_criteria from observation_agent -  object: " << success_req.current_object << "  tray: " << tray << endl;  

			// TODO - map tray to placement positions here
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
		

			// TODO PickPlace could also be configured by param server 
			spp_request.pickX = 282; // before 278; 
			spp_request.pickY = 40;//before 50; 
			spp_request.pickZ = 12; 
			spp_request.isLocConfigEnabled = true; // override default drop locations

			cout << "place_x : " << spp_request.placeX << "place_y " << spp_request.placeY << "place_z " << spp_request.placeZ << endl; 

			// get Dobots CommanQueuIndex before calling the grasp service, service is comprised of 3 dobot API services, so when the grasp is finished the index should have incremented by 5 (4 would be when dobot drops object)
			hrc_ros::GetQueuedCmdCurrentIndex::Request cmd_index_req; 
			hrc_ros::GetQueuedCmdCurrentIndex::Response cmd_index_resp; 
			Dobot_getQueueIndex.call(cmd_index_req,cmd_index_resp);  
			int before_grasp_index =  cmd_index_resp.queuedCmdIndex; 
			int after_grasp_index = before_grasp_index + 5 ; 
			Dobot_SimplePickAndPlace.call(spp_request,spp_resp);

			//set grasping state parameter before grasping => always 0
			ros::param::set("/robot_grasping_state", grasping_state);
			
			// check if grasping is done in time and without interruption of warning 
			int grasp_time = 0;
			ros::Time start_grasp_time = ros::Time::now(); 
			ros::Time drop_time =ros::Time::now(); 
			while(grasp_time < 800 ){ // the time is only accurate if services are not dobot services are not available
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
			ros::Time stop_grasp_time = ros::Time::now();
			ros::Duration grasp_duration = stop_grasp_time - start_grasp_time;
			ros::Duration drop_duration = drop_time - start_grasp_time;   
			 

			// set grasping_state via parameter
			if(warning_received_flag == true ){
				grasping_state = 3; // grasping interrupted, since warning received 
				cout << "warning_received_during_grasp   grasp_state= " << grasping_state << "  took  : " << grasp_duration << " Time till droping the object  " << drop_duration << endl;
			} else if ( (warning_received_flag == false) && (grasp_done_in_time == true) ){
				grasping_state = 1; // grasping successfully finished in time 
				cout << "grasp_done_in_time = " << grasp_done_in_time << "  grasp_state = " << grasping_state  << "  took  : " << grasp_duration << " Time till droping the object  " << drop_duration << endl;
			} else { // timeout or other error grasping_state = 4 
				grasping_state = 4; 
				cout << "Grasp_not_successfull - is dobot running?   grasp_state=  " << grasping_state << endl; 
			}


			ros::param::set("/robot_grasping_state", grasping_state);

			// TODO remove when not testing with setup 
			//ros::Duration(10).sleep(); 
		} else { // only wait - do not call dobot services 
		  cout << " ~ sleeping " << endl;
		  sleep(wait_time); 
		}
		ros::param::get("/noDobot", no_Dobot_flag);	
		cout << " => finished grasping thread" << endl;
		
}

void cancelCallback(const std_msgs::Bool::ConstPtr& msg) {
		cout << " In cancel thread ";
		ros::param::get("/noDobot", no_Dobot_flag);	
		warning_received_flag = true; 
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


			hrc_ros::SetPTPCmdRequest				   	gotoStart_req;
			hrc_ros::SetPTPCmdRequest				   	gotoStart_resp; 

			// Standard position for the dobot arm 
			gotoStart_req.x = 215;   
			gotoStart_req.y = 45; 
			gotoStart_req.z = 30; 

			Dobot_SetQueuedCmdForceStopExec.call(forceStopQueue_req,forceSTopQueue_resp);
			Dobot_SetQueuedCmdClear.call(clearQueue_req, clearQueue_resp);
			Dobot_SetQueuedCmdStartExec.call(startQueue_req, startQueue_resp);
			Dobot_SetEndEffectorSuctionCup.call(suctionCup_req,suctionCup_resp); // turn off suctionCup -> Dobot will drop the object 
			Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);
		} else { 
			cout << " ~ sleeping " << endl;
			sleep(wait_time); 
		}
		
		cout << " => finished cancel action" << endl; 
		ros::param::get("/noDobot", no_Dobot_flag);	
}

void planCallback(const std_msgs::Bool::ConstPtr& msg) {
	  ros::param::get("/noDobot", no_Dobot_flag);	
	  cout << " In planning thread" << endl;
	  sleep(wait_time);
	  cout << " => finished planning thread" << endl; 
	  ros::param::get("/noDobot", no_Dobot_flag);
}

void idleCallback(const std_msgs::Bool::ConstPtr& msg) {
	  ros::param::get("/noDobot", no_Dobot_flag);	
	  cout << " In IDLE thread" << endl;
	  sleep(wait_time);
	  cout << " => finished planning thread" << endl;
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
			sleep(wait_time); 
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
			gotoStart_req.x = 215;   
			gotoStart_req.y = 45; 
			gotoStart_req.z = 30; 

			Dobot_gotoPoint.call(gotoStart_req,gotoStart_resp);
		} else { // dobot not present - wait only 
			cout << " ~ sleeping " << endl;
			sleep(wait_time);
		}

		cout << " finished returnHome thread " << endl; 
		ros::param::get("/noDobot", no_Dobot_flag);
}

void receiveObjectToGraspCallback(const hrc_ros::ObjectGraspColourMsg &msg ){

	if (msg.object_colour < 4 ){ // not 4 => not reference point 
			object_to_grasp_colour = msg.object_colour; 
			//cout << "object_to_grasp  " << object_to_grasp_colour << endl; 
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
		sleep(wait_time); 
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

	sleep(60); 

  cout << "goto IDLE position" << endl; 
	// goto IDLE position: 
	goto_request.x = 215;   
	goto_request.y = 45; 
	goto_request.z = 30; 
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

	ros::init(argc, argv, "robot_motion_agent_multi");
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



