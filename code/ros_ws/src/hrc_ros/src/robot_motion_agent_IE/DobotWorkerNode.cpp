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

// Variables that switch between fullDobot setup and a wait only version 
bool no_Dobot_flag = false; // used for debugging without dobot (true= only wait | false= call dobot service ) || is set by ros_param 
int wait_time = 20; // time dobot should wait in noDobot mode

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
		ros::param::get("/noDobot", no_Dobot_flag);
		if (no_Dobot_flag == false ) { // call dobot api services 
			cout << " - calling services" << endl; 
			hrc_ros::SimplePickAndPlace::Request spp_request;
			hrc_ros::SimplePickAndPlace::Response spp_resp; 

			hrc_ros::RequestSuccessCriteria::Request success_req; 
			hrc_ros::RequestSuccessCriteria::Response success_resp;

			request_success_criteria.call(success_req, success_resp);

			int object = success_resp.object; 
			int tray   = success_resp.tray;

			cout << "Requested succes_criteria from observation_agent -  object: " << object << "  tray: " << tray << endl;  

			// TODO - map tray to placement positions here 
			spp_request.pickX = 278; 
			spp_request.pickY = 50; 
			spp_request.pickZ = 16; 
			spp_request.placeX = 252;
			spp_request.placeY = -204;
			spp_request.placeZ =  -38; 
			Dobot_SimplePickAndPlace.call(spp_request,spp_resp);
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


int main(int argc, char **argv) {

	ros::init(argc, argv, "robot_motion_agent_multi");
	ros::NodeHandle nh;
	ros::CallbackQueue my_queue; 
	// declare object here is class is use 
	//RobotMotionAgent robot_motion_agent;

  	ROS_INFO("Robot Motion Agent multi is ready - Dobot can be controlled now !");

	ros::AsyncSpinner spinner(11 /*number of threads*/, &my_queue /* spinner exclusively for my_queue */); 
	
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

	// starting spinners with multiple threads 
	spinner.start();
	ros::waitForShutdown();
}



