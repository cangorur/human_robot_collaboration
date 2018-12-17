#include "ros/ros.h"
#include <ros/package.h>
#include "object_tracking/Feature.h"
#include "object_tracking/InformActionRecognized.h"
#include <iostream>
#include <fstream>
#include <thread>



//			rosservice call /dobot_arm_app/contPickAndPlace "{isEnabled: true, isLocConfigEnabled: false, pickX: 269.80, pickY: 48.50, pickZ: 12.94, placeX: 244.50, placeY: -206.10, placeZ: -35.85, placeInt: 90.0, placeIntZ: 40.0, placeLocR: 0, placeLocG: 1, placeLocB: 2, boxLimR: 3, boxLimG: 3, boxLimB: 3, storageLim: 3}"


using namespace std;

ofstream myfile;

ros::ServiceClient *clientPtr;

int stop_flag = 0;
bool toggle = false;

void call_iar(int action){
	ROS_INFO("CALL INFORM ACTION RECOGNIZED");

	object_tracking::InformActionRecognized iar;

	if(action>0){
		iar.request.human_detected = true;
		iar.request.human_looking_around = false;
		iar.request.action_duration = 5;
		iar.request.action = "idle";
		iar.request.action = "grasp";
		iar.request.action = "gesture";
	}

	if(clientPtr->call(iar)) //request service from the client
    {
        ROS_INFO("Success service call");
    }
    else
    {
        ROS_ERROR("Fail call service");
    }
}


bool feat_print(object_tracking::Feature::Request  &req,
		object_tracking::Feature::Response &res)
{

	res.sth = 1;
//	ROS_INFO("request: velo=%f, dist=%f", (float)req.velocity, (float)req.distance);
	ROS_INFO("request: glove z=%f", (float)req.glovez);
//	ROS_INFO("request: time sec:%d nsec:%d", (int)req.rostime.sec, (int)req.rostime.nsec);
//	ROS_INFO("sending back response: [%ld]", (long int)res.sth);

	if (myfile.is_open()){

		myfile <<(float)req.velocity<<",";
		myfile <<(float)req.distance<<",";
		myfile <<(int)req.rostime.sec<<":"<<(int)req.rostime.nsec;
		myfile<<endl;
	}
	else cout << "Unable to open file";

	thread thd(call_iar,1);
	thd.detach();



//	if((float)req.glovez>0 && (float)req.glovez<0.6){
//		if(stop_flag < 5){
//			stop_flag++;
//			return true;
//		}
//		else if(stop_flag==5){
//
//			stop_flag++;
//			ROS_INFO("HAND GESTURE");
//
//			//		thread thd(call_iar,1);
//			//		thd.detach();
//
//		}
//	}
//	else{
//		stop_flag = 0;
//	}


	return true;
}

int main(int argc, char **argv)
{

	myfile.open("features.arff");
	if (myfile.is_open())
	{
	myfile << "format: hand velocity, hand to nearest package distance, rostime sec:msec\n\n";
	}
	else cout << "Unable to open file";


	ros::init(argc, argv, "feature_processer");
	ros::NodeHandle n;

	ros::ServiceClient client_ = n.serviceClient<object_tracking::InformActionRecognized>("/observation_agent/inform_action_recognized");
	clientPtr = &client_;

	ros::ServiceServer service = n.advertiseService("feature_processing", feat_print);
	ROS_INFO("Ready to process features.");
	ros::spin();

	return 0;
}

