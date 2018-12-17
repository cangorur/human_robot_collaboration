#include "ros/ros.h"
#include <ros/package.h>
#include "object_tracking/Subfeatures.h"
#include <iostream>
#include <fstream>

//#include "dobot/OneTimePickAndPlace.h"
//#include "dobot/ContPickAndPlace.h"



using namespace std;

ofstream myfile;

//ros::ServiceClient *clientPtr; //pointer for a client
//ros::ServiceClient *clientPtrCont; //pointer for a client

int stop_flag = 0;
bool toggle = false;


void Callback(const object_tracking::Subfeatures::ConstPtr& msg)
{

	ROS_INFO("request: velo=%f, dist=%f", (float)msg->velocity, (float)msg->distance);
	ROS_INFO("request: glove z=%f", (float)msg->glovez);
	ROS_INFO("request: time sec:%d nsec:%d", (int)msg->rostime.sec, (int)msg->rostime.nsec);

	if (myfile.is_open()){

		myfile <<(float)msg->velocity<<",";
		myfile <<(float)msg->distance<<",";
		myfile <<(int)msg->rostime.sec<<":"<<(int)msg->rostime.nsec;
		myfile<<endl;
	}
	else cout << "Unable to open file";



	if((float)msg->glovez>0 && (float)msg->glovez<0.6){
		if(stop_flag < 5){
			stop_flag++;
			return;
		}
		else if(stop_flag==5){
			ROS_INFO("HAND GESTURE");

			stop_flag++;
			toggle = !toggle;
			toggle = false;
//			dobot::OneTimePickAndPlace srvpap;
//			srvpap.request.isCountEnabled = true;
//			dobot::ContPickAndPlace srvpap;
//			if(toggle){
//				srvpap.request.isEnabled = true;
//			}
//			else{
//				srvpap.request.isEnabled = false;
//			}
////			srvpap.request.isEnabled = true;
//
//			srvpap.request.isLocConfigEnabled = false;
//
////			redplace:
////			x: 244.503356934
////			y: -206.105560303
////			z: -35.85131073
////			pick:
////			x: 267.751495361
////			y: 58.1633338928
////			z: 11.4373397827
//
//
//
////			srvpap.request.pickX = 266.30;
////			srvpap.request.pickY = -45.46;
////			srvpap.request.pickZ = 12.20;
////			srvpap.request.placeX = 255.89;
////			srvpap.request.placeY = 197.92;
////			srvpap.request.placeZ = -38.49;
//			srvpap.request.pickX = 267.75;
//			srvpap.request.pickY = 58.16;
//			srvpap.request.pickZ = 11.43;
//			srvpap.request.placeX = 244.50;
//			srvpap.request.placeY = -206.10;
//			srvpap.request.placeZ = -35.85;
//
//			srvpap.request.placeInt = 90.0;
//			srvpap.request.placeIntZ = 30;
//			srvpap.request.placeLocR = 0;
//			srvpap.request.placeLocG = 1;
//			srvpap.request.placeLocB = 2;
//			srvpap.request.boxLimR = 3;
//			srvpap.request.boxLimG = 3;
//			srvpap.request.boxLimB = 3;
//			srvpap.request.storageLim = 3;
//
////			rosservice call /dobot_arm_app/contPickAndPlace "{isEnabled: true, isLocConfigEnabled: true, pickX: 272.94, pickY: 58.89, pickZ: 13.11, placeX: 244.50, placeY: -206.10, placeZ: -35.85, placeInt: 90.0, placeIntZ: 30.0, placeLocR: 0, placeLocG: 1, placeLocB: 2, boxLimR: 3, boxLimG: 3, boxLimB: 3, storageLim: 3}"
//
////		    if(clientPtr->call(srvpap)) //request service from the client
//			if(clientPtrCont->call(srvpap)) //request service from the client
//		    {
//		        ROS_INFO("Success service call");
//		    }
//		    else
//		    {
//		        ROS_ERROR("Fail call service");
//		    }
		}
	}
	else{
		stop_flag = 0;
	}

}

int main(int argc, char **argv)
{

	myfile.open("features_sub.arff");
	if (myfile.is_open())
	{
	myfile << "format: hand velocity, hand to nearest package distance, rostime sec:msec\n\n";
	}
	else cout << "Unable to open file";


	ros::init(argc, argv, "classify");
	ros::NodeHandle n;

//	ros::ServiceClient client = n.serviceClient<dobot::OneTimePickAndPlace>("/dobot_arm_app/oneTimePickAndPlace");
//	ros::ServiceClient client_ = n.serviceClient<dobot::ContPickAndPlace>("/dobot_arm_app/contPickAndPlace");
//	clientPtr = &client;
//	clientPtrCont = &client_;

	ros::Subscriber sub = n.subscribe("/camera_agent/motion_features", 1000, Callback);

	ROS_INFO("Ready to process features.");
	ros::spin();

	return 0;
}
