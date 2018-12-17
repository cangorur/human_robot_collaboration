#include "ros/ros.h"
#include <ros/package.h>
#include "object_tracking/NineD.h"
#include <iostream>
#include <fstream>


using namespace std;

//ros::ServiceClient *clientPtr; //pointer for a client
//ros::ServiceClient *clientPtrCont; //pointer for a client

vector<int> rules{0,0,0};

int c = 0;
int fail_counter = 0;
int consec_fails = 0;

void load_rules(){

	string pkg_path = ros::package::getPath("object_tracking");
    ifstream file(pkg_path+"/settings/rules.txt");
    string line;
    while (getline(file, line)) {

        istringstream sin(line.substr(line.find("=") + 1));

        if(line.find("red")!=-1){
        	sin>>rules[0];
        	cout <<"red rule loaded"<<endl;
        	cout<<rules[0]<<endl<<endl;
        }
        else if(line.find("green")!=-1){
        	sin>>rules[1];
        	cout <<"green rule loaded"<<endl;
        	cout<<rules[1]<<endl<<endl;
        }
        else if(line.find("blue")!=-1){
        	sin>>rules[2];
        	cout <<"blue rule loaded"<<endl;
        	cout<<rules[2]<<endl<<endl;
        }
        else if(line.find("end")!=-1){
        	cout<<"load end"<<endl<<endl;
        	return;
        }
    }
}

void Callback(const object_tracking::NineD::ConstPtr& msg)
{
	ROS_INFO("\n9d vector: %d %d %d %d %d %d %d %d %d   count: %d", msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7], msg->data[8],c);
  	ROS_INFO("\n%d.%d", (int)msg->stamp.sec, (int)msg->stamp.nsec);
  	c++;

  	int tray,package = 0;
  	for(int i = 0; i<msg->data.size();i++){
  		tray = i/3;
  		package = i%3;
  		if(msg->data[i]==1){
  			if(rules[package]==tray){
  				consec_fails = 0;
  				ROS_INFO("CORRECT package placement");
  			}
  			else{
  				consec_fails++;
  				fail_counter ++;
  				ROS_INFO("INCORRECT package placement total#%d consec#%d",fail_counter, consec_fails);
  			}
  		}
  	}
  	if(fail_counter == 3||consec_fails==2){
  		consec_fails == 2;
  		fail_counter = 0;
		ROS_INFO("ROBOT CALL TO ACT  ");

//  		fail_counter++;
 //			consec_fails == 2;
//		dobot::OneTimePickAndPlace srvpap;
//		srvpap.request.isCountEnabled = true;
//		dobot::ContPickAndPlace srvpap;
//		srvpap.request.isEnabled = true;
//
//		srvpap.request.isLocConfigEnabled = true;
////		redplace:
////		x: 244.503356934
////		y: -206.105560303
////		z: -35.85131073
////		pick:
////		x: 269.80078125
////		y: 48.5019340515
////		z: 12.9406280518
////
////		srvpap.request.pickX = 266.30;
////		srvpap.request.pickY = -45.46;
////		srvpap.request.pickZ = 12.20;
////		srvpap.request.placeX = 255.89;
////		srvpap.request.placeY = 197.92;
////		srvpap.request.placeZ = -38.49;
//		srvpap.request.pickX = 272.94;
//		srvpap.request.pickY = 58.89;
//		srvpap.request.pickZ = 13.11;
//		srvpap.request.placeX = 244.50;
//		srvpap.request.placeY = -206.10;
//		srvpap.request.placeZ = -35.85;
//
//
//		srvpap.request.placeInt = 90.0;
//		srvpap.request.placeIntZ = 30;
//		srvpap.request.placeLocR = 0;
//		srvpap.request.placeLocG = 1;
//		srvpap.request.placeLocB = 2;
//		srvpap.request.boxLimR = 3;
//		srvpap.request.boxLimG = 3;
//		srvpap.request.boxLimB = 3;
//		srvpap.request.storageLim = 3;
//
//		//		    if(clientPtr->call(srvpap)) //request service from the client
//					if(clientPtrCont->call(srvpap)) //request service from the client
//  	    {
//  	        ROS_INFO("Success service call");
//  	    }
//  	    else
//  	    {
//  	        ROS_ERROR("Fail call service");
//  	    }
  	}

}

int main(int argc, char **argv)
{
	load_rules();

	ros::init(argc, argv, "nined_receiver");
	
	ros::NodeHandle n;

//	ros::ServiceClient client = n.serviceClient<dobot::OneTimePickAndPlace>("/dobot_arm_app/oneTimePickAndPlace");
//	ros::ServiceClient client_ = n.serviceClient<dobot::ContPickAndPlace>("/dobot_arm_app/contPickAndPlace");
//	clientPtr = &client;
//	clientPtrCont = &client_;

	ros::Subscriber sub = n.subscribe("/camera_agent/tray_detection9D", 1000, Callback);


	ROS_INFO("Ready to get container information.");
	ros::spin();

	return 0;
}
