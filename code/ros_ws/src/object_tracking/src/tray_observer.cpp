#include "ros/ros.h"
#include <ros/package.h>
#include "object_tracking/TrayUpdateCamera.h"
#include <iostream>
#include <fstream>
#include <thread>


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

//void call_pap(){
//	ROS_INFO("CALLING PICK AND PLACE SERVICE");
//
//
////	dobot::OneTimePickAndPlace srvpap;
////	srvpap.request.isCountEnabled = true;
//
//	dobot::ContPickAndPlace srvpap;
//	srvpap.request.isEnabled = true;
//
//	srvpap.request.isLocConfigEnabled = false;
//
//	srvpap.request.pickX = 272.94;
//	srvpap.request.pickY = 58.89;
//	srvpap.request.pickZ = 13.11;
//	srvpap.request.placeX = 244.50;
//	srvpap.request.placeY = -206.10;
//	srvpap.request.placeZ = -35.85;
//
//
//	srvpap.request.placeInt = 90.0;
//	srvpap.request.placeIntZ = 30;
//	srvpap.request.placeLocR = 0;
//	srvpap.request.placeLocG = 1;
//	srvpap.request.placeLocB = 2;
//	srvpap.request.boxLimR = 3;
//	srvpap.request.boxLimG = 3;
//	srvpap.request.boxLimB = 3;
//	srvpap.request.storageLim = 3;
//
////	if(clientPtr->call(srvpap)) //request service from the client
//	if(clientPtrCont->call(srvpap)) //request service from the client
//    {
//        ROS_INFO("Success service call");
//    }
//    else
//    {
//        ROS_ERROR("Fail call service");
//    }
//}



bool tray_print(object_tracking::TrayUpdateCamera::Request  &req,
		object_tracking::TrayUpdateCamera::Response &res)
{

	res.sth = 1;
	
	ROS_INFO("\n9d vector: %d %d %d %d %d %d %d %d %d   count: %d", req.data[0], req.data[1], req.data[2], req.data[3], req.data[4], req.data[5], req.data[6], req.data[7], req.data[8],c);
	ROS_INFO("\n%d.%d", (int)req.stamp.sec, (int)req.stamp.nsec);
	c++;
	
	int tray,package = 0;
	for(int i = 0; i<req.data.size();i++){
		tray = i/3;
		package = i%3;
		if(req.data[i]==1){
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
  		consec_fails = 0;
  		fail_counter = 0;
		ROS_INFO("ROBOT CALL TO ACT  ");

//  		fail_counter++;
//			consec_fails == 2;
//		thread thd(call_pap);
//		thd.detach();
	}

	return true;	
}

int main(int argc, char **argv)
{
	load_rules();

	ros::init(argc, argv, "tray_observer");
	ros::NodeHandle n;


//	ros::ServiceClient client = n.serviceClient<dobot::OneTimePickAndPlace>("/dobot_arm_app/oneTimePickAndPlace");
//	ros::ServiceClient client_ = n.serviceClient<dobot::ContPickAndPlace>("/dobot_arm_app/contPickAndPlace");
//	clientPtr = &client;
//	clientPtrCont = &client_;

	ros::ServiceServer service = n.advertiseService("/observation_agent/inform_tray_update_old", tray_print);


	ROS_INFO("Ready to print tray information.");
	ros::spin();

	return 0;
}
