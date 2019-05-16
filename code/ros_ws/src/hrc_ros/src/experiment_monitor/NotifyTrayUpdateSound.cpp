/*
 *  Created on: 16.05.2019
 *  
 * 			Author: Elia Kargruber and Orhan Can Görür
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <stdlib.h> 				// for rand() and RAND_MAX
#include <string>
#include <thread>

#include <notifyTrayUpdate/notifyTrayUpdate.h>


using namespace std;

TrayNotifier::TrayNotifier() {
	ros::NodeHandle nh("~");
	initialize();
}

TrayNotifier::~TrayNotifier() {
}

void TrayNotifier::initialize(){
	ros::NodeHandle nh("~");

	/*
	 *Ros Publishers and Subscriber initialization
	 *
	 */
	// subscribe to a tray_update message published by observation_agent
	//tray_update_subs = nh.subscribe("/observation_agent/observedsuccess_status", 1,&TrayNotifier::receive_tray_update,this);
	tray_update_subs = nh.subscribe("/observation_agent/tray_status_NotifyToHuman", 1,&TrayNotifier::receive_tray_update,this);
	ROS_INFO("[TrayNotifier] is created !");
	ros::spin();
}


void TrayNotifier::receive_tray_update(const hrc_ros::SuccessStatusObserved & msg){
	TrayNotifier::sc.play(1,1.0); // sound, volume (max 1.0)
}

