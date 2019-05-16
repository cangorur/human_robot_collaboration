/*
 * HumanAgentNode.cpp
 *
 *  Created on: 18.04.2018
 *      Author: Orhan Can Görür
 */

#include <ros/ros.h>
#include <notifyTrayUpdate/notifyTrayUpdate.h>

int main(int argc, char **argv) {

	ros::init(argc, argv, "notify_tray_update_agent");
	ros::NodeHandle nh;

	TrayNotifier notify_tray_agent;

  	ROS_INFO("TrayNotifier is ready");


}
