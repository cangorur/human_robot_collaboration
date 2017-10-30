/*
 * HRCTaskManagerNode.cpp
 *
 *  Created on: 02.09.2017
 *      Author: Orhan Can Görür
 */

#include <ros/ros.h>
#include <hrc_task_manager/HRCTaskManager.h>

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "hrc_task_manager");
	ros::NodeHandle nh;
	
	HRCTaskManager hrc_task_manager;
	
  	ROS_INFO("Task Manager is ready!");
//	warehouseGateway.SetWebSocketServer();
	
  	ros::spin(); //carried out into WareHouseGateway class, SetWebSocket function under its on_message() thread. So that whenever a message arrives ros spins and gets uptodate rostopic callbacks
}