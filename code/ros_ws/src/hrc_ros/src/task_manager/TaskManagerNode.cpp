/*
 *  TaskManagerNode.cpp
 *
 *  Created on: 02.09.2017
 *  Modified on: 19.04.2018
 *  	Author: Orhan Can Görür
 */

#include <ros/ros.h>
#include <task_manager/TaskManager.h>

int main(int argc, char **argv) {

	ros::init(argc, argv, "task_manager");
	ros::NodeHandle nh;

	TaskManager task_manager;

  	ROS_INFO("Task Manager is ready!");

  	ros::spin();
}
