/*
 * HumanAgentNode.cpp
 *
 *  Created on: 18.04.2018
 *      Author: Orhan Can Görür
 */

#include <ros/ros.h>
#include <robot_agent/RobotAgent.h>

int main(int argc, char **argv) {

	ros::init(argc, argv, "robot_agent");
	ros::NodeHandle nh;

	RobotAgent robot_agent;

  	ROS_INFO("Robot Agent is ready !");

  	// ros::spin(); we cannot spin here as it opens a websocket server and runs on that thread
  	// spin is carried out under Update function of HumanAgent.cpp to spinOnce after certain operation
}
