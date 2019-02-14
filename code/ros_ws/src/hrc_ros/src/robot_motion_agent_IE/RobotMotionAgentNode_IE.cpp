/*
 * HumanAgentNode.cpp
 *
 *  Created on: 18.04.2018
 *      Author: Orhan Can Görür
 */

#include <ros/ros.h>
#include <robot_motion_agent_IE/RobotMotionAgent_IE.h>

int main(int argc, char **argv) {

	ros::init(argc, argv, "robot_motion_agent");
	ros::NodeHandle nh;

	RobotMotionAgent robot_motion_agent;

  	ROS_INFO("Robot Motion Agent is ready - Dobot can be controlled now !");

  	// ros::spin(); we cannot spin here as it opens a websocket server and runs on that thread
  	// spin is carried out under Update function of HumanAgent.cpp to spinOnce after certain operation
}
