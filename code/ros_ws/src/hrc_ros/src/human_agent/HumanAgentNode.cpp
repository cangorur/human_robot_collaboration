/*
 * HumanAgentNode.cpp
 *
 *  Created on: 18.04.2018
 *      Author: Orhan Can Görür
 */

#include <ros/ros.h>
#include <human_agent/HumanAgent.h>

int main(int argc, char **argv) {

	ros::init(argc, argv, "human_agent");
	ros::NodeHandle nh;

	HumanAgent human_agent;

  	ROS_INFO("Human Agent is ready !");

  	// ros::spin(); we cannot spin here as it opens a websocket server and runs on that thread
  	// spin is carried out under Update function of HumanAgent.cpp to spinOnce after certain operation
}
