/*
 * HumanAgentNode.cpp
 *
 *  Created on: 18.04.2018
 *      Author: Orhan Can Görür
 */

#include <ros/ros.h>
#include <observation_agent/ObservationAgent.h>

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "observation_agent");
	ros::NodeHandle nh;
	
	ObservationAgent observation_agent;
	
  	ROS_INFO("Observation Agent is ready !");
	
  	
}