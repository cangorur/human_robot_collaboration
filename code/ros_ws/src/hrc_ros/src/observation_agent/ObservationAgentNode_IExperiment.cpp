/*
 * HumanAgentNode.cpp
 *
 *  Created on: 18.04.2018
 *      Author: Orhan Can Görür
 */

#include <ros/ros.h>
#include <observation_agent/ObservationAgent_IExperiment.h>

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "observation_agent_IE");
	ros::NodeHandle nh;
	
	ObservationAgent observation_agent;
	
  	ROS_INFO("Observation Agent is ready !");
	
  	
}