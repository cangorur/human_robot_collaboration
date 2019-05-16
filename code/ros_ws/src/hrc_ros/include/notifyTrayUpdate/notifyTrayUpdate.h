/*
 *  ObservationAgent.cpp
 *
 *      Created on: 16.05.2019
 *      Author: Elia Kargruber
 *   
 */

#ifndef HRC_ROS_SRC_NOTIFYTRAYUPDATE_H
#define HRC_ROS_SRC_NOTIFYTRAYUPDATE_H

#include <ros/ros.h>
// include <package_name/service_type_name.h>
#include <std_srvs/Trigger.h>
#include <hrc_ros/SuccessStatusObserved.h>
#include <sound_play/sound_play.h>


using namespace std;



class TrayNotifier {
public:
	TrayNotifier();
	virtual ~TrayNotifier();

private:


	/**
	 * Initialize all the ROS services/clients/topics and parameters.
	 * @todo in the future can be a ROS service initialized by the task manager
	 */
	void initialize();

    ros::Subscriber tray_update_subs;

    void receive_tray_update(const hrc_ros::SuccessStatusObserved &msg);

   

public: 
    sound_play::SoundClient sc;

};

#endif /* HRC_ROS_SRC_NOTIFYTRAYUPDATE_H */
