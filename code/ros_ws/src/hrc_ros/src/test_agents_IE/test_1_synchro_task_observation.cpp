#include "ros/ros.h"
#include <hrc_ros/TrayUpdateCamera.h>
#include <cstdlib>
#include <hrc_ros/InformTrayUpdate.h>
#include <hrc_ros/InformActionRecognized.h>
#include <hrc_ros/InformHumanState.h>
#include <hrc_ros/SuccessStatusObserved.h> 
#include <test_agents_IE/TestAgent.h>

using namespace std; 

TestAgent::TestAgent() {
    ros::NodeHandle nh("~");
    initialize();
}

TestAgent::~TestAgent() {
}

void TestAgent::initialize(){
  ros::NodeHandle nh("~");
  // subscriber
  sucessStatus_sub = nh.subscribe("/observation_agent/observedsuccess_status",1000,&TestAgent::ReceiveSuccessStatusObserved, this);
  // service client to inform tray update 
  tray_client = nh.serviceClient<hrc_ros::InformTrayUpdate>("/observation_agent/inform_tray_update");
  //hrc_ros::InformTrayUpdate tray_srv;
  action_client = nh.serviceClient<hrc_ros::InformActionRecognized>("/observation_agent/inform_action_recognized");
  //hrc_ros::InformActionRecognized action_srv;

  //issue_tray_update((1),2);
  cout << "Test agent initialized" << endl; 

}


// ########### Functions ####################################################
void TestAgent::ReceiveSuccessStatusObserved(const hrc_ros::SuccessStatusObserved &msg){
    //std::cout << " In callback SuccessStatusReceived" << endl << endl;
    subtask_success_status = msg.subtask_success_status;

    // ##### print debugging messages 
    cout << endl << "  ## Received success status ## " << endl; 
    cout << "  current_object: " << msg.current_object << endl << "  current_tray: " << msg.current_tray; 
    cout << "  success_tray  : " << msg.success_tray << endl << endl;
    cout << "  task_counter  : " << msg.task_counter << "  subtask_counter: " << msg.subtask_counter << endl; 
    cout << "  task_sucess_status : " << msg.task_success_status << endl << "  subtask_success_status: " << msg.subtask_success_status << endl;   


    success_received = true; 
  

    //issue_tray_update(2,3);
}


bool TestAgent::issue_tray_update(int current_object_int,int current_tray_int){

     // inform about tray update 

        // compose message to be sent
        tray_srv.request.current_object = current_object_int; 
        tray_srv.request.current_tray = current_tray_int; 

        tray_srv.response.success = tray_client.call(tray_srv);
        if (tray_srv.response.success)
        {
          ROS_INFO(" ++++ Service call successfull  --- inform_tray_update");
        }
        else
        {
          ROS_ERROR(" ++++ Failed to call service    --- inform_tray_update");
          return 1;
        }


}


  int current_object_int; 
  int current_tray_int; 
 
  
 // ++++++++++  service client to inform action recognized 


/*
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_1_synchronisation_task_observation");
  ROS_INFO(" ### test_1_synchronisation_task_observation ###");

  

  //Issue tray update -> simulate observation agent 
  current_object_int = 1; 
  current_tray_int = 2; 
  bool tray_update_done = TestAgent::issue_tray_update(current_object_int,current_tray_int);

  // wait until success_update has been published by observation_agent 
  while(success_received == false){}
  success_received = false; 

 
  // TODO check here if the result is as expected 

  



  ros::spin();  // if the node should run continuously -> also remove the return 0 then 
} 

*/
