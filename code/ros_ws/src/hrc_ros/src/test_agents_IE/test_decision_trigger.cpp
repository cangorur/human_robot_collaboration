#include "ros/ros.h"
//#include <hrc_ros/TrayUpdateCamera.h>
#include <cstdlib>
#include <hrc_ros/InformTrayUpdate.h>
#include <hrc_ros/InformActionRecognized.h>
#include <hrc_ros/InformHumanState.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_decision_trigger");
  ROS_INFO(" currently only triggers decision once ");
  ros::NodeHandle nh; 
 
  // service client to inform the real human state 
  ros::ServiceClient human_state_client = nh.serviceClient<hrc_ros::InformHumanState>("/observation_agent/inform_new_human_state");

  // compiling the message for the real human state service 

  hrc_ros::InformHumanState human_state_srv; 
  human_state_srv.request.new_human_state = "TaskHuman";



  // service client to inform tray update 
  ros::ServiceClient client = nh.serviceClient<hrc_ros::InformTrayUpdate>("/observation_agent/inform_tray_update");

  // compiling the message for the service
  hrc_ros::InformTrayUpdate srv; 
  //srv.request.tray_obj_combination =  12; 
  srv.request.current_tray = 1;
  srv.request.current_object = 1; 
 

 


 // ++++++++++  service client to inform action recognized 
  ros::ServiceClient action_client = nh.serviceClient<hrc_ros::InformActionRecognized>("/observation_agent/inform_action_recognized");

  // compiling the message for the service
  hrc_ros::InformActionRecognized srv2;
  srv2.request.action = "warning";
  srv2.request.human_detected = true; 
  srv2.request.human_looking_around = false;
  
 while(1){

   // inform about tray update 
        srv.response.success = client.call(srv);
        ROS_INFO("Response is: %d\n",srv.response.success);
        if (srv.response.success)
        {
          ROS_INFO(" Service call successfull  --- inform_tray_update");
        }
        else
        {
          ROS_ERROR("Failed to call service    --- inform_tray_update");
          return 1;
        }

        ROS_INFO(" +++ Service call successfull  --- inform_tray_update \n\n ");


        ros::Duration(5.0).sleep();

// ******************  Inform about human action 

        srv2.response.success = action_client.call(srv2);
        ROS_INFO("Response is: %d\n",srv2.response.success);
        if (srv2.response.success)
        {
          ROS_INFO(" Service call successfull  --- inform_action_recognized");
        }
        else
        {
          ROS_ERROR("Failed to call service    --- inform_action_recognized");
          return 1;
        }

        ROS_INFO(" +++ Service call successfull  --- inform_action_recognized\n\n");

        ros::Duration(5.0).sleep();


    //***********************  inform about actual human state ***************************  

        
     /*   human_state_srv.response.success = human_state_client.call(human_state_srv); 
        
        ROS_INFO("Response is: %d\n",human_state_srv.response.success);
        if (human_state_srv.response.success)
        {
          ROS_INFO(" Service call successfull  --- inform_human_state");
        }
        else
        {
          ROS_ERROR("Failed to call service    --- inform_human_state");
          return 1;
        }

        ROS_INFO(" +++ Service call successfull  --- inform_human_state\n\n");

        ros::Duration(5.0).sleep();  
        */


 }



  ros::spin();  // if the node should run continuously -> also remove the return 0 then 
}
