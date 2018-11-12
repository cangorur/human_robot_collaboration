#include "ros/ros.h"
#include <hrc_ros/TrayUpdateCamera.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_decision_trigger");
  ROS_INFO(" currently only triggers decision once ");
  ros::NodeHandle nh; 
 
  ros::ServiceClient client = n.serviceClient<hrc_ros::InformTrayUpdate>("/observation_agent/inform_tray_update");

  // compiling the message for the service
  hrc_ros::InformTrayUpdate srv; 
  srv.request.message = " "; 

  if (client.call(srv))
  {
    ROS_INFO(" Service call successfull  --- inform_tray_update");
  }
  else
  {
    ROS_ERROR("Failed to call service    --- inform_tray_update");
    return 1;
  }

  return 0;

  //ros::spin()  // if the node should run continuously -> also remove the return 0 then 
}
