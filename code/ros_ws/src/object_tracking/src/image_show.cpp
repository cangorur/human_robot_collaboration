#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <fstream>
#include <string> 
#include <sstream>

using namespace cv;

const std::string RECEIVE_IMG_TOPIC_NAME = "/camera/rgb/image_raw";

cv::Mat current_image;


void imageCallback(const sensor_msgs::ImageConstPtr& msg) 
{

	
	ROS_INFO("in imageCALLBACK");
    try {
		// Cast the image to opencv format.
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		cv::Mat img = cv_ptr->image;
		if(!img.empty()) 
		{
			current_image = img;
		cv::imshow("rgb image", current_image);
		waitKey(1);
		}
    } 
	catch (cv_bridge::Exception& e) 
	{
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "im_show");
  	ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(RECEIVE_IMG_TOPIC_NAME, 1, imageCallback);
  	ros::spin();
 	return 0;
}
