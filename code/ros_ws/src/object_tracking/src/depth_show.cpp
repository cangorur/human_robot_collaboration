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
#include <stdio.h>

using namespace cv;
using namespace std;

const std::string RECEIVE_IMG_TOPIC_NAME = "/camera/depth/image";


//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

cv::Mat depth_img;
VideoWriter cvideo("depth_outcpp.avi",VideoWriter::fourcc('M','J','P','G'),30, Size(FRAME_WIDTH,FRAME_HEIGHT),true);



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{


	try
	{
		// Cast the image to opencv format.
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "32FC1");
		cv::Mat img = cv_ptr->image;
		if(!img.empty())
		{
			depth_img = img;

			cv::imshow("depth image", depth_img);
			double min, max;
			cv::minMaxLoc(depth_img, &min, &max);
			std::cout << "min: " << min << " max: " << max <<std::endl;
			std::cout<<depth_img.rows<<"   "<< depth_img.cols<<std::endl;
			cv::Mat m2(depth_img.rows, depth_img.cols, CV_8UC3, depth_img.data);

	        cv::Mat normalized;
	        cv_ptr->image.convertTo(normalized, CV_32FC1, 1.0/max, 0)  ;
	        imshow("normalized",normalized);

			float z;
			z = depth_img.at<float>(Point(400,330));
			ROS_INFO("%f",z);

			float max_depth = (float)max;
//			for(int y = 0; y < depth_img.rows; y++) {
//				for(int x = 0; x < depth_img.cols; x++) {
//					float ddistance = depth_img.at<float>(y, x);
//					if (ddistance == ddistance) { // exclude NaN
//						max_depth = max(ddistance, max_depth);
//					}
//				}
//			}
			Mat image = cv::Mat(depth_img.rows, depth_img.cols, CV_8UC3);

			for(int y = 0; y < depth_img.rows; y++) {
				for(int x = 0; x < depth_img.cols; x++) {
					float ddistance = depth_img.at<float>(y, x);
					unsigned int dist_clr = (unsigned int)(ddistance / max_depth * 255);
					image.at<cv::Vec3b>(y, x) = cv::Vec3b(dist_clr, dist_clr, dist_clr);
				}
			}

			cvideo.write(image);
			waitKey(1);
		}
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "depth_show");
  	ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(RECEIVE_IMG_TOPIC_NAME, 1, imageCallback);
  	ros::spin();
 	return 0;
}
