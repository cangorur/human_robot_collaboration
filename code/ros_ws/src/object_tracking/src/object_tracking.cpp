#include <ros/ros.h>
#include <ros/package.h>

#include "object_tracking/Feature.h"
#include "object_tracking/NineD.h"
#include "object_tracking/TrayUpdateCamera.h"
#include "hrc_ros/InformTrayUpdate.h"
#include "object_tracking/Subfeatures.h"

#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_ros/point_cloud.h>
//#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>


#include <fstream>
#include <string> 
#include <sstream>

//#include <zbar.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <stdio.h>
#include <ctime>
#include <cmath>
#include <thread>

//#include "dobot/OneTimePickAndPlace.h"


//#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
//#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

//using namespace sensor_msgs;
//using namespace message_filters;
using namespace std;
using namespace cv;
//using namespace zbar;


const string RGB_IMG_TOPIC_NAME = "/camera/rgb/image_raw";
const string DEPTH_TOPIC_NAME = "/camera/depth/image";
const bool rosbag = false; 		//if rosbag = true -> record videos!

//mats to save converted sens data in callbacks
cv::Mat current_image;
cv::Mat depth_img;

bool calib = false;
const bool load_from_file = true;

// maximum pixel distance in which an object can change locations per frame,
// otherwise skip and only do kalman predict for that object
const double err_dist = 0.12;

//hard coding colors to detect as HSV threshold, r_color, h_color etc. used to draw boundary box and contours




bool window_pos = true;

//red package
const Scalar g_HSVmin(152,112,126);
const Scalar g_HSVmax(205,256,256);
const Scalar g_color(0,0,255);
//blue package
const Scalar b_HSVmin(91,112,77);
const Scalar b_HSVmax(125,242,256);
const Scalar b_color(0,0,255);
//green package -> p1
const Scalar r_HSVmin(32,38,43);
const Scalar r_HSVmax(80,256,213);
const Scalar r_color(0,255,0);

//hand 1 (yellow)
Scalar h1_HSVmin(22,99,92);
Scalar h1_HSVmax(35,256,256);
Scalar h1_color(0,255,255);

//night red package
//(0,162,114);
//(16,256,256);

//
//night green package
//(32,120,46);
//(46,256,256);

//
//night blue package
//(93,143,47);
//(114,256,256);


//red green blue packages
vector<Scalar> pHSVmin{Scalar(0,172,126),Scalar(32,120,80),Scalar(93,143,47)};
vector<Scalar> pHSVmax{Scalar(17,256,256),Scalar(46,256,256),Scalar(114,256,256)};
vector<Scalar> pcolor{Scalar(0,0,255),Scalar(0,255,0),Scalar(255,0,0)};
Scalar nmov_min(130,69,164);
Scalar nmov_max(255,255,254);
Scalar conv_min(255,255,255);
Scalar conv_max(255,255,225);

//night red container (x3)
//(168,155,129);
//(256,207,256);
//new
//(164,75,104);
//(256,177,187);
//newer
//(130,101,83);
//(235,227,160);

//night green container
//(48,53,117);
//(80,119,195);
//night blue container (1.5)
//(79,77,159);
//(113,107,256);


//red green blue containers
vector<Scalar> bHSVmin{Scalar(130,101,83),Scalar(48,53,117),Scalar(79,38,129)};
vector<Scalar> bHSVmax{Scalar(235,227,176),Scalar(80,119,195),Scalar(113,107,256)};
vector<Scalar> bcolor{Scalar(0,0,255),Scalar(0,255,0),Scalar(255,0,0)};

/* not moving v1
(130,69,164);
(256,256,255);

not moving v2
(116,153,146);
(256,256,256);

moving
(0,162,132);
(16,256,256);
*/

int pamount = 3; //(1-3 packages to detect)
const int kalman_frames = 4; //only predict up to 15frames in consec, if more than 15frames reset kalman

//box1 blue
const Scalar b1_HSVmin(91,112,77);
const Scalar b1_HSVmax(125,242,256);
const Scalar b1_color(255,0,0);
//box2 green
const Scalar b2_HSVmin(72,146,185);
const Scalar b2_HSVmax(124,216,256);
const Scalar b2_color(255,0,0);
//box3 red
const Scalar b3_HSVmin(72,146,185);
const Scalar b3_HSVmax(124,216,256);
const Scalar b3_color(255,0,0);

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
int velocity_threshold = 100;  // pixel per vec_size frames , ie 25
int velocity_threshold_depth = 10000;  // meter/100000 per vec_size frames, ie 6000
int h_distance_threshold = 200;  // pixel amount for obj in hand, ie 150
int a_distance_threshold = 300;  // pixel amount for obj acted on




double second_c;
int frame_c = 0;
double ticks = getTickCount();
vector<KalmanFilter> KF;		//kalman filter for each object: p,h1,h2

vector<Point> ploc ,p_avg;     //location of package and hand, and moving averages of both locations
vector<float> ploc_z ,p_avg_z;
vector<Point3f> ploc_xyz, p_avg_xyz;
vector<Point> h1loc, h2loc, h1_avg, h2_avg;
vector<float> h1loc_z, h2loc_z, h1_avg_z, h2_avg_z;
vector<Point3f> h1loc_xyz, h2loc_xyz;//, h1_avg_xyz, h2_avg_xyz;
vector< vector<Point> > box1, box2, box3;	//contours of containers: success, fail, conveyor belt

vector< vector<Point> > bcontours{{},{},{}};	//contours of containers: success, fail, conveyor belt
vector<Point3f> box_xyz{{},{},{}};

vector< vector<Point> > ccontours{{},{},{}};	//contours of containers: success, fail, conveyor belt
Point3f conv_xyz;

const int vec_size = 10;
int fail_counter = 0;


//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string windowName8 = "blured after treshholding";
const string trackbarWindowName = "Trackbars";
void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed
}

Mat frame;
int vframe = 0;
int tframes = 0;
int dframes = 0;
int fps = 0;
double velocity1 = 0;
double velocity2 = 0;
double distanc = 0;
vector<bool> p1{false,false,false};
vector<bool> b1{false,false,false};
bool conv1 = false;
//bool p1 = false;
//bool p2 = false;
//bool p3 = false;
bool hand1 = false;
//bool b1 = false;
//bool b2 = false;
//bool b3 = false;
bool record = false;
bool idle = false;
bool grasp = false;
bool gesture = false;


//    std::time_t timeBegin = std::time(0);
//    int frameCounter = 0;
//    int tick = 0;

//	cameraFeed = imread("codeprestige.jpg");
//	  if( argc != 2 || !cameraFeed.data )
//	    {
//	      printf( "No image data \n" );
//	      return -1;
//	    }
//matrix storages
Mat cameraFeed, final;
Mat HSV;
Mat thresholdm;
Mat canny;
Mat blured;
Mat drawing;
//Mat kalm;

ofstream myfile;
//video capture object to acquire webcam feed
string path = ros::package::getPath("object_tracking");
VideoWriter video(path+"/records/outcpp.avi",VideoWriter::fourcc('M','J','P','G'),30, Size(FRAME_WIDTH,FRAME_HEIGHT),true);
VideoWriter cvideo(path+"/records/clean_outcpp.avi",VideoWriter::fourcc('M','J','P','G'),30, Size(FRAME_WIDTH,FRAME_HEIGHT),true);
//fps = capture.get(CAP_PROP_FPS);
// Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file.
//video = VideoWriter("outcpp.avi",VideoWriter::fourcc('M','J','P','G'),30, Size(FRAME_WIDTH,FRAME_HEIGHT),true);
//cvideo = VideoWriter("clean_outcpp.avi",VideoWriter::fourcc('M','J','P','G'),30, Size(FRAME_WIDTH,FRAME_HEIGHT),true);

string intToString(int number){


	std::stringstream ss;
	ss << number;
	return ss.str();
}



vector<Point> measv,kalmanvec;

void drawCross(Point center, Scalar color, int d , Mat& kalm) {
	line( kalm, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 1, CV_AA, 0);
	line( kalm, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 1, CV_AA, 0);
}

float get_velocity(float x, float y){
	return sqrt(pow(x,2)+pow(y,2));
}

float get_velocity(float x, float y, float z){ //pythagoras
	return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
}

vector<vector<Point3f>> meas3,kalman3;
vector<vector<Point>> meas2,kalman2;
vector<vector<float>>	kalmanv;


//counters to let kalman predict without measurement, after the counts dont predict anymore
int pmiss_c = 11;
vector<int> pmiss{11,11,11};
int h1miss_c = 11;
int q = 0;
int w = 0;
vector<int> s_flag{0,0,0,0};
bool calib_mov = false;

void load_settings(){

	string pkg_path = ros::package::getPath("object_tracking");
    ifstream file(pkg_path+"/settings/settings.txt");
    string line;
    string part;
    int p = 0;
    int eins, zwei,drei;
    while (getline(file, line)) {

        istringstream sin(line.substr(line.find("=") + 1));

        if (line.find("calib") != -1){
			if (line.find("true") != -1)calib=true;
			else if (line.find("false") != -1)calib=false;

			cout<<"load calib = "<<calib<<endl<<endl;
        }
        else if(line.find("record")!=-1){
			if (line.find("true") != -1)record=true;
			else if (line.find("false") != -1)record=false;

			cout<<"load record = "<<record<<endl<<endl;
        }
        else if(line.find("red")!=-1){
        	sin>>nmov_min.val[0]>>nmov_min.val[1]>>nmov_min.val[2];
        	sin>>nmov_max.val[0]>>nmov_max.val[1]>>nmov_max.val[2];
        	cout <<"not moving red loaded"<<endl;
        	cout<<nmov_min.val[0]<<" "<<nmov_min.val[1]<<" "<<nmov_min.val[2]<<endl;
        	cout<<nmov_max.val[0]<<" "<<nmov_max.val[1]<<" "<<nmov_max.val[2]<<endl<<endl;
        }
        else if(line.find("conveyor")!=-1){
        	sin>>conv_min.val[0]>>conv_min.val[1]>>conv_min.val[2];
        	sin>>conv_max.val[0]>>conv_max.val[1]>>conv_max.val[2];
        	cout <<"conveyor loaded"<<endl;
        	cout<<conv_min.val[0]<<" "<<conv_min.val[1]<<" "<<conv_min.val[2]<<endl;
        	cout<<conv_max.val[0]<<" "<<conv_max.val[1]<<" "<<conv_max.val[2]<<endl<<endl;
        }
        else if(line.find("conv_xyz")!=-1){
        	sin>>conv_xyz.x>>conv_xyz.y>>conv_xyz.z;
        	cout <<"reference point loaded"<<endl;
        	cout<<conv_xyz<<endl<<endl;
        	conv1 = true;
        }
        else if(line.find("p1")!=-1){
        	p=0;
        	sin>>pHSVmin[p].val[0]>>pHSVmin[p].val[1]>>pHSVmin[p].val[2];
        	sin>>pHSVmax[p].val[0]>>pHSVmax[p].val[1]>>pHSVmax[p].val[2];
        	cout <<"p"<<p+1<<" loaded"<<endl;
        	cout<<pHSVmin[p].val[0]<<" "<<pHSVmin[p].val[1]<<" "<<pHSVmin[p].val[2]<<endl;
        	cout<<pHSVmax[p].val[0]<<" "<<pHSVmax[p].val[1]<<" "<<pHSVmax[p].val[2]<<endl<<endl;
        }
        else if(line.find("p2")!=-1){
        	p=1;
        	sin>>pHSVmin[p].val[0]>>pHSVmin[p].val[1]>>pHSVmin[p].val[2];
        	sin>>pHSVmax[p].val[0]>>pHSVmax[p].val[1]>>pHSVmax[p].val[2];
        	cout <<"p"<<p+1<<" loaded"<<endl;
        	cout<<pHSVmin[p].val[0]<<" "<<pHSVmin[p].val[1]<<" "<<pHSVmin[p].val[2]<<endl;
        	cout<<pHSVmax[p].val[0]<<" "<<pHSVmax[p].val[1]<<" "<<pHSVmax[p].val[2]<<endl<<endl;
        }
        else if(line.find("p3")!=-1){
        	p=2	;
        	sin>>pHSVmin[p].val[0]>>pHSVmin[p].val[1]>>pHSVmin[p].val[2];
        	sin>>pHSVmax[p].val[0]>>pHSVmax[p].val[1]>>pHSVmax[p].val[2];
        	cout <<"p"<<p+1<<" loaded"<<endl;
        	cout<<pHSVmin[p].val[0]<<" "<<pHSVmin[p].val[1]<<" "<<pHSVmin[p].val[2]<<endl;
        	cout<<pHSVmax[p].val[0]<<" "<<pHSVmax[p].val[1]<<" "<<pHSVmax[p].val[2]<<endl<<endl;
        }
        else if(line.find("b1")!=-1){
        	p=0;
        	sin>>bHSVmin[p].val[0]>>bHSVmin[p].val[1]>>bHSVmin[p].val[2];
        	sin>>bHSVmax[p].val[0]>>bHSVmax[p].val[1]>>bHSVmax[p].val[2];
        	cout <<"b"<<p+1<<" loaded"<<endl;
        	cout<<bHSVmin[p].val[0]<<" "<<bHSVmin[p].val[1]<<" "<<bHSVmin[p].val[2]<<endl;
        	cout<<bHSVmax[p].val[0]<<" "<<bHSVmax[p].val[1]<<" "<<bHSVmax[p].val[2]<<endl<<endl;
        }
        else if(line.find("b2")!=-1){
        	p=1;
        	sin>>bHSVmin[p].val[0]>>bHSVmin[p].val[1]>>bHSVmin[p].val[2];
        	sin>>bHSVmax[p].val[0]>>bHSVmax[p].val[1]>>bHSVmax[p].val[2];
        	cout <<"b"<<p+1<<" loaded"<<endl;
        	cout<<bHSVmin[p].val[0]<<" "<<bHSVmin[p].val[1]<<" "<<bHSVmin[p].val[2]<<endl;
        	cout<<bHSVmax[p].val[0]<<" "<<bHSVmax[p].val[1]<<" "<<bHSVmax[p].val[2]<<endl<<endl;
        }
        else if(line.find("b3")!=-1){
        	p=2;
        	sin>>bHSVmin[p].val[0]>>bHSVmin[p].val[1]>>bHSVmin[p].val[2];
        	sin>>bHSVmax[p].val[0]>>bHSVmax[p].val[1]>>bHSVmax[p].val[2];
        	cout <<"b"<<p+1<<" loaded"<<endl;
        	cout<<bHSVmin[p].val[0]<<" "<<bHSVmin[p].val[1]<<" "<<bHSVmin[p].val[2]<<endl;
        	cout<<bHSVmax[p].val[0]<<" "<<bHSVmax[p].val[1]<<" "<<bHSVmax[p].val[2]<<endl<<endl;
        }
        else if(line.find("hand")!=-1){
        	sin>>h1_HSVmin.val[0]>>h1_HSVmin.val[1]>>h1_HSVmin.val[2];
        	sin>>h1_HSVmax.val[0]>>h1_HSVmax.val[1]>>h1_HSVmax.val[2];
        	cout <<"hand loaded"<<endl;
        	cout<<h1_HSVmin.val[0]<<" "<<h1_HSVmin.val[1]<<" "<<h1_HSVmin.val[2]<<endl;
        	cout<<h1_HSVmax.val[0]<<" "<<h1_HSVmax.val[1]<<" "<<h1_HSVmax.val[2]<<endl<<endl;
        }
        else if(line.find("end")!=-1){
        	cout<<"load end"<<endl<<endl;
        	return;
        }
    }
}

vector<int> rules{0,0,0};

void load_rules(){

	string pkg_path = ros::package::getPath("object_tracking");
    ifstream file(pkg_path+"/settings/rules.txt");
    string line;
    while (getline(file, line)) {

        istringstream sin(line.substr(line.find("=") + 1));

        if(line.find("red")!=-1){
        	sin>>rules[0];
        	cout <<"red rule loaded"<<endl;
        	cout<<rules[0]<<endl<<endl;
        }
        else if(line.find("green")!=-1){
        	sin>>rules[1];
        	cout <<"green rule loaded"<<endl;
        	cout<<rules[1]<<endl<<endl;
        }
        else if(line.find("blue")!=-1){
        	sin>>rules[2];
        	cout <<"blue rule loaded"<<endl;
        	cout<<rules[2]<<endl<<endl;
        }
        else if(line.find("end")!=-1){
        	cout<<"load end"<<endl<<endl;
        	return;
        }
    }
}


void init_KF(int amount){

	KF.clear();
	for(int i = 0; i < amount;i++){
		KF.push_back(KalmanFilter(6,3,0));
		meas2.push_back({});
		kalman2.push_back({});
		meas3.push_back({});
		kalman3.push_back({});
		kalmanv.push_back({});
	}
//	for(int i = 0; i < KF.size();i++){
//		//transitionmatrix will get set in beginning of the callback with update_transitionmatrix()
////		KF[i].transitionMatrix = (Mat_<float>(6, 6) << 	1, 0, 0, dt, 0, 0,
////														0, 1, 0, 0, dt, 0,
////														0, 0, 1, 0, 0, dt,
////														0, 0, 0, 1, 0, 0,
////														0, 0, 0, 0, 1, 0,
////														0, 0, 0, 0, 0, 1);
//		KF[i].statePre.at<float>(0) = 1;
//		KF[i].statePre.at<float>(1) = 1;
//		KF[i].statePre.at<float>(2) = 1;
//		KF[i].statePre.at<float>(3) = 0;
//		KF[i].statePre.at<float>(4) = 0;
//		KF[i].statePre.at<float>(5) = 0;
//	//	KF[i].statePost.at<float>(0) = 1;
//	//	KF[i].statePost.at<float>(1) = 1;
//	//	KF[i].statePost.at<float>(2) = 1;
//	//	KF[i].statePost.at<float>(3) = 0;
//	//	KF[i].statePost.at<float>(4) = 0;
//	//	KF[i].statePost.at<float>(5) = 0;
//		setIdentity(KF[i].transitionMatrix);
//		setIdentity(KF[i].measurementMatrix);						//F
//		setIdentity(KF[i].processNoiseCov, Scalar::all(1e-2));		//Q
//		setIdentity(KF[i].measurementNoiseCov, Scalar::all(1e-1));	//R
//		setIdentity(KF[i].errorCovPost, Scalar::all(.1));
//	}

}

void reinit_KF(Point3f p, int id){

	KF[id].init(6,3,0);
//	KF[id].statePre.at<float>(0) = p.x;
//	KF[id].statePre.at<float>(1) = p.y;
//	KF[id].statePre.at<float>(2) = p.z;
//	KF[id].statePre.at<float>(3) = 0;
//	KF[id].statePre.at<float>(4) = 0;
//	KF[id].statePre.at<float>(5) = 0;
	KF[id].statePost.at<float>(0) = p.x;
	KF[id].statePost.at<float>(1) = p.y;
	KF[id].statePost.at<float>(2) = p.z;
	KF[id].statePost.at<float>(3) = 0;
	KF[id].statePost.at<float>(4) = 0;
	KF[id].statePost.at<float>(5) = 0;
	setIdentity(KF[id].transitionMatrix);
	setIdentity(KF[id].measurementMatrix);						//F
	setIdentity(KF[id].processNoiseCov, Scalar::all(1e-2));		//Q
	setIdentity(KF[id].measurementNoiseCov, Scalar::all(1e-3));	//R
	setIdentity(KF[id].errorCovPost, Scalar::all(.1));

}

void reinit_KF(Point p, int id){

	KF[id].init(6,3,0);
//	KF[id].statePre.at<float>(0) = p.x;
//	KF[id].statePre.at<float>(1) = p.y;
//	KF[id].statePre.at<float>(2) = p.z;
//	KF[id].statePre.at<float>(3) = 0;
//	KF[id].statePre.at<float>(4) = 0;
//	KF[id].statePre.at<float>(5) = 0;
	KF[id].statePost.at<float>(0) = p.x;
	KF[id].statePost.at<float>(1) = p.y;
	KF[id].statePost.at<float>(2) = 1;
	KF[id].statePost.at<float>(3) = 0;
	KF[id].statePost.at<float>(4) = 0;
	KF[id].statePost.at<float>(5) = 0;
	setIdentity(KF[id].transitionMatrix);
	setIdentity(KF[id].measurementMatrix);						//F
	setIdentity(KF[id].processNoiseCov, Scalar::all(1e-2));		//Q
	setIdentity(KF[id].measurementNoiseCov, Scalar::all(1e-3));	//R
	setIdentity(KF[id].errorCovPost, Scalar::all(.1));

}

void update_transitionMatrix(float dt){
	for(int i = 0; i < KF.size(); i++){
		KF[i].transitionMatrix = (Mat_<float>(6, 6) << 	1, 0, 0, dt, 0, 0,
														0, 1, 0, 0, dt, 0,
														0, 0, 1, 0, 0, dt,
														0, 0, 0, 1, 0, 0,
														0, 0, 0, 0, 1, 0,
														0, 0, 0, 0, 0, 1);
	}
}

void doKF(Point p, int id, int type){ //type = 0 just predict, type=1 correct



	Mat_<float> measurement(3,1); measurement.setTo(Scalar(0));
	Mat prediction = KF[id].predict();
	Point3f predictPt(prediction.at<float>(0),prediction.at<float>(1),prediction.at<float>(2));
	Point3f velo_vect(prediction.at<float>(3),prediction.at<float>(4),prediction.at<float>(5));
	measurement(0)=p.x;
	measurement(1)=p.y;
	measurement(2)=1;
//	cout<<"MEASURE "<<p.x<<"|"<<p.y<<"|"<<"1"<<endl;
//	cout<<"PREDICT "<<predictPt.x<<"|"<<predictPt.y<<"|"<<predictPt.z<<endl;
//	cout<<"VELOCTY "<<velo_vect.x<<"|"<<velo_vect.y<<"|>>>>>>"<<get_velocity(velo_vect.x,velo_vect.y)<<endl;

	if(type>0){
		Mat estimate = KF[id].correct(measurement);
		Point3f correctPt(estimate.at<float>(0),estimate.at<float>(1),estimate.at<float>(2));
		Point3f velo_corr(estimate.at<float>(3),estimate.at<float>(4),estimate.at<float>(5));
//		cout<<"CORRECT "<<correctPt.x<<"|"<<correctPt.y<<"|"<<correctPt.z<<endl;
//		cout<<"COR_VEL "<<velo_corr.x<<"|"<<velo_corr.y<<"|"<<velo_corr.z<<endl<<endl;
		kalman2[id].push_back(Point(correctPt.x,correctPt.y));
		cv::putText(drawing, cv::format("CORRECT velocity: %f", get_velocity(velo_corr.x,velo_corr.y)  ), cv::Point(30, 30+30*(id-4)), cv::FONT_HERSHEY_SIMPLEX, 0.6, pcolor[id-4]); //green if correct
	}
	else{
//		cout<<"CORRECT xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"<<endl;
//		cout<<"COR_VEL xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"<<endl;
		kalman2[id].push_back(Point(predictPt.x,predictPt.y));
		cv::putText(drawing, cv::format("PREDICT velocity: %f", get_velocity(velo_vect.x,velo_vect.y) ), cv::Point(30, 30+30*(id-4)), cv::FONT_HERSHEY_SIMPLEX, 0.6, pcolor[id-4]); //blue if predict
	}
	// plot points

	meas2[id].push_back(p);
	if( meas2[id].size() == 50){
		  meas2[id].erase( meas2[id].begin() );
		  kalman2[id].erase( kalman2[id].begin() );
	}

//	drawCross( p, Scalar(0,255,0), 5 , drawing);
//	Scalar color = pcolor[id-4];
	drawCross( kalman2[id].back(), pcolor[id-4], 5 , drawing);
//
//	for (int i = 0; i < measv.size()-1; i++)
//	 line(drawing, measv[i], measv[i+1], Scalar(0,255,0), 1);

	for (int i = 0; i < kalman2[id].size()-1; i++)
	 line(drawing, kalman2[id][i], kalman2[id][i+1], pcolor[id-4], 1); //red if type =0 (corrected), blue if type=255 (predicted)



}




void doKF(Point3f p, int id){

	Mat_<float> measurement(3,1); measurement.setTo(Scalar(0));
	Mat prediction = KF[id].predict();
	Point3f predictPt(prediction.at<float>(0),prediction.at<float>(1),prediction.at<float>(2));
	Point3f velo_vect(prediction.at<float>(3),prediction.at<float>(4),prediction.at<float>(5));
	measurement(0)=p.x;
	measurement(1)=p.y;
	measurement(2)=p.z;
//	cout<<"MEASURE "<<p.x<<"|"<<p.y<<"|"<<p.z<<endl;
//	cout<<"PREDICT "<<predictPt.x<<"|"<<predictPt.y<<"|"<<predictPt.z<<endl;
	float velo = get_velocity(velo_vect.x,velo_vect.y,velo_vect.z);
//	cout<<"VELOCTY "<<velo_vect.x<<"|"<<velo_vect.y<<"|"<<velo_vect.z<<"|<<<<<<<<<<<<<<"<<velo<<endl;
	if(isnan(p.z)){
		cout<<"--------------------------------------------------------"<<endl;
		kalman3[id].push_back(predictPt);
		kalmanv[id].push_back(velo);
		return;
	}

	Mat estimate = KF[id].correct(measurement);

	Point3f correctPt(estimate.at<float>(0),estimate.at<float>(1),estimate.at<float>(2));
	Point3f velo_corr(estimate.at<float>(3),estimate.at<float>(4),estimate.at<float>(5));
	// plot points
//	cout<<"CORRECT "<<correctPt.x<<"|"<<correctPt.y<<"|"<<correctPt.z<<endl;
	velo = get_velocity(velo_corr.x,velo_corr.y,velo_corr.z);
//	cout<<"COR_VEL "<<velo_corr.x<<"|"<<velo_corr.y<<"|"<<velo_corr.z<<"|<<<<<<<<<<<<<<"<<velo<<endl<<endl;


	if( meas3[id].size() == 50){
		  meas3[id].erase( meas3[id].begin() );
		  kalman3[id].erase( kalman3[id].begin() );
		  kalmanv[id].erase( kalmanv[id].begin() );
	}
	meas3[id].push_back(p);
	kalman3[id].push_back(correctPt);
	kalmanv[id].push_back(velo);


}

void doKF(int id){

	Mat prediction = KF[id].predict();
	Point3f predictPt(prediction.at<float>(0),prediction.at<float>(1),prediction.at<float>(2));
	Point3f velo_vect(prediction.at<float>(3),prediction.at<float>(4),prediction.at<float>(5));
//	cout<<"MEASURE ---------------------------------------------"<<endl;
//	cout<<"PREDICT "<<predictPt.x<<"|"<<predictPt.y<<"|"<<predictPt.z<<endl;
////	cout<<"velo x: "<< prediction.at<float>(3)<<" velo y: "<<prediction.at<float>(4)<<" velo z: "<<prediction.at<float>(5)<<endl;
//	cout<<"VELOCTY "<<velo_vect.x<<"|"<<velo_vect.y<<"|"<<velo_vect.z<<endl;
//	cout<<"CORRECT ---------------------------------------------"<<endl;
	float velo = get_velocity(velo_vect.x,velo_vect.y,velo_vect.z);
	if( kalman3[id].size() == 100){
		  kalman3[id].erase( kalman3[id].begin() );
		  kalmanv[id].erase( kalmanv[id].begin() );
	}
	kalman3[id].push_back(predictPt);
	kalmanv[id].push_back(velo);

}

void printP(Point p){

	cout <<"("<<p.x<<","<<p.y<<") "<<endl;

}


float getZ(Point p, int x_thresh, int y_thresh, vector<Point> contour){
	int limit = 20;
	bool xf = true, yf = true;
	int x = p.x;
	int y = p.y;
	int offx = 1, offy = 1;

	float z = 0;

	if(pointPolygonTest( contour, Point2f((float)p.x,(float)p.y), false ) >= 0){
//		printP(Point(x+offx,y+i));
		z = depth_img.at<float>(p);
		if(isnan(z)==false)return z;
	}
	else{
		cout<<"get neighbor Z"<<endl;
	}

//
//	cout<< "PRINTING SPACE";
//	printP(p);
//	cout<<"x thresh: "<<x_thresh<<endl;
//	cout<<"y thresh: "<<y_thresh<<endl;
//	cout<<"------------------------"<<endl;
	//if(isnan(poi.z)==false)return poi.z;
//	while(xf==true||yf==true){
//	//while(1){
//		if(limit==10)return;
	for(int r = 0;r<limit;r++){
//		cout<<"                    limit: "<<limit<<endl;
		for(int i = 0; i <= min(offx,offy); i++){
			if(xf == true){
//				cout<<"print x"<<endl;
				if(i>0){
					if(pointPolygonTest( contour, Point2f((float)x+offx,(float)y+1), false ) >= 0){
	//					printP(Point(x+offx,y+i));
						z = depth_img.at<float>(Point(x+offx,y+i));
						if(isnan(z)==false)return z+100;
					}
					if(pointPolygonTest( contour, Point2f((float)x-offx,(float)y+1), false ) >= 0){
	//					printP(Point(x-offx,y+i));
						z = depth_img.at<float>(Point(x-offx,y+i));
						if(isnan(z)==false)return z+200;
					}

				}
				if(pointPolygonTest( contour, Point2f((float)x+offx,(float)y-1), false ) >= 0){
	//				printP(Point(x+offx,y-i));
					z = depth_img.at<float>(Point(x+offx,y-i));
					if(isnan(z)==false)return z+300;
				}
				if(pointPolygonTest( contour, Point2f((float)x-offx,(float)y-1), false ) >= 0){
	//				printP(Point(x-offx,y-i));
					z = depth_img.at<float>(Point(x-offx,y-i));
					if(isnan(z)==false)return z+400;
				}
			}

			if(i<min(offx,offy)&&yf==true){
//				cout<<"print y"<<endl;
				if(i>0){
					if(pointPolygonTest( contour, Point2f((float)x+i,(float)y+offy), false ) >= 0){
	//					printP(Point(x+i,y+offy));
						z = depth_img.at<float>(Point(x+i,y+offy));
						if(isnan(z)==false)return z+500;
					}
					if(pointPolygonTest( contour, Point2f((float)x+i,(float)y-offy), false ) >= 0){
	//					printP(Point(x+i,y-offy));
						z = depth_img.at<float>(Point(x+i,y-offy));
						if(isnan(z)==false)return z+600;
					}

				}
				if(pointPolygonTest( contour, Point2f((float)x-i,(float)y+offy), false ) >= 0){
	//				printP(Point(x-i,y+offy));
					z = depth_img.at<float>(Point(x-i,y+offy));
					if(isnan(z)==false)return z+700;
				}
				if(pointPolygonTest( contour, Point2f((float)x-i,(float)y-offy), false ) >= 0){
	//				printP(Point(x-i,y-offy));
					z = depth_img.at<float>(Point(x-i,y-offy));
					if(isnan(z)==false)return z+800;
				}
			}
		}
		//limit++;

		if((x+offx+1)==x_thresh){
			xf = false;
			offx++;
			cout<<"      X reached boundary!"<<endl;
		}
		else if((x+offx+1)<x_thresh){
			offx++;
		}
		if((y+offy+1)==y_thresh){
			yf = false;
			cout<<"      Y reached boundary!"<<endl;
		}
		else if((y+offy+1)<y_thresh){
			offy++;
		}
		if((xf==false) && (yf==false)){
			cout<<"    XY reached boundary!"<<endl;
			return 9876;
		}
	}
	cout<<"nan"<<endl;
	return 12345;
}

Point3f getZd(Point p, int x_thresh, int y_thresh, vector<Point> contour,const PointCloud::ConstPtr& msg){
	int limit = 10;
	bool xf = true, yf = true;
	int x = p.x;
	int y = p.y;
	int offx = 1, offy = 1;

//	pcl::PointXYZRGB point1 = msg->at(p.x, p.y);
//	printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//	printf ("x= %f ,y= %f ,z= %f , red = %d\n",(float)point1.x,(float)point1.y,(float)point1.z,point1.r);

	Point3f ret(0,0,0);
//	float x = 0, y = 0, z = 0;

	if(pointPolygonTest( contour, Point2f((float)p.x,(float)p.y), false ) >= 0){
//		printP(Point(x+offx,y+i));
		ret.x = (float)msg->at(x, y).x;
		ret.y = (float)msg->at(x, y).y;
		ret.z = (float)msg->at(x, y).z;
		if(isnan(ret.z)==false)return ret;
	}
//	else{
//		cout<<"get neighbor Z"<<endl;
//	}

//
//	cout<< "PRINTING SPACE";
//	printP(p);
//	cout<<"x thresh: "<<x_thresh<<endl;
//	cout<<"y thresh: "<<y_thresh<<endl;
//	cout<<"------------------------"<<endl;
	//if(isnan(poi.z)==false)return poi.z;
//	while(xf==true||yf==true){
//	//while(1){
//		if(limit==10)return;
	for(int r = 0;r<limit;r++){
//		cout<<"                    limit: "<<limit<<endl;
		for(int i = 0; i <= min(offx,offy); i++){
			if(xf == true){
//				cout<<"print x"<<endl;
				if(i>0){
					if(pointPolygonTest( contour, Point2f((float)x+offx,(float)y+1), false ) >= 0){
	//					printP(Point(x+offx,y+i));
						ret.x = (float)msg->at(x+offx,y+i).x;
						ret.y = (float)msg->at(x+offx,y+i).y;
						ret.z = (float)msg->at(x+offx,y+i).z;
						if(isnan(ret.z)==false)return ret;
					}
					if(pointPolygonTest( contour, Point2f((float)x-offx,(float)y+1), false ) >= 0){
	//					printP(Point(x-offx,y+i));
						ret.x = (float)msg->at(x-offx,y+i).x;
						ret.y = (float)msg->at(x-offx,y+i).y;
						ret.z = (float)msg->at(x-offx,y+i).z;
						if(isnan(ret.z)==false)return ret;
					}

				}
				if(pointPolygonTest( contour, Point2f((float)x+offx,(float)y-1), false ) >= 0){
	//				printP(Point(x+offx,y-i));
					ret.x = (float)msg->at(x+offx,y-i).x;
					ret.y = (float)msg->at(x+offx,y-i).y;
					ret.z = (float)msg->at(x+offx,y-i).z;
					if(isnan(ret.z)==false)return ret;
				}
				if(pointPolygonTest( contour, Point2f((float)x-offx,(float)y-1), false ) >= 0){
	//				printP(Point(x-offx,y-i));
					ret.x = (float)msg->at(x-offx,y-i).x;
					ret.y = (float)msg->at(x-offx,y-i).y;
					ret.z = (float)msg->at(x-offx,y-i).z;
					if(isnan(ret.z)==false)return ret;
				}
			}

			if(i<min(offx,offy)&&yf==true){
//				cout<<"print y"<<endl;
				if(i>0){
					if(pointPolygonTest( contour, Point2f((float)x+i,(float)y+offy), false ) >= 0){
	//					printP(Point(x+i,y+offy));
						ret.x = (float)msg->at(x+i,y+offy).x;
						ret.y = (float)msg->at(x+i,y+offy).y;
						ret.z = (float)msg->at(x+i,y+offy).z;
						if(isnan(ret.z)==false)return ret;
					}
					if(pointPolygonTest( contour, Point2f((float)x+i,(float)y-offy), false ) >= 0){
	//					printP(Point(x+i,y-offy));
						ret.x = (float)msg->at(x+i,y-offy).x;
						ret.y = (float)msg->at(x+i,y-offy).y;
						ret.z = (float)msg->at(x+i,y-offy).z;
						if(isnan(ret.z)==false)return ret;
					}

				}
				if(pointPolygonTest( contour, Point2f((float)x-i,(float)y+offy), false ) >= 0){
	//				printP(Point(x-i,y+offy));
					ret.x = (float)msg->at(x-i,y+offy).x;
					ret.y = (float)msg->at(x-i,y+offy).y;
					ret.z = (float)msg->at(x-i,y+offy).z;
					if(isnan(ret.z)==false)return ret;
				}
				if(pointPolygonTest( contour, Point2f((float)x-i,(float)y-offy), false ) >= 0){
	//				printP(Point(x-i,y-offy));
					ret.x = (float)msg->at(x-i,y-offy).x;
					ret.y = (float)msg->at(x-i,y-offy).y;
					ret.z = (float)msg->at(x-i,y-offy).z;
					if(isnan(ret.z)==false)return ret;
				}
			}
		}
		//limit++;

		if((x+offx+1)==x_thresh){
			xf = false;
			offx++;
//			cout<<"      X reached boundary!"<<endl;
		}
		else if((x+offx+1)<x_thresh){
			offx++;
		}
		if((y+offy+1)==y_thresh){
			yf = false;
//			cout<<"      Y reached boundary!"<<endl;
		}
		else if((y+offy+1)<y_thresh){
			offy++;
		}
		if((xf==false) && (yf==false)){
//			cout<<"nan boundary box limit reached"<<endl;
//			return Point3f(9876,9876,9876);
			return ret;
		}
	}
//	cout<<"nan limit reached"<<endl;
//	return Point3f(9000,9000,9000);
	return ret;
}

double get_distance(Point one, Point two) {
	return sqrt(pow(one.x-two.x,2) + pow(one.y-two.y,2));
}

double get_distance_xyz(Point3f one, Point3f two) {
	return sqrt(pow(one.x-two.x,2) + pow(one.y-two.y,2) + pow(one.z-two.z,2));
}

double zcheck(float x, float y){
	double temp = x-y;
	if(temp<0)return temp*-1;
	return temp;
}

bool twod_boxcheck(int x, int y, int type){
	double polytest1 = 0;
	for(int i = 0; i<pamount;i++){
		if(b1[i]==true){

//			cout<<"inside polytest "<<type<<" box "<<i<<endl;
			polytest1 = pointPolygonTest( bcontours[i], Point2f((float)x,(float)y), true );

			if((polytest1 > -20)){
//				cout<<"IN BOX detected with distance: "<<polytest1<<endl;
				return true;
			}

		}

	}
	return false;
}

void calc_avg(vector<Point> &loc, vector<Point> &avg){
	int x=0,y=0;
		if(loc.size()==vec_size){
			for(unsigned int i = 0; i < loc.size(); ++i){
				x = loc[i].x + x;
				y = loc[i].y + y;
			}
			x = x / vec_size;
			y = y / vec_size;

			if(avg.size()==vec_size){
				avg.erase(avg.begin());
			}
			avg.push_back(Point(x,y));

		}
}

void createTrackbars(){
	//create window for trackbars


    namedWindow(trackbarWindowName,0);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH),
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
    createTrackbar( "v thresh pixel", trackbarWindowName, &velocity_threshold, velocity_threshold, on_trackbar );
    createTrackbar( "v thresh meter", trackbarWindowName, &velocity_threshold_depth, velocity_threshold_depth, on_trackbar );
    createTrackbar( "in hand distance thresh", trackbarWindowName, &h_distance_threshold, h_distance_threshold, on_trackbar );
    createTrackbar( "acted on distance thresh", trackbarWindowName, &a_distance_threshold, a_distance_threshold, on_trackbar );


}
void drawObject(int x, int y,Mat &frame, Scalar color){

	//drawing circle and coordinates at object location
	circle(frame,Point(x,y),1,color,2);
	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,color,2);

}
void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(1,1));
    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));


	//erode(thresh,thresh,erodeElement);
	//erode(thresh,thresh,erodeElement);


	//dilate(thresh,thresh,dilateElement);
	//dilate(thresh,thresh,dilateElement);



}

bool trackFilteredObject(Mat &threshold_, Mat &cameraFeed_, Mat &drawing, int type, const PointCloud::ConstPtr& msg){

	//type: 1 = p1, 2 = hand, 3 =blue box b1
	Mat temp1;
	int x = 0;
	int y = 0;
	double dist = 999;
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//Canny(threshold_,temp,80,240,3);
	//temp.copyTo(threshold_);
	threshold_.copyTo(temp1);
	// RETR_EXTERNAL for only outer layer contours or standard CV_RETR_CCOMP
	findContours(temp1,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE );
	//largest contour area with max as contour index
	double area = 0;
	double area_sec = 0;
	int max = -1;
	int max_sec = -1;
	double c_size = 0;
	vector<int> safes;
	safes.clear();

	if (contours.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<1000){
//			for (int index = 0; index >= 0; index = hierarchy[index][0]) {
//			for (int index = 0; index< contours.size(); index=hierarchy[index][0]) {
			for (int index = 0; index< contours.size(); index++) {

//				// get area with moments function to find largest area from all contours
//				Moments moment = moments((cv::Mat)contours[index]);
//				if(area<moment.m00){
//					area = moment.m00;
//					max = index;
//				}
				c_size = contourArea((Mat)contours[index]);

				if(c_size>15){

//					if(area<c_size){
//						area_sec = area;
//						area = c_size;
//						max_sec = max;
//						max = index;
//						if(max_sec >= 0)safes.push_back(max_sec);
//					}
//					else if(area_sec<c_size){
//						area_sec = c_size;
//						max_sec = index;
//						if(max_sec >= 0)safes.push_back(max_sec);
//					}
//					else{
//						safes.push_back(index);
//					}
					safes.push_back(index);
					if(area<c_size){
						area_sec = area;
						area = c_size;
						max_sec = max;
						max = index;
					}
					else if(area_sec<c_size){
						area_sec = c_size;
						max_sec = index;
					}
				}

			}
//			if(calib)cout<<area_sec<<"                                      "<<area<<endl;

			// get and draw bounding box, calculate middle point of the box
			if(area>15){	//theres at least one object bigger than 15 pixels
//				putText(cameraFeed_,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				//Scalar color = Scalar(255,0,255);
				Rect r = boundingRect((cv::Mat)contours[max]);
				x = (r.br().x + r.tl().x)/2;
				y = (r.tl().y + r.br().y)/2;
				Point3f temp = getZd(Point(x,y),r.br().x,r.br().y,contours[max],msg);
				// depending on type, write location points into vectors to get average later!
				if(type < 3){   //area_sec to check if second largest area is bigger than 10pixels


					double temp_dist = -1;
					for(int i=0; i< safes.size();i++){
//						cout<<safes.size()<<" packages type "<<type<<endl;
						r = boundingRect((cv::Mat)contours[safes[i]]);
						x = (r.br().x + r.tl().x)/2;
						y = (r.tl().y + r.br().y)/2;
						temp = getZd(Point(x,y),r.br().x,r.br().y,contours[safes[i]],msg);


						if(kalman3[type].empty()){	//no package tracked atm
							if(twod_boxcheck(x,y,type))continue;
						}
						else{
							double dist1 =  get_distance_xyz(kalman3[type].back(),temp);
							if((dist1 > err_dist) && twod_boxcheck(x,y,type)){
//								cout<<"p"<<type<<" err dist"<<dist1<<" + inbox"<<endl;
								continue;
							}

						}

						if(!kalman3[3].empty()){

							if(!isnan(temp.z)){

								temp_dist =  get_distance_xyz(kalman3[3].back(),temp);   //dist h1 to package
								if(temp_dist<dist){
//									if(type==0)cout<<"prefound dist: "<<temp_dist<<" at "<<safes[i]<<endl;

//									cout<<"found "<<type<<endl;
									dist = temp_dist;
									max = safes[i];
								}

							}
						}
						else{
							return false;
						}

					}
					if(temp_dist==-1)return false; //no pkg outside of box or hand detected
//					if(type==0)cout<<max_sec<<endl;



					r = boundingRect((cv::Mat)contours[max]);
					x = (r.br().x + r.tl().x)/2;
					y = (r.tl().y + r.br().y)/2;
					temp = getZd(Point(x,y),r.br().x,r.br().y,contours[max],msg);
					drawContours( drawing, contours, max, pcolor[type], 1, 8, vector<Vec4i>(), 0, Point() );
					rectangle( cameraFeed_, r, pcolor[type], 1, 8, 0 );
					drawObject(x,y,drawing,pcolor[type]);
					drawObject(x,y,cameraFeed_,pcolor[type]);

//					putText(cameraFeed_,format("z: %f",temp.z),Point(0,400),2,1,Scalar(0,255,0),2);


					if(isnan(temp.z))return false;

					if(kalman3[type].empty()){	//check new location with past location, if too far apart wrongly detected
//						double polytest1 = 0;
//						for(int i = 0; i<pamount;i++){
//							if(b1[i]==true){
//
//								cout<<"inside polytest "<<type<<" box "<<i<<endl;
//								polytest1 = pointPolygonTest( bcontours[i], Point2f((float)x,(float)y), true );
//
//								if((polytest1 > -20)){
//									cout<<"NOOOO"<<endl;
//									return false;
//								}
//
//							}
//
//						}
						reinit_KF(temp,type);
						reinit_KF(Point(x,y),type+4);
					}
					else{
						dist =  get_distance_xyz(kalman3[type].back(),temp);
						if(dist > err_dist){
//							cout<<"p"<<type<<" err dist!!"<<err_dist<<endl;
							//cout<<"package ["<<type<< "] distance to itself: "<<dist<<endl;
//							cout<<"package distance to itself: "<<dist<<endl;
//							drawObject(x,y,cameraFeed_,h1_color);
							return false;
						}
					}

					doKF(temp,type);
					doKF(Point(x,y),type+4,1);
					return true;
				}
				if(type == 3 && area>15){ //hand
					drawContours( drawing, contours, max, h1_color, 1, 8, vector<Vec4i>(), 0, Point() );
					rectangle( cameraFeed_, r, h1_color, 1, 8, 0 );
					drawObject(x,y,drawing,h1_color);
					drawObject(x,y,cameraFeed_,h1_color);



					if(isnan(temp.z))return false;

					if(kalman3[type].empty()){	//check new location with past location, if too far apart wrongly detected
						reinit_KF(temp,type);
					}
					else{
						dist =  get_distance_xyz(kalman3[type].back(),temp);
//						cout<<"hand ["<<type<< "] distance to itself: "<<dist<<endl;
						if(dist >err_dist){
//							cout<<"package distance to itself: "<<dist<<endl;
//							drawObject(x,y,cameraFeed_,h1_color);
							return false;
						}
					}

					doKF(temp,type);
					return true;
				}
				// get initial box contours to use with pointpolygontest later, also save contour in box vector, uncomment for more containers
				if(type == 4||type==5||type==6){
					if(isnan(temp.z)==false){
						box_xyz[type-4] = temp;
					}
					else{
						return false;
					}
					drawContours( drawing, contours, max, bcolor[type-4], 1, 8, vector<Vec4i>(), 0, Point() );
					drawContours( cameraFeed_, contours, max, bcolor[type-4], 1, 8, vector<Vec4i>(), 0, Point() );
					box1.push_back(contours[max]);
					bcontours[type-4] = contours[max];
//					box1_xyz = getZd(Point(x,y),r.br().x,r.br().y,contours[max],msg);

					cout<<"box"<<(type-4)<<"  z: "<<box_xyz[type-4].z<<endl;
					return true;
				}

				if(type == 10){
					if(isnan(temp.z)==false){
						conv_xyz = temp;
					}
					else{
						return false;
					}
					drawContours( drawing, contours, max, Scalar(255,255,255), 1, 8, vector<Vec4i>(), 0, Point() );
					drawContours( cameraFeed_, contours, max, Scalar(255,255,255), 1, 8, vector<Vec4i>(), 0, Point() );
					ccontours[0] = contours[max];
					conv_xyz.z = conv_xyz.z - 0.07;
					return true;
				}


				//calib mode contour drawing
				if(type == 20){
					for(int i =0;i<safes.size();i++){
						r = boundingRect((cv::Mat)contours[safes[i]]);
						drawContours( drawing, contours, safes[i], pcolor[q+w], 1, 8, vector<Vec4i>(), 0, Point() );

						rectangle( cameraFeed_, r, pcolor[q+w], 1, 8, 0 );
						rectangle( drawing, r, pcolor[q+w], 1, 8, 0 );
						drawObject(x,y,drawing,pcolor[q+w]);
						drawObject(x,y,cameraFeed_,pcolor[q+w]);

					}
//					if(!calibh && area_sec>15){
//						r = boundingRect((cv::Mat)contours[max_sec]);
//						x = (r.br().x + r.tl().x)/2;
//						y = (r.tl().y + r.br().y)/2;
//						temp = getZd(Point(x,y),r.br().x,r.br().y,contours[max_sec],msg);
//						drawContours( drawing, contours, max_sec, pcolor[0], 1, 8, vector<Vec4i>(), 0, Point() );
//
//						rectangle( cameraFeed_, r, pcolor[0], 2, 8, 0 );
//						rectangle( drawing, r, pcolor[0], 2, 8, 0 );
//						drawObject(x,y,drawing,pcolor[0]);
//						drawObject(x,y,cameraFeed_,pcolor[0]);
//					}
//					else if(calibh){
//						if(area<50)return false;
//						drawContours( drawing, contours, max, pcolor[0], 1, 8, vector<Vec4i>(), 0, Point() );
//
//						rectangle( cameraFeed_, r, pcolor[0], 2, 8, 0 );
//						rectangle( drawing, r, pcolor[0], 2, 8, 0 );
//						drawObject(x,y,drawing,pcolor[0]);
//						drawObject(x,y,cameraFeed_,pcolor[0]);
//					}

				}

			}

		}
	}

	return false;
}


float get_float(Point x){
	float z = 0;
	Point y = x;
	z = depth_img.at<float>(y);
	if(isnan(z)){
		for(int i = 0; i<10;i++){
			for(int k = 0; k<10;k++){
				y.x = y.x+i-5;
				y.y = y.y+k-5;
				z = depth_img.at<float>(y);
				if(isnan(z)==false)return z;
			}
		}
		z = 0.123456; // return this if no value found
	}
	return z;
}

void call_itu(ros::ServiceClient client_, hrc_ros::InformTrayUpdate itu){
	if (client_.call(itu)){
//		ROS_INFO("Response: %d", srv.response.sth);
	}
	else{
		ROS_ERROR("Failed to call service tray update new");
	}
}


class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<PointCloud>("/kalman", 5);
    nined_pub_ = n_.advertise<object_tracking::NineD>("/camera_agent/tray_detection9D", 5);
    motion_features_pub_ = n_.advertise<object_tracking::Subfeatures>("/camera_agent/motion_features", 5);

    //Topic you want to subscribe
    sub_ = n_.subscribe<PointCloud>("/camera/depth_registered/points", 5, &SubscribeAndPublish::callback, this);

    client_ = n_.serviceClient<object_tracking::Feature>("feature_processing");
    tray_upd_client_ = n_.serviceClient<object_tracking::TrayUpdateCamera>("/observation_agent/inform_tray_update_old");
    new_tray_client_ = n_.serviceClient<hrc_ros::InformTrayUpdate>("/observation_agent/inform_tray_update");

//    pickandplace_client_ = n_.serviceClient<dobot::OneTimePickAndPlace>("/dobot_arm_app/oneTimePickAndPlace");



  }

  void callback(const PointCloud::ConstPtr& msg)
  {

//void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){


//	//calc dt in seconds
//	time_t temp = (std::time(0) - time1);
//	time1 = std::time(0);
//	cout << temp<<endl;

//	  ROS_INFO("in processing");


	object_tracking::Feature srv;
	object_tracking::TrayUpdateCamera tuc;
	object_tracking::NineD nined_info;
	object_tracking::Subfeatures subf;
	hrc_ros::InformTrayUpdate itu;


//	srv.request.rostime = ros::Time::now();
	pcl_conversions::fromPCL( msg->header.stamp, srv.request.rostime);
	pcl_conversions::fromPCL( msg->header.stamp, tuc.request.stamp);
	pcl_conversions::fromPCL( msg->header.stamp, nined_info.stamp);
	pcl_conversions::fromPCL( msg->header.stamp, subf.rostime);
	pcl_conversions::fromPCL( msg->header.stamp, itu.request.stamp);

	double min_dist = 999;
	double temp = ticks;
	ticks = (double) getTickCount(); //opencv functions

	float dt = (float)(ticks - temp) / getTickFrequency(); //seconds from last cloud_cb
	second_c += dt;
	frame_c++;
	if(second_c>=1){
//		cout<<"fps: "<<frame_c<<endl;
		fps = frame_c;
		second_c = 0;
		frame_c = 0;
	}

//	cout <<"dt: "<<dt<<endl;
	update_transitionMatrix(dt);
//	KF[0].transitionMatrix = (Mat_<float>(6, 6) << 	1, 0, 0, dt, 0, 0,
//													0, 1, 0, 0, dt, 0,
//													0, 0, 1, 0, 0, dt,
//													0, 0, 0, 1, 0, 0,
//													0, 0, 0, 0, 1, 0,
//													0, 0, 0, 0, 0, 1);

	Mat img;
	if (msg->isOrganized()) {
//	    if (!msg->empty())printM(msg);
		if(!img.empty())cout<<"test1"<<endl;
		img = cv::Mat(msg->height, msg->width, CV_8UC3);
//		if(!img.empty())cout<<"test2"<<endl;

		if (!msg->empty()) {

			for (int h=0; h<img.rows; h++) {
				for (int w=0; w<img.cols; w++) {
					pcl::PointXYZRGB point = msg->at(w, h);

//	                Eigen::Vector3i rgb = point.getRGBVector3i();
//
//	                img.at<cv::Vec3b>(h,w)[0] = rgb[2];
//	                img.at<cv::Vec3b>(h,w)[1] = rgb[1];
//	                img.at<cv::Vec3b>(h,w)[2] = rgb[0];

					img.at<cv::Vec3b>(h,w)[0] = point.b;
					img.at<cv::Vec3b>(h,w)[1] = point.g;
					img.at<cv::Vec3b>(h,w)[2] = point.r;
				}
			}
		}
	}

	if(!img.empty()){

		current_image = img;
//		cv::imshow("rgb image", current_image);
		//black image to draw contours later
		drawing = Mat::zeros( FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3 );
//		kalm = Mat::zeros( FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3 );
		//store image to matrix
		cameraFeed  = img;
		if(tframes<10000)tframes++;
		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
		//calib to get the object color values by hand

		Mat testings;



		if(calib==true){


			cameraFeed.copyTo(testings);
//			vector<Point> polygon{Point(FRAME_WIDTH,FRAME_HEIGHT),Point(FRAME_WIDTH,0),Point(500,0),Point(0,380),Point(0,FRAME_HEIGHT)};
			vector<Point> polygon{Point(FRAME_WIDTH,FRAME_HEIGHT),Point(FRAME_WIDTH,435),Point(0,216),Point(0,FRAME_HEIGHT)};

			fillConvexPoly(testings,polygon,Scalar(255,255,255));

			imshow("cut off area",testings);
			Mat HSV_cut;
			cvtColor(testings,HSV_cut,COLOR_BGR2HSV);
			imshow("cut off hsv",HSV_cut);

//			Mat testings2;
//			testings.copyTo(testings2);
//			vector<Point> polygon2{Point(FRAME_WIDTH,470),Point(FRAME_WIDTH,0),Point(0,0),Point(0,180)};
//			fillConvexPoly(testings2,polygon2,Scalar(255,255,255));
//			cvtColor(testings2,HSV_cut,COLOR_BGR2HSV);
//			imshow("2cuts",testings2);

//			pamount=1;
			//thresholding image by HSV values
			inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),thresholdm);

			if(calib_mov==true){
				Mat nmov;
				inRange(HSV,nmov_min,nmov_max,nmov);
				bitwise_or(thresholdm,nmov,thresholdm);
			}
			//smoothing the image
			medianBlur(thresholdm, blured, 3);
			//morphological operations to filter noise
			morphOps(blured);
			//copying matrixes to display them later with imshow
			blured.copyTo(canny);
			cameraFeed.copyTo(final);
			// canny filter, findcontours and creating boundary box for type > 0
			p1[0] = trackFilteredObject(canny,final,drawing,20,msg);
			//imshow(windowName1,HSV);
			//imshow(windowName2,thresholdm);
			imshow(windowName8,blured);
		}
		else{
//type: 1 = p1, 2 = hand, 3 =blue box b1
			//package



			cameraFeed.copyTo(final); //final to get result, drawing to get contours

			cameraFeed.copyTo(testings);


			//435,216
//			vector<Point> polygon{Point(FRAME_WIDTH,FRAME_HEIGHT),Point(FRAME_WIDTH,0),Point(500,0),Point(0,380),Point(0,FRAME_HEIGHT)};
			vector<Point> polygon{Point(FRAME_WIDTH,FRAME_HEIGHT),Point(FRAME_WIDTH,435),Point(0,216),Point(0,FRAME_HEIGHT)};

			fillConvexPoly(testings,polygon,Scalar(255,255,255));

			imshow("cut off area",testings);
			Mat HSV_cut;
			cvtColor(testings,HSV_cut,COLOR_BGR2HSV);
			imshow("cut off hsv",HSV_cut);


			//435,216
			Mat testings2;
			testings.copyTo(testings2);
			vector<Point> polygon2{Point(FRAME_WIDTH,470),Point(FRAME_WIDTH,0),Point(0,0),Point(0,180)};
			fillConvexPoly(testings2,polygon2,Scalar(255,255,255));
			imshow("2cuts",testings2);

			Mat HSV_cut2;
			cvtColor(testings2,HSV_cut2,COLOR_BGR2HSV);
			imshow("cut off hsv",HSV_cut2);




			//hand 1
			inRange(HSV,h1_HSVmin,h1_HSVmax,thresholdm);
			medianBlur(thresholdm, blured, 3);
			morphOps(blured);
			blured.copyTo(canny);
			hand1 = trackFilteredObject(canny,final,drawing,3,msg);
			Mat all_blured;
			blured.copyTo(all_blured);


			//packages + trays
			for(int i=0;i<pamount;i++){
				inRange(HSV_cut,pHSVmin[i],pHSVmax[i],thresholdm);

				if(i==0){
					Mat nmov;
					inRange(HSV_cut,nmov_min,nmov_max,nmov);
					bitwise_or(thresholdm,nmov,thresholdm);
				}


				medianBlur(thresholdm, blured, 3);
				morphOps(blured);
				blured.copyTo(canny);
				p1[i] = trackFilteredObject(canny,final,drawing,i,msg);

				bitwise_or(all_blured,blured,all_blured);

				if(tframes>10){
					if(b1[i] == false){
						inRange(HSV_cut,bHSVmin[i],bHSVmax[i],thresholdm);
						medianBlur(thresholdm, blured, 3);
						morphOps(blured);
						blured.copyTo(canny);
						b1[i] = trackFilteredObject(canny,final,drawing,i+4,msg);
					}else{
						drawContours( drawing, bcontours, i, bcolor[i], 1, 8, vector<Vec4i>(), 0, Point() );
						drawContours( final, bcontours, i, bcolor[i], 1, 8, vector<Vec4i>(), 0, Point() );
					}
//					if(conv1 == false){
//						inRange(HSV_cut2,conv_min,conv_max,thresholdm);
//						medianBlur(thresholdm, blured, 3);
//						morphOps(blured);
//						blured.copyTo(canny);
//						conv1 = trackFilteredObject(canny,final,drawing,10,msg);
//					}else{
//						drawContours( drawing, ccontours, 0, Scalar(255,255,255), 1, 8, vector<Vec4i>(), 0, Point() );
//						drawContours( final, ccontours, 0,Scalar(255,255,255), 1, 8, vector<Vec4i>(), 0, Point() );
//					}
				}
			}

			imshow("all blured",all_blured);
		    moveWindow("all blured",710,0);

//			ImageScanner scanner;
//			scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
//			Mat grey;
//			Mat qr;
//			current_image.copyTo(qr);
//			cvtColor(qr,grey,CV_BGR2GRAY);
//
//			int width = qr.cols;
//			int height = qr.rows;
//			uchar *raw = (uchar *)grey.data;
//			// wrap image data
//			Image image(width, height, "Y800", raw, width * height);
//			// scan the image for barcodes
//			int n = scanner.scan(image);
//			// extract results
//			for(Image::SymbolIterator symbol = image.symbol_begin();symbol != image.symbol_end(); ++symbol) {
//				vector<Point> vp;
//				// do something useful with results
//				cout << "decoded " << symbol->get_type_name()  << " symbol \"" << symbol->get_data() << '"' <<" "<< endl;
//			   int n = symbol->get_location_size();
//			   for(int i=0;i<n;i++){
//					vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
//			   }
//			   RotatedRect r = minAreaRect(vp);
//			   Point2f pts[4];
//			   r.points(pts);
//			   for(int i=0;i<4;i++){
//					line(qr,pts[i],pts[(i+1)%4],Scalar(255,0,0),3);
//			   }
//			   //cout<<"Angle: "<<r.angle<<endl;
//			}
//
//			imshow("qr-code", qr); //show the frame in "MyVideo" window


			//get container contour, uncomment for more containers
			if(rosbag)
				record = true;
//			if(rosbag || tframes>30){

	//			if(b2 == false){
	//				inRange(HSV,b2_HSVmin,b2_HSVmax,thresholdm);
	//				medianBlur(thresholdm, blured, 3);
	//				morphOps(blured);
	//				blured.copyTo(canny);
	//				cameraFeed.copyTo(final);
	//				b2 = trackFilteredObject(canny,final,drawing,4);
	//			}else{
	//				drawContours( drawing, box2, 0, b2_color, 1, 8, vector<Vec4i>(), 0, Point() );
	//				drawContours( final, box2, 0, b2_color, 1, 8, vector<Vec4i>(), 0, Point() );
	//			}
	//
	//			if(b3 == false){
	//				inRange(HSV,b3_HSVmin,b3_HSVmax,thresholdm);
	//				medianBlur(thresholdm, blured, 3);
	//				morphOps(blured);
	//				blured.copyTo(canny);
	//				cameraFeed.copyTo(final);
	//				b3 = trackFilteredObject(canny,final,drawing,5);
	//			}else{
	//				drawContours( drawing, box3, 0, b3_color, 1, 8, vector<Vec4i>(), 0, Point() );
	//				drawContours( final, box3, 0, b3_color, 1, 8, vector<Vec4i>(), 0, Point() );
	//			}

		}


		//to check if a point is in contour: pointPolygonTest(contour, point2f , false: -> +1 0 +1 -> +1 inside 0 on edge)

		//show frames
	//		imshow(windowName,cameraFeed);
		imshow("location",final);
//		imshow("kalmann",kalm);

		// updating the averages


		if(hand1){
			h1miss_c = 0;
		//	calc_avg(h1loc, h1_avg);
		}
		else{
			h1miss_c++;
			if(h1miss_c>kalman_frames){
//				h1loc.clear();
////				h1loc_z.clear();
//				h1_avg.clear();
//				h1loc_xyz.clear();
////				h1_avg_xyz.clear();
				meas3[3].clear();
				kalman3[3].clear();
				kalmanv[3].clear();
				kalmanvec.clear();
				measv.clear();
			}
			else{
				if(!kalman3[3].empty())doKF(3);
//				if(h1loc_xyz.size()>0)doKF(3);
//				if(h1loc.size()>1)doKF(h1loc.back(),1,0);
			}
		}
		for(int i=0; i<pamount;i++){
			if(p1[i]){
				pmiss[i] = 0;
				//cout<<"not missing"<<endl;
			}
			else{
				pmiss[i]++;
				if(pmiss[i]>kalman_frames){
//					ploc.clear();
//					p_avg.clear();
//					ploc_xyz.clear();
					meas2[i+4].clear();
					kalman2[i+4].clear();
					meas3[i].clear();
					kalman3[i].clear();
					kalmanv[i].clear();
					s_flag[i]=0;
				}
				else{
					if(!kalman3[i].empty())doKF(i);
					if(!kalman2[i+4].empty())doKF(Point(0,0),i+4,0);
				}
			}


		}

		vector<float> distanz{999,999,999};
		if(!kalman3[3].empty()){
			for(int i = 0; i<pamount;i++){
				if(!kalman3[i].empty()){
					distanz[i] = get_distance_xyz(kalman3[3].back(),kalman3[i].back());
				}
				else{
					distanz[i] = 999;
				}
			}
		}

		imshow("contour",drawing);




	//		//helper frame to display information! like printf
		cameraFeed.copyTo(frame);
	//        frameCounter++;
	//
	//        std::time_t timeNow = std::time(0) - timeBegin;
	//
	//        // gets number of frames since the last second
	//        if (timeNow - tick >= 1)
	//        {
	//            tick++;
	//            fps = frameCounter;
	//            frameCounter = 0;
	//        }


		// printing fps and last avg locations for hand and package
		int offset = 30;
		int offc = 1;
		cv::putText(frame, cv::format("FPS %d total frames: %d", fps, tframes  ), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255));
//		offc++;
//		if( p_avg.size() > 0){
//			cv::putText(frame, cv::format("p (%d, %d)", p_avg.back().x, p_avg.back().y ), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, r_color);
//			//cv::putText(frame, cv::format("p z (%f)", get_float(p_avg.back() )), cv::Point(230, 80), cv::FONT_HERSHEY_SIMPLEX, 0.6, r_color);
//		}
//		offc++;
//		if( h1_avg.size() > 0 ){
//			cv::putText(frame, cv::format("h1 (%d, %d)", h1_avg.back().x, h1_avg.back().y ), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, h1_color);
//			offc++;
//			cv::putText(frame, cv::format("h1loc z (%f)", h1loc_xyz.back().z), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, h1_color);
//	//        	cv::putText(frame, cv::format("h1loc z (%f)", depth_img.at<float>(h1loc.back() )), cv::Point(30, 380), cv::FONT_HERSHEY_SIMPLEX, 0.6, h1_color);
//		}
//		offc++;
//
//		offc++;
//		if(hand1 == true && p1[0] == true && h1_avg.size() > 0 && p_avg.size()> 0 ){
//			//calculating distance from last average package and hand location
//			distanc = get_distance(p_avg.back(),h1_avg.back());
//			if(distanc < h_distance_threshold){
//				cv::putText(frame, "HAND ON OBJECT", cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, h1_color);
//				offc++;
//				cv::putText(frame, "OBJECT NOT ACTED ON", cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, h1_color);
//			}
//			else{
//				cv::putText(frame, "HAND NOT ON OBJECT", cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, h1_color);
//				//comparing distance from the last frames: vec_size  vec_size/2 and current frame
//				offc++;
//				if(distanc < a_distance_threshold){
//					cv::putText(frame, "OBJECT ACTED ON", cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, h1_color);
//				}
//				else{
//					cv::putText(frame, "OBJECT NOT ACTED ON", cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, h1_color);
//
//				}
//
//			}
//		}


		//moving or not: 1 or 2
//		int mov1 = 0;


		// checking movement state by comparing hand location from last vec_size frame average and current frame average
//		offc++;
//		if(hand1 == true && h1_avg.size()==vec_size){
//			velocity1 = get_distance(h1_avg.front(),h1_avg.back());
//	//			cout << "velocity h1: "<< velocity1 << endl;
//			if(velocity1 > velocity_threshold){
//				cv::putText(frame, "MOVING", cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, h1_color);
//				mov1 = 1;
//			}
//			else{
//				cv::putText(frame, "NOT MOVING", cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, h1_color);
//				mov1 = 2;
//
//			}
//		}
//		double vel_thresh = (double)velocity_threshold_depth/100000;
//		if(hand1 == true && h1loc_xyz.size()==vec_size){
//			velocity1 = get_distance_xyz(h1loc_xyz.front(),h1loc_xyz.back());
////			cout << "threshhold: "<< vel_thresh<< " || velocity xyz h1: "<< velocity1 << endl;
//			if(velocity1 > vel_thresh){
//				cv::putText(frame, "MOVING", cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, h1_color);
//				mov1 = 1;
//			}
//			else{
//				cv::putText(frame, "NOT MOVING", cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, h1_color);
//				mov1 = 2;
//
//			}
//		}


		offc++;
		if(b1[0] && p1[0] && p_avg.size()>0){

			if(pointPolygonTest( box1[0], Point2f((float)p_avg.back().x,(float)p_avg.back().y), false ) > 0){
				cv::putText(frame, "in box1", cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, b1_color);
			}
			else{
				cv::putText(frame, "not in box1", cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, b1_color);
			}

		}

//		for(int i=0;i<pamount;i++){
//			offc++;
//			if(b1[i] && !kalman2[i+4].empty()){
//
//				if(pointPolygonTest( bcontours[i], Point2f((float)kalman2[i+4].back().x,(float)kalman2[i+4].back().y), false ) > 0){
//					cv::putText(frame, cv::format("obj in box[%d]: YES", i), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, pcolor[i]);
//				}
//				else{
//					cv::putText(frame, cv::format("obj in box[%d]: NO", i), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6,pcolor[i]);
//				}
//
//			}
//
//		}
		float ztest;
		double polytest;

		for(int i=0;i<pamount;i++){
			for(int k =0;k<pamount;k++){
				offc++;
				if(b1[i] && !kalman2[k+4].empty() && !kalman3[k].empty()){

					ztest = zcheck(kalman3[k].back().z,box_xyz[i].z); //~0.06 ~0.058
					polytest = pointPolygonTest( bcontours[i], Point2f((float)kalman2[k+4].back().x,(float)kalman2[k+4].back().y), true );

					if((polytest > -6)&&(ztest<0.085)){
						//zcheck should be about 0.06 to be within the container
						cv::putText(frame, cv::format("obj #%d in box[%d]: YES %f", k, i, polytest ), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, pcolor[i]);
					}
					else{
						cv::putText(frame, cv::format("obj #%d in box[%d]: NO %f", k, i, polytest), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6,pcolor[i]);
					}

				}
			}

		}


		for(int i=0;i<pamount;i++){
			offc++;
			if(b1[i] ){
				cv::putText(frame, cv::format("box[%d] z (%f)", i,box_xyz[i].z), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, pcolor[i]);
			}

		}
		offc++;
		if(hand1){
			cv::putText(frame, cv::format("CORRECT velocity h1 (%f)",kalmanv[3].back() ), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, h1_color);
		}
		else if(!kalmanv[3].empty()){
			cv::putText(frame, cv::format("PREDICT velocity h1 (%f)",kalmanv[3].back() ), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, h1_color);
		}
		else{
			cv::putText(frame, cv::format("no h1 detected for last 10frames" ), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, h1_color);
		}
		int offc2 = 0;
		int saved_obj =0;
		for(int i =0; i<pamount;i++){
			offc2++;
			cv::putText(frame, cv::format("h1 obj#%d dist (%f)", i,  distanz[i] ), cv::Point(375, offset*offc2), cv::FONT_HERSHEY_SIMPLEX, 0.6, pcolor[i]);
			if(min_dist>distanz[i]){
				saved_obj = i;
				min_dist = distanz[i];
			}
		}

		if(min_dist > 100){
			if(!kalman3[3].empty()){
				if(conv1){
					min_dist = get_distance_xyz(kalman3[3].back(),conv_xyz);
					saved_obj = pamount;
				}
			}
		}
		if(s_flag[saved_obj]==1){
			if(!kalman3[3].empty()){
				if(conv1){
					double c_dist = get_distance_xyz(kalman3[3].back(),conv_xyz);
					if(c_dist<distanz[saved_obj]){
						min_dist = c_dist;
						saved_obj = pamount;
					}
				}
			}
		}

//		vector<bool> detected{false,false,false,false,false,false,false,false,false};
		vector<int> detected{0,0,0,0,0,0,0,0,0};
		offc2++;
		if(saved_obj<pamount)
			cv::putText(frame, cv::format("min h1 obj%d dist (%f)", saved_obj, min_dist ), cv::Point(375, offset*offc2), cv::FONT_HERSHEY_SIMPLEX, 0.6, pcolor[saved_obj]);
		if(saved_obj==pamount)
			cv::putText(frame, cv::format("min h1 obj%d dist (%f)", saved_obj, min_dist ), cv::Point(375, offset*offc2), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(123,123,123));


		offc2++;
		if(min_dist<100){
			if(s_flag[saved_obj]==0){
				for(int i=0;i<pamount;i++){

					//clearing all other inside box packages
					if(i!=saved_obj && s_flag[i]==1){
						cout<<"special clear"<<endl;
						meas2[i+4].clear();
						kalman2[i+4].clear();
						meas3[i].clear();
						kalman3[i].clear();
						kalmanv[i].clear();
						s_flag[i]=0;
					}

					if(saved_obj==3)continue;

					if(b1[i] && !kalman2[saved_obj+4].empty() && !kalman3[saved_obj].empty()){

						ztest = zcheck(kalman3[saved_obj].back().z,box_xyz[i].z); //~0.06 ~0.058
						polytest = pointPolygonTest( bcontours[i], Point2f((float)kalman2[saved_obj+4].back().x,(float)kalman2[saved_obj+4].back().y), true );

						if((polytest > -6)&&(ztest<0.085)){
							detected[saved_obj+(3*i)]=1;
							s_flag[saved_obj] = 1;

//							if(rules[saved_obj]==i){
//								cout<<"SUCCESS"<<endl;
//							}
//							else{
//								fail_counter++;
//								cout<<"FAIL"<<endl;
//								if(fail_counter==3){
//									//do service call
//								}
//							}

							for(int k =0; k<detected.size();k++){

								int c_index = k/3;
//								int c_index = 0;
//								if(k<3)c_index = 0;
//								else if(k<6)c_index = 1;
//								else if(k<9)c_index = 2;
								cv::putText(frame, cv::format("%d", detected[k]), cv::Point(375+(k*12), offset*offc2), cv::FONT_HERSHEY_SIMPLEX, 0.6, pcolor[c_index]);
								cv::putText(frame, cv::format("%d",s_flag[saved_obj]), cv::Point(375, offset*(offc2+1)), cv::FONT_HERSHEY_SIMPLEX, 0.6, pcolor[c_index]);

							}
							nined_info.data = detected;
//							nined_pub_.publish(nined_info);

							tuc.request.data = detected;

							itu.request.current_object = saved_obj+1;
							itu.request.current_tray = i;

//							thread thd(call_itu,new_tray_client_,itu);
//							thd.detach();

							if (new_tray_client_.call(itu)){
						//		ROS_INFO("Response: %d", srv.response.sth);
							}
							else{
								ROS_ERROR("Failed to call service tray update new");
							}

							if (tray_upd_client_.call(tuc)){
						//		ROS_INFO("Response: %d", srv.response.sth);
							}
							else{
								ROS_ERROR("Failed to call service tray update old");
							}
							cout<<"                                        SEND "<<endl;

						}

					}

				}
			}
		}


//		offc++;
//			cv::putText(frame, cv::format("h1 packet distance (%f)", distanz ), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, b1_color);

		offc++;
		if(grasp)
		cv::putText(frame, cv::format("GRASP"), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, b1_color);

		if(idle)
		cv::putText(frame, cv::format("IDLE"), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, b1_color);

		if(gesture)
		cv::putText(frame, cv::format("GESTURE"), cv::Point(30, offset*offc), cv::FONT_HERSHEY_SIMPLEX, 0.6, b1_color);


		if(!calib)imshow("print", frame);
		//cout<<"OpenCV Version used:"<<CV_MAJOR_VERSION<<"."<<CV_MINOR_VERSION<<endl;

//		if(rosbag){
		if(grasp||idle||gesture){
		  if (myfile.is_open()){

			  if(!kalmanv[3].empty())
				  myfile <<kalmanv[3].back()<<","; //h1 velocity
			  else myfile << "'not detected',";

			  if(!kalmanv[3].empty() &&  min_dist<999){
				  myfile <<min_dist<<",";			//distance h1 - p1

			  }
			  else myfile << "'not detected',";

			  if(idle)myfile<<"idle";
			  else if(grasp)myfile<<"grasp";
			  else if(gesture)myfile<<"gesture";

			  myfile<<endl;
		  }
		  else cout << "Unable to open file";
		}





		char c=(char)waitKey(1);
		//press o to print out min and max hsv, esc to close
		if(c==111){
//			cout << "printing hsv min and hsv max" << endl;
//			cout << "(" << H_MIN << "," << S_MIN << "," << V_MIN << ");"<< endl;
//			cout << "(" << H_MAX << "," << S_MAX << "," << V_MAX << ");"<< endl;
			cout << "setting file format hsv (H_MIN S_MIN V_MIN H_MAX S_MAX V_MAX)"<<endl;
			cout << H_MIN << " " << S_MIN << " " << V_MIN << " " << H_MAX << " " << S_MAX << " " << V_MAX << endl;
		}
//		if(c==114){	// press r to toggle recording feature
//			record = !record;
//			if(record)
//				cout << "recording" << endl;
//			if(!record)
//				cout << "recording  stopped" << endl;
//
//		}
		if(c==27){
			//break;

			if (myfile.is_open())myfile.close();
			cout << "ESC" << endl;
		}
		if(c==103){  //g to toggle grasp label
			grasp = !grasp;
			if (myfile.is_open())myfile<<endl<<"at Frame: "<<tframes<<endl;
			idle=false;
			gesture=false;
		}
		if(c==105){	//i to toggle idle label
			idle = !idle;
			if (myfile.is_open())myfile<<endl<<"at Frame: "<<tframes<<endl;
			grasp=false;
			gesture=false;
		}
		if(c==115){	//s to toggle gesture
			gesture = !gesture;
			if (myfile.is_open())myfile<<endl<<"at Frame: "<<tframes<<endl;
			idle=false;
			grasp=false;
		}
		if(c==113){ //press q, load package colors
			w=0;
			q++;
			if(q==pamount)q=0;
			H_MIN = pHSVmin[q].val[0];
			H_MAX = pHSVmax[q].val[0];
			S_MIN = pHSVmin[q].val[1];
			S_MAX = pHSVmax[q].val[1];
			V_MIN = pHSVmin[q].val[2];
			V_MAX = pHSVmax[q].val[2];
		}
		if(c==119){ //press w, load box colors
			q=0;
			w++;
			if(w==pamount)w=0;
			H_MIN = bHSVmin[w].val[0];
			H_MAX = bHSVmax[w].val[0];
			S_MIN = bHSVmin[w].val[1];
			S_MAX = bHSVmax[w].val[1];
			V_MIN = bHSVmin[w].val[2];
			V_MAX = bHSVmax[w].val[2];
		}
		if(c==101){ //press e, load hand color
			H_MIN = h1_HSVmin.val[0];
			H_MAX = h1_HSVmax.val[0];
			S_MIN = h1_HSVmin.val[1];
			S_MAX = h1_HSVmax.val[1];
			V_MIN = h1_HSVmin.val[2];
			V_MAX = h1_HSVmax.val[2];
		}
		if(c==114){ //press r, reset color
			H_MIN = 0;
			H_MAX = 256;
			S_MIN = 0;
			S_MAX = 256;
			V_MIN = 0;
			V_MAX = 256;
		}
		if(c==116){ //press t, load conveyor color
			H_MIN = conv_min.val[0];
			H_MAX = conv_max.val[0];
			S_MIN = conv_min.val[1];
			S_MAX = conv_max.val[1];
			V_MIN = conv_min.val[2];
			V_MAX = conv_max.val[2];
		}
		if(c==109){ //press m, to show still red packages in calibmode
			calib_mov = !calib_mov;
		}
		if(c==99){ //press c, to set current package as reference point
			if(!kalman3[saved_obj].empty()){
				conv_xyz = kalman3[saved_obj].back();
	        	cout<<"reference point: "<<conv_xyz<<endl;
	        	conv1 = true;
			}
		}
		if(c==88){ //press x, to load new rules
			load_rules();
		}
		if(c==116){ // t to print red package coordinate
			if(!kalman3[0].empty()){
				cout<<"Z: "<<kalman3[0].back().z<<endl;
			}
		}

	}

	if(meas3[0].size()>0){
		PointCloud::Ptr pmsg (new PointCloud);
		pmsg->header.frame_id = "some_tf_frame";
	//	pmsg->height = msg ->height;
	//	pmsg->width = msg->width;
	//	pmsg->height = pmsg->width = 1;
	//	pmsg->points.push_back (pcl::PointXYZRGB(1.0, 2.0, 3.0));
		*pmsg = *msg;
		pmsg->width = meas3[0].size()+kalman3[0].size();
		pmsg->height = 1;

		for (int i = 0; i < meas3[0].size(); i++){

			pcl::PointXYZRGB point1;
			point1.x = meas3[0][i].x; point1.y = meas3[0][i].y; point1.z = meas3[0][i].z;
			point1.g = 255;
			pmsg->points[i]=(point1);

		}
		for (int i = 0; i < kalman3[0].size(); i++){

			pcl::PointXYZRGB point1;
			point1.x = kalman3[0][i].x; point1.y = kalman3[0][i].y; point1.z = kalman3[0][i].z;
			point1.r = 255;
			pmsg->points[i+meas3[0].size()]=(point1);

		}

		pcl_conversions::toPCL(ros::Time::now(), pmsg->header.stamp);

		pub_.publish(pmsg);
	}


	if(!kalmanv[3].empty()){
		srv.request.velocity = kalmanv[3].back();
		subf.velocity = kalmanv[3].back();
	}
	else{
		srv.request.velocity = 999;
		subf.velocity = 999;
	}

	if(!kalman3[3].empty()){
		srv.request.glovez = kalman3[3].back().z;
		subf.glovez = kalman3[3].back().z;
	}

	srv.request.distance = min_dist;
	subf.distance = min_dist;


//	motion_features_pub_.publish(subf);
	if (client_.call(srv)){
//		ROS_INFO("Response: %ld", (long int)srv.response.sth);
	}
	else{
		ROS_ERROR("Failed to call service feature_processing");
	}


	if(record){
//		vframe++;
//		cvideo.write(cameraFeed);
//
//		cv::putText(cameraFeed, cv::format("%d", vframe ), cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255));
//		cv::putText(cameraFeed, cv::format("%d.%d", (int)subf.rostime.sec, (int)subf.rostime.nsec ), cv::Point(0, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,0,0));
//		video.write(cameraFeed);

		if (myfile.is_open()){
			myfile<<subf.velocity<<","<<subf.distance<<","<<subf.rostime.sec<<"."<<subf.rostime.nsec<<","<<vframe;
			myfile<<endl;
		}
		else cout << "Unable to open file";

//		cv::imshow("record window", cameraFeed);
	}

	cv::imshow("record window", cameraFeed);




	//moving windows
	if(window_pos){
		if(calib){
		    moveWindow("location",0,450);
		    moveWindow(windowName8,710,0);
		    moveWindow("contour",0,0);
		    moveWindow(trackbarWindowName,1000,500);
		}
		else {
		    moveWindow("print",0,0);
		    moveWindow("contour",0,450);
		}

	    window_pos = false;

	}



  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Publisher nined_pub_;
  ros::Publisher motion_features_pub_;
  ros::Subscriber sub_;
  ros::ServiceClient client_;
//  ros::ServiceClient pickandplace_client_;
  ros::ServiceClient tray_upd_client_;
  ros::ServiceClient new_tray_client_;

};//End of class SubscribeAndPublish


int main(int argc, char **argv) 
{	


	if(load_from_file){
		load_settings();
		load_rules();
	}


//	if(rosbag){
	if(calib==false){
//		myfile.open("example.arff");
//		if (myfile.is_open())
//		{
////		myfile << "% 1. Title: example file for ground truth data to train a tree\n\n";
////		myfile << "@relation grasp_or_not\n";
////		myfile << "@attribute 'movement' {'move', 'not move','not detected'}\n";
////		myfile << "@attribute 'objInHand' {'y','n'}\n";
////		myfile << "@attribute 'class' { 'grasping', 'not grasping'}\n";
////		myfile << "@data\n";
//		myfile << "% 1. Title: example file for ground truth data to train a tree\n\n";
//		myfile << "@relation grasp_or_not\n";
//		myfile << "@attribute 'movement' {'move', 'not move','not detected'}\n";
//		myfile << "@attribute 'objInHand' {'y','n'}\n";
//		myfile << "@attribute 'class' { 'grasping', 'not grasping'}\n";
//		myfile << "@data\n";
//		}
//		else cout << "Unable to open file";
		myfile.open(path+"/records/full_data_set.csv");

	}else{
		createTrackbars();
	}

	init_KF(7);



	ros::init(argc, argv, "obj_tracking");
//	ros::NodeHandle nh;
////	image_transport::ImageTransport it(nh);
////	image_transport::Subscriber sub = it.subscribe(RGB_IMG_TOPIC_NAME, 1, imageCallback);
////	image_transport::Subscriber subd = it.subscribe(DEPTH_TOPIC_NAME, 1, depthCallback);
//	ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth_registered/points", 1, cloud_cb);

//	message_filters::Subscriber<sensor_msgs::Image> sub(nh, RGB_IMG_TOPIC_NAME, 10);
//	message_filters::Subscriber<sensor_msgs::Image> subd(nh, DEPTH_TOPIC_NAME, 10);
//	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;
//
//		//	   ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
//	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub, subd);
//	sync.registerCallback(boost::bind(&imageCallback, _1, _2));

//	message_filters::Subscriber<sensor_msgs::PointCloud2> subp(nh, "/camera/depth_registered/points", 10);
//	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub, subd,subp);
//	sync.registerCallback(boost::bind(&imageCallback, _1, _2, _3));
	SubscribeAndPublish SAPObject;


	ros::spin();
	return 0;
}


