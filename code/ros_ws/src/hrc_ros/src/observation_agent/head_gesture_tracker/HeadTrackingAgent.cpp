/*
 *  Created on: 25.04.2019
 *      Author: Elia Kargruber
 */


/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/


#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

#include <iostream>

// ########## includes for ros 
#include <ros/ros.h>
#include <stdlib.h> 				// for rand() and RAND_MAX
#include <string>
#include <hrc_ros/HeadGestureMsg.h>



using namespace std;
using namespace cv;

int global_former_id = 0; // 5= left | 6= middle | 7= right
bool show_image_flag = false; 

namespace {
const char* about = "Basic marker detection";
const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 1     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{r        |       | show rejected candidates too }"
        "{refine   |       | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1,"
        "CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}";
}

namespace {
const char* about_1 = "Create an ArUco marker image";
const char* keys_1  =
        "{@outfile |<none> | Output image }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{id       |       | Marker id in the dictionary }"
        "{ms       | 200   | Marker size in pixels }"
        "{bb       | 1     | Number of bits in marker borders }"
        "{si       | false | show generated image }";
}

/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}



/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

void createMarkers(int argc, char *argv[]){
	CommandLineParser parser(argc, argv, keys_1);
	parser.about(about_1);

	if(argc < 4) {
		parser.printMessage();
		return;
	}

	int dictionaryId = parser.get<int>("d");
	int markerId = parser.get<int>("id");
	int borderBits = parser.get<int>("bb");
	int markerSize = parser.get<int>("ms");
	bool showImage = parser.get<bool>("si");

	String out = parser.get<String>(0);

	if(!parser.check()) {
		parser.printErrors();
		return;
	}

	Ptr<aruco::Dictionary> dictionary =
		aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	Mat markerImg;
	aruco::drawMarker(dictionary, markerId, markerSize, markerImg, borderBits);

	if(showImage) {
		imshow("marker", markerImg);
		waitKey(0);
	}

	imwrite(out, markerImg);

	return;
}



void detectMarkers(int argc, char *argv[]){

	ros::init(argc,argv, "HeadTrackingNode");
	ros::NodeHandle nh;
	//ros::CallbackQueue my_queue;  

	//ros::AsyncSpinner spinner(12 /*number of threads*/, &my_queue /* spinner exclusively for my_queue */); 
	
	// bind the queue to the node handle
	//nh.setCallbackQueue( &my_queue );

	// ################# Services, subscribers publishers ... 

	// define publishers ... 
	ros::Publisher headGesture_pub = nh.advertise<hrc_ros::HeadGestureMsg>("/headTrackingAgent/head_gesture_pub/",1);

	CommandLineParser parser(argc, argv, keys);
	parser.about(about);

	if(argc < 2) {
		parser.printMessage();
		return;
	}

	int dictionaryId = parser.get<int>("d");
	bool showRejected = parser.has("r");
	bool estimatePose = parser.has("c");
	float markerLength = parser.get<float>("l");

	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	if(parser.has("dp")) {
		bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
		if(!readOk) {
			cerr << "Invalid detector parameters file" << endl;
			return;
		}
	}

	if (parser.has("refine")) {
		//override cornerRefinementMethod read from config file
		detectorParams->cornerRefinementMethod = parser.get<int>("refine");
	}
	std::cout << "Corner refinement method (0: None, 1: Subpixel, 2:contour, 3: AprilTag 2): " << detectorParams->cornerRefinementMethod << std::endl;

	int camId = parser.get<int>("ci");

	String video;
	if(parser.has("v")) {
		video = parser.get<String>("v");
	}

	if(!parser.check()) {
		parser.printErrors();
		return;
	}

	Ptr<aruco::Dictionary> dictionary =
		aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	Mat camMatrix, distCoeffs;
	if(estimatePose) {
		bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
		if(!readOk) {
			cerr << "Invalid camera file" << endl;
			return;
		}
	}

	VideoCapture inputVideo;
	int waitTime;
	if(!video.empty()) {
		inputVideo.open(video);
		waitTime = 0;
	} else {
		inputVideo.open(camId);
		waitTime = 10;
	}

	double totalTime = 0;
	int totalIterations = 0;

	//spinner.start();
	//ros::waitForShutdown();

	while(inputVideo.grab() && nh.ok()) {
		Mat image, imageCopy;
		inputVideo.retrieve(image);

		double tick = (double)getTickCount();

		vector< int > ids;
		vector< vector< Point2f > > corners, rejected;
		vector< Vec3d > rvecs, tvecs;

		// detect markers and estimate pose
		aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
		if(estimatePose && ids.size() > 0)
			aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
											 tvecs);

		double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
		totalTime += currentTime;
		totalIterations++;
		if(totalIterations % 30 == 0) {
			//cout << "Detection Time = " << currentTime * 1000 << " ms "
				// << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
		}

		if ( show_image_flag == true){ 
			// draw results
			image.copyTo(imageCopy);
			if(ids.size() > 0) {
				aruco::drawDetectedMarkers(imageCopy, corners, ids);

				if(estimatePose) {
					for(unsigned int i = 0; i < ids.size(); i++)
						aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
										markerLength * 0.5f);
				}
			}

			if(showRejected && rejected.size() > 0)
				aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

			imshow("out", imageCopy);
			char key = (char)waitKey(waitTime);
			if(key == 27) break;
		}


		

		// publish what has been detected 

		if(ids.size() > 0){ // if an ID has been detected 
			
			if (ids.at(0) != global_former_id){
				global_former_id = ids.at(0); 
				 
				hrc_ros::HeadGestureMsg hg_msg; 
				
				// assign human looking around 
				if(ids.at(0)==5 || ids.at(0)==7){ // one of the side markers is detected -> looking around 
					hg_msg.humanLookingAround = true; 
					
					// fill in details about the head gesture 
					if(ids.at(0)==5){
						hg_msg.headGestureDetails = string("left");
					} else if (ids.at(0) == 7){
						hg_msg.headGestureDetails = string("right");
					}
					//cout << "Human looking around" << endl; 
				} else { 
					hg_msg.humanLookingAround = false; 
					hg_msg.headGestureDetails = string("center");
					//cout << "NOT looking around" << endl;
				}

				hg_msg.stamp = ros::Time::now(); 
				headGesture_pub.publish(hg_msg);

				// publisch global_former_id !!! 
			} else { //cout << "same ID as before"; 
			}

		}
		
		ros::spinOnce();
	}

	return;
}

/**
 */
int main(int argc, char *argv[]) {
    //createMarkers(argc, argv);
    detectMarkers(argc, argv);
}

