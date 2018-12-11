#ifndef LOCALIZER_H
#define LOCALIZER_H

//standard includes
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <vector>


//opencv includes
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/objdetect.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
//ros includes
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#pragma GCC diagnostic pop

int MAX_FEATURES = 500;
const float GOOD_MATCH_PERCENT = 0.001f;

class Localizer
{
public:
    Localizer(ros::NodeHandle& nh);

    //Declare Variables
    std::string mapImgDIR;
    cv::Mat mapImg, templ, mapHog, grayMapImg, mapORB;
    ros::Publisher pose_estimate;
    geometry_msgs::PoseStamped mavPose;

    int numSegments;
    int frameSize_x;
    int frameSize_y;
    int blockSize;
    int blockStride;
    int cellSize;
    int gradientBinSize;
    int camRes_x;
    int camRes_y;

    float h_FOV;
    float v_FOV;
    float mapScale;

    struct ORB_Features 
	{
	    //Orb Variables
	    std::vector<cv::KeyPoint> keypoints;
	    cv::Mat descriptors;
	};

    struct mapFrame //map frame segment of the google map image 
	  {
      int px;
      int py;
		//Hog variables
	    std::vector<float> descriptors;
            //std::vector<Point> locations;
	    cv::HOGDescriptor hog;
            //Orb Variables
	    Localizer::ORB_Features features;
	    	
            cv::Mat visFrame;
	    cv::Mat imgFrame;
	    
	  };

    struct onboardImgFrame //May be removed in future. Stores same variables ass mapFrame
	  {
      int px;
      int py;
	    std::vector<float> descriptors;
            //std::vector<Point> locations;
            cv::Mat visFrame;
	    cv::Mat imgFrame;
	    cv::HOGDescriptor hog;
	  };

    std::vector<mapFrame> frames;	//Vector of map frames to srtore segmented map frames

    //Function Declarations
    void loadMap();
    void segmentMap();
    cv::Mat getHogImg(cv::Mat& im);
    void callback(const sensor_msgs::ImageConstPtr& rect_msg, const nav_msgs::Odometry::ConstPtr& odom_msg, Localizer &localizer_ptr);
    cv::Mat getHogVis(cv::Mat& origImg, std::vector<float>& descriptorValues);
    Localizer::ORB_Features getOrbFeatures(cv::Mat& im);

    void _ORB_Feature_Matcher(struct ORB_Features onboardFeatures, cv::Mat& im);

private:
    ros::NodeHandle nh_;

};
#endif // LOCALIZER_H
