#pragma once

// open cv libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp> //SIFT
#include "opencv2/calib3d/calib3d.hpp" //findHomography

#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>
//to meassure timing
#include <ratio>
#include <chrono>

#include <rw/common/Log.hpp>

#define RAD_45 0.78539816 //45 degrees in radians

// TODO FIRST!!
//org = original_images.at(image_in_set).clone();
//std::vector<cv::Point> points;
//int x = 0,y = 0;
//accepted_width = org.cols;
//accepted_height = org.rows;
//if( old_position != cv::Point(0,0) ){
//    accepted_width = img_object.cols * 2;
//    accepted_height = img_object.rows * 2;
//    x = old_position.x - accepted_width/2;
//    y = old_position.y - accepted_height/2;
//    if(x > (org.cols-accepted_width) ){
//        x = org.cols-accepted_width;
//    } else if(x < 0){
//        x = 0;
//    }
//    if(y > (org.rows-accepted_height) ){
//        y = org.rows-accepted_height;
//    } else if(y < 0){
//        y = 0;
//    }
//}
//cv::Mat cropped(org, cv::Rect(x,y,accepted_width,accepted_height));

//bool ans;
//ans = findMarker03(cropped, points);
//if( !ans ){
//    old_position = cv::Point(0,0);
//} else if(points.size() > 0 ){
//    old_position.x = points[0].x + x;
//    old_position.y = points[0].y + y;
//}




namespace featureextraction {

void init_marker03(cv::Mat &img_object);

bool findMarker03(const cv::Mat &img_scene, std::vector<cv::Point> &points, bool locate_one_point = true, bool constraintTime = false, double maxprocessingtime = 0);

void get_homography_flann(cv::Mat &H, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors, std::vector<cv::DMatch> &good_matches);

void get_marker_descriptors(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);

double get_area(std::vector<cv::Point2f> &corners);


}
