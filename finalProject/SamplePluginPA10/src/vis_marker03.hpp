#pragma once

// open cv libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp> //SIFT
#include <opencv2/calib3d/calib3d.hpp> //findHomography

#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>
//to meassure timing
#include <ratio>
#include <chrono>

#include <rw/common/Log.hpp>



namespace featureextraction {

double get_area(std::vector<cv::Point2f> &corners);

void init_marker03(cv::Mat &img_object);

bool findMarker03(const cv::Mat &img_scene, std::vector<cv::Point> &points, bool locate_one_point = true, bool constraintTime = false, double maxprocessingtime = 0);

void get_homography_flann(cv::Mat &H, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors, std::vector<cv::DMatch> &good_matches);

void get_marker_descriptors(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);


}
