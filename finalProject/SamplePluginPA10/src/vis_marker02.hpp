#pragma once

// open cv libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>

#include <vector>
#include <algorithm>
#include <chrono>
#include <ratio>

#include <rw/common/Log.hpp>


namespace featureextraction {

std::vector<cv::Point> find_blobs(cv::Mat img, std::vector<std::vector<cv::Point> > &contours, std::vector<std::vector<cv::Point> > &good_contours);

cv::Point find_center(cv::Mat &org, std::vector<cv::Point> &center, std::vector<std::vector<cv::Point> > &contours);

bool findMarker02(cv::Mat &img, std::vector<cv::Point> &points, bool constraintTime = false, double maxprocessingtime = 0);


}
