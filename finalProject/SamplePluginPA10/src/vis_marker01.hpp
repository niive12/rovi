#pragma once

// open cv libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>

#include <rw/common/Log.hpp>

#include <vector>
#include <algorithm>

#define RAD_45 0.78539816 //45 degrees in radians

namespace featureextraction {

std::vector<cv::Point> find_circles(const cv::Mat &image, size_t max_circles, int min_area = 500, int max_area = 60000);

bool findMarker01(const cv::Mat &img, std::vector<cv::Point> &points);

}

struct comparator_functor {
  bool operator() (const std::vector<cv::Point> a, const std::vector<cv::Point> b) const {
    return a.size() > b.size();
  }
};
