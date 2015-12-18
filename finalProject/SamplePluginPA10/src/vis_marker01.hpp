#pragma once

// open cv libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>

#include <chrono>
#include <ratio>

#include <vector>
#include <algorithm>

#include <rw/common/Log.hpp>

namespace featureextraction {

bool findMarker01(const cv::Mat &img, std::vector<cv::Point> &points, bool locate_one_point = true, bool constraintTime = false, double maxprocessingtime = 0);

}
