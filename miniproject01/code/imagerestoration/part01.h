#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "areas.h"
#include "histogram.h"
#include "median_filter.h"

#pragma once

void part01(cv::Mat_<float> &original_image, cv::Mat_<float> &output_image);

double what_is_the_S_P_damage(cv::Mat &img, std::string frame="none");
