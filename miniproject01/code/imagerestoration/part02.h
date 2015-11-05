#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "areas.h"
#include "histogram.h"
#include "median_filter.h"

#pragma once

void part02(cv::Mat image, cv::Mat out);
