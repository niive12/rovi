#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "areas.h"
#include "histogram.h"
#include "median_filter.h"

#pragma once

void part01(cv::Mat &original_image,
            cv::Mat &output_image,
            cv::Mat &hist_uniform,
            cv::Mat &hist_median,
            cv::Mat &complex_median,
            cv::Mat &complex_geometric_blurred,
            cv::Mat &complex_harmonic_blurred,
            cv::Mat &complex_difference
            );
