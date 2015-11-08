#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "areas.h"
#include "histogram.h"
#include "median_filter.h"
#include "mean_filter.h"

#pragma once

void part02(cv::Mat &image,
            cv::Mat &final_out,
            cv::Mat &hist_uniform,
            cv::Mat &hist_median,
            cv::Mat &hist_bilatteral,
            cv::Mat &complex_median,
            cv::Mat &complex_harmonic,
            cv::Mat &complex_bilatteral,
            cv::Mat &complex_histeq,
            cv::Mat &complex_smoothed_histeq
            );
