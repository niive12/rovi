#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "areas.h"
#include "histogram.h"
#include "mean_filter.h"

#pragma once

void part03(cv::Mat_<float> &original_image,
            cv::Mat_<float> &out_hist_normal,
            cv::Mat_<float> &out_image_geo,
            cv::Mat_<float> &out_image_har,
            cv::Mat_<float> &out_image_ari,
            cv::Mat_<float> &out_hist_uni_geo,
            cv::Mat_<float> &out_hist_uni_har,
            cv::Mat_<float> &out_hist_uni_ari,
            cv::Mat_<float> &out_hist_post,
            cv::Mat_<float> &out_hist_pre,
            cv::Mat_<float> &out_im_harVSgeo);
