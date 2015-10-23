#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>

#include "dft_shift.h"


// takes in a real image returns the frequency image
void visualize_frequency(cv::Mat_<float> &original_image, cv::Mat_<float> &output_image);
