#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



// the butter function calculator
float butter(int u, int v, float D0, int n, int P, int Q);

// makes the butterworth mask for a shifted dft by multiplying the coefficient to the dst
void butterFilter(cv::Mat_<float> &dst, int u, int v, float D0, int n);
