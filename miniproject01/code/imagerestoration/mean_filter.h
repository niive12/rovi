#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// apply different mean filters
// geometric mean. harmonic mean. arithmetic mean.


void applyGeometricMean(cv::Mat_<float> &img_src, cv::Mat_<float> &img_dst, int kernelsize);

void applyHarmonicMean(cv::Mat_<float> &img_src, cv::Mat_<float> &img_dst, int kernelsize);

void applyArithmeticMean(cv::Mat_<float> &img_src, cv::Mat_<float> &img_dst, int kernelsize);
