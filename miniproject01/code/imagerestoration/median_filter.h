#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

void median_filter(const cv::Mat &input, cv::Mat &output, int kernel_size = 3, int mean_width = 1, double offset = 0.5);

#endif // MEDIAN_FILTER_H

