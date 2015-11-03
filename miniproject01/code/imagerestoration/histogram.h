#ifndef HISTOGRAM_H
#define HISTOGRAM_H

void make_histogram(cv::Mat &input, cv::Mat &output, cv::Mat &histogram, int hist_width=512, int hist_height=512);

void make_histogram_equalization(cv::Mat &image, cv::Mat &histogram);

#endif // HISTOGRAM_H

