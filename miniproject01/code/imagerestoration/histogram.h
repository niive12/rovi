#ifndef HISTOGRAM_H
#define HISTOGRAM_H

void make_histogram(cv::Mat &input, cv::Mat &hist_output, cv::Mat &histogram, int hist_width=512, int hist_height=512);

void make_histogram_equalization(cv::Mat &image, cv::Mat &histogram);

void applyHistogramEqualization(cv::Mat &img_src, cv::Mat &img_out, int border = 5);
#endif // HISTOGRAM_H

