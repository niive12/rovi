#ifndef HISTOGRAM_H
#define HISTOGRAM_H

void make_histogram(cv::Mat &input, cv::Mat &hist_output, cv::Mat &histogram, int hist_width=512, int hist_height=256);

void make_histogram_equalization(cv::Mat &image, cv::Mat &histogram);

void applyHistogramEqualization(cv::Mat &img_src, cv::Mat &img_out, int border = 5);

void stat(const cv::Mat &img_src, cv::Mat &histImage, double &mean, double &var, int border=5);

double what_is_the_S_P_damage(cv::Mat &img, std::string frame="none");

#endif // HISTOGRAM_H

