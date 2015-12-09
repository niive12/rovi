// open cv libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>

#include <rw/common/Log.hpp>

#include <vector>
#include <algorithm>

#define RAD_45 0.78539816 //45 degrees in radians

namespace featureextraction {

std::vector<cv::Point> find_circles(cv::Mat &image, int min_area = 500, int max_area = 60000);

void separateChannels (cv::Mat img);

bool findMarker01(cv::Mat img, std::vector<cv::Point> &points);

}
