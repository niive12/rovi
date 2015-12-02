// open cv libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>

#include <vector>
#include <algorithm>

#define RAD_45 0.78539816 //45 degrees in radians

struct image_in_image_out{
    cv::Mat *src;
    cv::Mat *dst;
};

int line_threshold = 94;

cv::Mat edges, lines_img;
std::string window_name = "Lines";

//taken from 5th semester cupcollector / R2D2
void merge_similar_lines(std::vector<cv::Vec2f> &lines){
    const float threshold_angle = 15 * CV_PI / 180;
    const float threshold_dist = 30;
    bool changeHappened = true;
    while(changeHappened){
        changeHappened = false;
        // iterate through the whole vector and compare all (start over if two were combined)
        for(size_t i = 0; i < lines.size(); ++i){
            for(size_t j = (i + 1); j < lines.size(); ++j){
                // get deviation of the two lines
                float deviationOfDistance = abs(abs(lines[i][0]) - abs(lines[j][0]));
                float deviationOfAngle = abs(abs(lines[i][1]) - abs(lines[j][1]));
                // if appropiate to merge, do so
                if(deviationOfAngle < threshold_angle && deviationOfDistance < threshold_dist){
                    changeHappened = true;
                    // merge datasets
                    lines[i][0] = (lines[i][0] + lines[j][0])/2;
                    lines[i][1] = (lines[i][1] + lines[j][1])/2;
                    // remove old dataset and line
                    lines.erase(lines.begin() + j);
                }
            }
        }
    }
}

void on_trackbar(int, void*){
    lines_img.setTo(0);
    std::vector<cv::Vec2f> lines;
    if(line_threshold < 1){
        line_threshold = 1;
    }
    cv::HoughLines(edges, lines, 1, CV_PI /180, line_threshold, 0,0);
    const float tol_par = 5 * CV_PI / 180;
    const float tol_cross = (90) * CV_PI / 180;
    float other_angle;
    merge_similar_lines(lines);

    for( size_t line = 0; line < lines.size(); ++line){
        float rho = lines[line][0], theta = lines[line][1];
        char n_parallel = 0;
        char n_crossing = 0;
        for( size_t i = 0; i < lines.size(); ++i){
            other_angle = theta - (lines[i][1]);
            if(other_angle > (-tol_par) && other_angle <= tol_par) {
                ++n_parallel;
            }
            if( (other_angle > (tol_cross - tol_par) && other_angle < (tol_cross + tol_par)) || (other_angle > (-tol_cross - tol_par) && other_angle < (-tol_cross + tol_par)) )  {
                ++n_crossing;
            }
        }
        if( ( n_parallel > 2 && n_parallel <= 4 && n_crossing >= 3 && n_crossing <= 4) ){
            cv::Point pt1, pt2;
            double C = cos(theta), S = sin(theta);
            double x0 = C*rho, y0 = S*rho;
            pt1.x = cvRound(x0 + 100*(-S));
            pt1.y = cvRound(y0 + 100*(C));
            pt2.x = cvRound(x0 - 1000*(-S));
            pt2.y = cvRound(y0 - 1000*(C));
            cv::line( lines_img, pt1, pt2, cv::Scalar(0,0,255), 1, CV_AA);
        }
    }
    cv::imshow(window_name,lines_img);
}


int main(int argc, char* argv[]){
    cv::Mat org;
    std::vector<cv::Vec2f> lines;
    if(argc == 2){
        org = cv::imread(argv[1]);
    } else {
        org = cv::imread("../marker_thinline/marker_thinline_18.png");
    }
    double th_low = 100, th_high = 300;

    cv::imshow("org", org);
    cv::Canny(org, edges, th_low, th_high);

    cv::imshow("org", edges);

    cv::waitKey();

    lines_img = org.clone();

    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::createTrackbar("hough votes", window_name,&line_threshold,255,on_trackbar);
    on_trackbar(0,nullptr);
    cv::waitKey();
    return 0;
}
