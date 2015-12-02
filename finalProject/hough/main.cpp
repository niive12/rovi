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


void on_trackbar(int, void*){
    lines_img.setTo(0);
    std::vector<cv::Vec2f> lines;
    if(line_threshold < 1){
        line_threshold = 1;
    }
    cv::HoughLines(edges, lines, 1, CV_PI /180, line_threshold, 0,0);
    const float tol_par = 0.0001 * CV_PI / 180;
    const float tol_cross = (0.0001 + 90) * CV_PI / 180;
    std::cout << tol_par;
    float other_angle, other_dist;
    for( size_t line = 0; line < lines.size(); ++line){
        float rho = lines[line][0], theta = lines[line][1];
        char n_parallel = 0;
        char n_crossing = 0;
        char n_same = 0;
        for( size_t i = line; i < lines.size(); ++i){
            other_angle = theta - lines[i][1];
            other_dist  = rho - lines[i][0];
            if(other_dist < -5 || other_dist > 5){
                ++n_same;
            }
            if(other_angle > (-1 * tol_par) && other_angle <= tol_par) {
                ++n_parallel;
            }
            if(other_angle > (-1 * tol_cross) && other_angle <= tol_cross) {
                ++n_crossing;
            }
            if(n_parallel > 2 && n_parallel < 5 && n_crossing > 15 && n_crossing < 25 && n_same < 25){
                std::cout << other_dist << " " << (int) n_same << " " <<  (int) n_parallel << " " << (int) n_crossing << std::endl;
                //                std::cout << other_angle << '\n';
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
    }
    cv::imshow(window_name,lines_img);
}


int main(int argc, char* argv[]){
    cv::Mat org;
    std::vector<cv::Vec2f> lines;
    if(argc == 2){
        org = cv::imread(argv[1]);
    } else {
//        org = cv::imread("../SamplePluginPA10/markers/Marker1_modified.ppm");
//        org = cv::imread("../SamplePluginPA10/markers/Marker2a.ppm");
        //        org = cv::imread("../marker_color_hard/marker_color_hard_18.png");
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
    on_trackbar(100,nullptr);
    cv::waitKey();
    return 0;
}
