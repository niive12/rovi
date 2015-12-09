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

int hough_votes = 94;
int threshold_angle = 15;
int threshold_dist = 30;

cv::Mat org;
cv::Mat edges, lines_img;
std::string window_name = "Lines";

//taken from 5th semester cupcollector / R2D2
void merge_similar_lines(std::vector<cv::Vec2f> &lines, float threshold_dist, float threshold_angle){
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

void print_lines(std::vector<cv::Vec2f> &lines, cv::Mat &image){
    for(auto i: lines){
        cv::Point pt1, pt2;
        float rho = i[0], theta = i[1];

        float C = cos(theta), S = sin(theta);
        float x0 = C*rho, y0 = S*rho;
        pt1.x = cvRound(x0 + 1000*(-S));
        pt1.y = cvRound(y0 + 1000*(C));
        pt2.x = cvRound(x0 - 1000*(-S));
        pt2.y = cvRound(y0 - 1000*(C));
        cv::line( image, pt1, pt2, cv::Scalar(0,0,255), 1, CV_AA);
    }
    cv::imshow(window_name,image);
}

std::vector<cv::Vec2f> find_lines_hough(int hough_votes){
    lines_img.setTo(0);
    std::vector<cv::Vec2f> lines;
    if(hough_votes < 1){
        hough_votes = 1;
    }
    cv::HoughLines(edges, lines, 1, CV_PI /180, hough_votes, 0,0);
    return std::move(lines);
}

std::vector<cv::Vec2f> find_marker_lines(std::vector<cv::Vec2f> lines){
    const float tol_par = 5 * CV_PI / 180;
    const float tol_cross = (90) * CV_PI / 180;
    float other_angle;

    std::vector<cv::Vec2f> good_lines;
    for( size_t line = 0; line < lines.size(); ++line){
        float theta = lines[line][1];
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
            good_lines.push_back(lines[line]);
        }
    }
    return std::move(good_lines);
}

void on_trackbar(int, void*){
    std::vector<cv::Vec2f> Hough_lines = find_lines_hough(hough_votes);
    print_lines(Hough_lines,lines_img);
    cv::imshow("edges", lines_img);
    float threshold_angle_rad = threshold_angle * CV_PI / 180;
    merge_similar_lines(Hough_lines, threshold_dist, threshold_angle_rad);
    std::vector<cv::Vec2f> lines = find_marker_lines(Hough_lines);
    print_lines(lines,lines_img);

}

int image_in_set = 7;
std::vector<cv::Mat> original_images;
void image_trackbar(int, void*){
    org = original_images.at(image_in_set).clone();
    double th_low = 100, th_high = 300;

    cv::imshow("original", org);
    cv::Canny(org, edges, th_low, th_high);

    lines_img = org.clone();
    on_trackbar(0,nullptr);
}

int main(int argc, char* argv[]){
    std::vector<cv::Vec2f> lines;

    cv::namedWindow("original", cv::WINDOW_NORMAL);
    if(argc == 2){
        original_images.push_back( cv::imread(argv[1]) );
    } else {
        std::string filename;
        char numbera, numberb;
        for(int i = 1; i <= 30; ++i){
            numbera = i/10 %10 + '0';
            numberb = i%10 + '0';
            filename = "../marker_thinline/marker_thinline_";
            filename += numbera;
            filename += numberb;
            filename += ".png";
            std::cout << filename << "\n";
            original_images.push_back( cv::imread(filename) );
        }
    }
    cv::createTrackbar("Selected image", "original",&image_in_set,original_images.size()-1,image_trackbar);
    image_trackbar(0,nullptr);
    cv::waitKey();

    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::createTrackbar("Hough votes", window_name,&hough_votes,255,on_trackbar);
    cv::createTrackbar("Line dist", window_name,&threshold_dist,100,on_trackbar);
    cv::createTrackbar("Line angle", window_name,&threshold_angle,45,on_trackbar);
    on_trackbar(0,nullptr);
    cv::waitKey();
    return 0;
}
