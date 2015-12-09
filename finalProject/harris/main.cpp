// open cv libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <queue>

#include <vector>
#include <algorithm>

#define RAD_45 0.78539816 //45 degrees in radians

struct sort_by_x_func {
    bool operator() (const cv::Point a, const cv::Point b) const {
        return a.x < b.x; //for min heap
    }
} sort_by_x;

struct sort_by_y_func {
  bool operator() (const cv::Point a, const cv::Point b) const {
    return a.y < b.y; //for min heap
  }
} sort_by_y;


cv::Mat src, src_gray;
int threshold = 94;
int max_thresh = 255;
int kernel_size_neighbor = 45;

std::string source_window = "Source image";
std::string window_name = "Corners detected";

void on_trackbar(int, void*){
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros( src_gray.size(), CV_32FC1 );

    /// Detector parameters
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;

    /// Detecting corners
    cv::cornerHarris( src_gray, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT );

    /// Normalizing
    cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
//    cv::convertScaleAbs( dst_norm, dst_norm_scaled );
    dst_norm_scaled = dst_norm.clone();

    int kernel_size = 2; // 5x5

    // Merge points
//    std::priority_queue<cv::Point,std::vector<cv::Point>,sort_by_x> corners;
    std::vector<cv::Point> corners;
    for( int j = kernel_size; j < dst_norm.rows-kernel_size; ++j){
        for( int i = kernel_size; i < dst_norm.cols-kernel_size; ++i ){
            if( dst_norm_scaled.at<float>(j,i) > threshold ){
                corners.push_back(cv::Point(i,j)); //add corner and remove neighbors in a 5x5 kernel
                for(int dy = -kernel_size; dy <= kernel_size; ++dy){ //
                    for( int dx = -kernel_size; dx <= kernel_size; ++dx){
                        if( (dy != 0 || dx != 0) ){//&& dst_norm_scaled.at<float>(j+dy,i+dx) > threshold ){
                            dst_norm_scaled.at<float>(j+dy,i+dx) = 0;
                        }
                    }
                }
            }
        }
    }
    std::sort(corners.begin(), corners.end(), sort_by_x);
    cv::threshold(dst_norm_scaled, dst_norm, 94, 254,cv::THRESH_BINARY);
    dst_norm_scaled = dst_norm.clone();


    std::vector<cv::Point> crossings_x;
    std::vector<cv::Point> crossings_y;
//    kernel_size_neighbor = 50;
    int desired_distance_min = 6;
    int desired_distance_max = 13;
    double dist = 0;
    /* Drawing a circle around corners
    for( int j = kernel_size_neighbor; j < dst_norm.rows - kernel_size_neighbor; ++j){
        for( int i = kernel_size_neighbor; i < dst_norm.cols - kernel_size_neighbor; ++i ){
            int neighbor_cnt = 0;
            if( dst_norm_scaled.at<float>(j,i) >= threshold ){
                for(int dy = -kernel_size_neighbor; dy <= kernel_size_neighbor; ++dy){
                    for( int dx = -kernel_size_neighbor; dx <= kernel_size_neighbor; ++dx){
                        if(dy != 0 || dx != 0){
                            if(dst_norm_scaled.at<float>(j+dy,i+dx) >= threshold){
                                dist = sqrt(dy*dy + dx*dx);
                                if( dist > desired_distance_min && dist <= desired_distance_max){
                                    ++neighbor_cnt;
                                } else if( dist > desired_distance_max){
                                    neighbor_cnt = 10;
                                    break;
                                }
                            }
                        }
                    }
                }
                if( neighbor_cnt >= 1 && neighbor_cnt <= 4 ){
                    crossings_x.push_back(cv::Point(i,j));
                    crossings_y.push_back(cv::Point(i,j));
                    cv::circle( dst_norm, cv::Point( i, j ), kernel_size_neighbor,  cv::Scalar(255), 1, 8, 0 );
                }
            }
        }
    }

    /*/

    //alternative method
    int neighbor_cnt, dx,dy;

    for(auto i : corners){
        neighbor_cnt = 0;
        for(auto n : corners){
            dx = i.x - n.x;
            dy = i.y - n.y;
            if( dx > -kernel_size_neighbor){
                if( dx < kernel_size_neighbor){
                    dist = sqrt(dy*dy + dx*dx);
                    if( dist > desired_distance_min && dist <= kernel_size_neighbor){
                        if(dist > desired_distance_max){
                            neighbor_cnt = 10; // I don't want neighbours that are ouside my desired region and inside my kernel
                        }
                        ++neighbor_cnt;
                    }
                    if(i.x == 585 && i.y==405 && n.x==595 && n.y==411){
                        std::cout << dist << "\t" << neighbor_cnt << '\n';
                    }
                }
            } else {
                break;
            }
        }
        if( neighbor_cnt > 1 && neighbor_cnt <= 4 ){
            crossings_x.push_back(i);
            crossings_y.push_back(i);
            cv::circle( dst_norm, i, kernel_size_neighbor,  cv::Scalar(255), 1, 8, 0 );
        }
    }

    //*/
    std::sort(crossings_x.begin(), crossings_x.end(), sort_by_x);
    std::sort(crossings_y.begin(), crossings_y.end(), sort_by_y);
    if( !crossings_x.empty() ){
        cv::Point p( ( crossings_x.front().x + crossings_x.back().x )/2, ( crossings_y.front().y + crossings_y.back().y )/2 );
        std::cout << p << std::endl;
        cv::circle( dst_norm, p, 15,  cv::Scalar(255), 3, 8, 0 );
    }
    for(auto i : crossings_x){
        std::cout << i << '\n';
    }

    /// Showing the result
    cv::namedWindow( window_name, CV_WINDOW_AUTOSIZE );
    cv::imshow( window_name, dst_norm );
}


int main(int argc, char* argv[]){
    cv::Mat org;
    std::vector<cv::Vec2f> lines;
    if(argc == 2){
        org = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    } else {
        org = cv::imread("../marker_thinline/marker_thinline_18.png",CV_LOAD_IMAGE_GRAYSCALE);
    }
//    cv::imshow("org", org);

    src_gray = org.clone();

    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::createTrackbar("Threshold", window_name,&kernel_size_neighbor,100,on_trackbar);
    on_trackbar(0,nullptr);
    cv::waitKey();
    return 0;
}
