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

int number_of_lines = 10;
int threshold_angle = 15;
int threshold_dist = 2;

cv::Mat org;
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
    image.setTo(0);
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
}

std::vector<cv::Vec2f> find_lines_hough(const cv::Mat &edges, int hough_votes){
    std::vector<cv::Vec2f> lines;
    if(hough_votes < 1){
        hough_votes = 1;
    }
    cv::HoughLines(edges, lines, 1, CV_PI /180,100, 0,0);
    if(lines.size() > hough_votes){
        lines.erase(lines.begin() + hough_votes, lines.end());
    }
    return std::move(lines);
}

std::vector<cv::Vec2f> find_marker_lines(const std::vector<cv::Vec2f> &lines, std::vector<cv::Point> &intersections){
    bool debug_img = true;
    const float tol_par = 2 * CV_PI / 180;
    const float tol_cross = (90) * CV_PI / 180;
    float other_angle;
    float other_dist;

    std::vector<cv::Vec2f> good_lines;
    for( size_t line = 0; line < lines.size(); ++line){
        float theta = lines[line][1];
        char n_parallel = 0;
        char n_crossing = 0;
        for( size_t i = 0; i < lines.size(); ++i){
            other_angle = theta - (lines[i][1]);
            other_dist = lines[line][0] - lines[i][0];
//            std::cout << other_dist << std::endl;
            if(other_angle > (-tol_par) && other_angle <= tol_par) {
                ++n_parallel;
            } else if( (other_angle > (tol_cross - tol_par) && other_angle < (tol_cross + tol_par)) || (other_angle > (-tol_cross - tol_par) && other_angle < (-tol_cross + tol_par)) )  {
                    ++n_crossing;
            }
        }
        if( ( n_parallel > 1 && n_crossing > 0) ){
            good_lines.push_back(lines[line]);
        }
    }

    if(debug_img){
        for(size_t i = 0; i < good_lines.size(); ++i){
            for(size_t j = i+1; j < good_lines.size(); ++j){
                other_angle = good_lines[i][1] - good_lines[j][1];
                if( (other_angle > (tol_cross - tol_par) && other_angle < (tol_cross + tol_par)) || (other_angle > (-tol_cross - tol_par) && other_angle < (-tol_cross + tol_par)) )  {
                    cv::Point intersection(cos(good_lines[i][1])*good_lines[i][0] + cos(good_lines[j][1])*good_lines[j][0],
                                           sin(good_lines[i][1])*good_lines[i][0] + sin(good_lines[j][1])*good_lines[j][0]);
                    intersections.push_back(intersection);
                }
            }
        }
    }

    return std::move(good_lines);
}

bool find_center(std::vector<cv::Point> &intersections, std::vector<cv::Point> &points, cv::Mat &lines_img, bool find_single_point = true){
    cv::Point midpoint(0,0);
    std::vector<cv::Point> corners;
    char circles = 0;
    float a, b, dist;
    for(size_t i = 0; i < intersections.size(); ++i){
        for(size_t j = i+1; j < intersections.size(); ++j){
            a = intersections[i].x - intersections[j].x;
            b = intersections[i].y - intersections[j].y;
            dist = a*a + b*b;
            if( dist > 0 && dist < 30){
                intersections.erase(intersections.begin()+j);
            }
        }
    }

    for(auto i : intersections){
        float max = 0,min = 1e6;
        for(auto p : intersections){
            a = i.x - p.x;
            b = i.y - p.y;
            dist = a*a + b*b;
            if( dist > 0 && dist < min){
                min = dist;
            } else if( dist > max){
                max = dist;
            }

        }
        if(max > 160000){
            corners.push_back(i);
            cv::circle(lines_img,i,30,cv::Scalar(0,255,0));
            midpoint.x += i.x;
            midpoint.y += i.y;
        }
    }
    if( corners.size() >= 4){
        midpoint.x = midpoint.x / corners.size();
        midpoint.y = midpoint.y / corners.size();
        if(find_single_point){
            points.push_back(midpoint);
        } else {
            for(auto i : corners){
                points.push_back(i);
            }
        }
        return true;
    } else {
        std::cout << "no solution. " << corners.size() << std::endl;
        points.push_back(midpoint);
        return false;
    }
}

bool findMarker02(const cv::Mat &img, std::vector<cv::Point> &points, bool locate_one_point = true){
    cv::Mat edges;
    cv::Mat lines_img = img.clone();
    lines_img.setTo(0);
    double th_low = 100, th_high = 300;
    cv::Canny(img, edges, th_low, th_high);
//    edges = img.clone();
//    cv::cvtColor(img, edges, CV_BGR2GRAY);

    std::vector<cv::Vec2f> Hough_lines = find_lines_hough(edges, number_of_lines);
    print_lines(Hough_lines,lines_img);
    std::vector<cv::Point> intersections;
    std::vector<cv::Vec2f> lines = find_marker_lines(Hough_lines,intersections);
    print_lines(lines,lines_img);
    bool ans = find_center(intersections, points, lines_img, locate_one_point);
    cv::imshow(window_name,lines_img);
    return ans;
}


void on_trackbar(int, void*){
}



int image_in_set = 7;
std::vector<cv::Mat> original_images;
void image_trackbar(int, void*){
    org = original_images.at(image_in_set).clone();
    std::vector<cv::Point> points;
    bool ans = findMarker02(org, points);
    if(ans){
        cv::circle(org,points[0],100,cv::Scalar(255,255,255));
    }
    cv::imshow("original", org);
}

int main(int argc, char* argv[]){
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
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
//            std::cout << filename << "\n";
            original_images.push_back( cv::imread(filename) );
        }
    }
    cv::createTrackbar("Selected image", "original",&image_in_set,original_images.size()-1,image_trackbar);
    image_trackbar(0,nullptr);

    cv::createTrackbar("Hough votes", window_name,&number_of_lines,255,on_trackbar);
    cv::createTrackbar("Line dist", window_name,&threshold_dist,100,on_trackbar);
    cv::createTrackbar("Line angle", window_name,&threshold_angle,45,on_trackbar);
    cv::waitKey();
    return 0;
}
