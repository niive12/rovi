// open cv libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>

#include <vector>
#include <algorithm>

int image_in_set = 82;
std::vector<cv::Mat> original_images;
cv::Mat org;

//check if at least 3 of the bounding box of a circle is the right color
bool is_circle_near_color(cv::Mat &color, cv::Vec3f &circle){
    int sum = 0;
    int err = 10;

    sum += color.at<uchar>(cv::Point(circle[0]+circle[2]+err, circle[1]+circle[2]+err) );
    sum += color.at<uchar>(cv::Point(circle[0]+circle[2]+err, circle[1]-circle[2]-err) );
    sum += color.at<uchar>(cv::Point(circle[0]-circle[2]-err, circle[1]-circle[2]-err) );
    sum += color.at<uchar>(cv::Point(circle[0]-circle[2]-err, circle[1]+circle[2]+err) );
    return sum > 510;
}

bool findMarker01(const cv::Mat &img, std::vector<cv::Point> &points, bool locate_one_point = true){
    points.clear();
    cv::Mat src_gray;
    cv::Mat drawing = img.clone();

    cv::Mat imghsv;
    cv::cvtColor(org, imghsv, CV_BGR2HSV);
    cv::Mat imgBGR[3];

    int hue_min, hue_max; //range 0 - 180 degrees
    int sat_min = 0.4 * 255, sat_max = 1.0 * 255; // range 0 - 255   radius
    int val_min = 0.10 * 255, val_max = 1 * 255; // range 0 - 255 intensity

    hue_min = 210 / 2 ; hue_max = 270 /2;
    cv::inRange(imghsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max,sat_max,val_max), imgBGR[0]);

    hue_min = 0 / 2; hue_max = 20/ 2;
    cv::inRange(imghsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max,sat_max,val_max), imgBGR[2]);

//    hue_min = 70 / 2; hue_max = 145 / 2;
    hue_min = 70 / 2; hue_max = 175 / 2;
    sat_min = 0.1 * 255;
    sat_max = 1.0 * 255;
    cv::inRange(imghsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max,sat_max,val_max), imgBGR[1]);
    cv::cvtColor( img, src_gray, CV_BGR2GRAY );

    std::vector<cv::Vec3f> red_circles;
    std::vector<cv::Vec3f> blue_circles;
    std::vector<cv::Vec3f> green_circles;

    double min_dist_between_circles = 100;
    double upper_limit_for_canny = 300;
    double threshold_for_center = 14;
    double min_radius_circle = 30; //radius is about 60 (30 on marker.ppm)
    double max_radius_circle = 75;
    cv::HoughCircles( imgBGR[1],
            green_circles,
            CV_HOUGH_GRADIENT,
            1,
            min_dist_between_circles,
            upper_limit_for_canny,
            threshold_for_center,
            min_radius_circle,
            max_radius_circle );

    for( auto i : green_circles ){
        cv::Point center(cvRound(i[0]), cvRound(i[1]));
        int radius = cvRound(i[2]);
        // circle center
        if( imgBGR[2].at<uchar>(center) > 250 ){
            red_circles.push_back(i);
        } else if( imgBGR[0].at<uchar>(center) > 250 ){
            blue_circles.push_back(i);
        }
    }
    float dist, a, b;
    std::vector<cv::Vec3f> circles;
    if( red_circles.size() < 1){
        cv::HoughCircles( imgBGR[2],
                          red_circles,
                          CV_HOUGH_GRADIENT,
                          1,
                          min_dist_between_circles,
                          upper_limit_for_canny,
                          threshold_for_center,
                          min_radius_circle,
                          max_radius_circle );
    }
    if( blue_circles.size() < 3){
        circles = blue_circles;
        cv::HoughCircles( imgBGR[0],
                          blue_circles,
                          CV_HOUGH_GRADIENT,
                          1,
                          min_dist_between_circles,
                          upper_limit_for_canny,
                          threshold_for_center,
                          min_radius_circle,
                          max_radius_circle );
        for(auto i : blue_circles){
            if(is_circle_near_color(imgBGR[1],i)){
                circles.push_back(i);
            }
        }
        for(size_t i = 0; i < circles.size(); ++i){
            cv::Point a(circles[i][0],circles[i][1]);
            for(size_t j = i+1; j < circles.size(); ++j){
                cv::Point b(circles[j][0],circles[j][1]);
                cv::Point d = a - b;
                if(abs(d.x) < 20 && abs(d.y) < 20){
                    circles.erase(circles.begin() + i);
                    --i;
                }
            }
        }
        for(auto i : circles)
            std::cout << i;
        blue_circles = circles;
    }
    cv::Point midpoint(0,0);
    circles = blue_circles;
    for(auto r : red_circles){
        circles.push_back(r);
    }
    for(auto i : circles){
        midpoint.x += i[0];
        midpoint.y += i[1];
    }
    midpoint.x = midpoint.x / circles.size();
    midpoint.y = midpoint.y / circles.size();
//    cv::circle( drawing, midpoint, blue_circles[0][2], cv::Scalar(0,0,255), 10, 8, 0 );

    for(auto i: circles){
        cv::circle( drawing, cv::Point(i[0],i[1]), 3, cv::Scalar(0,0,255), -1, 8, 0 );
        // circle outline
        cv::circle( drawing, cv::Point(i[0],i[1]), i[2], cv::Scalar(255,255,255), 3, 8, 0 );
    }



    if( circles.size() > 4){
        for(size_t i = 0; i < circles.size(); ++i){
            if(!is_circle_near_color(imgBGR[1],circles[i])){
                cv::circle( drawing, cv::Point(circles[i][0],circles[i][1]), circles[i][2], cv::Scalar(255,255,255), 10, 8, 0 );
                circles.erase(circles.begin()+i);
                --i;
            } else {
                cv::circle( drawing, cv::Point(circles[i][0],circles[i][1]), circles[i][2], cv::Scalar(0,255,255), 10, 8, 0 );
            }
        }
        midpoint = cv::Point(0,0);
        for(auto i : circles){
            midpoint.x += i[0];
            midpoint.y += i[1];
        }
        midpoint.x = midpoint.x / circles.size();
        midpoint.y = midpoint.y / circles.size();
        cv::circle( drawing, midpoint, blue_circles[0][2], cv::Scalar(255,0,255), 10, 8, 0 );
//        cv::imshow("image", drawing);
//        cv::waitKey();
    }

    cv::imshow("image", drawing);
    if(circles.size() != 4) {
        std::cout << "not found: " << image_in_set << " size: " << circles.size() << std::endl;
        points.push_back(midpoint);
        return false;
    } else {
        if(locate_one_point){
            points.push_back(midpoint);
        } else {
            for(auto i : circles){
                points.push_back(cv::Point( cvRound(i[0]), cvRound(i[1]) ));
            }
        }
        return true;
    }

}

void image_trackbar(int, void*){
    org = original_images.at(image_in_set).clone();
    cv::imshow("original", org);

    std::vector<cv::Point> points;
    bool found = findMarker01(org, points);
//    std::cout << "midpoint = " << points[0] << std::endl;
}

int main(int argc, char* argv[]){
    cv::namedWindow("original", cv::WINDOW_NORMAL);
    cv::namedWindow("image", cv::WINDOW_NORMAL);
    if(argc == 2){
        original_images.push_back( cv::imread(argv[1]) );
    } else {
        std::string filename;
        char numbera, numberb;

        original_images.push_back( cv::imread("../SamplePluginPA10/markers/Marker1.ppm"));
        original_images.push_back( cv::imread("../color_bg.png"));

        for(int i = 1; i <= 30; ++i){
            numbera = i/10 %10 + '0';
            numberb = i%10 + '0';
            filename = "../marker_color/marker_color_"; //30
            filename += numbera;
            filename += numberb;
            filename += ".png";
            original_images.push_back( cv::imread(filename) );
        }
        for(int i = 1; i <= 52; ++i){
            numbera = i/10 %10 + '0';
            numberb = i%10 + '0';
            filename = "../marker_color_hard/marker_color_hard_"; //52
            filename += numbera;
            filename += numberb;
            filename += ".png";
            original_images.push_back( cv::imread(filename) );
        }
    }

    cv::createTrackbar("Selected image", "original",&image_in_set,original_images.size()-1,image_trackbar);
    //* <---------remove one slash to envoke trackbar instead of autoplay
    for(int i = 0; i < original_images.size()-1; ++i){
        image_in_set = i;
        image_trackbar(0,nullptr);
        if( cv::waitKey(300) == 'q'){
            break;
        }
    }
    /*/
    image_in_set = 80;
    image_trackbar(0,nullptr);
    cv::waitKey();
    //*/
    return 0;
}