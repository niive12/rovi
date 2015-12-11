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
            cv::circle( drawing, center, 3, cv::Scalar(0,0,255), -1, 8, 0 );
            // circle outline
            cv::circle( drawing, center, radius, cv::Scalar(255,255,255), 3, 8, 0 );
        } else if( imgBGR[0].at<uchar>(center) > 250 ){
            blue_circles.push_back(i);
            cv::circle( drawing, center, 3, cv::Scalar(0,0,255), -1, 8, 0 );
            // circle outline
            cv::circle( drawing, center, radius, cv::Scalar(255,255,255), 3, 8, 0 );
        }
    }
    float dist, a, b;
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
    if( false && blue_circles.size() < 3){
        cv::HoughCircles( imgBGR[0],
                          blue_circles,
                          CV_HOUGH_GRADIENT,
                          1,
                          min_dist_between_circles,
                          upper_limit_for_canny,
                          threshold_for_center,
                          min_radius_circle,
                          max_radius_circle );
    }
    cv::Point midpoint(0,0);
    std::vector<cv::Vec3f> circles = blue_circles;
    for(auto r : red_circles){
        circles.push_back(r);
    }
    for(auto i : circles){
        midpoint.x += i[0];
        midpoint.y += i[1];
    }
    midpoint.x = midpoint.x / circles.size();
    midpoint.y = midpoint.y / circles.size();
    cv::circle( drawing, midpoint, blue_circles[0][2], cv::Scalar(0,0,255), 10, 8, 0 );

    if( circles.size() > 4){
        for(size_t i = 0; i < circles.size(); ++i){
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            a = circles[i][0] - midpoint.x;
            b = circles[i][1] - midpoint.y;
            dist =  a * a + b * b;
            if( dist > 30000 ){
                cv::circle( drawing, center, circles[i][2], cv::Scalar(0,0,255), 10, 8, 0 );
                circles.erase(circles.begin()+ i,circles.begin()+ i+1);
            } else {
                cv::circle( drawing, center, circles[i][2], cv::Scalar(0,255,255), 10, 8, 0 );
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
    }

    cv::imshow("image", drawing);

    if(circles.size() != 4) {
        std::cout << image_in_set << " size: " << circles.size() << std::endl;
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

    findMarker01(org, points);
    std::cout << "midpoint = " << points[0] << std::endl;
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
    image_in_set = 0;
    for(int i = 0; i < original_images.size()-1; ++i){
        ++image_in_set;
        image_trackbar(0,nullptr);
        cv::waitKey(300);
    }
    /*/
    image_trackbar(0,nullptr);
    cv::waitKey();
    //*/
    return 0;
}
