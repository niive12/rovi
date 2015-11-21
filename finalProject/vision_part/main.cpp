// open cv libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>

#include <vector>
#include <algorithm>

#define RAD_45 0.78539816 //45 degrees in radians

std::vector<cv::Point> find_circles(cv::Mat &image, int min_area = 500, int max_area = 60000){
    int threshold = 127;

    //Apply threshold to binarize the image
    cv::Mat threshold_output;
    std::vector<cv::Vec4i> hierarchy;
    cv::threshold(image, threshold_output, threshold, 255,  cv::THRESH_BINARY);
//    cv::imshow( "Threshold.png", threshold_output );

    //Find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours( threshold_output, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE);
//    cv::Mat drawing = cv::Mat::zeros( image.size(), CV_8UC3 );
    std::cout << "contour size: " << contours.size() << '\n';

    //select circles
    std::vector<cv::Point> center;
    cv::Moments mu; //use moments to get center of mass
//    cv::Scalar color = cv::Scalar(255, 255, 255);
    for( size_t i = 0; i< contours.size(); i++ ){
        double area = cv::contourArea(contours[i]);
        double circumference = cv::arcLength(contours[i],false);
        double circle_likeness = 4 * M_PI * area / (circumference * circumference);
        if (circle_likeness > 0.90 && area > min_area && area < max_area) {
            mu = cv::moments(contours[i],false);
            cv::Point com = cv::Point( mu.m10/mu.m00 , mu.m01/mu.m00 );
            center.push_back(com);
//            drawing.at<cv::Vec3b>(com) = cv::Vec3b(255,255,255);
//            cv::drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, cv::Point() );
       }
    }
//    imshow( "Contours.png", drawing );
//    cv::waitKey(0);
    return center;
}

int main(int argc, char* argv[]){
//    cv::Mat org = cv::imread("../SamplePluginPA10/markers/Marker1_modified.ppm");
    cv::Mat org = cv::imread("../SamplePluginPA10/markers/Marker1.ppm");

    cv::Mat imgBGR[3];
    cv::split(org,imgBGR);

//    cv::imshow("marker 1", org);
//    cv::imshow("blue" , imgBGR[0]);
//    cv::imshow("green", imgBGR[1]);
//    cv::imshow("red"  , imgBGR[2]);
//    cv::waitKey(0);

    std::vector<cv::Point> blue =  find_circles(imgBGR[0]);
    std::vector<cv::Point> red = find_circles(imgBGR[2]);

    if( blue.size() == 3 && red.size() == 1){
        blue.push_back(red[0]);

        //sort points
        int x_direction[4];
        int y_direction[4];

        for( int i = 0; i < 4; ++i){
            x_direction[i] = blue[i].x;
            y_direction[i] = blue[i].y;
        }
        std::sort(x_direction, x_direction+4);
        std::sort(y_direction, y_direction+4);

        //get midpoint
        cv::Point midpoint;

        int most_left = (x_direction[1] - x_direction[0])/2 + x_direction[0];
        int most_right = (x_direction[3] - x_direction[2])/2 + x_direction[2];
        midpoint.x = (most_right - most_left)/ 2 + most_left;

        most_left  = (y_direction[1] - y_direction[0])/2 + y_direction[0];
        most_right = (y_direction[3] - y_direction[2])/2 + y_direction[2];
        midpoint.y = (most_right - most_left)/ 2 + most_left;

        cv::Point midpnt_uv;
        midpnt_uv.x = org.cols / 2 - midpoint.x;
        midpnt_uv.y = org.rows / 2 - midpoint.y;


        //get rotation
        double rot = atan( (double)(red[0].y-midpoint.y)/(red[0].x-midpoint.x) ) - RAD_45;

        std::cout << midpoint << midpnt_uv << '\n';
        std::cout << "rotation in degrees: " << rot * 180/M_PI<< '\n';

    } else if(blue.size() < 3) {
        std::cout << "too few markers found\n";
    } else if(blue.size() > 3) {
        std::cout << "too many markers found\n";
    }
}
