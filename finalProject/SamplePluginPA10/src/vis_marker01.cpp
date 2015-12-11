#include "vis_marker01.hpp"

std::vector<cv::Point> featureextraction::find_circles(const cv::Mat &image, size_t max_circles, int min_area, int max_area){
    int threshold = 127;

    //Apply threshold to binarize the image
    cv::Mat threshold_output;
    std::vector<cv::Vec4i> hierarchy;
    cv::threshold(image, threshold_output, threshold, 255,  cv::THRESH_BINARY);
    cv::Mat tmp = threshold_output.clone();
    cv::blur(tmp,threshold_output,cv::Size(5,5));

    //Find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours( threshold_output, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE);

    comparator_functor sort_by_size;
    std::sort(contours.begin(),contours.end(), sort_by_size);

    size_t n = contours.size();

    //select circles
    std::vector<cv::Point> center;
    cv::Moments mu; //use moments to get center of mass
    for( size_t i = 0; i< n; i++ ){
        double area = cv::contourArea(contours[i]);
        double circumference = cv::arcLength(contours[i],false);
        double circle_likeness = 4 * M_PI * area / (circumference * circumference);
        if (circle_likeness > 0.81 && area > min_area && area < max_area) {
            mu = cv::moments(contours[i],false);
            cv::Point com = cv::Point( mu.m10/mu.m00 , mu.m01/mu.m00 );
            center.push_back(com);
            if(center.size() == max_circles){
                break;
            }
        }
    }
    return center;
}

bool featureextraction::findMarker01(const cv::Mat &img, std::vector<cv::Point> &points){
    bool ret = false;

    cv::Mat imgBGR[3];
    cv::split(img,imgBGR);

    cv::Mat imghsv;
    cv::cvtColor(img, imghsv, CV_BGR2HSV);

    int hue_min, hue_max; //range 0 - 180 degrees
    int sat_min = 0.3 * 255, sat_max = 1.0 * 255; // range 0 - 255   radius
    int val_min = 0.10 * 255, val_max = 1 * 255; // range 0 - 255 intensity

    hue_min = 210 / 2 ; hue_max = 270 /2;
    cv::inRange(imghsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max,sat_max,val_max), imgBGR[0]);

    hue_min = 70 / 2; hue_max = 145 / 2;
    cv::inRange(imghsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max,sat_max,val_max), imgBGR[1]);

    hue_min = 0 / 2; hue_max = 30/ 2;
    cv::inRange(imghsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max,sat_max,val_max), imgBGR[2]);

    std::vector<cv::Point> blue =  find_circles(imgBGR[0], 3);
    std::vector<cv::Point> red = find_circles(imgBGR[2], 1);

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
        midpnt_uv.x = midpoint.x - img.cols / 2;
        midpnt_uv.y = img.rows / 2 - midpoint.y;


        //get rotation
//        double rot = atan( (double)(red[0].y-midpoint.y)/(red[0].x-midpoint.x) ) - RAD_45;

//        std::cout << "center : "<< midpoint << '/' << midpnt_uv << '\t'
//                  << "rotation in degrees: " << rot * 180/M_PI<< '\n';

        points = std::vector< cv::Point >{midpnt_uv};
        ret = true;

    } else if(blue.size() < 3) {
        rw::common::Log::log().info() << "Too few markers found. B#: " << blue.size() << ", R#: " << red.size() << "\n";
    } else if(blue.size() > 3) {
        rw::common::Log::log().info() << "Too many markers found. B# : " << blue.size() << ", R#: " << red.size() << "\n";
    } else {
        rw::common::Log::log().info() << "no red marker found\n";
    }

    return ret;
}
