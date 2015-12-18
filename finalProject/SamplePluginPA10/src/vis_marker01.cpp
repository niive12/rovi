#include "vis_marker01.hpp"

//check if at least 5 coordinates alone the bounding box of a circle is the right color
bool is_circle_near_color(cv::Mat &color, cv::Vec3f &circle){
    int sum = 0;
    int err = 12;

    sum += color.at<uchar>(cv::Point(circle[0]+circle[2]+err, circle[1]+circle[2]+err) );
    sum += color.at<uchar>(cv::Point(circle[0]+circle[2]+err, circle[1]-circle[2]-err) );
    sum += color.at<uchar>(cv::Point(circle[0]-circle[2]-err, circle[1]-circle[2]-err) );
    sum += color.at<uchar>(cv::Point(circle[0]-circle[2]-err, circle[1]+circle[2]+err) );

    sum += color.at<uchar>(cv::Point(circle[0], circle[1]+circle[2]+err) );
    sum += color.at<uchar>(cv::Point(circle[0], circle[1]-circle[2]-err) );
    sum += color.at<uchar>(cv::Point(circle[0]+circle[2]+err, circle[1]) );
    sum += color.at<uchar>(cv::Point(circle[0]-circle[2]-err, circle[1]) );

    return sum > 5*255; //at least 5 points
}

bool featureextraction::findMarker01(const cv::Mat &img, std::vector<cv::Point> &points, bool locate_one_point, bool constraintTime, double maxprocessingtime){
    std::chrono::high_resolution_clock::time_point start;
    start = std::chrono::high_resolution_clock::now();

    points.clear();
    cv::Mat src_gray;
    cv::Mat imghsv;
    cv::cvtColor(img, imghsv, CV_BGR2HSV);
    cv::Mat imgBGR[3];

    int hue_min, hue_max; //range 0 - 180 degrees
    int sat_min = 0.4 * 255, sat_max = 1.0 * 255; // range 0 - 255   radius
    int val_min = 0.10 * 255, val_max = 1 * 255; // range 0 - 255 intensity

    hue_min = 210 / 2 ; hue_max = 270 /2;
    cv::inRange(imghsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max,sat_max,val_max), imgBGR[0]);

    hue_min = 0 / 2; hue_max = 20/ 2;
    cv::inRange(imghsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max,sat_max,val_max), imgBGR[2]);

    if(constraintTime){
        std::chrono::high_resolution_clock::time_point now;
        now = std::chrono::high_resolution_clock::now();
        double runTime = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
        if(runTime > maxprocessingtime){
            return false;
        }
    }

    hue_min = 70 / 2; hue_max = 175 / 2;
    sat_min = 0.1 * 255;
    sat_max = 1.0 * 255;
    cv::inRange(imghsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max,sat_max,val_max), imgBGR[1]);
    cv::cvtColor( img, src_gray, CV_BGR2GRAY );

    if(constraintTime){
        std::chrono::high_resolution_clock::time_point now;
        now = std::chrono::high_resolution_clock::now();
        double runTime = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
        if(runTime > maxprocessingtime){
            return false;
        }
    }

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

    if(constraintTime){
        std::chrono::high_resolution_clock::time_point now;
        now = std::chrono::high_resolution_clock::now();
        double runTime = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
        if(runTime > maxprocessingtime){
            return false;
        }
    }

    for( auto i : green_circles ){
        cv::Point center(cvRound(i[0]), cvRound(i[1]));
        if(is_circle_near_color(imgBGR[1],i)){
            if( imgBGR[2].at<uchar>(center) > 250 ){
                red_circles.push_back(i);
            } else if( imgBGR[0].at<uchar>(center) > 250 ){
                blue_circles.push_back(i);
            }
        }
    }

    if(constraintTime){
        std::chrono::high_resolution_clock::time_point now;
        now = std::chrono::high_resolution_clock::now();
        double runTime = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
        if(runTime > maxprocessingtime){
            return false;
        }
    }

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

    if(constraintTime){
        std::chrono::high_resolution_clock::time_point now;
        now = std::chrono::high_resolution_clock::now();
        double runTime = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
        if(runTime > maxprocessingtime){
            return false;
        }
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

        if(constraintTime){
            std::chrono::high_resolution_clock::time_point now;
            now = std::chrono::high_resolution_clock::now();
            double runTime = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
            if(runTime > maxprocessingtime){
                return false;
            }
        }

        for(unsigned int i = 0; i < circles.size(); ++i){
            cv::Point a(circles[i][0],circles[i][1]);
            for(unsigned int j = i+1; j < circles.size(); ++j){
                cv::Point b(circles[j][0],circles[j][1]);
                cv::Point d = a - b;
                if(abs(d.x) < 20 && abs(d.y) < 20){
                    circles.erase(circles.begin() + i);
                    --i;
                }
            }
        }
        blue_circles = circles;
    }

    cv::Point midpoint(0,0);
    circles = blue_circles;
    for(auto r : red_circles){
        if(is_circle_near_color(imgBGR[1],r)){
            circles.push_back(r);
        }
    }

    if(circles.size() > 0){
        for(auto i : circles){
            midpoint.x += i[0];
            midpoint.y += i[1];
        }
        midpoint.x = midpoint.x / circles.size();
        midpoint.y = midpoint.y / circles.size();
    }

    if(constraintTime){
        std::chrono::high_resolution_clock::time_point now;
        now = std::chrono::high_resolution_clock::now();
        double runTime = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
        if(runTime > maxprocessingtime){
            return false;
        }
    }

    if( circles.size() > 4){
        for(unsigned int i = 0; i < circles.size(); ++i){
            if(!is_circle_near_color(imgBGR[1],circles[i])){
                circles.erase(circles.begin()+i);
                --i;
            }
        }
        midpoint = cv::Point(0,0);
        for(auto i : circles){
            midpoint.x += i[0];
            midpoint.y += i[1];
        }
        if(circles.size() > 0){
            midpoint.x = midpoint.x / circles.size();
            midpoint.y = midpoint.y / circles.size();
        }
    }

    if(circles.size() != 4) {
        return false;
    } else {
        if(locate_one_point){
            midpoint.x -= img.cols / 2;
            midpoint.y = img.rows / 2 - midpoint.y;
            points.push_back(midpoint);
        } else {
            for(auto i : circles){
                points.push_back(cv::Point( (i[0] - img.cols / 2), (img.rows / 2 - i[1]) ));
            }
        }
        return true;
    }
}
