#include "vis_marker02.hpp"

struct comparator_functor {
  bool operator() (const std::vector<cv::Point> a, const std::vector<cv::Point> b) const {
    return a.size() > b.size();
  }
} sorting_func;


std::vector<cv::Point> featureextraction::find_blobs(cv::Mat img, std::vector<std::vector<cv::Point> > &contours, std::vector<std::vector<cv::Point> > &good_contours){
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( img, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE);

    std::sort(contours.begin(),contours.end(), sorting_func);

    double min_area = 33000, max_area = 100000;
    std::vector<cv::Point> center;
    cv::Moments mu; //use moments to get center of mass
    for( size_t i = 0; i< contours.size(); ++i ){
        double area = cv::contourArea(contours[i]);
        if (area > min_area && area < max_area) {
            mu = cv::moments(contours[i],false);
            cv::Point com = cv::Point( mu.m10/mu.m00 , mu.m01/mu.m00 );
            center.push_back(com);
            good_contours.push_back(contours[i]);
//            cv::drawContours( drawing, contours, (int)i, cv::Scalar(0,0,255), 2, 8, hierarchy, 0, cv::Point() );
        }
        if(center.size() > 2 || area < 500){
            break;
        }
    }
    return center;
}

cv::Point featureextraction::find_center(cv::Mat &org, std::vector<cv::Point> &center, std::vector<std::vector<cv::Point> > &contours){
    cv::Point midpoint(0,0);
    if(center.size() > 0){
        std::vector<cv::Point> good_centers;
        float dist, a, b;
        if(center.size() > 1){
            for(size_t i = 0; i < center.size(); ++i){
                for(size_t j = i+1; j < center.size(); ++j){
                    a = center[i].x - center[j].x;
                    b = center[i].y - center[j].y;
                    dist =  a * a + b * b;
                    if(dist < 1000 ){
                        good_centers.push_back(center[i]);
                        good_centers.push_back(center[j]);
                    }
                }
            }
            if(good_centers.size() == 0){
                float compactness, area, circumference;
                a = org.cols/2;
                b = org.rows/2;
                float max_dist =  a * a + b * b;
                float feature_match, feature_match2;

                float p1 = 0.8, p2=0.2;
                for(size_t i = 0; i < center.size(); ++i){
                    a = org.cols/2 - center[i].x;
                    b = org.cols/2 - center[i].y;
                    dist =  1 - (a * a + b * b) / max_dist;

                    area = cv::contourArea(contours[i]);
                    circumference = cv::arcLength(contours[i],false);
                    compactness = 4 * CV_PI * area / (circumference * circumference);

                    feature_match = p1 * compactness + p2 * dist;
                    for(size_t j = i+1; j < center.size(); ++j){
                        area = cv::contourArea(contours[j]);
                        circumference = cv::arcLength(contours[j],false);
                        compactness = 4 * CV_PI * area / (circumference * circumference);

                        a = org.cols/2 - center[j].x;
                        b = org.rows/2 - center[j].y;
                        dist =  1 - (a * a + b * b) / max_dist;

                        feature_match2 = p1 * compactness + p2 * dist;

                        if(feature_match > feature_match2){
                            good_centers.push_back(center[i]);
                        } else {
                            good_centers.push_back(center[j]);
                        }
                    }
                }
            }
        } else {
            good_centers.push_back(center[0]);
        }

        for(size_t i = 0; i < good_centers.size(); ++i){
            midpoint.x += good_centers[i].x;
            midpoint.y += good_centers[i].y;
//            drawing.at<cv::Vec3b>(good_centers[i]) = cv::Vec3b(0,0,255);
        }
        midpoint.x = midpoint.x / good_centers.size();
        midpoint.y = midpoint.y / good_centers.size();

//        cv::circle( drawing, midpoint, 200,  cv::Scalar(0,0,255), 2, 8, 0 );
    }
    return midpoint;
}

bool featureextraction::findMarker02(cv::Mat &img, std::vector<cv::Point> &points){
    cv::Mat imghsv = img.clone();
    cv::Mat white;
    cv::cvtColor(img, imghsv, CV_BGR2HSV);
    cv::inRange(imghsv, cv::Scalar(0, 0, 127), cv::Scalar(180,50,255), white);

    //    cv::imshow("white", org);
    //    cv::waitKey(0);
    std::vector<std::vector<cv::Point> > contours;
//    cv::Mat drawing = original_images.at(image_in_set).clone();
    std::vector<std::vector<cv::Point> > good_contours;

    points.clear();

    std::vector<cv::Point> center = find_blobs(white, contours, good_contours);
    cv::Point midpoint = find_center(img, center, good_contours);
    midpoint.x -= img.cols / 2;
    midpoint.y = img.rows / 2 - midpoint.y;
    points.push_back(midpoint);
//    cv::imshow("image", drawing);

    if(midpoint.x == 0 && midpoint.y == 0){
        return false;
    } else {
        return true;
    }
}

