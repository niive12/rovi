// open cv libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp> //SIFT
#include "opencv2/calib3d/calib3d.hpp" //findHomography

#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>
//to meassure timing
#include <ratio>
#include <chrono>


int image_in_set = 26; //26-27 giver problemer
std::vector<cv::Mat> original_images;
cv::Mat org;

void get_marker_descriptors(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors){
    std::chrono::high_resolution_clock::time_point t1;
    std::chrono::high_resolution_clock::time_point t2;

    cv::SiftFeatureDetector detector;
    t1 = std::chrono::high_resolution_clock::now();
    detector.detect(img, keypoints);
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "detector: " << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() << '\t';

    cv::SiftDescriptorExtractor extractor;
    t1 = std::chrono::high_resolution_clock::now();
    extractor.compute( img, keypoints, descriptors);
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "extractor: " << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() << '\t';

}

cv::Mat img_object = cv::imread("../SamplePluginPA10/markers/Marker3.ppm");
std::vector<cv::KeyPoint> keypoints_object;
cv::Mat descriptors_object;
std::vector<cv::Point2f> obj_corners(4);

bool findMarker03(const cv::Mat &img_scene, std::vector<cv::Point> &points, bool locate_one_point = true){
    std::chrono::high_resolution_clock::time_point t1;
    std::chrono::high_resolution_clock::time_point t2;
    bool debug_images = true;
    points.clear();


    std::vector<cv::KeyPoint> keypoints_scene;
    cv::Mat descriptors_scene;


    t1 = std::chrono::high_resolution_clock::now();
    get_marker_descriptors(img_scene, keypoints_scene, descriptors_scene);
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "get_marker_descriptors : " << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() << '\t';

    if(debug_images && false){
        cv::Mat output;
        cv::drawKeypoints(img_scene, keypoints_scene, output);
        cv::imshow("image", output);
    }

    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_object, descriptors_scene, matches );

    double max_dist = 0; double min_dist = 1e6;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_object.rows; i++ )
    { double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector<cv::DMatch> good_matches;

    for( int i = 0; i < descriptors_object.rows; ++i ){
        if( matches[i].distance < 3*min_dist ){
            good_matches.push_back( matches[i]);
        }
    }

    std::vector<cv::Point2f> object;
    std::vector<cv::Point2f> scene;

    for(auto i : good_matches){
        object.push_back(keypoints_object[i.queryIdx].pt);
        scene.push_back( keypoints_scene[i.trainIdx].pt );
    }
    cv::Mat H = cv::findHomography( object, scene, CV_RANSAC );

    std::vector<cv::Point2f> scene_corners(4);
    cv::perspectiveTransform( obj_corners, scene_corners, H);

    if(debug_images){
        cv::Mat drawing = img_scene.clone();
        cv::line( drawing, scene_corners[0],
                           scene_corners[1], cv::Scalar( 0, 255, 0), 4 );
        cv::line( drawing, scene_corners[1],
                           scene_corners[2], cv::Scalar( 0, 255, 0), 4 );
        cv::line( drawing, scene_corners[2],
                           scene_corners[3], cv::Scalar( 0, 255, 0), 4 );
        cv::line( drawing, scene_corners[3],
                           scene_corners[0], cv::Scalar( 0, 255, 0), 4 );
        cv::imshow("image", drawing);
    }
    cv::Point midpoint(0,0);
    std::cout << "matches: " << good_matches.size() << "\t";
    if(good_matches.size() < 50 ){
        points.push_back(midpoint);
        return false;
    }
    if(locate_one_point){
        for(auto i : scene_corners){
            midpoint.x += i.x;
            midpoint.y += i.y;
        }
        midpoint.x = midpoint.x / scene_corners.size();
        midpoint.y = midpoint.y / scene_corners.size();
        points.push_back(midpoint);
    } else {
        for(auto i : scene_corners){
            points.push_back( cv::Point(cvRound(i.x),cvRound(i.y)) );
        }
    }
    return true;
}

cv::Point old_position(0,0);
int accepted_width = img_object.cols * 2;
int accepted_height = img_object.rows * 2;

void image_trackbar(int, void*){
    org = original_images.at(image_in_set).clone();
    cv::imshow("original", org);

    std::vector<cv::Point> points;

    std::chrono::high_resolution_clock::time_point t1;
    std::chrono::high_resolution_clock::time_point t2;

    int x = 0,y = 0;
    accepted_width = org.cols;
    accepted_height = org.rows;
    if( old_position != cv::Point(0,0) ){
        accepted_width = img_object.cols * 2;
        accepted_height = img_object.rows * 2;
        x = old_position.x - accepted_width/2;
        y = old_position.y - accepted_height/2;
        if(x > (org.cols-accepted_width) ){
            x = org.cols-accepted_width;
        } else if(x < 0){
            x = 0;
        }
        if(y > (org.rows-accepted_height) ){
            y = org.rows-accepted_height;
        } else if(y < 0){
            y = 0;
        }
    }
    cv::Mat cropped(org, cv::Rect(x,y,accepted_width,accepted_height));

    bool ans;
    t1 = std::chrono::high_resolution_clock::now();
    ans = findMarker03(cropped, points);
    t2 = std::chrono::high_resolution_clock::now();
    if( !ans ){
        old_position = cv::Point(0,0);
    } else if(points.size() > 0 ){
        old_position.x = points[0].x + x;
        old_position.y = points[0].y + y;
    }
    std::cout << " total : " <<std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() << " img " << image_in_set<< std::endl;

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

        original_images.push_back( cv::imread("../color_bg3.png"));

        for(int i = 1; i <= 30; ++i){
            numbera = i/10 %10 + '0';
            numberb = i%10 + '0';
            filename = "../markers/marker_corny/marker_corny_"; //30
            filename += numbera;
            filename += numberb;
            filename += ".png";
            original_images.push_back( cv::imread(filename) );
        }
        for(int i = 1; i <= 52; ++i){
            numbera = i/10 %10 + '0';
            numberb = i%10 + '0';
            filename = "../markers/marker_corny_hard/marker_corny_hard_"; //52
            filename += numbera;
            filename += numberb;
            filename += ".png";
            original_images.push_back( cv::imread(filename) );
        }
    }


    obj_corners[0] = cv::Point2f(0,0);
    obj_corners[1] = cv::Point2f( img_object.cols, 0 );
    obj_corners[2] = cv::Point2f( img_object.cols, img_object.rows );
    obj_corners[3] = cv::Point2f( 0, img_object.rows );

    get_marker_descriptors(img_object, keypoints_object, descriptors_object);
    std::cout << "\r";

    cv::createTrackbar("Selected image", "original",&image_in_set,original_images.size()-1,image_trackbar);
    //* <---------remove one slash to envoke trackbar instead of autoplay
    image_in_set = 0;
    int key;
    for(int i = image_in_set; i < original_images.size()-1; ++i){
        ++image_in_set;
        image_trackbar(0,nullptr);
        if(cv::waitKey(200) == 'q'){ break;}
    }
    /*/
    image_trackbar(0,nullptr);
    cv::waitKey();
    //*/
    return 0;
}
