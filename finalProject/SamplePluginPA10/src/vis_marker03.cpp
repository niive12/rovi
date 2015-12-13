#include "vis_marker03.hpp"

std::vector<cv::KeyPoint> keypoints_object;
cv::Mat descriptors_object;
std::vector<cv::Point2f> obj_corners(4);


void featureextraction::init_marker03(cv::Mat &img_object){
    // get descriptors of orig marker
    obj_corners[0] = cv::Point2f(0,0);
    obj_corners[1] = cv::Point2f( img_object.cols, 0 );
    obj_corners[2] = cv::Point2f( img_object.cols, img_object.rows );
    obj_corners[3] = cv::Point2f( 0, img_object.rows );

    get_marker_descriptors(img_object, keypoints_object, descriptors_object);
}

void featureextraction::get_marker_descriptors(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors){
//    std::chrono::high_resolution_clock::time_point t1;
//    std::chrono::high_resolution_clock::time_point t2;

    cv::SiftFeatureDetector detector;
//    t1 = std::chrono::high_resolution_clock::now();
    detector.detect(img, keypoints);
//    t2 = std::chrono::high_resolution_clock::now();
//    std::cout << "detector: " << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() << '\t';

    cv::SiftDescriptorExtractor extractor;
//    t1 = std::chrono::high_resolution_clock::now();
    extractor.compute( img, keypoints, descriptors);
//    t2 = std::chrono::high_resolution_clock::now();
//    std::cout << "extractor: " << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() << '\t';

}


void featureextraction::get_homography_flann(cv::Mat &H, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors, std::vector<cv::DMatch> &good_matches){
    cv::FlannBasedMatcher flann;
    std::vector< cv::DMatch > matches;
    flann.match( descriptors_object, descriptors, matches );

    double min_dist = 1e6;

    for( int i = 0; i < descriptors_object.rows; ++i ){
        double dist = matches[i].distance;
        if( dist < min_dist ){
            min_dist = dist;
        }
    }

    for( int i = 0; i < descriptors_object.rows; ++i ){
        if( matches[i].distance < 3*min_dist ){
            good_matches.push_back( matches[i]);
        }
    }

    std::vector<cv::Point2f> object;
    std::vector<cv::Point2f> scene;

    for(auto i : good_matches){
        object.push_back(keypoints_object[i.queryIdx].pt);
        scene.push_back( keypoints[i.trainIdx].pt );
    }
    H = cv::findHomography( object, scene, CV_RANSAC );
}

bool featureextraction::findMarker03(const cv::Mat &img_scene, std::vector<cv::Point> &points, bool locate_one_point){
//    std::chrono::high_resolution_clock::time_point t1;
//    std::chrono::high_resolution_clock::time_point t2;
    bool debug_images = true;
    points.clear();


    std::vector<cv::KeyPoint> keypoints_scene;
    cv::Mat descriptors_scene;


//    t1 = std::chrono::high_resolution_clock::now();
    get_marker_descriptors(img_scene, keypoints_scene, descriptors_scene);
//    t2 = std::chrono::high_resolution_clock::now();
//    std::cout << "get_marker_descriptors : " << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() << '\t';

    if(debug_images && false){
        cv::Mat output;
        cv::drawKeypoints(img_scene, keypoints_scene, output);
        cv::imshow("image", output);
    }

    cv::Mat H;
    std::vector<cv::DMatch> good_matches;

    get_homography_flann(H,keypoints_scene,descriptors_scene, good_matches);
    std::vector<cv::Point2f> scene_corners(4);
    cv::perspectiveTransform( obj_corners, scene_corners, H);

    cv::Mat drawing = img_scene.clone();
    if(debug_images && false){
        cv::line( drawing, scene_corners[0],
                           scene_corners[1], cv::Scalar( 0, 0, 255), 4 );
        cv::line( drawing, scene_corners[1],
                           scene_corners[2], cv::Scalar( 0, 0, 255), 4 );
        cv::line( drawing, scene_corners[2],
                           scene_corners[3], cv::Scalar( 0, 0, 255), 4 );
        cv::line( drawing, scene_corners[3],
                           scene_corners[0], cv::Scalar( 0, 0, 255), 4 );
        cv::imshow("image", drawing);
        cv::waitKey(100);
    }
    cv::Point midpoint(0,0);
//    rw::common::Log::log().info() << "matches: " << good_matches.size() << "\n";
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

