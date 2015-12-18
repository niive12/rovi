#include "vis_marker03.hpp"

std::vector<cv::KeyPoint> keypoints_object;
cv::Mat descriptors_object;
std::vector<cv::Point2f> obj_corners(4);

double featureextraction::get_area(std::vector<cv::Point2f> &corners){
    //finds the area of the triangle expanding from the first corner.
    if(corners.size() != 4){
        return 0;
    }
    double dx, dy, a, b, c, area;
    dx = fabs(corners[0].x - corners[3].x);
    dy = fabs(corners[0].y - corners[3].y);
    a = sqrt(dx*dx + dy*dy);
    dx = fabs(corners[0].x - corners[1].x);
    dy = fabs(corners[0].y - corners[1].y);
    b = sqrt(dx*dx + dy*dy);
    dx = fabs(corners[3].x - corners[1].x);
    dy = fabs(corners[3].y - corners[1].y);
    c = sqrt(dx*dx + dy*dy);

    area = 0.25 * sqrt(pow(a*a +b*b + c*c,2)-2*(pow(a,4) + pow(b,4) + pow(c,4)));
    if(a == 0 || b == 0 && c == 0){
        return 0;
    }
    if(a/b > 2 || b/a > 2){
        return 0;
    }
    return area;
}


void featureextraction::init_marker03(cv::Mat &img_object){
    // get descriptors of orig marker
    obj_corners[0] = cv::Point2f(0,0);
    obj_corners[1] = cv::Point2f( img_object.cols, 0 );
    obj_corners[2] = cv::Point2f( img_object.cols, img_object.rows );
    obj_corners[3] = cv::Point2f( 0, img_object.rows );

    get_marker_descriptors(img_object, keypoints_object, descriptors_object);
}

void featureextraction::get_marker_descriptors(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors){
    cv::SiftFeatureDetector detector;
    detector.detect(img, keypoints);

    cv::SiftDescriptorExtractor extractor;
    extractor.compute( img, keypoints, descriptors);
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

    if(min_dist < 40){
        min_dist = 40;
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
    if(object.size() >= 4){
        H = cv::findHomography( object, scene, CV_RANSAC );
    }

}

bool featureextraction::findMarker03(const cv::Mat &img_scene, std::vector<cv::Point> &points, bool locate_one_point, bool constraintTime, double maxprocessingtime){
//    std::chrono::high_resolution_clock::time_point t1;
//    std::chrono::high_resolution_clock::time_point t2;
    std::chrono::high_resolution_clock::time_point start;
    start = std::chrono::high_resolution_clock::now();

    bool debug_images = false;
    points.clear();


    std::vector<cv::KeyPoint> keypoints_scene;
    cv::Mat descriptors_scene;

    get_marker_descriptors(img_scene, keypoints_scene, descriptors_scene);
    if(constraintTime){
        std::chrono::high_resolution_clock::time_point now;
        now = std::chrono::high_resolution_clock::now();
        double runTime = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
        if(runTime > maxprocessingtime){
            return false;
        }
    }


    if(debug_images && false){
        cv::Mat output;
        cv::drawKeypoints(img_scene, keypoints_scene, output);
        cv::imshow("image", output);
    }

    cv::Mat H;
    std::vector<cv::DMatch> good_matches;

    get_homography_flann(H,keypoints_scene,descriptors_scene, good_matches);

    if(constraintTime){
        std::chrono::high_resolution_clock::time_point now;
        now = std::chrono::high_resolution_clock::now();
        double runTime = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
        if(runTime > maxprocessingtime){
            return false;
        }
    }

    std::vector<cv::Point2f> scene_corners(4);
    if(!H.empty()){
        cv::perspectiveTransform( obj_corners, scene_corners, H);
    }

    if(constraintTime){
        std::chrono::high_resolution_clock::time_point now;
        now = std::chrono::high_resolution_clock::now();
        double runTime = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
        if(runTime > maxprocessingtime){
            return false;
        }
    }

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
    double area = get_area(scene_corners);
    if( area < 25000 || area > 100000 || H.empty()){
        points.push_back(midpoint);
        return false;
    }
    if(locate_one_point){
        for(auto i : scene_corners){
            midpoint.x += i.x;
            midpoint.y += i.y;
        }
        midpoint.x = midpoint.x / 4;
        midpoint.y = midpoint.y / 4;
        points.push_back(midpoint);
    } else {
        for(auto i : scene_corners){
            points.push_back( cv::Point(cvRound(i.x),cvRound(i.y)) );
        }
    }
    return true;
}
