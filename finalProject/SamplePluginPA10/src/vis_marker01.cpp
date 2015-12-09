#include "vis_marker01.hpp"

std::vector<cv::Point> featureextraction::find_circles(cv::Mat &image, int min_area, int max_area){
    int threshold = 127;

    //Apply threshold to binarize the image
    cv::Mat threshold_output;
    std::vector<cv::Vec4i> hierarchy;
    cv::threshold(image, threshold_output, threshold, 255,  cv::THRESH_BINARY);
    cv::Mat tmp = threshold_output.clone();
    cv::blur(tmp,threshold_output,cv::Size(5,5));
//                    cv::imshow( "Threshold", threshold_output );
    //Find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours( threshold_output, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE);
    cv::Mat drawing = cv::Mat::zeros( image.size(), CV_8UC3 );
//    std::cout << "contour size: " << contours.size() << '\n';

    //select circles
    std::vector<cv::Point> center;
    cv::Moments mu; //use moments to get center of mass
    cv::Scalar color = cv::Scalar(255, 255, 255);
    for( size_t i = 0; i< contours.size(); i++ ){
        double area = cv::contourArea(contours[i]);
        double circumference = cv::arcLength(contours[i],false);
        double circle_likeness = 4 * M_PI * area / (circumference * circumference);
        if (circle_likeness > 0.81 && area > min_area && area < max_area) {
            mu = cv::moments(contours[i],false);
            cv::Point com = cv::Point( mu.m10/mu.m00 , mu.m01/mu.m00 );
            center.push_back(com);
            drawing.at<cv::Vec3b>(com) = cv::Vec3b(255,255,255);
            cv::drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, cv::Point() );
//       } else {
//            if(area > min_area && circumference > 0)
//            std::cout << circle_likeness << " " << area << " " << circumference << "\n";
        }
    }
//    imshow( "Contours.png", drawing );
//    cv::waitKey(0);
    return center;
}

void featureextraction::separateChannels (cv::Mat img) {
    //initialize a matrix for each channel in img
    cv::Mat ch1(img.rows, img.cols, CV_8UC1);
    cv::Mat ch2(img.rows, img.cols, CV_8UC1);
    cv::Mat ch3(img.rows, img.cols, CV_8UC1);

    //initialize an int array of size 2
    int from_to[2];

    //copy channel 2 of img to channel 0 of ch1
    from_to[0] = 2;
    from_to[1] = 0;
    cv::mixChannels( &img, 1, &ch1, 1, from_to, 1);
//    cv::imshow("Value", ch1);

    //copy channel 1 of img to channel 0 of ch2
    from_to[0] = 1;
    from_to[1] = 0;
    cv::mixChannels( &img, 1, &ch2, 1, from_to, 1);
//    cv::imshow("Saturation", ch2);

    //copy channel 0 of img to channel 0 of ch3
    from_to[0] = 0;
    from_to[1] = 0;
    cv::mixChannels( &img, 1, &ch3, 1, from_to, 1);
//    cv::imshow("Hue", ch3);

    cv::waitKey(0);
}

bool featureextraction::findMarker01(cv::Mat img, std::vector<cv::Point> &points){
    bool ret = false;
    cv::Mat org = img;
//    if(argc == 2){
//        org = cv::imread(argv[1]);
//    } else {
////        org = cv::imread("../SamplePluginPA10/markers/Marker1_modified.ppm");
//        org = cv::imread("../SamplePluginPA10/markers/Marker1.ppm");
////        org = cv::imread("../marker_color_hard/marker_color_hard_18.png");
//    }
    cv::Mat imgBGR[3];
    cv::split(org,imgBGR);

    cv::imshow("marker 1", org);
    cv::imshow("blue orig" , imgBGR[0]);
//    cv::imshow("green", imgBGR[1]);
//    cv::imshow("red"  , imgBGR[2]);
//    cv::waitKey(0);

    cv::Mat imghsv;
    cv::cvtColor(org, imghsv, CV_BGR2HSV);
//    separateChannels(imghsv);

    cv::Mat imgHSV[3];
    cv::split(imghsv, imgHSV);
//    cv::imshow("hue", imgHSV[0]);
//    cv::imshow("sat", imgHSV[1]);
//    cv::imshow("val", imgHSV[2]);
//    cv::waitKey(0);

    int hue_min, hue_max; //range 0 - 180 degrees
    int sat_min = 0.3 * 255, sat_max = 1.0 * 255; // range 0 - 255   radius
    int val_min = 0.10 * 255, val_max = 1 * 255; // range 0 - 255 intensity

//    hue_min = 220 / 2 ; hue_max = 260 /2;
    hue_min = 210 / 2 ; hue_max = 270 /2;
    cv::inRange(imghsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max,sat_max,val_max), imgBGR[0]);
    cv::imshow("blue", imgBGR[0]);
    cv::waitKey(1);

    hue_min = 70 / 2; hue_max = 145 / 2;
    cv::inRange(imghsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max,sat_max,val_max), imgBGR[1]);
//    cv::imshow("green", imgBGR[1]);

    hue_min = 0 / 2; hue_max = 30/ 2;
    cv::inRange(imghsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max,sat_max,val_max), imgBGR[2]);
//    cv::imshow("red", imgBGR[2]);

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
//        double rot = atan( (double)(red[0].y-midpoint.y)/(red[0].x-midpoint.x) ) - RAD_45;

//        std::cout << "center : "<< midpoint << '/' << midpnt_uv << '\t'
//                  << "rotation in degrees: " << rot * 180/M_PI<< '\n';

        points = std::vector< cv::Point >{midpnt_uv};
        ret = true;

    } else if(blue.size() < 3) {
//        rw::common::Log::log().info() << "Too few markers found. B#: " << blue.size() << ", R#: " << red.size() << "\n";
    } else if(blue.size() > 3) {
//        rw::common::Log::log().info() << "Too many markers found. B# : " << blue.size() << ", R#: " << red.size() << "\n";
    } else {
//        rw::common::Log::log().info() << "no red marker found\n";
    }

//    for(int i = 0; i < blue.size();i++){
//        rw::common::Log::log().info() << blue[i] << "\n";
//    }

    return ret;
}
