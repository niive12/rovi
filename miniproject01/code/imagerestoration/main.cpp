#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "histogram.h"
#include "median_filter.h"
#include <vector>

#define WIDTH 1000
#define HEIGHT 500

int main(){
    std::cout << "Hello World!" << "\n";

    std::vector<std::string> image_names;
    image_names.push_back("../images/Image1.png"         );
    image_names.push_back("../images/Image2.png"         );
    image_names.push_back("../images/Image3.png"         );
    image_names.push_back("../images/Image4_1.png"       );
    image_names.push_back("../images/Image4_2.png"       );
    image_names.push_back("../images/Image5_optional.png");


    cv::Mat image = cv::imread( image_names.at(1) , CV_LOAD_IMAGE_GRAYSCALE );
    cv::Mat modified = image.clone();
    namedWindow("Restored Image", cv::WINDOW_NORMAL);
//    median_filter(image,modified,7,1);
    cv::medianBlur(image,modified,7);
    cv::imshow( "Restored Image", modified);
    cv::resizeWindow("Restored Image", WIDTH, HEIGHT);
    cv::Mat histogram;
    cv::Mat histImage( 512, 1024, CV_8UC3 );
    make_histogram(image,histImage,histogram,1024);
//    cv::imshow( "Original Histogram", histImage);
    cv::waitKey(0);
    /*
    for(auto i : image_names){
        cv::Mat image = cv::imread( i , CV_LOAD_IMAGE_GRAYSCALE );
        cv::Mat histogram;
        cv::Mat histImage( 512, 1024, CV_8UC3 );
        make_histogram(image,histImage,histogram,1024);
        cv::imshow( "Original Histogram", histImage);
        cv::waitKey(0);
    }//*/
    return 0;
}

