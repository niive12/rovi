#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "histogram.h"

int main(int argc, char** argv ){
    std::cout << "Hello World!" << "\n";
    if ( argc != 2 ) {
        //No single image passed as argument
        return -1;
    }

    cv::Mat image = cv::imread( "../images/Image1.png", CV_LOAD_IMAGE_GRAYSCALE );
    cv::Mat histogram;
    cv::Mat histImage( 512, 1024, CV_8UC3 );
    make_histogram(image,histImage,histogram,1024);
//    cv::imshow( "Original Image", image);
    cv::imshow( "Original Histogram", histImage);

    cv::waitKey(0);


    return 0;
}

