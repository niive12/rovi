#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "histogram.h"

int main(){
    std::cout << "Hello World!" << "\n";

    cv::Mat image = cv::imread( "../images/Image2.png", CV_LOAD_IMAGE_GRAYSCALE );

    if(!image.data){
        std::cout << "img read\n";

    }

    std::cout << "img read\n";
    cv::Mat histogram;
    cv::Mat histImage( 512, 1024, CV_8UC3 );
    make_histogram(image,histImage,histogram,1024);
//    cv::imshow( "Original Image", image);
    cv::imshow( "Original Histogram", histImage);



    cv::waitKey(0);


    return 0;
}

