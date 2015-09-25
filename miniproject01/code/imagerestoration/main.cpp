#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "histogram.h"
#include <vector>

int main(int argc, char** argv ){
    std::cout << "Hello World!" << "\n";
    if ( argc != 2 ) {
        //No single image passed as argument
        return -1;
    }
    std::vector<std::string> image_names;
    image_names.push_back("../images/Image1.png"         );
    image_names.push_back("../images/Image2.png"         );
    image_names.push_back("../images/Image3.png"         );
    image_names.push_back("../images/Image4_1.png"       );
    image_names.push_back("../images/Image4_2.png"       );
    image_names.push_back("../images/Image5_optional.png");

    for(auto i : image_names){
        cv::Mat image = cv::imread( i , CV_LOAD_IMAGE_GRAYSCALE );
        cv::Mat histogram;
        cv::Mat histImage( 512, 1024, CV_8UC3 );
        make_histogram(image,histImage,histogram,1024);
        //    cv::imshow( "Original Image", image);
        cv::imshow( "Original Histogram", histImage);
        cv::waitKey(0);
    }



    return 0;
}

