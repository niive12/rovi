// open cv libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// own functionality
#include "analyse_freq.h"
#include "histogram.h"
#include "median_filter.h"
#include "analyse_freq.h"
#include "part01.h"
#include "part04.h"
#include "areas.h"

// std
#include <iostream>
#include <vector>


int main(){
    std::vector<std::string> image_names;
    image_names.push_back("../images/Image1.png"         );
    image_names.push_back("../images/Image2.png"         );
    image_names.push_back("../images/Image3.png"         );
    image_names.push_back("../images/Image4_1.png"       );
    image_names.push_back("../images/Image4_2.png"       );
    image_names.push_back("../images/Image5_optional.png");

    cv::Mat_<float> image = cv::imread( image_names.at(0) , CV_LOAD_IMAGE_GRAYSCALE );
    cv::Mat_<float> modified = image.clone();
    cv::Mat_<float> out = image.clone();
    std::cout << "org size: " << image.rows << " " << image.cols << "\n";
    part01(modified,out);
    namedWindow("Restored Image", cv::WINDOW_NORMAL);
    cv::normalize(image,image,0,1,CV_MINMAX);
    cv::imshow("Image", image);
    return 0;
    std::cout << "Image loaded\n";


    /*
//    median_filter(image,modified,7,1);
    cv::medianBlur(image,modified,7);
    cv::imshow( "Restored Image", modified);
    cv::resizeWindow("Restored Image", WIDTH, HEIGHT);
    cv::Mat histogram;
    cv::Mat histImage( 512, 1024, CV_8UC3 );
    make_histogram(image,histImage,histogram,1024);
//    cv::imshow( "Original Histogram", histImage);
*/

    // part 4
    // analysis
    cv::Mat_<float> image_freq_04;
    visualize_frequency(image,image_freq_04);
    cv::normalize(image_freq_04, image_freq_04, 0, 255, CV_MINMAX);
    cv::imwrite("../images/frequency_analysis_04.png", image_freq_04);
    // compute resulting image
    cv::Mat_<float> image_res_04 = image.clone();
    part04(image, image_res_04);
    namedWindow("Restored Image", cv::WINDOW_NORMAL);
    cv::imshow("Restored Image", image_res_04);
    cv::normalize(image_res_04, image_res_04, 0, 255, CV_MINMAX);
    cv::imwrite("../images/image_result_04.png",image_res_04);

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

