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
#include "part02.h"
#include "part03.h"
#include "part04.h"
#include "areas.h"
#include "mean_filter.h"

// std
#include <iostream>
#include <vector>

#define PART(x) 1<<x

int main(){
    std::vector<std::string> image_names;
    image_names.push_back("../images/Image1.png"         );
    image_names.push_back("../images/Image2.png"         );
    image_names.push_back("../images/Image3.png"         );
    image_names.push_back("../images/Image4_1.png"       );
    image_names.push_back("../images/Image4_2.png"       );
    image_names.push_back("../images/Image5_optional.png");

    cv::Mat_<float> image;
    cv::Mat_<float> out;


//    int to_run = PART(1) | PART(4);
//    int to_run = PART(2);
    int to_run = PART(3);

    // part 1
    if(to_run & PART(1)){
        image = cv::imread( image_names.at(0) , CV_LOAD_IMAGE_GRAYSCALE );

        part01(image,out);

        cv::normalize(out,out,0,255,CV_MINMAX);
        cv::imwrite("../images/image_result_1.png",out);
    }
    // part 2
    if(to_run & PART(2)){
        image = cv::imread( image_names.at(1) , CV_LOAD_IMAGE_GRAYSCALE );
        out = image.clone();
        part02(image, out);
        cv::normalize(out,out,0,255,CV_MINMAX);
        cv::imwrite("../images/image_result_2.png",out);
      }
    // part 3
    if(to_run & PART(3)){
        image = cv::imread( image_names.at(2) , CV_LOAD_IMAGE_GRAYSCALE );

        cv::Mat_<float> i_har, i_geo, i_ari, h_har, h_geo, h_ari, h_pre; // i_ = images, h_ = histograms
        part03(image, h_pre, i_geo, i_har, i_ari, h_geo, h_har, h_ari);

        cv::imwrite("../images/histogram_uniform_harmonic_03.png", h_har);
        cv::imwrite("../images/histogram_uniform_geometrical_03.png", h_geo);
        cv::imwrite("../images/histogram_uniform_arithmetic_03.png", h_ari);
        cv::imwrite("../images/histogram_uniform_03.png", h_pre);
        cv::imwrite("../images/image_harmonic_03.png", i_har);
        cv::imwrite("../images/image_geometrical_03.png", i_geo);
        cv::imwrite("../images/image_arithmetic_03.png", i_ari);

        cv::Mat_<float> diff = (i_har - i_geo) + 127;
        cv::Mat_<float> diff_complex(diff.clone(), AREA_COMPLEX);
        cv::imwrite("../images/image_harVSgeo_complex_03.png", diff_complex);


//        namedWindow("image_harmonic", cv::WINDOW_NORMAL);
//        cv::normalize(i_har, i_har, 0, 1, CV_MINMAX);
//        cv::imshow("image_harmonic",i_har);

//        namedWindow("image_arithmetic", cv::WINDOW_NORMAL);
//        cv::normalize(i_ari, i_ari, 0, 1, CV_MINMAX);
//        cv::imshow("image_arithmetic",i_ari);

//        namedWindow("image_geometric", cv::WINDOW_NORMAL);
//        cv::normalize(i_geo, i_geo, 0, 1, CV_MINMAX);
//        cv::imshow("image_geometric",i_geo);

//        namedWindow("hist har", cv::WINDOW_NORMAL);
//        cv::imshow("hist har",h_har);

//        namedWindow("hist ari", cv::WINDOW_NORMAL);
//        cv::imshow("hist ari",h_ari);

//        namedWindow("hist geo", cv::WINDOW_NORMAL);
//        cv::imshow("hist geo",h_geo);

//        namedWindow("hist normal", cv::WINDOW_NORMAL);
//        cv::imshow("hist normal",h_pre);

        cv::waitKey(0);


    }
    // part 4
    if(to_run & PART(4)){
        image = cv::imread( image_names.at(3) , CV_LOAD_IMAGE_GRAYSCALE );

        // analysis
        // save frequency domain of full image
        cv::Mat_<float> image_freq_04;
        visualize_frequency(image,image_freq_04);
        cv::normalize(image_freq_04, image_freq_04, 0, 255, CV_MINMAX);
        cv::imwrite("../images/frequency_analysis_04.png", image_freq_04);

        // save freq iamge of uniform area
        cv::Mat_<float> image_uniform_04(image.clone(), AREA_UNIFORM);
        cv::Mat_<float> image_freq_uniform_04;
        visualize_frequency(image_uniform_04,image_freq_uniform_04);
        cv::normalize(image_freq_uniform_04, image_freq_uniform_04, 0, 255, CV_MINMAX);
        cv::imwrite("../images/frequency_analysis_uniform_04.png", image_freq_uniform_04);

        // compute resulting image
        cv::Mat_<float> image_res_04 = image.clone();
        part04(image, image_res_04);
        namedWindow("Restored Image", cv::WINDOW_NORMAL);
        cv::imshow("Restored Image", image_res_04);
        cv::normalize(image_res_04, image_res_04, 0, 255, CV_MINMAX);
        cv::imwrite("../images/image_result_04.png",image_res_04);
        cv::waitKey(0);
      }


    return 0;
}

