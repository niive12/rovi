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
        cv::Mat temp;
        cv::Mat org = image.clone();
        cv::normalize(image,image,0,255,CV_MINMAX);
        cv::Mat uniform(org,AREA_UNIFORM);
        cv::normalize(uniform,uniform,0,255,CV_MINMAX);
        uniform.convertTo(uniform,CV_8U);
        double quantile = what_is_the_S_P_damage(uniform,"histogram of uniform area");
        cv::imshow("img", uniform);
        cv::waitKey(0);
        uniform.convertTo(uniform,CV_32F);
        temp = uniform.clone();
        median_filter(uniform, temp,3,1,quantile);
        cv::normalize(temp,temp,0,255,CV_MINMAX);
        temp.convertTo(temp,CV_8U);
//I found an error: I do not detect white in histogram.
        what_is_the_S_P_damage(temp,"new hist");
        cv::imshow("new", temp);
        cv::waitKey(0);
        return 0;
        //apply to large image
        median_filter(image, out,5,1,quantile);
        cv::normalize(out,out,0,255,CV_MINMAX);
        cv::imwrite("../images/image_result_2.png",out);
      }
    // part 3
    if(to_run & PART(3)){
        image = cv::imread( image_names.at(2) , CV_LOAD_IMAGE_GRAYSCALE );

        cv::Mat_<float> i_har, i_geo, i_ari, h_har, h_geo, h_ari; // i_ = images, h_ = histograms
        part03(image,i_geo, i_har, i_ari, h_geo, h_har, h_ari);


        //        cv::normalize(image_harmonic, image_harmonic, 0, 1, CV_MINMAX);
        //        namedWindow("image_harmonic", cv::WINDOW_NORMAL);
        //        cv::imshow("image_harmonic",image_harmonic);

        //        cv::normalize(image_geometric, image_geometric, 0, 1, CV_MINMAX);
        //        namedWindow("image_geometric", cv::WINDOW_NORMAL);
        //        cv::imshow("image_geometric",image_geometric);

        //        cv::normalize(image_arithmetic, image_arithmetic, 0, 1, CV_MINMAX);
        //        namedWindow("image_arithmetic", cv::WINDOW_NORMAL);
        //        cv::imshow("image_arithmetic",image_arithmetic);

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

