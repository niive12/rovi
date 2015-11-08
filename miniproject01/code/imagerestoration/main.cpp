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


    int to_run = PART(1) | PART(2) | PART(3) | PART(4);

    // part 1
    if(to_run & PART(1)){
        cv::Mat hist_uniform, hist_median, complex_median, complex_geometric_blurred, complex_harmonic_blurred, complex_difference;
        image = cv::imread( image_names.at(0) , CV_LOAD_IMAGE_GRAYSCALE );

        part01(image, out, hist_uniform, hist_median, complex_median, complex_geometric_blurred, complex_harmonic_blurred, complex_difference);
        cv::imwrite("../images/image_result_1.png",out);
        cv::imwrite("../../report/graphics/hist1_uniform.png", hist_uniform);
        cv::imwrite("../../report/graphics/hist1_uniform2.png", hist_median);
        cv::imwrite("../../report/graphics/complex1_step2.png", complex_median);
        cv::imwrite("../../report/graphics/complex1_blurred.png", complex_geometric_blurred);
        cv::imwrite("../../report/graphics/complex1_harmonic_blurred.png", complex_harmonic_blurred);
        cv::imwrite("../../report/graphics/complex1_blurr_difference_50.png",complex_difference);
    }
    // part 2
    if(to_run & PART(2)){
        cv::Mat hist_uniform, hist_median, hist_bilatteral, complex_median, complex_harmonic, complex_bilatteral, complex_histeq, complex_smoothed_histeq;
        image = cv::imread( image_names.at(1) , CV_LOAD_IMAGE_GRAYSCALE );
        out = image.clone();
        part02(image, out, hist_uniform, hist_median, hist_bilatteral, complex_median, complex_harmonic, complex_bilatteral, complex_histeq, complex_smoothed_histeq);
        cv::imwrite("../images/image_result_2.png",out);
        cv::imwrite("../../report/graphics/hist2_uniform.png",hist_uniform);
        cv::imwrite("../../report/graphics/hist2_after_median.png",hist_median);
        cv::imwrite("../../report/graphics/hist2_after_bilatteral.png",hist_bilatteral);
        cv::imwrite("../../report/graphics/complex2_median.png",complex_median);
        cv::imwrite("../../report/graphics/complex2_harmonic.png",complex_harmonic);
        cv::imwrite("../../report/graphics/complex2_bilatteral.png",complex_bilatteral);
        cv::imwrite("../../report/graphics/complex2_histeq.png",complex_histeq);
        cv::imwrite("../../report/graphics/complex2_histeq_smoothed.png",complex_smoothed_histeq);
      }
    // part 3
    if(to_run & PART(3)){
        image = cv::imread( image_names.at(2) , CV_LOAD_IMAGE_GRAYSCALE );

        cv::Mat_<float> i_har, i_geo, i_ari, h_har, h_geo, h_ari, h_uni_pre, h_post, h_pre, i_harVSgeo; // i_ = images, h_ = histograms
        part03(image, h_uni_pre, i_geo, i_har, i_ari, h_geo, h_har, h_ari, h_post, h_pre, i_harVSgeo);

        cv::imwrite("../images/histogram_uniform_harmonic_03.png", h_har);
        cv::imwrite("../images/histogram_uniform_geometrical_03.png", h_geo);
        cv::imwrite("../images/histogram_uniform_arithmetic_03.png", h_ari);
        cv::imwrite("../images/histogram_uniform_03.png", h_uni_pre);
        cv::imwrite("../images/histogram_full_post_03.png", h_post);
        cv::imwrite("../images/histogram_full_pre_03.png", h_pre);
        cv::imwrite("../images/image_harmonic_03.png", i_har);
        cv::imwrite("../images/image_geometrical_03.png", i_geo);
        cv::imwrite("../images/image_arithmetic_03.png", i_ari);
        cv::imwrite("../images/image_harVSgeo_complex_03.png", i_harVSgeo);
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
        cv::Mat_<float> image_res_04;
        part04(image, image_res_04);

        cv::Mat_<float> freq_restored;
        visualize_frequency(image_res_04,freq_restored);

        cv::normalize(freq_restored, freq_restored, 0, 255, CV_MINMAX);
        cv::imwrite("../images/image_result_freq_04.png",freq_restored);

        cv::normalize(image_res_04, image_res_04, 0, 255, CV_MINMAX);
        cv::imwrite("../images/image_result_04.png",image_res_04);
      }
    return 0;
}

