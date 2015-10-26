// open cv libs
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// own functionality
#include "analyse_freq.h"
#include "histogram.h"
#include "median_filter.h"
#include "part04.h"
#include "analyse_freq.h"

// std
#include <iostream>
#include <vector>

#define AREA_UNIFORM cv::Rect(810,1500,300,300)
#define AREA_SIMPLE  cv::Rect(900,1225,300,300)
#define AREA_COMPLEX cv::Rect(1225,250,300,300)

#define PART(x) 1<<x

void part01(cv::Mat_<float> &original_image, cv::Mat_<float> &output_image);

int main(){
    std::vector<std::string> image_names;
    image_names.push_back("../images/Image1.png"         );
    image_names.push_back("../images/Image2.png"         );
    image_names.push_back("../images/Image3.png"         );
    image_names.push_back("../images/Image4_1.png"       );
    image_names.push_back("../images/Image4_2.png"       );
    image_names.push_back("../images/Image5_optional.png");

    cv::Mat_<float> image = cv::imread( image_names.at(3) , CV_LOAD_IMAGE_GRAYSCALE );
    std::cout << "Image loaded\n";
    cv::Mat_<float> modified = image.clone();
    cv::Mat_<float> out = image.clone();

    int to_run = PART(1) | PART(4);

    if(to_run & PART(1)){
        part01(modified,out);
        namedWindow("Restored Image", cv::WINDOW_NORMAL);
        cv::normalize(image,image,0,1,CV_MINMAX);
        cv::imshow("Image", image);
      }


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
    if(to_run & PART(4)){
        cv::normalize(image,image,0,1,CV_MINMAX);

        // analysis
        cv::Mat_<float> image_freq_04;
        visualize_frequency(image,image_freq_04);
        cv::normalize(image_freq_04, image_freq_04, 0, 255, CV_MINMAX);
        cv::imwrite("../images/frequency_analysis_04.png", image_freq_04);

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
      }
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

double what_is_the_S_P_damage(cv::Mat &img, std::string frame="none"){
    double salt, pebber;
    cv::Mat histogram;
    cv::Mat histImage( 512, 1024, CV_8UC3 );
    make_histogram(img,histImage,histogram,1024);
    if(frame != "none")
        cv::imshow(frame,histImage);
    salt   = histogram.at<float>(255) / (img.cols * img.rows);
    pebber = histogram.at<float>(0)   / (img.cols * img.rows);

    std::cout << "salt : " << salt << " & pebber: " << pebber << '\n';
    double quantile = (pebber-salt+1)/2;
    std::cout << "quantile: " << quantile << '\n';
    return quantile;
}

void part01(cv::Mat_<float> &original_image, cv::Mat_<float> &output_image){
    cv::Mat org = original_image.clone();
    cv::Mat uniform(org,AREA_UNIFORM);
//    cv::Mat out(original_image,AREA_UNIFORM);
    cv::Mat out = uniform.clone();

    double quantile = what_is_the_S_P_damage(uniform,"qwe");
    int kernel_size = 7;
    int mean_width = 3;

    median_filter(uniform, out,kernel_size,mean_width,quantile);

    cv::normalize(uniform,uniform,0,1,CV_MINMAX);
    cv::imshow("uniform", uniform);
    cv::normalize(out,out,0,1,CV_MINMAX);
    cv::imshow("output", out);
    cv::waitKey(0);

//    median_filter(original_image, output_image,kernel_size,mean_width,quantile);
//    cv::normalize(output_image,output_image,0,1,CV_MINMAX);
//    cv::imshow("Final output", output_image);
//    cv::imwrite("Final_output.png",output_image);
//    cv::waitKey(0);
}
