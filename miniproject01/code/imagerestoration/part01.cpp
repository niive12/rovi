#include "part01.h"
#include "analyse_freq.h"

void part01(cv::Mat_<float> &original_image, cv::Mat_<float> &output_image){
    cv::Mat org = original_image.clone();
    cv::Mat temp_image;
    cv::Mat uniform(org,AREA_UNIFORM);
    cv::Mat out = uniform.clone();

    double quantile = what_is_the_S_P_damage(uniform,"histogram of uniform area");

    median_filter(uniform, out,3,1,quantile);
    median_filter(uniform, out,7,3,quantile);

    cv::normalize(uniform,uniform,0,1,CV_MINMAX);
    cv::imshow("uniform area", uniform);
    cv::normalize(out,out,0,1,CV_MINMAX);
    cv::imshow("filtered uniform area", out);
    cv::waitKey(0);
    cv::destroyAllWindows();
    cv::Mat complex(org,AREA_COMPLEX);

    out = complex.clone();
    cv::normalize(out,out,0,1,CV_MINMAX);
    cv::imshow("Original image", out);
    temp_image = complex.clone();
    median_filter(complex, temp_image, 3, 1, quantile);
    complex = temp_image.clone();

    out = complex.clone();
    cv::normalize(out,out,0,1,CV_MINMAX);
    cv::imshow("3x3 median filter", out);

    median_filter(complex, temp_image, 7, 3, quantile);
    complex = temp_image.clone();

    cv::normalize(temp_image,temp_image,0,255,CV_MINMAX);
    temp_image.convertTo(temp_image,CV_8U);
    cv::imshow("7x7 median filter, mean of 3", temp_image);

    cv::waitKey(0);
    //Apply all this to the large image:
    output_image = original_image.clone();
    median_filter(original_image, output_image,3,1,quantile);
    temp_image = output_image.clone();
    median_filter(temp_image, output_image,7,3,quantile);
}

double what_is_the_S_P_damage(cv::Mat &img, std::string frame){
    double salt, pebber;
    cv::Mat histogram;
    cv::Mat histImage( 256, 512, CV_8UC3 );
    std::cout << '\n';
    make_histogram(img,histImage,histogram,512,256);
    if(frame != "none")
        cv::imshow(frame,histImage);
    salt   = histogram.at<float>(255) / (img.cols * img.rows);
    pebber = histogram.at<float>(0)   / (img.cols * img.rows);

    std::cout << "salt : " << salt << " & pebber: " << pebber << '\n';
    double quantile = (pebber-salt+1)/2;
    std::cout << "quantile: " << quantile << '\n';
    return quantile;
}
