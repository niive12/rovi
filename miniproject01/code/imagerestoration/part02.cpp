#include "part02.h"
#include "mean_filter.h"

void part02(cv::Mat &image,
            cv::Mat &final_out,
            cv::Mat &hist_uniform,
            cv::Mat &hist_median,
            cv::Mat &hist_bilatteral,
            cv::Mat &complex_median,
            cv::Mat &complex_harmonic,
            cv::Mat &complex_bilatteral,
            cv::Mat &complex_histeq,
            cv::Mat &complex_smoothed_histeq
            ){
    cv::Mat temp, temp2;
    cv::Mat org = image.clone();
    cv::Mat uniform(org,AREA_UNIFORM);
    cv::Mat histImg, histogram;

//    investigate uniform area
    make_histogram(uniform,histImg,histogram);
    double quantile = what_is_the_S_P_damage(uniform);
        hist_uniform = histImg.clone();

    temp = uniform.clone();
    temp2 = temp.clone();
    median_filter(uniform, temp,5,1,quantile);
    temp2 = temp.clone();
    applyHarmonicMean(temp2,temp,5);
        make_histogram(temp,histImg,histogram);
        hist_median = histImg.clone();
    temp2 = temp.clone();
    double mean,var;
    stat(temp2,temp,mean,var,10);
    std::cout << "mean: " << mean << " var: " << var << '\n';
    cv::bilateralFilter(temp2,temp,11,var,3);
    temp2 = temp.clone();
    make_histogram(temp,histImg,histogram);
    hist_bilatteral = histImg.clone();
    double new_var;
    stat(temp2,temp,mean,new_var,10);
    std::cout << "mean: " << mean << " var: " << new_var << '\n';
    temp = temp2.clone();

//    Investigate complex area
    cv::Mat complex(org,AREA_COMPLEX);
    median_filter(complex,temp,5,1,quantile);
    complex_median = temp.clone();
    applyHarmonicMean(complex_median,temp,5);
    complex_harmonic = temp.clone();
    cv::bilateralFilter(complex_harmonic,temp,11,var,3);
    complex_bilatteral = temp.clone();
//    See if the image gets better with equalization
    applyHistogramEqualization(complex_bilatteral,temp,15);
    complex_histeq = temp.clone();

    median_filter(temp2,temp,7,3);
    temp2 = temp.clone();
    applyHistogramEqualization(temp2,temp,15);
    complex_smoothed_histeq = temp.clone();

//    apply to large image
    median_filter(image, final_out,5,1,quantile);
    applyHarmonicMean(final_out, temp, 5);
    cv::bilateralFilter(temp,final_out,11,var,3);
}
