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
    std::cout << "Image 2:\n";
//    investigate uniform area
    make_histogram(uniform,histImg,histogram);
    double quantile = what_is_the_S_P_damage(uniform);
        hist_uniform = histImg.clone();

    temp = uniform.clone();
    temp2 = temp.clone();
    median_filter(uniform, temp,5,1,quantile);
    temp2 = temp.clone();
//        cv::imshow("median filter 5,1", temp / 255);
    applyHarmonicMean(temp2,temp,5);
//        cv::imshow("harmonic mean", temp / 255);
        make_histogram(temp,histImg,histogram);
        hist_median = histImg.clone();
    temp2 = temp.clone();
    double mean,var;
    stat(temp2,temp,mean,var,10);
    std::cout << "image 2, after median, \tmean: " << mean << " var: " << var << '\n';
    cv::bilateralFilter(temp2,temp,11,var,3);
    temp2 = temp.clone();
//        cv::imshow("bilateral 11, 3, uniform", temp / 255);
    make_histogram(temp,histImg,histogram);
        hist_bilatteral = histImg.clone();
//        cv::imshow("histogram after1",histImg);
//        cv::imshow("median2", temp2 / 255);
    double new_var;
    stat(temp2,temp,mean,new_var,10);
    std::cout << "image 2, after bilateral, \tmean: " << mean << " var: " << new_var << '\n';
    temp = temp2.clone();

//    Investigate complex area
    cv::Mat complex(org,AREA_COMPLEX);
    median_filter(complex,temp,5,1,quantile);
//    cv::imshow("median",temp/255);
    complex_median = temp.clone();
    applyHarmonicMean(complex_median,temp,5);
//    cv::imshow("har",temp/255);
    complex_harmonic = temp.clone();
    cv::bilateralFilter(complex_harmonic,temp,11,var,3);
//    cv::imshow("bilateral",temp/255);
    complex_bilatteral = temp.clone();
    applyHistogramEqualization(complex_bilatteral,temp,15);
//    cv::imshow("hiseq",temp/255);
    complex_histeq = temp.clone();
//    See if the image gets better with equalization
    temp2 = temp.clone();
    median_filter(temp2,temp,7,3);
//    cv::imshow("median2",temp/255);
    temp2 = temp.clone();
    applyHistogramEqualization(temp2,temp,15);
//    cv::imshow("histeq alt",temp/255);
    complex_smoothed_histeq = temp.clone();
//    cv::waitKey(0);
//    cv::destroyAllWindows();

//    apply to large image
    median_filter(image, final_out,5,1,quantile);
    applyHarmonicMean(final_out, temp, 5);
    cv::bilateralFilter(temp,final_out,11,var,3);
}
