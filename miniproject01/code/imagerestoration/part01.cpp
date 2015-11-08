#include "part01.h"
#include "analyse_freq.h"
#include "mean_filter.h"

void part01(cv::Mat &original_image,
            cv::Mat &output_image,
            cv::Mat &hist_uniform,
            cv::Mat &hist_median,
            cv::Mat &complex_median,
            cv::Mat &complex_geometric_blurred,
            cv::Mat &complex_harmonic_blurred,
            cv::Mat &complex_difference
            ){
    cv::Mat org = original_image.clone();
    cv::Mat temp_image;
    cv::Mat uniform(org,AREA_UNIFORM);
    cv::Mat out = uniform.clone();
    cv::Mat histImg;
    cv::Mat histogram;

    make_histogram(uniform,histImg,histogram);
    hist_uniform = histImg.clone();
    double quantile = what_is_the_S_P_damage(uniform);

    median_filter(uniform, out,3,1,quantile);
    what_is_the_S_P_damage(out);
    make_histogram(out,histImg,histogram);
    uniform = out.clone();
    median_filter(uniform, out,7,3,quantile);
    what_is_the_S_P_damage(out);
    make_histogram(out,histImg,histogram);
    hist_median = histImg.clone();

    cv::Mat complex(org,AREA_COMPLEX);

    out = complex.clone();
    temp_image = complex.clone();
    median_filter(complex, temp_image, 3, 1, quantile);
    complex = temp_image.clone();

    out = temp_image.clone();

    median_filter(out, temp_image, 7, 3, quantile);

    complex = temp_image.clone();
    complex_median = temp_image.clone();

    applyGeometricMean(complex,temp_image,5);
    complex_geometric_blurred = temp_image.clone();

    applyHarmonicMean(complex,out,5);
    complex_harmonic_blurred = out.clone();
//              white                    - black
    complex = ((complex_harmonic_blurred - complex_geometric_blurred)*50 + 127);
    complex_difference = complex.clone();

    //Apply all this to the large image:
    output_image = original_image.clone();
    median_filter(original_image, output_image,3,1,quantile);
    temp_image = output_image.clone();
    median_filter(temp_image, output_image,7,3,quantile);
    temp_image = output_image.clone();
    applyHarmonicMean(temp_image,output_image,5);
}
