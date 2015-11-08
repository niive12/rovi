#include "part01.h"
#include "analyse_freq.h"
#include "mean_filter.h"

void part01(cv::Mat_<float> &original_image, cv::Mat_<float> &output_image){
    cv::Mat org = original_image.clone();
    cv::Mat temp_image;
    cv::Mat uniform(org,AREA_UNIFORM);
    cv::Mat out = uniform.clone();
    cv::Mat histImg;
    cv::Mat histogram;

    make_histogram(uniform,histImg,histogram);
    cv::imwrite("../../report/graphics/hist1_uniform.png", histImg);
    double quantile = what_is_the_S_P_damage(uniform);

    median_filter(uniform, out,3,1,quantile);
    what_is_the_S_P_damage(out);
    make_histogram(out,histImg,histogram);
    cv::imshow("../../report/graphics/hist1_uniform1.png", histImg);
    uniform = out.clone();
    median_filter(uniform, out,7,3,quantile);
    what_is_the_S_P_damage(out);
    make_histogram(out,histImg,histogram);
    cv::imwrite("../../report/graphics/hist1_uniform2.png", histImg);

    cv::imshow("uniform area", uniform / 255);
    cv::imshow("filtered uniform area", out / 255);
    cv::waitKey(0);
//    cv::destroyAllWindows();
    cv::Mat complex(org,AREA_COMPLEX);

    out = complex.clone();
    cv::imshow("Original image", out / 255);
    temp_image = complex.clone();
    median_filter(complex, temp_image, 3, 1, quantile);
    complex = temp_image.clone();

    out = temp_image.clone();
    cv::imshow("3x3 median filter", out / 255);

    median_filter(out, temp_image, 7, 3, quantile);

    complex = temp_image.clone();
    cv::imwrite("../../report/graphics/complex1_step2.png", temp_image);
    cv::imshow("7x7 median filter, mean of 3", temp_image / 255);

    cv::blur(complex,temp_image,cv::Size(5,5));
    cv::imwrite("../../report/graphics/complex1_blurred.png", temp_image);
    cv::imshow("alm smooth",temp_image / 255);

    applyHomomorphicBlur(complex,out,5);
    cv::imwrite("../../report/graphics/complex1_homomorphic_blurred.png", out);
    cv::imshow("homomorphic filtered",out / 255);
//              white - black
    complex = ((out   - temp_image)*50 + 127);
    cv::imwrite("../../report/graphics/complex1_blurr_difference_50.png",complex);

    cv::waitKey(0);
    //Apply all this to the large image:
    output_image = original_image.clone();
    median_filter(original_image, output_image,3,1,quantile);
    temp_image = output_image.clone();
    median_filter(temp_image, output_image,7,3,quantile);
    temp_image = output_image.clone();
    applyHomomorphicBlur(temp_image,output_image,5);
}
