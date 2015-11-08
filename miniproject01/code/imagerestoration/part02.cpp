#include "part02.h"
#include "mean_filter.h"

void part02(cv::Mat image, cv::Mat out){
    cv::Mat temp, temp2;
    cv::Mat org = image.clone();
    cv::Mat uniform(org,AREA_UNIFORM);
    cv::Mat histImg, histogram;
    //investigate uniform area
    make_histogram(uniform,histImg,histogram);
    double quantile = what_is_the_S_P_damage(uniform);
        cv::imwrite("../../report/graphics/hist2_uniform.png",histImg);
        cv::imshow("Uniform area", uniform/255);

    temp = uniform.clone();
    median_filter(uniform, temp,5,1,quantile);
    temp2 = temp.clone();
        cv::imshow("median filter 5,1", temp / 255);
    applyHomomorphicBlur(temp2,temp,5);
    make_histogram(temp,histImg,histogram);
        cv::imwrite("../../report/graphics/hist2_after_median.png",histImg);
        cv::imshow("homomophic blured", temp / 255);
    temp2 = temp.clone();
    double mean,var;
    stat(temp2,temp,mean,var);
    cv::imshow("histogram before",temp);
    std::cout << "mean: " << mean << " var: " << var << '\n';
    applyHomomorphicBilatteral(temp2,temp,11,var,3);
        cv::imshow("bilateral 11, 3, homomorphic uniform", temp / 255);
    make_histogram(temp,histImg,histogram);
        cv::imwrite("../../report/graphics/hist2_after_bilatteral.png",histImg);
    double new_var;
    cv::imshow("median2", temp2 / 255);
    stat(temp2,temp,mean,new_var);
    cv::imshow("histogram after",temp);
    std::cout << "mean: " << mean << " var: " << new_var << '\n';
    temp = temp2.clone();
    cv::waitKey(0);

    cvDestroyAllWindows();
    cv::Mat complex(org,AREA_COMPLEX);
    median_filter(complex,temp,5,1,quantile);
    temp2 = temp.clone();
        cv::imshow("median filtered", temp / 255);
    applyHarmonicMean(temp2,temp,5);
    temp2 = temp.clone();
        cv::imshow("harmonic mean", temp / 255);
    applyHomomorphicBilatteral(temp2,temp,11,var,3);
        cv::imwrite("../../report/graphics/complex2_bilatteral.png",temp);
        cv::imshow("bilateral 11, 3, homomorphic", temp / 255);
    temp2 = temp.clone();
    applyHistogramEqualization(temp2,temp,15);
    cv::imwrite("../../report/graphics/complex2_histeq.png",temp);
    cv::imshow("hist eq", temp / 255);
    temp = temp2.clone();
    median_filter(temp2,temp,7,3);
        cv::imshow("median 2", temp / 255);
    temp2 = temp.clone();
    applyHistogramEqualization(temp2,temp,15);
    cv::imwrite("../../report/graphics/complex2_histeq_smoothed.png",temp);
        cv::imshow("hist eq after", temp / 255);
    cv::waitKey(0);
//    return;
    //apply to large image
    median_filter(image, out,5,1,quantile);
    applyHarmonicMean(out, temp, 5);
    applyHomomorphicBilatteral(temp,out,11,var,3);
    median_filter(out,temp,7,3);
    applyHistogramEqualization(temp,out,15);
}
