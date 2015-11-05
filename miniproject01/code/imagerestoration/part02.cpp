#include "part02.h"
#include "part01.h" //this is bad, I only needed sp_damage...

void part02(cv::Mat image, cv::Mat out){
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
    median_filter(uniform, temp,5,1,quantile);
    cv::normalize(temp,temp,0,255,CV_MINMAX);
    temp.convertTo(temp,CV_8U);
    what_is_the_S_P_damage(temp,"new hist");
    cv::imshow("new", temp);
    cv::waitKey(0);
    //apply to large image
    median_filter(image, out,5,1,quantile);
    cv::normalize(out,out,0,1,CV_MINMAX);
    cv::imshow("Figure_2", out);
}
