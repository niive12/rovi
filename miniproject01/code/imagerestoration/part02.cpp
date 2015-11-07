#include "part02.h"
#include "part01.h" //this is bad, I only needed sp_damage...
#include "mean_filter.h"

void stat(const cv::Mat &img_src, cv::Mat &histImage, double &mean, double &var, int border=5){
    cv::Mat temp = img_src.clone();
    cv::Mat img = temp(cv::Rect(border,border,img_src.cols-2*border, img_src.rows-2*border));
    temp = img.clone();
    cv::Mat histogram;
    cv::Mat histimg( 256, 512, CV_8UC3, cv::Scalar::all(255) );

    make_histogram(img,histimg,histogram,512,256);
    histImage = histimg.clone();

    int histSize = histogram.rows;
    long long unsigned sum = 0;
    for(int x = 0; x < img.cols; ++x){
        for(int y = 0; y < img.rows; ++y){
            sum += img.at<float>(cv::Point(x,y));
        }
    }
    mean = sum/(img.rows*img.cols) ;

    sum = 0;
    for(int x = 0; x < img.cols; ++x){
        for(int y = 0; y < img.rows; ++y){
            sum += ((img.at<float>(cv::Point(x,y))-mean)*(img.at<float>(cv::Point(x,y))-mean));
        }
    }
    var = 1.0/((img.rows*img.cols) -1)  * sum;
}

void part02(cv::Mat image, cv::Mat out){
    cv::Mat temp, temp2;
    cv::Mat org = image.clone();
    cv::Mat uniform(org,AREA_UNIFORM);

    //investigate uniform area
    double quantile = what_is_the_S_P_damage(uniform,"histogram of uniform area");
    cv::imshow("Uniform area", uniform/255);
    temp = uniform.clone();
    median_filter(uniform, temp,5,1,quantile);
    temp2 = temp.clone();
    cv::imshow("median filter 5,1", temp / 255);
    applyHomomorphicBlur(temp2,temp,5);
    cv::imshow("homomophic blured", temp / 255);
    temp2 = temp.clone();
    double mean,var;
    stat(temp2,temp,mean,var);
    cv::imshow("histogram before",temp);
    std::cout << "mean: " << mean << " var: " << var << '\n';
    applyHomomorphicBilatteral(temp2,temp,11,var,3);
        cv::imshow("bilateral 11, 3, homomorphic uniform", temp / 255);
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
        cv::imshow("bilateral 11, 3, homomorphic", temp / 255);
    temp2 = temp.clone();
    median_filter(temp2,temp,7,3);
        cv::imshow("median 2", temp / 255);
    temp2 = temp.clone();
    applyHistogramEqualization(temp2,temp,15);
        cv::imshow("hist eq", temp / 255);
    cv::waitKey(0);
    return;
    //apply to large image
    median_filter(image, out,5,1,quantile);
    applyHarmonicMean(out, temp, 5);
    applyHomomorphicBilatteral(temp,out,11,var,3);
    median_filter(out,temp,7,3);
    applyHistogramEqualization(temp,out,15);
}
