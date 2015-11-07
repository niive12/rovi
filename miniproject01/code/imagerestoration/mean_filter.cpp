#include "mean_filter.h"


void applyGeometricMean(cv::Mat &img_src, cv::Mat &img_dst, int kernelsize){
    // prod(g())^(1/mn)
    cv::Mat img_result(img_src.clone());
    img_dst.copySize(img_src);
    if(kernelsize % 2 == 0){
        std::cout << "ERROR: K is even.\n";
    } else {
        int k = (kernelsize - 1)/2;
        // loop through image - border
        for(int i = k; i < img_src.rows - k; i++){
            for(int j = k; j < img_src.cols - k; j++){
                // loop through filter
                float filterres = 1;
                for(int s = -k; s <= k; s++){
                    for(int t = -k; t <= k; t++){
                        filterres *= pow((img_src.at<float>(i+s, j+k)),(1/pow(kernelsize,2)));
                    }
                }
                img_result.at<float>(i,j) = filterres;
            }
        }
        img_dst = img_result;
    }
}

void applyHarmonicMean(cv::Mat &img_src, cv::Mat &img_dst, int kernelsize){
    // mn/(sum(1/g()))
    cv::Mat img_result(img_src.clone());
    img_dst.copySize(img_src);
    if(kernelsize % 2 == 0){
        std::cout << "ERROR: K is even.\n";
    } else {
        int k = (kernelsize - 1)/2;
        // loop through image - border
        for(int i = k; i < img_src.rows - k; i++){
            for(int j = k; j < img_src.cols - k; j++){
                // loop through filter
                float filterres = 0;
                for(int s = -k; s <= k; s++){
                    for(int t = -k; t <= k; t++){
                        filterres += 1/(img_src.at<float>(i+s, j+k));
                    }
                }
                filterres = pow(kernelsize,2)/filterres;
                img_result.at<float>(i,j) = filterres;
            }
        }
        img_dst = img_result;
    }
}

void applyArithmeticMean(cv::Mat &img_src, cv::Mat &img_dst, int kernelsize){
    // avg sum of elements
    cv::blur(img_src, img_dst, cv::Size(kernelsize,kernelsize));
}

