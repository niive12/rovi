#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "histogram.h"
#include "median_filter.h"
#include <vector>

#define AREA_UNIFORM cv::Rect(810,1500,300,300)
#define AREA_SIMPLE  cv::Rect(900,1225,300,300)
#define AREA_COMPLEX cv::Rect(1225,250,300,300)

void dftshift(cv::Mat_<float>& magnitude) {
   const int cx = magnitude.cols/2;
   const int cy = magnitude.rows/2;

   cv::Mat_<float> tmp;
   cv::Mat_<float> topLeft(magnitude, cv::Rect(0, 0, cx, cy));
   cv::Mat_<float> topRight(magnitude, cv::Rect(cx, 0, cx, cy));
   cv::Mat_<float> bottomLeft(magnitude, cv::Rect(0, cy, cx, cy));
   cv::Mat_<float> bottomRight(magnitude, cv::Rect(cx, cy, cx, cy));

   topLeft.copyTo(tmp);
   bottomRight.copyTo(topLeft);
   tmp.copyTo(bottomRight);

   topRight.copyTo(tmp);
   bottomLeft.copyTo(topRight);
   tmp.copyTo(bottomLeft);
}


float butter(int u, int v, float D0, int n, int P, int Q){
    float butter = 1/(1 + pow(D0 / pow( pow( u - P/2, 2) + pow( v - Q/2, 2), 0.5 ), 2 * n) );

    return butter;
}

// makes the butterworth mask for a shifted dft by multiplying the coefficient to the dst
void butterFilter(cv::Mat_<float> &dst, int u, int v, float D0, int n){
    int p = dst.rows;
    int q = dst.cols;

    for(int row = 0; row < p; row++){
        for(int col = 0; col < q; col++){
            dst.at<float>(row, col) = dst.at<float>(row, col) * butter(row - u, col + v, D0, n, p, q) * butter(row + u, col - v, D0, n, p, q);
        }
    }
}

void part01(cv::Mat_<float> &original_image, cv::Mat_<float> &output_image);
void part04(cv::Mat_<float> &original_image, cv::Mat_<float> &output_image);

void visualize_frequency(cv::Mat_<float> &original_image, cv::Mat_<float> &output_image);


int main(){
    std::vector<std::string> image_names;
    image_names.push_back("../images/Image1.png"         );
    image_names.push_back("../images/Image2.png"         );
    image_names.push_back("../images/Image3.png"         );
    image_names.push_back("../images/Image4_1.png"       );
    image_names.push_back("../images/Image4_2.png"       );
    image_names.push_back("../images/Image5_optional.png");

    cv::Mat_<float> image = cv::imread( image_names.at(0) , CV_LOAD_IMAGE_GRAYSCALE );
    cv::Mat_<float> modified = image.clone();
    cv::Mat_<float> out = image.clone();
    part01(modified,out);
    namedWindow("Restored Image", cv::WINDOW_NORMAL);
    cv::normalize(image,image,0,1,CV_MINMAX);
    cv::imshow("Image", image);

    std::cout << "Image loaded\n";


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


//    cv::Mat_<float> image_res = image.clone();
//    part04(image, image_res);
//    cv::waitKey(0);


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

void part04(cv::Mat_<float> &original_image, cv::Mat_<float> &output_image){
    std::cout << "Running for the 4th image!\n";
    // make padded image
    cv::Mat_<float> padded;

    int m = cv::getOptimalDFTSize( original_image.rows * 2 );
    int n = cv::getOptimalDFTSize( original_image.cols * 2 ); // on the border add zero values
    cv::copyMakeBorder(original_image, padded, 0, m - original_image.rows, 0, n - original_image.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

    // put real and imaginary part of image together
    cv::Mat_<cv::Vec2f> image_full;
    std::vector<cv::Mat> image_vec;
    image_vec.push_back(padded.clone());
    image_vec.push_back((cv::Mat::zeros(padded.rows, padded.cols, CV_32F)));

    // merge the two images
    cv::merge(image_vec,image_full);

    // make frequency domain image
    cv::dft(image_full,image_full);

    // split in two and make phase / magnitude
    cv::Mat_<float> mag, phase;

    cv::split(image_full, image_vec);
    cv::cartToPolar(image_vec[0], image_vec[1], mag, phase);

    // shift the quarters
    dftshift(mag);

    // make mask for frequency domain
    cv::Mat_<float> mask = cv::Mat::ones(mag.rows, mag.cols, CV_32F);

    butterFilter(mask, 615, 615, 50, 2);
    butterFilter(mask, 200, -200, 20, 2);

    // apply mask
    cv::mulSpectrums(mag, mask, mag, 0);

    // merge them back
    dftshift(mag);
    cv::polarToCart(mag, phase, image_vec[0], image_vec[1]);
    cv::merge(image_vec,image_full);

    // inverse dft
    cv::Mat_<float> final_image;
    cv::dft(image_full, final_image, cv::DFT_INVERSE + cv::DFT_SCALE + cv::DFT_REAL_OUTPUT);

    // output image
    final_image = final_image(cv::Rect(0,0,final_image.cols/2,final_image.rows/2));
    cv::normalize(final_image, final_image, 0, 1, CV_MINMAX);

    output_image = final_image;
}


// takes in a real image returns the frequency image
void visualize_frequency(cv::Mat_<float> &original_image, cv::Mat_<float> &output_image){
    std::cout << "Visualize image in frequency domain!\n";
    // make padded image
    cv::Mat_<float> padded;

    int m = cv::getOptimalDFTSize( original_image.rows * 2 );
    int n = cv::getOptimalDFTSize( original_image.cols * 2 ); // on the border add zero values
    cv::copyMakeBorder(original_image, padded, 0, m - original_image.rows, 0, n - original_image.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

    // put real and imaginary part of image together
    cv::Mat_<cv::Vec2f> image_full; // used to store real and imaginary part
    std::vector<cv::Mat> image_vec;
    image_vec.push_back(padded.clone());
    image_vec.push_back((cv::Mat::zeros(padded.rows, padded.cols, CV_32F)));

    // merge the two images
    cv::merge(image_vec,image_full);

    // make frequency domain image
    cv::dft(image_full,image_full);

    // split in two and make phase / magnitude
    cv::Mat_<float> mag, phase;

    cv::split(image_full, image_vec);
    cv::cartToPolar(image_vec[0], image_vec[1], mag, phase);

    // shift the quarters
    dftshift(mag);

    // log / normailze
    mag = mag + cv::Mat::ones(mag.rows, mag.cols, CV_32F);
    cv::log(mag, mag);
    cv::normalize(mag, mag, 0, 1, CV_MINMAX);

    cv::namedWindow("Frequency Magnitude Plot", cv::WINDOW_NORMAL);
    cv::imshow("Frequency Magnitude Plot", mag);

    output_image = mag;
}
