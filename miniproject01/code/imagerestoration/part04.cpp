#include "part04.h"

void part04(cv::Mat_<float> &original_image, cv::Mat_<float> &output_image){
    std::cout << "Image 4:\n";
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

    double size_factor = 3;
    int power = 1;
    butterFilter(mask, 206, -201, 15*size_factor, power);
    butterFilter(mask, 623, 604, 30*size_factor, power);

    // apply mask
    cv::multiply(mag, mask, mag);

    // merge them back
    dftshift(mag);
    cv::polarToCart(mag, phase, image_vec[0], image_vec[1]);
    cv::merge(image_vec,image_full);

    // inverse dft
    cv::Mat_<float> final_image;
    cv::dft(image_full, final_image, cv::DFT_INVERSE + cv::DFT_REAL_OUTPUT );

    // output image
    final_image = final_image(cv::Rect(0,0,original_image.cols,original_image.rows));

    output_image = final_image;
}


