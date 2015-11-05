#include "analyse_freq.h"

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
    cv::normalize(image_full, image_full, 0, 1, CV_MINMAX);
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
