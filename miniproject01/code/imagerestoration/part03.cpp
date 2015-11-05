#include "part03.h"

void part03(cv::Mat_<float> &original_image,
            cv::Mat_<float> &out_image_geo,
            cv::Mat_<float> &out_image_har,
            cv::Mat_<float> &out_image_ari,
            cv::Mat_<float> &out_hist_uni_geo,
            cv::Mat_<float> &out_hist_uni_har,
            cv::Mat_<float> &out_hist_uni_ari ){
    // only consider uniform area
    // make histogram of it
    cv::Mat_<float> image_uniform_03(original_image.clone(), AREA_UNIFORM);
    cv::Mat histogram;
    cv::Mat histImage( 256, 512, CV_8UC3 );
    make_histogram(image_uniform_03,histImage,histogram,512,256);

    // problem is uniform to normal noise
    // is a filter to solve this, adaptive? geometric mean? harmonic mean? arithmetic mean?
    cv::Mat_<float> image_harmonic(original_image.clone());
    applyHarmonicMean(image_harmonic, out_image_har, 11);

    cv::Mat_<float> image_geometric(original_image.clone());
    applyGeometricMean(image_geometric, out_image_geo, 11);

    cv::Mat_<float> image_arithmetic(original_image.clone());
    applyArithmeticMean(image_arithmetic, out_image_ari, 11);


    cv::Mat_<float> image_uniform_03_har(image_harmonic.clone(), AREA_UNIFORM);
    cv::Mat histogram_har;
    make_histogram(image_uniform_03_har,histImage_har,histogram_har,512,256);

    cv::Mat_<float> image_uniform_03_geo(image_geometric.clone(), AREA_UNIFORM);
    cv::Mat histogram_geo;
    make_histogram(image_uniform_03_geo,out_hist_uni_geo,histogram_geo,512,256);

}
