#include "part03.h"

void part03(cv::Mat_<float> &original_image,
            cv::Mat_<float> &out_hist_normal,
            cv::Mat_<float> &out_image_geo,
            cv::Mat_<float> &out_image_har,
            cv::Mat_<float> &out_image_ari,
            cv::Mat_<float> &out_hist_uni_geo,
            cv::Mat_<float> &out_hist_uni_har,
            cv::Mat_<float> &out_hist_uni_ari){

    int k = 5;

    // only consider uniform area
    // make histogram of it
    cv::Mat_<float> image_uniform_03(original_image.clone(), AREA_UNIFORM);
    cv::Mat histogram;
    make_histogram(image_uniform_03,out_hist_normal,histogram,512,256);

    // problem is uniform to normal noise
    // is a filter to solve this, adaptive? geometric mean? harmonic mean? arithmetic mean?
    // apply different filters
    cv::Mat_<float> image_harmonic(original_image.clone());
    applyHarmonicMean(image_harmonic, image_harmonic, k);
    out_image_har = image_harmonic;

    cv::Mat_<float> image_geometric(original_image.clone());
    applyGeometricMean(image_geometric, image_geometric, k);
    out_image_geo = image_geometric;

    cv::Mat_<float> image_arithmetic(original_image.clone());
    applyArithmeticMean(image_arithmetic, image_arithmetic, k);
    out_image_ari = image_arithmetic;

    // make hist of the uniform areafor the 3 images
    cv::Mat_<float> image_uniform_03_har(image_harmonic.clone(), AREA_UNIFORM);
    cv::Mat histogram_har;
    make_histogram(image_uniform_03_har,out_hist_uni_har,histogram_har,512,256);

    cv::Mat_<float> image_uniform_03_geo(image_geometric.clone(), AREA_UNIFORM);
    cv::Mat histogram_geo;
    make_histogram(image_uniform_03_geo,out_hist_uni_geo,histogram_geo,512,256);

    cv::Mat_<float> image_uniform_03_ari(image_arithmetic.clone(), AREA_UNIFORM);
    cv::Mat histogram_ari;
    make_histogram(image_uniform_03_ari,out_hist_uni_ari,histogram_ari,512,256);

}
