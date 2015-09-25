#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void make_histogram(cv::Mat &input, cv::Mat &output, cv::Mat &histogram, int hist_width, int hist_height){

    int histSize = 256;
    float range[] = { 0, 255 } ;
    const float* histRange = { range };

    cv::Mat hist;

    cv::calcHist( &input, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);

    int bin_w = cvRound( (double) hist_width/histSize );
    double max = 0;
    cv::minMaxLoc(hist, nullptr, &max);

    double scale = double(hist_height)/max ;
    output.setTo(cv::Scalar::all(255));
    for( int i = 0; i < histSize; ++i ){
        cv::rectangle(output,
                      cv::Rect(bin_w*(i),
                      hist_height-cvRound(scale * hist.at<float>(i)),
                      bin_w,
                      cvRound(scale * hist.at<float>(i))),
                      cv::Scalar::all(170),
                      CV_FILLED);
    }
    cv::Mat hist_image;
    cv::copyMakeBorder(output, hist_image,1,1,1,1,cv::BORDER_CONSTANT, cv::Scalar::all(0));
    output = hist_image.clone();
    histogram = hist.clone();
}
