#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// input
// output = the image
// hist = vector of raw data
// hist_XX = dimensions of 'output'
void make_histogram(cv::Mat &input, cv::Mat &output, cv::Mat &histogram, int hist_width, int hist_height){

    int histSize = 256;
    float range[] = { 0, 256 } ;
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

void make_histogram_equalization(cv::Mat &image,
                                 cv::Mat &histogram){
    int histSize = histogram.rows;
    long int sum = 0;
    for(int i = 0; i < histSize; ++i){
        sum += histogram.at<float>(i);
    }
    int average = sum/histSize;

    int current_sum = 0;
    int new_bin[histSize];
    int new_hist[histSize];
    int current_color = 0;
    int over_flow_error = 0;
    for(int array_index = 0; array_index < histSize; ++array_index){
        while(current_sum > average){ // hvis der er overflow (mere end 1).
            current_sum = current_sum - average;
            ++current_color;
            over_flow_error = current_sum;
        }
        current_sum += cvRound(histogram.at<float>(array_index));
        new_bin[array_index] = current_color;

        new_hist[current_color] = current_sum - over_flow_error;
    }

    //apply to image

    for(int x=0; x<image.cols; ++x){
        for(int y=0; y<image.rows; ++y){
            image.at<uchar>(y, x) = cv::saturate_cast<uchar>(new_bin[image.at<uchar>(y, x)]);
        }
    }
}
