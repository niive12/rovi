#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
// input
// output = the image
// hist = vector of raw data
// hist_XX = dimensions of 'output'
void make_histogram(cv::Mat &input, cv::Mat &hist_output_image, cv::Mat &histogram_data, int hist_width, int hist_height){

    int histSize = 256;
    float range[] = { 0, 256 } ;
    const float* histRange = { range };

    cv::Mat hist;
    cv::Mat histImage(hist_height, hist_width, CV_8UC3 );

    cv::calcHist( &input, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);

    int bin_w = cvRound( (double) hist_width/histSize );
    double max = 0;
    cv::minMaxLoc(hist, nullptr, &max);

    double scale = double(hist_height)/max ;
    histImage.setTo(cv::Scalar::all(255));
    for( int i = 0; i < histSize; ++i ){
        cv::rectangle(histImage,
                      cv::Rect(bin_w*(i),
                               hist_height-cvRound(scale * hist.at<float>(i)),
                               bin_w,
                               cvRound(scale * hist.at<float>(i))),
                      cv::Scalar::all(170),
                      CV_FILLED);
    }
    cv::Mat hist_image;
    cv::copyMakeBorder(histImage, hist_image,0,1,0,0,cv::BORDER_CONSTANT, cv::Scalar::all(0));
    hist_output_image = hist_image.clone();
    histogram_data = hist.clone();
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
            image.at<float>(y, x) = cv::saturate_cast<float>(new_bin[cvRound(image.at<float>(y, x))]);
        }
    }
}

void applyHistogramEqualization(cv::Mat &img_src, cv::Mat &img_out, int border){
    cv::Mat temp = img_src.clone();
    cv::Mat img = temp(cv::Rect(border,border,img_src.cols-2*border, img_src.rows-2*border));
    temp = img.clone();
    cv::Mat histogram;
    cv::Mat histImage(256, 512, CV_8UC3, cv::Scalar::all(255) );

    make_histogram(img,histImage,histogram,512,256);
    img_out = img.clone();
    make_histogram_equalization(img_out, histogram);
}

void stat(const cv::Mat &img_src, cv::Mat &histImage, double &mean, double &var, int border){
    cv::Mat temp = img_src.clone();
    cv::Mat img = temp(cv::Rect(border,border,img_src.cols-2*border, img_src.rows-2*border));
    temp = img.clone();
    cv::Mat histogram;
    cv::Mat histimg( 256, 512, CV_8UC3, cv::Scalar::all(255) );

    make_histogram(img,histimg,histogram,512,256);
    histImage = histimg.clone();

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

double what_is_the_S_P_damage(cv::Mat &img, std::string frame){
    double salt, pebber;
    cv::Mat histogram;
    cv::Mat histImage( 256, 512, CV_8UC3 );
    std::cout << '\n';
    make_histogram(img,histImage,histogram,512,256);
    if(frame != "none")
        cv::imshow(frame,histImage);
    salt   = histogram.at<float>(255) / (img.cols * img.rows);
    pebber = histogram.at<float>(0)   / (img.cols * img.rows);

    std::cout << "salt : " << salt << " & pebber: " << pebber << '\n';
    double quantile = (pebber-salt+1)/2;
    std::cout << "quantile: " << quantile << '\n';
    return quantile;
}

