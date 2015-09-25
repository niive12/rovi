#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void median_filter(cv::Mat &input, cv::Mat &output, int kernel_size, int mean_width){
    if( (input.type() + output.type()) != 0){
        throw("error");
    }
    int length = kernel_size*kernel_size;
    uchar neighbours[length];
    uchar temp;
    int j;
    int middel_value = (length) / 2;
    int middel_offset = mean_width / 2;
    int kernel_offset = kernel_size / 2;
    int mean;

    for(int x=kernel_offset; x<input.cols-kernel_offset; ++x){ //borders are just kept; in a 3 by 3 kernel this means 1st pixel is ignored
        for(int y=kernel_offset; y<input.rows-kernel_offset; ++y){
            //gather neighbouring pixels in array
            for(int k_x=0;k_x<kernel_size;++k_x){
                for(int k_y=0; k_y<kernel_size; ++k_y){
                    neighbours[k_x+k_y*kernel_size] = input.at<uchar>(y+(k_y-kernel_offset),x+(k_x-kernel_offset));
                }
            }
            //sort the array
            for(int i = 1; i<kernel_size*kernel_size;++i){
                j = i;
                while(neighbours[j] < neighbours[j-1] && j > 0){
                    temp = neighbours[j-1];
                    neighbours[j-1] = neighbours[j];
                    neighbours[j--] = temp;
                }
            }
            //mean of middle values
            mean = 0;
            for(int i=(middel_value-middel_offset); i<=(middel_value+middel_offset);++i){
                mean += neighbours[i];
            }
            mean = mean / mean_width;
            output.at<uchar>(y, x) = mean;
        }
    }
}
