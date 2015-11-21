#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
void median_filter(const cv::Mat &input, cv::Mat &output, int kernel_size, int mean_width, double offset){
    int length = kernel_size*kernel_size;
    float neighbours[length];
    float temp;
    int j;
    int middel_value = int(double(length) * offset);
    int middel_offset = mean_width / 2;
    int kernel_offset = kernel_size / 2;
    int mean;

    for(int x=kernel_offset; x<input.cols-kernel_offset; ++x){ //borders are just kept; in a 3 by 3 kernel this means 1st pixel is ignored
        for(int y=kernel_offset; y<input.rows-kernel_offset; ++y){
            //gather neighbouring pixels in array
            for(int k_x=0;k_x<kernel_size;++k_x){
                for(int k_y=0; k_y<kernel_size; ++k_y){
                    neighbours[k_x+k_y*kernel_size] = input.at<float>(y+(k_y-kernel_offset),x+(k_x-kernel_offset));
                }
            }
            //sort the array
            for(int i = 1; i<length;++i){
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
            output.at<float>(y, x) = (float) mean;
        }
    }
}

void adaptive_median_filter(const cv::Mat &input, cv::Mat &output){
    int kernel_size = 5;
    int length = kernel_size*kernel_size;
    const int max_kernel = 25;
    float temp;
    int j;
    int middel_value;
    int kernel_offset = kernel_size / 2;
    float median;
    bool finished;
    for(int x=kernel_offset; x<input.cols-kernel_offset; ++x){ //borders are just kept; in a 3 by 3 kernel this means 1st pixel is ignored
        for(int y=kernel_offset; y<input.rows-kernel_offset; ++y){
            kernel_size = 5;
            finished = false;
            while(!finished && (kernel_size < max_kernel)){
                length = kernel_size*kernel_size;
                middel_value = length/2;
                float neighbours[length];

                //gather neighbouring pixels in array
                for(int k_x=0;k_x<kernel_size;++k_x){
                    for(int k_y=0; k_y<kernel_size; ++k_y){
                        neighbours[k_x+k_y*kernel_size] = input.at<float>(y+(k_y-kernel_offset),x+(k_x-kernel_offset));
                    }
                }
                //sort the array
                for(int i = 1; i<length;++i){
                    j = i;
                    while(neighbours[j] < neighbours[j-1] && j > 0){
                        temp = neighbours[j-1];
                        neighbours[j-1] = neighbours[j];
                        neighbours[j--] = temp;
                    }
                }
                //if the midpoint is not equal to the extremes, move on, else increase kernel size
                median = neighbours[middel_value];
                if( (median > neighbours[0]) || (median < neighbours[length]) ){
                    finished = true;
                } else if( ( (y - kernel_size) < 0         ) ||
                           ( (y + kernel_size) > input.rows) ||
                           ( (x - kernel_size) < 0         ) ||
                           ( (x + kernel_size) > input.cols) ){
                    kernel_size += 2;
                } else { //to avoid going outside borders.
                    finished = true;
                }
            }
            output.at<float>(y, x) = median;
        }
    }
}
