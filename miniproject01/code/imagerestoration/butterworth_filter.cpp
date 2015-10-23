#include "butterworth_filter.h"


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
