#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main(int argc, char** argv )
{
    cout << "Hello World!" << endl;
    if ( argc != 2 ) {
        //No single image passed as argument
        return -1;
    }

    cv::Mat image = cv::imread( string(argv[1]) );

    return 0;
}



