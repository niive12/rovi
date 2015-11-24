#include <iostream>
#include <fstream>
#include <string>

#include <rw/math/Q.hpp>
#include <rw/rw.hpp>
#include <rw/math/Math.hpp>


rw::math::Vector3D<double> W_rot(rw::math::Rotation3D<double> &R){
    double theta = acos((R(0,0) + R(1,1) + R(2,2) - 1) / 2);

    rw::math::Vector3D<double> W(0,0,0);
    W[0] = (R(2,1) - R(1,2));
    W[1] = (R(0,2) - R(2,0));
    W[2] = (R(1,0) - R(0,1));

    if(theta < 10e-6 || theta > (rw::math::Pi - 10e-6)){
        W *= 0.5;
    } else{
        double length = W.norm2();
        W *= (theta / length);
    }
    return W;
}


int main(){

     rw::math::Transform3D<double> F_0(rw::math::Vector3D<double>(15, 8, 3), rw::math::Rotation3D<double>::identity());
     rw::math::Transform3D<double> F_1(rw::math::Vector3D<double>(10, 4, 2), rw::math::Rotation3D<double>(0, 1, 0, -1, 0, 0, 0, 0, 1));
     rw::math::Transform3D<double> F_2(rw::math::Vector3D<double>(6, 0, -2), rw::math::Rotation3D<double>(0, 0, 1, -1, 0, 0, 0, -1, 0));

     rw::math::Rotation3D<double> i = (F_0.R().inverse()) * F_1.R();
     rw::math::Vector3D<double> W = W_rot(i);
     std::cout << W << "\n";

     i = (F_1.R().inverse()) * F_2.R();
     W = W_rot(i);
     std::cout << W << "\n";

    return 0;
}

