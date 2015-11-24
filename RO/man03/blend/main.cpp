#include <iostream>
#include <fstream>
#include <string>

#include <rw/math/Q.hpp>
#include <rw/rw.hpp>
#include <rw/math/Math.hpp>


rw::math::Vector3D<double> W_rot(const rw::math::Rotation3D<double> &R){
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

rw::math::Vector3D<double> p_linear_int_back(const rw::math::Vector3D<double> &P_i_low, const rw::math::Vector3D<double> &P_i, double &t_i_low, double &t_i, double &t){
    rw::math::Vector3D<double> P_t(P_i);
    double scale = (t - t_i);
    scale /= (t_i_low - t_i);
    rw::math::Vector3D<double> p_vec(P_i_low - P_i);

    P_t += (scale * p_vec);
    return P_t;
}

rw::math::Rotation3D<double> R_eaa(const rw::math::Vector3D<double> &v, double &theta){
    rw::math::Rotation3D<double> Reaa;
    double c = cos(theta), s = sin(theta);

    for(int i = 0; i < 3; i++){
        Reaa(i,i) = v[i] * v[i] * (1 - c) + c;
    }
    double v12 = v[0] * v[1] * (1 - c);
    double v13 = v[0] * v[2] * (1 - c);
    double v23 = v[1] * v[2] * (1 - c);
    double v3s = v[2] * s;
    double v2s = v[1] * s;
    double v1s = v[0] * s;

    Reaa(1,0) = v12 + v3s;
    Reaa(2,0) = v13 - v2s;
    Reaa(2,1) = v23 + v1s;
    Reaa(0,1) = v12 - v3s;
    Reaa(0,2) = v13 + v2s;
    Reaa(1,2) = v23 - v1s;

    return Reaa;
}

rw::math::Rotation3D<double> R_eaa(const rw::math::Vector3D<double> &theta_v){
    // R_eaa(theta*v) == R_eaa(v, theta) where ||v|| = 1

    double theta = theta_v.norm2();
    rw::math::Vector3D<double> v = theta_v / theta;

    return R_eaa(v, theta);
}

rw::math::Rotation3D<double> R_linear_int_back(const rw::math::Rotation3D<double> &R_i_low, const rw::math::Rotation3D<double> &R_i, double &t_i_low, double &t_i, double &t){
    double scale = (t - t_i);
    scale /= (t_i_low - t_i);
    rw::math::Rotation3D<double> R_W_rot = R_i; // note that this is done because inverse manipulates with the data it is called on...
    R_W_rot = R_W_rot.inverse();
//    std::cout << " inv: " << R_W_rot << ", r i low: " << R_i_low << "\n";
    R_W_rot = R_i_low * R_W_rot;
    rw::math::Vector3D<double> tmp = scale * W_rot(R_W_rot);
    rw::math::Rotation3D<double> Reaa = R_eaa(tmp);

//    std::cout << "R_lin: Reaa: " << Reaa << ", tmp: " << tmp << ", R_W_rot: " << R_W_rot << ", scale: " << scale << "\n";

    rw::math::Rotation3D<double> R_t = Reaa * R_i;
    return R_t;
}


int main(){

    // transformations
    const rw::math::Transform3D<double> F_0(rw::math::Vector3D<double>(15, 8, 3), rw::math::Rotation3D<double>::identity());
    const rw::math::Transform3D<double> F_1(rw::math::Vector3D<double>(10, 4, 2), rw::math::Rotation3D<double>(0, 1, 0, -1, 0, 0, 0, 0, 1));
    const rw::math::Transform3D<double> F_2(rw::math::Vector3D<double>(6, 0, -2), rw::math::Rotation3D<double>(0, 0, 1, -1, 0, 0, 0, -1, 0));
    // time
    double t_0 = 0, t_1 = 1, t_2 = 4;


    rw::math::Rotation3D<double> i = (F_0.R());
    i = i.inverse();
    i = i * F_1.R();
    rw::math::Vector3D<double> W = W_rot(i);
    std::cout << "i) W_rot(R0T R1): " << W << "\n";

    i = (F_1.R());
    i = i.inverse();
    i = i * F_2.R();
    W = W_rot(i);
    std::cout << "i) W_rot(R1T R2): " <<  W << "\n";

    double time_step = 0.1;
    for(double t = 0; t < t_2 + time_step; t += time_step){
        rw::math::Vector3D<double> segment_p;
        rw::math::Rotation3D<double> segment_r;
        if( t < t_1){
            segment_p = p_linear_int_back(F_0.P(), F_1.P(), t_0, t_1, t);
            segment_r = R_linear_int_back(F_0.R(), F_1.R(), t_0, t_1, t);
        }else{
            segment_p = p_linear_int_back(F_1.P(), F_2.P(), t_1, t_2, t);
            segment_r = R_linear_int_back(F_1.R(), F_2.R(), t_1, t_2, t);
        }
        std::cout << "t: " << t << ", " << segment_p << "\n";
        std::cout << "t: " << t << ", " << segment_r << "\n";
    }



    return 0;
}

