#include "visualServoing.hpp"


rw::math::Vector2D<double> visualServoing::uv(double x, double y, double z, double f){
    rw::math::Vector2D<double> uv( f * x / z, f * y / z);
    return uv;
}


rw::math::Jacobian visualServoing::imageJacobian(double x, double y, double z, double f){

    rw::math::Vector2D<double> uvs = uv(x, y, z, f);

    return imageJacobian(uvs, f, z);
}

rw::math::Jacobian visualServoing::imageJacobian(rw::math::Vector2D<double> &uv, double f, double z){
    rw::math::Jacobian imageJ = rw::math::Jacobian::zero(2, 6);

    double u = uv(0);
    double v = uv(1);

    imageJ(0,0) = - f / z;
    imageJ(1,1) = - f / z;
    imageJ(0,2) = u / z;
    imageJ(1,2) = v / z;
    imageJ(0,3) = u * v / f;
    imageJ(1,3) = ( f * f + v * v ) / f;
    imageJ(0,4) = - ( f * f + u * u ) / f;
    imageJ(1,4) = - ( u * v ) / f;
    imageJ(0,5) = v;
    imageJ(1,5) = -u;

    return imageJ;
}


Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> visualServoing::z_image(rw::math::Jacobian &imageJacobian, const rw::math::Rotation3D<double> &R_base_tool_ofQ, rw::math::Jacobian &JofQ){
    // this could probably be made more efficient by doing the calculations on the same time instead of first converting to Eigen::Matrix

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> ret;

    // compute S(q)
    Eigen::Matrix<double, 6, 6> SofQ;

    rw::math::Rotation3D<double> R_T = R_base_tool_ofQ;
    R_T = R_T.inverse();
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            SofQ(i,j) = R_T(i,j);
            SofQ(i + 3,j + 3) = R_T(i,j);
            SofQ(i + 3,j + 3) = R_T(i,j);
            SofQ(i + 3,j) = 0;
            SofQ(i,j + 3) = 0;
        }
    }

    // make a Jof Q or the type Eigen::Matrix
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> JQ;
    JQ.resize(6,JofQ.size2());

    for(int i = 0; i < JofQ.size1(); i++){
        for(int j = 0; j < JofQ.size2(); j++){
            JQ(i,j) = JofQ(i,j);
        }
    }

    // make the image jacobian of Eigen::Matrix type
    Eigen::Matrix<double, 2, 6> imgJ;
    for(int i = 0; i < imageJacobian.size1(); i++){
        for(int j = 0; j < imageJacobian.size2(); j++){
            imgJ(i,j) = imageJacobian(i,j);
        }
    }

    // compute the Z_image = J_image * S(q) * J(q)
    ret = (imgJ * SofQ);
    ret *= JQ;

    return ret;
}


