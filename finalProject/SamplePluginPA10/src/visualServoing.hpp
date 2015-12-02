#include <iostream>

#include <math.h>

#include <rw/math.hpp> // Pi, Deg2Rad
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/LinearAlgebra.hpp>

#include <vector>


namespace visualServoing {

rw::math::Vector2D<double> uv(double x, double y, double z, double f);

rw::math::Jacobian imageJacobian(double x, double y, double z, double f);

rw::math::Jacobian imageJacobian(rw::math::Vector2D<double> &uv, double f, double z);

//    rw::math::LinearAlgebra::EigenMatrix<double> z_image(rw::math::Jacobian &imageJacobian, const rw::math::Rotation3D<double> &R_base_tool_ofQ, rw::math::Jacobian &JofQ);

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> z_image(rw::math::Jacobian &imageJacobian, const rw::math::Rotation3D<double> &R_base_tool_ofQ, rw::math::Jacobian &JofQ);

}


