#pragma once
#include <iostream>


#include <rw/math.hpp> // Pi, Deg2Rad
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/models.hpp>

#include <opencv2/calib3d/calib3d.hpp>

#include <vector>

#include <cmath>

namespace visualServoing {

// calculate u and v from 3d coords and f
rw::math::Vector2D< double > uv(double x, double y, double z, double f);
std::vector< rw::math::Vector2D< double > > uv(std::vector< double > &x, std::vector< double > &y, std::vector< double > &z, double f);

// find the image jacobian
rw::math::Jacobian imageJacobian(double x, double y, double z, double f);
rw::math::Jacobian imageJacobian(rw::math::Vector2D<double> &uv, double f, double z);
rw::math::Jacobian imageJacobian(std::vector< double > &x, std::vector< double> &y, double f, std::vector< double > &z);
rw::math::Jacobian imageJacobian(std::vector< rw::math::Vector2D< double > > &uv, double f, std::vector< double > &z);

// make the Z_image
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> z_image(rw::math::Jacobian &imageJacobian, const rw::math::Rotation3D< double > &R_base_tool_ofQ, rw::math::Jacobian &JofQ);


// visual servoing without velocity constraint
rw::math::Q visualServoing(double x, double y, double z, double f, rw::models::Device::Ptr &device, rw::kinematics::Frame * &endeffector, rw::kinematics::State &state);
rw::math::Q visualServoing(rw::math::Vector2D< double > &uv, double z, double f, rw::models::Device::Ptr &device, rw::kinematics::Frame * &endeffector, rw::kinematics::State &state);
rw::math::Q visualServoing(std::vector< double > &x, std::vector< double > &y, std::vector< double > &z, double f, rw::models::Device::Ptr &device, rw::kinematics::Frame * &endeffector, rw::kinematics::State &state, std::vector< cv::Point > &mapping);
rw::math::Q visualServoing(std::vector< rw::math::Vector2D< double > > &uv, std::vector< double > &z, double f, rw::models::Device::Ptr &device, rw::kinematics::Frame * &endeffector, rw::kinematics::State &state, std::vector< cv::Point > &mapping);

// apply the velocity constraint, return true if velocity was constrained
bool velocityConstraint(rw::math::Q &dq, rw::models::Device::Ptr &device, double timestep, rw::math::Q &constrained_dq);

// find the du given a set of uv's and mappings
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> du_fixed(std::vector< rw::math::Vector2D< double > > &uv, std::vector< cv::Point > &mappings);

// approximate the distance to the object
double approxDist(std::vector< rw::math::Vector2D< double > > &uv, double f, double actualdist);

}


