#include <iostream>
#include <fstream>
#include <string>

#include <rw/math/Q.hpp>
#include <rw/rw.hpp>
#include <rw/math/Math.hpp>


// make cubic spline for M datapoints of N dimensions
rw::math::Vector3D<double> C(rw::math::Vector3D<double> &P_s, rw::math::Vector3D<double> &P_f, rw::math::Vector3D<double> &V_s, rw::math::Vector3D<double> &V_f, double t, double t_s, double t_f);

rw::math::Transform3D<double> cubic_spline(rw::math::Transform3D<double> &P_s, rw::math::Transform3D<double> &P_f, rw::math::Transform3D<double> &V_s, rw::math::Transform3D<double> &V_f, double t, double t_s, double t_f);

// this check collision function was taken from Lars's code example, I think to remember...
bool checkCollisions(rw::models::Device::Ptr device, const rw::kinematics::State &state, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q);
