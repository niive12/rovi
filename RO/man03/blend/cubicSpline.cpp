#include "cubicSpline.hpp"

// make cubic spline for M datapoints of N dimensions
rw::math::Vector3D<double> C(rw::math::Vector3D<double> &P_s, rw::math::Vector3D<double> &P_f, rw::math::Vector3D<double> &V_s, rw::math::Vector3D<double> &V_f, double t, double t_s, double t_f){
    rw::math::Vector3D<double> dP = P_f - P_s;
    double dt = t_f - t_s;

    rw::math::Vector3D<double> C = (-2 * dP + dt * (V_s + V_f)) * pow((t - t_s) / dt, 3) + (3 * dP - dt * (2 * V_s + V_f)) * pow((t - t_s) / dt, 2) + V_s * (t - t_s) + P_s;

    return C;
}



rw::math::Transform3D<double> cubic_spline(rw::math::Transform3D<double> &P_s, rw::math::Transform3D<double> &P_f, rw::math::Transform3D<double> &V_s, rw::math::Transform3D<double> &V_f, double t, double t_s, double t_f){
    rw::math::Vector3D<double> pos = C(P_s.P(), P_f.P(), V_s.P(), V_f.P(), t, t_s, t_f);
    rw::math::Rotation3D<double> rot = P_s.R(); //C< rw::math::Rotation3D<double> >(P_s.R(), P_f.R(), V_s.R(), V_f.R(), t, t_s, t_f);

    rw::math::Transform3D<double> res(pos, rot);

    return res;
}

// this check collision function was taken from Lars's code example, I think to remember...
bool checkCollisions(rw::models::Device::Ptr device, const rw::kinematics::State &state, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q) {
    rw::kinematics::State testState = state;
    rw::proximity::CollisionDetector::QueryResult data;
    bool ret = true;

    device->setQ(q,testState);
    if (detector.inCollision(testState,&data)) {
        ret = false;
    }
    return ret;
}
