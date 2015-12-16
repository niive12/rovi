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

rw::math::Vector3D<double> P_linear_int_back(const rw::math::Vector3D<double> &P_i_low, const rw::math::Vector3D<double> &P_i, double &t_i_low, double &t_i, double &t){
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

    std::cout << " reaa, theta: " << theta << ", v: " << v << ", norm: " << v.norm2() << "\n";

    return R_eaa(v, theta);
}

rw::math::Rotation3D<double> R_linear_int_back(const rw::math::Rotation3D<double> &R_i_low,
                                               const rw::math::Rotation3D<double> &R_i,
                                               double &t_i_low, double &t_i, double &t){
    double scale = (t - t_i);
    scale /= (t_i_low - t_i);
    rw::math::Rotation3D<double> R_W_rot = R_i; // note that this is done because inverse manipulates with the data it is called on...
    R_W_rot = R_W_rot.inverse();
    R_W_rot = R_i_low * R_W_rot;
    rw::math::Vector3D<double> tmp = scale * W_rot(R_W_rot);
    rw::math::Rotation3D<double> Reaa = R_eaa(tmp);

    rw::math::Rotation3D<double> R_t = Reaa * R_i;
    return R_t;
}

rw::math::Transform3D<double> linear_segmentation(const rw::math::Transform3D<double> &T_i_low,
                                                  const rw::math::Transform3D<double> &T_i,
                                                  double &t_i_low, double &t_i, double &t){
    rw::math::Transform3D<double> T;
    T.R() = R_linear_int_back(T_i_low.R(), T_i.R(), t_i_low, t_i, t);
    T.P() = P_linear_int_back(T_i_low.P(), T_i.P(), t_i_low, t_i, t);

    return T;
}

rw::math::Vector3D<double> parabola(double dt, const rw::math::Vector3D<double> &X,
                                    const rw::math::Vector3D<double> &v1,
                                    const rw::math::Vector3D<double> &v2,
                                    double tau){
    rw::math::Vector3D<double> P;

    P = (v2 - v1);
    P /= (4 * tau);
    P *= (dt + tau) * (dt + tau);
    P += (v1 * dt);
    P += X;

    return P;
}

rw::math::Transform3D<double> parabolic_blend(const rw::math::Transform3D<double> &T_i,
                                              const rw::math::Transform3D<double> &T_i_low,
                                              const rw::math::Transform3D<double> &T_i_high,
                                              double t_i, double t_i_low, double t_i_high, double t, double tau){
    rw::math::Transform3D<double> T;

    // consider positional part
    rw::math::Vector3D<double> P_low = (T_i_low.P() - T_i.P()) / (t_i_low - t_i);
    rw::math::Vector3D<double> P_high = (T_i_high.P() - T_i.P()) / (t_i_high - t_i);

    T.P() = parabola(t - t_i, T_i.P(), P_low, P_high, tau);

    // consider rotational part
    rw::math::Rotation3D<double> R_low = T_i.R();
    R_low = R_low.inverse();
    rw::math::Rotation3D<double> R_high = R_low;
    R_low = T_i_low.R() * R_low;
    R_high = T_i_high.R() * R_high;

    rw::math::Vector3D<double> W_low = W_rot(R_low);
    rw::math::Vector3D<double> W_high = W_rot(R_high);
    W_low /= (t_i_low - t_i);
    W_high /= (t_i_high - t_i);

    rw::math::Vector3D<double> P = parabola(t - t_i, rw::math::Vector3D<double>::zero(), W_low, W_high, tau);
    rw::math::Rotation3D<double> Reaa = R_eaa(P);

    T.R() = Reaa * T_i.R();

    return T;
}

// make cubic spline for M datapoints of N dimensions
template<typename T>
T cubic_spline(T &P_s, T &P_f, T &V_s, T &V_f, double t, double t_s, double t_f){
    T dP = P_f - P_s;
    double dt = t_f - t_s;

    T C = -2 * dP + dt * (V_s + V_f) * pow((t - t_s) / dt, 3) + 3 * dP - dt * (2 * V_s + V_f) * pow((t - t_s) / dt, 2) + V_s * (t - t_s) + P_s;

    return C;
}

int main(){
    // load absolute path
    std::ifstream ifs;
    ifs.open ("/home/.absolutepath.mypath", std::ifstream::in);

    std::string myPath;
    if(ifs.is_open()){
        ifs >> myPath;
        std::cout << "My path is: " << myPath << "\n";
    }else{
        std::cout << "File could not be opened.\n";
    }

    // name of the device in the WC
    std::string deviceName = "KukaKr16";
    std::string boxName = "Bottle";
    std::string toolMount = "ToolMount";
    const std::string wcFile = myPath + "/RO/URInterpolate/Scene.wc.xml";

//    rw::math::Q from(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
//    rw::math::Q to(6,1.571,0.006,0.030,0.153,0.762,4.490);

    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcFile);
    rw::models::Device::Ptr device = wc->findDevice(deviceName);

    // find the bottle frame
//    rw::kinematics::Frame* item = wc->findFrame(boxName);
//    rw::kinematics::Frame* tool_frame = wc->findFrame(toolMount);
//    if (device == NULL) {
//        std::cerr << "Device: " << deviceName << " not found!" << std::endl;
//        return 0;
//    }
    rw::kinematics::State state = wc->getDefaultState();


//    // transformations
//    const rw::math::Transform3D<double> F_0(rw::math::Vector3D<double>(15, 8, 3), rw::math::Rotation3D<double>::identity());
//    const rw::math::Transform3D<double> F_1(rw::math::Vector3D<double>(10, 4, 2), rw::math::Rotation3D<double>(0, 1, 0, -1, 0, 0, 0, 0, 1));
//    const rw::math::Transform3D<double> F_2(rw::math::Vector3D<double>(6, 0, -2), rw::math::Rotation3D<double>(0, 0, 1, -1, 0, 0, 0, -1, 0));
//    // time
//    double t_0 = 0, t_1 = 1, t_2 = 4, tau = 0.1;


//    rw::math::Rotation3D<double> i = (F_0.R());
//    i = i.inverse();
//    i = i * F_1.R();
//    rw::math::Vector3D<double> W = W_rot(i);
//    std::cout << "i) W_rot(R0T R1): " << W << "\n";

//    i = (F_1.R());
//    i = i.inverse();
//    i = i * F_2.R();
//    W = W_rot(i);
//    std::cout << "i) W_rot(R1T R2): " <<  W << "\n";

//    double time_step = 0.1;
//    for(double t = 0; t < t_2 + time_step; t += time_step){
//        rw::math::Transform3D<double> segment;
//        if( t < t_1){
//            segment = linear_segmentation(F_0, F_1, t_0, t_1, t);
//        }else{
//            segment = linear_segmentation(F_1, F_2, t_1, t_2, t);
//        }
//        rw::math::Transform3D<double> parabolic_segment;
//        parabolic_segment = parabolic_blend(F_1, F_0, F_2, t_1, t_0, t_2, t, tau);

//        std::cout << "t_l: " << t << ", " << segment << "\n";
//        std::cout << "t_p: " << t << ", " << parabolic_segment << "\n";
//    }

//    rw::math::VectorND<2, double> p1, p2, v1, v2;
//    p1[0] = p1[1] = v1[0] = v2[1] = 0;
//    v1[1] = p2[0] = v2[0] = 1;
//    p2[1] = 2;
//    double t1 = 0, t2 = 1;

//    rw::math::VectorND<2, double> cubic_segmentation = cubic_spline< rw::math::VectorND<2, double> >(p1, p2, v1, v2, 0.5, t1, t2);

//    std::cout << cubic_segmentation << "\n";

    return 0;
}

