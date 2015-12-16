#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <rw/math/Q.hpp>
#include <rw/loaders.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/trajectory/CubicSplineInterpolator.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/kinematics/MovableFrame.hpp>

#include "cubicSpline.hpp"
#include "lua.hpp"


/*

//rw::math::Vector3D<double> W_rot(const rw::math::Rotation3D<double> &R){
//    double theta = acos((R(0,0) + R(1,1) + R(2,2) - 1) / 2);

//    rw::math::Vector3D<double> W(0,0,0);
//    W[0] = (R(2,1) - R(1,2));
//    W[1] = (R(0,2) - R(2,0));
//    W[2] = (R(1,0) - R(0,1));

//    if(theta < 10e-6 || theta > (rw::math::Pi - 10e-6)){
//        W *= 0.5;
//    } else{
//        double length = W.norm2();
//        W *= (theta / length);
//    }
//    return W;
//}

//rw::math::Vector3D<double> P_linear_int_back(const rw::math::Vector3D<double> &P_i_low, const rw::math::Vector3D<double> &P_i, double &t_i_low, double &t_i, double &t){
//    rw::math::Vector3D<double> P_t(P_i);
//    double scale = (t - t_i);
//    scale /= (t_i_low - t_i);
//    rw::math::Vector3D<double> p_vec(P_i_low - P_i);

//    P_t += (scale * p_vec);
//    return P_t;
//}

//rw::math::Rotation3D<double> R_eaa(const rw::math::Vector3D<double> &v, double &theta){
//    rw::math::Rotation3D<double> Reaa;
//    double c = cos(theta), s = sin(theta);

//    for(int i = 0; i < 3; i++){
//        Reaa(i,i) = v[i] * v[i] * (1 - c) + c;
//    }
//    double v12 = v[0] * v[1] * (1 - c);
//    double v13 = v[0] * v[2] * (1 - c);
//    double v23 = v[1] * v[2] * (1 - c);
//    double v3s = v[2] * s;
//    double v2s = v[1] * s;
//    double v1s = v[0] * s;

//    Reaa(1,0) = v12 + v3s;
//    Reaa(2,0) = v13 - v2s;
//    Reaa(2,1) = v23 + v1s;
//    Reaa(0,1) = v12 - v3s;
//    Reaa(0,2) = v13 + v2s;
//    Reaa(1,2) = v23 - v1s;

//    return Reaa;
//}

//rw::math::Rotation3D<double> R_eaa(const rw::math::Vector3D<double> &theta_v){
//    // R_eaa(theta*v) == R_eaa(v, theta) where ||v|| = 1

//    double theta = theta_v.norm2();
//    rw::math::Vector3D<double> v = theta_v / theta;

//    std::cout << " reaa, theta: " << theta << ", v: " << v << ", norm: " << v.norm2() << "\n";

//    return R_eaa(v, theta);
//}

//rw::math::Rotation3D<double> R_linear_int_back(const rw::math::Rotation3D<double> &R_i_low,
//                                               const rw::math::Rotation3D<double> &R_i,
//                                               double &t_i_low, double &t_i, double &t){
//    double scale = (t - t_i);
//    scale /= (t_i_low - t_i);
//    rw::math::Rotation3D<double> R_W_rot = R_i; // note that this is done because inverse manipulates with the data it is called on...
//    R_W_rot = R_W_rot.inverse();
//    R_W_rot = R_i_low * R_W_rot;
//    rw::math::Vector3D<double> tmp = scale * W_rot(R_W_rot);
//    rw::math::Rotation3D<double> Reaa = R_eaa(tmp);

//    rw::math::Rotation3D<double> R_t = Reaa * R_i;
//    return R_t;
//}

//rw::math::Transform3D<double> linear_segmentation(const rw::math::Transform3D<double> &T_i_low,
//                                                  const rw::math::Transform3D<double> &T_i,
//                                                  double &t_i_low, double &t_i, double &t){
//    rw::math::Transform3D<double> T;
//    T.R() = R_linear_int_back(T_i_low.R(), T_i.R(), t_i_low, t_i, t);
//    T.P() = P_linear_int_back(T_i_low.P(), T_i.P(), t_i_low, t_i, t);

//    return T;
//}

//rw::math::Vector3D<double> parabola(double dt, const rw::math::Vector3D<double> &X,
//                                    const rw::math::Vector3D<double> &v1,
//                                    const rw::math::Vector3D<double> &v2,
//                                    double tau){
//    rw::math::Vector3D<double> P;

//    P = (v2 - v1);
//    P /= (4 * tau);
//    P *= (dt + tau) * (dt + tau);
//    P += (v1 * dt);
//    P += X;

//    return P;
//}

//rw::math::Transform3D<double> parabolic_blend(const rw::math::Transform3D<double> &T_i,
//                                              const rw::math::Transform3D<double> &T_i_low,
//                                              const rw::math::Transform3D<double> &T_i_high,
//                                              double t_i, double t_i_low, double t_i_high, double t, double tau){
//    rw::math::Transform3D<double> T;

//    // consider positional part
//    rw::math::Vector3D<double> P_low = (T_i_low.P() - T_i.P()) / (t_i_low - t_i);
//    rw::math::Vector3D<double> P_high = (T_i_high.P() - T_i.P()) / (t_i_high - t_i);

//    T.P() = parabola(t - t_i, T_i.P(), P_low, P_high, tau);

//    // consider rotational part
//    rw::math::Rotation3D<double> R_low = T_i.R();
//    R_low = R_low.inverse();
//    rw::math::Rotation3D<double> R_high = R_low;
//    R_low = T_i_low.R() * R_low;
//    R_high = T_i_high.R() * R_high;

//    rw::math::Vector3D<double> W_low = W_rot(R_low);
//    rw::math::Vector3D<double> W_high = W_rot(R_high);
//    W_low /= (t_i_low - t_i);
//    W_high /= (t_i_high - t_i);

//    rw::math::Vector3D<double> P = parabola(t - t_i, rw::math::Vector3D<double>::zero(), W_low, W_high, tau);
//    rw::math::Rotation3D<double> Reaa = R_eaa(P);

//    T.R() = Reaa * T_i.R();

//    return T;
//}
//*/


bool velocityConstraint(rw::math::Q &dq, rw::models::Device::Ptr &device, double &timestep, double &tau){
    if(device->getDOF() != dq.size()){
        rw::common::Log::log().error() << "ERROR: Dimensions of the input of dq and device dof must agree in velocityConstraint.\n";
        rw::common::Log::log().error() << " - dq: " << dq.size() << ", dof: " << device->getDOF() << "\n";
    }
    if(!(timestep > 0)){
        rw::common::Log::log().error() << "ERROR: Timestep must be greater than 0.\n";
        rw::common::Log::log().error() << " - dt: " << timestep << "\n";
    }


    bool ret = false;
    rw::math::Q vC = device->getVelocityLimits();

//        rw::common::Log::log().info() << " dq:\n" << dq << "\n";
//        rw::common::Log::log().info() << " dq_act:\n" << dq/timestep << "\n";
    //    rw::common::Log::log().info() << " dq_vec:\n" << vC << "\n";

    // find how much to fast it's moving
    rw::math::Q timescale(vC.size());
    double maxscale = 0;
    for(unsigned int i = 0; i < timescale.size(); i++){
        timescale(i) = fabs((dq(i) / timestep) / vC(i));
        if(timescale(i) > maxscale){
            maxscale = timescale(i);
        }
    }

    // apply timescaling to make it go within the bounds
    if(maxscale > 1){
        tau = timestep * maxscale;
        ret = true;
    } else{
        tau = timestep;
    }

    return ret;
}

void plottime(std::string filename, std::vector< double > &time, double dt){
    std::fstream out(filename, std::fstream::out);
    double totalTime = 0;

    if(out.is_open()){
        out << 0 << ", " << 0 << "\n";
        for(int i = 0; i < time.size(); i++){
            totalTime += time[i];
            out << dt * (i + 1) << ", " << totalTime << "\n";
        }
    } else{
        std::cerr << "ERROR: could not open file.\n";
    }
    out.close();
}


int main(){
    // load absolute path
    std::ifstream ifs;
    ifs.open ("/home/.absolutepath.mypath", std::ifstream::in); //a file where the path to the code git repository is defined

    std::string myPath;
    if(ifs.is_open()){
        ifs >> myPath;
        std::cout << "My path is: " << myPath << "\n";
    }else{
        std::cout << "File could not be opened.\n";
    }

    rw::math::RPY<double> angle(0, rw::math::Pi / 4, rw::math::Pi);
//    rw::math::RPY<double> angle(rw::math::Pi / 4, 0, rw::math::Pi);

    rw::math::Transform3D<double> T1(rw::math::Vector3D<double>(-0.975, -0.45, -0.032), angle.toRotation3D());
    rw::math::Transform3D<double> T2(rw::math::Vector3D<double>(-0.975, 0.440, -0.032), angle.toRotation3D());
    rw::math::Transform3D<double> T3(rw::math::Vector3D<double>(-0.974, 0.449, -0.032), angle.toRotation3D());
    rw::math::Transform3D<double> T4(rw::math::Vector3D<double>(-0.965, 0.450, -0.032), angle.toRotation3D());
    rw::math::Transform3D<double> T5(rw::math::Vector3D<double>(-0.475, 0.450, -0.032), angle.toRotation3D());

    double t1 = 0, t2 = 1, t3 = 1.2, t4 = 1.4, t5 = 2.4;

    rw::math::Transform3D<double> Tv1(rw::math::Vector3D<double>(0,0,0), rw::math::Rotation3D<double>::identity());
    rw::math::Transform3D<double> Tv2(rw::math::Vector3D<double>(0,0.5,0), rw::math::Rotation3D<double>::identity());
    rw::math::Transform3D<double> Tv3(rw::math::Vector3D<double>(0.02,0.02,0), rw::math::Rotation3D<double>::identity());
    rw::math::Transform3D<double> Tv4(rw::math::Vector3D<double>(0.5,0,0), rw::math::Rotation3D<double>::identity());
    rw::math::Transform3D<double> Tv5(rw::math::Vector3D<double>(0,0,0), rw::math::Rotation3D<double>::identity());

    // name of the device in the WC
    std::string deviceName = "UR-6-85-5-A";
    std::string boxName = "Pallet";
    std::string toolName = "Tool";
    const std::string wcFile = myPath + "/RO/man03/URInterpolate/Scene.wc.xml";

    rw::math::Q robotInit(6, 0.476, -0.440, 0.62, -0.182, 2.047, -1.574);

    // load wc and det state
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcFile);
    rw::kinematics::State state = wc->getDefaultState();

    // find he robot and set its configuration
    rw::models::Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
        std::cerr << "Device: " << deviceName << " not found!" << std::endl;
        return 0;
    }

    device->setQ(robotInit, state);

//    // find the box and move it according to specs
//    rw::math::Transform3D<double> moveBox(rw::math::Vector3D<double>(0, 0.11, 0), rw::math::Rotation3D<double>::identity());
//    rw::kinematics::MovableFrame* box = (rw::kinematics::MovableFrame*)wc->findFrame(boxName);
//    if (box == NULL) {
//        std::cerr << "Box: " << boxName << " not found!" << std::endl;
//        return 0;
//    }
//    rw::math::Transform3D<double> boxInit = box->getTransform(state);
//    box->setTransform((moveBox * boxInit), state);
//    std::cout << "The initistd::string filename, al Box transform:\n" << boxInit << "\n";
//    std::cout << "The final Box transform:\n" << box->getTransform(state) << "\n";

    // get tool mount and worldTtool
    rw::kinematics::Frame* tool = wc->findFrame(toolName);
    if (tool == NULL) {
        std::cerr << "Toll: " << toolName << " not found!" << std::endl;
        return 0;
    }

    rw::math::Transform3D<double> T1_verif = device->baseTframe(tool, state);
    std::cout << "The transform of robot base to tool:\n" << T1_verif << "\n";
    std::cout << "The actual T1:\n" << T1 << "\n";

    int tessellation = 241;
    double totalTime = t5 - t1;
    double dt = totalTime / (tessellation - 1);

    // tesselate
    int notValidQ = 0, noQfound = 0, velConstraintsApplied = 0;
    std::cout << "Interpolating T." << std::endl;
    std::vector< rw::math::Transform3D<double> > Ttessellated;
    Ttessellated.resize(tessellation);
    std::vector< rw::math::Q > Qtessellated;
    Qtessellated.resize(tessellation);
    std::vector< double > time;
    time.resize(tessellation-1);

    rw::invkin::JacobianIKSolver ik(device, tool, state);
//        ik.setCheckJointLimits(true);
//        ik.setClampToBounds(true);
//    ik.setEnableInterpolation(true);
//    ik.setSolverType(rw::invkin::JacobianIKSolver::SVD);

    for(int i = 0; i < tessellation; i++){
        // find time to find point at
        double t = t1 + i * dt;
        // tessellate the Transformation (only transforms the P() and assumes that R_s() == R_f())
        rw::math::Transform3D<double> Ttes;
        if(t >= t1 && t < t2){
            Ttes = cubic_spline(T1, T2, Tv1, Tv2, t, t1, t2);
        } else if(t >= t2 && t < t3){
            Ttes = cubic_spline(T2, T3, Tv2, Tv3, t, t2, t3);
        } else if(t >= t3 && t < t4){
            Ttes = cubic_spline(T3, T4, Tv3, Tv4, t, t3, t4);
        } else if(t >= t4 && t <= t5){
            Ttes = cubic_spline(T4, T5, Tv4, Tv5, t, t4, t5);
        } else {
            std::cerr << "Error: No valid t used.\n";
        }
        // save transform
        Ttessellated[i] = Ttes;

        // make invese to get Q for robot
        rw::math::Q robotConfig;
        // inverse kinematics setup

        // find the possible configurations to the problem
        std::vector< rw::math::Q > possibleConfigurations;
        // ---- retract ----
        possibleConfigurations = ik.solve(Ttes, state);

        // set the next configuration to be the first
        // TODO: maybe check the whole vector and take the nearest or just run through to take the first valid
        // (does not take into account that the first found configuration could be wrong, but should be done by the IK solver)
//        std::cout << "# of solutions: " << possibleConfigurations.size() << "\n";
        if(possibleConfigurations.size()){
            robotConfig = possibleConfigurations.at(0);

            // set the robot to the found configuration if it is a valid location
            rw::proximity::CollisionDetector detector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

            if(!checkCollisions(device, state, detector, robotConfig)){
                std::cerr << "Error: The Q used is not valid!\n";
                notValidQ++;
            }

        } else{
            std::cerr << "Error: No set of valid Q was found!\n";
            noQfound++;
        }

        Qtessellated[i] = robotConfig;
        device->setQ(robotConfig, state);

        // apply time constraints
        if(i > 0){ // only if between two points in time, consider now and prev
            double tau = dt;
            rw::math::Q dq = robotConfig - Qtessellated[i-1];
            if(velocityConstraint(dq, device, dt, tau)){
                velConstraintsApplied++;
            }
            time[i-1] = tau;
        }

        // output the Q's
        if(t == 0.5 || t == 1.05 || t == 1.32 || t == 1.7){
            std::cout << "The position for t = " << t << ":\n" << Ttes.P() << "\n";
            std::cout << "The configuration for t = " << t << ":\n" << robotConfig << "\n";
            std::cout << "tau = " << i << ":\t" << tau << "\n";
        }
    }
    std::cout << "# Q not found " << noQfound << " and # of Q colliding " << notValidQ << "\n";
    std::cout << "# velocity constraint applied " << velConstraintsApplied << "\n";

    rw::trajectory::QPath path(Qtessellated);
    outputLuaPath(("./lua.txt"), path, deviceName, robotInit);

    plottime("../Rplots/time.csv", time, dt);


    return 0;
}

