#include "visualServoing.hpp"


rw::math::Q visualServoing::visualServoing(double x, double y, double z, double f,
                                           rw::models::Device::Ptr &device,
                                           rw::kinematics::Frame * &endeffector, rw::kinematics::State &state){
    rw::math::Vector2D<double> uv = visualServoing::uv(x,y,z,f);

    return visualServoing(uv, z, f, device, endeffector, state);
}


rw::math::Q visualServoing::visualServoing(rw::math::Vector2D< double > &uv, double z, double f,
                                           rw::models::Device::Ptr &device, rw::kinematics::Frame * &endeffector,
                                           rw::kinematics::State &state){
    std::vector< rw::math::Vector2D<double> > v_uv = {uv};
    std::vector< double > v_z = {z};
    std::vector< cv::Vec2d > mapping = {cv::Vec2d(0,0)};

    return visualServoing(v_uv, v_z, f, device, endeffector, state, mapping);
}


rw::math::Q visualServoing::visualServoing(std::vector< double > &x, std::vector< double > &y,
                                           std::vector< double > &z, double f, rw::models::Device::Ptr &device,
                                           rw::kinematics::Frame * &endeffector, rw::kinematics::State &state,
                                           std::vector< cv::Vec2d > &mapping){
    if(x.size() != z.size() || z.size() != y.size()){
        rw::common::Log::log().error() << "ERROR: Dimensions of the input of x, y and z must agree in visualServoing.\n";
        rw::common::Log::log().error() << " - x: " << x.size() << ", y: " << y.size() << ", z: " << z.size() << "\n";
    }
    std::vector< rw::math::Vector2D<double> > v_uv = uv(x,y,z,f);


    return visualServoing(v_uv, z, f, device, endeffector, state, mapping);
}


rw::math::Q visualServoing::visualServoing(std::vector< rw::math::Vector2D< double > > &uv, std::vector< double > &z, double f,
                                           rw::models::Device::Ptr &device, rw::kinematics::Frame * &endeffector,
                                           rw::kinematics::State &state, std::vector< cv::Vec2d > &mapping){
    if(uv.size() != z.size()){
        rw::common::Log::log().error() << "ERROR: Dimensions of the input of uv and z must agree in visualServoing.\n";
        rw::common::Log::log().error() << " - uv: " << uv.size() << ", z: " << z.size() << "\n";
    }
    //    rw::common::Log::log().info() << "-------- New calc --------\n";

    // J_image, image jacobian
    rw::math::Jacobian j_image = imageJacobian(uv, f, z);

    //    rw::common::Log::log().info() << " - J_image\n" << j_image << "\n";

    // T_base_toll
    rw::math::Transform3D<double> T_base_tool = device->baseTframe(endeffector,state);

    // J(q), manipulator jacobian
    rw::math::Jacobian J_base_tool(6, device->getDOF());
    J_base_tool = device->baseJframe(endeffector, state);

    // Z_image
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> z_img = z_image(j_image, T_base_tool.R(), J_base_tool);

    //    rw::common::Log::log().info() << " - z_img\n" << z_img << "\n";

    // converting du into eigen matrix to do multiplication
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> du_eig = du_fixed(uv, mapping);

    //    rw::common::Log::log().info() << " - du\n" << du_eig << "\n";

    // find z_Transpose
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> z_t;
    z_t.resize(z_img.cols(), z_img.rows());
    for(int i = 0; i < z_t.rows(); i++){
        for(int j = 0; j < z_t.cols(); j++){
            z_t(i,j) = z_img(j,i);
        }
    }

    // inverting Z_image * Z_transpose
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> z_inv = z_img * z_t;
    z_inv = rw::math::LinearAlgebra::pseudoInverse(z_inv);

    // compute y
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> y = z_inv * du_eig;

    // compute dq
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> dq_eig;

    if(z_img.rows() > z_img.cols()){
        dq_eig = z_inv * z_img * y;
    } else{
        dq_eig = z_t * y;
    }

    //    rw::common::Log::log().info() << "y:\n" << y << "\n";

    // convert dq into rw::math::Q
    rw::math::Q dq(device->getDOF());

    for(unsigned int j = 0; j < device->getDOF(); j++){
        dq[j] = dq_eig(j);
    }

    return dq;
}


bool visualServoing::velocityConstraint(rw::math::Q &dq, rw::models::Device::Ptr &device, double timestep, rw::math::Q &constrained_dq){
    if(device->getDOF() != dq.size()){
        rw::common::Log::log().error() << "ERROR: Dimensions of the input of dq and device dof must agree in velocityConstraint.\n";
        rw::common::Log::log().error() << " - dq: " << dq.size() << ", dof: " << device->getDOF() << "\n";
    }


    bool ret = false;
    constrained_dq = dq;
    rw::math::Q vC = device->getVelocityLimits();

    //    rw::common::Log::log().info() << " dq_act:\n" << dq/timestep << "\n";
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
        constrained_dq /= maxscale;
        ret = true;
    }
    return ret;
}



rw::math::Vector2D< double > visualServoing::uv(double x, double y, double z, double f){
    rw::math::Vector2D<double> uv( f * x / z, f * y / z);
    return uv;
}

std::vector< rw::math::Vector2D< double > > visualServoing::uv(std::vector< double > &x, std::vector< double > &y, std::vector< double > &z, double f){
    if(x.size() != y.size() || y.size() != z.size()){
        rw::common::Log::log().error() << "ERROR: Dimensions of the input of x, y and z must agree in uv.\n";
        rw::common::Log::log().error() << " - x: " << x.size() << ", y: " << y.size() << ", z: " << z.size() << "\n";
    }

    std::vector< rw::math::Vector2D< double > > v_uv;
    for(unsigned int i = 0; i < x.size(); i++){
        double x_t = x[i], y_t = y[i], z_t = z[i];
        v_uv.emplace_back(uv(x_t, y_t, z_t, f));
    }
    return v_uv;
}



rw::math::Jacobian visualServoing::imageJacobian(double x, double y, double z, double f){

    rw::math::Vector2D< double > uvs = uv(x, y, z, f);

    return imageJacobian(uvs, f, z);
}

rw::math::Jacobian visualServoing::imageJacobian(rw::math::Vector2D< double > &uv, double f, double z){
    std::vector< rw::math::Vector2D< double > > v_uv = {uv};
    std::vector< double > v_z = {z};

    return imageJacobian(v_uv, f, v_z);
}

rw::math::Jacobian visualServoing::imageJacobian(std::vector< double > &x, std::vector< double> &y, double f, std::vector<double> &z){
    if(x.size() != y.size() || y.size() != z.size()){
        rw::common::Log::log().error() << "ERROR: Dimensions of the input of x, y and z must agree in imageJacobian.\n";
        rw::common::Log::log().error() << " - x: " << x.size() << ", y: " << y.size() << ", z: " << z.size() << "\n";
    }

    std::vector< rw::math::Vector2D< double > > v_uv = uv(x, y, z, f);

    return imageJacobian(v_uv, f, z);
}


rw::math::Jacobian visualServoing::imageJacobian(std::vector< rw::math::Vector2D< double > > &uv, double f, std::vector< double > &z){
    if(uv.size() != z.size()){
        rw::common::Log::log().error() << "ERROR: Dimensions of the input of x, y and z must agree in imageJacobian.\n";
        rw::common::Log::log().error() << " - uv: " << uv.size() << ", z: " << z.size() << "\n";
    }

    rw::math::Jacobian imageJ(uv.size() * 2, 6);
    //    imageJ.re

    for(unsigned int i = 0; i < uv.size(); i++){
        double u = uv[i](0);
        double v = uv[i](1);

        imageJ(i*2,0) = - f / z[i];
        imageJ(i*2,1) = 0;
        imageJ(i*2,2) = u / z[i];
        imageJ(i*2,3) = u * v / f;
        imageJ(i*2,4) = - ( f * f + u * u ) / f;
        imageJ(i*2,5) = v;
        imageJ(i*2 + 1,0) = 0;
        imageJ(i*2 + 1,1) = - f / z[i];
        imageJ(i*2 + 1,2) = v / z[i];
        imageJ(i*2 + 1,3) = ( f * f + v * v ) / f;
        imageJ(i*2 + 1,4) = - ( u * v ) / f;
        imageJ(i*2 + 1,5) = -u;
    }
    return imageJ;
}


Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> visualServoing::z_image(rw::math::Jacobian &imageJacobian, const rw::math::Rotation3D< double > &R_base_tool_ofQ, rw::math::Jacobian &JofQ){
    // this could probably be made more efficient by doing the calculations on the same time instead of first converting to Eigen::Matrix

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> ret;

    // compute S(q)
    Eigen::Matrix<double, 6, 6> SofQ;

    rw::math::Rotation3D< double > R_T = R_base_tool_ofQ;
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

    for(unsigned int i = 0; i < JofQ.size1(); i++){
        for(unsigned int j = 0; j < JofQ.size2(); j++){
            JQ(i,j) = JofQ(i,j);
        }
    }

    // make the image jacobian of Eigen::Matrix type
    Eigen::Matrix<double, Eigen::Dynamic, 6> imgJ;
    imgJ.resize(imageJacobian.size1(), Eigen::NoChange);
    for(unsigned int i = 0; i < imageJacobian.size1(); i++){
        for(unsigned int j = 0; j < imageJacobian.size2(); j++){
            imgJ(i,j) = imageJacobian(i,j);
        }
    }

    // compute the Z_image = J_image * S(q) * J(q)
    ret = imgJ * SofQ * JQ;

    return ret;
}


Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> visualServoing::du_fixed(std::vector< rw::math::Vector2D< double > > &uv, std::vector< cv::Vec2d > &mappings){
    if(uv.size() != mappings.size()){
        rw::common::Log::log().error() << "ERROR: uv and mapping not the same size in 'du'.\n";
        rw::common::Log::log().error() << " - uv: " << uv.size() << ", mappings: " << mappings.size() << "\n";
    }
    // match to the corresponding circles but n the same shape as supposed.
    // use homography, opencv::findHomography , modules/calib3d

    Eigen::Matrix<double, Eigen::Dynamic, 1> du_eig;
    du_eig.resize(uv.size() * 2, Eigen::NoChange);
    if(uv.size() == 1){
        // if just one point align it with translation only
        du_eig(0,0) = -uv[0](0) + mappings[0][0];
        du_eig(1,0) = -uv[0](1) + mappings[0][1];
    } else if (uv.size() == 2){
        // match to closest point (for small displacements it will follow same point, if not it might flip the marker)
        double dxone = uv[0](0) - mappings[0][0], dyone = uv[0](1) - mappings[0][1];
        double dxtwo = uv[1](0) - mappings[1][0], dytwo = uv[1](1) - mappings[1][1];
        double done = sqrt(dxone * dxone + dyone * dyone) + sqrt(dxtwo * dxtwo + dytwo * dytwo);
        dxone = uv[0](0) - mappings[1][0];
        dyone = uv[0](1) - mappings[1][1];
        dxtwo = uv[1](0) - mappings[0][0];
        dytwo = uv[1](1) - mappings[0][1];
        double dtwo = sqrt(dxone * dxone + dyone * dyone) + sqrt(dxtwo * dxtwo + dytwo * dytwo);

        if(done < dtwo){ // 1 -> 1, 2 -> 2 mapping
            du_eig(0,0) = -uv[0](0) + mappings[0][0];
            du_eig(1,0) = -uv[0](1) + mappings[0][1];
            du_eig(2,0) = -uv[1](0) + mappings[1][0];
            du_eig(3,0) = -uv[1](1) + mappings[1][1];
        } else{ // 1 -> 2, 2 -> 1 mapping
            du_eig(0,0) = -uv[0](0) + mappings[1][0];
            du_eig(1,0) = -uv[0](1) + mappings[1][1];
            du_eig(2,0) = -uv[1](0) + mappings[0][0];
            du_eig(3,0) = -uv[1](1) + mappings[0][1];
        }

    } else if (uv.size() == 3){
        // assuming for small steps the triangle will be the one that has its 3 points closest to the mapping
        // find the triangle for which the corners needs to be moved the least distance
        rw::math::Vector3D<unsigned int> map;
        double mindist = -1;
        for(unsigned int i = 0; i < uv.size(); i++){
            for(unsigned int j = 0; j < uv.size(); j++){
                for(unsigned int k = 0; k < uv.size(); k++){
                    if(i != j && i != k && j != k){ // the mappings may not go to the same corner
                        // calculate the distance for the current mapping
                        double dxone = uv[0](0) - mappings[i][0], dyone = uv[0](1) - mappings[i][1];
                        double dxtwo = uv[1](0) - mappings[j][0], dytwo = uv[1](1) - mappings[j][1];
                        double dxthree = uv[2](0) - mappings[k][0], dythree = uv[2](1) - mappings[k][1];
                        double dist = sqrt(dxone * dxone + dyone * dyone) + sqrt(dxtwo * dxtwo + dytwo * dytwo) + sqrt(dxthree * dxthree + dythree * dythree);
                        if(mindist == -1 || dist < mindist){
                            mindist = dist;
                            map = {i,j,k};
                        }
                    }
                }
            }
        }
        // computing du
        du_eig(0,0) = -uv[0](0) + mappings[map(0)][0];
        du_eig(1,0) = -uv[0](1) + mappings[map(0)][1];
        du_eig(2,0) = -uv[1](0) + mappings[map(1)][0];
        du_eig(3,0) = -uv[1](1) + mappings[map(1)][1];
        du_eig(4,0) = -uv[2](0) + mappings[map(2)][0];
        du_eig(5,0) = -uv[2](1) + mappings[map(2)][1];

    } else{
        // for count >= 4
        // use homography for the mapping
        // does NOT need assumption of small changes in the marker
        std::vector< cv::Vec2d > uv_vec2d, dst;
        for(unsigned int i = 0; i < uv.size(); i++){
            uv_vec2d.emplace_back(uv[i](0), uv[i](1));
        }

        cv::Mat H = cv::findHomography(uv_vec2d, mappings,CV_RANSAC);

        cv::perspectiveTransform(uv_vec2d, dst, H);

        for(unsigned int i = 0; i < dst.size(); i++){
            rw::common::Log::log().info() << dst[i] << "\n";
        }
    }
    //        rw::common::Log::log().info() << mappings[i]._x << ", " << mappings[i]._y << "\n";
    //        rw::common::Log::log().info() << -uv[i](0) << ", " << -uv[i](1) << " -> " << du_eig(i * 2, 0)  << ", " << du_eig(2 * i + 1, 0) << "\n";

    return du_eig;
}

double visualServoing::approxDist(std::vector< rw::math::Vector2D< double > > &uv, double f, double actualdist){
    double retdist = 0.5;
    if (uv.size() == 3){
        // find the size of the obj
        double maxd = 0;
        for(unsigned int i = 0; i < uv.size(); i++){
            for(unsigned int j = i+1; j < uv.size(); j++){
                double dx = uv[i](0) - uv[j](0), dy = uv[i](1) - uv[j](1);
                double dist = sqrt(dx * dx + dy * dy);
                //                rw::common::Log::log().info() << "dist: " << dist << "\n";
                if(dist > maxd){
                    maxd = dist;
                }
            }
        }

        // find estimated size = diag^2 = 2 * x^2
        retdist = actualdist * maxd / f;
    }

    return retdist;
}

