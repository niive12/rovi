#include "visualServoing.hpp"


rw::math::Q visualServoing::visualServoing(double x, double y, double z, double f,
                                           rw::models::Device::Ptr &device,
                                           rw::kinematics::Frame * &endeffector, rw::kinematics::State &state){
    cv::Point uv = visualServoing::uv(x,y,z,f);

    return visualServoing(uv, z, f, device, endeffector, state);
}


rw::math::Q visualServoing::visualServoing(cv::Point &uv, double z, double f,
                                           rw::models::Device::Ptr &device, rw::kinematics::Frame * &endeffector,
                                           rw::kinematics::State &state){
    std::vector< cv::Point > v_uv = {uv};
    std::vector< double > v_z = {z};
    std::vector< cv::Point > mapping = {cv::Point(0,0)};

    return visualServoing(v_uv, v_z, f, device, endeffector, state, mapping);
}


rw::math::Q visualServoing::visualServoing(std::vector< double > &x, std::vector< double > &y,
                                           std::vector< double > &z, double f, rw::models::Device::Ptr &device,
                                           rw::kinematics::Frame * &endeffector, rw::kinematics::State &state,
                                           std::vector< cv::Point > &mapping){
    if(x.size() != z.size() || z.size() != y.size()){
        rw::common::Log::log().error() << "ERROR: Dimensions of the input of x, y and z must agree in visualServoing.\n";
        rw::common::Log::log().error() << " - x: " << x.size() << ", y: " << y.size() << ", z: " << z.size() << "\n";
    }
    std::vector< cv::Point > v_uv = uv(x,y,z,f);


    return visualServoing(v_uv, z, f, device, endeffector, state, mapping);
}


rw::math::Q visualServoing::visualServoing(std::vector< cv::Point > &uv, std::vector< double > &z, double f,
                                           rw::models::Device::Ptr &device, rw::kinematics::Frame * &endeffector,
                                           rw::kinematics::State &state, std::vector< cv::Point > &mapping){
    if(uv.size() != z.size()){
        rw::common::Log::log().error() << "ERROR: Dimensions of the input of uv and z must agree in visualServoing.\n";
        rw::common::Log::log().error() << " - uv: " << uv.size() << ", z: " << z.size() << "\n";
    }
    if(uv.size() > mapping.size()){
        rw::common::Log::log().error() << "ERROR: Dimensions of the mapping >= uv must agree in visualServoing.\n";
        rw::common::Log::log().error() << " - uv: " << uv.size() << ", mapping: " << mapping.size() << "\n";
    }
    //    rw::common::Log::log().info() << "-------- New calc --------\n";

    //
    for(unsigned int i = 0; i < uv.size(); i++){
        uv[i] = -uv[i];
    }

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

//    if(z_img.rows() > z_img.cols()){
//        dq_eig = z_inv * z_img * y;
//    } else{
        dq_eig = z_t * y;
//    }

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



cv::Point visualServoing::uv(double x, double y, double z, double f){
    cv::Point uv;
    uv.x = f * x / z;
    uv.y = f * y / z;
    return uv;
}

std::vector< cv::Point > visualServoing::uv(std::vector< double > &x, std::vector< double > &y, std::vector< double > &z, double f){
    if(x.size() != y.size() || y.size() != z.size()){
        rw::common::Log::log().error() << "ERROR: Dimensions of the input of x, y and z must agree in uv.\n";
        rw::common::Log::log().error() << " - x: " << x.size() << ", y: " << y.size() << ", z: " << z.size() << "\n";
    }

    std::vector< cv::Point > v_uv;
    for(unsigned int i = 0; i < x.size(); i++){
        double x_t = x[i], y_t = y[i], z_t = z[i];
        v_uv.emplace_back(uv(x_t, y_t, z_t, f));
    }
    return v_uv;
}



rw::math::Jacobian visualServoing::imageJacobian(double x, double y, double z, double f){

    cv::Point uvs = uv(x, y, z, f);

    return imageJacobian(uvs, f, z);
}

rw::math::Jacobian visualServoing::imageJacobian(cv::Point &uv, double f, double z){
    std::vector< cv::Point > v_uv = {uv};
    std::vector< double > v_z = {z};

    return imageJacobian(v_uv, f, v_z);
}

rw::math::Jacobian visualServoing::imageJacobian(std::vector< double > &x, std::vector< double> &y, double f, std::vector<double> &z){
    if(x.size() != y.size() || y.size() != z.size()){
        rw::common::Log::log().error() << "ERROR: Dimensions of the input of x, y and z must agree in imageJacobian.\n";
        rw::common::Log::log().error() << " - x: " << x.size() << ", y: " << y.size() << ", z: " << z.size() << "\n";
    }

    std::vector< cv::Point > v_uv = uv(x, y, z, f);

    return imageJacobian(v_uv, f, z);
}


rw::math::Jacobian visualServoing::imageJacobian(std::vector< cv::Point > &uv, double f, std::vector< double > &z){
    if(uv.size() != z.size()){
        rw::common::Log::log().error() << "ERROR: Dimensions of the input of x, y and z must agree in imageJacobian.\n";
        rw::common::Log::log().error() << " - uv: " << uv.size() << ", z: " << z.size() << "\n";
    }

    rw::math::Jacobian imageJ(uv.size() * 2, 6);
    //    imageJ.re

    for(unsigned int i = 0; i < uv.size(); i++){
        double u = uv[i].x;
        double v = uv[i].y;

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

    // compute the Z_image = J_image * S(q) * J(q)
    ret = imageJacobian.e() * SofQ * JofQ.e();

    return ret;
}


Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> visualServoing::du_fixed(std::vector< cv::Point > &uv, std::vector< cv::Point > &mappings){
    if(uv.size() > mappings.size()){
        rw::common::Log::log().error() << "ERROR: uv and mapping are not the same size nor is uv smaller than mapping in 'du'.\n";
        rw::common::Log::log().error() << " - uv: " << uv.size() << ", mappings: " << mappings.size() << "\n";
    }
    // match to the corresponding circles but n the same shape as supposed.
    // use homography, opencv::findHomography , modules/calib3d - for >= 4

    bool astar = true;
    Eigen::Matrix<double, Eigen::Dynamic, 1> du_eig;
    du_eig.resize(uv.size() * 2, Eigen::NoChange);
    if(uv.size() == 1){
        // if just one point align it with translation only
        if(uv.size() == mappings.size()){
            // track with offset if only one point is tracked
            du_eig(0,0) = -uv[0].x + mappings[0].x;
            du_eig(1,0) = -uv[0].y + mappings[0].y;
        } else {
            // track to center if multi point tracking
            du_eig(0,0) = -uv[0].x;
            du_eig(1,0) = -uv[0].y;
        }

    } else if(uv.size() > 1 && astar){
        // create cost array
        std::vector< std::vector< double > > cost;
        cost.resize(mappings.size());
        for(unsigned int m = 0; m < mappings.size(); m++){
            cost[m].resize(uv.size());
            for(unsigned int p = 0; p < uv.size(); p++){
                // calc cost for this point and map
                double dx = uv[p].x - mappings[m].x, dy = uv[p].y - mappings[m].y;
                cost[m][p] = sqrt(dx * dx + dy * dy);
            }
        }

        // find best mapping
        std::vector< int > map;
        if(bestMatching(cost, map)){
            // use the array to find du
            for(unsigned int i = 0; i < uv.size(); i++){
                du_eig(i * 2, 0) = -uv[i].x + mappings[map[i]].x;
                du_eig(i * 2 + 1, 0) = -uv[i].y + mappings[map[i]].y;
            }

        } else{
            rw::common::Log::log().error() << "ERROR: No matches could be found.\n";
        }

    } else{
        rw::common::Log::log().error() << "ERROR: Not entering the mapping functions.\n";
    }

    return du_eig;
}

// comparator used for the graph > heap, to find the next leaf to consider
struct Comp
{
   bool operator()( std::shared_ptr<visualServoing::node> a, std::shared_ptr<visualServoing::node> b)
   {
       return (a->getCost()) > (b->getCost());
   }
};

#define NULLMAPPER (-1)
bool visualServoing::bestMatching(std::vector< std::vector< double > > &costTable, std::vector< int > &bestMatch){
    // Dijkstra implementation, as of now
    // [row][col], row = mapper, col = point
    // create openlist
    int totalPoints = costTable[0].size(), totalMappings = costTable.size();
    std::vector< std::shared_ptr< node > > leafs;
    std::make_heap(leafs.begin(), leafs.end(), Comp()); //

    bool goalFound = false, moreLeafs = true;
    std::shared_ptr<node> front = nullptr;
    do{
        // check if it is goal
        double costTillNow = 0;
        int mapper = 0;
        if(front != nullptr){
            costTillNow = front->getCost();
            mapper = front->getMapper() + 1;
            if(front->getMapper() == totalMappings - 1){ // reached the end
                // test if allpoints found are used
                std::shared_ptr<node> next = front;
                int pointsUsed = 0;
                while(next != nullptr){
                    if(next->getPoint() != NULLMAPPER){
                        pointsUsed++;
                    }
                    next = next->getParent();
                }
                if(pointsUsed >= totalPoints){ // mappings >= points hence always all points must be used
                    goalFound = true;
                }
            }
        }

        if(!goalFound){
            // put all points in the openlist that are not a parent or older to the new leaf
            for(int point = 0; point < totalPoints; point++){
                // check if it exists in this path
                std::shared_ptr<node> family = front;
                bool exists = false;
                while(family != nullptr && !exists){
                    if(family->getPoint() == point){
                        exists = true;
                    }
                    family = family->getParent();
                }
                // if not, add it
                if(!exists){
                    std::shared_ptr<node> child(new node(front, costTillNow + costTable[mapper][point], mapper, point));
                    leafs.push_back(child);
                }

            }
            // add zero cost step only if mappings are more than points
            if(totalMappings > totalPoints){
                std::shared_ptr<node> child(new node(front, costTillNow, mapper, NULLMAPPER));
                leafs.push_back(child);
            }
            // sort the heap
            std::push_heap(leafs.begin(), leafs.end(), Comp());
            // pop the next one
            if(leafs.size()){
                front = leafs.front();
                std::pop_heap (leafs.begin(),leafs.end(), Comp());
                leafs.pop_back();
            } else{
                moreLeafs = false;
            }
        }
    } while(moreLeafs && !goalFound);

    // return best match
    bestMatch.clear();
    bestMatch.resize(totalPoints);
    if(goalFound){
        std::shared_ptr<node> family = front;
        while(family != nullptr){
            if(family->getPoint() != NULLMAPPER){
                bestMatch[family->getPoint()] = family->getMapper();
            }
            family = family->getParent();
        }
    }

    return goalFound;
}


double visualServoing::approxDist(std::vector< cv::Point > &uv, double f, double actualdist){
    double retdist = 0.5;
    if (uv.size() == 3){
        // find the size of the obj
        double maxd = 0;
        for(unsigned int i = 0; i < uv.size(); i++){
            for(unsigned int j = i+1; j < uv.size(); j++){
                double dx = uv[i].x - uv[j].x, dy = uv[i].y - uv[j].y;
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

