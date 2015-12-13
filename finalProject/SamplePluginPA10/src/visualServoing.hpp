#pragma once
#include <iostream>
#include <memory>
#include <cmath>
#include <vector>

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



namespace visualServoing {

// calculate u and v from 3d coords and f
std::vector< cv::Point > uv(std::vector< double > &x, std::vector< double > &y, std::vector< double > &z, double f);
cv::Point uv(double x, double y, double z, double f);

// find the image jacobian
rw::math::Jacobian imageJacobian(std::vector< cv::Point > &uv, double f, std::vector< double > &z);
rw::math::Jacobian imageJacobian(std::vector< double > &x, std::vector< double> &y, double f, std::vector< double > &z);
rw::math::Jacobian imageJacobian(cv::Point &uv, double f, double z);
rw::math::Jacobian imageJacobian(double x, double y, double z, double f);

// make the Z_image
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> z_image(rw::math::Jacobian &imageJacobian, const rw::math::Rotation3D< double > &R_base_tool_ofQ, rw::math::Jacobian &JofQ);


// visual servoing without velocity constraint
rw::math::Q visualServoing(std::vector< cv::Point > &uv, std::vector< double > &z, double f, rw::models::Device::Ptr &device, rw::kinematics::Frame * &endeffector, rw::kinematics::State &state, std::vector< cv::Point > &mapping);
rw::math::Q visualServoing(std::vector< double > &x, std::vector< double > &y, std::vector< double > &z, double f, rw::models::Device::Ptr &device, rw::kinematics::Frame * &endeffector, rw::kinematics::State &state, std::vector< cv::Point > &mapping);
rw::math::Q visualServoing(cv::Point &uv, double z, double f, rw::models::Device::Ptr &device, rw::kinematics::Frame * &endeffector, rw::kinematics::State &state);
rw::math::Q visualServoing(double x, double y, double z, double f, rw::models::Device::Ptr &device, rw::kinematics::Frame * &endeffector, rw::kinematics::State &state);

// apply the velocity constraint, return true if velocity was constrained
bool velocityConstraint(rw::math::Q &dq, rw::models::Device::Ptr &device, double timestep, rw::math::Q &constrained_dq);

// find the du given a set of uv's and mappings
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> du_fixed(std::vector< cv::Point > &uv, std::vector< cv::Point > &mappings);

// approximate the distance to the object
double approxDist(std::vector< cv::Point > &uv, double f, double actualdist);

// best matching, may not visit the same row twice
bool bestMatching(std::vector< std::vector< double > > &costTable, std::vector< int > &bestMatch);


class node
{
public:

    node(std::shared_ptr<node> &parent, double cost, int mapper, int point):
        parent_(parent), cost_(cost), mapper_(mapper), point_(point)
    {

    }

    node():parent_(nullptr),cost_(0){

    }

    node(const node &init){
        *this = init;
    }

    double getCost(){ return cost_; }

    int getMapper(){ return mapper_; }

    int getPoint(){ return point_; }

    std::shared_ptr<node> getParent(){ return parent_; }

    void setCost(double cost){
      cost_ = cost;
    }

    void setMapper(int mapper){
      mapper_ = mapper;
    }

    void setPoint(int point){
      point_ = point;
    }

    void setParent(std::shared_ptr<node> &parent){
      parent_ = parent;
    }



private:

    // cost and
    double cost_;
    int mapper_, point_;
    // the node we came from
    std::shared_ptr<node> parent_;
};


}


