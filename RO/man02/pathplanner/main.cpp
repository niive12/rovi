#include <iostream>
#include <fstream>
#include <string>

#include <rw/math/Q.hpp>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/pathplanning/PathAnalyzer.hpp>
#include <rw/math/Math.hpp>

#define MAXTIME 15.

void outputLuaPath(rw::trajectory::QPath &path, std::string &robot, rw::math::Q &start_frame, std::string &item, std::string &tool_frame){
    // output default stuff
    std::cout << "wc = rws.getRobWorkStudio():getWorkCell()\n"
                 "state = wc:getDefaultState()\n"
                 "device = wc:findDevice(\""
              << robot << "\")\n"
                 "item = wc:findFrame(\""
              << item <<
                    "\")\n"
                 "function setQ(q)\n"
                 "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n"
                 "device:setQ(qq,state)\n"
                 "rws.getRobWorkStudio():setState(state)\n"
                 "rw.sleep(0.5)\n"
                 "end\n\n"
                 "setQ({"
                 << start_frame[0];
    // add the start frame
    for(uint i = 1; i < start_frame.size(); i++){
        std::cout << "," << start_frame[i];
    }
    // end start frame and grip item onto tool_frame
    std::cout << "})\n"
                 "tool_frame = wc:findFrame(\""
              << tool_frame <<
                 "\")\n"
                 "rw.gripFrame(item, tool_frame, state)\n\n";



    for (rw::trajectory::QPath::iterator it = path.begin(); it < path.end(); it++) {
        std::cout << "setQ({"
                  << (*it)[0];

        for(uint i = 1; i < (*it).size(); i++){
            std::cout << "," << (*it)[i];
        }
        std::cout << "})\n";
    }
}

bool path_collision(rw::models::Device::Ptr device, rw::kinematics::State &state, rw::proximity::CollisionDetector &detector, rw::trajectory::QPath &path){
    rw::kinematics::State test_state = state;
    rw::proximity::CollisionDetector::QueryResult data;
    bool collision = false;
    for (rw::trajectory::QPath::iterator it = path.begin(); it < path.end(); it++) {
        device->setQ((*it), test_state);
        collision = detector.inCollision(test_state, &data);
        if(collision)
            return true;
    }
    return false;
}

rw::trajectory::QPath test_planner(rw::pathplanning::QToQPlanner::Ptr planner, rw::math::Q from, rw::math::Q to, rw::pathplanning::PathAnalyzer analysis, rw::kinematics::Frame* tool_frame, std::string output, int samples, rw::models::Device::Ptr device, rw::kinematics::State &state, rw::proximity::CollisionDetector &detector){
    rw::trajectory::QPath best_path;
    rw::trajectory::QPath path;
    rw::common::Timer t;
    rw::pathplanning::PathAnalyzer::CartesianAnalysis result;

    //redirect if not debugging
    std::ofstream out(output);
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buffer
    if(output != "debug")
        std::cout.rdbuf(out.rdbuf()); //redirect std::cout to csv file

    std::cout << "time,\tlength\n";
    double best_length = 200;
    for(int i = 0; i < samples; ++i){
        t.resetAndResume();
        planner->query(from,to,path,MAXTIME);
        t.pause();
        result = analysis.analyzeCartesian(path, tool_frame);
        if(path_collision(device, state, detector, path))
            result.length = 0;
        std::cout << t.getTimeMs() << ",\t"
                  << result.length << '\n';
        if(result.length < best_length && result.length > 0){
            best_length = result.length;
            best_path = path;
        }
        path.clear();
    }
    std::cout.rdbuf(coutbuf); //reset to standard output again
    return best_path;
}

int main(){
    rw::math::Math::seed();
    // name of the device in the WC
    std::string deviceName = "KukaKr16";
    std::string itemName = "Bottle";
    std::string toolMount = "ToolMount";
    const std::string wcFile = "/home/lukas/workcells/Kr16WallWorkCell/Scene.wc.xml";
//    const std::string wcFile = "/home/niko/kode/rovi/robotic/Kr16WallWorkCell/Scene.wc.xml";

    rw::math::Q from(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
    rw::math::Q to(6,1.571,0.006,0.030,0.153,0.762,4.490);

    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcFile);
    rw::models::Device::Ptr device = wc->findDevice(deviceName);

    // find the bottle frame
    rw::kinematics::Frame* item = wc->findFrame(itemName);
    rw::kinematics::Frame* tool_frame = wc->findFrame(toolMount);
    if (device == NULL) {
        std::cerr << "Device: " << deviceName << " not found!" << std::endl;
        return 0;
    }
    rw::kinematics::State state = wc->getDefaultState();
    // grip frame
    rw::kinematics::Kinematics::gripFrame(item, tool_frame, state);

    rw::proximity::CollisionDetector detector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,device,state);

    /** More complex way: allows more detailed definition of parameters and methods */
    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(device),constraint.getQConstraintPtr());
    rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean< rw::math::Q >();
    rw::pathplanning::PathAnalyzer analysis(device, state);

    std::cout << "Planning from " << from << " to " << to << std::endl;
    rw::pathplanning::PathAnalyzer::CartesianAnalysis result;
    double best_length = 200;
    rw::trajectory::QPath path, best_path;
    std::string file_name;
    for(double eps = 0.1; eps < 0.9; eps+=0.1){
        rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, eps, rwlibs::pathplanners::RRTPlanner::RRTConnect);
        file_name = "../statistics/eps" + std::to_string(eps) + ".csv";
        std::cout << "Testing: " << file_name << '\n';
        path = test_planner(planner, from, to, analysis, tool_frame, file_name, 30, device, state, detector);
        std::cout << "Test complete\n";
        result = analysis.analyzeCartesian(path, tool_frame);
        if(result.length < best_length && result.length > 0){
            best_length = result.length;
            best_path = path;
        }
    }
    // change output stuff
    std::ofstream out("out.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!

    outputLuaPath(best_path, deviceName,from, itemName, toolMount);

    std::cout.rdbuf(coutbuf); //reset to standard output again

    return 0;
}

