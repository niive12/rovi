#include <iostream>
#include <fstream>

#include <rw/math/Q.hpp>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/math/Math.hpp> //set seed

#include <string>

#include <rw/kinematics/FKTable.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/pathplanning/PathAnalyzer.hpp>

#define MAXTIME 15.

// Kinematics::gripFrame()
// new state
// remember to update planner with new state



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

rw::trajectory::QPath test_planner(rw::pathplanning::QToQPlanner::Ptr planner, rw::math::Q from, rw::math::Q to, rw::pathplanning::PathAnalyzer analysis, rw::kinematics::Frame* tool_frame, std::string output, samples = 30){
    rw::trajectory::QPath best_path;
    rw::trajectory::QPath path;
    rw::common::Timer t;

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
        rw::pathplanning::PathAnalyzer::CartesianAnalysis result = analysis.analyzeCartesian(path, tool_frame);
        std::cout << t.getTimeMs() << ",\t"
                  << result.length << '\n';
        if(result.length < best_length){
            best_length = result.length;
            best_path = path;
        }
        path.clear();
    }
    std::cout.rdbuf(coutbuf); //reset to standard output again
    return best_path;
}


int main()
{
    rw::math::Math::seed();
    // name of the device in the WC
    std::string deviceName = "KukaKr16";
    std::string itemName = "Bottle";
    std::string toolMount = "ToolMount";
    // path to WC, must be with respect to /home, ~/ is not valid
//    const std::string wcFile = "/home/lukas/workcells/Kr16WallWorkCell/Scene.wc.xml";
    const std::string wcFile = "/home/niko/kode/rovi/robotic/Kr16WallWorkCell/Scene.wc.xml";


    // final start and end positions
    rw::math::Q from(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
    rw::math::Q to(6,1.571,0.006,0.030,0.153,0.762,4.490);
//    rw::math::Q from(6,-0.2,-0.6,1.5,0.0,0.6,1.2);
//    rw::math::Q to(6,1.7,0.6,-0.8,0.3,0.7,-0.5); // Very difficult for planner
//    rw::math::Q to(6,1.4,-1.3,1.5,0.3,1.3,1.6);

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

    // ...
    rw::proximity::CollisionDetector detector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,device,state);

    /** More complex way: allows more detailed definition of parameters and methods */
    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(device),constraint.getQConstraintPtr());
    rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean< rw::math::Q >();
    rw::pathplanning::PathAnalyzer analysis(device, state);

    std::cout << "Planning from " << from << " to " << to << std::endl;

    rw::trajectory::QPath path;
    std::string file_name;
    for(double eps = 0.1; eps < 0.9; eps+=0.1){
        rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, eps, rwlibs::pathplanners::RRTPlanner::RRTConnect);
//        file_name = "../statistics/eps" + std::to_string(eps) + ".csv";
        file_name = "eps" + std::to_string(eps) + ".csv";
        std::cout << "Testing: " << file_name << '\n';
        path = test_planner(planner, from, to, analysis, tool_frame, file_name, 30);
//        path = test_planner(planner, from, to, analysis, tool_frame, "debug", 10);
        std::cout << "Test complete\n";
    }

    // change output stuff
    std::ofstream out("out.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!

    outputLuaPath(path, deviceName,from, itemName, toolMount);

    std::cout.rdbuf(coutbuf); //reset to standard output again

    return 0;
}

