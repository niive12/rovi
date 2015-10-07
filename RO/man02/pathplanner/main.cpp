#include <iostream>

#include <rw/math/Q.hpp>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <string>

using namespace std;


void outputLuaPath(rw::trajectory::QPath &path, string &robot){
    // output default stuff
    cout << "wc = rws.getRobWorkStudio():getWorkCell()\n"
            "state = wc:getDefaultState()\n"
            "device = wc:findDevice(\""
            << robot << "\")\n\n"
            "function setQ(q)\n"
            "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n"
            "device:setQ(qq,state)\n"
            "rws.getRobWorkStudio():setState(state)\n"
            "rw.sleep(1)\n"
            "end\n\n";

    for (rw::trajectory::QPath::iterator it = path.begin(); it < path.end(); it++) {
        cout << "setQ({"
                << (*it)[0];

        for(int i = 1; i < (*it).size(); i++){
            cout << "," << (*it)[i];
        }
        cout << "})\n";
    }
}


int main()
{
    cout << "Hello World!" << endl;

    string robot = "KukaKr16";

    rw::math::Q start_pos(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
    rw::math::Q end_pos(6,1.571,0.006,0.030,0.153,0.762,4.490);

    rw::trajectory::QPath path_test;
    path_test.emplace_back(start_pos);
    path_test.emplace_back(end_pos);

    outputLuaPath(path_test, robot);

    return 0;
}

