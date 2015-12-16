#include <iostream>
#include <fstream>
#include <string>

#include <rw/math/Q.hpp>
#include <rw/rw.hpp>
#include <rw/math/Math.hpp>


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
