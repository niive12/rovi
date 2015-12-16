#include "lua.hpp"


void outputLuaPath(rw::trajectory::QPath &path, std::string &robot, rw::math::Q &start_frame){
    // output default stuff
    std::cout << "wc = rws.getRobWorkStudio():getWorkCell()\n"
                 "state = wc:getDefaultState()\n"
                 "device = wc:findDevice(\""
              << robot << "\")\n"
                 "function setQ(q)\n"
                 "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n"
                 "device:setQ(qq,state)\n"
                 "rws.getRobWorkStudio():setState(state)\n"
                 "rw.sleep(0.02)\n"
                 "end\n\n"
                 "setQ({"
                 << start_frame[0];
    // add the start frame
    for(uint i = 1; i < start_frame.size(); i++){
        std::cout << "," << start_frame[i];
    }
    // end start frame
    std::cout << "})\n";

    for (rw::trajectory::QPath::iterator it = path.begin(); it < path.end(); it++) {
        std::cout << "setQ({"
                  << (*it)[0];

        for(uint i = 1; i < (*it).size(); i++){
            std::cout << "," << (*it)[i];
        }
        std::cout << "})\n";
    }
}
