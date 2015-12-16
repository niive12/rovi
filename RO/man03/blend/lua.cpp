#include "lua.hpp"


void outputLuaPath(std::string filename, rw::trajectory::QPath &path, std::string &robot, rw::math::Q &start_frame){
    std::fstream out(filename, std::fstream::out);

    if(out.is_open()){
    // output default stuff
    out << "wc = rws.getRobWorkStudio():getWorkCell()\n"
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
        out << "," << start_frame[i];
    }
    // end start frame
    out << "})\n";

    for (rw::trajectory::QPath::iterator it = path.begin(); it < path.end(); it++) {
        out << "setQ({"
                  << (*it)[0];

        for(uint i = 1; i < (*it).size(); i++){
            out << "," << (*it)[i];
        }
        out << "})\n";
    }
    } else{
        std::cerr << "ERROR: Could not open the file\n";
    }
}
