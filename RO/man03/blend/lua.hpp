#include <iostream>
#include <fstream>
#include <string>

#include <rw/math/Q.hpp>
#include <rw/rw.hpp>
#include <rw/math/Math.hpp>


void outputLuaPath(std::string filename, rw::trajectory::QPath &path, std::string &robot, rw::math::Q &start_frame);
