//
// Created by NikoohematS on 19-6-2018.
//

#ifndef INDOOR_GEOMETRY_REOCNSTRUCTION_DIRECTORY_PROCESSING_H
#define INDOOR_GEOMETRY_REOCNSTRUCTION_DIRECTORY_PROCESSING_H

#endif //INDOOR_GEOMETRY_REOCNSTRUCTION_DIRECTORY_PROCESSING_H

#include <string>
#include "boost/filesystem.hpp"

using namespace boost::filesystem;
using std::string;

std::vector<std::pair<std::string, std::string>> RecursiveDirFileNameAndPath (const std::string &dirname);
