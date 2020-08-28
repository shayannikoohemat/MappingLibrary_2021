//
// Created by NikoohematS on 19-6-2018.
//

#include "Directory_Processing.h"
#include <iostream>

/* recursively return the "path" and "filename" as a pair. */
std::vector<std::pair<string, string>> RecursiveDirFileNameAndPath (const string &dirname){

    std::pair<string, string> path_file;
    std::vector<std::pair<string, string>> path_file_list;
    for ( boost::filesystem::recursive_directory_iterator end, dir(dirname);
          dir != end; ++dir ) {
        //std::cout << *dir << "\n";  // full path
        //std::cout << dir->path().filename() << "\n"; // just last bit
        if (!boost::filesystem::is_directory(*dir)){ // skip directory paths
            path_file = std::make_pair(dir ->path().string() , dir->path().filename().string());
            path_file_list.push_back(path_file);
        }
    }
    return path_file_list;
}


void Laserpoints_info(string dirname){

}
