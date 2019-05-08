
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/


#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include "InlineArguments.h"
#include <unistd.h>
#include <fstream>

using namespace std;


void PrintUsage()
{
  printf("Usage: createbatch -name <projectname>\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  //void laserbounds_cpp(char *, char *, char *, int);

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-name")) {
    printf("Error: -name is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  string curr_dir, raw_dir, tiles_dir, road_dir ;
  
  string createblock, laserbounds, createtiles, laser2p3d, createroad, roughclassify;
  
  char *path=NULL;

  size_t size;
  
  path=getcwd(path,size);
  
  curr_dir=path;  //curr_dir.replace("\", )
  
  //createblock -f "D:/MLS/Enschede/strip4/raw/*.laser" -b "D:/MLS/Enschede/strip4/raw/strip4.block" -name strip4

  size_t found;
  while((found=curr_dir.find("\\"))!=string::npos)
  {
  curr_dir.replace(found, 1, "/");
  }
  
  createblock="createblock -f \""; createblock+=curr_dir; createblock+="/raw/*.laser\" -b \"";
  createblock+=curr_dir; 
  createblock+="/raw/"; 
  createblock+=args->String("-name"); 
  createblock+=".block\" -name ";
  createblock+=args->String("-name");
  
  //laserbounds -i "D:/MLS/Enschede/strip4/raw/strip4.block"
  laserbounds="laserbounds -i \"";laserbounds+=curr_dir; laserbounds+="/raw/";
  laserbounds+=args->String("-name"); laserbounds+=".block\"";
  
  //createtiles -i "D:/MLS/Enschede/strip4/raw/strip4.block" -bi -tw 50 -th 50 -tb 0 -root strip4 -odir "D:/MLS/Enschede/strip4/tiles" -meta
   
  //system("rmdir tiles /Q");
  //system("mkdir tiles");
   
  createtiles="createtiles -i \""; createtiles+=curr_dir; createtiles+="/raw/";
  createtiles+=args->String("-name"); createtiles+=".block\" -bi -tw 30 -th 30 -tb 0 -root ";
  createtiles+=args->String("-name"); createtiles+=" -odir \"";
  createtiles+=curr_dir;createtiles+="/tiles\" -meta";
   
  //laser2p3d traj.laser traj.pts
  laser2p3d="laser2p3d traj.laser traj.pts";
  
  //system("rmdir roadparts /Q");
  //system("mkdir roadparts");
  
  //createroad -gps "D:/MLS/Enschede/strip4/traj.pts" -name strip4 -l 40 -w 30 -o 0 -b "D:/MLS/Enschede/strip4/tiles/strip4.block" -odir "D:/MLS/Enschede/strip4/roadparts" -olp -clip
  createroad="createroad -gps \""; createroad+=curr_dir; createroad+="/traj.pts\" -name ";
  createroad+=args->String("-name"); createroad+=" -l 30 -w 30 -o 0 -b \"";
  createroad+=curr_dir; createroad+="/tiles/";
  createroad+=args->String("-name"); createroad+=".block\" -odir \"";
  createroad+=curr_dir; createroad+="/roadparts\" -olp -clip";
  
  roughclassify="rc roadparts/"; roughclassify+=args->String("-name");
  roughclassify+=".road traj.pts";
  
  //cout<<createblock.c_str()<<endl;
  //cout<<laserbounds.c_str()<<endl;
  //cout<<createtiles.c_str()<<endl;
  //cout<<laser2p3d.c_str()<<endl;
  //cout<<createroad.c_str()<<endl;
  //cout<<roughclassify.c_str()<<endl;
  
   ofstream outfile;
   
    outfile.open ("command.bat");
    outfile<<createblock<<endl<<laserbounds<<endl;
    outfile<<"rmdir tiles /Q"<<endl;
    outfile<<"mkdir tiles"<<endl;
    outfile<<createtiles<<endl;
    outfile<<laser2p3d<<endl;
    outfile<<"rmdir roadparts /Q"<<endl;
    outfile<<"mkdir roadparts"<<endl;
    outfile<<createroad<<endl;
    outfile<<"rmdir features /Q"<<endl;
    outfile<<"mkdir features"<<endl;
    //outfile<<roughclassify<<endl;
    
    outfile.close();

  
  
  
  return EXIT_SUCCESS;
}
