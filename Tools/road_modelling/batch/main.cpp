
/*
                      Copyright 2010 University of Twente
 
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
#include <dirent.h>
#include <vector>
#include <fstream>

using namespace std;

void PrintUsage()
{
  printf("Usage: batch [commands for single files under current directories.]\n"); 
}


int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  
   // Check on required input files
  

  
  if(argc<2||args->Contains("-usage"))
  {
  PrintUsage();
  exit(1);
  }
  
  
  
  DIR *dp;
  struct dirent *ep;
  string filename;
  std::vector<string> filenames;
  
  string parameter;
  char parameter_c[256];
  ofstream batchfile;
  /*
  if(args->Contains("-parameterfile") )
  {
  myfile.open (args->String("-parameterfile"));
  if(myfile.is_open())
  {
  myfile.getline (parameter_c,256);
  myfile.close();
  }
  else
  cout<<"Cannot open file."<<endl;
                                   
  }
  */
  //parameter=parameter_c;
  //cout<<"parameter file contain "<<parameter<<endl;
  
  
     dp = opendir ("./");
       if (dp != NULL)
         {
           while (ep = readdir (dp))
             {
                 filename.clear();
                 
                filename+=ep->d_name; 
                 
                 if(string::npos!=filename.find('.')||string::npos!=filename.find("d_name"))
                 {
                 continue;}
                 
                // puts (filename.c_str());
                 
                 filenames.push_back(filename);
             
               
             }
           (void) closedir (dp);
         }
       else
         perror ("Couldn't open the directory");
   cout<<"There are "<<filenames.size()<<" folders."<<endl;
   
   string command;
   
   command=argv[1];
   
   command+=".bat";
   
   cout<<"output file: "<<command<<endl;
   
   batchfile.open(command.data());
   
   if(!batchfile.is_open())
   cout<<"Cannot open "<<command<<endl;
   
   for(int i=0;i<filenames.size();i++)
   {
   
    
    batchfile<<"cd "<< filenames[i]<<endl;
    
    for(int j=1;j<argc;j++)
    batchfile<<argv[j]<<" ";
    
    batchfile<<endl;
    
    batchfile<<"cd.."<<endl;
    /*
    command=args->String("-command"); command+=" -i "; 
    command+=filenames[i]; command+="/";
    command+=args->String("-i"); 
    if(args->Contains("-parameterfile") )
    {command+=" "; command+=parameter_c;}
    
    cout<<command<<endl;
    system(command.data());
  */
  }            
      
    batchfile.close();
    
    return EXIT_SUCCESS;
}
