
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
#include "InlineArguments.h"
#include "laservoxel.h"

using namespace std;
void PrintUsage()
{
  printf("Usage: \tmandatory:\nlaservoxel -i <laserpoints file> -l <length of voxel edge>\n");
  printf("       \toptional:\n           -s <min number of points in a voxel> -n <min number of points inside 26-neighborhood> -v verbose output\n");
}

                        
int main(int argc, char *argv[])
{
  char output_prefix[500];
  strcpy (output_prefix,"laser_voxel_l");
  
  InlineArguments *args = new InlineArguments(argc, argv);
  
  bool verbose=args->Contains("-v");
  
  if (args->Contains("-usage") || args->Contains("-h")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i") || !args->Contains("-l")) {
    printf("Error: -i and -l are required arguments.\n");
    PrintUsage();
    exit(0);
  }
  
  
  LaserPoints l;
l.ReInitialise();
	  if (!l.Read(args->String("-i"),1)) {
		  fprintf(stderr, "Error reading file %s\n", l.Read(args->String("-i")));
		  exit(0);
		}

///initialize the LaserVoxel
LaserVoxel vox(l,args->Double("-l",1)); 

strcat (output_prefix,args->String("-l"));



///if desired: filter according min. number of points in a voxel
if (args->Contains("-s")) {
  vox.filter_size(args->Integer("-s",1),verbose); 

  strcat (output_prefix,"_s");
  strcat (output_prefix,args->String("-s"));
  
  if (verbose)
  {
    printf("statistics after filte size:\n");
    vox.statistics();
  }
  
}

///if desired: filter according min. number of points in the neighborhood
if (args->Contains("-n")) {
  vox.filter_neighbour(args->Integer("-n",1),verbose); 

  strcat (output_prefix,"_n");
  strcat (output_prefix,args->String("-n"));
  
  if (verbose)
  {
    printf("statistics after filte neighbour:\n");
    vox.statistics();
  }
  
}

char fin_all[600];
char fin_vox[600];
strcpy(fin_all,output_prefix);
strcpy(fin_vox,output_prefix);

strcat(fin_all,".laser");
strcat(fin_vox,"_vox_centres.laser");

printf("Writing output file with all filtered points to %s\n",fin_all);
LaserPoints exp=vox.export_all();
exp.Write(fin_all,0);
printf("\n");
printf("Writing output file only with voxel centres to %s\n",fin_vox);
exp=vox.export_vox_centres();
exp.Write(fin_vox,0);




  
/*
const char * laser_file="/home/gerke/rfv/microdrone/seq4/constraints_sift_dist_io/triang_sift_3rays_all_minbl1/XYZ_RGB.laser";
//const char * laser_file="/home/gerke/rfv/Enschede_SouthEast/adjust/dense_sequence_3raysall/XYZ_RGB_part.laser";
//const char * laser_file="/home/gerke/rfv/seefeldchurch/adjust_siftconstraints_autosequence/dense_combined/dense_combine_all.laser";

LaserPoints l;
l.ReInitialise();
	  if (!l.Read(laser_file,0)) {
		  fprintf(stderr, "Error reading file %s\n", laser_file);
		  exit(0);
		}

      
LaserVoxel vox(l,0.5,1);  //0.1 seefeld

printf("statistics after read in:\n");
vox.statistics();

vox.filter_size(2); //10 seefeld
printf("statistics after filte size:\n");
vox.statistics();


vox.filter_neighbour(2); //5 seefeld
printf("statistics after filter neighb.:\n");
vox.statistics();


LaserPoints exp=vox.export_all();
exp.Write("/tmp/test.laser",0);

exp=vox.export_vox_centres();
exp.Write("/tmp/test_voxelcentres.laser",0);
*/





printf("\nbye\n");


  return EXIT_SUCCESS;
}

