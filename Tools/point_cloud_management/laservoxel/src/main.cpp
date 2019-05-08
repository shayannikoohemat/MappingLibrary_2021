#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"
#include "laservoxel.h"

using namespace std;
void PrintUsage()
{
  printf("Usage: \tmandatory:\nlaservoxel -i <laserpoints file> -l <length of voxel edge>\n");
  printf("       \toptional:\n           -s <min number of points in a voxel> -n <min number of points inside 26-neighborhood> -v verbose output\n");
  printf("       \toptional:\n           -t <export textfile for import to marchingcubes>\n");
  
  printf("\n\n if Filling of gaps is desired: instead of -s /-n use -g <kernel size for closing in objectspace>\n");
  printf("optional: use -gf <fillUpLaserpoints file> for data which can be used to fill up potential holes\n");
  printf("optional: use -gp to project all points on the initial plane\n");
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
		  fprintf(stderr, "Error reading file %s\n", args->String("-i"));
		  exit(0);
		}

LaserVoxel *vox;

strcat (output_prefix,args->String("-l"));

///initialize the LaserVoxel with the laserdata, but only if no gapfilling is desired
if (!args->Contains("-g"))
{
 vox= new LaserVoxel(l,args->Double("-l",1)); 
/*
 printf("TEST output of voxel sin a vector<LaserPoints>\n");
 vector<LaserPoints> vox_vec=vox->LaserPoints_per_voxel();
 
 printf("number of voxels with data:%u\n",vox_vec.size());
*/
  ///if desired: filter according min. number of points in a voxel
  if (args->Contains("-s")) {
    vox->filter_size(args->Integer("-s",1),verbose); 

    strcat (output_prefix,"_s");
    strcat (output_prefix,args->String("-s"));
    
    if (verbose)
    {
      printf("statistics after filte size:\n");
      vox->statistics();
    }
    
  }

  ///if desired: filter according min. number of points in the neighborhood
  if (args->Contains("-n")) {
    vox->filter_neighbour(args->Integer("-n",1),verbose); 

    strcat (output_prefix,"_n");
    strcat (output_prefix,args->String("-n"));
    
    if (verbose)
    {
      printf("statistics after filte neighbour:\n");
      vox->statistics();
    }
    
  }
}  //if filtering is desired

else if (args->Contains("-g")) //Filling of gaps is desired
{ 
  strcat (output_prefix,"_g");
  strcat (output_prefix,args->String("-g"));

  vox= new LaserVoxel(args->Double("-l",1)); //only set the voxel length
   
   
   if (args->Contains("-gp")) 
   {
     printf("-gp selected: projecting final points to the respective plane\n");
      strcat (output_prefix,"_gp");
   }
//eventually load the fill-UP information
LaserPoints *FillUpData=NULL;
if (args->Contains("-gf")) 
{
  printf("Reading %s to be used to fill up holes\n", args->String("-gf"));
  strcat (output_prefix,"_gf");
  FillUpData=new LaserPoints;
  if (!FillUpData->Read(args->String("-gf"),1)) {
		  fprintf(stderr, "Error reading file %s\n", args->String("-gf"));
		  exit(0);
		}
}
//fill gaps;
LaserPoints *filledLS=new LaserPoints;

vox->fill_gaps(l,args->Double("-g",1),filledLS,FillUpData,args->Contains("-gp"));
delete FillUpData;

delete vox;
vox= new LaserVoxel(*filledLS,args->Double("-l",1));


}

char fin_all[600];
char fin_all_ply[600];
char fin_vox[600];
char fin_latice_objpts[600];
char fin_latice_top[600];
strcpy(fin_all,output_prefix);
strcpy(fin_all_ply,output_prefix);
strcpy(fin_vox,output_prefix);
strcpy(fin_latice_objpts,output_prefix);
strcpy(fin_latice_top,output_prefix);

strcat(fin_all,".laser");
strcat(fin_all_ply,".ply");
strcat(fin_vox,"_vox_centres.laser");
strcat(fin_latice_objpts,"_vox_latice.objpts");
strcat(fin_latice_top,"_vox_latice.top");

printf("Writing output file with all filtered points to %s\n",fin_all);
LaserPoints exp=vox->export_all();
exp.Write(fin_all,0);
printf("\n");
//writing PLY file for meshlab
vox->export_all_ply(fin_all_ply,exp);

printf("Writing output file only with voxel centres to %s\n",fin_vox);
exp=vox->export_vox_centres();
exp.Write(fin_vox,0);

printf("Writing output files for the voxel latice to %s and %s\n",fin_latice_objpts,fin_latice_top);
printf("COMMENTED IN CODE!\n");
/*
ObjectPoints objpts;
LineTopologies linetops;
vox->export_vox_latice(objpts,linetops);
objpts.Write(fin_latice_objpts);
linetops.Write(fin_latice_top);
*/
if (args->Contains("-t")) {
  printf("\nWriting output file only with voxel centres to textfile for marchingcubes %s\n",args->String("-t"));
vox->export_vox_centres_text(args->String("-t"));   
}




  
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

