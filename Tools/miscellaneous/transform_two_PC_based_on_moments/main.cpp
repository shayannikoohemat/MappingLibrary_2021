#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <math.h>
#include "LaserPoints.h"
#include "RotationParameter.h"
#include "InlineArguments.h"
#include "ImageLines.h"
#include "laservoxel.h"
#include <map>

#include "DataBounds3D.h"
#include "KNNFinder.h"

using namespace std;

//background: INACHUS, matching of WP3-collapse model with WP4-sensing-based model. Since the collapse model is in a local coord.system, and the latter one in a global, it is necessary to find the transformation paramters.
//in this programm we do a "rough" transformation based on geometric moments. See papers from Gerke et al 2001.
//this program takes

//-a point cloud containing one building in a local system
//-a point cloud with the same building in a global system

// WE assume, however, that the point clouds are just roated around the z-axis.

//steps
//-move both clouds to make the center or gravity (COG) 0,0,0. Store these transformation parameters.
//-PROBLEM with that step: the centre of gravity (in x,y) is not exactlz in the middle compared to the extents because of the 3D-stacking of points (indirect weighting though). Hence, first an approximate centre is computed and later corrected: the PC is rasterized and all further moments are computed in that raster

//-for both centered PCs compute the geometric moments (see papers) - possibly waited by the z-value?!
//-compute the bounding box for each PC: Lx, Ly and theta
//-rotate the global (centered) cloud by the difference of theta, so that both have the same theta. 
//-open question: what is if the Lx and Ly of both PCs are largely different? The assumption should be that the motments are quite insensitive to small differences?! Also not clear if rotations about nx90 can be detected
    
//-output will be the final rotation matrix and transformation vector from global to local and/or vice versa
using namespace std;


void compute_moments(LaserPoints &lp, vector< vector < double > > & Moments, int imax=2, int jmax=2, bool weight_with_z=0)
{
  //the index starts at 0, so we need to reserve a bit more memory
  if (imax !=2 || jmax !=2) {printf("ERROR: imax and jmax not 2 not implemented yet\n"); exit(0);}
  Moments.resize(imax+1);
  for (int i=0; i<=imax;i++) 
    {Moments[i].resize(jmax+1);
    for (int j=0; j<=jmax;j++) Moments[i][j]=0;
    }
  
  Moments[0][0]=lp.size();
     
  for (int k=0;k<lp.size();k++)
  {
    double f=1; if (weight_with_z) f=lp[k].GetZ();
    
    Moments[1][0]+=lp[k].GetX()*f;
    Moments[0][1]+=lp[k].GetY()*f;
    Moments[2][0]+=lp[k].GetX()*f*lp[k].GetX()*f;
    Moments[1][1]+=lp[k].GetX()*f*lp[k].GetY()*f;
    Moments[0][2]+=lp[k].GetY()*f*lp[k].GetY()*f;
     
  } 
  
}

double compute_theta(vector< vector < double > > & Moments)
{
  return .5*atan2(Moments[1][1]*2,Moments[2][0]-Moments[0][2]); 
}

void compute_Lx_Ly(double M00, double M11, double M20, double M02, double theta, double &Lx, double &Ly)
{
  //first compute rotation corrected M20,M02 (M00 is the same)
  double cs=cos(theta);
  double si=sin(theta);
  
  
  double M20_=cs*cs*M20+2*cs*si*M11+si*si*M02;
  double M02_=si*si*M20-2*cs*si*M11+cs*cs*M02;
  
  Lx=sqrt(12*M20_/M00);
  Ly=sqrt(12*M02_/M00);
}


void PrintUsage()
{
printf("transform_two_PC_based_on_moments: ");
  printf("\nUsage: \ttransform_two_PC_based_on_moments -p_local <laserfile> -p_global <laserfile> \n");
} 

int main(int argc, char *argv[])
{

LaserPoints localLS, globalLS;
LaserPoints localLS_centered, globalLS_centered;

//final transformed
LaserPoints globalLS_to_localLS, localLS_to_globalLS;
LaserPoints globalLS_to_localLS_scaled, localLS_to_globalLS_scaled;

vector< vector < double > > Moments_local, Moments_global;
vector< vector < double > > Moments_local_rotation_invariant, Moments_global_rotation_invariant;
double theta_local, theta_global;

printf("transform_two_PC_based_on_moments\n");



 InlineArguments *args = new InlineArguments(argc, argv);
 
   
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-p_local") || !args->Contains("-p_global"))
 {
    printf("Error: not all arguments given\n");
    PrintUsage();
    exit(0);
  }
 
//read the point clouds
printf("\n Reading local point cloud ...\n");
localLS.ReInitialise();
	  if (!localLS.Read(args->String("-p_local"),1)) {
		  fprintf(stderr, "Error reading file %s\n", args->String("-p_local"));
		  exit(0);
		}
		
printf("\n Reading global point cloud ...\n");
globalLS.ReInitialise();
	  if (!globalLS.Read(args->String("-p_global"),1)) {
		  fprintf(stderr, "Error reading file %s\n", args->String("-p_global"));
		  exit(0);
		}
		
//compute the COGs using moments and move the PCs
double COG_local_X=0, COG_local_Y=0, COG_local_Z=0;
for (int i=0;i<localLS.size();i++)
{
  COG_local_X+=localLS[i].GetX()/localLS.size();
  COG_local_Y+=localLS[i].GetY()/localLS.size();
  COG_local_Z+=localLS[i].GetZ()/localLS.size();
}

printf("COG local= %.2f %.2f %.2f\n", COG_local_X, COG_local_Y, COG_local_Z);

double COG_global_X=0, COG_global_Y=0, COG_global_Z=0;
for (int i=0;i<globalLS.size();i++)
{
  COG_global_X+=globalLS[i].GetX()/globalLS.size();
  COG_global_Y+=globalLS[i].GetY()/globalLS.size();
  COG_global_Z+=globalLS[i].GetZ()/globalLS.size();
}
printf("COG global= %.2f %.2f %.2f\n", COG_global_X, COG_global_Y, COG_global_Z);

localLS_centered=localLS;
globalLS_centered=globalLS;

//set all Z to the same height in order to avoid indirect weithing in the Momentscomputation. Also put the LS then in a laservoxel and just take occupied voxelcentres
for (int i=0;i<localLS.size();i++) 
 {
  localLS_centered[i].SetX(localLS[i].GetX()-COG_local_X);
  localLS_centered[i].SetY(localLS[i].GetY()-COG_local_Y);
 //localLS_centered[i].SetZ(localLS[i].GetZ()-COG_local_Z);
  localLS_centered[i].SetZ(0);
} 

for (int i=0;i<globalLS.size();i++) 
 {
  globalLS_centered[i].SetX(globalLS[i].GetX()-COG_global_X);
  globalLS_centered[i].SetY(globalLS[i].GetY()-COG_global_Y);
  //globalLS_centered[i].SetZ(globalLS[i].GetZ()-COG_global_Z);
  globalLS_centered[i].SetZ(0);
} 

/*Compute the rasterized representation using LserVoxel*/
LaserVoxel localLS_vox(localLS_centered,.5,1);
localLS_centered=localLS_vox.export_vox_centres(false);

LaserVoxel globalLS_vox(globalLS_centered,.5,1);
globalLS_centered=globalLS_vox.export_vox_centres(false);


localLS_centered.Write("localLS_centered.laser",0);
globalLS_centered.Write("globalLS_centered.laser",0);

//localLS:
//compute Moments 
compute_moments(localLS_centered, Moments_local, 2,2,0);
compute_moments(globalLS_centered, Moments_global, 2,2,0);

//the centore of gravitz must be at 0,0 - correct it! See comments above why this is not exactly at 0,0
double cog_local_x_test=Moments_local[1][0]/Moments_local[0][0];
double cog_local_y_test=Moments_local[0][1]/Moments_local[0][0];

double cog_global_x_test=Moments_global[1][0]/Moments_global[0][0];
double cog_global_y_test=Moments_global[0][1]/Moments_global[0][0];

printf("Shifts of COGs: local_x: %.2f, local_y: %.2f, global_x: %.2f global_y: %.2f\n",cog_local_x_test, cog_local_y_test, cog_global_x_test, cog_global_y_test);

//correct for the shifts and update the COG_xxx_X and _Y
for (int i=0;i<localLS.size();i++) 
 {
  localLS_centered[i].SetX(localLS_centered[i].GetX()-cog_local_x_test);
  localLS_centered[i].SetY(localLS_centered[i].GetY()-cog_local_y_test);
} 

for (int i=0;i<globalLS.size();i++) 
 {
  globalLS_centered[i].SetX(globalLS_centered[i].GetX()-cog_global_x_test);
  globalLS_centered[i].SetY(globalLS_centered[i].GetY()-cog_global_y_test);
} 

//update the COGXXX ///SIGN???CHECK ALSO ABOVE..
COG_local_X+=cog_local_x_test;
COG_local_Y+=cog_local_y_test;
COG_global_X+=cog_global_x_test;
COG_global_Y+=cog_global_y_test;

//COMPUTE moments again

compute_moments(localLS_centered, Moments_local, 2,2,0);
compute_moments(globalLS_centered, Moments_global, 2,2,0);

//TEST START
/*
//the centore of gravitz must be at 0,0 - correct it! See comments above why this is not exactly at 0,0
cog_local_x_test=Moments_local[1][0]/Moments_local[0][0];
cog_local_y_test=Moments_local[0][1]/Moments_local[0][0];

cog_global_x_test=Moments_global[1][0]/Moments_global[0][0];
cog_global_y_test=Moments_global[0][1]/Moments_global[0][0];

printf("TEST: Shifts of COGs must be 0: local_x: %.2f, local_y: %.2f, global_x: %.2f global_y: %.2f\n",cog_local_x_test, cog_local_y_test, cog_global_x_test, cog_global_y_test);
*/

//TEST END

theta_local=compute_theta(Moments_local);
theta_global=compute_theta(Moments_global);

printf("Theta local (degree): %.2f, Theta global (degree): %.2f\n", theta_local*180/3.1415, theta_global*180/3.1415);

//compute the size of the bounding box, corrected for theta

double Lx_local, Ly_local, Lx_global, Ly_global;

compute_Lx_Ly(Moments_local[0][0],Moments_local[1][1], Moments_local[2][0],Moments_local[0][2], theta_local, Lx_local, Ly_local);
compute_Lx_Ly(Moments_global[0][0],Moments_global[1][1],Moments_global[2][0],Moments_global[0][2], theta_global, Lx_global, Ly_global);

printf("local: Lx=%.2f, Ly=%.2f; global: Lx=%.2f, Ly=%.2f\n", Lx_local, Ly_local, Lx_global, Ly_global);

//DEBUG: put a line to show THETA (one for local, one for global), also plot the rotated bounding boxes

ObjectPoint p_th_line_local1, p_th_line_local2, p_th_line_global1, p_th_line_global2, p_tmp1, p_tmp2;
int objnumber = 0;
int linenumber = 1;
ObjectPoints _points;
LineTopology line;
LineTopologies _topos;
//define the points at y=-10*tan theta, x=-10, y=10*tan theta, x=10

double tan_thet_local=tan(theta_local);
double tan_thet_global=tan(theta_global);

p_th_line_local1 = ObjectPoint (-100, -100*tan_thet_local, 0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_th_line_local1);
p_th_line_local2 = ObjectPoint (+100, +100*tan_thet_local, 0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_th_line_local2);
line = LineTopology (linenumber++, 1, objnumber - 1, objnumber);
_topos.push_back(line);

p_th_line_global1 = ObjectPoint (-100, -100*tan_thet_global, 0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_th_line_global1);
p_th_line_global2 = ObjectPoint (+100, +100*tan_thet_global, 0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_th_line_global2);
line = LineTopology (linenumber++, 1, objnumber - 1, objnumber);
_topos.push_back(line);

//NOW the bounding boxes
//First put the box in a local system parallel to x and y around 0,0, then rotate around z-axis by theta
LaserPoints bb_local;bb_local.ReInitialise();
bb_local.push_back(LaserPoint(-Lx_local/2,Ly_local/2,0)); 
bb_local.push_back(LaserPoint(Lx_local/2,Ly_local/2,0));
bb_local.push_back(LaserPoint(Lx_local/2,-Ly_local/2,0));
bb_local.push_back(LaserPoint(-Lx_local/2,-Ly_local/2,0));

 Vector3D rotaxis=Vector3D(0,0,1);
 Rotation3D rotmat(rotaxis,(-1)*theta_local);
 for(uint k=0;k<bb_local.size();k++) 
	  (bb_local)[k]=rotmat.Rotate(bb_local[k]);
 
p_tmp1 = ObjectPoint (bb_local[0].GetX(), bb_local[0].GetY(), 0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_tmp1);
//store the objnumbeer from the first point to close the polygon later
int objnumber_first=objnumber;
p_tmp2 = ObjectPoint (bb_local[1].GetX(), bb_local[1].GetY(),0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_tmp2);
line = LineTopology (linenumber++, 1, objnumber - 1, objnumber); _topos.push_back(line);
p_tmp1 = ObjectPoint (bb_local[2].GetX(), bb_local[2].GetY(), 0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_tmp1);
line = LineTopology (linenumber++, 1, objnumber - 1, objnumber); _topos.push_back(line);
p_tmp2 = ObjectPoint (bb_local[3].GetX(), bb_local[3].GetY(), 0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_tmp2);
line = LineTopology (linenumber++, 1, objnumber - 1, objnumber); _topos.push_back(line);
line = LineTopology (linenumber++, 1, objnumber_first, objnumber); _topos.push_back(line);

//NOW the global bb
LaserPoints bb_global;bb_global.ReInitialise();
bb_global.push_back(LaserPoint(-Lx_global/2,Ly_global/2,0)); 
bb_global.push_back(LaserPoint(Lx_global/2,Ly_global/2,0));
bb_global.push_back(LaserPoint(Lx_global/2,-Ly_global/2,0));
bb_global.push_back(LaserPoint(-Lx_global/2,-Ly_global/2,0));

 
 rotmat=Rotation3D(rotaxis,(-1)*theta_global);
 for(uint k=0;k<bb_global.size();k++) 
	  (bb_global)[k]=rotmat.Rotate(bb_global[k]);
 
p_tmp1 = ObjectPoint (bb_global[0].GetX(), bb_global[0].GetY(), 0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_tmp1);
//store the objnumbeer from the first point to close the polygon later
objnumber_first=objnumber;
p_tmp2 = ObjectPoint (bb_global[1].GetX(), bb_global[1].GetY(), 0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_tmp2);
line = LineTopology (linenumber++, 1, objnumber - 1, objnumber); _topos.push_back(line);
p_tmp1 = ObjectPoint (bb_global[2].GetX(), bb_global[2].GetY(), 0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_tmp1);
line = LineTopology (linenumber++, 1, objnumber - 1, objnumber); _topos.push_back(line);
p_tmp2 = ObjectPoint (bb_global[3].GetX(), bb_global[3].GetY(), 0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_tmp2);
line = LineTopology (linenumber++, 1, objnumber - 1, objnumber); _topos.push_back(line);
line = LineTopology (linenumber++, 1, objnumber_first, objnumber); _topos.push_back(line);

//DEBUG: add 0,0 as a small box to visualize it
p_tmp1 = ObjectPoint (-3, 3, 0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_tmp1);
//store the objnumbeer from the first point to close the polygon later
objnumber_first=objnumber;
p_tmp2 = ObjectPoint (3, 3 , 0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_tmp2);
line = LineTopology (linenumber++, 1, objnumber - 1, objnumber); _topos.push_back(line);
p_tmp1 = ObjectPoint (3, -3 , 0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_tmp1);
line = LineTopology (linenumber++, 1, objnumber - 1, objnumber); _topos.push_back(line);
p_tmp2 = ObjectPoint (-3, -3 , 0, ++objnumber, 0, 0, 0, 0, 0, 0); _points.push_back(p_tmp2);
line = LineTopology (linenumber++, 1, objnumber - 1, objnumber); _topos.push_back(line);
line = LineTopology (linenumber++, 1, objnumber_first, objnumber); _topos.push_back(line);
 
_points.Write("_lines.objpts");
_topos.Write("_lines.top");



//DEBUG END

//Computation of transformation parameters between the two PCs
//From global to local it is COG_local - COG_global. Rotation is theta_local - theta_global (so we first do translation)
Position3D translation_global_local(COG_local_X-COG_global_X, COG_local_Y - COG_global_Y, COG_local_Z - COG_global_Z);
Rotation3D rotmat_global_to_local(rotaxis,theta_global-theta_local);

Position3D translation_local_global(translation_global_local.GetX()*(-1), translation_global_local.GetY()*(-1), translation_global_local.GetZ()*(-1));
Rotation3D rotmat_local_to_global(rotaxis,theta_global-theta_local);

globalLS_to_localLS=globalLS;

//transform each point. Do transformation in two steps and store the translated PC just for demonstration purpose

for (int i=0;i<globalLS_to_localLS.size();i++) 
 {
  globalLS_to_localLS[i]+=translation_global_local;
 // (globalLS_to_localLS[i])=rotmat_global_to_local.Rotate(globalLS_to_localLS[i] - Position3D(COG_local_X, COG_local_Y, COG_local_Z ));
 // globalLS_to_localLS[i]+=Position3D(COG_local_X, COG_local_Y, COG_local_Z );
 
} 
globalLS_to_localLS.Write("global_transformed_to_local_tmp_just_transform.laser",0);	
		
for (int i=0;i<globalLS_to_localLS.size();i++) 
 {
  //globalLS_to_localLS[i]+=translation_global_local;
  (globalLS_to_localLS[i])=rotmat_global_to_local.Rotate(globalLS_to_localLS[i] - Position3D(COG_local_X, COG_local_Y, COG_local_Z ));
  globalLS_to_localLS[i]+=Position3D(COG_local_X, COG_local_Y, COG_local_Z );
 
} 
//write this transformed PC to disk
globalLS_to_localLS.Write("global_transformed_to_local.laser",0);

localLS_to_globalLS=localLS;
//transform each point


	
for (int i=0;i<localLS_to_globalLS.size();i++) 
 {
  localLS_to_globalLS[i]+=translation_local_global;
  (localLS_to_globalLS[i])=rotmat_local_to_global.Rotate(localLS_to_globalLS[i] - Position3D(COG_global_X, COG_global_Y, COG_global_Z ));
  localLS_to_globalLS[i]+=Position3D(COG_global_X, COG_global_Y, COG_global_Z );
} 
//write this transformed PC to disk
localLS_to_globalLS.Write("local_transformed_to_global.laser",0);

//finally we might need to apply a scale correction since the local model is based on a template.
//so the assumption is that the COG is somewhat correct and we just scale 

//first decide if we need to scale:
double scale_x_global_to_local=Lx_local/Lx_global;
double scale_y_global_to_local=Ly_local/Ly_global;
printf("scale global to local x: %.2f / y: %.2f\n", scale_x_global_to_local, scale_y_global_to_local);

//if (( scale_x_global_to_local<= 1.05) && (scale_x_global_to_local >= 0.95) && (scale_y_global_to_local <= 1.05) && (scale_y_global_to_local >= 0.95)) return EXIT_SUCCESS;

//printf("scaling is needed\n");

//first: from global to local ...> basis for the point-to-plane or point-to-point distance measurements because this will be later easier to identify cavities since the template remains untouched.
//problem: the Lx and Ly refer to the main axis of the object, so we need to project them to the coordinate axes before applying a simple axis scale
//in fact we just need to transform the scales in x and y to the actual coordinate axes: 
double scale_x_global_to_local_T=scale_x_global_to_local*cos (theta_local) - scale_y_global_to_local * sin(theta_local);
double scale_y_global_to_local_T=scale_y_global_to_local*cos (theta_local) + scale_x_global_to_local * sin(theta_local);
//now we transform using the two scales in principal it is: for each point the difference to 0,0 is computed (eg dx), then it is 

globalLS_to_localLS_scaled=globalLS_to_localLS;

//transform each point
//also immediately write the global to local poitns scaled to a pts file for CloudCompare
FILE* global_transformed_to_local_scaled_F;
	  if ((global_transformed_to_local_scaled_F=fopen("global_transformed_to_local_scaled.pts","w")) == NULL)
		{
			printf("global_transformed_to_local_scaled.pts!\n");
			return(0);
		}


for (int i=0;i<globalLS_to_localLS.size();i++) 
 {
   double DX=globalLS_to_localLS[i].GetX()-COG_local_X;
   double DY=globalLS_to_localLS[i].GetY()-COG_local_Y;
   double DX_t=DX*scale_x_global_to_local_T;
   double DY_t=DY*scale_y_global_to_local_T;
   globalLS_to_localLS_scaled[i].SetX(COG_local_X+DX_t);
   globalLS_to_localLS_scaled[i].SetY(COG_local_Y+DY_t);
   //THE Z WILL NOT BE scaled (so far?!)!!! What should be the scale? 
   
    fprintf(global_transformed_to_local_scaled_F, "%.3f %.3f %.3f\n", globalLS_to_localLS_scaled[i].GetX(), globalLS_to_localLS_scaled[i].GetY(), globalLS_to_localLS_scaled[i].GetZ());
  
} 
//write this transformed PC to disk
globalLS_to_localLS_scaled.Write("global_transformed_to_local_scaled.laser",0);

//write out the transformation parameters

FILE *trans_par;
  if ((trans_par=fopen("transformation_parameters.txt","w")) == NULL)
		{
			printf("transformation_parameters.txt!\n");
			return(0);
		}

fprintf(trans_par,"Global shift of center of gravity: X=%.3f, Y=%.3f, Z=%.3f\n",translation_global_local.GetX()*(-1), translation_global_local.GetY()*(-1), translation_global_local.GetZ()*(-1));
fprintf(trans_par,"rotation from local to global system  (point cloud need to be shifted first so that the center of gravity is at 0,0,0): %.4f\n", theta_global-theta_local);
fprintf(trans_par,"scale along the x-and y-axis of the coordinate system (from local to global); SX: %.3f, SY: %.3f\n", 1/scale_x_global_to_local_T, 1/scale_y_global_to_local_T);

return EXIT_SUCCESS;
}
