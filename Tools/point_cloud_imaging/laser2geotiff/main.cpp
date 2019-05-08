#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <math.h>
#include "LaserPoints.h"
#include "InlineArguments.h"

#include "DataBounds3D.h"

#include "laservoxel.h"
#include "cv.h"
#include "highgui.h"

/*Takes the output of the 3D classification algs (i.e. LaserPoint file, representing the points in voxelgrid (this is the default,later it could be changed to be more general); the color encodes the label, the underlying voxel side length and the label info. Output: per class a tif file with a binary label mask,including tfw file
 
 
 Process: put the point cloud again in the voxel grid
 create n tif files, n=number of classes, names combined from laser name and class name. resolution same as xy voxel grid. Initialize as black
 We do a 2D projection, i.e. per 2D voxel we assign a color of the voxel in that column with the largest height
 Paint the respective cell of the respecive output tif white

*/


using namespace std;

void compute_pix_coordinate_from_X_Y(double &X, double &Y, double &global_X_min, double &global_Y_max, double &vox_side_length, int &pix_X, int &pix_Y)
{
 
 double pix_X_dou=(X-global_X_min)/vox_side_length;
 double pix_Y_dou=(global_Y_max-Y)/vox_side_length;

 //printf("X=%.2f, Y=%.2f, pix_X_dou=%.8f, pix_Y_dou=%.8f\n",X,Y,pix_X_dou, pix_Y_dou);
 pix_X=(int) round(pix_X_dou);
 pix_Y=(int) round(pix_Y_dou);

}

void write_world_file(char *tfw_filename, double min_X, double min_Y, double vox_length, int vox_num_Y, double offset_for_tfw_X, double offset_for_tfw_Y)
{
  /*
   * A world file file is a plain ASCII text file consisting of six values separated by newlines. The format is:

 pixel X size
 rotation about the Y axis (usually 0.0)
 rotation about the X axis (usually 0.0)
 negative pixel Y size
 X coordinate of upper left pixel center
 Y coordinate of upper left pixel center
*/
  double X=min_X;//+(double) ((double) (0.5)* vox_length);  //because we assume vox. centres in input we do not need to shift by half voxel anymore! (TIFW also assumes pix centre
  double Y=min_Y+(double) ((double) (vox_num_Y/*+0.5*/)* vox_length);
  
  FILE *tfw=fopen(tfw_filename,"w");
 fprintf(tfw,"%f\n0.0\n0.0\n%f\n%.3f\n%.3f\n",vox_length,-1*vox_length,offset_for_tfw_X+X,offset_for_tfw_Y+ Y);
 fclose(tfw);
}


int label_at_Point(LaserPoint & p1, ::map<uint, int> & label_R, ::map<uint, int>  & label_G, ::map<uint, int> & label_B)
{
      
		//iterate over the assigned label colors. If one fits: take that one
		for (::map<uint, int>::iterator it_R=label_R.begin();it_R!=label_R.end();it_R++)
		{	uint labelid=it_R->first;
			if (label_R[labelid]==p1.Red() && label_G[labelid]==p1.Green() && label_B[labelid]==p1.Blue()) {
				return labelid;
				//printf("FOUND LABEL with ID %d at row=%d col=%d\n",labelid,(int) pos.GetX(),(int) pos.GetY());
				}
		}

//else return void label (-1)
return -1;
}

int point_has_color(LaserPoint & p1)
{
  
 if (p1.Red()==0 && p1.Green()==0 && p1.Blue()==0) return 0; 
 
   return 1;
}

void PrintUsage()
{
printf("laser2geotiff : Takes the output of the 3D classification algs (i.e. LaserPoint file, representing the points in voxelgrid (this is the default,later it could be changed to be more general); the color encodes the label, the underlying voxel side length and the label info. Output: per class a tif file with the binary label mask,including tfw file\n");
  printf("Usage: \t3D_classification_to_geotiff -input <laserfile: with color coding class, or just other color>  -voxel_side_length: double val -assume_voxel_centres 1/0 <default:1: means that the input laserfiles already represent laservoxel centres> -output_prefix\n");
  printf("Optional: -label_info <textfile: describing available classes>\n");
  printf("Optional: if the laserdata has been cropped, we can add the offset again for the tfw file: -offset_for_tfw_X xxx -offset_for_tfw_Y xxx\n");
  printf("Optional: restrict the area to be exported to a subset: -minX , -maxX , -minY, -maxY \n");
  
 } 


int main(int argc, char *argv[])
{

FILE *labelinfo;
::map<uint, string> label_name; //maps the label id to the labelname
::map<uint, int> label_R; //maps the label id to the coding (R channel)
::map<uint, int> label_G; //maps the label id to the coding (G channel)
::map<uint, int> label_B; //maps the label id to the coding (B channel)

LaserPoints in_;
in_.ReInitialise();


int numClasses;

printf("3D_classification_to_geotiff \n");

 InlineArguments *args = new InlineArguments(argc, argv);
 
  
  if(args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
     
  if (!args->Contains("-input") || !args->Contains("-voxel_side_length") ||!args->Contains("-output_prefix")) 
    {
	printf("Error: not all arguments given\n");
	PrintUsage();
	exit(0);
    }
      
  bool assume_voxel_centres=args->Integer("-assume_voxel_centres",0);
  double voxel_side_length=args->Double("-voxel_side_length",0);
  double offset_for_tfw_X=args->Double("-offset_for_tfw_X",0);
  double offset_for_tfw_Y=args->Double("-offset_for_tfw_Y",0);
  
  double max_num_voxels_per_tile=args->Double("-max_num_voxels_per_tile",12E6);
  
  bool has_label_info=args->Contains("-label_info");
  has_label_info?printf("LABEL_INFO available!\n"):printf("LABEL_INFO NOT available!\n"); 
  
  printf("Will add an offset to tfw file... X-axis: %.3f, Y-axis: %.3f\n",offset_for_tfw_X,offset_for_tfw_Y);
  
  //if (!assume_voxel_centres) {
   // printf("currently only gridded 3D data is recognized.\n");exit(0);}
 
      //interpreting the label info for image classes file	 if available  
  if (has_label_info)
  {
   printf("reading the label_info file for image classes\n");
  if ((labelinfo=fopen(args->String("-label_info",""),"r")) == NULL)
		{
			printf("CANNOT OPEN %s!\n", args->String("-label_info",""));
			return(0);
		}
		
printf("converting the label names to lower case to ease later on smoothness constraint computation\n");
 while(!feof(labelinfo))
		 {
			int labelid;
			char labelname[500];
			int R,G,B;
			
			fscanf(labelinfo,"%d %d %d %d %s",&labelid, &R, &G, &B, labelname);	
			if (labelid==-99) continue;	
			if (feof(labelinfo)) break;
			//printf("%d %d %d %d %s\n",labelid, R, G, B, labelname );

			label_name[(uint)labelid]=labelname;
			label_R[(uint)labelid]=R;
			label_G[(uint)labelid]=G;
			label_B[(uint)labelid]=B;
			
			printf("%d %d %d %d %s\n",labelid, R, G, B, label_name[(uint)labelid].c_str() );
			
		 }
fclose(labelinfo);
 numClasses=label_name.size();
 printf("-->will expect %d different classes\n",numClasses);
 
    
  }//if label info is there

  else{
 
 numClasses=0;
 printf("-->will expect 0 classes, just RGB color values\n");
  }
  
 //if we have them already as voxels directly read into the in_ instance, otherwise, first as dummy and then create the laservoxels
 if (assume_voxel_centres) {
 
  printf("\n Reading classified laser points, assume they are already voxel centres...\n");
	  if (!in_.Read(args->String("-input"),1)) {
		  fprintf(stderr, "Error reading file %s\n", args->String("-input"));
		  exit(0);
	  }
	  
 }
 
 else //we need to create the voxels firstr
  {
    printf("\n Reading classified laser points, assume they are NOT YET voxel centres, but irregular...\n");
    LaserPoints dummy; dummy.ReInitialise();
    if (!dummy.Read(args->String("-input"),1)) {
		    fprintf(stderr, "Error reading file %s\n", args->String("-input"));
		    exit(0);
	    }
	    
  LaserVoxel lv(dummy,voxel_side_length,1,0);	
  in_=lv.export_vox_centres(0,0); 
  
 //IMPORTANT! From now on we need to assume voxel centres becaue we are just working with the created voxels!
 
 
 in_.Write("laservoxel_created.laser",1);
 
 assume_voxel_centres=1;
 
  }
  
 
	///ADD TILING HERE TO BE ABLE TO HANDLE LARGE AREA POINTCLOUDS!!!
	//SEE 3D Graphcut.. similar. only be careful with the labelfiles... row/col offset according to tile.
	//Instantiate the images before the actual tiling  
	  /*Tiling stuff...*/
	 DataBoundsLaser db_global=in_.DeriveDataBounds(0);
	 
	  printf("overall bounds: X: from %.2f to %.2f, Y:  from %.2f to %.2f\n",db_global.Minimum().X(),db_global.Maximum().X(),db_global.Minimum().Y(),db_global.Maximum().Y());
	  
	  //if given in the command line: overwrite the X and Y global bounds!!!
	  if (args->Contains("-minX") && args->Contains("-maxX") && args->Contains("-minY") && args->Contains("-maxY"))
	  {
	  
	  db_global.SetMinimumX(args->Double("-minX",0));
           db_global.SetMinimumY(args->Double("-minY",0));
            db_global.SetMaximumX(args->Double("-maxX",0));
           db_global.SetMaximumY(args->Double("-maxY",0));
	  
	   printf("OVERWRITTEN BOUNDS: overall bounds: X: from %.2f to %.2f, Y:  from %.2f to %.2f\n",db_global.Minimum().X(),db_global.Maximum().X(),db_global.Minimum().Y(),db_global.Maximum().Y());
	   
	  }
	  
	  //exit(0);
	  
	  //For Z use the overall bounds; only cropping in X-Y-Plane
	  double overall_X=db_global.Maximum().X()-db_global.Minimum().X();
	  double overall_Y=db_global.Maximum().Y()-db_global.Minimum().Y();
	  double overall_Z=db_global.Maximum().Z()-db_global.Minimum().Z();
	  

	 
	   

	  //we want to have 1-2 Mio Voxels per run, and the Z is fixed, so we remove those already
	  double vox_for_XY_per_tile=max_num_voxels_per_tile/(overall_Z/voxel_side_length);
	  printf("\ndeltas: overall_X=%.1f  overall_Y=%.1f   overall_Z=%.1f\n",overall_X,overall_Y,overall_Z);
	  printf("since %.2e voxels are wanted per run and given the overall_Z, there remain %.1f ^2 voxel for the XY plane per run\n",max_num_voxels_per_tile,vox_for_XY_per_tile);
	  uint vox_total_X=(uint) (round(overall_X/voxel_side_length)+1); //... it is +1 because we assume voxel centres as input, thus the left and right (upper/lower) half-voxel is missing in overall_XXX
	  
	  uint vox_total_Y=(uint) (round(overall_Y/voxel_side_length)+1);
	  uint vox_total_XY=vox_total_X*vox_total_Y;
	  
	  uint tilenumber_total=ceil ((float) vox_total_XY/vox_for_XY_per_tile);
	  if (tilenumber_total%2==1) tilenumber_total++;
	  
	  printf("Overall number of voxels :%u, i.e. we need %u tiles in total\n",vox_total_XY, tilenumber_total);
	  
	  //as approximation we assume quadratic tiles, i.e. the tile_length_voxel is sqrt(vox_for_XY_per_tile)
	  double t_x_s=sqrt(vox_for_XY_per_tile);
	  
	  //now we correct that with the ration overall_X/overall_Y
	  double ratio_x_y=overall_X/overall_Y;
	  double t_x=t_x_s/ratio_x_y;  //tile size (voxel) in x
	  double t_y=t_x_s*ratio_x_y;
	  
	  printf("\ntile size (voxel) in x: %.0f, in y:%.0f (cross check: area: %.1f), tile size in m: x= %.1f, y=%.1f\n",t_x,t_y,t_x*t_y,t_x*voxel_side_length, t_y*voxel_side_length);
	  
	  
int tilecount_x=-1, tilecount_y;


//LaserVoxel lv(in_,voxel_side_length,1,assume_voxel_centres);

//uint num_voxels=lv.get_vox_num_X()*lv.get_vox_num_Y()*lv.get_vox_num_Z();

//printf("Grid size: %d x %d x %d\n",lv.get_vox_num_X(),lv.get_vox_num_Y(),lv.get_vox_num_Z());


//copy the final 2D labels into the lowest layer (k=0): iterate over all x y cells and find the topmost voxel which contains data valid class only!)
 //then put that point in the k==0 cell 
 //then we need to fill gaps per label: 
   //iterate over all labels, then over the k==0 grid plane. For all points which are void, but have at least 7 neighbors of the same color: fill it in!

//ITERATION OVER TILES

//copy data to Images: iterate over all x y cells in k==0 then fill the respet.cell in the respective image with white, or the color respecively

printf("the output images will have a dimension of %d X %d pixels\n",vox_total_X,vox_total_Y);
		  CvScalar val_grey_white;val_grey_white.val[0]=255;
		  
		  //create num_classes image instances... Also a color image which will show all classes
		   vector<IplImage*> imgout(numClasses);
		if (has_label_info)
		{
		  
		  for (int c=0;c<numClasses;c++)
		  {
		    imgout[c] = cvCreateImage( cvSize(vox_total_X,vox_total_Y), 8, 1 );
		    cvSetZero(imgout[c]);
		  }
		}
		
               IplImage *imgout_allclasses;
		imgout_allclasses= cvCreateImage( cvSize(vox_total_X,vox_total_Y), 8, 3 );
		  cvSetZero(imgout_allclasses);
		
LaserPoints final_lp_one_layer_all_classes; final_lp_one_layer_all_classes.ReInitialise();
		 

   
for (double minX=db_global.Minimum().X();minX<=db_global.Maximum().X()-2*voxel_side_length;minX+=(t_x-4)*voxel_side_length) //2*... arbitrary, to avoid conflicts with rounding
{tilecount_x++;tilecount_y=-1;
 for (double minY=db_global.Minimum().Y();minY<=db_global.Maximum().Y()-2*voxel_side_length;minY+=(t_y-4)*voxel_side_length)
 {tilecount_y++;
   //the tiles should overlap because of neigborhood search
   double maxX,maxY;
   //maxX=(minX<db_global.Maximum().X()-t_x*voxel_side_length)?minX+t_x*voxel_side_length-0.02:minX+t_x*voxel_side_length;
   //maxY=(minY<db_global.Maximum().Y()-t_y*voxel_side_length)?minY+t_y*voxel_side_length-0.02:minY+t_y*voxel_side_length;
   maxX=minX+(t_x+4)*voxel_side_length;
   maxY=minY+(t_y+4)*voxel_side_length;
   
   maxX=MIN(maxX,db_global.Maximum().X());
   maxY=MIN(maxY,db_global.Maximum().Y());
   
   printf("tile: %d,%d (from approx %.0f, %.0f): bounds used for this tile: X: from %.2f to %.2f, Y:  from %.2f to %.2f\n",tilecount_x,tilecount_y,vox_total_X/t_x, vox_total_Y/t_y, minX,maxX,minY,maxY);
   
//DEBUG: move to a particular tile
 //if (tilecount_x!=1 || tilecount_y!=15) continue;
   
   //debug: for some testadata go to tile 1 1 to force adding of meaning to background 
   //if (tilecount_x==0 || tilecount_y==0) continue;
   
   //debug: stof after firstr tile column
    //if (tilecount_x >0) {printf("Debug: end after first column of tiles\n");continue;}
    
  //crop it (ReduceData) using DataBoundsLaser. 
  DataBoundsLaser db_local(db_global); //inherit the global Z, so we only need to modify X Y locally
  
  
  db_local.SetMinimumX(minX);
  db_local.SetMinimumY(minY);
  db_local.SetMaximumX(maxX);
  db_local.SetMaximumY(maxY);
  
   //clone the data
  LaserPoints in_crop(in_);
  in_crop.ReduceData(db_local);
  
  if (in_crop.size()<3) continue;
  
  in_crop.Write("part_crop.laser",0);
  
  
  LaserVoxel lv(in_crop,voxel_side_length,1,assume_voxel_centres);
		
  
  LaserPoints debug_added; debug_added.ReInitialise();



		for (uint i=0; i<lv.get_vox_num_X();i++)
		    for (uint j=0; j<lv.get_vox_num_Y();j++)
		    for (int k=lv.get_vox_num_Z()-1;k>=0;k--)
		    {
		      //printf("i=%d, j=%d,k=%d\n",i,j,k);
		      LaserPoint lp_here;
		      if (lv.export_vox_centre_at(i,j,k, lp_here,1)) //there is a valid point here! if it has a valid label go on!
			{
			  
			  if (has_label_info)
			  {
			  int cla=label_at_Point(lp_here,label_R,label_G,label_B);
			  //printf("i=%d, j=%d, k=%d,class=%d\n",i,j,k,cla);
			  
			  if(cla!=-1)//it is a label from the label_info (if it applies)
			      {
				//printf("i=%d, j=%d, k=%d,class=%d\n",i,j,k,cla);
				//put the point to the lowest level (k==0)
				if (k!=0) //if it is already 0, we do not need to do anything
				{
				  //remove the point which is in this voxel (if there is any)
				  lp_here.SetZ(0);//put the Z to an artifical number
				  lv.empty_voxel_at_ikj( i, j, 0 );
				  lv.add_a_LaserPoint_to_vox(i, j,  0, lp_here);
				  debug_added.push_back(lp_here);
				
				//break the k-loop and go to the next xy cell
				break;
				}
			      }
			  } //if we have label info
			  
			  else // just go on if there is color in this cell
			    {
			       
			      if (point_has_color(lp_here))
			      {
				
				//printf("i=%d, j=%d, k=%d,class=%d\n",i,j,k,cla);
				//put the point to the lowest level (k==0)
				if (k!=0) //if it is already 0, we do not need to do anything
				{
				  //remove the point which is in this voxel (if there is any)
				  lp_here.SetZ(0);//put the Z to an artifical number
				  lv.empty_voxel_at_ikj( i, j, 0 );
				  lv.add_a_LaserPoint_to_vox(i, j,  0, lp_here);
				  debug_added.push_back(lp_here);
				
				//break the k-loop and go to the next xy cell
				//break;
				}
				
				
				
			      } //if there is color
			      
			      
			    }			
			  
			}
		      }
		      
		lv.export_all().Write("debug.laser",0);
		debug_added.Write("debug_added_atkzero.laser",0);
		
		
		      //NOW in the layer k==0 we have the final XY grid with all the labels (or colors, respctively). The problem now is that there are voids still in this raster (e.g. from laserscanning void areas etc). 
		      //for the final raster those gaps must be closed, but we need to ensure that the outline of objects (fist of all of buildings) does not get extended or modified. Therefore also no morphologic closing in the images later is done.
		      //idea: iterate over the labels, then over the raster and if there is a void cell check if it has at least 7 neighbors in the XY neighborhood (8 neigbors) of the same label. If yes: fill it...
		        
	
		  
		uint neighb_size=3;   //3   //5 //!!====>5 for vox size 0.4, 3 for vox_size 0.2
		int min_neighbors=12; //12 //5   //12 or 10 ==> not too small... objects grow together
		
		//these loops will not be entered when there is no label info (numClasses=0). This is fine, but the case when we do not have labels needs to be covered below
		  for (int c=0;c<numClasses;c++)
		  for (uint i=0; i<lv.get_vox_num_X();i++)
		    for (uint j=0; j<lv.get_vox_num_Y();j++)
		    {
		     
		      LaserPoint lp_here;
		      if (lv.export_vox_centre_at(i,j,0, lp_here,1)) //there is avalid point here! if it has a valid label go on!
			{
			   int pix_X, pix_Y;
                           compute_pix_coordinate_from_X_Y(lp_here.X(), lp_here.Y(), db_global.Minimum().X(), db_global.Maximum().Y(), voxel_side_length, pix_X, pix_Y);
			   //debug: check the range of pix_X Y
			  if (pix_X<0 || pix_X>vox_total_X-1) {printf("error: pix_X outside range: %d\n",pix_X);exit(0);}
 			   if (pix_Y<0 || pix_Y>vox_total_Y-1) {printf("error: pix_Y outside range: %d\n",pix_Y);exit(0);}

			  int cla=label_at_Point(lp_here,label_R,label_G,label_B);
			  //printf("i=%d, j=%d,class=%d\n",i,j,cla);
			  
			  if(cla==c)//it is a label from the label_info and the current class... so write it directly..
			      {
				//paint the respective label image white and the all_image with the color  
				cvSet2D(imgout[c], pix_Y, pix_X, val_grey_white);
				//also the multi color image
				CvScalar s; 
				  s.val[0]=label_B[c];
				  s.val[1]=label_G[c];
				  s.val[2]=label_R[c];
				cvSet2D(imgout_allclasses,pix_Y, pix_X,s);

				//also add to the final debug laserpoints instance
				final_lp_one_layer_all_classes.push_back(lp_here);
			      }
			} //wrote an existing point with label c to image
			/**/
			else //this is a void voxel. Check if in the 3x3 (5x5?) neighborhood we have enought points of the target class
		      {
			double dummy_X,dummy_Y,dummy_Z;
			LaserPoints neig;
			int num_neig=lv.neighbors_at_ijk(i,j,0,neighb_size, 1, neig, dummy_X, dummy_Y, dummy_Z, 1);
			
			int neighbrs_same_cls=0;
			
			
			for (int ne=0; ne<neig.size();ne++) 
			  if (label_at_Point(neig[ne],label_R,label_G,label_B) == c) neighbrs_same_cls++;
			
			  if (neighbrs_same_cls<min_neighbors) continue;
			
			   int pix_X, pix_Y;
                           compute_pix_coordinate_from_X_Y(dummy_X, dummy_Y, db_global.Minimum().X(), db_global.Maximum().Y(), voxel_side_length, pix_X, pix_Y);
			   //debug: check the range of pix_X Y
			  if (pix_X<0 || pix_X>vox_total_X-1) {printf("error: pix_X outside range: %d\n",pix_X);exit(0);}
 			   if (pix_Y<0 || pix_Y>vox_total_Y-1) {printf("error: pix_Y outside range: %d\n",pix_Y);exit(0);}

			  //at this point we can paint the cell!!!
			  cvSet2D(imgout[c], pix_Y, pix_X, val_grey_white);
			  //also the multi color image
			  CvScalar s; 
			    s.val[0]=label_B[c];
			    s.val[1]=label_G[c];
			    s.val[2]=label_R[c];
			  cvSet2D(imgout_allclasses,pix_Y, pix_X,s);
			  
			  //also add this point to the voxelgrid!!!
			  LaserPoint dummyp(dummy_X,dummy_Y,0);
			  dummyp.SetColour(label_R[c],label_G[c],label_B[c]);
			  lv.add_a_LaserPoint_to_vox(i, j,  0, dummyp);
			  
			  
			  //Add a dummy point to the laserfile for debug
			debug_added.push_back(dummyp);
			
		
		      }
		      /**/
		      
		      }
		      
		      
		      
		  if (!has_label_info)
		  {
		    neighb_size=0; //for color do not spread so much, may be even set it to 0
		    //dirty soluton: copy and paste and modify for color where necessary
		    
		    for (uint i=0; i<lv.get_vox_num_X();i++)
		    for (uint j=0; j<lv.get_vox_num_Y();j++)
		    {
		     
		      LaserPoint lp_here;
		      if (lv.export_vox_centre_at(i,j,0, lp_here,1)) //there is av alid point here! if it has a valid color go on!
			{
			   int pix_X, pix_Y;
                           compute_pix_coordinate_from_X_Y(lp_here.X(), lp_here.Y(), db_global.Minimum().X(), db_global.Maximum().Y(), voxel_side_length, pix_X, pix_Y);
			   //debug: check the range of pix_X Y
			  if (pix_X<0 || pix_X>vox_total_X-1) {printf("error: pix_X outside range: %d\n",pix_X);exit(0);}
 			   if (pix_Y<0 || pix_Y>vox_total_Y-1) {printf("error: pix_Y outside range: %d\n",pix_Y);exit(0);}

			  
			  
			  if (point_has_color(lp_here))//it has some color
			      {
				
				//color the multi color image
				CvScalar s; 
				  s.val[0]=lp_here.Blue();
				  s.val[1]=lp_here.Green();
				  s.val[2]=lp_here.Red();
				cvSet2D(imgout_allclasses,pix_Y, pix_X,s);

				//also add to the final debug laserpoints instance
				final_lp_one_layer_all_classes.push_back(lp_here);
			      }
			} //wrote an existing point with label c to image
			/**/
			else //this is a void voxel. Compute the MEDIAN COLOR FROM THE DEFINED NEIGHORHOOD
		      {
			
			double dummy_X,dummy_Y,dummy_Z;
			LaserPoints neig;
			int num_neig=lv.neighbors_at_ijk(i,j,0,neighb_size, 1, neig, dummy_X, dummy_Y, dummy_Z, 1);
			  
			//if there are no neigbors in that area, leave it black!
			if (neig.size()==0) continue;
			
			vector<uint> r_vec, g_vec, b_vec;r_vec.resize(0);g_vec.resize(0);b_vec.resize(0);
			
			for (int ne=0; ne<neig.size();ne++) 
			{
			r_vec.push_back(neig[ne].Red());
			g_vec.push_back(neig[ne].Green());
			b_vec.push_back(neig[ne].Blue());
			}
			
			uint pos=floor(num_neig/2);
			int med_r=(int) r_vec[pos];
			int med_g=(int) g_vec[pos];
			int med_b=(int) b_vec[pos];
			  
			   int pix_X, pix_Y;
                           compute_pix_coordinate_from_X_Y(dummy_X, dummy_Y, db_global.Minimum().X(), db_global.Maximum().Y(), voxel_side_length, pix_X, pix_Y);
			   //debug: check the range of pix_X Y
			  if (pix_X<0 || pix_X>vox_total_X-1) {printf("error: pix_X outside range: %d\n",pix_X);exit(0);}
 			   if (pix_Y<0 || pix_Y>vox_total_Y-1) {printf("error: pix_Y outside range: %d\n",pix_Y);exit(0);}

			 //color the final image with the median color!
			  CvScalar s; 
			    s.val[0]=med_b;
			    s.val[1]=med_g;
			    s.val[2]=med_r;
			  cvSet2D(imgout_allclasses,pix_Y, pix_X,s);
			  
			  //also add this point to the voxelgrid!!!
			  LaserPoint dummyp(dummy_X,dummy_Y,0);
			  dummyp.SetColour(med_r,med_g,med_b);
			  lv.add_a_LaserPoint_to_vox(i, j,  0, dummyp);
			  
			  
			  //Add a dummy point to the laserfile for debug
			debug_added.push_back(dummyp);
			
		
		      }
		      /**/
		      
		      }
		    
		    
		  }//if no label info is provided
		     
		      
		      
		      
		      
		  
		debug_added.Write("debug_added_atkzero_plus_gapfill.laser",0);
		
		//printf("Debug exit after first tile!\n");break;


 }//tiles ...y
 //printf("Debug exit after first row!\n");break;
}//tiles...x

final_lp_one_layer_all_classes.Write("debug_all_one_layer_nogaps.laser",0);
//write images
                  
char name[255];
      strcpy(name,args->String("-output_prefix"));
      strcat(name,"_all_classes.tif");
  
 cvSaveImage(name,imgout_allclasses);
 
 //write tifw file!
   strcpy(name,args->String("-output_prefix"));
      strcat(name,"_all_classes.tfw");
   
      
  //!lv.write_world_file(name,offset_for_tfw_X,offset_for_tfw_Y);
      write_world_file(name, db_global.Minimum().X(), db_global.Minimum().Y(), voxel_side_length, vox_total_Y, offset_for_tfw_X, offset_for_tfw_Y);
      
//the following is done anyhow only if label_info is provided (numClasses >0)
for (int c=0;c<numClasses;c++)
{
  char name[255];
      strcpy(name,args->String("-output_prefix"));
      strcat(name,"_");
      strcat(name,label_name[c].c_str());
      strcat(name,".tif");
  
  cvSaveImage(name,imgout[c]);
//write tifw file!
  strcpy(name,args->String("-output_prefix"));
      strcat(name,"_");
      strcat(name,label_name[c].c_str());
      strcat(name,".tfw");
      
      //!lv.write_world_file(name,offset_for_tfw_X,offset_for_tfw_Y);
      write_world_file(name, db_global.Minimum().X(),db_global.Minimum().Y(), voxel_side_length, vox_total_Y, offset_for_tfw_X, offset_for_tfw_Y);
}

//write the individual files after some morphologic closing operation
  int kernelsize_erosion=3;
  int kernelsize_dilation=3;
 
   int anc_ero=(int) floor((double)kernelsize_erosion/2);//-1
   int anc_dil=(int) floor((double)kernelsize_dilation/2);//-1
   IplConvKernel* kern_ero=cvCreateStructuringElementEx(kernelsize_erosion,kernelsize_erosion,anc_ero,anc_ero,CV_SHAPE_ELLIPSE);// CV_SHAPE_RECT / CV_SHAPE_ELLIPSE
   IplConvKernel* kern_dil=cvCreateStructuringElementEx(kernelsize_dilation,kernelsize_dilation,anc_dil,anc_dil,CV_SHAPE_ELLIPSE);// CV_SHAPE_RECT / CV_SHAPE_ELLIPSE

 
 for (int c=0;c<numClasses;c++)
{
     //Closing: dilation (+) erosion
    cvDilate(imgout[c],imgout[c],kern_dil,1);
     //cvSaveImage("/tmp/planetoimg_DILATE.tif",img);
    cvErode(imgout[c],imgout[c],kern_ero,1);
     //cvSaveImage("/tmp/planetoimg_CLOSED.tif",img);
     
  char name[255];
      strcpy(name,args->String("-output_prefix"));
      strcat(name,"_");
      strcat(name,label_name[c].c_str());
      strcat(name,"_CLOSING.tif");
  
  cvSaveImage(name,imgout[c]);
//write tifw file!
  strcpy(name,args->String("-output_prefix"));
      strcat(name,"_");
      strcat(name,label_name[c].c_str());
      strcat(name,"_CLOSING.tfw");
      
      //!lv.write_world_file(name,offset_for_tfw_X,offset_for_tfw_Y);
      write_world_file(name, db_global.Minimum().X(),db_global.Minimum().Y(), voxel_side_length, vox_total_Y, offset_for_tfw_X, offset_for_tfw_Y);
}
return EXIT_SUCCESS;
}
