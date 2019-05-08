#include <laservoxel.h>

//LaserVoxel::LaserVoxel(void)
//{
 
//}

LaserVoxel::LaserVoxel(LaserPoints &ls, double vox_l,bool verbose)
{
 vox_length=vox_l;
 

 //get the databounds from the ls
 DataBoundsLaser db=ls.DeriveDataBounds(0);
 min_X=db.Minimum().GetX();
 min_Y=db.Minimum().GetY();
 min_Z=db.Minimum().GetZ();
 
 //length of vector componetnts: compute index for max vals - +1;
 vox_num_X=index_from_realX(db.Maximum().GetX())+1;
 vox_num_Y=index_from_realY(db.Maximum().GetY())+1;
 vox_num_Z=index_from_realZ(db.Maximum().GetZ())+1;
 if (verbose)
 {
 printf("Number of Laserpoints:%d\n",ls.size()); 
 printf("length of voxel:%.2f\n",vox_length);
 printf("min_X=%.2f, min_Y=%.2f, min_Z=%.2f\n",min_X,min_Y,min_Z);
 printf("num of voxels in X:%d, Y:%d, Z:%d: Total:%d\n",vox_num_X,vox_num_Y,vox_num_Z,vox_num_X*vox_num_Y*vox_num_Z);
 
 printf("...allocating memory...wait\n");
 }
 //allocate memory
 Vox.resize(vox_num_X);
 for (uint i=0;i<vox_num_X;i++)
 {
   Vox[i].resize(vox_num_Y);
   
   for (uint j=0; j<vox_num_Y;j++)
   {
     Vox[i][j].resize(vox_num_Z); 
     for (uint k=0;k<vox_num_Z;k++)
       Vox[i][j][k]=NULL;
   }
   
 }
 
 if ( verbose) printf("...putting data into structure...wait\n");
 //put the data into the voxel
 LaserPoints::iterator point;
    for (point=ls.begin(); point!=ls.end(); point++)
	  {
	    LaserPoint lp=point->LaserPointRef();
	    double X=lp.GetX();
	    double Y=lp.GetY();
	    double Z=lp.GetZ();
	    
	    uint i=index_from_realX(X);
	    uint j=index_from_realY(Y);
	    uint k=index_from_realZ(Z);
	    
	    //printf("in reading: i=%d, j=%d, k=%d\n",i,j,k);
	    if (Vox[i][j][k]==NULL) Vox[i][j][k]=new LaserPoints;
	    
	    Vox[i][j][k]->push_back(lp);
	    
	  }
if (verbose) printf("...done!\n");
} //Constructor

LaserVoxel::LaserVoxel(double vox_l)
{
  
 vox_length=vox_l;
 bool verbose=1;
 
 printf("Instantiating Vox only with vox_length for filling gaps\n");
 
}
LaserVoxel::~LaserVoxel()
{
 for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     {
       if (Vox[i][j][k]!=NULL) delete(Vox[i][j][k]);
      } 
}
 vector<LaserPoints> LaserVoxel::LaserPoints_per_voxel()
 {
   vector<LaserPoints> retvec; retvec.resize(0); 
   
    for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     {
       if (Vox[i][j][k]!=NULL) 
       {
	 LaserPoints tmpl;tmpl.ReInitialise();
	 //printf("in cell %d %d %d: %d points\n",i,j,k,Vox[i][j][k]->LaserPointsReference().size());
	uint size=Vox[i][j][k]->LaserPointsReference().size();
	for (uint l=0;l<size;l++)
	  tmpl.push_back(Vox[i][j][k]->LaserPointsReference()[l]);
	  
	retvec.push_back(tmpl);
	
       }
     }
   
   return (retvec);
 }
 
void LaserVoxel::statistics()
{
::map<uint, uint > statmap; //first: number of points per voxel, second: occurence
  for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     {
       if (Vox[i][j][k]!=NULL) 
       {
	 //printf("in cell %d %d %d: %d points\n",i,j,k,Vox[i][j][k]->LaserPointsReference().size());
	uint size=Vox[i][j][k]->LaserPointsReference().size();
	statmap[size]++;
       }
     }
 
printf("\nStatistics:\n"); 
uint total_voxels=0;
for (::map<uint, uint>::iterator it=statmap.begin();it!=statmap.end();it++)
{
    printf("number of points/voxel: %d, occurence: %d\n",it->first,it->second);
    total_voxels+=it->second;
}
printf("\nTotal number of voxels:%d\n",total_voxels);
printf("\n\n");
}

void LaserVoxel::filter_size(uint min,bool verbose)
{
  if (verbose) printf("Voxel fiter: min number of points per voxel required: %d\n",min);
 for (uint i=0;i<vox_num_X;i++)
   for (uint j=0;j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     {
       if (Vox[i][j][k]!=NULL)
	 if (Vox[i][j][k]->LaserPointsReference().size() < min) 
	 {//delete(Vox[i][j][k]);
	  Vox[i][j][k]=NULL;
	 }
      }  
   if (verbose) printf("...done\n");
}

void LaserVoxel::filter_neighbour(uint min, bool verbose)
{
  if (verbose) printf("Voxel fiter: min number of points in neighborhood required: %d\n",min);
  for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     {
       if (Vox[i][j][k]!=NULL)
       {
	 uint found_p=0;
	 uint zero=0;
	
	   for (uint a=max(i-1,zero);a<=i+1;a++) //the indices may not be <0
	   {
	      if (a>=vox_num_X) continue;
	      
	      for (uint b=max(j-1,zero);b<=j+1;b++)
	      {
		if (b>=vox_num_Y) continue;
		  
		for (uint c=max(k-1,zero);c<=k+1;c++)
		  {if (c>=vox_num_Z) continue;
		      if (a==i && b==j && c==k) continue; //the point itself is not counted
			if (Vox[a][b][c]!=NULL)
			  //found_p+=Vox[a][b][c]->LaserPointsReference().size(); 	//count the number of original points
			  found_p++; //only count the number of filled voxels
		    }//c
	      }//b	   
	   }//a
	   if (found_p<min) Vox[i][j][k]=NULL;
       }
      }
  if (verbose) printf("...done\n");
}

int  LaserVoxel::number_of_points_in_voxel_at(double X, double Y, double Z, LaserPoints *PointsInVoxel)
{
   if (X<min_X || Y<min_Y || Z<min_Z)  {return 0;}
 
   
   uint i=index_from_realX(X);
   uint j=index_from_realY(Y);
   uint k=index_from_realZ(Z);
   
   if (i <0 || i >= vox_num_X) {return 0;}
   if (j <0 || j >= vox_num_Y) {return 0;}
   if (k <0 || k >= vox_num_Z) {return 0;}
   
   if (Vox[i][j][k]==NULL) return 0;
   else 
   {
     if (PointsInVoxel!=NULL)
       PointsInVoxel=Vox[i][j][k]->LaserPointsPointer();
     
     return Vox[i][j][k]->LaserPointsReference().size(); 
   }
   
}

IplImage* LaserVoxel::plane_segment_to_image()
{

//the voxels are supposed to be in a plane, with DZ=0. If this is not the case exit!
  if (vox_num_Z!=1) {printf("ERROR in implementation: in LaserVoxel::plane_to_image the extent in Z is !=1 !! . Exit\n!"); exit(0);}
IplImage *img = cvCreateImage( cvSize(vox_num_X,vox_num_Y), 8, 1 );
cvZero(img);

for (uint j=0;j<vox_num_Y;j++)
{
   uchar* ptr = (uchar*) (img->imageData + j * img->widthStep);
   for (uint i=0; i<vox_num_X;i++)	 
     if (Vox[i][j][0]!=NULL) 
      {
        ptr[i]=255;
      } 
}


return img;
}

void LaserVoxel::image_to_plane_segment(IplImage* img)
{

//the voxels are supposed to be in a plane, with DZ=0. If this is not the case exit!
  if (vox_num_Z!=1) {printf("ERROR in implementation: in LaserVoxel::plane_to_image the extent in Z is !=1 !! . Exit\n!"); exit(0);}

for (uint j=0;j<vox_num_Y;j++)
{
   uchar* ptr = (uchar*) (img->imageData + j * img->widthStep);
   for (uint i=0; i<vox_num_X;i++)	 
     if (ptr[i]!=0 && Vox[i][j][0]==NULL) //if this raster point is set, but no Voxel yet: fill it with a point at the centre
      {
          double X=min_X+(double) ((double) (i+0.5)* vox_length);
          double Y=min_Y+(double) ((double) (j+0.5)* vox_length);
          double Z=min_Z+(double) ((double) (0.5)* vox_length);
	  LaserPoint lp(X,Y,Z);
	  lp.SetColour(255,255,255);
          Vox[i][j][0]=new LaserPoints;    
	  Vox[i][j][0]->push_back(lp);
      } 
}

}


bool LaserVoxel::fill_gaps_for_coplanar_points(LaserPoints &ls, double kernelsize_objectspace, LaserPoints *FillUpDataSegment, bool project_all_to_plane)
{
  //given: Laserpoints which are supposed to be on a plane (eg. through segmentation in PCM, and then filtered according to segment number
  //idea:
  //1) put them into a local voxelfield
  //2) compute the plane normal (only from voxelcentres)
  //3) use LaserPoints::ProjectToPlane to project the voxelcentres to a plane
  //4) those points on the plane: put into another voxel (actually it is than a raster since dz=0)
  //5) create an IPLimage from that raster
  //6) do some image processing: closing using openCV 
  //7) convert this image back to the voxel/raster field from step 4)
  //8) after that we need to find those points which have been added: for all voxels in the initial voxelfield (step 1) (ALSO EMPTY ONES):
    //check, if the voxel which is on the projection to that plane is empty or not. Change the actual voxel accordingly.
    //-> this has the advantage that it is quite flexible, so if (may be in another image processing) some points are removed, they will also be removed from the voxelfield
  //9) the LaserPoints instance is then composed out of the final voxel centres
  
  //1)
  LaserVoxel vox_planeorig(ls,this->vox_length*0.9); //the voxellength is a little smaller to minimize quantification (aliasing) effects
  
  //2) 
  LaserPoints vox_planeorig_centres=vox_planeorig.export_vox_centres(0); //only the ones which are filled)
  
   Plane plorig(ls[0],ls[1],ls[2]);
     for(int k=3;k<ls.size();k++)
       plorig.AddPoint(ls[k],true);
     
  Vector3D NormalPlane=plorig.Normal();
  
  //test: original normal_voxorig
  //Vector3D normal_orig=ls.Normal();
  //printf("normal components: orig x %.2f  y %.2f  z %.2f\n from vox centres x %.2f  y %.2f  z %.2f\n",normal_orig.X(),normal_orig.Y(),normal_orig.Z(),normal_voxorig.X(),normal_voxorig.Y(),normal_voxorig.Z());
  
  //3) 
  LaserPoints voxcentres_to_plane=vox_planeorig_centres.ProjectToPlane(NormalPlane);
  //test: export
  vox_planeorig_centres.Write("/tmp/orig_voxcenters.laser",0);
  voxcentres_to_plane.Write("/tmp/plane_voxcenters.laser",0);
  
  //4)
  LaserVoxel vox_plane_xy(voxcentres_to_plane,this->vox_length*0.75);//again make the voxels a little smaller
  
  //5
  IplImage* img=vox_plane_xy.plane_segment_to_image();
  
   cvSaveImage("/tmp/planetoimg.tif",img);
  
  //6
  //create a quadratic kernel (test with circular as well?)
  //convert from kernelsize in objectspace to imagespace
   int kernelsize=(int) ceil (kernelsize_objectspace/(this->vox_length*0.75));
   
   if (kernelsize%2 !=1) kernelsize++; //make it odd;
   printf("Kernesize required in objectspace:%.1f , voxellength for this plane:%.2f --> kernelsize in image:%d\n", kernelsize_objectspace,this->vox_length*0.75,kernelsize);
   int anc=(int) floor((double)kernelsize/2);
   IplConvKernel* kern=cvCreateStructuringElementEx(kernelsize,kernelsize,anc,anc,CV_SHAPE_ELLIPSE);// CV_SHAPE_RECT

    //Closing: dilation (+) erosion
    cvDilate(img,img,kern,1);
     cvSaveImage("/tmp/planetoimg_DILATE.tif",img);
    cvErode(img,img,kern,1);
     cvSaveImage("/tmp/planetoimg_CLOSED.tif",img);
    
   //7
   vox_plane_xy.image_to_plane_segment(img);
   vox_plane_xy.export_vox_centres(0).Write("/tmp/plane_voxcenters_after_fill.laser",0);
     
   //8
   //we need to find for every voxel in the initial set (also the empty voxels) if the projection to the plane is empty or not. If it is not empty (but ws before: add a voxel!)
    LaserPoints vox_planeorig_centres_ALL=vox_planeorig.export_vox_centres(1);
  
     //vox_planeorig_centres_ALL.Write("/tmp/orig_voxcenters_ALL.laser",0);
      
     
     //we now manually do the projection loop because we need to modify it
     Vector3D xv,yv;
     NormalPlane.PerpendicularVectors(xv, yv);
   
     //if some original data is provided: put it into a voxelfield as well
     LaserVoxel *vox_fillup;
      if (FillUpDataSegment!=NULL) vox_fillup=new LaserVoxel(*FillUpDataSegment,this->vox_length*1.5); //larger voxels to account for some inaccuracy
      uint filleduppointsnum=0;
   
    for(int k=0;k<vox_planeorig_centres_ALL.size();k++)
    {
      if (plorig.Distance(vox_planeorig_centres_ALL[k]) > 1*this->vox_length) continue;
      
         double dx = xv.DotProduct(vox_planeorig_centres_ALL[k]);  
         double dy = yv.DotProduct(vox_planeorig_centres_ALL[k]);  
	 //If at the position dx,dy,0 in the filled-up voxelfield we have some data, but not in the original plane_orig voxel, then we there add a points
	 //but also we need to be careful with the distance to the plane - > in the voxelfield we have the bounding box, so at 45° angle of a plane the hole cube will be filled
	 //thus: the distance to the original plane must be smaller than vox_length (filtered in the begin of the loop)
	 if (vox_plane_xy.number_of_points_in_voxel_at(dx,dy,0)!=0)
	   if (vox_planeorig.number_of_points_in_voxel_at(vox_planeorig_centres_ALL[k].X(),vox_planeorig_centres_ALL[k].Y(),vox_planeorig_centres_ALL[k].Z())==0)
	   {
	     //we first check: is also some fillup data here? If no: set a dummy from the original voxelfield at that point
	     bool has_fillupdata=0;
	     if (FillUpDataSegment!=NULL)
	     {
	       LaserPoints FillUp;
	       if (vox_fillup->number_of_points_in_voxel_at(vox_planeorig_centres_ALL[k].X(),vox_planeorig_centres_ALL[k].Y(),vox_planeorig_centres_ALL[k].Z(),&FillUp)!=0)
	       {
		 has_fillupdata=1;
		 //printf("found %d points in fill up data!\n",FillUp.size());
		 filleduppointsnum+=FillUp.size();
		 ls.AddPoints(FillUp);
	       }
		 
	     }
	     
	     if (!has_fillupdata)
	     {
	     LaserPoint projected(plorig.Project(vox_planeorig_centres_ALL[k]));
	     projected.SetColour(255,255,255);
	      ls.push_back(projected);
	     }
	     
	   }
   }
   
   //if in the end all points are to be projected onto the intial plane: do it here
   if (project_all_to_plane)
   {
  LaserPoints::iterator point;
    for (point=ls.begin(); point!=ls.end(); point++)
	  {
	    Position3D projected(plorig.Project(*point));
	    point->SetX(projected.GetX());
	    point->SetY(projected.GetY());
	    point->SetZ(projected.GetZ());
	  }
   }
  
  if (FillUpDataSegment!=NULL) 
  {delete vox_fillup;
   printf("Added %d points from fill up data!\n",filleduppointsnum);
  }
  
  cvReleaseStructuringElement( &kern);
  cvReleaseImage(&img);
  return 1;
  
}



void LaserVoxel::fill_gaps(LaserPoints &ls,double kernelsize_objectspace, LaserPoints *filledLS, LaserPoints *FillUpData, bool project_all_to_plane)
{
//process: we assume a segmented dataset. If not: exit(0);
//1: collect the different segment ids 
//2: iterate over each segment and call the method for filling gaps for the respective laserpoints
//3: since they might be too dense: filter them by putting into a temp. voxelstructure of voxel_length and only export the centers
 ::map<int, uint > segmentids; //first: number of points per voxel, second: occurence
 
 LaserPoints::iterator point;
    for (point=ls.begin(); point!=ls.end(); point++)
	  {
	    LaserPoint lp=point->LaserPointRef();
	    segmentids[lp.SegmentNumber()]++;
	  }
	  
  if (segmentids.size()==0) {printf("Error: Laserdata not segmented\n");exit(0);}
  else  printf("Found %d segments\n",segmentids.size());
	    


  for (::map<int, uint>::iterator it=segmentids.begin();it!=segmentids.end();it++)
  {
      printf("processing points with segment id: %d\n",it->first);
      
      
      LaserPoints segmentls; segmentls.Initialise();
      LaserPoints::iterator point;
      for (point=ls.begin(); point!=ls.end(); point++)
	    {
	      LaserPoint lp=point->LaserPointRef();
	      if(lp.SegmentNumber()==it->first) segmentls.push_back(lp);
	    }
      segmentls.Write("/tmp/onlysegment.laser",0);
      
      //LaserPoints *FillUpDataSegment=NULL;
      //IF we have FillUpData to fill up potential holes: cut out the data according to the segment (to save processing time)
      if (FillUpData!=NULL)
      {
	DataBoundsLaser boundssegment= segmentls.DataBounds();
	LaserPoints FillUpDataSegment;
	FillUpData->Select(FillUpDataSegment, boundssegment);
	fill_gaps_for_coplanar_points(segmentls, kernelsize_objectspace,&FillUpDataSegment,project_all_to_plane);
      }
      
      else fill_gaps_for_coplanar_points(segmentls, kernelsize_objectspace,NULL,project_all_to_plane);
      //now:  the segmentls are filled up. 
      //put them in a tmp voxelfield with the given length and only export then the vox. centres as final points. Collect those in the global
      //LaserPoints instance which is finally put into *this voxelfield and exported  via main.
      //before, add the segment number to the points
      for (point=segmentls.begin(); point!=segmentls.end(); point++)
	    {
	    point->SetSegmentNumber(it->first);
            point->Label(it->first);
	    }
      
      //LaserVoxel tmpvox(segmentls,this->vox_length*0.75);
      //filledLS->AddPoints(tmpvox.export_vox_centres(0));
      filledLS->AddPoints(segmentls);
      
  }
  return;
}

LaserPoints LaserVoxel::export_all()
{
 LaserPoints l;
  for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     if (Vox[i][j][k]!=NULL) 
      {
       //l=l+Vox[i][j][k]->LaserPointsReference(); //+ operator too expensive
	  LaserPoints::iterator point;
	  LaserPoints tmpl=Vox[i][j][k]->LaserPointsReference();
	  for (point=tmpl.begin(); point!=tmpl.end(); point++)
	  {
	    LaserPoint lp=point->LaserPointRef();
	    l.push_back(lp);
	  }
      }
  
  return l;
}
void LaserVoxel::export_all_ply(char * filename,LaserPoints l)
{
  FILE *data;
//printf("\nwriting voxel information to %s\n",filename);
if ((data=fopen(filename,"w")) == NULL)
		{
			printf("CAN NOT OPEN %s!\n", filename);
			return;
		}
//header
fprintf(data,"ply\nformat ascii 1.0\n"); 
fprintf(data,"element vertex %d\n",l.size());
fprintf(data,"property float x\nproperty float y\nproperty float z\n");
fprintf(data,"property uchar diffuse_red\nproperty uchar diffuse_green\nproperty uchar diffuse_blue\n");
fprintf(data,"end_header\n");

 LaserPoints::iterator point;
	      for (point=l.begin(); point!=l.end(); point++)
	      {
		LaserPoint lpl=point->LaserPointRef();
		fprintf(data,"%.5lf %.5lf %.5lf %d %d %d\n",lpl.X(),lpl.Y(),lpl.Z(),lpl.Red(),lpl.Green(),lpl.Blue());
	      }

fclose (data);
}
/*
ply
format ascii 1.0
element vertex 1090871
property float x
property float y
property float z
property float nx
property float ny
property float nz
property uchar diffuse_red
property uchar diffuse_green
property uchar diffuse_blue
end_header
-7.41108 2.75513 27.3666 0.710301 0.208068 -0.672443 167 178 199
*/


LaserPoints LaserVoxel::export_vox_centres(bool also_voids)
{
LaserPoints l,lvoids;
l.ReInitialise();
lvoids.ReInitialise();
uint num_voids=0;

  for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     {
       double X=min_X+(double) ((double) (i+0.5)* vox_length);
        double Y=min_Y+(double) ((double) (j+0.5)* vox_length);
        double Z=min_Z+(double) ((double) (k+0.5)* vox_length);
	
	if (Vox[i][j][k]!=NULL) 
	  {
	    //also approximate the color...
		//CHECK if we have set some segmentNumber. If yes, set it also for the vox centre. The problem is of course: we use one point (center) to represent the whole set of points in this voxel. So, if different segment Ids are present, we have a conflicet. Here: Just take the first valid one
	      int rsum=0, gsum=0, bsum=0, num=0;
	      LaserPoints::iterator point;
	      LaserPoints tmpl=Vox[i][j][k]->LaserPointsReference();
		int segmentID=0;
	      for (point=tmpl.begin(); point!=tmpl.end(); point++)
	      {
		LaserPoint lpl=point->LaserPointRef();
		rsum+=lpl.Red();
		gsum+=lpl.Green();
		bsum+=lpl.Blue();
		num++;

		if (lpl.SegmentNumber() > 0 && segmentID==0) segmentID=lpl.SegmentNumber();
	      }
	      
	     LaserPoint lp(X,Y,Z);
	    
	    lp.SetColour((int) rsum/num,(int) gsum/num, (int) bsum/num);
	    lp.SetSegmentNumber(segmentID);
	    
		

	    l.push_back(lp);
	      
	  }
	  else if (Vox[i][j][k]==NULL && also_voids) 
	  {
	    //printf("i=%d  j=%d   k=%d\n",i,j,k);
	    //printf("size of l: %d, to be added at X=%.2f Y=%.2f Z=%.2f\n",l.size(), X,Y,Z);
	    LaserPoint lpv(X,Y,Z,1,1);
	    
	    lpv.SetColour(255,255,255);
	    lvoids.push_back(lpv);
	   
	  }
     } //for
     
     
     if (also_voids){  
       //I do not know why, but a direct push_back or AddPoints to l will result in a Segmentation fault. So write to file and read again
     lvoids.Write("/tmp/voidpoints.laser",0);
      
     LaserPoints tmpvoids;
     tmpvoids.Read("/tmp/voidpoints.laser");
     printf("size of tmpvoids:%d\n", tmpvoids.size());
     l.AddPoints(tmpvoids);
     //for (int k=0;k<tmpvoids.size();k++) l.push_back(tmpvoids[k]);
     
     }
     
   return l;

}

 void LaserVoxel::export_vox_centres_text(char * filename)
{
FILE *data;
//printf("\nwriting voxel information to %s\n",filename);
if ((data=fopen(filename,"w")) == NULL)
		{
			printf("CAN NOT OPEN %s!\n", filename);
			return;
		}
//first row: number of voxels in x y z
fprintf(data,"%d %d %d\n",vox_num_X, vox_num_Y, vox_num_Z); 
//second row: offset X Y Z, voxellength 
fprintf(data,"%lf %lf %lf %lf\n",min_X, min_Y, min_Z, vox_length); 


/*
for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
{int sign=-1;
//test for 2.5D data: if we come from min we start with -100 (inside vol), when we found a point:0, then only 100 (outside vol)
     for (uint k=0;k<vox_num_Z;k++)
	{
	if (Vox[i][j][k]!=NULL && sign==-1) sign =0; //hit the surface
	fprintf(data,"%lf %d %d %d\n",(double) sign*100,i,j,k); //data outside vol: positive

	if (sign==0) sign=1;
	}
}
*/

for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
{int sign;
     for (uint k=0;k<vox_num_Z;k++)
	{
	if (Vox[i][j][k]!=NULL) sign =0; //hit the surface
	else sign=-1; //outside
	fprintf(data,"%lf %d %d %d\n",(double) sign*1,i,j,k); //data outside vol: positive

	}
}
fclose(data);

}

void LaserVoxel::export_vox_latice(ObjectPoints &objpts,LineTopologies &linetops)
{
//This is not optimized: now, evey voxel corner is written, so at adjacent voxels, they are written multiple times
int objoffset=0;
int linenumber = 0;  //only used for visualisation, so every voxel has an own number. A unique linenumber is not necessary (yet?)
LineTopology line;
/*
 		  ObjectPoint pt1 = ObjectPoint (p1.X(), p1.Y(),p1.Z(), ++objnumber_footp, 0, 0, 0, 0, 0, 0);
        	     ObjectPoint pt2 = ObjectPoint (p2.X(), p2.Y(),p2.Z(), ++objnumber_footp, 0, 0, 0, 0, 0, 0);
        	     line = LineTopology (linenumber_footp, 2, objnumber_footp - 1, objnumber_footp);
        	     linetops_footp.push_back(line);

        	     ObjectPoint pt3 = ObjectPoint (p3.X(), p3.Y(),p3.Z(), ++objnumber_footp, 0, 0, 0, 0, 0, 0);
        	     line = LineTopology (linenumber_footp, 2, objnumber_footp - 1, objnumber_footp);
        	     linetops_footp.push_back(line);

        	     ObjectPoint pt4 = ObjectPoint (p4.X(), p4.Y(),p4.Z(), ++objnumber_footp, 0, 0, 0, 0, 0, 0);
        	     line = LineTopology (linenumber_footp, 2, objnumber_footp - 1, objnumber_footp);
        	     linetops_footp.push_back(line);

        	     line = LineTopology (linenumber_footp, 2, objnumber_footp - 3, objnumber_footp);
        	     linetops_footp.push_back(line);

				 objpts_footp.push_back(pt1);objpts_footp.push_back(pt2);
				 objpts_footp.push_back(pt3);objpts_footp.push_back(pt4);
*/

  for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
	if (Vox[i][j][k]!=NULL)      
	{
         
       double X1=min_X+(double) ((double) (i+0)* vox_length);
        double Y1=min_Y+(double) ((double) (j+0)* vox_length);
        double Z1=min_Z+(double) ((double) (k+0)* vox_length);

        double X2=min_X+(double) ((double) (i+1)* vox_length);
        double Y2=min_Y+(double) ((double) (j+0)* vox_length);
        double Z2=min_Z+(double) ((double) (k+0)* vox_length);

	double X3=min_X+(double) ((double) (i+0)* vox_length);
        double Y3=min_Y+(double) ((double) (j+0)* vox_length);
        double Z3=min_Z+(double) ((double) (k+1)* vox_length);

 	double X4=min_X+(double) ((double) (i+1)* vox_length);
        double Y4=min_Y+(double) ((double) (j+0)* vox_length);
        double Z4=min_Z+(double) ((double) (k+1)* vox_length);

        double X5=min_X+(double) ((double) (i+0)* vox_length);
        double Y5=min_Y+(double) ((double) (j+1)* vox_length);
        double Z5=min_Z+(double) ((double) (k+0)* vox_length);

 	double X6=min_X+(double) ((double) (i+1)* vox_length);
        double Y6=min_Y+(double) ((double) (j+1)* vox_length);
        double Z6=min_Z+(double) ((double) (k+0)* vox_length);
	
	double X7=min_X+(double) ((double) (i+0)* vox_length);
        double Y7=min_Y+(double) ((double) (j+1)* vox_length);
        double Z7=min_Z+(double) ((double) (k+1)* vox_length);

	double X8=min_X+(double) ((double) (i+1)* vox_length);
        double Y8=min_Y+(double) ((double) (j+1)* vox_length);
        double Z8=min_Z+(double) ((double) (k+1)* vox_length);
	
	  ObjectPoint pt1 = ObjectPoint (X1, Y1,Z1, objoffset+1, 0, 0, 0, 0, 0, 0);objpts.push_back(pt1);

	  ObjectPoint pt2 = ObjectPoint (X2, Y2,Z2, objoffset+2, 0, 0, 0, 0, 0, 0);objpts.push_back(pt2);
	  
	  ObjectPoint pt3 = ObjectPoint (X3, Y3,Z3, objoffset+3, 0, 0, 0, 0, 0, 0);objpts.push_back(pt3);
	
	  ObjectPoint pt4 = ObjectPoint (X4, Y4,Z4, objoffset+4, 0, 0, 0, 0, 0, 0);objpts.push_back(pt4);

	  ObjectPoint pt5 = ObjectPoint (X5, Y5,Z5, objoffset+5, 0, 0, 0, 0, 0, 0);objpts.push_back(pt5);

	  ObjectPoint pt6 = ObjectPoint (X6, Y6,Z6, objoffset+6, 0, 0, 0, 0, 0, 0);objpts.push_back(pt6);

	  ObjectPoint pt7 = ObjectPoint (X7, Y7,Z7, objoffset+7, 0, 0, 0, 0, 0, 0);objpts.push_back(pt7);

	  ObjectPoint pt8 = ObjectPoint (X8, Y8,Z8, objoffset+8, 0, 0, 0, 0, 0, 0);objpts.push_back(pt8);

          line = LineTopology (linenumber, 2, objoffset+1, objoffset+2); linetops.push_back(line);
          line = LineTopology (linenumber, 2, objoffset+1, objoffset+3); linetops.push_back(line);
          line = LineTopology (linenumber, 2, objoffset+2, objoffset+4); linetops.push_back(line);
          line = LineTopology (linenumber, 2, objoffset+3, objoffset+4); linetops.push_back(line);
          line = LineTopology (linenumber, 2, objoffset+1, objoffset+5); linetops.push_back(line);
          line = LineTopology (linenumber, 2, objoffset+2, objoffset+6); linetops.push_back(line);
          line = LineTopology (linenumber, 2, objoffset+3, objoffset+7); linetops.push_back(line);
          line = LineTopology (linenumber, 2, objoffset+4, objoffset+8); linetops.push_back(line);
          line = LineTopology (linenumber, 2, objoffset+5, objoffset+6); linetops.push_back(line);
          line = LineTopology (linenumber, 2, objoffset+5, objoffset+7); linetops.push_back(line);
          line = LineTopology (linenumber, 2, objoffset+7, objoffset+8); linetops.push_back(line);
          line = LineTopology (linenumber, 2, objoffset+6, objoffset+8); linetops.push_back(line);

	    
	    linenumber++;
	    objoffset+=8;
	   
	  
	 
     } //for
     
     
}
