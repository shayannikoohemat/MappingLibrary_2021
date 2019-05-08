#include <laservoxel.h>
uint label_at_Pt(LaserPoint & p1, ::map<uint, int> & image_label_R, ::map<uint, int>  & label_G, ::map<uint, int> & label_B)
{
      
		//iterate over the assigned label colors. If one fits: take that one
		for (::map<uint, int>::iterator it_R=image_label_R.begin();it_R!=image_label_R.end();it_R++)
		{	uint labelid=it_R->first;
			if (image_label_R[labelid]==p1.Red() && label_G[labelid]==p1.Green() && label_B[labelid]==p1.Blue()) {
				return labelid;
				//printf("FOUND LABEL with ID %d at row=%d col=%d\n",labelid,(int) pos.GetX(),(int) pos.GetY());
				}
		}

//else return error, because here ALL points will have a label!
printf("Error: found a point with no label assigned!"); exit(0);
}

//LaserVoxel::LaserVoxel(void)
//{
 
//}

LaserVoxel::LaserVoxel(LaserPoints &ls, double vox_l,bool verbose,bool assume_voxel_centres)
{
 vox_length=vox_l;
 

 //get the databounds from the ls
 DataBoundsLaser db=ls.DeriveDataBounds(0);
 min_X=db.Minimum().GetX();
 min_Y=db.Minimum().GetY();
 min_Z=db.Minimum().GetZ();
 
 //if the assume_voxel_centres switch is set we reduce all the min s by vox_l half! otherwise we would shift the data!
 if (assume_voxel_centres)
 {
   printf("reduce all the min s by vox_l half! otherwise we would shift the data!\n");
   
  min_X-=vox_l/2;
  min_Y-=vox_l/2;
  min_Z-=vox_l/2;
 }
 
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
 //also save them in an overall list
 original_points=new LaserPoints;
 
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
	    original_points->push_back(lp);
	    
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

uint LaserVoxel::index_for_gco(uint &i,uint &j,uint &k,uint &l,uint &num_labels) //given the index per X Y Z and label it returns the index in the onedimensionl vector data[num_voxels*num_labels]. Makes it easier to find pos. in the 3D lattice
{
uint offsetX=vox_num_Y*vox_num_Z*num_labels;
uint offsetY=vox_num_Z*num_labels;
uint offsetZ=num_labels;

return (i*offsetX + j*offsetY + k*offsetZ + l);

}

void LaserVoxel::ijkl_from_index_for_gco(uint &index, uint &i ,uint &j,uint &k,uint &l,uint &num_labels) //reverse fct from the above one   
{
uint offsetX=vox_num_Y*vox_num_Z*num_labels;
uint offsetY=vox_num_Z*num_labels;
uint offsetZ=num_labels;

l=((index%(offsetZ))%offsetY)%offsetX;
k=(((index-l)%(offsetY))%offsetX)/offsetZ;
j=((index-l-k*offsetZ)%(offsetX))/offsetY;
i=(index-l-k*offsetZ-j*offsetY)/offsetX;
  
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

int LaserVoxel::neighbors_at_ijk(uint i, uint j, uint k, uint n_size, bool assume_voxel_centres, LaserPoints &neig, double &X, double &Y, double &Z, bool only_2D)
{
  neig.ReInitialise();
  X=min_X+(double) ((double) (i+0.5)* vox_length);
  Y=min_Y+(double) ((double) (j+0.5)* vox_length);
  Z=min_Z+(double) ((double) (k+0.5)* vox_length);
  
  uint max_step_aside=(n_size -1)/2;
  
  //printf("max setp aside=%d\n",max_step_aside);
  
  uint found_p=0;
	 uint zero=0;
	
	   for (uint a=max(i-max_step_aside,zero);a<=i+max_step_aside;a++) //the indices may not be <0
	   {
	      if (a>=vox_num_X) continue;
	      
	      for (uint b=max(j-max_step_aside,zero);b<=j+max_step_aside;b++)
	      {
		if (b>=vox_num_Y) continue;
		
		if (!only_2D)
		{
		for (uint c=max(k-max_step_aside,zero);c<=k+max_step_aside;c++)
		  {if (c>=vox_num_Z) continue;
		      if (a==i && b==j && c==k) continue; //the point itself is not counted
			if (Vox[a][b][c]!=NULL)
			{
			  LaserPoints tmpl=Vox[a][b][c]->LaserPointsReference();

			    if (tmpl.size() !=1 && assume_voxel_centres) {printf("ERROR: assuming_voxel_centres, but more than one point in voxel\n");exit(0);}
			    else if (!assume_voxel_centres){printf("exporting more than one point per voxel not yet implemented\n");exit(0);}
			      
			      neig.push_back(tmpl[0].LaserPointRef());
			      //found_p+=Vox[a][b][c]->LaserPointsReference().size(); 	//count the number of original points
			      found_p++; //only count the number of filled voxels  
			}
		    }//c
		}
		else //only2D
		{
		  uint c=k;
		  if (a==i && b==j) continue; //the point itself is not counted
			if (Vox[a][b][c]!=NULL)
			{
			  LaserPoints tmpl=Vox[a][b][c]->LaserPointsReference();

			    if (tmpl.size() !=1 && assume_voxel_centres) {printf("ERROR: assuming_voxel_centres, but more than one point in voxel\n");exit(0);}
			    else if (!assume_voxel_centres){printf("exporting more than one point per voxel not yet implemented\n");exit(0);}
			      
			      neig.push_back(tmpl[0].LaserPointRef());
			      //found_p+=Vox[a][b][c]->LaserPointsReference().size(); 	//count the number of original points
			      found_p++; //only count the number of filled voxels  
			}
		}
		
	      }//b	   
	   }//a
  
  return found_p;
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

bool LaserVoxel::export_vox_centre_at(uint i, uint j, uint k, LaserPoint &lp,bool assume_voxel_centres)
{
  //looks at cell at the given index. if assume_voxel_centres it checks if there is a real point and returns that one (only implemeted so far)
  if(!assume_voxel_centres) {printf("ERROR: export_vox_centre_at for assume_voxel_centres=false not yet implemented\n");exit(0);}
  
  if (Vox[i][j][k]==NULL) {return 0;}
  
  LaserPoints tmpl=Vox[i][j][k]->LaserPointsReference();

  if (tmpl.size() !=1) {printf("ERROR: assuming_voxel_centres, but more than one point in voxel\n");exit(0);}
  
  lp=tmpl[0].LaserPointRef();
  return 1;
}

LaserPoints LaserVoxel::export_vox_centres(bool also_voids,bool assume_voxel_centres)
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
	      //int rsum=0, gsum=0, bsum=0, num=0;
	      int num=0;
	      
	      //also save the median of the reflectance
	      vector<uint> refl_vec, r_vec, g_vec, b_vec; refl_vec.resize(0);r_vec.resize(0);g_vec.resize(0);b_vec.resize(0);
	      LaserPoints::iterator point;
	      LaserPoints tmpl=Vox[i][j][k]->LaserPointsReference();
		int segmentID=0;
	  
	      for (point=tmpl.begin(); point!=tmpl.end(); point++)
	      {
		LaserPoint lpl=point->LaserPointRef();
		//rsum+=lpl.Red();
		//gsum+=lpl.Green();
		//bsum+=lpl.Blue();
		
		num++;
		refl_vec.push_back(lpl.Reflectance());
		r_vec.push_back(lpl.Red());
		g_vec.push_back(lpl.Green());
		b_vec.push_back(lpl.Blue());
		
		
		if (lpl.SegmentNumber() > 0 && segmentID==0) segmentID=lpl.SegmentNumber();
		
		if (assume_voxel_centres && num!=1) {printf("Error: assuming only to have voxel centres, but there are more in a cube\n");exit(0);}
	      }
	      
	     LaserPoint lp(X,Y,Z);
	    
	     if (assume_voxel_centres)
	     {
	       double distanceP=(lp-tmpl[0]).Length();
	       if (distanceP > 0.1) {printf("Error: assuming only to have voxel centres, but the actual point and the suppopsed centre are too far away!\n");exit(0);}
	     }
	    //lp.SetColour((int) rsum/num,(int) gsum/num, (int) bsum/num);
	    lp.SetSegmentNumber(segmentID);
	    uint pos=floor(num/2);
	    int med_refl=(int) refl_vec[pos]; 
	    int med_r=(int) r_vec[pos];
	    int med_g=(int) g_vec[pos];
	    int med_b=(int) b_vec[pos];
	    
		//printf("median refletance %d\n",med_refl);
	    lp.SetReflectance(med_refl);
	    lp.SetColour(med_r,med_g,med_b);
	    
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

//attention: labels=numClasses+1!!!
void LaserVoxel::fill_data_terms_from_supervised(int **data,uint & num_labels,double & mean, double &stddev,::map < uint, vector < double > > &responses_per_segment_TRAINING, ::map < uint, vector < double > > & responses_per_segment_VALIDATION, ::map < uint, vector < double > > &responses_per_segment_OTHERS)
{
  /*
  int max_energy=100;//100
  int min_energy_for_background_empty_vox=(int) (max_energy*0.5);//0.1 .... 0.5. 0.5?! to leave a litte chance that the label can change from background to meaning
  int energy_unknown=(int) (max_energy*0.66);//0.66 For voxel without classification so far, but labels with a meaning
  int energy_unknown_background_for_nonempty_vox=(int) (max_energy*0.99);//0.95?make it less likely that a previously classified voxel becomes background (however, this features is interesting to reduce noise in the point cloud)
  */
  
  //mean adaboost haiti: 0.61, stddev 0.27, rtrees: 0.17,0.22
  //double mean=0.61;//17;//61;
  //double stdddev=0.27;//22;//27; 
  
  //for adaboost, haiti the following is ok
  //double factor_empty_is_background=0.5*(mean+stdddev);
  //double factor_empty_has_meaning=mean+stdddev;
 //double factor_non_empty_is_backround=mean+stdddev;
  
  double factor_empty_is_background=0.5*(mean+stddev); //factor_empty_is_background=max(factor_empty_is_background,0.5);
  double factor_empty_has_meaning=mean+(stddev);
  double factor_non_empty_is_backround=mean+(4*stddev);factor_non_empty_is_backround=min(factor_non_empty_is_backround,0.99);
  
  int max_energy=1000;//100
  int empty_is_background=max_energy*factor_empty_is_background;
  int empty_has_meaning=max_energy*factor_empty_has_meaning;
  int non_empty_is_background=max_energy*factor_non_empty_is_backround;
  
  
  
  
 printf ("Defining data terms: max_energy:%d, empty_is_background=%d, empty_has_meaning=%d  non_empty_is_background=%d\n",max_energy,empty_is_background,empty_has_meaning,non_empty_is_background);
  
   for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     {//get the info from the LaserPoints in the voxel:
      //if in that voxel: is it from predicted raining, Validation or others? Get the correspongin responses (from the segment number)
      //if the voxel is empty: for the correspoing cell set all energy to a reasonalbe value (energy_unknown), but for the last label (background: some less energy (min energy?)
    
     
     bool is_empty=1;
      vector < double > responses_this_point;
      
      if (Vox[i][j][k]!=NULL) 
	{is_empty=0;
          LaserPoints tmpl=Vox[i][j][k]->LaserPointsReference();
	  if (tmpl.size()!=1) {printf("error: multiple points per voxel. not yet treated!!\n");exit(0);}
	  //get the responses... (depending on PulseCount and SegementNumber)
	  uint origin=tmpl[0].PulseCount();
	  uint segment_id=tmpl[0].SegmentNumber();
      
	  if (origin==1) {responses_this_point=responses_per_segment_TRAINING[segment_id];}//printf("is training, segment=%d\n",segment_id);}
	  else if (origin==2) {responses_this_point=responses_per_segment_VALIDATION[segment_id];}//printf("is validation, segment=%d\n",segment_id);}
	  else if (origin==3) {responses_this_point=responses_per_segment_OTHERS[segment_id];}//printf("is others, segment=%d\n",segment_id);}
	  
	  else {printf("Error: origin %d unknown!\n",origin);exit(0);}
	}
      
//TODO?!: define per voxel an individual energy for "unknown" (i.e. background). 
	  for (uint l=0;l<num_labels;l++)
	{
	    uint this_index=index_for_gco(i,j,k,l,num_labels);
	    
	    /* test ijkl from index! (or make a lookup table? no!!)
	     uint i_test,j_test,k_test,l_test;
	     ijkl_from_index_for_gco(this_index, i_test , j_test,k_test,l_test,num_labels);
	     
	     if (i_test!=i || j_test !=j || k_test!=k || l_test !=l) 
	     {printf("error in ijkl from index:index: %d\ni test: %d, i: %d\n j test: %d,  j: %d\n k test: %d k: %d\n l test: %d, l: %d\n\n",this_index,i_test,i,j_test,j,k_test,k,l_test,l);exit(0);}
	     
	     //else printf("ijkl from index:index: %d\ni test: %d, i: %d\n j test: %d,  j: %d\n k test: %d k: %d\n l test: %d, l: %d\n\n",this_index,i_test,i,j_test,j,k_test,k,l_test,l);
	     */
	    //cmpute the data term: for the corresponding label it is 
	    //(1-likelihood)*max_energy. (so the smaller the likelihood the larger the enery and vice versa and: scale to some reasonable integer)
	    //Attention: the last label is not in the response vector, it is the background. for that one the energy is some reasonale, like 2/3*max_energy==> energy_unknown
      
      //compute the index for this ijkl combination  and write the energy term , switch according to is_empty
      
	    if (is_empty && l!=(num_labels-1)) //!!no classification result given and not the background class
	      data[0][this_index]=empty_has_meaning;
	    else if (is_empty)//!!!no classification result given and!!! the background class==> min_energy, because this is the most likely one!
	      data[0][this_index]=empty_is_background;
	    else if (!is_empty && l!=(num_labels-1)) //!!!classification result given and not the background class:get it from the response info!
		data[0][this_index]=(1-responses_this_point[l]) * max_energy;
	    else if (!is_empty) ///!classification result, but background class!==> energy_unknown_background_for_nonempty_vox
	        data[0][this_index]=non_empty_is_background;
	}
     }

}

void LaserVoxel::fill_data_terms_for_unsupervised_segment(int **data,uint & num_labels,::map < uint, vector < int > > &energy_per_segment,int scale_to)
{
  int empty_is_background=scale_to/2;//scale_to*.1;
  int empty_has_meaning=pow((double) scale_to,3);//*.99;
  //int non_empty_is_background=scale_to*.99;
  
  
  
  printf("in fill_data_terms_for_unsupervised: num labels (incl. background):%d\n",num_labels);
 printf ("Defining data terms: max_energy:%d, empty_is_background=%d, empty_has_meaning=%d  \n",scale_to,empty_is_background,empty_has_meaning);
  
   for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     {
    
     
     bool is_empty=1;
      vector < int > energies_this_point;
      
      if (Vox[i][j][k]!=NULL) 
	{is_empty=0;
          LaserPoints tmpl=Vox[i][j][k]->LaserPointsReference();
	  if (tmpl.size()!=1) {printf("error: multiple points per voxel. not yet treated!!\n");exit(0);}
	  //get the responses... (depending on PulseCount and SegementNumber)
	  uint segment_id=tmpl[0].SegmentNumber();
	 //printf("segment_id=%d .... ",segment_id);
	 
	  energies_this_point=energy_per_segment[segment_id];
	  
	  if (energies_this_point.size()!=num_labels) {is_empty=1; /*printf("warning: segment %d has no data?!\n",segment_id);*/}
	  
	  
	}
      
//TODO?!: define per voxel an individual energy for "unknown" (i.e. background). 
	  for (uint l=0;l<num_labels;l++)
	{
	    uint this_index=index_for_gco(i,j,k,l,num_labels);
	    
	    /* test ijkl from index! (or make a lookup table? no!!)
	     uint i_test,j_test,k_test,l_test;
	     ijkl_from_index_for_gco(this_index, i_test , j_test,k_test,l_test,num_labels);
	     
	     if (i_test!=i || j_test !=j || k_test!=k || l_test !=l) 
	     {printf("error in ijkl from index:index: %d\ni test: %d, i: %d\n j test: %d,  j: %d\n k test: %d k: %d\n l test: %d, l: %d\n\n",this_index,i_test,i,j_test,j,k_test,k,l_test,l);exit(0);}
	     
	     //else printf("ijkl from index:index: %d\ni test: %d, i: %d\n j test: %d,  j: %d\n k test: %d k: %d\n l test: %d, l: %d\n\n",this_index,i_test,i,j_test,j,k_test,k,l_test,l);
	     
	    */
	    
      //compute the index for this ijkl combination  and write the energy term , switch according to is_empty
  
   
	    if (is_empty && l!=(num_labels-1)) //!!no energies given and not the background class
	      data[0][this_index]=empty_has_meaning;
	    else if (is_empty && l==(num_labels-1))//!!!no energies given and!!! the background class==> min_energy, because this is the most likely one!
	      data[0][this_index]=empty_is_background;
	    else if (!is_empty) //!!!energies given ==> in contrast to supervised we already account for the background class, so no special treatment here!!!
		data[0][this_index]=energies_this_point[l];
	    
	    //if (!is_empty) printf("...label %d got energy %d\n",l,data[0][this_index]);
	    /*
	    else if (!is_empty && l!=(num_labels-1)) //!!!energies given and and not the background class:get it from the response info!
		data[0][this_index]=energies_this_point[l];
	    else if (!is_empty && l==(num_labels-1)) ///energies given, but background class!==> energy_unknown_background_for_nonempty_vox
	        data[0][this_index]=non_empty_is_background;
	    */
	}
     }

  
}

void LaserVoxel::fill_data_terms_for_unsupervised_voxel(int **data,uint & num_labels,::map < uint, vector < int > > &energy_per_voxel,int scale_to)
{
  int empty_is_background=scale_to/2;//scale_to*.1;
  int empty_has_meaning=pow((double) scale_to,3);//*.99;
  //int non_empty_is_background=scale_to*.99;
  
  
  
  printf("in fill_data_terms_for_unsupervised: num labels (incl. background):%d\n",num_labels);
 printf ("Defining data terms: max_energy:%d, empty_is_background=%d, empty_has_meaning=%d  \n",scale_to,empty_is_background,empty_has_meaning);
  
   for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     {
    
     
     bool is_empty=1;
      vector < int > energies_this_point;
      
      if (Vox[i][j][k]!=NULL) 
	{is_empty=0;
          LaserPoints tmpl=Vox[i][j][k]->LaserPointsReference();
	  if (tmpl.size()!=1) {printf("error: multiple points per voxel. not yet treated!!\n");exit(0);}
	  //in main we added the index as Comonent Number
	  
	  uint voxel_index=tmpl[0].Attribute(ComponentNumberTag);
	 //printf("segment_id=%d .... ",segment_id);
	 
	  energies_this_point=energy_per_voxel[voxel_index];
	  
	  if (energies_this_point.size()!=num_labels) {is_empty=1; /*printf("warning: segment %d has no data?!\n",segment_id);*/}
	  
	  
	}
      
//TODO?!: define per voxel an individual energy for "unknown" (i.e. background). 
	  for (uint l=0;l<num_labels;l++)
	{
	    uint this_index=index_for_gco(i,j,k,l,num_labels);
	    
	    /* test ijkl from index! (or make a lookup table? no!!)
	     uint i_test,j_test,k_test,l_test;
	     ijkl_from_index_for_gco(this_index, i_test , j_test,k_test,l_test,num_labels);
	     
	     if (i_test!=i || j_test !=j || k_test!=k || l_test !=l) 
	     {printf("error in ijkl from index:index: %d\ni test: %d, i: %d\n j test: %d,  j: %d\n k test: %d k: %d\n l test: %d, l: %d\n\n",this_index,i_test,i,j_test,j,k_test,k,l_test,l);exit(0);}
	     
	     //else printf("ijkl from index:index: %d\ni test: %d, i: %d\n j test: %d,  j: %d\n k test: %d k: %d\n l test: %d, l: %d\n\n",this_index,i_test,i,j_test,j,k_test,k,l_test,l);
	     
	    */
	    
      //compute the index for this ijkl combination  and write the energy term , switch according to is_empty
  
   
	    if (is_empty && l!=(num_labels-1)) //!!no energies given and not the background class
	      data[0][this_index]=empty_has_meaning;
	    else if (is_empty && l==(num_labels-1))//!!!no energies given and!!! the background class==> min_energy, because this is the most likely one!
	      data[0][this_index]=empty_is_background;
	    else if (!is_empty) //!!!energies given ==> in contrast to supervised we already account for the background class, so no special treatment here!!!
		data[0][this_index]=energies_this_point[l];
	    
	    //if (!is_empty) printf("...label %d got energy %d\n",l,data[0][this_index]);
	    /*
	    else if (!is_empty && l!=(num_labels-1)) //!!!energies given and and not the background class:get it from the response info!
		data[0][this_index]=energies_this_point[l];
	    else if (!is_empty && l==(num_labels-1)) ///energies given, but background class!==> energy_unknown_background_for_nonempty_vox
	        data[0][this_index]=non_empty_is_background;
	    */
	}
     }

  
}
void LaserVoxel::add_a_LaserPoint_to_vox(uint i, uint j, uint k, LaserPoint &lp)
{
  original_points->push_back(lp);  
   //get the pointer to that very last point
	  LaserPoints::iterator point;
	  LaserPoint lpref;
	  for (point=original_points->end()-2; point!=original_points->end(); point++) lpref=point->LaserPointRef();
	  
	  if (Vox[i][j][k]==NULL) Vox[i][j][k]=new LaserPoints;
	    
	    Vox[i][j][k]->push_back(lpref);
  
}

void LaserVoxel::update_labels(int **result,uint & num_labels,::map<uint, int> & label_R, ::map<uint, int>  & label_G, ::map<uint, int> & label_B)
{
  //we have 3 different cases w.r.t emptiness of a voxel. 
	//1.It can be that an empty voxel (no class before, only background) now gets a real class
        //2. the other way around: a real class becomes background 
	//3. a real class remains the same or becomes another real class
	
	//in case 1 we need to add a laser point as voxel centre and add the color (whether it is from training, validation et. we cannot tell, so leave the PulseCount empty
	//in case 2 we remove the pointer
	//in case 3 we just change the label (i.e. the color)
	
	//make a small statistics on the cases
	
 uint new_points_with_meaning=0;
 uint remove_points_with_meaning=0;
 uint label_changes=0;
 uint label_remains=0;
 uint background_remains=0;
 
  uint offsetX=vox_num_Y*vox_num_Z;
  uint offsetY=vox_num_Z;
  
  LaserPoints lp_new_points_with_meaning;lp_new_points_with_meaning.ReInitialise();
  LaserPoints lp_new_points_background;lp_new_points_background.ReInitialise();

  printf("Attention: adding of new points (when background gets a meaning) not tested!!!!\n");
  
  for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     {
	uint index_=(i*offsetX + j*offsetY + k ); //index in the array
	uint label_result=result[0][index_];
	bool is_now_background=(label_result==(num_labels-1))?1:0;
        
   
	//case 1: an empty voxel has a meaning now
	if (Vox[i][j][k]==NULL && !is_now_background)
	{
	  new_points_with_meaning++;
	  //add the centre as a physical point... and set the color. Also set the pointer to this voxel!!! dont forget
	  double X=min_X+(double) ((double) (i+0.5)* vox_length);
          double Y=min_Y+(double) ((double) (j+0.5)* vox_length);
          double Z=min_Z+(double) ((double) (k+0.5)* vox_length);
	  
	  
	  LaserPoint lp(X,Y,Z);lp.SetColour(label_R[label_result],label_G[label_result],label_B[label_result]);
	  original_points->push_back(lp);  
	   
	  //get the pointer to that very last point
	  LaserPoints::iterator point;
	  LaserPoint lpref;
	  for (point=original_points->end()-2; point!=original_points->end(); point++) lpref=point->LaserPointRef();
	  Vox[i][j][k]=new LaserPoints;
	    
	    Vox[i][j][k]->push_back(lpref);
	    
	    //also update a debug laserfile whichis written to disk
	    lp_new_points_with_meaning.push_back(lpref);
	  
	}
	//case 2: a former voxel with meaning becomes background
         else  if (Vox[i][j][k]!=NULL && is_now_background) 
	{
          remove_points_with_meaning++;
	
	 lp_new_points_background.push_back(Vox[i][j][k]->LaserPointsReference()[0]);
	 
	 Vox[i][j][k]=NULL;
	  
	}
	//case 3: a former voxel with meaning still has a meaning
         else  if (Vox[i][j][k]!=NULL && !is_now_background) 
	 {
	    LaserPoints tmpl=Vox[i][j][k]->LaserPointsReference();
	     if (tmpl.size()!=1) {printf("error: multiple points per voxel. not yet treated!!\n");exit(0);}
	    uint label_from_color= label_at_Pt(tmpl[0], label_R, label_G, label_B);
	    //compre the labels and change if needed. update stats.
	    if (label_from_color==label_result) 
	    {
	      label_remains++;
	      
	    }
	    
	    else
	    {
	     label_changes++;
	     //if (label_result!=num_labels-2) printf("is now label %u\n",label_result); //debug
	     //TBD: change the color. Attention with the pointer
	    //LaserPoints tmpl=Vox[i][j][k]->LaserPointsReference();
	     //tmpl[0].SetColour(label_R[label_result],label_G[label_result],label_B[label_result]);
	     Vox[i][j][k]->LaserPointsReference()[0].SetColour(label_R[label_result],label_G[label_result],label_B[label_result]);
	    }
	   
	 }
	 //case 4: background remains
	 else if (Vox[i][j][k]==NULL && is_now_background)
	{//only increase the counter
	  background_remains++;
	}
	
	else
	{//only for debug: should not occur.
	  printf("Error: should not reach here!\n");
	  exit(0);
	}
      
	
     }
     
     printf("Statistics after optimization:\n %u points have same label as before\n %u points have a different label\n %u points changed from background to a meaning\n %u had a meaning before, but are now background\n %u remain background\n",label_remains,label_changes,new_points_with_meaning,remove_points_with_meaning, background_remains);
     
     if (label_remains+label_changes+new_points_with_meaning+remove_points_with_meaning+background_remains!=vox_num_X*vox_num_Y*vox_num_Z) {printf(
    "Error: sum is incorrect\n"); exit(0);}
    if (lp_new_points_background.size()) lp_new_points_background.Write("/tmp/points_that_became_background.laser",0);
    if (lp_new_points_with_meaning.size()) lp_new_points_with_meaning.Write("/tmp/added_new_points_with_meaning.laser",0);

}

void LaserVoxel::append_to_outputfiles(LaserPoints & out_training, LaserPoints & out_validation, LaserPoints &out_others)
{
 //do the same as the normal export function but append the points to the correct file, according to PulseCount. 
 //But: newly added points are not considered, because they do not have the PulseCount, and removed points will miss
 uint new_points_here=0;
 for (uint i=0;i<vox_num_X;i++)
   for (uint j=0; j<vox_num_Y;j++)
     for (uint k=0;k<vox_num_Z;k++)
     if (Vox[i][j][k]!=NULL) 
      {
       //l=l+Vox[i][j][k]->LaserPointsReference(); //+ operator too expensive
	  LaserPoints::iterator point;
	  LaserPoints tmpl=Vox[i][j][k]->LaserPointsReference();
	  uint origin=tmpl[0].PulseCount();
      
	  if (origin==1) {out_training.push_back(tmpl[0]);}
	  else if (origin==2) {out_validation.push_back(tmpl[0]);}//printf("is validation, segment=%d\n",segment_id);}
	  else if (origin==3) {out_others.push_back(tmpl[0]);}//printf("is others, segment=%d\n",segment_id);}
	  
	  else {new_points_here++;/*printf("Found a newly added point (origin = 0)!\n");*/}
	  
      }
 printf("Check: %u points are added (from background to a meaning, but not assigned to one of the types (training/validation/others)\n",new_points_here);
  
}


void LaserVoxel::write_world_file(char *tfw_filename, double offset_for_tfw_X, double offset_for_tfw_Y)
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
  double X=min_X-(double) ((double) (0.5)* vox_length);
  double Y=min_Y+(double) ((double) (vox_num_Y-0.5)* vox_length);
  
  FILE *tfw=fopen(tfw_filename,"w");
 fprintf(tfw,"%f\n0.0\n0.0\n%f\n%.3f\n%.3f\n",vox_length,-1*vox_length,offset_for_tfw_X+X,offset_for_tfw_Y+ Y);
 fclose(tfw);
}