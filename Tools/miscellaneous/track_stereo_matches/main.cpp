#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <math.h>
#include "Point2D.h"
#include "LaserPoints.h"
#include <map>

#include "KNNFinder.h"

#define DIST_THRESH 0.1 //half a pixel distance threshold for the same point
using namespace std;
class MATCHED_POINT_C;

bool check_value_in_vec(vector<uint> &v, uint val, uint &index)
{
	for (uint i=0; i<v.size();i++)
		if (v[i]==val) {index=i;return 1;}

	return 0;
}

//THis struct just used for reading the data from file. Data as such then copied to MATCHED_POINT_C-vector
typedef struct STEREO_MATCH {
    uint id;
    Point2D LeftP;
    Point2D RightP;
    uint LeftImg;
    uint RightImg;
} STEREO_MATCH;


typedef struct MULTI_MATCH { //IN THE VECTOR the id will be the INDEX!!!
    uint id;
    vector<MATCHED_POINT_C *> matched_point_V;
    vector<uint> ImageVec;
    bool from_sequence; //means that thre is a sequence : img 1 -> img 2 -> img 3
    bool has_a_base_image; //means that at least one image is matched to at least 2 images, eg img 1 -> img 2 / img -> img 3
    //(both options are possible at the same time!)
} MULTI_MATCH;

class MATCHED_POINT_C : public LaserPoint //inherits LaserPoints, since then we can directly make use of all the KNN stuff, and also link to the partner efficiently
{
public:
  //CONSTRUCTORS
   MATCHED_POINT_C () : LaserPoint() {
    tags = NULL; attributes = NULL;
    Initialise();
  } ;
  
   MATCHED_POINT_C(double x, double y, double z) : LaserPoint(x, y, z)
  {
    tags = NULL; attributes = NULL;
    Initialise();
  }     
  
  uint id;
  uint image_id;
  MATCHED_POINT_C * partner;
  //MULTI_MATCH * part_of_multi_match;
  uint part_of_multi_match_id;
};

uint imageID_for_IMAGE(vector<string> & image_names_V, string nameThis) //checks if the nameTHis is alreadz in the vector. If yes: just return the index, if no: add it to the vector and return the index
{
  
  //for (uint i=0;i<image_names_V.size();i++) if (image_names_V[i]==nameThis) return i;
  for (uint i=0;i<image_names_V.size();i++) if (!strcmp(image_names_V[i].c_str(),nameThis.c_str())) return i;
  
  image_names_V.push_back(nameThis);

  return image_names_V.size()-1;
}

void add_point_to_stereomatches(vector <STEREO_MATCH> &stereo_match_V, uint id, double x, double y, uint imageID) //checks if this id is already in one stereomatch, if not: push a new one back (and add it as left point), if yes add it as RIGHTmg
{
  for (uint i=0;i<stereo_match_V.size();i++) if (stereo_match_V[i].id==id)
  {
    //add the point as new point as right point. if this already exists, we have an error (more than two points with this id)
    if (stereo_match_V[i].RightP.X()!=0) {printf("ERROR in implementation or in data: right point for match id %d already there. Note: in this file each match id must be unique, i.e. exactly there for 2 points!!!\n",id);exit(0);}
    Point2D p(x,y,0,0,0,0);
    stereo_match_V[i].RightP=p;
    stereo_match_V[i].RightImg=imageID;
    
    return;
  }
  
  //if we end up here it means that no stereo match with that id is existing ... add a new one and add this point as left image point, and initiatilye the right one as 0,0
     Point2D pl(x,y,0,0,0,0);
     Point2D pr(0,0,0,0,0,0);
    
     STEREO_MATCH st;
     st.id=id;
     st.LeftP=pl;
     st.RightP=pr;
     st.LeftImg=imageID;
     stereo_match_V.push_back(st);
}

void check_for_complete_stereomatches(vector <STEREO_MATCH> &stereo_match_V) //checks if all stereo matches contain exactly the two points (if there are more with one id, it is detected in the add_point_to_stereomatches function)
{
  for (uint i=0;i<stereo_match_V.size();i++) if (stereo_match_V[i].RightP.X()==0)  {printf("ERROR in implementation or in data: right point for match id %d not available!\n",stereo_match_V[i].id);exit(0);}
}

void dump_match(MATCHED_POINT_C &m, vector<string> &image_names_V, vector<MULTI_MATCH> &multi_match_V)
{
 m.Print();
 printf("ID:%d ... IMAGE ID:%d, image name:%s\n",m.id,m.image_id,image_names_V[m.image_id].c_str());
 printf("\tPARTNER:\n"); m.partner->Print(); printf("\tPARTNER ID:%d ... IMAGE ID:%d, image name:%s\n",m.partner->id,m.partner->image_id,image_names_V[m.partner->image_id].c_str());
 //(m.part_of_multi_match==NULL)?printf("is NOT part of a multimatch\n"):printf("is PART of a multi match:%d!\n",m.part_of_multi_match->id);
 (m.part_of_multi_match_id==1E8)?printf("This match is NOT part of a multimatch\n"):printf("This match is PART of a multi match:%d!\n",m.part_of_multi_match_id);
 printf("\n\n");
}

void dump_multimatch(MULTI_MATCH &m)
{
    
    printf("Multimatch dump START\n");
    printf("id:%d, number of points:%d, is_from_sequence:%d, has_a_base_image:%d\n",m.id,m.matched_point_V.size(),m.from_sequence,m.has_a_base_image);
   
    for (int i=0;i<m.matched_point_V.size();i++)
    {
      printf("ID of matching point: %d, is in image %d\n",m.matched_point_V[i]->id,m.matched_point_V[i]->image_id);
      //debug: double check image id
      if (m.matched_point_V[i]->image_id != m.ImageVec[i]) {printf("ERROR in implementation, inconsistent image numbering in multi match\n"); exit(0);}
    }
    //printf("TBD for final output to file: each image only once of course...\n");
    printf("Multimatch dump END\n\n");
}

void PrintUsage()
{
printf("track_stereo_matches: takes one textfile as input which contains stereomatches in the form imagename, id, col, row. The ids must be unique, i.e. each id only appers twice: left and right image. Output is a matchfile with the same syntax but where stereo matches are combined to multi-view matches");
  printf("\nUsage: \ttrack_stereo_matches textfile.txt\n\n]n");
} 

int main(int argc, char *argv[])
{


vector<STEREO_MATCH> stereo_match_V;stereo_match_V.clear();
vector<MULTI_MATCH> multi_match_V;multi_match_V.clear();
vector<string> image_names_V; image_names_V.clear(); 
vector<MATCHED_POINT_C> matched_point_V; matched_point_V.clear();


printf("\ntrack_stereo_matches\n\n");

                       
  if (argc!=2)
 {
    printf("Error: not all arguments given\n");
    PrintUsage();
    exit(0);
  }
 
//read the textfile with stereo matches
printf("\n Reading text file ...\n");
  FILE* stmatchF;
	  if ((stmatchF=fopen(argv[1],"r")) == NULL)
		{
			printf("CAN NOT OPEN the file!\n");
			return(0);
		}		

while(!feof(stmatchF))
		 {
			int id;
			double x,y;
			char imgname[100];
			
			int got=fscanf(stmatchF,"%[^,],%d,%lf,%lf\n",imgname, &id, &x, &y);	
			
			if (got!=4) {printf("line not read: (possibly empty last line?) %d %s  %d  %.4f  %.4f \n",got,imgname, id, x,y);}
			
			//printf("GOT=%d  name=%s  id=%d  x=%.4f  y=%.4f \n",got,imgname, id, x,y);
			
			//Add this image to the image vector and also get its ID
			uint imageid=imageID_for_IMAGE(image_names_V,imgname);
			//printf("this image %s has id %u\n",imgname,imageid);
			
			add_point_to_stereomatches(stereo_match_V,id,x,y,imageid);
			
		 }
check_for_complete_stereomatches(stereo_match_V);

printf("Read %d stereo matches\n",stereo_match_V.size());

//COPY the data from stereo_match_V to the class instances and remove the initial data
matched_point_V.resize(0);

//uint next_index=0;
for (uint i=0;i<stereo_match_V.size();i++)
{
 //each stereo match induces two matched points...
 MATCHED_POINT_C left(stereo_match_V[i].LeftP.X(),stereo_match_V[i].LeftP.Y(),0.0);
 MATCHED_POINT_C right(stereo_match_V[i].RightP.X(),stereo_match_V[i].RightP.Y(),0.0);
 
 left.id=stereo_match_V[i].id;
 right.id=stereo_match_V[i].id;
 
 left.image_id=stereo_match_V[i].LeftImg;
 right.image_id=stereo_match_V[i].RightImg;
 
 //left.part_of_multi_match=NULL;
 //right.part_of_multi_match=NULL;
 left.part_of_multi_match_id=1E8;
 right.part_of_multi_match_id=1E8;
 
 //add them to the vector
 matched_point_V.push_back(left);
 matched_point_V.push_back(right);
 
}

//add the pointers  within the structure...
uint next_index=0;
for (uint i=0;i<stereo_match_V.size();i++)
{
 matched_point_V[next_index].partner=&matched_point_V[next_index+1]; //the right (next_index+1) is the partner of the left and vice versa
 matched_point_V[next_index+1].partner=&matched_point_V[next_index];
 next_index+=2;
}

//REMOVE the stereo_match_V...
stereo_match_V.clear();

for (uint i=0;i<matched_point_V.size();i++) dump_match(matched_point_V[i],image_names_V,multi_match_V);


fclose(stmatchF);

//now we fill up the multi_match_V:
//idea: we put all  matched points in the current image into a LaserPoints instance and put them into a KNN tree (so just for the KNN tree, the orignal matched_point_V remains!)
//normally this could be done per image (only makes sense), but if we filter out per image we loose the index within the original set. Filtering per point in the next loop!

//we iterate over all images
//then we iterate over points in matched_point_V (just those in the current image of course) again and find duplicates/triplets etc, by looking for the clostest points in the knn tree (all points with distance < DIST_THRESH). 

//Duplicates (2), Triplets (3), etc (n), means that a particular point contributes to (n+1) stereo matches (if there is not at least a duplicate, it is only a stereo match -- not of interest for start of a new multimatch. It can however be of course the mate of another stereomatch which contributes to a multimatch)
//Therefore we collect all stereo match ids of the points contributing to the duplicate/triplet etc (plus the original one, so we have (n) ids of stereomatches)
//We check if at least one of those points is already part of a multimatch. IF YES: add all others of these corresponding ones INCLUDING partners to that match
//IF no: NEW MULTUIMATCH intstance with all points here! IF there are 2 matches at a point it is a SEQUENTIAL match, if there are more: it is a baseline match!

//put all points in a kd tree
LaserPoints LP_for_knn; LP_for_knn.ReInitialise();
for (uint i=0;i<matched_point_V.size();i++) LP_for_knn.push_back(matched_point_V[i]);
KNNFinder<LaserPoint> finder_ref(LP_for_knn, 2); //initiaize with 2 dimensions!!!
  printf("Put %d points into the KNN tree\n",LP_for_knn.size());
 
  
//ITERATE over all images

for (uint img=0;img<image_names_V.size();img++)
{
  printf("checking points in image %d (%s)\n",img,image_names_V[img].c_str());
 
  //ITERATE over the points in this image and find coincidencing points
  for (uint i=0;i<matched_point_V.size();i++)
    if (matched_point_V[i].image_id==img)
    {
     //if it is already assigned to a multimatch, continue (the partner is not imporant for this check)
     if (matched_point_V[i].part_of_multi_match_id!=1E8) continue;
      
      dump_match(matched_point_V[i],image_names_V,multi_match_V);
   
    Vector3D v_3(matched_point_V[i]);

     vector<uint> matching_points(0);
     for (int k=1;k<LP_for_knn.size();k++) //break the loop anyways when the distance becomes too big
	   {
	     int index_hit; double distance_hit;
	     finder_ref.FindIndexAndDistance (v_3, index_hit, distance_hit, k, DIST_THRESH);
	     //if the match is not in the same image ... take next one (by incidence a point in an other image can be quite close tho this one of course)
	     if (matched_point_V[index_hit].image_id!=img) continue;
	      //DEBUG
	       Vector3D v_3_test(matched_point_V[index_hit]);
	      
	      double distanceP=(v_3-v_3_test).Length();
	      
	     //if (distance_hit !=distanceP) {printf("ERROR in implementation: disatnce from KNN:%.4f, distance from computation %.4f\n",distance_hit,distanceP); exit(0);}
	      
	       if (distanceP<=DIST_THRESH) matching_points.push_back(index_hit);
		else break;
		
		 printf("k=%d,id:%d, distance from KNN:%.4f, distance from computation %.4f\n",k,matched_point_V[index_hit].id,distance_hit,distanceP);	      
		
	   }
     
    printf("==> THIS POINT matches with %d points in image %d\n\n",matching_points.size(),img);
    
    //we are now only interested in these match if for this point it appears at least twice (one is always the point itself-if there are no more, it is not overlapping with other matches)
    
    if (matching_points.size()<2) {printf("==>!NOT of interest to start a multimatch, or to add it\n");continue;}
    
    //iterate over all points now and check if they are part of a multi_match already
    MULTI_MATCH *multi=NULL;
    
    for (int ma=0;ma<matching_points.size();ma++)
      //if (matched_point_V[matching_points[ma]].part_of_multi_match!=NULL){multi=matched_point_V[matching_points[ma]].part_of_multi_match;break;}
      if (matched_point_V[matching_points[ma]].part_of_multi_match_id!=1E8){multi=&multi_match_V[matched_point_V[matching_points[ma]].part_of_multi_match_id];break;}
      
      if (multi==NULL)
      {printf("no multimatch so far --> add a new instance!\n"); 
	
	MULTI_MATCH tmp_multi;
	tmp_multi.matched_point_V.resize(0);
	tmp_multi.ImageVec.resize(0);
	tmp_multi.from_sequence=0;
	tmp_multi.has_a_base_image=0;
	//if there are more than 2 mathcing points - it has a base  image, else (exactly 2) it is from a sequential match (can be both, but so far we cannot check it, only later)
	if (matching_points.size()==2) tmp_multi.from_sequence=1;
	else if (matching_points.size()>2) tmp_multi.has_a_base_image=1;
	
	tmp_multi.id=multi_match_V.size();//size so far!
	
	
	//Iterate again over all the points, add them to the new multimatch instance (including the partners!)
	for (int ma=0;ma<matching_points.size();ma++)
	{
	  tmp_multi.matched_point_V.push_back(&matched_point_V[matching_points[ma]]);
	  tmp_multi.matched_point_V.push_back(matched_point_V[matching_points[ma]].partner);
	  tmp_multi.ImageVec.push_back(matched_point_V[matching_points[ma]].image_id);
	  tmp_multi.ImageVec.push_back(matched_point_V[matching_points[ma]].partner->image_id);
	}
	
	multi_match_V.push_back(tmp_multi);
	//iterate AGAIN over the points and add the pointer to the newly created multimatch instance
	/*
	for (int ma=0;ma<matching_points.size();ma++)
	{  
	  if (multi_match_V.size()!=0){
	  matched_point_V[matching_points[ma]].part_of_multi_match=&multi_match_V[multi_match_V.size()-1];
	  matched_point_V[matching_points[ma]].partner->part_of_multi_match=&multi_match_V[multi_match_V.size()-1];
	  }
	  else{
	  MULTI_MATCH *elem = multi_match_V.data();  //first element
	  matched_point_V[matching_points[ma]].part_of_multi_match=elem;//multi_match_V.data(); //-> data points to the first entry
	  matched_point_V[matching_points[ma]].partner->part_of_multi_match=elem;//multi_match_V.data();
	  }
	}
	*
	*/
	//iterate AGAIN over the points and add the ids of the multi match to the simple matehs
	for (int ma=0;ma<matching_points.size();ma++)
	{  
	  matched_point_V[matching_points[ma]].part_of_multi_match_id=tmp_multi.id;
	  matched_point_V[matching_points[ma]].partner->part_of_multi_match_id=tmp_multi.id;
	}
	printf("AFTER CREATION of NEW MULTIMATCH instance\n");
	dump_multimatch(tmp_multi);
	
	
      }//if a new multimatch needs to be added
      
      else  
	  
      
      {printf("multimatch to be extended!\n");
	//update the flags
        if (matching_points.size()==2) multi->from_sequence=1;
	else if (matching_points.size()>2) multi->has_a_base_image=1;
	
	//Iterate again over all the points, add them to the multimatch instance (including the partners!) IF THEY ARE NOT YET PART OF IT	
	for (int ma=0;ma<matching_points.size();ma++)
	{ 
	  //if (matched_point_V[matching_points[ma]].part_of_multi_match==NULL) 
	  if (matched_point_V[matching_points[ma]].part_of_multi_match_id==1E8) 
	  {
	    multi->matched_point_V.push_back(&matched_point_V[matching_points[ma]]);
	    multi->ImageVec.push_back(matched_point_V[matching_points[ma]].image_id);
	    //matched_point_V[matching_points[ma]].part_of_multi_match=multi;}
	   matched_point_V[matching_points[ma]].part_of_multi_match_id=multi->id;
	   
	   
	  }
	   
	   //if (matched_point_V[matching_points[ma]].partner->part_of_multi_match==NULL)
	   if (matched_point_V[matching_points[ma]].partner->part_of_multi_match_id==1E8)
	  { 
	    multi->matched_point_V.push_back(matched_point_V[matching_points[ma]].partner);
	    multi->ImageVec.push_back(matched_point_V[matching_points[ma]].partner->image_id);
	    //matched_point_V[matching_points[ma]].partner->part_of_multi_match=multi;
	    matched_point_V[matching_points[ma]].partner->part_of_multi_match_id=multi->id;
	  }
	}

	dump_multimatch(*multi);
      }
   //exit(0);

    
  }//iteration over images
  
  
}//END iteration over images

//write all initial matches which are not used in a multimatch 
printf("START - the following initial stereo matches ARE NOT used in multimatches\n");
for (uint i=0;i<matched_point_V.size();i++)
  if (matched_point_V[i].part_of_multi_match_id==1E8) {printf("%d\n",matched_point_V[i].id);i++;}
printf("END\n");
//write the final matches to three files: depending on whether it those are only sequence matches, only base line, or both

FILE* fseq;
FILE* fbase;
FILE* fseqbase;

if ((fseq=fopen("sequence_only_matches.txt","w")) == NULL)
		{
			printf("CAN NOT OPEN the file!\n");
			return(0);
		}
		
if ((fbase=fopen("baseline_only_matches.txt","w")) == NULL)
		{
			printf("CAN NOT OPEN the file!\n");
			return(0);
		}	
		
if ((fseqbase=fopen("seq_baseline_matches.txt","w")) == NULL)
		{
			printf("CAN NOT OPEN the file!\n");
			return(0);
		}	

//iterate over all multi matches. then itereate overall image numbers (from 0 to n) and check if the respective image is in the multimathc. By this measn: have always increasing vimage name, independent from internal processes, and each only once

for (int m=0;m<multi_match_V.size();m++)
{
  //iterate over the images
  for (uint img=0;img<image_names_V.size();img++)
  {
    //check if this image is part in the multimatch
    uint index;
    if (check_value_in_vec(multi_match_V[m].ImageVec,img,index))
    {
      //this image is present in the match
      printf("%s,%d,%.3f,%.3f\n",image_names_V[img].c_str(),multi_match_V[m].id,multi_match_V[m].matched_point_V[index]->X(),multi_match_V[m].matched_point_V[index]->Y());
      
      //depeningon the matching type write in one of the files
      if (multi_match_V[m].from_sequence && !multi_match_V[m].has_a_base_image)
       fprintf(fseq,"%s,%d,%.3f,%.3f\n",image_names_V[img].c_str(),multi_match_V[m].id,multi_match_V[m].matched_point_V[index]->X(),multi_match_V[m].matched_point_V[index]->Y());
      
      else if (!multi_match_V[m].from_sequence && multi_match_V[m].has_a_base_image)
	 fprintf(fbase,"%s,%d,%.3f,%.3f\n",image_names_V[img].c_str(),multi_match_V[m].id,multi_match_V[m].matched_point_V[index]->X(),multi_match_V[m].matched_point_V[index]->Y());
     
      else if (multi_match_V[m].from_sequence && multi_match_V[m].has_a_base_image)
	 fprintf(fseqbase,"%s,%d,%.3f,%.3f\n",image_names_V[img].c_str(),multi_match_V[m].id,multi_match_V[m].matched_point_V[index]->X(),multi_match_V[m].matched_point_V[index]->Y());
     
      else {printf("error in implementation .... this should not be reached\n");exit(0);}
    }
    
    
  }//images
  
  
  
}//multimatches

  
    fclose(fseq);
    fclose(fbase);
    fclose(fseqbase);
    
		
return EXIT_SUCCESS;
}
