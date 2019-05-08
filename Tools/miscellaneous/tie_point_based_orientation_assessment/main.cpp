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
#include <map>

#include "DataBounds3D.h"
#include "KNNFinder.h"

using namespace std;

//this program takes
//-the prepared ref data (see ../tie_point_based_orientation_assessment_reference_preparation): LS point cloud, file with normals per segment
//-the ls file with the tie points

//aim:
//compute a statistics of errors/distances between tie points and corresponding planes in the reference
//the tie points might be too sparse in order to do a plane segmentation  - so we cannot rely on that
//also we should not use point-to-point distance because of the different area quantifications/resolutions.
//procedure:
//0) find a global offset, i.e. first point in the reference or so, to reduce all absolute cooridnates
//1) crop tie points to the area defined by the area of the reference
//2) load the normals (as planes) (shorten the COGs1)
//3) for each tie point:find the corresponding plane:
  //-there are two conditions to be fulfilled: 
    //a: the point is within the enclosing polygon of the ref.plane (rotate the plane and point to the hz plane, use enclosingrectance, see ../tie_point_based_orientation_assessment_reference_preparation
    //b: the point-to-plane distance is below a threshold
  //since in theory more than one plane can satisfy those conditions for a point we need to visit all candidate planes.
  //to avoid an expensive brute force search we do the following:
//A) preparation: translate/rotate all planes to the origin (individual transformation per plane) once, store the enclosing rectangles and the transformation parameters per plane
//B) put all reference points in to KNN tree
// C)Iterate over all tie points
  //find all points in the reference point cloud which are closer than the dist thresh (may be a bit bigger). For each of those points determine the corresponding segment: make a unique list of candidate segments (the point to plane distance must also be smaller than the thresh)
  //for each candidate segment: transform the tiepoint into the xy plane using the parameters of that plane and check if it is inside, if yes: store as candidate with the distance as variable
  //from all (possible) candidate planes take the one with the smallest dist. 
    
using namespace std;


bool mean_stddev_vector (vector<double> &vec, double &mean, double &stddev)
{
  if (vec.size()<2) return false;
  
  mean=0;stddev=0;
 //first loop sum
 for (int v=0;v<vec.size();v++)
   mean+=vec[v];
 mean/=vec.size();
  
 //second loop: add quadratic differences
 double stddev_=0;
 for (int v=0;v<vec.size();v++)
   stddev_+=(vec[v]-mean)*(vec[v]-mean);
 
 stddev=sqrt(stddev_/(vec.size()-1));
 
 return true;
}

bool test_tie_point_surrounded_by_refpoints_2D(LaserPoint &tie, LaserPoints &refs) //just do it in the plane projecteion to make it easier
{
 //compute vectoes from t to all refs, compute the angle in the hz plane and make sure that all 4 quadrants are covered
 const double PI=3.1416;
 vector<double> theta_v(0);
 //printf("tie x=%.3f tie y=%.3f\n",tie.GetX(),tie.GetY());
 for (int i=0;i<refs.size();i++)
 {
   if (fabs(refs[i].GetZ())>1) {/*printf("offset from plane too big, skip it: dz tie=%.3d, dz ref=%.3f\n",fabs(refs[i].GetZ()));*/ continue;}
   
   double th=atan2(refs[i].GetY()-tie.GetY(),refs[i].GetX()-tie.GetX());
   
    //printf("ref X=%.3f, ref Y=%.3f \t\t Th=%.3f\n",refs[i].GetX(),refs[i].GetY(),th);
  
   theta_v.push_back(th);
 }
 
 bool in_q1=0, in_q2=0, in_q3=0, in_q4=0;
 
 for (int i=0;i<theta_v.size();i++)
 {
   //printf("theta_v=%.3f\n",theta_v[i]);
   if (theta_v[i]>=-PI && theta_v[i]<-PI/2) in_q1=1;
   else if (theta_v[i]>=-PI/2 && theta_v[i]<0) in_q2=1;
   else if (theta_v[i]>=0 && theta_v[i]<PI/2) in_q3=1;
   else if (theta_v[i]>=(PI/2) && theta_v[i]<=PI) in_q4=1;
   else {printf("ERROR in test_tie_point_surrounded_by_refpoints_2D...\n");exit(0);}
   
   if (in_q1 && in_q2 && in_q3 && in_q4) return 1; 
 } 
  
  return 0;
   
}

void PrintUsage()
{
printf("tie_point_based_orientation_assessment: computing statistics from tiepoint - to ref plane distances");
  printf("\nUsage: \ttie_point_based_orientation_assessment -reference <laserfile> -reference_normals <txtfile> (both files output from tie_point_based_orientation_assessment_reference_preparation) -tiepoints <laserfile> -average_point_distance_reference <m>: used to determine the snapping distance for ref points. for TLS around .1 max, for ALS .3 -thresh_pointplanedist <m> max distance to be considered\n");
} 

int main(int argc, char *argv[])
{

LaserPoints referenceLS, tiepointLS;
//!TODO: put thresh_horizontal_point_pointdist as command line (eg as average_point_distance_reference)

double thresh_horizontal_point_pointdist; //This is actually a function of point density of the reference: we need to make sure that the tie point is projected on the plane PLUS it is close to a ref. point in that plane. Imagine a plane with a hole, eg window. points in that whole must not be used for the assessment becasue they are not in plane!!!
//also add a test which ensures that a valid tie point is surrounded by ref points (no ref points just at one side)

double thresh_pointplanedist; //max dist to the fitting plane


double scale_for_errorvectors=1000;

::map<uint,Position3D> planeCOG_per_segment; //stores in the first field the number of the segment, and then the Plane parameters
::map<uint,Vector3D> planeNormal_per_segment; //stores in the first field the number of the segment, and then the Plane parameters
::map<uint,Rotation3D> rotation_to_hz_per_segment; //stores the rotatoin matrix per segment (plane) used to rotate parallel to hz plane. the offset is the centre of gravity as stored in the plane
::map<uint,ObjectPoints> encl_rect_points_per_segment; //the encl. rectangle (points) of segement, rotated/translated to origin, hz plane
::map<uint,LineTopology> encl_rect_topo_per_segment;

::map<uint,uint> tie_point_to_segment_assignment; //in first the tie point number is stored, in second the corresponding segment number
::map<uint,double> tie_point_dist_to_plane; //save the dist to the corresponding plane. Easier then to do the statistics

::map<uint,vector<double> > segment_to_distances; //store per segment the distances the respective tie points have in a vector. from there compute mean/stddev etc...

printf("tie_point_based_orientation_assessment\n");



 InlineArguments *args = new InlineArguments(argc, argv);
 
   
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-reference") || !args->Contains("-reference_normals") || !args->Contains("-tiepoints") || !args->Contains("-average_point_distance_reference") || !args->Contains("-thresh_pointplanedist"))
 {
    printf("Error: not all arguments given\n");
    PrintUsage();
    exit(0);
  }
 
//read the reference file
printf("\n Reading reference point cloud ...\n");
referenceLS.ReInitialise();
	  if (!referenceLS.Read(args->String("-reference"),1)) {
		  fprintf(stderr, "Error reading file %s\n", args->String("-reference"));
		  exit(0);
		}
		
//read the tiepoint file
printf("\n Reading tie point cloud ...\n");
tiepointLS.ReInitialise();
	  if (!tiepointLS.Read(args->String("-tiepoints"),1)) {
		  fprintf(stderr, "Error reading file %s\n", args->String("-tiepoints"));
		  exit(0);
		}
		
thresh_horizontal_point_pointdist=args->Double("-average_point_distance_reference",0); //default will not be used since it is a mandatory option
thresh_pointplanedist=args->Double("-thresh_pointplanedist",0); //default will not be used since it is a mandatory option

printf("parameters: thresh_horizontal_point_pointdist: %.2f, thresh_pointplanedist: %.2f\n", thresh_horizontal_point_pointdist, thresh_pointplanedist);


//to reduce the risk of numerical problems we shorten the coordinates: the first poit will be substracted from all, i.e. the first is then 0,0,0
const Position3D globaloffset=tiepointLS[0];


for (int k=0;k< tiepointLS.size();k++)
  tiepointLS[k]-=globaloffset;

for (int k=0;k< referenceLS.size();k++)
  referenceLS[k]-=globaloffset;

//crop the tie points to the area defined by the reference
//PROBLEM: the REduceData method which uses the databounds to crop the tiepoints also looks at the attributes. To avoid a modification of the mapping lib we here simply put just the ref.points (without attributes) into a temp Laserpoints instance
LaserPoints reference_TEMP; reference_TEMP.ReInitialise();
for (int k=0;k< referenceLS.size();k++) reference_TEMP.push_back(LaserPoint(referenceLS[k].GetX(),referenceLS[k].GetY(),referenceLS[k].GetZ()));

DataBoundsLaser bound=reference_TEMP.DeriveDataBounds(0);
  //increase the box a bit

  bound.SetMaximumX(bound.Maximum().GetX()+2);
  bound.SetMaximumY(bound.Maximum().GetY()+2);
  bound.SetMaximumZ(bound.Maximum().GetZ()+2);
  bound.SetMinimumX(bound.Minimum().GetX()-2);
  bound.SetMinimumY(bound.Minimum().GetY()-2);
  bound.SetMinimumZ(bound.Minimum().GetZ()-2);
  
  
  tiepointLS.ReduceData(bound);
  reference_TEMP.ReInitialise();
  //debug
  tiepointLS.Write("tmp_tiepoints_cropped_localsystem.laser",1);
  referenceLS.Write("tmp_referencpoints_localsystem.laser",1);
  
//open the normals/COG file and put the planes into the map, do not forget to remove the global offset
FILE *normsF;
  if ((normsF=fopen(args->String("-reference_normals"),"r")) == NULL)
		{
			printf("CAN NOT OPEN the normals file!\n");
			return(0);
		}
printf("Plane parameters\n");
uint num_planes=0;
while(!feof(normsF))
		 {
			int segnum;
			double COGX, COGY, COGZ, normX, normY,normZ;
			fscanf(normsF,"%d  %lf %lf %lf %lf %lf %lf",&segnum, &COGX, &COGY, &COGZ, &normX, &normY,&normZ);	
			
			//put the planes into the map instance
			Position3D COG_reduced(COGX-globaloffset.GetX(),COGY-globaloffset.GetY(),COGZ-globaloffset.GetZ());
			
			planeNormal_per_segment[segnum]=Vector3D(normX,normY,normZ);
			planeCOG_per_segment[segnum]=COG_reduced;
			
			//printf("%d   %.3f   %.3f   %.3f   %.3f   %.3f   %.3f\n",segnum, COG_reduced.GetX(), COG_reduced.GetY(), COG_reduced.GetZ(), normX, normY,normZ);
			
			num_planes++;
		 }
fclose (normsF);		
printf("Read %u planes\n",num_planes);


//Set up the knnfinder for the reference
KNNFinder<LaserPoint> finder_ref(referenceLS);

//rotate each plane into the hz plane/origin and save relevant parameters
for (::map<uint, Vector3D>::iterator it_=planeNormal_per_segment.begin();it_!=planeNormal_per_segment.end();it_++)
{
  //printf("Processing segment number %u\n",it_->first);
  //extract the points into a LaserPoints instance
  LaserPoints thissegment_points=referenceLS.SelectTagValue (SegmentNumberTag, it_->first);
	//printf("...this segment has %d points\n",thissegment_points.size());

  Vector3D PlNormal=it_->second;
  Position3D PlCOG=planeCOG_per_segment[it_->first];
  
 //rotation matrix, see ..._preparation
  Vector3D rotaxis=PlNormal.VectorProduct(Vector3D(0,0,1));
  //printf("rotaxis: %.2f %.2f %.2f\n",rotaxis.X(),rotaxis.Y(),rotaxis.Z());
  if (rotaxis.Length()>1E-3) rotaxis=rotaxis.Normalize();
  double rotangle=acos(PlNormal.DotProduct(Vector3D(0,0,1)));
  Rotation3D rotmat(rotaxis,(-1)*rotangle);
  
  //first move, than rotate all the points
  //printf("COG: %.2f %.2f %.2f\n",PlCOG.GetX(),PlCOG.GetY(),PlCOG.GetZ());
  //printf("Normal: %.2f %.2f %.2f\n",PlNormal.X(),PlNormal.Y(),PlNormal.Z());
  
  for(uint k=0;k<thissegment_points.size();k++) 
	{
	  thissegment_points[k]-=PlCOG;
	  (thissegment_points)[k]=rotmat.Rotate((thissegment_points)[k]);
	}
 //thissegment_points.Write("tmp_thisseg.laser",1);  
 
   ObjectPoints points;
   LineTopology topo;

  thissegment_points.VerifyTIN();

   thissegment_points.DeriveDataBounds(1);
   thissegment_points.EnclosingRectangle(1,points,topo);
 
  //in the end put the info to the maps
  rotation_to_hz_per_segment[it_->first]=rotmat;
  encl_rect_points_per_segment[it_->first]=points;
  encl_rect_topo_per_segment[it_->first]=topo;

}

//now: for each tie point find the corresponding plane (if there is any)
//ultmate aim is to populate ::map<uint,uint> tie_point_to_segment_assignment;  and ::map<uint,double> tie_point_dist_to_plane; 
LaserPoints debug_list_of_taken_tiepoints;debug_list_of_taken_tiepoints.ReInitialise();
LaserPoints debug_list_of_skipped_candidate_tiepoints;debug_list_of_skipped_candidate_tiepoints.ReInitialise();
printf("select tie points which are assigned to a reference plane\n");


double slantthres_sq=thresh_horizontal_point_pointdist*thresh_horizontal_point_pointdist+thresh_pointplanedist*thresh_pointplanedist;
//thrshod for slant distance for neigbh ref point, used to ensure that the tie point is not at hte edge of a refpoint plane (clustered points))
LaserPoints ref_points_local_here; //help for that

    for (int i=0; i<tiepointLS.size(); i++)
	  {
	   Vector3D t_orig(tiepointLS[i]);
	   
	   //printf("this tiepoint: %.2f  %.2f  %.2f\n",t.X(),t.Y(),t.Z());
	   //we assume that the ref point cloud is dense enough to find good tie points in the area.
	   //find all the points in the reference (then in the end the corresponding segments) which are closer than thresh_pointplanedist to the tie point
	   //PointNumberList segment_candidates; //stores all the potential segment ids of planes which are close enough to this tie point
	    
	   //printf("DOUBLE CHECK finder call here - can be it is not correcdt, see track_stereo_matches");
	   for (int k=1;;k++) //break the loop when the distance becomes too big
	   {
	     int index_hit; double distance_hit;
	     finder_ref.FindIndexAndDistance (t_orig, index_hit, distance_hit, k);
	     
	     //distance hit is the slant distance from the closest point to the tie point in quwestion. Now compute the hz compoennt by uswing also the plane!
	     
	     //distance_hit=sqrt(distance_hit); //the finder returns the square
	      //printf("k:%d\n",k);
	     //printf("cooresponding ref.point: %.2f  %.2f  %.2f\n",referenceLS[index_hit].GetX(),referenceLS[index_hit].GetY(),referenceLS[index_hit].GetZ());
	     
	     //we really need to find a ref point close to it, otherwise the risk is to have a tie point eg in a window: it is part of the surrounding plane, but no ref point is around it
	     //if (distance_hit > (thresh_pointpointdist)) {debug_list_of_skipped_candidate_tiepoints.push_back(tiepointLS[i]); break;} //bail out 
	     //add the segment number of the corresponding reference point to the list
	     //printf("tiepoint number: %d, k=%d, index_hit=%d, distance_hit=%.2f ..... ",i,k,index_hit,distance_hit);
	     //printf("corresponding segment number: %d\n",segmenttmp);
	     
	     
	      int segnum=(int) referenceLS[index_hit].SegmentNumber();
	      Plane plseg=Plane(planeCOG_per_segment[segnum],planeNormal_per_segment[segnum]);
	      double dist_tie_to_plane=plseg.Distance(t_orig); //signed distance!
	      
	      //compute the orthogonal dist of the closest refpoint to the plane. we need that to correct the hz distance later
	      double dist_ref_to_plane=plseg.Distance(referenceLS[index_hit]);
	      
	      double dist_to_plane_diff=dist_tie_to_plane-dist_ref_to_plane; //diff (signed)
	      
	      //compute the horizontal distance from the normal distance (orthogonal) and the slant dist. if this is > thresh_horizontal_point_pointdist skip it!!!
	      //distance_hit is already square!
	      double hz_dist_sq=distance_hit-(dist_to_plane_diff*dist_to_plane_diff);
	      if (hz_dist_sq<0) {printf("error in implementation: hz < 0:%.3f, slant distance is %.3f, dist tie to plane:%.3f, dist ref to plane:%.3f. dist diff=%.3d \n",hz_dist_sq, sqrt(distance_hit), dist_tie_to_plane, dist_ref_to_plane,dist_to_plane_diff); exit(0);}
	      
	      if ((sqrt(hz_dist_sq) > (thresh_horizontal_point_pointdist)) || (abs(dist_tie_to_plane)>thresh_pointplanedist)) {debug_list_of_skipped_candidate_tiepoints.push_back(tiepointLS[i]); break;} //bail out 
	      
	       Vector3D t=t_orig-planeCOG_per_segment[segnum];
	       t=rotation_to_hz_per_segment[segnum].Rotate(t);
	    
	       //put t again as a tmp LAserpoint
	       LaserPoint t_l(t);
	       int is_in=t_l.InsidePolygon(encl_rect_points_per_segment[segnum],encl_rect_topo_per_segment[segnum]);
		//LaserPoint::InsidePolygon (const ObjectPoints2D &polygon_points, const PointNumberList &topology, bool skip_polygon_check=false) const 	 
	       if (is_in ==0) {debug_list_of_skipped_candidate_tiepoints.push_back(tiepointLS[i]);continue;}//go to the  next closest one

	      //now: select the n=4 next closest points and see if the tie point is in between them, ie to avoid that it is at the border 
	      //The problem with this approach is: if the gap in the segment is smaller than the allowed distance, still we find well distributed points around it, so it is important to make the thresholds not larger than necessary
	      //transform those points to the plane and write into a vecgtor of laserpoints
	      ref_points_local_here.ReInitialise();
	      
	     
	      //printf("before l loop\n");
	      
	      for (int l=1;l<=4;l++)
	      {
		finder_ref.FindIndexAndDistance (t_orig, index_hit, distance_hit, k+l);
		
		
		//if this point is on another segment  skip it
		//if (referenceLS[index_hit].SegmentNumber() != segnum) break;
		//if the slant distance is already larger than allowed, it will be skipped anyways (use only the slant distance because it might be on a different plane already, in turn be a bit more toeralnt)Atthention, all now in square!
		 
		 if (distance_hit > slantthres_sq) break;//once the next closest point is farer away than allowed, also the other ones are out of reach
		
		 //if (sqrt(distance_hit) > 0.3) break;
		 
	         //transform to hz plane
	        Vector3D this_refpoint=referenceLS[index_hit];
		this_refpoint-=planeCOG_per_segment[segnum];
		this_refpoint=rotation_to_hz_per_segment[segnum].Rotate(this_refpoint);
		ref_points_local_here.push_back(LaserPoint(this_refpoint));
		
		
	      }
	      
	      //printf("after l loop\n");
	      //check if the tie point is surrounded. if not: skip this point!
	      if (ref_points_local_here.size() < 4) {debug_list_of_skipped_candidate_tiepoints.push_back(tiepointLS[i]);continue;} //if there are less than 4 points, it cannot be well embraced

//debug_ write to disk
/*
	      t_l.SegmentNumber()=3;
	      LaserPoints t_ls;t_ls.ReInitialise();t_ls.push_back(t_l);
	      t_ls.Write("/tmp/t_l.laser",1);
	      ref_points_local_here.Write("/tmp/ref_points_local_here.laser",1);
*/	      
		//printf("before test tie function\n");
	      if (!test_tie_point_surrounded_by_refpoints_2D(t_l,ref_points_local_here)) {debug_list_of_skipped_candidate_tiepoints.push_back(tiepointLS[i]);continue;}
		
		//printf("after test tie function\n");
	      debug_list_of_taken_tiepoints.push_back(tiepointLS[i]);
	      tie_point_to_segment_assignment[i]=segnum;
	      
	      
	      tie_point_dist_to_plane[i]=dist_tie_to_plane;
	      
	      //also put to the segment_candidates
	      segment_to_distances[segnum].push_back(dist_tie_to_plane);
	      
	      break;
	   }
	    
	  //printf("this tiepoint has as potential segment candidates:\n");
	  //for (int p=0;p<segment_candidates.size();p++) printf("%d\n",(int) segment_candidates[p].Number());
	   
	  //now for each segment: test the distance to the plane -> is it also smaller than the threshold? --> no, this test is not necessary, already made sure above
	  //if yes: project the tiepoint into the horizontal using the same parameters as used before for this segment and check if it falls into the enclosing rectangle
	
	    
	    int percentage=(int) (100*i/tiepointLS.size());
	    //if ((percentage % 1)==0) 
	    printf("%d percent done\r",percentage);
	  }

	  debug_list_of_taken_tiepoints.Write("debug_list_of_taken_tiepoints.laser",1);
	  debug_list_of_skipped_candidate_tiepoints.Write("debug_list_of_skipped_candidate_tiepoints.laser",1);
	  
	  
	  //now do some statistics and visualisation
	  //add vectors: objpts/top files, pointing from the tiepoint to the refrence, scaled by dist and a unique scale
	  //iterate over the map with the tiepoint to segm tie_point_to_segment_assignment
	  
	  FILE* signed_dist_to_plane_per_tie;
	  if ((signed_dist_to_plane_per_tie=fopen("signed_distances_to_plane_per_tiepoint.pts","w")) == NULL)
		{
			printf("CAN NOT OPEN distances_to_plane.dat!\n");
			return(0);
		}
		
	  FILE* abs_dist_to_plane_per_tie;
	  if ((abs_dist_to_plane_per_tie=fopen("absolute_distances_to_plane_per_tiepoint.pts","w")) == NULL)
		{
			printf("CAN NOT OPEN distances_to_plane.dat!\n");
			return(0);
		}
		
	  ObjectPoint pt1, pt2;
	  ObjectPoints vector_points;
	  LineTopology line;
	  LineTopologies vector_topos;
	  int objnumber = 0;
	  int linenumber = 1;
	  
	  fprintf(signed_dist_to_plane_per_tie,"pos global X Y Z, local X Y Z, signed distance\n");
	   fprintf(abs_dist_to_plane_per_tie,"pos global X Y Z, local X Y Z, abs distance\n");
	 
	   for (::map<uint, uint>::iterator it_=tie_point_to_segment_assignment.begin();it_!=tie_point_to_segment_assignment.end();it_++)
	  {
	    int tiepointnum=it_->first;
	    int segnum=it_->second;
	    double dist_to_plane=tie_point_dist_to_plane[tiepointnum];
	    
	    if (abs(dist_to_plane) > thresh_pointplanedist) {printf("BUG in code..., dist to plane larger than threshold: %.3f\n",dist_to_plane);exit;}
	    
	    Vector3D norm=planeNormal_per_segment[segnum];
	    
	   //the vector points from the tie point towards the reference plane 
	    pt1 = ObjectPoint (tiepointLS[tiepointnum].GetX(), tiepointLS[tiepointnum].GetY(), tiepointLS[tiepointnum].GetZ(), ++objnumber, 0, 0, 0, 0, 0, 0);
           		
	    //pt1+=globaloffset;
	    vector_points.push_back(pt1);
	
	    pt2 = ObjectPoint (tiepointLS[tiepointnum].GetX()+norm.X()*dist_to_plane*scale_for_errorvectors, tiepointLS[tiepointnum].GetY()+norm.Y()*dist_to_plane*scale_for_errorvectors, tiepointLS[tiepointnum].GetZ()+norm.Z()*dist_to_plane*scale_for_errorvectors, ++objnumber, 0, 0, 0, 0, 0, 0);
           		
	    //pt2+=globaloffset;
	    vector_points.push_back(pt2);
	
           line = LineTopology (linenumber++, 1, objnumber - 1, objnumber);
           vector_topos.push_back(line);
	   
	   //also add the signed distance to a XYZ dist file, for visualisation eg in CloudCompare and for the histogram (gnuplot)
	   fprintf(signed_dist_to_plane_per_tie,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",tiepointLS[tiepointnum].GetX()+globaloffset.GetX(),tiepointLS[tiepointnum].GetY()+globaloffset.GetY(),tiepointLS[tiepointnum].GetZ()+globaloffset.GetZ(), tiepointLS[tiepointnum].GetX(), tiepointLS[tiepointnum].GetY(), tiepointLS[tiepointnum].GetZ(),dist_to_plane);
	   fprintf(abs_dist_to_plane_per_tie,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",tiepointLS[tiepointnum].GetX()+globaloffset.GetX(),tiepointLS[tiepointnum].GetY()+globaloffset.GetY(),tiepointLS[tiepointnum].GetZ()+globaloffset.GetZ(),tiepointLS[tiepointnum].GetX(), tiepointLS[tiepointnum].GetY(), tiepointLS[tiepointnum].GetZ(),abs(dist_to_plane));
	
	  }
	  
	  vector_points.Write("errorsPERTIE_scaled.objpts");
	  vector_topos.Write("errorsPERTIE_scaled.top");
	  
	  //do similar per segment: compute the mean distance and check if it is significant using the stddev
	  vector_points.resize(0);
	  vector_topos.resize(0);
	  //also write relevant data into a txt file
	  FILE* csv_f;
	  if ((csv_f=fopen("distances_all.csv","w")) == NULL)
		{
			printf("CAN NOT OPEN distances.csv!\n");
			return(0);
		}
		
	   FILE* abs_dist_to_plane_only_f;
	  if ((abs_dist_to_plane_only_f=fopen("absolute_distances_to_plane.dat","w")) == NULL)
		{
			printf("CAN NOT OPEN distances_to_plane.dat!\n");
			return(0);
		}
		
	  FILE* signed_dist_to_plane_only_f;
	  if ((signed_dist_to_plane_only_f=fopen("signed_distances_to_plane.dat","w")) == NULL)
		{
			printf("CAN NOT OPEN distances_to_plane.dat!\n");
			return(0);
		}
	
	FILE* hz_only_f;//for Gnuplot
	  if ((hz_only_f=fopen("hz_dist_only.dat","w")) == NULL)
		{
			printf("CAN NOT OPEN hz_dist_only!\n");
			return(0);
		}
	//the same for vertical
	FILE* vertical_only_f;//for Gnuplot
	  if ((vertical_only_f=fopen("vertical_dist_only.dat","w")) == NULL)
		{
			printf("CAN NOT OPEN vertical_dist_only!\n");
			return(0);
		}
	
	  fprintf(csv_f,"segnum,number of points assigned,location of Segment X, location of Segment Y, mean of orthogonal distance to plane,stddev,mean of dZ projection,stddev,mean of hz projection,stddev, signed horizontal projection\n");
	      
		
	  //Store the min X, because we will place a reference scale into the plot. This will be left of the actual candlestics
	  double min_X=1e6;
	   for (::map<uint, vector <double> >::iterator it_=segment_to_distances.begin();it_!=segment_to_distances.end();it_++)
	   {
	     uint segnum=it_->first;
	     if (it_->second.size()<2) continue;
	     
	     Vector3D norm=planeNormal_per_segment[segnum];
	     //for (int i=0;i<it_->second.size();i++)
	     //printf("seg num: %d, distance: %.2f\n",segnum,it_->second[i]);
	     
	     
	     double mean, stddev;
	     mean_stddev_vector(it_->second,mean,stddev);
	     
	     printf("seg num: %d,  number of points assigned: %d, mean of orthogonal distance to plane %.2f, stddev %.2f ...\n",segnum, it_->second.size(),mean, stddev);
	     
	     //compute the dZ and the projection in the plane and also compute the mean and stddev of that
	     vector<double> dz_vec(0), hz_vec(0);
	     for (int i=0;i<it_->second.size();i++) 	
	     {
	       double dist=it_->second[i];
	       dz_vec.push_back(dist*norm.Z());
	       hz_vec.push_back(sqrt(dist*norm.X()*dist*norm.X()+dist*norm.Y()*dist*norm.Y()));
	     }
	     
	     double mean_dz, mean_hz, stddev_dz,stddev_hz,mean_ortho, dummy;//mean ortho is to be able to add a sign to the mean hz distanc (useful for close range: inside or outside the bulding?)
	     mean_stddev_vector(dz_vec,mean_dz,stddev_dz);
	     mean_stddev_vector(hz_vec,mean_hz,stddev_hz);
	     mean_stddev_vector(it_->second,mean_ortho,dummy);
	     
	     double mean_hz_signed=mean_hz; if (mean_ortho<0) mean_hz_signed*=-1;
	     
	     //for the gnuplot input file we use the sigma
	     //in gnuplot we print the xy position of the segment (COG) and indicate the mean distance as a box and the sigma as sticks (candlestick)
	    //the distances/confidence radii will be scaled using the predifined scale
	     //if we have a vertical plane (norm.Z is zero), it doese not make sense to show the dZ distance, it is zero because of that and will influence the statistics
	     //the same holds for hz planes (norm z is one) and hz value
	     
	     double X_=planeCOG_per_segment[segnum].GetX();
	     double Y_=planeCOG_per_segment[segnum].GetY();
	     if (min_X > X_) min_X=X_;
	     
	     //only print the mean distance as input for the histogram gnuplot thing
	      fprintf(abs_dist_to_plane_only_f,"%.3f\n",abs(mean));
	      
	       fprintf(signed_dist_to_plane_only_f,"%.3f\n",mean);
	      
	     if (abs(norm.Z()) > 0.1 && abs(norm.Z()) <0.9)
	     {
	     
	      printf("\tmean of dZ projection %.2f, stddev %.2f ...\n",mean_dz, stddev_dz);
	      printf("\tmean of hz projection %.2f, stddev %.2f ...\n",mean_hz, stddev_hz);
	    
	      fprintf(csv_f,"%d,%d,%.1f, %.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",segnum, it_->second.size(),planeCOG_per_segment[segnum].GetX(), planeCOG_per_segment[segnum].GetY(),mean, stddev, mean_dz, stddev_dz,mean_hz, stddev_hz,mean_hz_signed);
	      
	      
	      //fprintf(hz_only_f,"%.1f\t %.1f \t %.3f \t %.3f\n",planeCOG_per_segment[segnum].GetX(),planeCOG_per_segment[segnum].GetY(),mean_hz, stddev_hz);
	     //X, y-mean/2*scale, lower bound of confidenceintervall (y-(mean/2+2*sig)*scale), upper bound of confidenceintervall(y+(mean/2+2*sig)*scale), y+mean/2*scale, y)
	     //hz we interpret as an error diameter
	      //also add mean hz!!! as last col for the 3D plot
	      fprintf(hz_only_f,"%.0f\t %.0f\t %.0f\t %.0f\t %.0f\t %.0f\t %.3f\t  %.3f\n",X_,Y_-((mean_hz/2)*scale_for_errorvectors),Y_-((mean_hz/2+stddev_hz)*scale_for_errorvectors),Y_+((mean_hz/2+stddev_hz)*scale_for_errorvectors),Y_+((mean_hz/2)*scale_for_errorvectors),Y_,mean_hz, mean_hz_signed);
	      
	      //for dz we need to distinguish: is mean negative or positive???
	      //in the last col just add mean_dz for the 3d plot
	      if (mean_dz<0) //attention with the sign!
		fprintf(vertical_only_f,"%.0f\t %.0f\t %.0f\t %.0f\t %.0f\t %.0f\t %.3f\n",X_,Y_+(mean_dz*scale_for_errorvectors),Y_+((mean_dz-stddev_dz)*scale_for_errorvectors),Y_+stddev_dz*scale_for_errorvectors,Y_,Y_,mean_dz);
	      else 
		fprintf(vertical_only_f,"%.0f\t %.0f\t %.0f\t %.0f\t %.0f\t %.0f\t %.3f\n",X_,Y_,Y_-(stddev_dz*scale_for_errorvectors),Y_+(mean_dz+stddev_dz)*scale_for_errorvectors,Y_+mean_dz*scale_for_errorvectors,Y_,mean_dz);
	     }
	     
	     else if (abs(norm.Z()) <= 0.1)
	     {
	       printf("\tvertical plane .... dZ not useful\n");
	      printf("\tmean of hz projection %.2f, stddev %.2f ...\n",mean_hz, stddev_hz);
	    
	     fprintf(csv_f,"%d,%d,%.1f, %.1f,%.3f,%.3f,xx,xx,%.3f,%.3f,%.3f\n",segnum, it_->second.size(),planeCOG_per_segment[segnum].GetX(), planeCOG_per_segment[segnum].GetY(),mean, stddev,mean_hz, stddev_hz,mean_hz_signed);
	     
	      //fprintf(hz_only_f,"%.1f\t %.1f \t %.3f \t %.3f\n",planeCOG_per_segment[segnum].GetX(),planeCOG_per_segment[segnum].GetY(),mean_hz, stddev_hz);
	      fprintf(hz_only_f,"%.0f\t %.0f\t %.0f\t %.0f\t %.0f\t %.0f\t %.3f\t  %.3f\n",X_,Y_-((mean_hz/2)*scale_for_errorvectors),Y_-((mean_hz/2+stddev_hz)*scale_for_errorvectors),Y_+((mean_hz/2+stddev_hz)*scale_for_errorvectors),Y_+((mean_hz/2)*scale_for_errorvectors),Y_,mean_hz,mean_hz_signed);
	      
	     //DEBUG: write the whole line as well
	     //fprintf(csv_f,"XXX %d,%d,%.1f, %.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",segnum, it_->second.size(),planeCOG_per_segment[segnum].GetX(), planeCOG_per_segment[segnum].GetY(),mean, stddev, mean_dz, stddev_dz,mean_hz, stddev_hz);
	    
	     }
	     else if (abs(norm.Z()) >= 0.9)
	     {
	        printf("\tmean of dZ projection %.2f, stddev %.2f ...\n",mean_dz, stddev_dz);
	      printf("\thorizontal plane .... dhZ not useful\n");
	    
	      fprintf(csv_f,"%d,%d,%.1f, %.1f,%.3f,%.3f,%.3f,%.3f,xx,xx,xx\n",segnum, it_->second.size(),planeCOG_per_segment[segnum].GetX(), planeCOG_per_segment[segnum].GetY(),mean, stddev,mean_dz, stddev_dz);
	      
	      //for dz we need to distinguish: is mean negative or positive???
	      if (mean_dz<0) //attention with the sign!
		fprintf(vertical_only_f,"%.0f\t %.0f\t %.0f\t %.0f\t %.0f\t %.0f\t %.3f\n",X_,Y_+(mean_dz*scale_for_errorvectors),Y_+((mean_dz-stddev_dz)*scale_for_errorvectors),Y_+stddev_dz*scale_for_errorvectors,Y_,Y_,mean_dz);
	      else 
		fprintf(vertical_only_f,"%.0f\t %.0f\t %.0f\t %.0f\t %.0f\t %.0f\t %.3f\n",X_,Y_,Y_-(stddev_dz*scale_for_errorvectors),Y_+(mean_dz+stddev_dz)*scale_for_errorvectors,Y_+mean_dz*scale_for_errorvectors,Y_,mean_dz);
	
	      
	      //DEBUG: write the whole line as well
	     //fprintf(csv_f,"XXX %d,%d,%.1f, %.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",segnum, it_->second.size(),planeCOG_per_segment[segnum].GetX(), planeCOG_per_segment[segnum].GetY(),mean, stddev, mean_dz, stddev_dz,mean_hz, stddev_hz);
	     
	     }
	      //first visualisatoin: vector from the centre point of the segment along the normal. length (mean) is scaled by the actual length and the predefined scale. 
	      
	    pt1 = ObjectPoint (planeCOG_per_segment[segnum].GetX(), planeCOG_per_segment[segnum].GetY(), planeCOG_per_segment[segnum].GetZ(), ++objnumber, 0, 0, 0, 0, 0, 0);
           		
	    //pt1+=globaloffset;
	    vector_points.push_back(pt1);
	
	    pt2 = ObjectPoint (planeCOG_per_segment[segnum].GetX()+norm.X()*mean*scale_for_errorvectors, planeCOG_per_segment[segnum].GetY()+norm.Y()*mean*scale_for_errorvectors, planeCOG_per_segment[segnum].GetZ()+norm.Z()*mean*scale_for_errorvectors, ++objnumber, 0, 0, 0, 0, 0, 0);
           		
	    //pt2+=globaloffset;
	    vector_points.push_back(pt2);
	
           line = LineTopology (linenumber++, 1, objnumber - 1, objnumber);
           vector_topos.push_back(line);
	     //if (abs(mean) < (2*stddev)) {printf(" .... NOT significant\n"); continue;}
	     //else printf(" .... significant!\n");
	   }
	   
	   vector_points.Write("errorsPERsegment_scaled.objpts");
	  vector_topos.Write("errorsPERsegment_scaled.top");
	  
	  FILE* ref_scale_f;//for Gnuplot: add a scale left to the actual data
	  if ((ref_scale_f=fopen("dist_reference.dat","w")) == NULL)
		{
			printf("CAN NOT OPEN dist_reference.dat!\n");
			return(0);
		}
		
	  fprintf(ref_scale_f,"%.0f\t %.0f\t %.0f\t %.0f\t %.0f\n",min_X-50,-0.025*scale_for_errorvectors,-0.025*scale_for_errorvectors,0.025*scale_for_errorvectors,0.025*scale_for_errorvectors);
	      
		
	  
	  fclose(csv_f);
	  fclose(hz_only_f);
	  fclose(ref_scale_f);
	  fclose(abs_dist_to_plane_only_f);
	  fclose(signed_dist_to_plane_only_f);
	  fclose(signed_dist_to_plane_per_tie);
	  fclose(abs_dist_to_plane_per_tie);
return EXIT_SUCCESS;
}
