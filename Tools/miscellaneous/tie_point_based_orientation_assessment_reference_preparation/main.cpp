
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
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"
#include "cxcore.h"
#include "KNNFinder.h"

using namespace std;

//Helpfunction which computes the covmatrix (structural tensor) from an input LaserPoints cloud and a refpoint (eg centre of gravity) and computes eigenvalue related goemetric measures like curvature, scatter etc)
bool eigen_attributes(LaserPoints &lp, Position3D &centreP, double &scatter, double &linearity, double &planarity, double &curvature,double &entropy)
{
  //setup the matrix for the calccov
	CvMat** srcMatrix = new CvMat*[lp.size()+1]; //one more, because the centre needs to be added
	
	for (int k=0;k<=lp.size();k++)  	
		{srcMatrix[k]=cvCreateMat(1,3,CV_32FC1);cvZero(srcMatrix[k]);}

	CvMat *dstMatrix=cvCreateMat(3,3,CV_32FC1);
	CvMat *avgVector=cvCreateMat(1,3,CV_32FC1);
	CvMat *evects=cvCreateMat(3,3,CV_32FC1);
	CvMat *evals=cvCreateMat(3,1,CV_32FC1);
	cvZero(evects);
	cvZero(evals);
	
	//fill in the data. First: tentre, , then the lp
	cvmSet(srcMatrix[0], 0, 0, centreP.GetX());
	cvmSet(srcMatrix[0], 0, 1, centreP.GetY());
	cvmSet(srcMatrix[0], 0, 2, centreP.GetZ());

	for (int k=1;k<=lp.size();k++)
	{
	cvmSet(srcMatrix[k], 0, 0, lp[k-1].GetX());
	cvmSet(srcMatrix[k], 0, 1, lp[k-1].GetY());
	cvmSet(srcMatrix[k], 0, 2, lp[k-1].GetZ());
	}
  
      //DOIT
	cvCalcCovarMatrix((const CvArr**)srcMatrix, lp.size()+1,dstMatrix, avgVector,CV_COVAR_NORMAL);
	
	cvEigenVV(dstMatrix, evects, evals,DBL_EPSILON, 0, 0);
	
	
	double lam1=cvmGet(evals, 0, 0);
	double lam2=cvmGet(evals, 1, 0);
	double lam3=cvmGet(evals, 2, 0);

	//scatter
	scatter=lam3/lam1;
	linearity=(lam1-lam2)/lam1;
	planarity=(lam2-lam3)/lam1;
	curvature=lam3/(lam1+lam2+lam3);
	entropy=-1*(lam1*log(lam1)+lam2*log(lam2)+lam3*log(lam3));
	
	//FREE the MEM!DOUBLECHECK
	
	for (int k=0;k<=lp.size();k++) cvReleaseMat(&srcMatrix[k]); free(srcMatrix);

	cvReleaseMat(&dstMatrix);
	cvReleaseMat(&avgVector);
	cvReleaseMat(&evects);
	cvReleaseMat(&evals);
	
	return true;
}

//overload the eigenvalue fct: get it dircetly form the eigenvalues, eg provided by Plane
bool eigen_attributes(double lam1, double lam2, double lam3, double &scatter, double &linearity, double &planarity, double &curvature,double &entropy)
{
	//scatter
	scatter=lam3/lam1;
	linearity=(lam1-lam2)/lam1;
	planarity=(lam2-lam3)/lam1;
	curvature=lam3/(lam1+lam2+lam3);
	entropy=-1*(lam1*log(lam1)+lam2*log(lam2)+lam3*log(lam3));
	
	
	return true;
}

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

//help function: for the given LaserPoints it computes the connected component number (majority vote), also return the percentage of points falling into this component
void connected_component_of_points(LaserPoints lp, uint &connected_comp, double &percentage)
{
  ::map<uint,uint> connected_comp_occ;
   
  for (int i=0;i<lp.size();i++)
  {
    connected_comp_occ[(uint) lp[i].Attribute(ComponentNumberTag)]++;
   //printf("this comp=%d\n",lp[i].Attribute(ComponentNumberTag));
  }
  
  
  int comp_max, count_max=0;
  for (::map<uint, uint>::iterator it_=connected_comp_occ.begin();it_!=connected_comp_occ.end();it_++)
    if(it_->second > count_max) {count_max=it_->second; comp_max=it_->first;}
    
  connected_comp=comp_max;
  percentage=(double) count_max*100/lp.size();
  
  //printf("in fct: max.occur: component %d, percentage: %.2f\n",comp_max,percentage);
}

//Helpfunction which checks for the given rectangle if the smallest side is at least as big as required
bool check_size_rectangle(ObjectPoints points,LineTopology topo,double min_length_rectangle)
{
//compute the width and length using Distance
  double dist0_1=points[0].Distance(points[1]);
  if (dist0_1 <min_length_rectangle) return 0;
  
  double dist1_2=points[1].Distance(points[2]);
  if (dist1_2 <min_length_rectangle) return 0;
  
  double dist2_3=points[2].Distance(points[3]);
  if (dist2_3 <min_length_rectangle) return 0;
  
  double dist0_3=points[0].Distance(points[3]);
  if (dist0_3 <min_length_rectangle) return 0;
  
  //printf("dist 01:%.3f, 12:%.3f, 23;%.3f; 03: %.3f\n",dist0_1,dist1_2,dist2_3,dist0_3);
  
return 1;
}

void PrintUsage()
{
printf("tie_point_based_orientation_assessment_reference_preparation: preprocessing for tie_point_based_orientation_assessment:normal computation, filtering of useful patches. IT IS ASSUMED that in the input file here only complete buildings are contained, no trees, roads or similar. The buildings must be separated so that a connected component method should cluster per building or roof. A plane-based segmentation in PCM should have been done, as well\n Alternatively the input file can already have connected components information to save time here.");
  printf("\nUsage: \ttie_point_based_orientation_assessment_reference_preparation -reference <laserfile> (output name automatically derived)\n");
} 

int main(int argc, char *argv[])
{

LaserPoints referenceLS, referenceLS_final;

int thresh_NUM_components=50; //1000 for close range
int thresh_NUM_segments=50; //this threshold is not so imporatnatn, in contrarz: it makes it all a bit more sensitive, it actually is affected by the density. More important is the geometry of the segment, i.e. the min_length_rectangle.

double max_dist_to_plane=0.3;  //0.02 for close range //filter planes where the segmentation went wrong
double max_dist_to_plane_stddev=0.3; //0.05 for close range //filter uncertain plane fittings
double max_curvature_plane=8E-2; //found empirically: 8E-5
double min_height_component_bb=0; //used for the connected components. we only look for tall objects
double min_width_componenn_bb=3; //used for the connected components: the x/y extend must be at least this
double min_length_rectangle=3.0; //(used for segment check)

::map<uint,Position3D> COG_per_connected_component; //stores in the first field the number of the CC, and then the Centre of Gravity

printf("tie_point_based_orientation_assessment_reference_preparation\n");

//assume a laser file where 
//workflow: 
//compute connected components, quite large tolerance values (eg up to 2.5 m difference) to ensure that we have single buildings per cluster, use 2D neighborhood!
//per segment and per connected compentn compute the minimal enclosing rectangle. Omit any segment/component which is too tall or too thin
//compute the geometric center per valid cluster/building. This is needed to derive the correct normal direction (all normals point to outside of the building)

//per valid segment compute the plane equation. The normal points outside (i.e. the centre of the building must have a negative distance) In case of ALS we mostly only have roofs, so if there is a flat roof we just let the normal point upwards. For slanted roof landscapes we need to assume that the centre point lies inside. TEST: if the centre point is too close to the segment and the normal is not (close to) hoirizontal we assume a roof segment. Then we can orient the normals also just putting a point above the segment.
//save the normal per segment (into the laserfile??)
 InlineArguments *args = new InlineArguments(argc, argv);
 
  
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-reference"))
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

referenceLS_final.ReInitialise();

//to reduce the risk of numerical problems we shorten the coordinates: the first poit will be substracted from all, i.e. the first is then 0,0,0
Position3D globaloffset=referenceLS[0];

for (int k=0;k< referenceLS.size();k++)
  referenceLS[k]-=globaloffset;
  
printf("After offset removal\n");

		
//check if we have segments, if not error...
vector< int >  list_of_initial_segments= referenceLS.AttributeValues(SegmentNumberTag);
if (list_of_initial_segments.size()<1) 
 {printf("NON-segmented file! Error!\n"); exit(0);} 


//check if we have already connected components
vector< int >  list_of_initial_components(0);
//list_of_initial_components= referenceLS.AttributeValues(ComponentNumberTag);

printf("FORCCING Connected Components computation!\n");

//if (list_of_initial_components.size()<1) 
if (true) 
 {printf("NO connected components in the laser file, compute it now!\n");  

//do the connected components, see example code also in 3D_supervised_classification
TINEdges     *edges;
     SegmentationParameters* segmentation_parameters = new SegmentationParameters();
     segmentation_parameters->MaxDistanceInComponent()=2.5; 
     segmentation_parameters->ComponentAttribute()=ComponentNumberTag;
     segmentation_parameters->DistanceMetricDimension()=2; //do it in 2D!!! mitigates the roof/extension-problem a bit
     
     printf("Distance between components: %.1lf; distancemetric dimension: %d, store component number in %s\n",segmentation_parameters->MaxDistanceInComponent(),segmentation_parameters->	DistanceMetricDimension(),AttributeName(segmentation_parameters->ComponentAttribute(),1));
      // Derive the edges that define the neighbour relations
     edges = referenceLS.DeriveEdges(*segmentation_parameters);
     
     
     // Remove long edges
    if (segmentation_parameters->MaxDistanceInComponent() > 0.0)
        referenceLS.RemoveLongEdges(edges->TINEdgesRef(), 
                       segmentation_parameters->MaxDistanceInComponent(),
                       segmentation_parameters->DistanceMetricDimension()==2);
		       
     // Label the connected components
      referenceLS.LabelComponents(edges->TINEdgesRef(), segmentation_parameters->ComponentAttribute());
    delete edges;
    delete segmentation_parameters;
    //add the globaloffset again beofre writing to disk
    for (int k=0;k< referenceLS.size();k++)
    referenceLS[k]+=globaloffset;
    
    referenceLS.Write("debug_connected_comps.laser",0);
    
     //remove it again, otherwise it will add double later
    for (int k=0;k< referenceLS.size();k++)
    referenceLS[k]-=globaloffset;
    list_of_initial_components= referenceLS.AttributeValues(ComponentNumberTag);
 }
else printf("input file contains components already\n");

//now compute for each component and segment the enclosing rectangle and remove too small ones
vector< int >  list_of_final_components(0);
vector< int >  list_of_final_segments(0);


//start with the compoentns
printf("%d initial connected components and %d initial segments\n",list_of_initial_components.size(), list_of_initial_segments.size());
    
     for (int i=0;i<list_of_initial_components.size();i++)
      {
	LaserPoints thiscomponent_points=referenceLS.SelectTagValue (ComponentNumberTag, list_of_initial_components[i]);
	//if there are less than thresh_NUM points, ignore it
	
	printf("...this component (%d) has %d points...",list_of_initial_components[i],thiscomponent_points.size());
	if (thiscomponent_points.size()<thresh_NUM_components) {printf("too less points\n"); continue;}
	
	//compute the bounding box
	DataBoundsLaser bound=thiscomponent_points.DeriveDataBounds(0);
	//printf("X range=%.2f, Y=%.2f, Z=%.2f\n",bound.XRange(),bound.YRange(),bound.ZRange());
	if (bound.ZRange() < min_height_component_bb) {printf("too small\n"); continue;}
	if (bound.XRange() < min_width_componenn_bb) {printf("too small\n"); continue;}
	if (bound.YRange() < min_width_componenn_bb) {printf("too small\n"); continue;}
	
	printf("ACCEPT it!\n");
	 list_of_final_components.push_back(list_of_initial_components[i]);
      }

//check the segmentss (enclosing rectangle) The encl rectangle is the projecton in xy plane, so this is not so accurate for slanted planes, however, it should not be too complicated. : we rotate the segment into the x-y-plane and then use the encl.rectangle

LaserPoints LS_debug_in_plane; LS_debug_in_plane.ReInitialise();//to test: here we will put all points which should be in the plane. Will be a mess, but good for visual check

FILE *debug_eigen;
  if ((debug_eigen=fopen("eigen_attributes.pts","w")) == NULL)
		{
			printf("CAN NOT OPEN eigen_attributes.pts!\n");
			return(0);
		}
		
for (int i=0;i<list_of_initial_segments.size();i++)
      { 
	LaserPoints thissegment_points=referenceLS.SelectTagValue (SegmentNumberTag, list_of_initial_segments[i]);
	printf("...this segment (%d) has %d points\n",list_of_initial_segments[i],thissegment_points.size());
	
	//if there are less than thresh_NUM points, ignore it
	if (thissegment_points.size()<thresh_NUM_segments){printf("this segment is too small\n"); continue;}
	
	//determine the connected component this segment is assigned to. If the connected component is not in the final ones, just also skip this segment!
	uint connected_comp; double percentage;
	connected_component_of_points(thissegment_points, connected_comp, percentage);
	printf("in main: max.occur: component %d, percentage: %.2f\n",connected_comp,percentage);
	
	//if the percentage is lower than 90% this segment seems to be split over too many components --> reject
	//also if the componenet was not accepted above skip it!
	
	if (percentage < 90) {printf("skip this segment, because it seems to be split over many components -> not reliable\n"); continue;}
	
	bool in_final_list=0;
	for (int j=0;j<list_of_final_components.size();j++)
	  if (list_of_final_components[j]==connected_comp) {in_final_list=1;break;}
	  if (!in_final_list){printf("this segment is not part of an accepted connnected component\n");continue;}
	
	

	//check size of this segmentnd push back to the list of final segments if applicable	
	//thissegment_points.VerifyTIN();
        //thissegment_points.DeriveDataBounds(0);
	
	//rotate the points into the horizontal plane!
	//thissegment_points.Write("segmentORIGINAL.laser",1);
	//compute the center of gravity and the normal vector
	
	//fit a plane to the points, 
	Plane plorig(thissegment_points[0],thissegment_points[1],thissegment_points[2]);
	for(uint k=3;k<thissegment_points.size();k++) plorig.AddPoint(thissegment_points[k],true);
	
	
	// PointNumberList tmplist;
        // for(uint k=0;k<thissegment_points.size();k++) /tmplist.push_back(k);
        // Position3D COG_test=thissegment_points.CentreOfGravity(tmplist);
      
	//scaled with -1 to be abel to transforem right awwy
	Vector3D PlaneNormal=plorig.Normal().Normalize();
	Position3D PlaneCentreGrav=plorig.CentreOfGravity()*(-1);
	
	//check the plane geometry, especially curvature: if it is too big: continue, ie skip this one. Also if the points are not on that plane skip it
	//test one: mean distance (and stddev ) of points to this plane. if mean distance > max_dist_to_plane or the stdev> max_dist_to_plane_stddev continue
	//put all the distances in to vector<double>
	vector<double> dist_to_plane_vec(0);
	for (int k=0; k<thissegment_points.size(); k++)
	  {
	   Vector3D t(thissegment_points[k]);
	   dist_to_plane_vec.push_back(plorig.Distance(t));
	  }
	//compute mean_stddev_vector
	double mean_dist, stddev_dist;
	mean_stddev_vector(dist_to_plane_vec,mean_dist, stddev_dist);
	printf("segnum: %d, mean_distance all points: %.2f, stddev: %.3f\n",list_of_initial_segments[i],mean_dist, stddev_dist);
	if (abs(mean_dist) > max_dist_to_plane || stddev_dist > max_dist_to_plane_stddev) {printf("TOO BAD!!!==> skip"); continue;}
	
	//compute eigen value attribs
	double scatter, linearity, planarity,curvature,entropy;
	
	//eigen_attributes(thissegment_points,PlaneCentreGrav, scatter, linearity, planarity, curvature, entropy);
	eigen_attributes(plorig.Eigenvalue(0),plorig.Eigenvalue(1),plorig.Eigenvalue(2), scatter, linearity, planarity, curvature, entropy);
	printf("segnum: %d, scatter: %.3f, linearity: %.3f, planarity: %.3f, curvature: %.7f, entropy: %.3f\n",list_of_initial_segments[i],scatter, linearity, planarity, curvature, entropy);
	
	//debug: add all points in a pts file and add one of the eigen attributes, eg to visualize in CC
	for (int k=0; k<thissegment_points.size(); k++)
	  fprintf(debug_eigen,"%.3f   %.3f   %.3f   %.9f\n",thissegment_points[k].GetX(),thissegment_points[k].GetY(),thissegment_points[k].GetZ(),/*stddev_dist*/curvature);
	    
	 
	
	if (curvature > max_curvature_plane) {printf("curvature TOO BIG!!!==> skip"); continue;}
	
	 
	  
        //printf("COG from laserpoints X=%.2f Y=%.2f Z=%.2f\n",COG_test.X(),COG_test.Y(),COG_test.Z());
     
	
	//do the rotation: compute an axis for rotation and and angle, seehttp://stackoverflow.com/questions/13199126/find-opengl-rotation-matrix-for-a-plane-given-the-normal-vector-after-the-rotat
	//RotationAxis = cross(N, N')
	//RotationAngle = arccos(dot(N, N') / (|N| * |N'|))
	//here N' is 0,0,1
	//vector in the horizontal
	//Need to check for PlaneNormal: must not be approx 0,0,-1 (because then the crossproduct is not unique). If the Plane normal is around that value, just invert
	  
	if (PlaneNormal.Z() < -0.8) {PlaneNormal.X()*=(-1);PlaneNormal.Y()*=(-1);PlaneNormal.Z()*=(-1);}
	
	
	Vector3D rotaxis=PlaneNormal.VectorProduct(Vector3D(0,0,1));
	//the sin of the angle between the to normals is  1/length of the vector
	//double sin_rotangle=rotaxis.Length();
	
	//double asin_rotangle=asin(sin_rotangle);
	
	//printf("angle from vector product: sin=%.4f, angle=%.4f\n",sin_rotangle, asin_rotangle);
	//normalize
	rotaxis=rotaxis.Normalize();
	double cos_rotangle=PlaneNormal.DotProduct(Vector3D(0,0,1));
	double rotangle=acos(cos_rotangle);
	Rotation3D rotmat(rotaxis,(-1)*rotangle);
	
	printf("COG X=%.2f Y=%.2f Z=%.2f\n",PlaneCentreGrav.X(),PlaneCentreGrav.Y(),PlaneCentreGrav.Z());
	printf("Normal X=%.4f Y=%.4f Z=%.4f\n",PlaneNormal.X(),PlaneNormal.Y(),PlaneNormal.Z());
       
	
	printf("rotaxis X=%.5f rotaxis Y=%.5f rotaxis Z=%.5f, rotangle=%.4f\n",rotaxis.X(),rotaxis.Y(),rotaxis.Z(),rotangle);
	
	//rotmat.Print(cout);
	
	
	//first move to 0,0,0, then rotate
	for(uint k=0;k<thissegment_points.size();k++) 
	{
	  thissegment_points[k]+=PlaneCentreGrav;
	  //Vector3D rotated=rotmat.Rotate((thissegment_points)[k]);
	  //(thissegment_points)[k]=rotated;
	  (thissegment_points)[k]=rotmat*Vector3D((thissegment_points)[k]);
	  
	  LS_debug_in_plane.push_back((thissegment_points)[k]);
	}
	
	Plane planetrans(thissegment_points[0],thissegment_points[1],thissegment_points[2]);
	for(uint k=3;k<thissegment_points.size();k++)
	planetrans.AddPoint(thissegment_points[k],true);
	
	Vector3D PlaneROTNormal=planetrans.Normal().Normalize();
	Position3D PlaneROTCentreGrav=planetrans.CentreOfGravity();
	
	//printf("COG after trans (must be zero) X=%.2f Y=%.2f Z=%.2f\n",PlaneROTCentreGrav.X(),PlaneROTCentreGrav.Y(),PlaneROTCentreGrav.Z());
	printf("Normal after trans (z must be 1) X=%.4f Y=%.4f Z=%.4f\n",PlaneROTNormal.X(),PlaneROTNormal.Y(),PlaneROTNormal.Z());
       if (abs(PlaneROTNormal.Z()) <0.99) {printf("implementation error: rotated segment not horizontal!\n");exit(0);}
       
	ObjectPoints points;
	LineTopology topo;
	thissegment_points.VerifyTIN();
	thissegment_points.DeriveDataBounds(1);
	thissegment_points.EnclosingRectangle(1,points,topo);
	//thissegment_points.DeriveContour3D(points,topo);

	//tmp write out points and polygons
	//thissegment_points.Write("segment_rotate.laser",1);
	//points.Write("encl_rect.objpts");
	//LineTopologies topos;topos.push_back(topo);
	//topos.Write("encl_rect.top");

	if (check_size_rectangle(points,topo,min_length_rectangle)) list_of_final_segments.push_back(list_of_initial_segments[i]);
	
	//exit (0);
      }

      fclose(debug_eigen);
      LS_debug_in_plane.Write("all_in_plane.laser.gz",1);
//for the referenceLS_final take all points which are listed in both:list_of_final_components and ...segments: the easiest way will be to simply: first copy all points according to the correct component number into one temporary laserpoints instance, from there then copy all the points with the correct segments.
LaserPoints tmp_ls;
tmp_ls.ReInitialise();
//also save per component the COG
for (int i=0;i<list_of_final_components.size();i++)
      {
	PointNumberList this_comp_point_numbers;
	LaserPoints thiscomponent_points=referenceLS.SelectTagValue (ComponentNumberTag, list_of_final_components[i]);
	for (int l=0;l<thiscomponent_points.size();l++) 
	{
	  tmp_ls.push_back(thiscomponent_points[l]);
	  this_comp_point_numbers.push_back(l);
	}
	COG_per_connected_component[list_of_final_components[i]]=thiscomponent_points.CentreOfGravity(this_comp_point_numbers);
      }

for (int i=0;i<list_of_final_segments.size();i++)
      {
	LaserPoints thissegment_points=tmp_ls.SelectTagValue (SegmentNumberTag, list_of_final_segments[i]);
	for (int l=0;l<thissegment_points.size();l++) referenceLS_final.push_back(thissegment_points[l]);
      }

 //add the globaloffset again beofre writing to disk
 for (int k=0;k< referenceLS_final.size();k++)
 referenceLS_final[k]+=globaloffset;

 referenceLS_final.Write("reference_ls_final.laser",0);
//tmp_ls.Write("debug_reference_ls_final.laser",0);

 //remove the offset again...
 for (int k=0;k< referenceLS_final.size();k++)
 referenceLS_final[k]-=globaloffset;
 
//list of centres of gravity
for (::map<uint, Position3D>::iterator it_=COG_per_connected_component.begin();it_!=COG_per_connected_component.end();it_++)
  {printf("connected component: %d, centre of grav: %.2f;%.2f;%.2f\n",it_->first,it_->second.GetX(),it_->second.GetY(),it_->second.GetZ());
  }
    
  //iterate over the final segments (need to extract them again from the final ls). Per segment compute the plane parameters and make sure that the normal points away from the COG of the corresponding connected component
  //CHECK: in case we only have a flat roof building the COG is quite close to the plane (or in it). So simply check if we have flat roof (ie normal z component around 1 or -1). if it is -1 just invert, because we assume that it is a roof and pointing upwards
  //open an ascii file for the plane normal components (segment number normal_x normal_y normal_z)
  FILE *normsF;
  if ((normsF=fopen("reference_ls_final_normals_per_segment.txt","w")) == NULL)
		{
			printf("CAN NOT OPEN reference_ls_final_normals_per_segment.txt!\n");
			return(0);
		}


  list_of_final_segments= referenceLS_final.AttributeValues(SegmentNumberTag);
  
  //write the vectors to objpts/topo to visually debug the method
  ObjectPoint pt1, pt2;
  ObjectPoints normal_points;
  LineTopology line;
  LineTopologies normal_topos;
  int scale_normalvectors=5;//just for visualisation
  int objnumber = 0;
  int linenumber = 1;
	
  for (int i=0;i<list_of_final_segments.size();i++)
      {
	LaserPoints thissegment_points=referenceLS_final.SelectTagValue (SegmentNumberTag, list_of_final_segments[i]);
	
	Plane plorig(thissegment_points[0],thissegment_points[1],thissegment_points[2]);
	for(uint k=3;k<thissegment_points.size();k++) plorig.AddPoint(thissegment_points[k],true);
	
	Vector3D PlaneNormal=plorig.Normal().Normalize();
	Position3D PlaneCentreGrav=plorig.CentreOfGravity();
	
	//compute the (signed) distance of the COG of the connected component where this segment is to the plane 
	uint connected_comp; double percentage;
	connected_component_of_points(thissegment_points, connected_comp, percentage);
	
	double dist_to_COG=plorig.Distance(COG_per_connected_component[connected_comp]);
	printf("segment number %d, is in component number %u, signed distance of this plane to the COG of this component:%.2f\n",list_of_final_segments[i],connected_comp,dist_to_COG);
	
	//if the distance is larger 0 the normal needs to be inverted!!!
	if (dist_to_COG > 0) {PlaneNormal.X()*=(-1);PlaneNormal.Y()*=(-1);PlaneNormal.Z()*=(-1);}
	//if we have a horizontal plane we make sure that it points upwards (so only if z is < -.9)
	if (PlaneNormal.Z() < -0.9) {PlaneNormal.X()*=(-1);PlaneNormal.Y()*=(-1);PlaneNormal.Z()*=(-1);}
	//another problem is with slanted roof surfaces in l-structured buildings: dendening on the relative position wrt the COG the normal might have the wrong direction. 
	//if we assume that non-vertical and non-horizontal planes are roof faces we can solve this by simply ensuring that the z comp is > 0. This is however not always true, eg a face at the facade which is tilted (encloses an angle with the vertical). However there it is only a problem if those faces point downward and from airboren images we wont see those planes anyway. If terr. images are invoved a better solution (manual correcton of normal compeonents in the resulting text file?) needs to be found
	//so if the zcomp is negatve (and between -0.1 and -0.9) we invert it
	if (PlaneNormal.Z() < -0.1 && PlaneNormal.Z() > -0.9) {PlaneNormal.X()*=(-1);PlaneNormal.Y()*=(-1);PlaneNormal.Z()*=(-1);}
	
	
	//write the segemnet number, the COG of this segment and the normal to file. REgardint the COG: add the offset!!!
	//add the globaloffset again beofre writing to disk
 
	fprintf(normsF,"%d   %.3f   %.3f   %.3f   %.3f   %.3f   %.3f\n",list_of_final_segments[i],PlaneCentreGrav.GetX()+globaloffset.GetX(), PlaneCentreGrav.GetY()+globaloffset.GetY(), PlaneCentreGrav.GetZ()+globaloffset.GetZ(),PlaneNormal.X(),PlaneNormal.Y(),PlaneNormal.Z());
	
	//write vectors for check in to a ObjectPoints/topo file
	pt1 = ObjectPoint (PlaneCentreGrav.GetX(), PlaneCentreGrav.GetY(), PlaneCentreGrav.GetZ(), ++objnumber, 0, 0, 0, 0, 0, 0);
           		
	pt1+=globaloffset;
	normal_points.push_back(pt1);
	
	pt2 = ObjectPoint (PlaneCentreGrav.GetX()+scale_normalvectors*PlaneNormal.X(), PlaneCentreGrav.GetY()+scale_normalvectors*PlaneNormal.Y(), PlaneCentreGrav.GetZ()+scale_normalvectors*PlaneNormal.Z(), ++objnumber, 0, 0, 0, 0, 0, 0);
           		
	pt2+=globaloffset;
	normal_points.push_back(pt2);
	
           		line = LineTopology (linenumber++, 1, objnumber - 1, objnumber);
           		normal_topos.push_back(line);
      }
   normal_points.Write("normals_scaled.objpts");
   normal_topos.Write("normals_scaled.top");
      
  fclose(normsF);
  return EXIT_SUCCESS;
}
