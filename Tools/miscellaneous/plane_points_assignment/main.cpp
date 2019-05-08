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

//Input: faces from a CAD (IFC) model in mapping lib map format (objpts, top) incl. some attributes like entity id, and a point cloud (optional). If a point cloud is given, the points assigned to the faces are selected (compare tie_point_based_orientation_assessment). Per face (its id) the allocated points are exported. If no point cloud is given, a point cloud fitting to the outer elements is created. This is used in RECONASS to do the model-UAV-data co-registration

void compute_inner_bb(DataBounds3D &bounds_model,  double &max_dist_to_bounding_box, double &socket_height, DataBounds3D &bounds_model_inner)
{
  //go along the bounding box and create new points
  Position3D min_model=bounds_model.Minimum();
  Position3D max_model=bounds_model.Maximum();

  //if the max_dist_to_bounding_box is zero this inner/outer thing will be disabled. Trick: set the bounds_model_inner to a very small box in the centre of the outer one
  
if (max_dist_to_bounding_box==0)
{
printf("max_dist_to_bounding_box is set to 0 .... shell not desired (eg for INACHUS)\n");
//compute centre for the outerone
double cent_X=(max_model.GetX()+min_model.GetX())/2;
double cent_Y=(max_model.GetY()+min_model.GetY())/2;
double cent_Z=(max_model.GetZ()+min_model.GetZ())/2;

bounds_model_inner.SetMinimumX(cent_X-0.01); bounds_model_inner.SetMaximumX(cent_X+0.01);
bounds_model_inner.SetMinimumY(cent_Y-0.01); bounds_model_inner.SetMaximumX(cent_Y+0.01);
bounds_model_inner.SetMinimumZ(cent_Z-0.01); bounds_model_inner.SetMaximumX(cent_Z+0.01);
return;
}
  
  //first remove the socket height from Z (outer);
  bounds_model.SetMinimumZ(min_model.GetZ()+socket_height);
  min_model=bounds_model.Minimum();
   
  //make a smaller inner box to decide later if a point is within the box
  bounds_model_inner=DataBounds3D(bounds_model);
  
  if (bounds_model.XRange() > 2*max_dist_to_bounding_box) {bounds_model_inner.SetMinimumX(min_model.GetX()+max_dist_to_bounding_box); bounds_model_inner.SetMaximumX(max_model.GetX()-max_dist_to_bounding_box);}
  
  if (bounds_model.YRange() > 2*max_dist_to_bounding_box) {bounds_model_inner.SetMinimumY(min_model.GetY()+max_dist_to_bounding_box); bounds_model_inner.SetMaximumY(max_model.GetY()-max_dist_to_bounding_box);}
   
  if (bounds_model.ZRange() > 2*max_dist_to_bounding_box) {bounds_model_inner.SetMinimumZ(min_model.GetZ()+max_dist_to_bounding_box); bounds_model_inner.SetMaximumZ(max_model.GetZ()-max_dist_to_bounding_box);}
 
 //test; remove all the areas in the building (ground installations) by extending the inner bound to the ground (Zmin)
 bounds_model_inner.SetMinimumZ(min_model.GetZ());
}


using namespace std;

void PrintUsage()
{
printf("plane_points_assignment: Input: faces from a CAD (IFC) model in mapping lib map format (objpts, top) incl. some attributes like entity id.  Per face (its id) a dense artificial point cloud is exported.  This is used in RECONASS to do the model-UAV-data co-registration (i.e. this program creates a reference point cloud with plane ids assigned to the points)\n");
  printf("\nUsage: \tplane_points_assignment -objpts <objpts-file> -top <top file>\n");
} 

int main(int argc, char *argv[])
{

::map<uint,Position3D> planeCOG_per_polygon; //stores in the first field the number of the polygon, and then the Plane parameters
::map<uint,Vector3D> planeNormal_per_polygon; //stores in the first field the number of the polygon, and then the Plane parameters
//::map<uint,Rotation3D> rotation_to_hz_per_polygon; //stores the rotatoin matrix per polygon used to rotate parallel to hz plane. the offset is the centre of gravity as stored in the plane

//store all the polygon object points (and topology) into containers. 
//vector<ObjectPoints> polygons_local_origin_points;
//vector<PointNumberList> polygons_local_origin_point_numbers;



double max_dist_to_bounding_box=0; //if it is 0: skip it... (eg INACHUS) for the RECONASS building: .8; //It might be necessary/handy to only use planes in the outer area and simply skip inlying planes 

double point_distance_densification=0.3; //RECONASS 0.1
double socket_height=0; // RECONASS 0.7; //a horizontal slice of this thickness from the ground will not be considered (eg foundation of the building)

printf("plane_points_assignment\n");



 InlineArguments *args = new InlineArguments(argc, argv);
 
   
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-objpts") || !args->Contains("-top"))
 {
    printf("Error: not all arguments given\n");
    PrintUsage();
    exit(0);
  }
 
 //read the mapping data
ObjectPoints pts(args->String("-objpts"));
int success;
LineTopologies  topo_tmp(args->String("-top"),&success); //topo_tmp...because later it will be filtered - all polygons not within the outer part of interest of the building will be removed

if (!success) {printf("error in reading topo file.\n");exit(0);}

printf("number of points in model:%d, number of initial LineTopologies:%d\n",pts.size(), topo_tmp.size());

DataBounds3D bounds_model=pts.Bounds();
Position3D min_model=bounds_model.Minimum();
Position3D max_model=bounds_model.Maximum();

DataBounds3D bounds_model_inner;

printf("Databounds of model: X: %.4lf to %.4lf, Y: %.4lf to %.4lf, Z: %.4lf to %.4lf\n",min_model.GetX(),max_model.GetX(),min_model.GetY(),max_model.GetY(),min_model.GetZ(),max_model.GetZ());

//if we do not havd a point cloud given the task is to compute one, namely one which represets the (outer) hull of the model. This means we compute a kind of artificial point cloud which resembles those faces. Usefule fo RECONASS for the co-registration of IBM PC and model
//For the processing it means the same: whether we have first an initial artificial point cloud (a "block of points") from where we select points close to the planes or if we have a matching-point cloud


compute_inner_bb(bounds_model, max_dist_to_bounding_box,socket_height, bounds_model_inner);

min_model=bounds_model_inner.Minimum();
max_model=bounds_model_inner.Maximum();
 printf("in main... INNER MODEL Databounds of model: X: %.4lf to %.4lf, Y: %.4lf to %.4lf, Z: %.4lf to %.4lf\n",min_model.GetX(),max_model.GetX(),min_model.GetY(),max_model.GetY(),min_model.GetZ(),max_model.GetZ());
  
LaserPoints PCloud; PCloud.ReInitialise();
//now we have a similar approach like in tie_point_based_orientation_assessment...
//1. compute normals and COG per line (i.e. poly--- need to check if all are closed) and store in the maps.
//2. rotate them into the hz-plane (before shft to origin)
//3. compute lasaer points falling into this local hz plane
//4. transform back into the real world


LineTopologies  topo; topo.resize(0); //now fill the real topo vector - only take one if the cog is in the area of interest (ie not in the inner bb)
int topo_current_index=0;

LaserPoints COG_LP;COG_LP.ReInitialise();

for (int i=0;i<topo_tmp.size();i++)
{
  if (topo_tmp[i].IsClosed()==0) {printf("Error: line number %d is not closed\n",i); exit(0);}
  //for the coG: we just take the midpoint of the bounding box. For the plane equation this is good enough.
  DataBounds3D db3= topo_tmp[i].Bounds(pts);
  Position3D COG_tmp=db3.MidPoint();
  
  //if this COG is in the inner bounding box - skip it ... 
  if (bounds_model_inner.Inside(COG_tmp) || !bounds_model.Inside(COG_tmp)) continue;
  else//take it
  {
  topo.push_back(topo_tmp[i]);
  planeNormal_per_polygon[topo_current_index]=topo_tmp[i].Normal(pts);
  planeCOG_per_polygon[topo_current_index]=db3.MidPoint();
  
  topo_current_index++;
  
  COG_LP.push_back(db3.MidPoint());
  }
  
}

COG_LP.Write("/tmp/COGs.laser",0);

printf("... densification...\n");

//rotate each plane into the hz plane
for (::map<uint, Vector3D>::iterator it_=planeNormal_per_polygon.begin();it_!=planeNormal_per_polygon.end();it_++)
{
  
  printf("%d/%d\r",it_->first+1,planeNormal_per_polygon.size());
  
  Vector3D PlNormal=it_->second;
  Position3D PlCOG=planeCOG_per_polygon[it_->first];
  
 //rotation matrix, see tie_point_based_orientation_assessment_preparation
  Vector3D rotaxis=PlNormal.VectorProduct(Vector3D(0,0,1));
  //printf("rotaxis: %.2f %.2f %.2f\n",rotaxis.X(),rotaxis.Y(),rotaxis.Z());
  if (rotaxis.Length()>1E-3) rotaxis=rotaxis.Normalize();
  double rotangle=acos(PlNormal.DotProduct(Vector3D(0,0,1)));
  Rotation3D rotmat(rotaxis,(-1)*rotangle);
  
  //rotation_to_hz_per_polygon[it_->first]=rotmat;
  
  //get this pointnumberlist and the objpts
   ObjectPoints tmp_obj;tmp_obj.resize(0); //the points are here still global, so store them temporally
   PointNumberList pnl; pnl.resize(0); //topology is independent from transforamtion
   ObjectPoints obj_local; obj_local.resize(0);
		 
   int firstPN=topo[it_->first].FirstPointNumber().Number();
   int lastPN=topo[it_->first].LastPointNumber().Number();
   
       //add the first point to the tmp lists
		   tmp_obj.push_back(pts.PointByNumber(firstPN));
		   pnl.push_back(firstPN);
		   
		   PointNumberList::iterator p_it=topo[it_->first].NodeIterator(firstPN);
		   
		   //printf("topo index:%d, Pointnumber:%d,\n",index_hit,p_it->Number());
		   for (int z=0;;z++)//will break inside
		      {
			//get the next point
			PointNumberList::iterator p_next=topo[it_->first].NextNode(p_it);
			int thisPN=p_next->Number();
			//add to the lists
			tmp_obj.push_back(pts.PointByNumber(thisPN));
			pnl.push_back(thisPN);
			
			 //printf("topo index:%d, Pointnumber:%d,\n",index_hit,thisPN); 
			if (thisPN==lastPN) break;
			
			p_it=p_next;
		      }
		      
		  //transform the objpts to the local system and store them
		  
		  for (int o=0; o<tmp_obj.size();o++)
		    {
		    Vector3D P_local=tmp_obj[o]-planeCOG_per_polygon[it_->first];
		    P_local=rotmat.Rotate(P_local);
		    Covariance3D cov_dummy;
		    //printf("point number tmp=%d\n",tmp_obj[o].NumberRef().Number());
		    
		    ObjectPoint P_local_obj(P_local,tmp_obj[o].NumberRef(),cov_dummy);
		    
		    obj_local.push_back(P_local_obj);
		    }
		  //Now we have the polygon points in the local coordsystem
		  //compute bounding box
		  DataBounds3D bounds_model_origin=obj_local.Bounds();
		  
		  LaserPoints P_densified_local; P_densified_local.ReInitialise();
		   Position3D min_model= bounds_model_origin.Minimum();
		    Position3D max_model= bounds_model_origin.Maximum();
		  
		   for(double X=min_model.GetX(); X<=max_model.GetX();X+=point_distance_densification)
		   {
		    for(double Y=min_model.GetY(); Y<=max_model.GetY();Y+=point_distance_densification)
		      {
			   
				  //only take this point if it is NOT in the inner box
				  LaserPoint LP(X,Y,0);
				  int is_in=LP.InsidePolygon(obj_local, pnl);
				  if (is_in)  P_densified_local.push_back(LP);
		     }
		   
		    }
		  
		   //P_densified_local.Write("/tmp/localPC.laser",0); exit(0); 
		   //no iterate over all those points, transfrom back and add to the final LP instance
		   //also add the plane ID as Segmetn ID
		   for (int i=0;i<P_densified_local.size();i++)
		   {
		    //first rotate
		    Vector3D P_global=rotmat.Transpose().Rotate(P_densified_local[i]);
		    P_global+=planeCOG_per_polygon[it_->first];
		    LaserPoint LP(P_global);
		    int segmentNumber=topo[it_->first].Attribute(0);
		    LP.SetSegmentNumber(segmentNumber);
		    PCloud.push_back(LP);
		   }
		      
}


printf("\n... done...\n");


  PCloud.Write("PCloud_artificial.laser",0);

  printf("artificial pointcloud resembling the structure stored to PCloud_artificial.laser\n");

  FILE *fpoint;
  fpoint=fopen("PCloud_artificial.pts","w");
  //write a pts file
  for (int i=0;i<PCloud.size();i++)
    fprintf(fpoint,"%.3lf  %.3lf  %.3lf  %d\n",PCloud[i].GetX(),PCloud[i].GetY(),PCloud[i].GetZ(),PCloud[i].SegmentNumber());

  fclose (fpoint);
return EXIT_SUCCESS;
}
