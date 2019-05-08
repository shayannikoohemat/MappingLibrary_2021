
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

//
//
/*
 * ObjectSpaceLine.cpp
 *
 *  Created on: 28 Sep 2009
 *      Author: gerke
 */

#include "ObjectSpaceLine.h"

bool compute_int_ext_from_P(double **PMat, int h, int w, InteriorOrientation &inttmp, ExteriorOrientation &exttmp, double offset_XNull, double offset_YNull)
{
	//!ATTENTION: in PMatrix we have x,y in image is col, row WHILE in the mapping lib we have x,y is row, col

	//InteriorOrientation (double pcc, double prh, double pch, double pk1, double pk2, double pk3,
	//double pp1, double pp2, double pshear, double prot, double psr, double psc, double pdr, double pdc)
	double pixsize=0.012; //in mm, is just a scale in image space, so in the end it does not matter
	matrix *K, *R, *t;

	KRt_from_P(*PMat, &K, &R, &t);
	//mat_print("K",K);
	//mat_print("R",R);
	//mat_print("t",t);
	//double pcc=sqrt(K->m[0][0]*K->m[1][1])*pixsize; //Princip.dist. in mm
	double pcc=K->m[0][0]*pixsize; //Princip.dist. in mm
	//printf("pcc in mm=%.2f  ",pcc);

	double prh=K->m[1][2];//princ.point in pixel row
	double pch=K->m[0][2];
	//printf("prh=%.2f and prc=%.2f",prh, pch);
	double aspectr=K->m[0][0]/K->m[1][1];  //refers to aspectratio col/row, thus pixsize in coldir=pixsize, in rowdir=pixsize*aspectr

	inttmp=InteriorOrientation (pcc, prh, pch, 0, 0, 0, 0, 0, 0 ,0, pixsize*aspectr, -1*pixsize, h,w);

	//debug
	//inttmp.Print();

	Vector3D projection_center= Vector3D(t->m[0][0]+offset_XNull,t->m[1][0]+offset_YNull,t->m[2][0]);
	//the rotationvector is the transpose of the one given in P
	/**/
	Rotation3D rotation_3D= Rotation3D(R->m[0][0],R->m[1][0],R->m[2][0],
			                           R->m[0][1],R->m[1][1],R->m[2][1],
			                           R->m[0][2],R->m[1][2],R->m[2][2]);
	 /**/
	/*
	Rotation3D rotation_3D= Rotation3D(R->m[0][0],R->m[0][1],R->m[0][2],
				                           R->m[1][0],R->m[1][1],R->m[1][2],
				                           R->m[2][0],R->m[2][1],R->m[2][2]);
	 */
	exttmp=ExteriorOrientation(&projection_center, &rotation_3D);

	//exttmp.Print();
	//Verify by forward projection of known 3D point to image (known as reference).
	//from camera0 microdrone set
/*
	matrix *P;
	P_convert(*PMat, &P);
	xelem Begin3Xelem,End3Xelem;
	//begin=auto_0, end=auto_10
	Begin3Xelem.id=0;
	Begin3Xelem.X=-2.4952 ;Begin3Xelem.Y=-18.6993 ;Begin3Xelem.Z=0.7621;

	End3Xelem.id=10;
	End3Xelem.X=-1.3818 ;End3Xelem.Y=-13.2545 ; End3Xelem.Z=1.7476;

	homo2 Begin2homo=::Project(P, &Begin3Xelem); //Project does normalization.
	homo2 End2homo=::Project(P, &End3Xelem);

	printf("zurueckprojeziert mit PMatrix: auto0 (muss col=114.204  row=137.191): x=%.2f y=%.2f   auto10(muss col=366.213  row=195.019) x=%.2f  y=%.2f\n",Begin2homo.x,Begin2homo.y,End2homo.x,End2homo.y);

	mat_free(P);

	//do the same with the Maplib (debug)
	ObjectPoint object_point_begin(-2.4952, -18.6993, 0.7621,0,0,0,0,0,0,0);
	CameraPoint camera_pointbegin=CameraPoint(&object_point_begin,&inttmp,&exttmp);
	ImagePoint image_pointbegin=ImagePoint(&camera_pointbegin,&inttmp);

	ObjectPoint object_point_end(-1.3818, -13.2545 , 1.7476,0,0,0,0,0,0,0);
		CameraPoint camera_pointend=CameraPoint(&object_point_end,&inttmp,&exttmp);
		ImagePoint image_pointend=ImagePoint(&camera_pointend,&inttmp);

	printf("rueck projez mit maplib: Imagepoint auto0 (muss col=114.204  row=137.191) x=%.2f  y=%.2f\n",image_pointbegin.GetX(),image_pointbegin.GetY());
	printf("rueck projez mit maplib: Imagepoint auto10 (muss col=366.213  row=195.019)  x=%.2f  y=%.2f\n",image_pointend.GetX(),image_pointend.GetY());
*/

	//test
    /*
	CameraPoint cp;
	ImagePoint ip (0,0.0,0.0,0.0);
	cp.Image2CameraPoint(&ip,&inttmp);
	cp.Print();

	ImagePoint ip1 (1,0.0,848.0,0.0);
	cp.Image2CameraPoint(&ip1,&inttmp);
	cp.Print();

	ImagePoint ip2 (2,460.0,0.0,0.0);
	cp.Image2CameraPoint(&ip2,&inttmp);
	cp.Print();

	ImagePoint ip3 (3,prh,pch,0.0);
		cp.Image2CameraPoint(&ip3,&inttmp);
		cp.Print();
    */

	mat_free(K); mat_free(R); mat_free(t);
	return 1;


}

bool Construct3DLine_maplib(InteriorOrientation &IOA, ExteriorOrientation &EOA, LineSegment2D &edgeA,InteriorOrientation &IOB, ExteriorOrientation &EOB, LineSegment2D &edgeB, Line3D &line)
{
	//use the maplib to compute the intersecting line as resulting from two image segments and the respective proj. centers
	//1. compute viewing rays for the two points in each view. For every view the normal as defined by the cross product of the viewing rays is the normal to the plane (dist. to origin==Proj. cneter)
	//2. so we have 2 planes from the 1. step, intersect them to obtain the wanted 3D line

	//bool ViewingRay(InteriorOrientation &IO, ExteriorOrientation &EO, Position2D imagepoint, Line3D &ray)
	//A-Image
	Position2D BeginA=edgeA.BeginPoint();
	Position2D EndA=edgeA.EndPoint();

	Line3D rayBeginA, rayEndA;
	if (!ViewingRay(IOA, EOA, BeginA, rayBeginA)) {printf("error in viewing ray computation!\n"); return 0;}
	if (!ViewingRay(IOA, EOA, EndA, rayEndA)) {printf("error in viewing ray computation!\n"); return 0;}

	//compute the Plane from the direction vectors (their crossproduct) and the proj. centre as point on the plane
	Vector3D projcentA(EOA.X(),EOA.Y(),EOA.Z());
	Vector3D normalA = rayBeginA.Direction().VectorProduct(rayEndA.Direction());
	Plane planeA(projcentA,normalA);

	//The same for B-Image
	Position2D BeginB=edgeB.BeginPoint();
	Position2D EndB=edgeB.EndPoint();

	Line3D rayBeginB, rayEndB;
	if (!ViewingRay(IOB, EOB, BeginB, rayBeginB)) {printf("error in viewing ray computation!\n"); return 0;}
	if (!ViewingRay(IOB, EOB, EndB, rayEndB)) {printf("error in viewing ray computation!\n"); return 0;}

	Vector3D projcentB(EOB.X(),EOB.Y(),EOB.Z());
	Vector3D normalB = rayBeginB.Direction().VectorProduct(rayEndB.Direction());
	Plane planeB(projcentB,normalB);

	//Debug
	//double   Z_At (double X, double Y, int *success) const
	//int succ;
	//printf("Debug plane A: Z_At Eo.X, Eo.Y must be Eo.Z: Z_AT=%.2f, Eo.Z=%.2f\n",planeA.Z_At(EOA.X(),EOA.Y(),&succ),EOA.Z());
	//printf("Debug plane B: Z_At Eo.X, Eo.Y must be Eo.Z: Z_AT=%.2f, Eo.Z=%.2f\n",planeB.Z_At(EOB.X(),EOB.Y(),&succ),EOB.Z());

	/**/
	//just for debug: use also plane intersection (directoin comp. must be the same as below
	//if (!Intersect2Planes (planeA, planeB, line)) {printf("error in plane intersection!\n"); return 0;}
	//printf("Direction components line from intersectoin: %.2f %.2f %.2f\n",line.Direction().X(),line.Direction().Y(),line.Direction().Z());
	//return 1;
	/**/
	//better:  another idea: take planeA and find the intersection points planeA->Ray from BeginB and planeA->Ray from EndB.
	//those to points are on the line we are looking for. The same can be done in the other direction as a double check

	//A): planeA and intersect with rays from B
	//bool   IntersectLine3DPlane (const Line3D &line, const Plane &plane, Position3D &pos)
	Position3D intersectplaneA_beginB,intersectplaneA_endB;
	if (!IntersectLine3DPlane (rayBeginB,planeA,intersectplaneA_beginB)) {printf("error in plane->ray intersection!\n"); return 0;}
	if (!IntersectLine3DPlane (rayEndB,planeA,intersectplaneA_endB)) {printf("error in plane->ray intersection!\n"); return 0;}
	//the line we are looking for is now instantied by these two points
	Line3D tmpline_planeA_raysB(intersectplaneA_beginB,intersectplaneA_endB);

	//do the same the other way around
	Position3D intersectplaneB_beginA,intersectplaneB_endA;
	if (!IntersectLine3DPlane (rayBeginA,planeB,intersectplaneB_beginA)) {printf("error in plane->ray intersection!\n"); return 0;}
	if (!IntersectLine3DPlane (rayEndA,planeB,intersectplaneB_endA)) {printf("error in plane->ray intersection!\n"); return 0;}
	//the line we are looking for is now instantied by these two points
	Line3D tmpline_planeB_raysA(intersectplaneB_beginA,intersectplaneB_endA);

	//if both lines have different directions: there occured an error, else just return one of them
	//printf("Direction components tmpline_planeA_raysB: %.2f %.2f %.2f\n",tmpline_planeA_raysB.Direction().X(),tmpline_planeA_raysB.Direction().Y(),tmpline_planeA_raysB.Direction().Z());
	//printf("Direction components tmpline_planeB_raysA: %.2f %.2f %.2f\n",tmpline_planeB_raysA.Direction().X(),tmpline_planeB_raysA.Direction().Y(),tmpline_planeB_raysA.Direction().Z());

	//also the Position3D points from one combination above must be on the line as obtained from the respective other combination
	//double   DistanceToPoint (const Position3D &pt) const
	double testdist_plAraB_beginA=tmpline_planeA_raysB.DistanceToPoint(intersectplaneB_beginA);
	double testdist_plAraB_endA=tmpline_planeA_raysB.DistanceToPoint(intersectplaneB_endA);
	double testdist_plBraA_beginB=tmpline_planeB_raysA.DistanceToPoint(intersectplaneA_beginB);
	double testdist_plBraA_endB=tmpline_planeB_raysA.DistanceToPoint(intersectplaneA_endB);

	if (fabs(testdist_plAraB_beginA) <1E-4 && fabs(testdist_plAraB_endA)<1E-4  && fabs(testdist_plBraA_beginB) <1E-4  && fabs(testdist_plBraA_endB) <1E-4 )
	{
		line=Line3D(tmpline_planeA_raysB);
		return 1;
	}

	else
	{
		printf("computation error in 3D Line construction:\n");
		//printf("tmpline_planeA_raysB: distances to poin intersectplaneB_beginA and intersectplaneB_endA :%.f %.2f\n",tmpline_planeA_raysB.DistanceToPoint(intersectplaneB_beginA),tmpline_planeA_raysB.DistanceToPoint(intersectplaneB_endA));
		//printf("tmpline_planeB_raysA: distances to poin intersectplaneA_beginB and intersectplaneA_endB :%.f %.2f\n",tmpline_planeB_raysA.DistanceToPoint(intersectplaneA_beginB),tmpline_planeB_raysA.DistanceToPoint(intersectplaneA_endB));
		printf("tmpline_planeA_raysB: distances to point intersectplaneB_beginA and intersectplaneB_endA :%.9f %.9f\n",testdist_plAraB_beginA,testdist_plAraB_endA);
		printf("tmpline_planeB_raysA: distances to point intersectplaneA_beginB and intersectplaneA_endB :%.9f %.9f\n",testdist_plBraA_beginB,testdist_plBraA_endB);
		return 0;
	}


}


bool Construct3DLine_pmat(double **PA, LineSegment2D &edgeA, double **PB, LineSegment2D &edgeB, Line3D &line)
{
	vector<double> L(6);


   if (!Construct3DLine(PA, edgeA, PB, edgeB, L)) return false;
   PluckerToEuclidean(L, line);

   return true;
}

bool Construct3DLine(double **PA, LineSegment2D &edgeA, double **PB, LineSegment2D &edgeB, vector<double> &L)

{
	bool ret=true;

	vector <double> piA(4),piB(4);


   ConstructPlane(*PA,edgeA,piA);
   ConstructPlane(*PB,edgeB,piB);

   if(!IntersectTwoPlanes(piA,piB,L))
     ret= false;


   return ret;
}



void ConstructPlane(double P[12], LineSegment2D edge, vector<double> &pi)
{
	//swap the xy coord, because in the maplib the x is col and y is row, here it is assumed vice versa
	//since we use the copy of the edge, not the reference, it is no problem to swap
	Position2D beg=edge.BeginPoint();
	Position2D end=edge.EndPoint();

	beg.SwapXY();
	end.SwapXY();
	LineSegment2D edgetmp(beg,end);


   double a=edgetmp.Getcosphi();
   double b=edgetmp.Getsinphi();
   double d=edgetmp.DistanceToOrigin();

   for(unsigned int i=0;i<4;i++)
   {
      pi[i]=(P[i])*a+(P[4+i])*b-(P[8+i])*d;
   }

   return;

}

bool IntersectTwoPlanes(vector<double> &A, vector<double> &B, vector<double> &L)
{
     L[0]=-A[2]*B[1]+A[1]*B[2];
     L[1]=A[2]*B[0]-A[0]*B[2];
     L[2]=-A[1]*B[0]+A[0]*B[1];
     L[3]=A[3]*B[0]-A[0]*B[3];
     L[4]=A[3]*B[1]-A[1]*B[3];
     L[5]=A[3]*B[2]-A[2]*B[3];
     double d=sqrt(L[0]*(L[0])+L[1]*(L[1])+L[2]*(L[2]));
     if(d<0.0001)
        return false;

     for(int j=0;j<6;j++)
        (L[j])=(L[j])/d;
     return true;
}

void PluckerToEuclidean(vector<double> &L, Line3D &line)
{
   double d,x,y,z;
   d=sqrt(pow(L[0],2)+pow(L[1],2)+pow(L[2],2));
   if(d<0.0001)
     printf("line L is wrong!\n");
   if(fabs(d-1)>0.00001)
     for(int i=0;i<6;i++)
      L[i]=L[i]/d;
   x=(L[1]*L[5]-L[2]*L[4]);
   y=(L[2]*L[3]-L[0]*L[5]);
   z=(L[0]*L[4]-L[1]*L[3]);
   line=Line3D(x,y,z,L[0],L[1],L[2]);

   return;
}



bool ViewingRay(matrix *P, Position2D imagepoint, Line3D &ray)
{
	bool ret=1;

	matrix *PPlus;//Pseudoinv
	matrix *K, *R, *t;

	 if (!mat_PseudoInverseofP(P, &PPlus)){printf("error in Pseudo-Inv computation\n"); mat_free(PPlus); return 0;}
	 KRt_from_P(P, &K, &R, &t);

	 mat_print("t:",t);
	 //Now, t is the first point on the ray and the image point is obtained through backproj.
	 //the two endpoints of the linesegment are converted to vektor*
	 vektor *v2_img=vec_alloc(2);


	 v2_img->v[0]=imagepoint.GetX();
	 v2_img->v[1]=imagepoint.GetY();

	 //DEBUG
	 //matrix *PT = mat_transpose_new(P);

	 homo2 h2_img=VecToHomo2(v2_img);
	 homo3 h3_img=BackProject (PPlus, &h2_img);
	 //homo3 h3_img=BackProject (PT, &h2_img);
	 //mat_free(PT);

	 vektor *v3_img=HomoToVec(&h3_img);

	 vec_print("v",v3_img);

	 //now v3_imgis the otherimage points in 3D space
	 //crate a Line3D
	 ray=Line3D(Position3D(t->m[0][0],t->m[1][0],t->m[2][0]),Position3D(v3_img->v[0],v3_img->v[1],v3_img->v[2]));

	 mat_free(PPlus);
	 mat_free(K); mat_free(R); mat_free(t);
	 vec_free(v2_img);
	 vec_free(v3_img);

	return ret;
}

bool ViewingRay(InteriorOrientation &IO, ExteriorOrientation &EO, Position2D imagepoint, Line3D &ray)
{
	//the viewing ray is defined by the proj. center of the camera and the direction vector
	//V=R*(x y -f)^T
	//imagepoint -> camerapoint , vector x y -f ==> V=R*v
	CameraPoint cp;
	ImagePoint ip (0,imagepoint.GetX(),imagepoint.GetY(),0.0);
	cp.Image2CameraPoint(&ip,&IO);
	//cp.Print();
	//printf("row=%.2f  col=%.2f  x=%.2f  y=%.2f\n",imagepoint.GetX(),imagepoint.GetY(),cp.GetX(),cp.GetY());

	/**/
	double x=cp.GetX()*1E-3; //in the class it is stored in mm
	double y=cp.GetY()*1E-3;
	double f=IO.CameraConstant()*1E-3;

	//printf("row=%.2f  col=%.2f  x=%.6f  y=%.6f f=%.6f\n",imagepoint.GetX(),imagepoint.GetY(),x,y,f);

	//printf("X=%.2f  Y=%.2f    Z=%.2f\n",EO.X(),EO.Y(),EO.Z());
	Vector3D dir(EO.R(0,0)*x+EO.R(0,1)*y-EO.R(0,2)*f,EO.R(1,0)*x+EO.R(1,1)*y-EO.R(1,2)*f,EO.R(2,0)*x+EO.R(2,1)*y-EO.R(2,2)*f);
	//Vector3D dir(EO.R(0,0)*x+EO.R(1,0)*y-EO.R(2,0)*f,EO.R(0,1)*x+EO.R(1,1)*y-EO.R(2,1)*f,EO.R(0,2)*x+EO.R(1,2)*y-EO.R(2,2)*f);

	ray=Line3D(Position3D(EO.X(),EO.Y(),EO.Z()),dir);
	/**/

	//ray=cp.CameraPoint2ObjectLine(&IO,&EO);
	return 1;
}

ImagePoint project3D2img(Position3D& pos, InteriorOrientation *Into, ExteriorOrientation *Exto)
{

	ObjectPoint object_point(pos.GetX(),pos.GetY(),pos.GetZ(),0,0,0,0,0,0,0);
	CameraPoint camera_point=CameraPoint(&object_point,Into,Exto);
	ImagePoint imp=ImagePoint(&camera_point,Into);

	return imp;
}

LineSegment2D project3D2img(LineSegment3D &line, InteriorOrientation *Into, ExteriorOrientation *Exto)
{
	Position3D Begin=line.BeginPoint();
	Position3D End=line.EndPoint();
	ImagePoint image_point_begin=project3D2img(Begin, Into, Exto);
	ImagePoint image_point_end=project3D2img(End, Into, Exto);

	return LineSegment2D(Position2D(image_point_begin.GetX(),image_point_begin.GetY()),Position2D(image_point_end.GetX(),image_point_end.GetY()));
}

bool check_LineSegment2D_identity(LineSegment2D &A,LineSegment2D &B, double parallel_thresh, double dist_thresh, double min_overlap)
{
//there are three constraints which are checked in sequence:
	//parallelism
	//distance
	//coverage

	//for parallelism use bool LineSegment2D::Collinear  	(  	const Line2D &   	 line,
	//double  	err_angle,
	//double  	err_dist
	//, but set the err_dist to 1E6
	//A.Print();
	//B.Print();
	//Linesegment2D::Collinear does not work if the direction is identical, thuse compute the angle between the directions
	//if (!A.Collinear(B,parallel_thresh,1E6)) return 0;
	double ang_deg=fabs(Angle2Lines(A,B)/PI*180);

	//printf("Angle=%.2f\n",ang_deg);
	if (ang_deg > parallel_thresh) //also check 180deg (it is ok if larger than 180-thresh
		if (ang_deg < (180-parallel_thresh))
		return 0;


	//for distance: do not use the ::Distance member from LineSegments, as with collinear segments, it computes the distance between endpoints,here
	//compute the distance from the endpoint to other line (since they are supposed to be parallel at this step, of
	//course, one combination is enough, and it does not matter, whether we take end or begin point
	//printf("distance:%.2f\n",A.DistanceToPoint(B.BeginPoint()));

	if (A.DistanceToPoint(B.BeginPoint()) > dist_thresh) return 0;

	 //the coverage: since now both segments are collinear we define a tmp Line2D and compare the skalars of the
	 //begein and endpoints of the segments. the sorted ones are then analysed:
	 Line2D tmpl(A.BeginPoint(),A.EndPoint());
	 //Add some tiny noise, because later the sorting function does not check for identical values in the scalars.
	 //It is quite unlikely that two edges are identical, but to be sure, add this
	 double SkABeg=tmpl.Scalar(A.BeginPoint())+1E-6;
	 double SkAEnd=tmpl.Scalar(A.EndPoint())+1E-7;
	 double SkBBeg=tmpl.Scalar(B.BeginPoint());
	 double SkBEnd=tmpl.Scalar(B.EndPoint());

	 //printf("SkABeg=%.2f,SkAEnd=%.2f,SkBBeg=%.2f,SkBEnd=%.2f\n",SkABeg,SkAEnd,SkBBeg,SkBEnd);
	 //if both from A are smaller or larger than the ones from B, there is no coverage at all

	 if (((SkABeg < SkBBeg) && (SkAEnd < SkBBeg)) && ((SkABeg < SkBEnd) && (SkAEnd < SkBEnd))) return 0;

	 if (((SkABeg > SkBBeg) && (SkAEnd > SkBBeg)) && ((SkABeg > SkBEnd) && (SkAEnd > SkBEnd))) return 0;

	 //now find the min and max skalars. The other two are the ones representing the overlapping area
	 double SKMin=1E6;double SKMax=-1E6;
	 SKMin=MIN(SKMin,SkABeg);SKMin=MIN(SKMin,SkAEnd);SKMin=MIN(SKMin,SkBBeg);SKMin=MIN(SKMin,SkBEnd);
	 SKMax=MAX(SKMax,SkABeg);SKMax=MAX(SKMax,SkAEnd);SKMax=MAX(SKMax,SkBBeg);SKMax=MAX(SKMax,SkBEnd);

	 double SKmid1,SKmid2;bool SKmid1_set=0;
	 if ((SkABeg!=SKMin) && (SkABeg!=SKMax)) {if (!SKmid1_set){SKmid1=SkABeg;SKmid1_set=1;} else SKmid2=SkABeg;}
	 if ((SkAEnd!=SKMin) && (SkAEnd!=SKMax)) {if (!SKmid1_set){SKmid1=SkAEnd;SKmid1_set=1;} else SKmid2=SkAEnd;}
	 if ((SkBBeg!=SKMin) && (SkBBeg!=SKMax)) {if (!SKmid1_set){SKmid1=SkBBeg;SKmid1_set=1;} else SKmid2=SkBBeg;}
	 if ((SkBEnd!=SKMin) && (SkBEnd!=SKMax)) {if (!SKmid1_set){SKmid1=SkBEnd;SKmid1_set=1;} else SKmid2=SkBEnd;}

	 //printf("SKMin=%.2f, SKMax=%.2f, Skmid1=%.2f, Skmid2=%.2f\n",SKMin,SKMax, SKmid1, SKmid2);
	 //The coverage is the ratio from the linesegment2D as defined bz mid1 and mid2 / min and  max
	LineSegment2D outer(tmpl.Position(SKMin),tmpl.Position(SKMax));
	LineSegment2D inner(tmpl.Position(SKmid1),tmpl.Position(SKmid2));
	double cov=inner.Length()/outer.Length();

	//printf("overlap is %.2f\n",cov);

	if (cov>min_overlap) return 1;
	else return 0;
}


ObjectSpaceLine::ObjectSpaceLine() {
	// TODO Auto-generated constructor stub
	ImageIdsV.resize(0);
	PMatV.resize(0);
	ImageLineV.resize(0);
	ObjectLineV.resize(0);
	IntOriV.resize(0);
	ExtOriV.resize(0);
	imgheight=0;
	imgwidth=0;
}

ObjectSpaceLine& ObjectSpaceLine::operator=(const ObjectSpaceLine& osl)
{
	this->Line3DReference()=osl.Line3DReference();
	this->ImageIdsV.resize(0);
	this->PMatV.resize(0);
	this->ImageLineV.resize(0);
	this->ImageLine_dresidualV.resize(0);
	this->ObjectLineV.resize(0);

	this->imgheight=osl.imgheight;
	this->imgwidth=osl.imgwidth;
	this->unique_instance_id_objectspacelines=osl.unique_instance_id_objectspacelines;
		for (unsigned int i=0;i<osl.ImageIdsV.size();i++)
		{
			double *Ptmp; Ptmp=(double*) malloc (12*sizeof(double));

				for(int j=0; j<12;j++)
				   {
					 Ptmp[j]=osl.PMatV[i][j];
					}

				this->PMatV.push_back(Ptmp);

				   	LineSegment2D lineseg(osl.ImageLineV[i].BeginPoint(),osl.ImageLineV[i].EndPoint());
				   	lineseg.Number()=osl.ImageLineV[i].Number();

				   	this->ImageIdsV.push_back(osl.ImageIdsV[i]);
				   	if (osl.ImageLine_dresidualV.size()==osl.ImageIdsV.size()) this->ImageLine_dresidualV.push_back(osl.ImageLine_dresidualV[i]);
				   	this->ImageLineV.push_back(lineseg);
				InteriorOrientation inttmp;
				ExteriorOrientation exttmp;

				if (!compute_int_ext_from_P(&Ptmp, imgheight, imgwidth, inttmp, exttmp)) {printf("error during conversion from P to Interior and Ext.ori"); exit(0);}

				this->IntOriV.push_back(inttmp);
				this->ExtOriV.push_back(exttmp);

				this->ObjectLineV.push_back(osl.ObjectLineV[i]);
		}
		this->ObjectLineCombined=osl.ObjectLineCombined.LineSegment3DRef();

	return (*this);
}


ObjectSpaceLine::ObjectSpaceLine(const ObjectSpaceLine &osl) //copy constructor
{
	this->Line3DReference()=osl.Line3DReference();
	this->ImageIdsV.resize(0);
	this->PMatV.resize(0);
	this->ImageLineV.resize(0);
	this->ImageLine_dresidualV.resize(0);
	this->ObjectLineV.resize(0);

	this->imgheight=osl.imgheight;
	this->imgwidth=osl.imgwidth;
	this->unique_instance_id_objectspacelines=osl.unique_instance_id_objectspacelines;

	for (unsigned int i=0;i<osl.ImageIdsV.size();i++)
			{
				double *Ptmp; Ptmp=(double*) malloc (12*sizeof(double));

					for(int j=0; j<12;j++)
					   {
						 Ptmp[j]=osl.PMatV[i][j];
						}

					this->PMatV.push_back(Ptmp);

					   	LineSegment2D lineseg(osl.ImageLineV[i].BeginPoint(),osl.ImageLineV[i].EndPoint());
					   	lineseg.Number()=osl.ImageLineV[i].Number();

					   	this->ImageLineV.push_back(lineseg);
					   	this->ImageIdsV.push_back(osl.ImageIdsV[i]);
						if (osl.ImageLine_dresidualV.size()==osl.ImageIdsV.size()) this->ImageLine_dresidualV.push_back(osl.ImageLine_dresidualV[i]);
					InteriorOrientation inttmp;
					ExteriorOrientation exttmp;

					if (!compute_int_ext_from_P(&Ptmp, imgheight, imgwidth, inttmp, exttmp)) {printf("error during conversion from P to Interior and Ext.ori"); exit(0);}

					this->IntOriV.push_back(inttmp);
					this->ExtOriV.push_back(exttmp);

					this->ObjectLineV.push_back(osl.ObjectLineV[i]);
			}
	this->ObjectLineCombined=osl.ObjectLineCombined.LineSegment3DRef();

}


ObjectSpaceLine::~ObjectSpaceLine() {
	// TODO Auto-generated destructor stub
	/*
	for (int k=0;k<PMatV.size();k++)
		if (PMatV[k]!=NULL) free(PMatV[k]);
		*/

}



void ObjectSpaceLine::computeQs()
{
	//printf("in compute Qs\n");

	QMatV.resize(0);
	for (unsigned int i=0;i<PMatV.size();i++)
	{
		//printf("image id:%d\n",ImageIdsV[i]);
		vector<double> Q;Q.resize(18);
		Q_from_P(PMatV[i],Q);
		QMatV.push_back(Q);

		//for (int k=0;k<12;k++) printf("P[%d]=%f \t ",k,PMatV[i][k]);

		//for (int k=0;k<18;k++) printf("Q[%d]=%f \t ",k,Q[k]);
		//printf("\n");
	}

}
void ObjectSpaceLine::add_ImageInfo(unsigned int Id, double *PMat, LineSegment2D &ImageLines)//, int imgheight)
{
	if ((imgheight==0) || (imgwidth==0)) {printf("first set the imgheight and width, it is necessary to define interior ext\n"); exit(0);}
	this->ImageIdsV.push_back(Id);


	double *Ptmp; Ptmp=(double*) malloc (12*sizeof(double));

	for(int j=0; j<12;j++)
	   {
		 Ptmp[j]=PMat[j];
		}


	   	//this->PMatV.push_back(PMat);
	   	this->PMatV.push_back(Ptmp);

	   	LineSegment2D lineseg(ImageLines.BeginPoint(),ImageLines.EndPoint());
	   	lineseg.Number()=ImageLines.Number();

	this->ImageLineV.push_back(lineseg);
	InteriorOrientation inttmp;
	ExteriorOrientation exttmp;

	if (!compute_int_ext_from_P(&Ptmp, imgheight, imgwidth, inttmp, exttmp)) {printf("error during conversion from P to Interior and Ext.ori"); exit(0);}

	this->IntOriV.push_back(inttmp);
	this->ExtOriV.push_back(exttmp);
	//this->imgheight=imgheight;
}

void ObjectSpaceLine::add_ImageInfo(unsigned int Id, double *PMat, ImageLines &iml, int lineid)//, int imgheight)
{
	LineSegment2D lineseg;

	if (iml.size()!=1) {printf("error in read data: more than one imagline added!");return;}

	for (unsigned int i_line=0;i_line<iml.size();i_line++){
		           //line coordinates

		           double ax=iml[i_line][0].X();
		           double ay=iml[i_line][0].Y();
		           double bx=iml[i_line][1].X();
		           double by=iml[i_line][1].Y();
		           /*
                   double ax=iml[i_line][0].Y();
	               double ay=(double) imgheight - iml[i_line][0].X();
	               double bx=iml[i_line][1].Y();
	               double by=(double) imgheight - iml[i_line][1].X();
	               */
		           lineseg=LineSegment2D (Position2D(ax,ay),Position2D(bx,by));
	}

	lineseg.Number()=lineid;

	double *Ptmp; Ptmp=(double*) malloc (12*sizeof(double));

		for(int j=0; j<12;j++)
		   {
			 Ptmp[j]=PMat[j];
			}

	this->add_ImageInfo(Id,Ptmp,lineseg);//, imgheight);

}
void ObjectSpaceLine::dump()
{

	printf("Size of of ImageIdsV: %d\n", ImageIdsV.size());

	for (int i=0; i< ImageIdsV.size();i++)
	{
		printf("length of line segment in image with Id %d: %.2f, lineid:%d\n", ImageIdsV[i], ImageLineV[i].Length(),ImageLineV[i].Number());
		printf("Line details:\n");
		//ImageLineV[i].Print();
		printf("Begin point: row=%.1f   col=%.1f\n",ImageLineV[i].BeginPoint().GetX(),ImageLineV[i].BeginPoint().GetY());
		printf("End   point: row=%.1f   col=%.1f\n\n",ImageLineV[i].EndPoint().GetX(),ImageLineV[i].EndPoint().GetY());

		printf("ObjectLine:\n");
		printf("Begin point: X=%.1f   Y=%.1f Z=%.1f\n",ObjectLineV[i].BeginPoint().GetX(),ObjectLineV[i].BeginPoint().GetY(),ObjectLineV[i].EndPoint().GetZ());
		printf("End   point: X=%.1f   Y=%.1f Z=%.1f\n\n",ObjectLineV[i].EndPoint().GetX(),ObjectLineV[i].EndPoint().GetY(),ObjectLineV[i].EndPoint().GetZ());
		printf("\n\n");

		printf("PMatrix for this image:\n");
		for (int k=0;k<12;k++) printf("%.2f ",PMatV[i][k]);
		printf("\n");
		printf("Interior Ori for this image:\n");
		IntOriV[i].Print();

		printf("Exterior Ori for this image:\n");
		ExtOriV[i].Print();
	}
	printf("ObjectLineCombined:\n");
	printf("Begin point: X=%.1f   Y=%.1f Z=%.1f\n",ObjectLineCombined.BeginPoint().GetX(),ObjectLineCombined.BeginPoint().GetY(),ObjectLineCombined.EndPoint().GetZ());
	printf("End   point: X=%.1f   Y=%.1f Z=%.1f\n\n",ObjectLineCombined.EndPoint().GetX(),ObjectLineCombined.EndPoint().GetY(),ObjectLineCombined.EndPoint().GetZ());
	printf("\n\n");


}

bool ObjectSpaceLine::compute_corresponding_objectsegments(bool after_line_adjustment)
{
//procedure: per image involved we have one line segment stored (if one image has more line segments, they are listed multiple times)
//we compute a viewing ray as defined by the proj.center and each linesegement end point. this viewing ray is intersected tiwht the actual 3DLine
//the intersection points define endpoints of 3DLinesegments in objectspace

if (!this->ImageIdsV.size() || !this->PMatV.size() ||!this->ImageLineV.size()) {printf("no image info in this instance of ObjectSpaceLine\n"); return 0;}
if ((this->ImageIdsV.size() != this->PMatV.size()) || (this->PMatV.size() != this->ImageLineV.size())) {printf("Bug detected: image info vectors have not the same size!\n"); return 0;}

//printf("this ObjectSpaceLine has %d images involved\n",ImageLineV.size());

for (int i=0; i<this->ImageLineV.size();i++)
{
	Position2D Begin=this->ImageLineV[i].BeginPoint();
	Position2D End=this->ImageLineV[i].EndPoint();

	Line3D rayBegin, rayEnd;
	if (!ViewingRay(this->IntOriV[i], this->ExtOriV[i], Begin, rayBegin)) {printf("error in viewing ray computation!\n"); return 0;}
	if (!ViewingRay(this->IntOriV[i], this->ExtOriV[i], End, rayEnd)) {printf("error in viewing ray computation!\n"); return 0;}


	Position3D Begin3,End3;
	if (!Intersection2Lines (*this, rayBegin,  Begin3)) {printf("Rays do not intersect (beginseg ray)\n");  return 0;}
	if (!Intersection2Lines (*this, rayEnd, End3)) {printf("Rays do not intersect (endseg ray)\n"); return 0;}

	//IF after adjustment: Project the old start-/endpoints onto *this and use it further
	if (after_line_adjustment)
	{
		Position3D Begin3old=this->ObjectLineV[i].BeginPoint();
		Position3D End3old=this->ObjectLineV[i].EndPoint();

		//printf("Begin3D before intersection: %e   %e   %e\n",Begin3old.GetX(),Begin3old.GetY(), Begin3old.GetZ());
		//printf("End3D before intersection: %e   %e   %e\n",End3old.GetX(),End3old.GetY(), End3old.GetZ());

		Line3D tmpthisline=*this;
		Begin3=tmpthisline.Project(Begin3old);
		End3=tmpthisline.Project(End3old);

		//printf("Begin3D after intersection: %e   %e   %e\n",Begin3.GetX(),Begin3.GetY(), Begin3.GetZ());
		//printf("End3D after intersection: %e   %e   %e\n",End3.GetX(),End3.GetY(), End3.GetZ());

	}
	//printf("Begin3:%.2f  %.2f   %.2f\n",Begin3.GetX(),Begin3.GetY(),Begin3.GetZ());
	//printf("End3:%.2f  %.2f   %.2f\n",End3.GetX(),End3.GetY(),End3.GetZ());

	//Test: Use an independent method (PMatrix) to forward project the points into the image and compare the given point
	//!!!THIS is only a computational check, since we have no redundant informatioin
	matrix *P;
	P_convert(this->PMatV[i], &P);
	xelem Begin3Xelem,End3Xelem;
	Begin3Xelem.id=0;
	Begin3Xelem.X=Begin3.GetX();Begin3Xelem.Y=Begin3.GetY();Begin3Xelem.Z=Begin3.GetZ();

	End3Xelem.id=0;
	End3Xelem.X=End3.GetX();End3Xelem.Y=End3.GetY();End3Xelem.Z=End3.GetZ();

	homo2 Begin2homo=::Project(P, &Begin3Xelem); //Project function performs normalization.
	homo2 End2homo=::Project(P, &End3Xelem);

	mat_free(P);

	//do the same with the Maplib (debug)

	ImagePoint image_pointbegin=project3D2img(Begin3, &this->IntOriV[i],&this->ExtOriV[i]);
	ImagePoint image_pointend=project3D2img(End3, &this->IntOriV[i],&this->ExtOriV[i]);


	//compute differences for the comput. check: differences between the original edge points and the projections obtained
	//from Pmat and from map_lib. all must be zero.
	//Attention: in map_lib x=row, y=col, in pmat:y=row, x=col!!!
	//only to be done when using the stereo intersections, not after adjustment (then it can not match by definition)
	//however, the implementation check (maplib vs. pmat) can be done anyhow

	double diff_beginx_pmat=Begin.GetX()-Begin2homo.y;
	double diff_beginy_pmat=Begin.GetY()-Begin2homo.x;
	double diff_endx_pmat=End.GetX()-End2homo.y;
	double diff_endy_pmat=End.GetY()-End2homo.x;

	double diff_beginx_maplib=Begin.GetX()-image_pointbegin.GetX();
	double diff_beginy_maplib=Begin.GetY()-image_pointbegin.GetY();
	double diff_endx_maplib=End.GetX()-image_pointend.GetX();
	double diff_endy_maplib=End.GetY()-image_pointend.GetY();

	double diff_beginx_maplib_pmat=Begin2homo.y-image_pointbegin.GetX();
	double diff_beginy_maplib_pmat=Begin2homo.x-image_pointbegin.GetY();
	double diff_endx_maplib_pmat=End2homo.y-image_pointend.GetX();
	double diff_endy_maplib_pmat=End2homo.x-image_pointend.GetY();


	if (!after_line_adjustment)
	{
	//the tolerance of one pix is quite large, normally we only need to consider rounding errors in 1/100 pix
	if ((abs(diff_beginx_pmat) > 1) || (abs(diff_beginy_pmat) > 1) || (abs(diff_endx_pmat) > 1) || (abs(diff_endy_pmat) > 1))
		{	printf("computation test failed: forward projection with Pmatrix does not yield the same image point as measured:\n");
			printf("Desired: Imagepoint begin x (row)=%.2f  y (col)=%.2f\n",Begin.GetX(),Begin.GetY());
			printf("Desired: Imagepoint end   x (row)=%.2f  y (col)=%.2f\n",End.GetX(),End.GetY());

			printf("projection: Imagepoint: begin: x (col)=%.2f y (row)=%.2f\n",Begin2homo.x,Begin2homo.y);
		    printf("projection: Imagepoint: end:   x (col)=%.2f y (row)=%.2f\n",End2homo.x,End2homo.y);


			return(0);
		}

	if ((abs(diff_beginx_maplib) > 1) || (abs(diff_beginy_maplib) > 1) || (abs(diff_endx_maplib) > 1) || (abs(diff_endy_maplib) > 1))
		{	printf("computation test failed: forward projection with Mapping Lib does not yield the same image point as measured:\n");
		    printf("Desired: Imagepoint begin x (row)=%.2f  y (col)=%.2f\n",Begin.GetX(),Begin.GetY());
		    printf("Desired: Imagepoint end   x (row)=%.2f  y (col)=%.2f\n",End.GetX(),End.GetY());

		    printf("projection: Imagepoint begin x (row)=%.2f  y (col)=%.2f\n",image_pointbegin.GetX(),image_pointbegin.GetY());
			printf("projection: Imagepoint end   x (row)=%.2f  y (col)=%.2f\n",image_pointend.GetX(),image_pointend.GetY());
			return(0);
		}

	//End Test wrt to image points
	}

	if ((abs(diff_beginx_maplib_pmat) > 1) || (abs(diff_beginy_maplib_pmat) > 1) || (abs(diff_endx_maplib_pmat) > 1) || (abs(diff_endy_maplib_pmat) > 1))
			{	printf("computation test failed: forward projection with Mapping Lib not the same as with PMat:\n");
				printf("PMat: Imagepoint: begin: x (col)=%.2f y (row)=%.2f\n",Begin2homo.x,Begin2homo.y);
				printf("PMat: Imagepoint: end:   x (col)=%.2f y (row)=%.2f\n",End2homo.x,End2homo.y);

			    printf("maplib: Imagepoint begin x (row)=%.2f  y (col)=%.2f\n",image_pointbegin.GetX(),image_pointbegin.GetY());
				printf("maplib: Imagepoint end   x (row)=%.2f  y (col)=%.2f\n",image_pointend.GetX(),image_pointend.GetY());
/*
				printf("i=%d, ImageIdsV[i]=%d\n",i,ImageIdsV[i]);
				printf("DEBUG: dump this line!\n");
			    this->dump();

			    printf("debug: compute int and ext ori from PMatV[i] and compare to original ones\n");

			    InteriorOrientation inttmp;
				ExteriorOrientation exttmp;

				if (!compute_int_ext_from_P(&PMatV[i], imgheight, imgwidth, inttmp, exttmp)) {printf("error during conversion from P to Interior and Ext.ori"); exit(0);}
				printf("int/ext ori from P computed:\n");
				inttmp.Print();
				exttmp.Print();

				printf("int/ext from osl:\n");
				this->IntOriV[i].Print();
				this->ExtOriV[i].Print();
*/
				printf("DEBUG: contiunue anyhow!\n");
				//return(0);
			}


	//skalars of the begin and endpoint

	double scalar_begin=this->Scalar (Begin3);
	double scalar_end=this->Scalar (End3);

	//printf("scalar_begin=%.2f, end=%.2f\n", scalar_begin, scalar_end);

	//test: the point as created from looking at the line3d at the scale must be identical with the intersection point
	Position3D beginobjsegtest=this->Position(scalar_begin);
	Position3D endobjsegtest=this->Position(scalar_end);

	double distbeg=Begin3.Distance(beginobjsegtest);
	double distend=End3.Distance(endobjsegtest);
	if ((abs(distbeg) > 1E-6) || (abs(distend) > 1E-6))
	{	printf("computation test failed: distances too large:\n");
		printf("Distance between beginobjseg as from intersection and as from re-computing from scalar:%.2f\n",distbeg);
		printf("Distance between endobjseg as from intersection and as from re-computing from scalar:%.2f\n",distend);
		return(0);
	}

	LineSegment3D LineS=LineSegment3D(Begin3,End3);//better the testpoints

	//ATTENTION: if after adjust DO NOT PUSH BACK!!! THE instance is already there!
	if (!after_line_adjustment) this->ObjectLineV.push_back(LineS);
	else this->ObjectLineV[i]=LineS;

}//ImageLineV

if (!compute_ObjectLineCombined()) {printf("Error in compute_ObjectLineCombined()");return 0;}
//else printf("Computed the ObjectLineCombined\n");/*
/*
	// if (after_line_adjustment)
	// {
	//DEBUG (dataset EnschSouthEast: write out only a specific line)

		 Position3D Begin3fin= ObjectLineCombined.BeginPoint();
		 Position3D End3fin= ObjectLineCombined.EndPoint();

		 printf("DEBUG: ONLY KEEP ONE SPECIAL LINE\n");
		 if (End3fin.GetZ()<50) return 0;
	// }

*/


return 1;
}

bool ObjectSpaceLine::compute_ObjectLineCombined()
{
vector<LineSegment3D>::iterator OL;
if (!this->ObjectLineV.size()) {printf("in ObjectSpaceLine::compute_ObjectLineCombined(): no ObjectSpaceLines available"); return(0);}

//Idea: collect the skalars from the ObjctLines (sgments) wrt to this Line3D. The min and max will then used for the outerpoints
//The problem is: if the ObjectLineV contains lines which have been but there after finding pairs (ObjectSpaceLines::FindMultipleViews())
//and if there was no adjustment afterwars, the lines can  be quite different (because of random errors), thus: if more than
//two are in this one and if the angle differene is too large, raise a warning.
if (this->ObjectLineV.size() > 2)
{
	for (OL=this->ObjectLineV.begin()+2;OL<this->ObjectLineV.end();OL++)
	{
		double ang_deg=fabs(Angle2Lines(*this->ObjectLineV.begin(), *OL)/PI*180);
		if(ang_deg > 5)
			//also check 180deg (it is ok if larger than 180-thresh
		 if (ang_deg < (180-5))
			{
				printf("%d lines in ObjectLineV-vector, and their deviate with a quite big angle (%.2f). The existing one remains. Better adjust!\n",ObjectLineV.size(),ang_deg);
				return 0;
			}
	}
}

double min_scalar=1E16;
double max_scalar=-1E16;


for (OL=this->ObjectLineV.begin();OL!=this->ObjectLineV.end();OL++)
{
	double scalar_begin=this->Scalar(OL->BeginPoint());
    double scalar_end=this->Scalar(OL->EndPoint());
    min_scalar=MIN(min_scalar,scalar_begin);
    min_scalar=MIN(min_scalar,scalar_end);
    max_scalar=MAX(max_scalar,scalar_begin);
    max_scalar=MAX(max_scalar,scalar_end);

    //printf("debug: OL print\n");
    //OL->Print();
}

this->ObjectLineCombined=LineSegment3D(this->Position(min_scalar),this->Position(max_scalar));
//printf("debug: combined line:\n");
//this->ObjectLineCombined.Print();

return 1;

}

bool ObjectSpaceLine::check_if_ObjectLineCombined_in_DataBounds(DataBounds3D bounds)
{
	//if (bounds.Inside(ObjectLineCombined.BeginPoint())) return 1;
	//if (bounds.Inside(ObjectLineCombined.EndPoint())) return 1;
	if (bounds.Inside(ObjectLineCombined.MiddlePoint())) return 1;

	return 0;
}

int ObjectSpaceLine::No_of_unique_ImageIds()
{
	vector<unsigned int> uniqueids; uniqueids.resize(0);
	for (int i=0;i<ImageIdsV.size();i++)
		if(!check_value_in_vec(uniqueids,ImageIdsV[i])) uniqueids.push_back(ImageIdsV[i]);

	return uniqueids.size();
}

bool ObjectSpaceLine::LSA_line()
{
	//Here we are using the Gauss-Markov Model, i.e. the Pluecker constraint is introduced as a soft constraint.
	//This is much easier and stable to implement and should give the same result, compared to Gauss-Helmert

	int MaximumIter=20;
	double sig_phi=2.0*3.1415/180;//for the estimation of covariance mat for the image lines
	double sig_d=1.5; //in pixel
	double sig_constraint=1E-4; //for the soft constraint on Pluecker.

	if(this->PMatV.size() <3) {//3
		printf("LSA with less than 3 input lines does not make sense!\n");
		return 0;
	}
	//for the non-linear case we need an approximation. For this, a 3D line is computed from the edge in the first and in the last vector in this member.
	//later if necessary it can be extended in the sense that the approximation is computed from the views having the largest spatial distance
	//1. firstly we compute the Q-Matrices
	this->computeQs();

	vector<double> LPlueck;LPlueck.resize(6);

	unsigned int lastidx=this->PMatV.size()-1;

	//Init the ImageLine_dresidualV: it has the same length as the ImageLineV, because at the respective position in the vector the image distance residual will be stored
	this->ImageLine_dresidualV.resize(ImageLineV.size());

	//printf("Begin LSA: OSL id:%d\n",this->unique_instance_id_objectspacelines);
	/*
	printf("ids of image lines involved:\n");
	for (int i=0;i< this->ImageLineV.size();i++)
		{
			printf("%d\n",this->ImageLineV[i].Number());
		}
    */
	//Construct the Pluecker coord from this line3D
/**/	Vector3D dir=this->Direction();
	Position3D pos=this->Position(0);
	Vector3D pos_vec(pos.X(),pos.Y(),pos.Z());
	Vector3D prod=pos_vec.VectorProduct(dir);
	LPlueck[0]=dir.X();
	LPlueck[1]=dir.Y();
	LPlueck[2]=dir.Z();
	LPlueck[3]=prod.X();
	LPlueck[4]=prod.Y();
	LPlueck[5]=prod.Z();

	//printf("Pluecker from Euclidean direct:\n");
	//for (int p=0;p<6;p++) printf("%.4f\n",LPlueck[p]);

	//constraint?!
	double constr=LPlueck[0]*LPlueck[3]+LPlueck[1]*LPlueck[4]+LPlueck[2]*LPlueck[5];
	//printf("constr:%.4f\n",constr);

	//TTEST start
		//debug: use l=QL to chck the correctness of Q
	/*
	for  (int i=0; i<this->PMatV.size();i++)
	{   printf("original image edges before approx: image id: %d  :\n",this->ImageIdsV[i]);

		printf("Edge id: %d\n",this->ImageLineV[i].Number());
		Position2D beg=this->ImageLineV[i].BeginPoint();
		Position2D end=this->ImageLineV[i].EndPoint();
		beg.SwapXY();
		end.SwapXY();
		LineSegment2D linetmp(beg,end);
		linetmp.Print();
		vector<double> lreproL;lreproL.resize(3);
		lQL(this->QMatV[i],LPlueck,lreproL);
		printf("reprojected line (attention: a is cosphi, b is sinphi):a %.5f, b %.5f, c %.5f, (d=%.5f)\n",lreproL[0],lreproL[1],lreproL[2],fabs(lreproL[2]));
	}
	*/
	//TST END


	//Put all Qs in one big double *
	double *Q=new double [18*this->QMatV.size()];
	for (int j=0;j<this->QMatV.size();j++)
		for(int i=0;i<18;i++)
			Q[j*18+i]=this->QMatV[j][i];

	int n_lines=this->ImageLineV.size()*3; //Number of obs=3* no of lines
	int n_constraint=1; //the constraint is introduced as an artifical observation
	int n_final=n_lines+n_constraint;

	//Use the same naming of variables like Tian, but the conversion to Matlab is not necessary, only use MathTools matrix functions
	matrix *L;//Pluecker Line
	matrix *l;//image lines in cosphi,sinphi,d representation
	matrix *Cll,*iCll;//image line covariance matrices, and its inverse
	matrix *l_final; //including the artificial obs
	matrix *Cll_final,*iCll_final;//...

	//copy the data to the matrices
	L=mat_alloc(6,1); for(int i=0;i<6;i++) L->m[i][0]=LPlueck[i];
	l=mat_alloc(n_lines,1);
	l_final=mat_alloc(n_final,1);

	Cll=mat_alloc(n_lines,n_lines);mat_set(Cll,0);
	iCll=mat_alloc(n_lines,n_lines);mat_set(Cll,0); //since Cll is a block diagonal matrix, its inverse is the block diagonal matrix (each block the single inverse)

	Cll_final=mat_alloc(n_final,n_final);mat_set(Cll_final,0);
	iCll_final=mat_alloc(n_final,n_final);mat_set(iCll_final,0);

	//attention: we need to swap x and y because of the different definition of the image coord. system in the map lib and in the proj. geometry code
	for (int j=0;j<n_lines/3;j++) {
		Position2D beg=this->ImageLineV[j].BeginPoint();
		Position2D end=this->ImageLineV[j].EndPoint();
		beg.SwapXY();
		end.SwapXY();
		LineSegment2D linetmp(beg,end);

		//ATTENTION: a is cosphi, b is sinphi!, c is -d
		l->m[j*3+0][0]=linetmp.Getcosphi();
		l->m[j*3+1][0]=linetmp.Getsinphi();
		l->m[j*3+2][0]=-linetmp.GetDisto();

	}

	//l_final: upper part from lines, lower part (last entry):0 (already set at initialisation), so just copy this line part to the beginning
	mat_copy(l,l_final,0,0);
	//compute the cov. mat for each line, see Diss. Heuel, p198

/*
	Cll=mat_alloc(n_lines,n_lines);mat_set(Cll,0);
	printf("DEBUG: Cll=eye\n");
	mat_eye(Cll);
	iCll=mat_clone(Cll);
*/
/**/

	//compute the cov. mat for each line, see Diss. Heuel, p198
		//Sig for the angle and dist

		matrix *SIG;SIG=mat_alloc(3,3);mat_set(SIG,0);
		SIG->m[0][0]=sig_phi*sig_phi;
		SIG->m[2][2]=sig_d*sig_d;
		//mat_print("SIG",SIG);

		//allocate the help matrices
		matrix *T,*TT,*R,*RT;
		T=mat_alloc(3,3);TT=mat_alloc(3,3);mat_eye(T);
		R=mat_alloc(3,3);RT=mat_alloc(3,3);mat_eye(R);

		Cll=mat_alloc(n_lines,n_lines);
		iCll=mat_alloc(n_lines,n_lines);
		//printf("debug: Cll only as a diag. matrix");
		for (int j=0;j<n_lines/3;j++) {
/**/
			double var_phi=sig_phi*sig_phi;
				double var_d=sig_d*sig_d;
				double a,b,tab;

					 a=l->m[j*3+0][0];
					 b=l->m[j*3+1][0];
					 //SWAP XY
					 tab=-b*this->ImageLineV[j].MiddlePoint().GetY()+a*this->ImageLineV[j].MiddlePoint().GetX();
					 matrix *tmpc=mat_alloc(3,3);

					          tmpc->m[0][0]=b*b*var_phi;
					          tmpc->m[0][1]=-a*b*var_phi;
					          tmpc->m[0][2]=tab*b*var_phi;
					          tmpc->m[1][0]=-a*b*var_phi;
					          tmpc->m[1][1]=a*a*var_phi;
					          tmpc->m[1][2]=-tab*a*var_phi;
					          tmpc->m[2][0]=tab*b*var_phi;
					          tmpc->m[2][1]=-tab*a*var_phi;
					          tmpc->m[2][2]=tab*tab*var_phi+var_d;


			//mat_print("tmpc",tmpc);
/**/
			//if only diagonal: start
			//ATTENTION: a is cosphi, b is sinphi!, c is -d
/*
			double a=l->m[j*3+0][0];
			double b=l->m[j*3+1][0];
			matrix *tmpc=mat_alloc(3,3);
			tmpc->m[0][0]=b*b*sig_phi*sig_phi; //see error prop.
			tmpc->m[1][1]=a*a*sig_phi*sig_phi;
			tmpc->m[2][2]=sig_d*sig_d;
			 //mat_print("cll",tmpc);
*/

			 //printf("determinant of tmpc (Cll): %e, rank=%d\n",mat_det(tmpc),mat_rank(tmpc));
			//matrix *itmpc=mat_clone(tmpc);



			 matrix * itmpc=mat_invert_opencv(tmpc);



			//mat_print("icll",itmpc);

			  //printf("begin of TEST invert\n");
			 			//matrix *Ninvinv=mat_invert_new(Ninv);
			 			matrix *cllinvinv=mat_invert_opencv(itmpc);
			 			//mat_print("tmpc",tmpc);
			 			//mat_print("cllinvinv",cllinvinv);
			 			if (!CheckMatrixIdentityRelativeError(cllinvinv,tmpc,0.01)) {printf("ATTENTION: Error in invert of Cll; stop here\n!");return 0;}
			 			mat_free(cllinvinv);

			 			//printf("END of TEST invert\n");

			mat_copy(tmpc,Cll,j*3,j*3);
			mat_copy(itmpc,iCll,j*3,j*3);
			mat_free(tmpc);mat_free(itmpc);

			}
		mat_free(T);mat_free(TT);mat_free(R);mat_free(RT);


/**/


	//mat_print("Cll",Cll);
	//mat_print("iCll",iCll);
    //printf("determinant of Cll: %e, rank=%d\n",mat_det(Cll),mat_rank(Cll));

	//Cll_final: upper part from lines, lower part (last entry):SIG_Constraint^2
	double var_constraint=sig_constraint*sig_constraint;
	mat_copy(Cll,Cll_final,0,0);
	mat_set_at(Cll_final,n_final-1,n_final-1,var_constraint);

	//iCll similar
	mat_copy(iCll,iCll_final,0,0);
	mat_set_at(iCll_final,n_final-1,n_final-1,1/var_constraint);

	//mat_print("Cll_final",Cll_final);
	//mat_print("iCll_final",iCll_final);
	//TODO!: currently if we do return 0 we do not free the pointers! later we should more split all the differnt processes into more modules



	 int i,j;
	   double *Avec=new double[n_lines*6];
	   double a,b,c,d;
	   //!TODO: remove unneeded matrices
	   //matrix *H,*HT,*B,*iB,*C,*D,*E,*Ch,*F,*G,*DL,*v,*vT,*I,*iI,,*eCLL,*Cvv;
	   matrix *Dl,*A,*AT,*Dl_final,*A_final, *AT_final, *N, *Ninv, *AtP, *v,*l_adjusted;
	   double test1_valzero;
	   bool LSA_fail=0;
	   double former_v_max;
	   bool found=0;

	   for(i=0;i<MaximumIter;i++)  //200
	   {

		   Dl=mat_alloc(n_lines,1);
		   //for analytical A computation: allocate here AT (A then in loop), for numerical diff A: allocate A here
		   A=mat_alloc(n_lines,6);
		   //AT=mat_alloc(6,n_lines);
		   //DL=mat_alloc(6,1);

		   Dl_final=mat_alloc(n_final,1);


		   A_final=mat_alloc(n_final,6);
		   //AT_final=mat_alloc(6,n_final);




	      for(j=0;j<n_lines/3;j++)
	      {
	         a=Q[18*j+0]*L->m[0][0]+Q[18*j+1]*L->m[1][0]+Q[18*j+2]*L->m[2][0]+Q[18*j+3]*L->m[3][0]+Q[18*j+4]*L->m[4][0]+Q[18*j+5]*L->m[5][0];
	         b=Q[18*j+6]*L->m[0][0]+Q[18*j+7]*L->m[1][0]+Q[18*j+8]*L->m[2][0]+Q[18*j+9]*L->m[3][0]+Q[18*j+10]*L->m[4][0]+Q[18*j+11]*L->m[5][0];
	         c=Q[18*j+12]*L->m[0][0]+Q[18*j+13]*L->m[1][0]+Q[18*j+14]*L->m[2][0]+Q[18*j+15]*L->m[3][0]+Q[18*j+16]*L->m[4][0]+Q[18*j+17]*L->m[5][0];
	         d=sqrt(a*a+b*b);

	         //Using LQl, because later Q[xx] should be removed
	         vector<double> l_back;
	         lQL(this->QMatV[j], L, l_back);
/*
	         printf("j=%d\n, l:%.3f, %.3f, %.3f\n",j,l->m[3*j][0],l->m[3*j+1][0],l->m[3*j+2][0]);
	         printf("rueckprojez mit Q[xxx] %.3f  %.3f  %.3f\n",a/d,b/d,c/d);
	         printf("rueckprojez mit lQL %.3f  %.3f  %.3f\n",l_back[0],l_back[1],l_back[2] );
*/
	         double A_sign=1;

	         //Dl
	         if(a*l->m[3*j][0]>0)
	         {
	        	 //printf("substr\n");
	           Dl->m[3*j][0]=l->m[3*j][0]-a/d;
	           Dl->m[3*j+1][0]=l->m[3*j+1][0]-b/d;
	           Dl->m[3*j+2][0]=l->m[3*j+2][0]-c/d;
	           A_sign=1;

	           //also adding the residual distance between the observed and the reprojected image line to the osl-instanc
	           //it will be used later during removal of wrong lines
	           this->ImageLine_dresidualV[j]=fabs(l->m[3*j+2][0]-c/d);
/*
	           Avec[18*j]=(b*b*Q[18*j]-a*b*Q[18*j+6])/d/d/d;
	           Avec[18*j+1]=(b*b*Q[18*j+1]-a*b*Q[18*j+7])/d/d/d;
	           Avec[18*j+2]=(b*b*Q[18*j+2]-a*b*Q[18*j+8])/d/d/d;
	           Avec[18*j+3]=(b*b*Q[18*j+3]-a*b*Q[18*j+9])/d/d/d;
	           Avec[18*j+4]=(b*b*Q[18*j+4]-a*b*Q[18*j+10])/d/d/d;
	           Avec[18*j+5]=(b*b*Q[18*j+5]-a*b*Q[18*j+11])/d/d/d;
	           Avec[18*j+6]=(a*a*Q[18*j+6]-a*b*Q[18*j])/d/d/d;
	           Avec[18*j+7]=(a*a*Q[18*j+7]-a*b*Q[18*j+1])/d/d/d;
	           Avec[18*j+8]=(a*a*Q[18*j+8]-a*b*Q[18*j+2])/d/d/d;
	           Avec[18*j+9]=(a*a*Q[18*j+9]-a*b*Q[18*j+3])/d/d/d;
	           Avec[18*j+10]=(a*a*Q[18*j+10]-a*b*Q[18*j+4])/d/d/d;
	           Avec[18*j+11]=(a*a*Q[18*j+11]-a*b*Q[18*j+5])/d/d/d;
	           Avec[18*j+12]=(d*d*Q[18*j+12]-a*c*Q[18*j]-b*c*Q[18*j+6])/d/d/d;
	           Avec[18*j+13]=(d*d*Q[18*j+13]-a*c*Q[18*j+1]-b*c*Q[18*j+7])/d/d/d;
	           Avec[18*j+14]=(d*d*Q[18*j+14]-a*c*Q[18*j+2]-b*c*Q[18*j+8])/d/d/d;
	           Avec[18*j+15]=(d*d*Q[18*j+15]-a*c*Q[18*j+3]-b*c*Q[18*j+9])/d/d/d;
	           Avec[18*j+16]=(d*d*Q[18*j+16]-a*c*Q[18*j+4]-b*c*Q[18*j+10])/d/d/d;
	           Avec[18*j+17]=(d*d*Q[18*j+17]-a*c*Q[18*j+5]-b*c*Q[18*j+11])/d/d/d;
*/
	         }
	         else
	         {
	        	 //printf("adding\n");
	        	 Dl->m[3*j][0]=l->m[3*j][0]+a/d;
	        	 Dl->m[3*j+1][0]=l->m[3*j+1][0]+b/d;
	        	 Dl->m[3*j+2][0]=l->m[3*j+2][0]+c/d;
	        	 A_sign=-1;

	        	 this->ImageLine_dresidualV[j]=fabs(l->m[3*j+2][0]+c/d);
/*
	           Avec[18*j]=-(b*b*Q[18*j]-a*b*Q[18*j+6])/d/d/d;
	           Avec[18*j+1]=-(b*b*Q[18*j+1]-a*b*Q[18*j+7])/d/d/d;
	           Avec[18*j+2]=-(b*b*Q[18*j+2]-a*b*Q[18*j+8])/d/d/d;
	           Avec[18*j+3]=-(b*b*Q[18*j+3]-a*b*Q[18*j+9])/d/d/d;
	           Avec[18*j+4]=-(b*b*Q[18*j+4]-a*b*Q[18*j+10])/d/d/d;
	           Avec[18*j+5]=-(b*b*Q[18*j+5]-a*b*Q[18*j+11])/d/d/d;
	           Avec[18*j+6]=-(a*a*Q[18*j+6]-a*b*Q[18*j])/d/d/d;
	           Avec[18*j+7]=-(a*a*Q[18*j+7]-a*b*Q[18*j+1])/d/d/d;
	           Avec[18*j+8]=-(a*a*Q[18*j+8]-a*b*Q[18*j+2])/d/d/d;
	           Avec[18*j+9]=-(a*a*Q[18*j+9]-a*b*Q[18*j+3])/d/d/d;
	           Avec[18*j+10]=-(a*a*Q[18*j+10]-a*b*Q[18*j+4])/d/d/d;
	           Avec[18*j+11]=-(a*a*Q[18*j+11]-a*b*Q[18*j+5])/d/d/d;
	           Avec[18*j+12]=-(d*d*Q[18*j+12]-a*c*Q[18*j]-b*c*Q[18*j+6])/d/d/d;
	           Avec[18*j+13]=-(d*d*Q[18*j+13]-a*c*Q[18*j+1]-b*c*Q[18*j+7])/d/d/d;
	           Avec[18*j+14]=-(d*d*Q[18*j+14]-a*c*Q[18*j+2]-b*c*Q[18*j+8])/d/d/d;
	           Avec[18*j+15]=-(d*d*Q[18*j+15]-a*c*Q[18*j+3]-b*c*Q[18*j+9])/d/d/d;
	           Avec[18*j+16]=-(d*d*Q[18*j+16]-a*c*Q[18*j+4]-b*c*Q[18*j+10])/d/d/d;
	           Avec[18*j+17]=-(d*d*Q[18*j+17]-a*c*Q[18*j+5]-b*c*Q[18*j+11])/d/d/d;
*/
	         }

	         //Amatrix: row according to l, col: L0 to L5
	         /**/
				 for (int Lindex=0; Lindex<6; Lindex ++)
				 {
					 double dl1DLi,dl2DLi,dl3DLi;
					 //numerical diff.
					 dlQdLi_num(this->QMatV[j], L, Lindex, &dl1DLi, &dl2DLi, &dl3DLi);//Jacobian (only with respect to Li) by numercial diff,

					 A->m[3*j][Lindex]=A_sign*dl1DLi;
					 A->m[3*j+1][Lindex]=A_sign*dl2DLi;
					 A->m[3*j+2][Lindex]=A_sign*dl3DLi;
				 }
			 /**/

	         //for analytic sol.: Avec --> A
				 /*
	    	 for (int Lindex=0; Lindex<6; Lindex ++)
	    				 {A->m[3*j][Lindex]=Avec[18*j+Lindex];
						 A->m[3*j+1][Lindex]=Avec[18*j+Lindex+6];
						 A->m[3*j+2][Lindex]=Avec[18*j+Lindex+12];
	    				 }
	    				 */

	     }

	      AT=mat_transpose_new(A); //for num. solution



	      //Avec->A
	      /*ATTENTION: ERROR IN Conversion Avec-->AT?!?!?!
	      for(int c=0;c<6;c++)
	           for (int r=0;r<n_lines;r++)
	      	      		AT->m[c][r]=Avec[c*6+r];
						//AT->m[c][r]=Avec[r*6+c];
	       */
	      //A=mat_transpose_new(AT); //for analyt. solutoin

	      //printf("Avec:\n");
	      //for (int r=0;r<n*6;r++) printf("%.5f\n",Avec[r]);

	      //printf("AT:\n");
	      //mat_print(AT);

	      /**/



	     //mat_print("A (only lines)",A);
	     //mat_print("AT(only lines)",AT);



	      //Final Dl, A, AT
	      //Dl_final: upper part from the lines, the last one is 0-constraint(from L pluecker)
	      mat_copy(Dl,Dl_final,0,0);
	      mat_set_at(Dl_final,n_final-1,0,-(L->m[0][0]*L->m[3][0]+L->m[1][0]*L->m[4][0]+L->m[2][0]*L->m[5][0]));

	      //mat_print("Dl_final (last element: constraint",Dl_final);
		  double largest_Dl=mat_largest_abs_element(Dl_final);
		  if (largest_Dl < 1E-3)
		  {
			  printf("in iteration: %d: the error is already minimized: largest error is %e; stop after the next iteration\n",i,largest_Dl);
			  found=1;LSA_fail=0;
		  }

	      //A_final: the last row contains the respective derivativew wrt L plueck
	      mat_copy(A,A_final,0,0);
	      //deriv wrt to L[0]=L[3] etc
	      mat_set_at(A_final,n_final-1,0,L->m[3][0]);
	      mat_set_at(A_final,n_final-1,1,L->m[4][0]);
	      mat_set_at(A_final,n_final-1,2,L->m[5][0]);
	      mat_set_at(A_final,n_final-1,3,L->m[0][0]);
	      mat_set_at(A_final,n_final-1,4,L->m[1][0]);
	      mat_set_at(A_final,n_final-1,5,L->m[2][0]);

	      AT_final=mat_transpose_new(A_final);

	      //mat_print("A final (last element: deriv. wrt constraints",A_final);

	      //Compute N, Ninv, AtP

	     AtP=mat_prod_new(AT_final,iCll_final);


			//printf("Computing N=A^TPA..\n");
			N=mat_prod_new(AtP,A_final);
			//mat_print("\nN",N);

			//printf("rank (N):%d\n",mat_rank(N));
			//printf("rank (A):%d\n",mat_rank(A));

			if  (mat_rank(N)!=mat_rank(A)) {printf("Error: rank (N) != rank (A)!->do not use LSA result\n"); LSA_fail=1;}

			//printf("Det of N:%e\n",mat_det(N));
			//mat_print("\nN",N);

			//printf("begin inversion of N\n");
			//Ninv=mat_invert_new(N);
			//Ninv=mat_invert_lapack(N);
			Ninv=mat_invert_opencv(N);

			//printf("begin of TEST invert\n");
			//matrix *Ninvinv=mat_invert_lapack(Ninv);
			matrix *Ninvinv=mat_invert_opencv(Ninv);

			//mat_print("N",N);
			//mat_print("Ninvinv",Ninvinv);
			/*
			if (!CheckMatrixIdentityRelativeError(N,Ninvinv,0.01)) {printf("ATTENTION: Error in invert of N...\n!");}
			mat_free(Ninvinv);
			*/

			//printf("END of TEST invert\n");


			//call LSA_Gauss_Markov
			double k=50;

			//mat_print("L vor LSA",L);

			//Attention: if a blunder is detected during LSA: LSA_fail=1, because this is a hint that the combination of stereollines was not correct
			if (!LSA_Gauss_Markov(A_final, iCll_final, AtP, Ninv, Dl_final, k, L , &v, &test1_valzero,1)) LSA_fail=1;

			//mat_print("L nach LSA",L);

			l_adjusted=mat_add_new(l_final,v);

			//copy the adjusted part from l referrring to the image lines to l
			/*
			for (int o=0; o<n_lines;o++)
				l->m[o][0]=l_adjusted->m[o][0];
			*/

			double largest_v_abs=mat_largest_abs_element(v);
			//printf("Largest v value: %f\n", largest_v_abs);

			//if difference is larger than k/4 pixel
			if ((i > 1) && (fabs(largest_v_abs) - fabs(former_v_max))> (k/4))
			{
				printf("Attention: solution does not converge, LSA failed: largest_v_abs=%f, former v max:%f!\n",largest_v_abs, former_v_max);
				LSA_fail=1;

			}


			former_v_max=largest_v_abs;


	      //freee all matrices within this loop, BUT: if break (LSA OK: do not free final A, AT, Ninv,l_adjusted)
	      if (!found) {
	    	  A_final=mat_free(A_final);AT_final=mat_free(AT_final);Ninv=mat_free(Ninv);l_adjusted=mat_free(l_adjusted);
	      }

	        Dl=mat_free(Dl);A=mat_free(A);AT=mat_free(AT);
	        Dl_final=mat_free(Dl_final);//DL=mat_free(DL);
	        N=mat_free(N);AtP=mat_free(AtP);v=mat_free(v);

	       if (found || LSA_fail) break;

	   }
	   //Finally free the matrices
	   			 A_final=mat_free(A_final);AT_final=mat_free(AT_final);Ninv=mat_free(Ninv);l_adjusted=mat_free(l_adjusted);

	   //printf("number of iterartion:%d\n", i);
	   //printf("LSA fail=%d\n",LSA_fail);

		if (LSA_fail)
		{
			printf("LSA failed, so skip this line\n");
			delete(Q);
			delete(Avec);
			return 0;
		}

	//Copy the final L tp LPlueck
	   for(j=0;j<6;j++) LPlueck[j]=L->m[j][0];


	//End
	Line3D tmpl;
	PluckerToEuclidean(LPlueck, tmpl);
	//printf("final as Pluecker\n");
	//tmpl.Print();

	PluckerToEuclidean(LPlueck, *this);



	//In the calling function on ObjectSpaceLines: if this function here is not succesful (return 0): delete this line from the container!

    //the problem is that even the adjusted image lines will no more perfectly fit on the adjusted overall 3D line.
	//and: the line segments end points in the image (adjusted) are of course not accessible, therefore we
	//simply leave the observed image line segment untouched, the corresponding objectspaceline, however, needs to be adapted
			 //for this the this->compute_corresponding_objectsegments accepts a parameter to control this behaviour


	 /**/
	if (!this->compute_corresponding_objectsegments(1)) //it also calls compute_ObjectLineCombined()
	{
		printf("Error during compute_corresponding_objectsegments()\n");
		delete(Q);
		delete(Avec);
		return 0;
	}


	delete(Q);
	delete(Avec);

	return 1;
}

double ObjectSpaceLine::compute_lineresidualRMSE()
{
	double val=0;
	for (int i=0;i<ImageLine_dresidualV.size();i++)
	{
		//printf("debug: imagedres=%.2f\n",ImageLine_dresidualV[i]);
		val+=ImageLine_dresidualV[i]*ImageLine_dresidualV[i];
	}
	val/=ImageLine_dresidualV.size();

	//printf("\nRMSE=%.2f\n",sqrt(val));
	return sqrt(val);


}
