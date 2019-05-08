
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
 * ObjectSpaceLines.cpp
 *
 *  Created on: 28 Sep 2009
 *      Author: gerke
 */

#include "ObjectSpaceLines.h"

ObjectSpaceLines::ObjectSpaceLines() {
	// TODO Auto-generated constructor stub
	ImageIdsV.resize(0);
	PMatV.resize(0);
	ImageLinesV.resize(0);

	bounds3D.Initialise();

	imgheight=0;
	imgwidth=0;

	resize(0);

}

ObjectSpaceLines::~ObjectSpaceLines() {
	// TODO Auto-generated destructor stub

}

bool ObjectSpaceLines::check_if_view_line_combination_in_ObjectSpaceLine(ObjectSpaceLine &osl, int &view, int &lineNum)
{
	for (unsigned int i=0;i<osl.ImageIdsV.size();i++)
	  if ((osl.ImageIdsV[i]==view) &&(osl.ImageLineV[i].Number()==lineNum)) return 1;

	return 0;
}

bool ObjectSpaceLines::find_same_view_line_combination(ObjectSpaceLine &osl1,ObjectSpaceLine &osl2, int &view, int &lineNum)
{
	//iterate over the views/lines of osl1 and check if the same view and line is used in osl2
	int view_tmp,lineNum_tmp;
	for (unsigned int i=0;i<osl1.ImageIdsV.size();i++)
	{
		view_tmp=osl1.ImageIdsV[i];
		lineNum_tmp=osl1.ImageLineV[i].Number();

		if (check_if_view_line_combination_in_ObjectSpaceLine(osl2,view_tmp,lineNum_tmp))
		{
			view=view_tmp;
			lineNum=lineNum_tmp;
			return 1;
		}
	}

	return 0;
}

void ObjectSpaceLines::setBounds3D(DataBounds3D bounds3D)
    {
        this->bounds3D = bounds3D;
        Vector3D normalBB_hz(0,0,1);
        Vector3D BBuppoint(0,0,this->bounds3D.Maximum().GetZ());
        Vector3D BBlowpoint(0,0,this->bounds3D.Minimum().GetZ());
        this->bounds_upperplane=Plane(BBuppoint,normalBB_hz);
        this->bounds_lowerplane=Plane(BBlowpoint,normalBB_hz);

        printf("bb:min z=%.2f, max z=%.2f\n",this->bounds3D .Minimum().GetZ(),this->bounds3D .Maximum().GetZ());


    }

void ObjectSpaceLines::add_ImageInfo(unsigned int Id, double *PMat)//, int imgheight)
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

    	InteriorOrientation inttmp;
    	ExteriorOrientation exttmp;

    	if (!compute_int_ext_from_P(&Ptmp, imgheight, imgwidth, inttmp, exttmp)) {printf("error during conversion from P to Interior and Ext.ori"); exit(0);}

    	this->IntOriV.push_back(inttmp);
    	this->ExtOriV.push_back(exttmp);

    	//this->imgheight=imgheight;
    	//this->set_unique_line_segment_numbers();

    }

void ObjectSpaceLines::add_ImageInfo(unsigned int Id, double *PMat, LineSegments2D &ImageLines)//, int imgheight)
    {
	//Set unique Number for the lines
	int uniqueN=Id*10000;
	LineSegments2D::iterator ls;
		for(ls=ImageLines.begin();ls!=ImageLines.end() ;ls++)
		ls->Number()=++uniqueN;

		//for(ls=ImageLines.begin();ls!=ImageLines.end() ;ls++)
		//		printf("Linenumber: %d\n",ls->Number());

		double *Ptmp; Ptmp=(double*) malloc (12*sizeof(double));

			for(int j=0; j<12;j++)
			   {
				 Ptmp[j]=PMat[j];
				}

		this->add_ImageInfo(Id,Ptmp);
	    this->ImageLinesV.push_back(ImageLines);

    }

void ObjectSpaceLines::add_ImageInfo(unsigned int Id, double *PMat, ImageLines &iml)//, int imgheight)
{
	LineSegments2D lineseg;

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
		           lineseg.push_back(LineSegment2D (Position2D(ax,ay),Position2D(bx,by)));
	}

	double *Ptmp; Ptmp=(double*) malloc (12*sizeof(double));

		for(int j=0; j<12;j++)
		   {
			 Ptmp[j]=PMat[j];
			}

	this->add_ImageInfo(Id,Ptmp,lineseg);//,imgheight);

}

/*
 * !This function adds the data from an other instance: all the ObjectLine parts from the vector and if there is a new imaga
 *
 */
void ObjectSpaceLines::add_other_instance(ObjectSpaceLines &other)
{

	//1. if the bb is not set yet, add this first
	if (!this->bounds3D.HasSomeData()) this->setBounds3D(other.getBounds3D());

	//1a)
	if (this->imgwidth==0) {
		this->imgwidth=other.imgwidth;
		this->imgheight=other.imgheight;
	}
	//2. All images which are not known yet here, will be added

	for (unsigned int i=0; i<other.ImageIdsV.size();i++)
		if (!check_value_in_vec(this->ImageIdsV,other.ImageIdsV[i]))
		{
			this->add_ImageInfo(other.ImageIdsV[i], other.PMatV[i],other.ImageLinesV[i]);
		}



	//3. add the lines (ObjectSpaceLineInstances)
	for (int l=0;l<other.size();l++)
	{
		//printf("dump of osl to add\n");
		//other[l].dump();
		ObjectSpaceLine tmposl=other[l];
		this->push_back(tmposl);
		//this->push_back(ObjectSpaceLine(other[l]));
	}


}


void ObjectSpaceLines::copy_data_no_ObjectLine(const ObjectSpaceLines other)
{
	DataBounds3D bounds3D; //databounds to exclude possibly wrong matches
		 Plane bounds_upperplane,bounds_lowerplane;//planes representing the upper and lower bb plane. Used for restricting search space in images
		 vector <unsigned int> ImageIdsV; //a vector containing the ids of participating images
		 vector < double *  >PMatV; //a vector of Projectmatrices (the sequence is the same as in the ImageIds, so a assignment image id-> P Mat is possible)
		 vector <InteriorOrientation> IntOriV; //the intorivecotr and extoriv information will be creatd upton add_ImageInfo
		 vector <ExteriorOrientation> ExtOriV;
		 vector <LineSegments2D> ImageLinesV; //a vector of image edges per image (the sequence is the same as in the ImageIds, so a assignment image id-> P Mat is possible)
		 int imgheight,imgwidth;

	this->bounds3D=other.bounds3D;
	this->bounds_upperplane=other.bounds_upperplane;
	this->bounds_lowerplane=other.bounds_lowerplane;
	this->ImageIdsV=other.ImageIdsV;
	this->PMatV=other.PMatV;
	this->IntOriV=other.IntOriV;
	this->ExtOriV=other.ExtOriV;
	this->ImageLinesV=other.ImageLinesV;
	this->imgheight=other.imgheight;
	this->imgwidth=other.imgwidth;
}

void ObjectSpaceLines::Construct3Dlines_from_stereo()
{
	if (ImageIdsV.size()!=2) {printf("Not 2 images known, but %d, error!\n",ImageIdsV.size()); return;}
	if (ImageLinesV.size()!=2) {printf("Line information not available\n"); return;}
	//if no boundingbox defined, raise a warning!
	if (!bounds3D.HasSomeData()) {printf("Error! No databound is set, however, this is required here to substantely reduce the search space for stereo matching!\n"); return;}
	//else: for every edge combination do forward intersection, if within the bounding box: push back

	//dump();
	//double PA[12], PB[12];
	double *PA;double* PB;
	PA=(double*) malloc (12*sizeof(double));
	PB=(double*) malloc (12*sizeof(double));

	for (int k=0;k<12;k++)
	{
		PA[k]=PMatV[0][k];
		PB[k]=PMatV[1][k];
	}

	//if an intersection falls successfully in the boundingbox: create a new instance of Objectine (3D line) (including a copy of the PMat and image edge and image id) and push back
	//Idea: now: take an edge in the left image. The endpoints of this edge do intersect with the upper and lower plane of the
	//bounding box. Those 3D points are then projected into the right image and only edges falling into the bounding box defined by them will be intersected

	for (unsigned int lleft=0;lleft<ImageLinesV[0].size();lleft++)
	{//printf("index left line: %d (from %d)\n",lleft,ImageLinesV[0].size()-1);
	   //Now, first get the image points of this edge to compute 3D points from intersection with bb planes
	   //points
	   Position2D BegP=ImageLinesV[0][lleft].BeginPoint();
	   Position2D EndP=ImageLinesV[0][lleft].EndPoint();
	   //rays
	   Line3D BegRay, EndRay;
	   ViewingRay(IntOriV[0], ExtOriV[0], BegP, BegRay);
	   ViewingRay(IntOriV[0], ExtOriV[0], EndP, EndRay);
	   //3Dpoints from intersections with bb planes
	   Position3D BegP_objectspace_upper,BegP_objectspace_lower,EndP_objectspace_upper,EndP_objectspace_lower;
	   if (!IntersectLine3DPlane (BegRay,this->bounds_upperplane,BegP_objectspace_upper)) {printf("error in plane->ray intersection!\n"); continue;}
	   if (!IntersectLine3DPlane (BegRay,this->bounds_lowerplane,BegP_objectspace_lower)) {printf("error in plane->ray intersection!\n"); continue;}
	   if (!IntersectLine3DPlane (EndRay,this->bounds_upperplane,EndP_objectspace_upper)) {printf("error in plane->ray intersection!\n"); continue;}
	   if (!IntersectLine3DPlane (EndRay,this->bounds_lowerplane,EndP_objectspace_lower)) {printf("error in plane->ray intersection!\n"); continue;}

	   /*
	   printf("Ray intersection with bb planes:\n");
	   printf("%.2f %2.f %.2f\n",BegP_objectspace_upper.GetX(),BegP_objectspace_upper.GetY(),BegP_objectspace_upper.GetZ());
	   printf("%.2f %2.f %.2f\n",BegP_objectspace_lower.GetX(),BegP_objectspace_lower.GetY(),BegP_objectspace_lower.GetZ());
	   printf("%.2f %2.f %.2f\n",EndP_objectspace_upper.GetX(),EndP_objectspace_upper.GetY(),EndP_objectspace_upper.GetZ());
	   printf("%.2f %2.f %.2f\n",EndP_objectspace_lower.GetX(),EndP_objectspace_lower.GetY(),EndP_objectspace_lower.GetZ());
	    */

	   //since we only have one other image (right) we can now already project those points into that one (and make a tmp bounding box for the next loop
	   ImagePoint BegP_upper_inright=project3D2img(BegP_objectspace_upper,&IntOriV[1], &ExtOriV[1]);
	   ImagePoint BegP_lower_inright=project3D2img(BegP_objectspace_lower,&IntOriV[1], &ExtOriV[1]);
	   ImagePoint EndP_upper_inright=project3D2img(EndP_objectspace_upper,&IntOriV[1], &ExtOriV[1]);
	   ImagePoint EndP_lower_inright=project3D2img(EndP_objectspace_lower,&IntOriV[1], &ExtOriV[1]);

	   /*
	   BegP_upper_inright.Print();
	   BegP_lower_inright.Print();
	   EndP_upper_inright.Print();
	   EndP_lower_inright.Print();
	    */
	   DataBounds2D bounds_rightimg;

	   bounds_rightimg.Initialise();
	   bounds_rightimg.Update(BegP_upper_inright);
	   bounds_rightimg.Update(BegP_lower_inright);
	   bounds_rightimg.Update(EndP_upper_inright);
	   bounds_rightimg.Update(EndP_lower_inright);

	   //printf("boundingbox check:.... right image: is it in the right image?...\n");
	   //if the bb is outside the image, continue
	   if ((bounds_rightimg.Maximum().GetX() < 0) && (bounds_rightimg.Maximum().GetY() < 0)) continue;
	   if ((bounds_rightimg.Minimum().GetX() > imgheight) && (bounds_rightimg.Minimum().GetY() > imgwidth)) continue;
	   //printf("...yes\n");

      /*************/
	   //if (ImageIdsV[1]==16)
	   if(1==0)
	   {
	   //debug: project this as line into the image
	   char inimage[500],outimage[500];
	   //strcpy(inimage,"/home/gerke/rfv/microdrone_ipb/seq4/constraints_sift_dist_io/img_undistorted_0001.jpg");
	   strcpy(inimage,"/home/gerke/rfv/Enschede_SouthEast/adjust/img_undistorted_0016.jpg");

	   sprintf(outimage,"/tmp/boundingpoints_%d_%d.png",ImageIdsV[1],lleft);
	   IplImage *myimage;
	   		if ((myimage = cvLoadImage( inimage, 1)) == 0) {
	   			    printf("Error reading input image %s\n", inimage);
	   			    return;
	   			  }

	   		//mark line in the image
	      cvLine(myimage,
				 cvPoint(BegP_upper_inright.GetY(),BegP_upper_inright.GetX()),
				 cvPoint(BegP_lower_inright.GetY(),BegP_lower_inright.GetX()),
				 cvScalar(0,0,255), 1);//(0,0,255) for red, 1 pixel line width

	      cvLine(myimage,
				 cvPoint(EndP_upper_inright.GetY(),EndP_upper_inright.GetX()),
				 cvPoint(EndP_lower_inright.GetY(),EndP_lower_inright.GetX()),
				 cvScalar(0,0,255), 1);//(0,0,255) for red, 1 pixel line width

		   //write lined_image to file
		   if( !cvSaveImage(outimage, myimage) ){
			   printf("failed to write line image file\n");

			   return;
			   }
		   //release memory
		   cvReleaseImage(&myimage);
	   }
		   /******************/

		for (unsigned int lright=0;lright<ImageLinesV[1].size();lright++)
		{

			//if the midpoint of this edge NOT in the BB as defined above: continue
			if (!bounds_rightimg.Inside(ImageLinesV[1][lright].MiddlePoint())){
				//printf("This edge in the image is skipped, because it is not in the bb\n");
				//printf("right line segment begin\n:");ImageLinesV[1][lright].BeginPoint().Print();
			    //printf("right line segment end\n:");ImageLinesV[1][lright].EndPoint().Print();
			    continue;
			}

/*
			printf("index left line: %d, index right line:%d\n",lleft, lright);
			printf("left line segment begin\n:");ImageLinesV[0][lleft].BeginPoint().Print();
			printf("left line segment end\n:");ImageLinesV[0][lleft].EndPoint().Print();

			printf("right line segment begin\n:");ImageLinesV[1][lright].BeginPoint().Print();
		    printf("right line segment end\n:");ImageLinesV[1][lright].EndPoint().Print();
*/
			ObjectSpaceLine tmpline;
			 //if (!Construct3DLine_pmat(&PA, ImageLinesV[0][lleft], &PB, ImageLinesV[1][lright], tmpline))
			 //{printf("Error during line construction!"); continue;}

			 if (!Construct3DLine_maplib(IntOriV[0], ExtOriV[0], ImageLinesV[0][lleft],IntOriV[1], ExtOriV[1], ImageLinesV[1][lright], tmpline)) {printf("Error during line construction!\n"); continue;}
			 //printf("End Construct3DLine_maplib for this line pair\n");

			 //now  add the corresponding image infos
			 //tmpline.set_unique_instance_id_objectspacelines(this->unique_instance_id);

			 tmpline.set_unique_instance_id_objectspacelines(Random(100)*1E5+ImageIdsV[0]+ImageIdsV[1]+ImageLinesV[0][lleft].Number()+ImageLinesV[1][lright].Number());

			 //printf("debug: this stereolins has id %d",tmpline.get_unique_instance_id_objectspacelines());
			 tmpline.setImageExtent(imgheight, imgwidth);
			 tmpline.add_ImageInfo(ImageIdsV[0],PMatV[0],ImageLinesV[0][lleft]);//,imgheight);
			 tmpline.add_ImageInfo(ImageIdsV[1],PMatV[1],ImageLinesV[1][lright]);//,imgheight);
			 //tmpline.dump();

			 //Now, the corresponding 3D segments are computed, so the segemts on the current line as corresponding to the actual
			 //image lines
			 if (!tmpline.compute_corresponding_objectsegments()) {printf("Error during objectsegment computation!\n"); continue;}
			 //printf("End compute_corresponding_objectsegments for this line pair\n");

			 //further test if the  endpoints from the combined objectspace line are with the 3D bounding box. if not: do not write this one!
			 if(tmpline.check_if_ObjectLineCombined_in_DataBounds(bounds3D)) this->push_back(ObjectSpaceLine(tmpline));
			 //else printf("skipped this line, because not falling into bounds3d\n");
			 //printf("dump tmpline\n");
			 //tmpline.dump();

		}

	}//lleft
	printf("\n");
	free(PA); free(PB);
}

bool ObjectSpaceLines::FindMultipleViews(int min_views)
{
	//This function is to look for multiple views, this means:
	//initially only stereo combinations were used, and the resulting ObjectSpaceLine-Instances were pushed into one instance of ObjectSpaceLines
	//now we iterate over the ObjectSpaceLine-Vector:
	//-we fix one, then iterate over the remainig ones.
	//-if one of the view, and line combination (ie. the identical edge (id)) is involved (the first strong constraint):
		//check, if the reprojetion of the fixed 3D line into the image, which was not used fits with the measured edge in that image
		//if yes: add the additional view of the current one to the fixed one

	if (!size()) {printf("no lines in this container!\n");return 0;}

	ObjectSpaceLines::iterator osl,osl_outer,osl_inner;
	int total_count=size();
	int divisor= total_count/100; divisor=MAX(1,divisor); //avoid to write too many lines on the console
	printf("\nTotal number of lines: %d\nStarting to find multiple view matches. Wait\n",total_count);
	int current=1;
	double dist_thresh_objspace=.50;

	for(osl_outer=begin();osl_outer!=end()-1;osl_outer++)
	{
		//int last_matched_unique_instance_id_objectspacelines=-99; //this one is to remember from which original  2 view intersection (reflected by unique_instance_id_objectspacelines)
		//we had a valid match. We only need one per image combination
		if ((current % divisor)==0 || current==1) printf("looking for partners for line %d out of %d\r",current,total_count);
		current++;

			//printf("Looking at a new reference line (fixed object line: view left:%d, view right:%d)\n",osl_outer->ImageIdsV[0],osl_outer->ImageIdsV[1]);
			//ObjectSpaceLine fixed=*osl_outer;//use the iterator for the fixed one
			//see, if one of the view-edge-combination as involved in the fixed one, is involved elsewhere
			for (osl_inner=osl_outer+1;osl_inner!=end();osl_inner++)
			{

				//if the distance of the combined object lines (in object space) is larger than a threshold-> continue
				if (osl_outer->ObjectLineCombined.Distance(osl_inner->ObjectLineCombined.MiddlePoint())>dist_thresh_objspace) continue;
				int same_view,same_lineNum;
				if(!find_same_view_line_combination(*osl_outer,*osl_inner, same_view, same_lineNum)) continue;

				//printf("from fixed one (outer): same_view=%d, same_lineNum=%d\n",same_view,same_lineNum);
				//project the fixed 3DLine into the view which was not already used there.
				//if they match: put the image info into the fixed one and erase it here
				for (int i=0;i<osl_inner->ImageIdsV.size();i++)
					if (osl_inner->ImageIdsV[i]!=same_view)
					{

						LineSegment2D repro_line=project3D2img(osl_outer->ObjectLineCombined, &osl_inner->IntOriV[i],&osl_inner->ExtOriV[i]);
						if (!check_LineSegment2D_identity(osl_inner->ImageLineV[i],repro_line)) continue;
						//also: reproject this osl_inner->objectlinecom to the lines stored in the outer one;
						bool does_not_match=0;
						for (int j=0; j<osl_outer->ImageIdsV.size();j++)
						{
							LineSegment2D repro_line=project3D2img(osl_inner->ObjectLineCombined, &osl_outer->IntOriV[j],&osl_outer->ExtOriV[j]);
							if (!check_LineSegment2D_identity(osl_outer->ImageLineV[j],repro_line))
								{does_not_match=1;break;}
						}
						if (does_not_match) continue;

						//printf("The fixed object line, reprojected to image %d in this object line does match the respective extracted image line (imageline number:%d)!\n",osl_inner->ImageIdsV[i],osl_inner->ImageLineV[i].Number());
						//now: push this image-info to the fixed one
						int idtmp=osl_inner->ImageIdsV[i];//because the type is uint, and the refernce is handed over
						if (!check_if_view_line_combination_in_ObjectSpaceLine(*osl_outer, idtmp, osl_inner->ImageLineV[i].Number()))
						{

							osl_outer->add_ImageInfo(osl_inner->ImageIdsV[i],osl_inner->PMatV[i],osl_inner->ImageLineV[i]);
							osl_outer->add_ObjectLineSegment(osl_inner->ObjectLineV[i]);

							/*
							printf("Debug: added the following image line and object line segment (image id:%d):\n",idtmp);
							osl_inner->ImageLineV[i].Print();
							osl_inner->ObjectLineV[i].Print();
							*/
						}
					}

			}//inner FOR loop

	}//outer FOR loop


	//printf("After merging the views, before erasing useless instances (i.e. lines which have been assigned to other line instances during merging):%d\n", size());
	//make a new instance
	ObjectSpaceLines newinstance;newinstance.copy_data_no_ObjectLine(*this);
	 //{erase(osl);osl--;}
	//erase is too expensive

	for(osl=begin();osl!=end();osl++)
	{

	 if (osl->No_of_unique_ImageIds() >= min_views)
			{//printf("taken\n");
			 osl->compute_ObjectLineCombined();
			 newinstance.push_back(*osl);
			}

	}
	swap(newinstance);
	printf("\nFinal number of 3d lines with minumum %d views:%d\n",min_views,size());

return 1;
}
void ObjectSpaceLines::dump()
{
	printf("Number of images known: %d\n", ImageIdsV.size());
	printf("imgheight=%d , width=%d \n",imgheight,imgwidth);

	for (unsigned int i=0; i< ImageIdsV.size();i++)
	{
		printf("Number of lines in image with Id %d: %d\n", ImageIdsV[i], ImageLinesV[i].size());
		printf("Line details:\n");
		for (int j=0;j<ImageLinesV[i].size();j++)
		{	    printf("Line Number:%d\n",ImageLinesV[i][j].Number());
				printf("Begin point: row=%.1f   col=%.1f\n",ImageLinesV[i][j].BeginPoint().GetX(),ImageLinesV[i][j].BeginPoint().GetY());
				printf("End   point: row=%.1f   col=%.1f\n\n",ImageLinesV[i][j].EndPoint().GetX(),ImageLinesV[i][j].EndPoint().GetY());
		}
		printf("PMatrix for this image:\n");
		for (int k=0;k<12;k++) printf("%.2f ",PMatV[i][k]);
		printf("Interior Ori for this image:\n");
		IntOriV[i].Print();

		printf("Exterior Ori for this image:\n");
		ExtOriV[i].Print();

		printf("\n");
	}
	printf("The databound is set: %d\n",bounds3D.HasSomeData());

	printf("number of ObjectSpaceLine instances here: %d\n",size());
	printf("dump of them:\n");

	ObjectSpaceLines::iterator osl;
		for(osl=begin();osl!=end() ;osl++) osl->dump();
}

void ObjectSpaceLines::write_objectsegments_points_top(char * prefix, int min_views)
{
	//iterate over all containermembers and collect the objectsegments
	ObjectPoint pt1, pt2;
	ObjectPoints objpts;
	LineTopology line;
	LineTopologies linetops;
	int objnumber = 0;
	int linenumber = 0;

	//for debugging: if 2 rays are allowed, we anyhow only want the ones which were detected to be collinear (find multipleview on the initial stereo pair), this means the number
	//of image edges is larger than 2
	ObjectSpaceLines::iterator osl;
	for(osl=begin();osl!=end() ;osl++)
		if (osl->No_of_unique_ImageIds()>=min_views)
		{
			if (min_views==2 && osl->ImageIdsV.size()<3){
					printf("2 unique views are requested, but we need at least 3 image edges (DEbug version, normally do not use 2 rays at all)\n");
					continue;
				}


			//printf("Debug:Direction vector of this 3D Line %.2f %.2f %.2f:\n",osl->Direction().X(),osl->Direction().Y(),osl->Direction().Z());
		    //instead of iterating over the 3Dsegments, only use the combined one

			Position3D Begin=osl->ObjectLineCombined.BeginPoint();
			Position3D End=osl->ObjectLineCombined.EndPoint();

			//printf("in write: ObjectLineCombined: begin X=%.2f Y=%.2f  Z=%.2f  end X=%.2f Y=%.2f Z=%.2f\n",Begin.X(),Begin.Y(),Begin.Z(),End.X(),End.Y(),End.Z());

			pt1 = ObjectPoint (Begin.X(), Begin.Y(), Begin.Z(), ++objnumber, 0, 0, 0, 0, 0, 0);
			pt2 = ObjectPoint (End.X(), End.Y(), End.Z(), ++objnumber, 0, 0, 0, 0, 0, 0);
			objpts.push_back(pt1);
			objpts.push_back(pt2);

			int l=1;//label not really used yet. Later?
			line = LineTopology (++linenumber, l, objnumber - 1, objnumber);
			linetops.push_back(line);

		//iterate over the lineseg
		/*
		printf("DEBUG: writing the single objectsegments instead the combined one\n");
			for (int o=0; o<osl->ObjectLineV.size(); o++)
			{
				Position3D Begin=osl->ObjectLineV[o].BeginPoint();
				Position3D End=osl->ObjectLineV[o].EndPoint();

				printf("segment index=%d, begin X=%.2f Y=%.2f  Z=%.2f  end X=%.2f Y=%.2f Z=%.2f\n",o,Begin.X(),Begin.Y(),Begin.Z(),End.X(),End.Y(),End.Z());

				pt1 = ObjectPoint (Begin.X(), Begin.Y(), Begin.Z(), ++objnumber, 0, 0, 0, 0, 0, 0);
				pt2 = ObjectPoint (End.X(), End.Y(), End.Z(), ++objnumber, 0, 0, 0, 0, 0, 0);
				objpts.push_back(pt1);
				objpts.push_back(pt2);

				int l=1;//label not really used yet. Later?
				line = LineTopology (++linenumber, l, objnumber - 1, objnumber);
				linetops.push_back(line);
			}
		*/

		}

	char objectsegm_file_objpts[500];
	char objectsegm_file_top[500];

	sprintf(objectsegm_file_objpts,"%s.objpts",prefix);
	sprintf(objectsegm_file_top,"%s.top",prefix);

	if (objpts.size())
		{
		objpts.Write(objectsegm_file_objpts);
		linetops.Write(objectsegm_file_top);
		}

}

bool ObjectSpaceLines::project_objectsegments_to_img(int imageId, LineSegments2D &ImageLineSeg, int min_views,char * InImg, char *OutImg)
{
	if (this->imgheight==0 || this->imgwidth==0) {printf("imgheigh/width not set, but necessary for the determination whether a point falls in an image\n"); return 0;}

	//search the Int and Ext Ori for this image
	InteriorOrientation Into;
	ExteriorOrientation Exto;
	bool found=0;
	//empty the instance given to this function
	ImageLineSeg.resize(0);

	LineSegments2D OriginalImageLineSeg;OriginalImageLineSeg.resize(0);

	for (int i=0;i<ImageIdsV.size();i++)
		if (ImageIdsV[i]==imageId)
		{
		found=1;
		Into = IntOriV[i];
		Exto = ExtOriV[i];
		break;
		}

	if (!found) {printf("An image with id %d does not exist!\n",imageId); return 0;}
	ObjectSpaceLines::iterator osl;
	for(osl=begin();osl!=end() ;osl++)
		if (osl->No_of_unique_ImageIds()>=min_views)
			{

				//instead of iterating over all ObjectLines, only use the combined one. It is the same in the image
								Position3D Begin=osl->ObjectLineCombined.BeginPoint();
								Position3D End=osl->ObjectLineCombined.EndPoint();
								ImagePoint image_point_begin=project3D2img(Begin, &Into, &Exto);
								ImagePoint image_point_end=project3D2img(End, &Into, &Exto);


								//printf("index of Objectlinesegment: %d\n",o);
								//printf("rueck projez mit maplib begin: row=%.2f col=%.2f\n",image_point_begin.GetX(),image_point_begin.GetY());
								//printf("rueck projez mit maplib end: row=%.2f col=%.2f\n",image_point_end.GetX(),image_point_end.GetY());

								bool begin_is_in=1;
								bool end_is_in=1;

								if (image_point_begin.GetX() < 0 || image_point_begin.GetX() > this->imgheight) begin_is_in=0;
								if (image_point_begin.GetY() < 0 || image_point_begin.GetY() > this->imgwidth) begin_is_in=0;

								if (image_point_end.GetX() < 0 || image_point_end.GetX() > this->imgheight) end_is_in=0;
								if (image_point_end.GetY() < 0 || image_point_end.GetY() > this->imgwidth) end_is_in=0;

								if (begin_is_in || end_is_in)
								{
									LineSegment2D tmpseg(Position2D(image_point_begin.GetX(),image_point_begin.GetY()),Position2D(image_point_end.GetX(),image_point_end.GetY()));
									ImageLineSeg.push_back(tmpseg);


									//also write all original image files to another instance
									for (int i=0;i<osl->ImageIdsV.size();i++)
											if (osl->ImageIdsV[i]==imageId)
											OriginalImageLineSeg.push_back(osl->ImageLineV[i]);


								}

								//else printf("not falling in this image!\n");


			}

	//if both files are given and at least one line is in the image, write it to file...
	if (InImg != NULL && OutImg !=NULL && ImageLineSeg.size())
	{
		IplImage *myimage;
		if ((myimage = cvLoadImage( InImg, 1)) == 0) {
			    printf("Error reading input image %s\n", InImg);
			    return 0;
			  }


		 LineSegments2D::iterator ls;
		 	for(ls=ImageLineSeg.begin();ls!=ImageLineSeg.end() ;ls++)
		 			{
					/*if one point is outside the image, opencv does the correct placing of the visible part
					 */
					   int begin_point_x=(int)round(ls->BeginPoint().GetY());
					   int begin_point_y=(int)round(ls->BeginPoint().GetX());
					   int end_point_x=(int)round(ls->EndPoint().GetY());
					   int end_point_y=(int)round(ls->EndPoint().GetX());

					   //mark line in the image
					   cvLine(   myimage,
								 cvPoint(begin_point_x,begin_point_y),
								 cvPoint(end_point_x,end_point_y),
								 cvScalar(0,0,255), 1);//(0,0,255) for red, 1 pixel line width

						}

		 	//Also write the original image lines to the same image (green colour)
		 	for(ls=OriginalImageLineSeg.begin();ls!=OriginalImageLineSeg.end() ;ls++)
		 			 			{

		 						   int begin_point_x=(int)round(ls->BeginPoint().GetY());
		 						   int begin_point_y=(int)round(ls->BeginPoint().GetX());
		 						   int end_point_x=(int)round(ls->EndPoint().GetY());
		 						   int end_point_y=(int)round(ls->EndPoint().GetX());

		 						   //mark line in the image
		 						   cvLine(   myimage,
		 									 cvPoint(begin_point_x,begin_point_y),
		 									 cvPoint(end_point_x,end_point_y),
		 									 cvScalar(0,255,0), 1);//(0,255,0) for green, 1 pixel line width

		 							}

		   //write lined_image to file
		   if( !cvSaveImage(OutImg, myimage) ){
			   printf("failed to write line image file\n");

			   return 0;
			   }
		   //release memory
		   cvReleaseImage(&myimage);
		   OriginalImageLineSeg.resize(0);

		   printf("in red: reprojected image line, in green: original image line!\n");

	}
	else printf("Not writing lines to image, because either no image given to function or no imagelines falling in that particular image\n");

	return 1;


}

void ObjectSpaceLines::remove_multiple_imageline_assignments()
{
	//TODO!!! Remove all measures to avoid double assignment of lines from FindMultipleViews (see//debug0504)
	/**The problem is:
	during FindMultipleViews, lines are merged sequentially, based on a threshold (reprojection error).
	in principal, one image line can only contribute to one 3D line, not more. So, there is a task to find the best 3D line in case multiple
	3d lines were created from the same 2d image line.
	Because of the sequential process in FindMultipleViews, it is not optimal to fix the first line, because other might be better fitting and later will not be considered
	thus this method has been implemented:
	-based on the reprojection error it decides to take keep the best 3d line in case multiple lines have been created from one image line

	Strategy:
	do a loop:
	 *if a image line contributes to more than one ObjectSpaceLine: Remove all ObjectSpaceLines except the one where the image line has the lesat reprojection error
	do this loop until no image lines which occur multiple times are existing anymore


	*/

	 map<int, vector <int> > imageline_oslassignemt; //key:imagelinenumber, map: list of ids of osl where this imageline participates
	 map<int, vector <double> > imageline_residualassignment; //key:imagelinenumber, map: list of residuals in the actual osl

	 map<int, vector <int> >::iterator it_line_osl;

	 /*Example
	 imageline_oslassignemt[200003].size()=3
     imageline_oslassignemt[200003][2]=2032
	 imageline_residualassignemt[200003][2]=4.5
	 means: imageline with id 200003 (they are unique independent from image id) is participating in 3 ObjectSpaceLines, where one of them has id 2032, and the residual in the particular osl is 4.5 pixels
	 */

	 int iteration=1;
	 while(1)
	 {

	 imageline_oslassignemt.clear();
	 imageline_residualassignment.clear();
	 //printf("in iteration:%d\n",iteration++);
	 int max_number_of_imageline_assignment=1;//what is the maximum number of imagelines->osl assignment?
	 int imageline_with_max_osls;//imageline number which was assigned to max. number of osl
	 double least_residual=1E6;
	 int osl_where_least_residual;


	 ObjectSpaceLines::iterator osl;
	 	for(osl=begin();osl!=end() ;osl++)
	 	{
	 		for (int i=0;i< osl->ImageLineV.size();i++)
	 		{
	 			imageline_oslassignemt[osl->ImageLineV[i].Number()].push_back(osl->unique_instance_id_objectspacelines);
	 			//imageline_residualassignment[osl->ImageLineV[i].Number()].push_back(osl->ImageLine_dresidualV[i]); //only the reprojection error of that image line within this osl
				imageline_residualassignment[osl->ImageLineV[i].Number()].push_back(osl->compute_lineresidualRMSE()); //look at the overall accuracy of that osl
	 		}
	 	}


	 	//printf("statistics of image line assignment\n");
	 	for(it_line_osl = imageline_oslassignemt.begin(); it_line_osl != imageline_oslassignemt.end(); ++it_line_osl)
	 	    {
				//update statistics
				if (it_line_osl->second.size()>max_number_of_imageline_assignment)
				{
					max_number_of_imageline_assignment=it_line_osl->second.size();
					imageline_with_max_osls=it_line_osl->first;
					//find the least residual
					least_residual=1E6;
					for (int j=0; j<it_line_osl->second.size(); j++)
						if (imageline_residualassignment[it_line_osl->first][j]<least_residual)
						{
							least_residual=imageline_residualassignment[it_line_osl->first][j];
							osl_where_least_residual=it_line_osl->second[j];
						}


				}

				/*
				printf("Image line id: %d occurs in %d objectspacelines (distance residual): ",it_line_osl->first,it_line_osl->second.size());
				for (int j=0; j<it_line_osl->second.size(); j++) printf("%d (%.2f)   ",it_line_osl->second[j],imageline_residualassignment[it_line_osl->first][j]);
				printf("\n");
				*/
	 	    }

	 	//printf("max_number_of_imageline_assignment=%d, occured for imageline=%d\n",max_number_of_imageline_assignment,imageline_with_max_osls);
	 	//printf("the least residual is %.2f, occuring at osl :%d\n",least_residual,osl_where_least_residual);

	 	//now all osl where this imageline appears, except for the one with the least residual will be removed

	 	if (max_number_of_imageline_assignment==1) break;
		for(osl=begin();osl!=end() ;osl++)
		 	if ((osl->unique_instance_id_objectspacelines!=osl_where_least_residual) && check_value_in_vec(imageline_oslassignemt[imageline_with_max_osls],osl->unique_instance_id_objectspacelines))
		 	{
		 		//printf("Remove osl with id %d\n",osl->unique_instance_id_objectspacelines);
		 		erase(osl);osl--;
		 	}

	 }
}

void ObjectSpaceLines::remove_bad_lines(double max_rmse)
{
	//from the ImageLine_dresidualV[i] compute the RMSE per object line, and remove all which are worse
	for(ObjectSpaceLines::iterator osl=begin();osl!=end() ;osl++)
	if (osl->compute_lineresidualRMSE() > max_rmse)
	{
		//printf("Remove osl with id %d because RMSE too big\n",osl->unique_instance_id_objectspacelines);
		erase(osl);osl--;
	}

}

