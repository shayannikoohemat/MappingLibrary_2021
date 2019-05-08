
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
//#include "defs_TUB.h"
#include <cstdlib>
#include <iostream>
#include <math.h>
#include "MathTools_TUB.h"
#include "DataBounds3D.h"
#include "Tools.h"
#include "EdgeTools.h"
#include "ObjectSpaceLines.h"


using namespace std;

/*!!!
-after reading image information (Pmatrix, image file) and extracting the image lines:
-we instantiate a container for 3D lies (ObjectSpaceLines collected_osls).
For every stereo combination we define tempoary containers where we collect the image related information (orientation, image edges), and call the forward intersection method for the particular stereo pair
They are computed from every edge combination, but required to fall within the Boundingbox
The 3D lines from each stereo intersection are collected in the "collected_osls" instance

After the stereo combinations we find multiple view-3D lines, so lines where (in this case) at least 3 views are participating

The final 3D lines (with minium number of participating images) are adjusted.
Then, two postprocesses are applied:
1) Lines with a residual error exceeding a given threshold are removed
2) in principal every image edge can only contribute to one 3D line. In the last step all 3D lines where image lines are participating which are also contributing to other 3D lines are removed (only the 3d line where the particular 2d line has a least reprojection error is kept)

Last not least the lines are written to objpts/top files, and can also be reprojected to image files, see example below

REMARKS:
A) parameter tuning/setting is possible at 3 stages:
  -image line extraction (here currently hard coded in the function get_lines in Tools.cpp
  -tolerance for checking the reprojection error, this is used to identify a matching line through reprojecting an existing 3d line to another image.
  The parameters distance, angle, coverage are hard coded in the header file ObjectSpaceLine.h , function; check_LineSegment2D_identity
  -RMSE threshold for final accuracy check (RMSE from distance of image lines (extracted vs. reprojected), hard coded in ObjectSpaceLines.h, function: remove_bad_lines
  
B) The function extractlines_burns in EdgeTools.cpp expects the C++ function llinexBurns(char* imagename,char* linesfilename, ImageLines &iml, int bucket_width, int min_num_pixels, double min_magnitude, double vote, int gradient_mask) which is defined in burns_replacement in linex. 

C) Bounding box: if now bounding box is defined, the stereo intersection method will not work. I decided to do this in order to reduce the search space for the matching. In normal cases one should have an approximate idea about the matching area. If however you extract image lines only in a certain AIO, you can change that behavior in the particular method
*/
                        
int main()
{

const char * image_dir="data";  
const char * image_info_file="data/imageinfo.txt";


int image_height=-99, image_width=-99;
FILE* image_info;

::map<int, string> imagenames;
::map<int, vector<double> > pmatrices;
vector<int> viewNums;viewNums.clear();

printf("Demonstration of 3D line matchig using the classes ObjectSpaceLine(s)\n");

	//define a Boundingbox to restrict the search area
	     
	DataBounds3D db3d;
	printf("Define BB for a part of EnschedeSouthEast: Folded roof area\n");
	db3d.SetMinimumX(8300);
	db3d.SetMinimumY(340);
	db3d.SetMinimumZ(40);
	
	db3d.SetMaximumX(8450);
	db3d.SetMaximumY(500);
	db3d.SetMaximumZ(92);


	printf("\ndatabounds:\nMinimum: X=%.1f \t Y=%.1f \t Z=%1.f:\n",db3d.Minimum().GetX(),db3d.Minimum().GetY(),db3d.Minimum().GetZ());
	printf("Maximum: X=%.1f \t Y=%.1f \t Z=%1.f:\n",db3d.Maximum().GetX(),db3d.Maximum().GetY(),db3d.Maximum().GetZ());


printf("\nreading image information from %s\n",image_info_file);
if ((image_info=fopen(image_info_file,"r")) == NULL)
		{
			printf("CAN NOT OPEN %s!\n", image_info_file);
			return(0);
		}

 while(!feof(image_info))
		 {
			int viewNum;
			char imagename[500];
			double p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12;
	
			fscanf(image_info,"%d %s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",&viewNum, imagename,&p1,&p2,&p3,&p4,&p5,&p6,&p7,&p8,&p9,&p10,&p11,&p12 );
			
			viewNums.push_back(viewNum);
			imagenames[viewNum]=imagename;
			pmatrices[viewNum].push_back(p1);pmatrices[viewNum].push_back(p2);pmatrices[viewNum].push_back(p3);pmatrices[viewNum].push_back(p4);
			pmatrices[viewNum].push_back(p5);pmatrices[viewNum].push_back(p6);pmatrices[viewNum].push_back(p7);pmatrices[viewNum].push_back(p8);
			pmatrices[viewNum].push_back(p9);pmatrices[viewNum].push_back(p10);pmatrices[viewNum].push_back(p11);pmatrices[viewNum].push_back(p12);
	
			//add image width and image height
			IplImage* img = cvLoadImage(imagename,1);
			if (image_height<0) image_height=img->height;
			  else if (image_height != img->height) printf("warning: images do not have same size\n");
			if (image_width<0) image_width=img->width;
			  else if (image_width != img->width) printf("warning: images do not have same size\n");
			
			cvReleaseImage(&img);
			if (feof(image_info)) break;
		 }
fclose(image_info);

printf("\ninterpreted the textfile as follows:\n");
for (::map<int, string>::iterator it_names=imagenames.begin();it_names!=imagenames.end();it_names++)
{
    printf("viewNum: %d, imagename:%s, pmatrix:\n",it_names->first,it_names->second.c_str());
    for (int i=0;i<12;i++) printf("%.6f  ",pmatrices[it_names->first][i]);
    printf("\n\n");
}

printf("image dimension is %d x %d\n",image_width,image_height);

//starting 3d edge computation
ObjectSpaceLines collected_osls;
int unique_instance_osl=0; //per image pair a unique id is given for the ObjectSpaceLines

for (int i=0;i<viewNums.size()-1;i++)
  for (int j=i+1;j<viewNums.size()-1;j++)
  {  
    int l=viewNums[i];
    int r=viewNums[j];
    printf("\ndoing stereo line matching in image pair %d - %d\n",l,r);

    ImageLines  ILI_left;
    ImageLines  ILI_right;

    //converting  the PMatrices to double*
    double Pleft[12], Pright[12];

    for(int k=0; k<12;k++)
	{
	      Pleft[k]=pmatrices[l][k];
	      Pright[k]=pmatrices[r][k];
	}


    printf("\ngetting lines from image %d and %d\n",l,r);
    if (!get_lines(l,image_dir,imagenames[l].c_str(),ILI_left)) {printf ("error retrieving lines from image %d\n",l);continue;}
   
    if (!get_lines(r,image_dir,imagenames[r].c_str(),ILI_right)) {printf ("error retrieving lines from image %d\n",r);continue;}


    //push the data to the objectspacelines and call the Stereo intersection method
    ObjectSpaceLines osl;
    osl.set_unique_instance_id(unique_instance_osl++);
    osl.setImageExtent(image_height, image_width);
    osl.setBounds3D(db3d);
    osl.add_ImageInfo(l, Pleft, ILI_left);
    osl.add_ImageInfo(r, Pright, ILI_right);
    
    osl.Construct3Dlines_from_stereo(); //this is including objectsegment computation and test if those are in the bb
   
    printf("\nextracted %d 3D lines in stereo pair %d %d\n\n",osl.size(),l,r);
  
    //add those 3D lines to the container collected_osls
    collected_osls.add_other_instance(osl);
   
  }
  //End of stereo intersection

//now we do the Merge of the stereo views
  
    int minviews_multiple=3;//3;
    if (!collected_osls.FindMultipleViews(minviews_multiple)) printf("Error in FindMultipleViews\n");
    else printf("\n==>FindMultipleViews successful!\n\n");
    
    //call the LSA method per member
    collected_osls.call_LSA_per_member();
    
    //now we are finished, but we export some objpts/top data and we reproject the 3D lines to the original images
    char objectsegm_file_prefix[500];
    for (int minviews=3;minviews<9;minviews++)
    {
	printf("\n");
	sprintf(objectsegm_file_prefix,"lines3dcombined_min%dviews",minviews);
	collected_osls.write_objectsegments_points_top(objectsegm_file_prefix,minviews);
    }
    
    LineSegments2D ImageLineSeg; //dummy, not used here: reprojected 2d line seg
    char inimage[500],outimage[500];
	
    for (::map<int, string>::iterator it_names=imagenames.begin();it_names!=imagenames.end();it_names++)
    {
      printf("\n");
      strcpy(inimage,it_names->second.c_str());
      sprintf(outimage,"Projected_to_%d_min3views.png",it_names->first);
      collected_osls.project_objectsegments_to_img(it_names->first,ImageLineSeg,3,inimage,outimage);
    }
      
printf("\nbye\n");


  return EXIT_SUCCESS;
}
