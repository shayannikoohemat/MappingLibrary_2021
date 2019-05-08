
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
 * ObjectSpaceLines.h
 *
 *  Created on: 28 Sep 2009
 *      Author: gerke, Tian and Nyaruhuma
 */

#ifndef OBJECTSPACELINES_H_
#define OBJECTSPACELINES_H_
#include "ObjectSpaceLine.h"
#include <math.h>


class ObjectSpaceLines : public std::vector<ObjectSpaceLine> {
protected:
	 int unique_instance_id;
	 DataBounds3D bounds3D; //databounds to exclude possibly wrong matches
	 Plane bounds_upperplane,bounds_lowerplane;//planes representing the upper and lower bb plane. Used for restricting search space in images
	 vector <unsigned int> ImageIdsV; //a vector containing the ids of participating images
	 vector < double *  >PMatV; //a vector of Projectmatrices (the sequence is the same as in the ImageIds, so a assignment image id-> P Mat is possible)
	 vector <InteriorOrientation> IntOriV; //the intorivecotr and extoriv information will be creatd upton add_ImageInfo
	 vector <ExteriorOrientation> ExtOriV;
	 vector <LineSegments2D> ImageLinesV; //a vector of image edges per image (the sequence is the same as in the ImageIds, so a assignment image id-> P Mat is possible)
	 int imgheight,imgwidth;

	 //protected methods
	 bool check_if_view_line_combination_in_ObjectSpaceLine(ObjectSpaceLine &osl, int &view, int &lineNum);
	 bool find_same_view_line_combination(ObjectSpaceLine &osl1,ObjectSpaceLine &osl2, int &view, int &lineNum);
	 void remove_multiple_imageline_assignments();
	 void remove_bad_lines(double max_rmse=4);


public:
	ObjectSpaceLines();
	virtual ~ObjectSpaceLines();

    DataBounds3D getBounds3D() const
    {
        return bounds3D;
    }
    void set_unique_instance_id(int i) {unique_instance_id=i;}
    int get_unique_instance_id() {return unique_instance_id;}

    void setBounds3D(DataBounds3D bounds3D);

    void setImageExtent(int h, int w)
    {
    	this->imgheight=h;
    	this->imgwidth=w;
    }
    ///Add some data
    void add_ImageInfo(unsigned int Id, double *PMat, LineSegments2D &ImageLines);
    void add_ImageInfo(unsigned int Id, double *PMat, ImageLines &iml);
    void add_ImageInfo(unsigned int Id, double *PMat);

    //to concatenate instances
    void add_other_instance(ObjectSpaceLines &other);


    ///copy everything except for the actual objectSpacelines, so only the image related information
    ///used in conjunction with filtering of useless instances; to erase members is much too expensive
    void copy_data_no_ObjectLine(const ObjectSpaceLines other);

    ///if we have two images involved, we can call a function which creates 3D lines through backward intersection of lines in
    ///all combinations construct first the 3Dline, then also the 3D line segments corresponding to the respective imagelines
    ///A line is only accepted if all 3d objectlines fall within the Boundingbox
    void Construct3Dlines_from_stereo();
    ///After we pushed back all the 3dlines as derived from Construct3Dlines_from_stereo we want to merge them
    ///in a sense that we search for combintations of the same image lines in order to have multiple views
    bool FindMultipleViews(int min_views=3);
    //void set_unique_line_segment_numbers();

    ///non-linear adjustment
    void call_LSA_per_member(){ObjectSpaceLines::iterator osl;
								for(osl=begin();osl!=end();osl++)
									if (!osl->LSA_line())
									{
										erase(osl);osl--;
										printf("instance removed from container since LSA failed\n");
									}

								printf("Before removal of bad lines: number of instances: %d\n",size());
								//write_objectsegments_points_top("/tmp/3dlinescombinedafterLSA_beforeremovalbad_3views",3);

								remove_bad_lines(); //default min rmse set in the definition of this function
								printf("After removal of lines because of max rmse: number of instances: %d\n",size());


								//write_objectsegments_points_top("/tmp/3dlinescombinedafterLSA_beforeremovalmultipleimageline_3views",3);

								remove_multiple_imageline_assignments();

								printf("After removal of lines from multiple image line assignment: number of instances: %d\n",size());

							}
    void dump();
    void write_objectsegments_points_top(char * prefix, int min_views=0);
    bool project_objectsegments_to_img(int imageId, LineSegments2D &ImageLineSeg, int min_views=0,char * InImg=NULL, char *OutImg=NULL);


};

#endif /* OBJECTSPACELINES_H_ */
