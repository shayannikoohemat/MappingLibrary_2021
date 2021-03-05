
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


/*
--------------------------------------------------------------------------------
 Program for pole detection
 
 Initial creation:
 Author : Martin Rutzinger, ITC 
 Date   : 01-01-2010

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "LaserPoints.h"
#include "KNNFinder.h"

#include <malloc.h>
#include <time.h>
#include "LaserBlock.h"
#include "TINEdges.h"
#include "BNF_io.h"
#include <iostream>

#include <math.h>
#include "LaserDataTypes.h"
#include <ANN.h>




/*
--------------------------------------------------------------------------------
                         Declaration of external functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(const char *, const char *, int *);
extern void timer_start(clock_t *time1);
extern void timer_end(clock_t time1, char *string);

/*
--------------------------------------------------------------------------------
                         The main poles function
--------------------------------------------------------------------------------
*/

LaserPoints selpts(float minH, float maxH, LaserPoints inpts){
    //local variables
    DataBoundsLaser newbounds;
    LaserPoints outpts;
    
    newbounds.SetMinimumZ(minH);
    newbounds.SetMaximumZ(maxH);
    
    inpts.Select(outpts,newbounds);
    
    return outpts;
}

int rectcount(vector<ObjectPoints> parts, int numpart, double diff_diag, double diff_pos, double max_diagpart)
{
    //local variables
    Position2D pos_part1, pos_part2;
    double len1, len2, len_part1, len_part2, dist, difflen;
    int count=0, labelid=0;
    ObjectPoints rect_objpts1, rect_objpts2;
    Position3D p1a,p1b,p1c,p1d, p2a,p2b,p2c,p2d;
    Position2D interpt1,interpt2;
    Line2D diag1a,diag1b, diag2a,diag2b;
      
    for (int i=1; i<parts.size(); i++){
        //get current and next rectangel in list
        rect_objpts1 = parts[i];
        rect_objpts2 = parts[i-1];

        // current rectangle
        //extract Posistion3D from ObjectPoints
        p1a = rect_objpts1[0];
        p1b = rect_objpts1[1];
        p1c = rect_objpts1[2];
        p1d = rect_objpts1[3];
        
        //calculate lenth of diagonal p1-p2
        len1 = p1a.Distance(p1c);
        
        //calculate intersection point
        diag1a = Line2D(p1a,p1c);
        diag1b = Line2D(p1b,p1d);
        Intersection2Lines(diag1a, diag1b, interpt1);
    
        // next rectangle
        //extract Posistion3D from ObjectPoints
        p2a = rect_objpts2[0];
        p2b = rect_objpts2[1];
        p2c = rect_objpts2[2];
        p2d = rect_objpts2[3]; 
        //calculate lenth of diagonal p1-p2
        len2 = p2a.Distance(p2c);
        //calculate intersection point
        diag2a = Line2D(p2a,p2c);
        diag2b = Line2D(p2b,p2d);
        Intersection2Lines(diag2a, diag2b, interpt2);
    
        //calculate differences between diagonals and center positions
        dist = abs(interpt1.Distance(interpt2));
        difflen = abs(len1 - len2);

        //check if diagonales and position is with in allowed range        
        //check criteria and count how often it is fulfilled
        if (    (max_diagpart > len1 && max_diagpart > len2) &&
                (dist < diff_pos && dist > 0.0) &&
                (difflen < diff_diag && difflen > 0.0) ){
                count++;               
        }
    }     
    return count;

}

PointNumberList GetListByTag (LaserPointTag tag, int value, LaserPoints inpts){
    PointNumberList list;   
    for(int k=0;k<inpts.size();k++){
        if((inpts[k]).Attribute(tag)==value)
            list.push_back(k);
    }
    return list;            
}



void poles(char *long_filter, char *infile,
                  char *appendix, char *output_directory,
                  bool output_meta_data, bool overwrite,
                  int per,
                  int nperpole,
                  float height,
                  int maxpts,
                  int numpart,
                  float maxdiagpart,
                  float diffpos,
                  float diffdiag,
                  int tag)

{
  char                 *directory, *filter, *filename, *newname;
  int                  icon, fileclass;
  LaserBlock           block;
  LaserBlock::iterator unitptr;
  LaserUnit::iterator  subunitptr;
  LaserSubUnit         newsubunit;
  clock_t              start;

//******************************************************************************                                                                                                                                                       
  LaserPoints            points, sel_laser_points, sel_part, percpts;
  LaserPoints::iterator  it;
  LaserPoint             minpt, maxpt;
  DataBoundsLaser        selbounds;
  
  vector <int>           indices2d, count2d, indices3d, count3d, seg_nums;
  vector <double>        dists2d, dists3d, height2d, height3d;
  vector <int>::iterator seg_id;
  double                 zrange, minZ, maxZ, heightZ, radius, minpercZ, maxpercZ;
  int                    numpartstotal, newlabel, countpart;
  Position2D             position;  
  vector <ObjectPoints>      rectangles;
  LineTopology           rect_top;
  LineTopologies         pole_top;
  ObjectPoints           pole_objpts,objpts3d, rect_objpts;
  LaserPointTag          lstag;
  PointNumberList        pnl;


//******************************************************************************
  
  printf("begin of poles\n");
  // Set up the file filter for the input file(s)
  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

  // Collect all points from all input files
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    // Set up a laser block
    if (!block.Create(filename, &fileclass)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }

    // Loop over all units and subunits
    for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {
      for (subunitptr=unitptr->begin(); subunitptr!=unitptr->end();
	       subunitptr++) {

        // Derive the names of the output files. Note that the variable
        // newsubunit is only used for checking the existance of files. New file 
        // names are later transfered to subunitptr.
        if (!subunitptr->Name()) subunitptr->LaserDataFiles::DeriveName();
        if (appendix) {
          newname = (char *) malloc(strlen(subunitptr->Name()) +
                                    strlen(appendix) + 1);
          sprintf(newname, "%s%s", subunitptr->Name(), appendix);
        }
        else {
          newname = (char *) malloc(strlen(subunitptr->Name()) + 1);
          sprintf(newname, "%s", subunitptr->Name());
        }
        newsubunit.SetName(newname);
        newsubunit.DataOrganisation() = subunitptr->DataOrganisation();
        newsubunit.DeriveMetaDataFileName(output_directory);
        newsubunit.DerivePointFileName(output_directory);
        free(newname);

        // Do not go ahead if the point file and meta file already exists and 
        // overwriting these files is not permitted.
        if (overwrite) || !FileExists(newsubunit.MetaDataFile()) ||
            !FileExists(newsubunit.PointFile())) {
        
          // Read the point data
          if (!subunitptr->Read()) {
            fprintf(stderr, "Error reading laser points from file %s\n",
                    subunitptr->PointFile());
            exit(0);
          }

//******************************************************************************

          // After reading all data, the file names can be overwritten with the
          // names of the output files.
          //points = *subunitptr;
          //points.SetName(newsubunit.Name());
          //points.DeriveMetaDataFileName(output_directory);
          //points.DerivePointFileName(output_directory);
          //subunitptr->SetName(newsubunit.Name());
          //subunitptr->DeriveMetaDataFileName(output_directory);
          //subunitptr->DerivePointFileName(output_directory);


         //choose LaserPointTag
         if (tag == 1){
            lstag = LabelTag;
         }
         else if (tag == 2){
            lstag = ComponentNumberTag;
         }
/*       else{
            cout<<"ERROR: tag not set"<<endl;
            exit(0);
         }
*/

          //check if any laser points are already labeled as poles and set them to unlabeled
          //subunitptr->RenameAttribute(lstag 1, lstag -1)
          for (it=subunitptr->begin(); it!=subunitptr->end(); it++){
              if (it->Attribute(lstag) == 1){
                 it->RemoveAttribute(lstag);
              }
          }
          
          //get segment numbers
          seg_nums = subunitptr->AttributeValues(SegmentNumberTag);

          //clear vectors
          rectangles.clear();
          
          //iterate over segment numbers
          for (seg_id = seg_nums.begin(); seg_id!=seg_nums.end(); seg_id++){
              
              //get points from segment
              sel_laser_points = subunitptr->SelectTagValue(SegmentNumberTag, *seg_id);
              
              //more than 4 pts needed
              if (sel_laser_points.size()>4){
              
                  //get minimum and maximum Z
                  selbounds = sel_laser_points.DeriveDataBounds(0);
                  minpt = selbounds.Minimum();
                  maxpt = selbounds.Maximum();
                  zrange = selbounds.ZRange();
                  minZ = minpt.GetZ();
                  maxZ = maxpt.GetZ();
    
                  // get percentiles and start (minpercZ) and end (maxpercZ) height of nperpole
                  float percZ = zrange/float(per);        //hight of the percentile
                  minpercZ = minZ + percZ * (float(nperpole)-1);
                  maxpercZ = minpercZ + percZ;
                  
                  // extract points for nperpole
                  percpts = selpts(minpercZ, maxpercZ, sel_laser_points);
                 
                  //more than 4 pts needed
                  if (percpts.size()>4){
                                                  
                      // extract enclosing rectangels; see usage PCMData.cc line 1564                 
                      //number of enclosing rectangels in one stack (percpts)
                      int numpartstotal = int((maxpercZ-minpercZ)/height);
                      //height of first part starting at the minimum height in percentile
                      heightZ=minpercZ+height;
                      
                      //create enclosing rectangel for each part
                      for (int i=0; i<=numpartstotal; i++){
                          //select laser points in part
                          sel_part = selpts(minpercZ, heightZ, percpts);
        
                          //set new min and max height for next iteration                  
                          minpercZ=heightZ;
                          heightZ=heightZ+height;
                          
                          
                          //check if max number of points per part is exeeded
                          //cout<<"DEBUG: sel_part.size()"<<sel_part.size()<<endl;
                          if ((sel_part.size() <= maxpts) && (sel_part.size()>4)){
                             //get enclosing reactangel
                             //neede for enclosing rectangle
                             sel_part.DeriveTIN();
                             sel_part.DeriveDataBounds(0);
                             sel_part.EnclosingRectangle(0.1, rect_objpts, rect_top);
                                             
                             //collect rectangel pts in vector for one segment
                             rectangles.push_back(rect_objpts);
                             
                          }
/*                          else{
                             cout<<"WARNING: part "<<i<<" skipped because maxpts too large or too small: "<<sel_part.size() <<endl;
                          }
*/
                          //clear vector
                          sel_part.clear();
                          rect_objpts.clear();
                          rect_top.clear();
                      }

                      //count the diff of the enclosing rectangel do classification of poles
                      if (rectangles.size()>1){

                         countpart = rectcount(rectangles, numpart, diffdiag, diffpos, maxdiagpart);

                         //classification                         
                         if  (countpart >= numpart){                           
                             pnl = GetListByTag(SegmentNumberTag,*seg_id, *subunitptr);
                             subunitptr->Label(pnl, 1, lstag);
//                             cout<<"INFO: seg "<<*seg_id<<"   "<<countpart<<" >= "<<numpart<<endl;
                         }                                   
                      }  
                      //clear vector
                      rectangles.clear();
                                      
                                            
                  }
/*                  else{
                     cout<<"WARNING percpts: percentile of segment "<<*seg_id<<" skipped because percpts too small: "<<percpts.size() <<endl;
                  }
*/
              }
                  

          }
                  
          
          //points.Write();    
          //points.ErasePoints();          

          




//******************************************************************************
          // Write the points and meta data
          subunitptr->Write();
          if (output_meta_data) subunitptr->WriteMetaData();

          // Erase the points, the TIN, and the TIN edges
          subunitptr->ErasePoints();
        }
        else {
          printf("Sub unit %s was already done.\n", newsubunit.Name());
          if (!subunitptr->ReadMetaData(newsubunit.MetaDataFile())) {
            fprintf(stderr, "Error reading meta data from file %s\n",
                    newsubunit.MetaDataFile());
            exit(0);
          }
        }
      }

      // Output of meta data at unit level
      if (output_meta_data &&
          (fileclass == LASER_STRIP || fileclass == LASER_BLOCK) &&
          unitptr->DataOrganisation() & StripWise) {
        if (!unitptr->Name()) unitptr->LaserDataFiles::DeriveName();
        if (appendix) {
          newname = (char *) malloc(strlen(unitptr->Name()) +
                                    strlen(appendix) + 1);
          sprintf(newname, "%s%s", unitptr->Name(), appendix);
        }
        else {
          newname = (char *) malloc(strlen(unitptr->Name()) + 1);
          sprintf(newname, "%s", unitptr->Name());
        }
        unitptr->SetName(newname);
        free(newname);
        unitptr->DeriveMetaDataFileName(output_directory);
        if (unitptr->begin()->IsCompleteStrip())
          unitptr->DerivePointFileName(output_directory);
        else unitptr->SetPointFile(NULL);
        unitptr->WriteMetaData();
      }
    }

    // Output of meta data at block level
    if (output_meta_data && fileclass == LASER_BLOCK) {
      if (!block.Name()) block.DeriveName();
      if (appendix) {
        newname = (char *) malloc(strlen(block.Name()) +
                                  strlen(appendix) + 1);
        sprintf(newname, "%s%s", block.Name(), appendix);
      }
      else {
        newname = (char *) malloc(strlen(block.Name()) + 1);
        sprintf(newname, "%s", block.Name());
      }
      block.SetName(newname);
      free(newname);
      block.DeriveMetaDataFileName(output_directory);
      block.WriteMetaData();
    }
  }
}
