/*
                  Copyright 2010 University of Twente
 
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

 Initial creation:
 Author : Sander Oude Elberink
 Date   : 21-2-2013

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/


#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <matrix3.h>
#include "LaserPoints.h"
#include "LineTopsIterVector.h"

/*
--------------------------------------------------------------------------------
                           The main function
--------------------------------------------------------------------------------
*/

void prepare3DBGT(char *ascii_file, char *map_points_input,
	       char *map_topology_input, char *tagged_laser_points_output, char *map_points_output,
	       char *map_topology_output, double sample_distance,
           bool select_buildings, bool select_plantcover, bool debug,
           int reductionfactor)

{
  LaserPoints                  laser_points, sel_laser_points, out_laser_points,
                               tobedone;
  LaserPoints::iterator        laser_point;
  LaserPoint                   point, previous_point;
  ObjectPoints                 map_points, newmappoints,outbuildingpoints;
  ObjectPoint                  map_point;
  LineTopologies               map_lines, sel_map_lines, sorted_map_lines, empty_lines,
                               inner_rings, same_id_lines, outer_rings, master_rings,
                               outbuildings, notclosed_lines;
  LineTopology                 master_ring;
  LineTopologies::iterator     map_line, last_map_line;
  LineTopology::iterator             node;
  int                          count, success, index, pol_num,
                               iter, index1, index2, pn, value, segment, count2;
  bool                         found, done, remove, attr_removed, hole, dtm2map;
  Plane                        plane;
  vector<int>                  point_count;
  vector <int>                 segment_numbers, label_values, *values = new vector <int>(),
                               inner_ring_numbers, affected_lines;
  vector <int>::iterator       segment_number, label_value, stored_value, same_id;
 // bool debug=false;
  DataBounds3D                 bounds, mapbounds;
  Position3D                   pos;
  PointNumberList              pnl;
  FILE                         *ascii;
  char                         line[2048],*comma;
  double PI = 3.14159, area, orig_size, xmin,ymin,xmax,ymax; 
    double                       vecvalue[4];
  
  // Local constants to be made global
if (!select_buildings && !select_plantcover) printf("DTM information will be added to map height attribute (in cm, +10 meter)\n");
  
  // Read input data
  printf ("Reading in ascii laser points...");
  ascii = Open_Compressed_File(ascii_file, "r");
  if (!ascii) {
    fprintf(stderr, "Error opening input file %s\n", ascii_file);
    exit(0);
  }
//read in ascii laser points, transfer to laser format
fgets(line, 1024, ascii);//ignore the first line as this is probably a header

  do {
      index1++;
    if (fgets(line, 1024, ascii)) {
     while ((comma = strchr(line, ',')) != NULL) *comma = ' ';
     sscanf(line, "%lf %lf %lf %lf",
             vecvalue+ 1, vecvalue+ 2, vecvalue+ 3, vecvalue+ 4);

// Copy the data to the laser point

      point.X() = vecvalue[1] ;
      point.Y() = vecvalue[2] ; 
      point.Z() = vecvalue[3] ; 
      point.PolygonNumber() = (int)vecvalue[4] ;
      laser_points.push_back(point);         
      }
  //    }
      } while (!feof(ascii));
printf("Read %d points\n", laser_points.size());
//  if (!laser_points.Read(laser_input)) {
//    printf("Error reading laser points from file %s\n", laser_input);
//    exit(0);
//  }
  if (!map_points.Read(map_points_input)) {
    printf("Error reading map points from file %s\n", map_points_input);
    exit(0);
  }
  if (!map_lines.Read(map_topology_input)) {
    printf("Error reading map lines from file %s\n", map_topology_input);
    exit(0);
  }

printf ("Reduction factor to decrease amount of print information to screen: %d.\n", reductionfactor);
 for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (!map_line->IsClosed()) {
     
      notclosed_lines.push_back(*map_line);
      map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
  
  printf("Removed %d not closed polygons\n", count);
 if (debug) notclosed_lines.Write("notclosedlines.top", false);
 if (debug) map_points.Write("notclosedlines.objpts");
    
  
if (select_buildings || select_plantcover){
 sel_map_lines.erase(sel_map_lines.begin(), sel_map_lines.end());
 for (map_line=map_lines.begin(), count=0, count2=0;
       map_line!=map_lines.end(); map_line++) {
    if (map_line->Attribute(LineLabelTag) == 5501 && select_plantcover) { //only forest polygons
      if (!map_line->HasAttribute(PredictedHeight)){
      printf ("warning, mapline has no predicted dtm height, set to 0 m\r");
      map_line->Attribute(PredictedHeight) = 1000; //height in cm, 10m added (1000cm) to avoid problems below sea level
      }
      sel_map_lines.push_back(*map_line);
      count++;
    }
    if (map_line->TOP10MajorClass() == TOP10_Building && select_buildings) {
      if (!map_line->HasAttribute(PredictedHeight)){
      printf ("warning, mapline has no predicted dtm height, set to 0 m\r");
      map_line->Attribute(PredictedHeight) = 1000;
      }
      sel_map_lines.push_back(*map_line);
      count2++;
    }
  }
  map_lines = sel_map_lines;
   printf("\nSelected %d polygons from buildings and %d from plantcover\n", count2, count);
 sel_map_lines.erase(sel_map_lines.begin(), sel_map_lines.end());
 if (map_lines.size() ==0) {
 printf ("No polygons to process. Program has finished.\n");
 return;
 }
//    segment_numbers = laser_points.AttributeValues(SegmentNumberTag);

}

 // Densify the map points
if (!select_buildings && !select_plantcover){
  printf("Densifying map points ..."); fflush(stdout);
  count = map_points.size();
  map_lines.Densify(map_points, sample_distance); 
  printf("done, map points increased from %d to %d\n",
         count, map_points.size());
  // Merge points within 0.01 m
}  
printf("Start removing double points...(this can take a while)\n");
map_points.RemoveDoublePoints(map_lines, 0.01);
printf("Finished removing double points...\n");
  if (debug) printf("removing double nodes\n");
  
  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++, count++) {
    if (debug || count == (count/reductionfactor)*reductionfactor) printf("processing %d (%5.2f)\r",count, 100.0*count/map_lines.size());
         index1 = map_line->size();                         
       map_line->RemoveDoubleNodes(); 
       if (index1!=map_line->size()){
       if (debug) printf("\nremoved %d nodes from %d\n", index1-map_line->size(), map_line->Number());
       }
  }

 printf("\nStart removing tiny polygons....\n");

 for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (map_line->CalculateArea(map_points) < 0.1) {
      map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
  printf("Removed %d polygons smaller than 0.1 m2\n", count);


//map_points.Write(map_points_output);
//map_lines.Write(map_topology_output, false);

  bool set_building_number_tag = true;
  map_line = map_lines.begin();
  if (!map_line->HasAttribute(BuildingNumberTag)) set_building_number_tag=true;
  
 if (set_building_number_tag){
  printf("Start setting building number tag = map line ID number...\n");
  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++, count++) {
      if (debug || count == (count/reductionfactor)*reductionfactor) printf("%7d  %5.1f \r", count, 100.0 * count / map_lines.size());
       map_line->Attribute(BuildingNumberTag) = map_line->Attribute(IDNumberTag);
    }
}

printf("\nTransfering map info to laser points..\n");
   for (map_line=map_lines.begin(), index2=0;
         map_line!=map_lines.end(); map_line++, index2++) {
         if (debug || index2 == (index2/reductionfactor)*reductionfactor)printf("%7d  %5.1f ", index2, 100.0 * index2 / map_lines.size());
         map_line->Number() = index2;
         if (debug || index2 == (index2/reductionfactor)*reductionfactor)printf("%d  %d \r", map_line->Number(),map_line->Attribute(IDNumberTag));
         hole = false;
         if (map_line->Attribute(HoleTag) == 1){
         mapbounds = map_line->Bounds(map_points);
         hole = true;
         }

         for (laser_point = laser_points.begin(), count = 0; laser_point!=laser_points.end(); laser_point++){
             if (laser_point->Attribute(PolygonNumberTag) == map_line->Attribute(IDNumberTag) && !laser_point->Filtered()){
                laser_point->Attribute(LabelTag) = map_line->Attribute(LineLabelTag);
                laser_point->Attribute(PlaneNumberTag) = map_line->Attribute(IDNumberTag);
                if (hole){
                       if (laser_point->X()>mapbounds.Minimum().X() && laser_point->X()<mapbounds.Maximum().X()){
                       if (laser_point->Y()>mapbounds.Minimum().Y() && laser_point->Y()<mapbounds.Maximum().Y()){
                       if (laser_point->InsidePolygonJordan(map_points,
                                         map_line->LineTopologyReference())==1) {
                          laser_point->SetFiltered();
                          count++;
       //                   printf("\nremoving %d points within hole...\n", count);
                       }
                       }                           
                       }                
                }
             }
         }
          if (debug) printf("\n");
          laser_points.RemoveFilteredPoints();

      }  
   printf("\nStart writing files...\n");
   laser_points.RemoveFilteredPoints();

  
 dtm2map = true;
 if (select_buildings || select_plantcover) dtm2map = false; //not needed anymore when fusing nonground points to buildings and forest
 if (dtm2map){
printf ("start adding dtm attribute to building polygons (used a.o. for building floors in Lod0)\n");      

    double nbh_radius = 3, predictedheight;
    LineTopologies    map_lines_height;
    //LaserPoints       dtmpoints;
    PointNumberList::iterator          noden;
    PointNumberList neighbourhood;
    int             nearest_laser_point;
    TINEdges        edges;
    printf("Assigning DTM height in cm to map_line attribute predicted height. Added 10 meters (1000 cm) to avoid problems below sea level...\n");
  //  dtmpoints = laser_points;
   // laser_points.ErasePoints();
    laser_points.DeriveTIN();
      edges.Derive(laser_points.TINReference());

    for (map_line=map_lines.begin(), index2=0;
         map_line!=map_lines.end(); map_line++, index2++) {
         if (debug || index2 == (index2/reductionfactor)*reductionfactor) printf("%7d  %5.1f \r", index2, 100.0 * index2 / map_lines.size());
         sel_laser_points.ErasePoints();
         if (map_line->Attribute(HoleTag) == 1) continue; //attribute will be assigned to main polygon
         
        if (map_line->TOP10MajorClass() != TOP10_Building) continue;
         for(node = map_line->begin(); node!=map_line->end()-1; node++){
            map_point = *(map_points.PointIterator(*node));
            neighbourhood.erase(neighbourhood.begin(), neighbourhood.end());
            
            nearest_laser_point = laser_points.NearestPoint(map_point.Position3DRef(),
                                                    edges, true);
            nbh_radius = 3;
            do{
            neighbourhood = laser_points.Neighbourhood(PointNumber(nearest_laser_point),
                                       nbh_radius, edges,
                                       true, false);
            nbh_radius = nbh_radius*2;
            } while (neighbourhood.size()<2 && nbh_radius<50);
            
            for (noden=neighbourhood.begin(); noden!=neighbourhood.end(); noden++){
              sel_laser_points.push_back(laser_points[noden->Number()]);
            }
         }
         predictedheight = 0;
         
         if (sel_laser_points.size()>2){
         sel_laser_points.DeriveDataBounds(0);
         if (sel_laser_points.DataBounds().Minimum().Z()> -10 && sel_laser_points.DataBounds().Minimum().Z() < 400){ //lets make it reasonable in the NL
             predictedheight = sel_laser_points.DataBounds().Minimum().Z();
         }
         }
 //        map_line->Number() = index2;
         map_line->Attribute(PredictedHeight) = 1000 + int(100*predictedheight); //DTM height in cm, assigned to attribute predicted height
   //      map_lines_height.push_back(*map_line);
         if (debug) printf("\n");
    }
 //   map_lines = map_lines_height;
     edges.erase(edges.begin(), edges.end());
//     dtmpoints.ErasePoints();
    }
    printf("\ndone. Writing output of preparation stage...\n");
  
  
 
       map_points.Write(map_points_output);
       map_lines.Write(map_topology_output, false);

 if (!select_plantcover){
   printf("Start segmenting laser data...\n");
  
   double pointdistance = laser_points.MedianInterPointDistance(laser_points.size()-1);
   double radius = 1;
    if (pointdistance > 0.5) {
    radius = 2*pointdistance;
    printf("Median point distance is about %4.2f, setting seed radius to %4.2f, and growing radius to %4.2f...\n", pointdistance, radius, radius + 0.3);
    }
     SegmentationParameters *segpar;
  segpar=new SegmentationParameters();
  segpar->MaxDistanceInComponent()=1.5;
  segpar->SeedNeighbourhoodRadius()=radius;
  segpar->MaxDistanceSeedPlane()=0.15;
  segpar->GrowingRadius()=radius + 0.3;
  segpar->MaxDistanceSurface()=0.25;
 
  if (!debug) laser_points.SurfaceGrowing(*segpar, true, false);//without print statements
  if (debug) laser_points.SurfaceGrowing(*segpar);
}
  laser_points.Write(tagged_laser_points_output, false);
 
       printf("\nProgram prepare3DBGT finished.\n"); 
    

}
