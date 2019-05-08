
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


#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"
#include "LaserBlock.h"
#include "BNF_io.h"
#include <windows.h>

using namespace std;

void PrintUsage()
{
  printf("block2strips sorts points of a tiled block into strips.\n");
  printf("If strip names already occur in the directory -sdir\n");
  printf("this strip is not extracted again from the tilewise block.\n");
  printf("Usage: block2strips -i <tilewise block meta data file>\n");
  printf("                    -o <new stripwise block>\n");
  printf("                    -skip <number of tiles to skip>\n");
  printf("                    -odir <output directory>\n");
  printf("                    [-polpts <pologon object points of selected areas>]\n");
  printf("                    [-poltop <pologon topology of selected area>]\n");
}

int main(int argc, char *argv[])
{
  InlineArguments          *args = new InlineArguments(argc, argv);
  LaserBlock               in_block, out_block;
  LaserBlock::iterator     unit, strip, current_strip;
  LaserUnit::iterator      tile;
  LaserSubUnit             strip_part;
  LaserPoints::iterator    point, start_point, end_point;
  int                      sensor, rest, scandir, time, day, strip_number,
                           part_number, max_num_unstored=50, num_processed,
                           tile_count, corner;
  char                     *root_dir, sub_dir[13], strip_name[10], full_dir[200], *ch,
                           part_name[15], *strip_meta_file;
  bool                     done, inside;
  ObjectPoints             map_points;
  LineTopologies           map_tops;
  LineTopologies::iterator map_top;
  ObjectPoint2D            tile_corner[4];

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i")) {
    printf("Error: -i is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-o")) {
    printf("Error: -o is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-odir")) {
    printf("Error: -odir is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  // Read the meta data
  if (!in_block.ReadMetaData(args->String("-i"))) {
    printf("Error reading input meta data from %s\n", args->String("-i"));
    exit(0);
  }
  
  root_dir = args->String("-odir");
  strip_part.DataOrganisation() = StripWise;
  // Initialise output block, possible with partially collected data
  if (BNF_FileExists(args->String("-o"))) {
    // Avoid reading the possibly massive meta data of strips and parts
    out_block.ReadMetaData(args->String("-o"), false, false);
    printf("Restarting with data in %d strips\n", out_block.size());
  }
  else
    out_block.SetMetaDataFile(args->String("-o"));
  
  // Read selected area
  if (args->Contains("-polpts")) {
    if (!map_points.Read(args->String("-polpts"))) {
      printf("Error reading map points from file %s\n", args->String("-polpts"));
      exit(0);
    }
  }
  if (args->Contains("-poltop")) {
    if (!map_tops.Read(args->String("-poltop"))) {
      printf("Error reading map topology from file %s\n", args->String("-poltop"));
      exit(0);
    }
  }
  
  // Loop over all subunits
  num_processed = 0;
  tile_count = args->Integer("-skip", 0);
  for (unit=in_block.begin(); unit!=in_block.end(); unit++) {
    for (tile=unit->begin()+args->Integer("-skip", 0); tile!=unit->end();
         tile++, tile_count++) {
      // Check if a tile corner is within a selected area
      if (map_tops.size()) {
        tile_corner[0].X() = tile->TileBounds().Minimum().X();
        tile_corner[0].Y() = tile->TileBounds().Minimum().Y();
        tile_corner[1].X() = tile->TileBounds().Minimum().X();
        tile_corner[1].Y() = tile->TileBounds().Maximum().Y();
        tile_corner[2].X() = tile->TileBounds().Maximum().X();
        tile_corner[2].Y() = tile->TileBounds().Maximum().Y();
        tile_corner[3].X() = tile->TileBounds().Maximum().X();
        tile_corner[3].Y() = tile->TileBounds().Minimum().Y();
        inside = false;
        for (corner=0; corner<4 && !inside; corner++)
          for (map_top=map_tops.begin(); map_top!=map_tops.end() && !inside; map_top++)
            if (tile_corner[corner].InsidePolygon(map_points, *map_top)) inside = true;
        if (!inside) continue;
      }
        
      // Read tile points
      if (!tile->Read(tile->PointFile(), false)) {
        printf("Error reading points from file %s\n", tile->PointFile());
        exit(0);
      }
  
      printf("Tile %s with strips: ", tile->Name());
      fflush(stdout);    
      // Process tile points
      start_point = tile->begin();
      while (start_point != tile->end()) {
        strip_number = start_point->Attribute(ScanNumberTag);
        
        // Find all points with this strip number
        for (point=start_point, done=false; point!=tile->end() && !done; point++)
          if (point->Attribute(ScanNumberTag) != strip_number) {
            end_point = point-1;
            done = true;
          }
        if (!done) end_point = tile->end()-1;
        strip_part.insert(strip_part.end(), start_point, end_point);
        start_point = end_point+1; // For the next round
            
        // Determine strip name and directory
        // Number ABBCCCCD, name A_BB_CCC_{AFN}
        // A - 1 or 2 (sensor number)
        // B - day
        // C - time
        // D - scan direction 1=F, 2=N, 3=A
        sensor  = strip_number / 10000000;
        rest    = strip_number - sensor*10000000;
        day     = rest / 100000;
        rest    = rest - day * 100000;
        time    = rest / 10;
        scandir = rest - time * 10;
        
        sprintf(sub_dir, "%1d_%2d_%4d", sensor, day, time);
        // Replace blanks by 0
        ch = sub_dir; 
        while (*ch) { if (*ch == ' ') *ch = '0'; ch++; }
        strcpy(strip_name, sub_dir);
        switch (scandir) {
          case 1: strcat(strip_name, "_F"); break;
          case 2: strcat(strip_name, "_N"); break;
          case 3: strcat(strip_name, "_A"); break;
        }
        printf(" %s", strip_name); fflush(stdout);
        
        // Compose full directory name
        strcpy(full_dir, root_dir);
        if (full_dir[strlen(full_dir)-1] != '/')
          strcat(full_dir, "/");
        strcat(full_dir, sub_dir);

        // Create directory if needed
        if (!DirectoryExists(full_dir)) CreateDirectory(full_dir, NULL);
        
        // Compose meta data file name of strip
        strip_meta_file = ComposeFileName(full_dir, strip_name, ".strip");
          
        // Check if the strip already exists
        // Use meta data file name for matching as this is the only attribute
        // that is stored without reading the full meta data
        for (strip=out_block.begin(), current_strip=out_block.end();
             strip!=out_block.end() && current_strip==out_block.end(); strip++)
          if (!strcmp(strip_meta_file, strip->MetaDataFile())) current_strip = strip;
        
        // Create a new strip if needed
        if (current_strip == out_block.end()) {
          out_block.push_back(LaserUnit());
          current_strip = out_block.end() - 1;
          current_strip->DataOrganisation() = StripWise;
          current_strip->SetName(strip_name);
          current_strip->DeriveMetaDataFileName(full_dir);
        }
        // Otherwise read the strip meta data if the part information is missing
        else {
          if (current_strip->empty()) {
            if (!current_strip->ReadMetaData(strip_meta_file, false)) {
              printf("Error reading meta data from file %s\n", strip_meta_file);
              exit(0);
            }
          }
        }
        free(strip_meta_file);
          
        // Set meta data for the new strip part  
        part_number = current_strip->size() + 1;
        sprintf(part_name, "%s_%4d", strip_name, part_number);
        ch = part_name; while (*ch) { if (*ch == ' ') *ch = '0'; ch++; } // Replace blanks by 0
        strip_part.SetName(part_name);
        strip_part.DeriveMetaDataFileName(full_dir);
        strip_part.DerivePointFileName(full_dir);
          
        // Write points
        strip_part.Write(strip_part.PointFile(), false, false);
        
        // Erase points and remove them from the tile
        strip_part.ErasePoints();

        // Write part meta data
        strip_part.WriteMetaData();
        
        // Add strip part meta data to strip meta data
        current_strip->push_back(strip_part);
        
        // Erase points when we're done with this tile
        if (start_point == tile->end()) {
          tile->ErasePoints();
          start_point = tile->end(); // Re-set start pointer
        }
      }        
      printf("\n");
      
      // Check if we should save meta data again
      num_processed++;
      if (max_num_unstored*(num_processed/max_num_unstored) == num_processed) {
        printf("\nStoring meta data after processing %d tiles\n\n", tile_count);

        printf("Saving meta data for strips:");
        // Save strip meta data for all used strips since the last save
        for (strip=out_block.begin(); strip!=out_block.end(); strip++) {
          if (strip->size()) {
            printf("%s ", strip->Name());
            // Save strip meta data
            strip->WriteMetaData(false);
            // Erase part information from the strip (it should be re-read
            // if needed again)
            strip->erase(strip->begin(), strip->end());
          }
        }
        printf("\n\n");
        
        // Save block meta data
        out_block.WriteMetaData(false, false);
      }
    }
  }

  printf("Saving meta data for the last processed strips:");
  // Save strip meta data for all used strip since the last save
  for (strip=out_block.begin(); strip!=out_block.end(); strip++) {
    if (strip->size()) {
      printf("%s ", strip->Name());
      // Save strip meta data
      strip->WriteMetaData(false);
    }
  }
  printf("\n\n");
  
  // Write meta data of block and strips
  out_block.WriteMetaData(false, false);
  return EXIT_SUCCESS;
}
