
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


#include "DataAppearance.h"
#include "BNF_io.h"

DataAppearance & DataAppearance::operator = (const DataAppearance &appearance)
{
  data_type           = appearance.data_type;
  strcpy(data_type_name, appearance.data_type_name);
  show_points         = appearance.show_points;
  point_size          = appearance.point_size;
  point_colour_method = appearance.point_colour_method;
  point_colour        = appearance.point_colour;
  point_colour_cycle_length = appearance.point_colour_cycle_length;
  point_colour_cycle_phase  = appearance.point_colour_cycle_phase;
  selected_pulse_type = appearance.selected_pulse_type;
  show_lines          = appearance.show_lines;
  line_width          = appearance.line_width;
  line_colour_method  = appearance.line_colour_method;
  line_colour         = appearance.line_colour;
  first_point_size    = appearance.first_point_size;
  show_faces          = appearance.show_faces;
  face_colour_method  = appearance.face_colour_method;
  face_colour         = appearance.face_colour;
  highlighted         = appearance.highlighted;
  paint_first         = appearance.paint_first;
  return *this;
}

const char *DataAppearance::DataTypename()
{
  if (data_type_name[0] < 97) data_type_name[0] += 32;
  return data_type_name;
}

const char *DataAppearance::DataTypeName()
{
  if (data_type_name[0] >= 97) data_type_name[0] -= 32;
  return data_type_name;
}

void DataAppearance::SetDataType(DataType new_data_type)
{
  data_type = new_data_type;
  switch (data_type) {
    case MapData:           strcpy(data_type_name, "Map"); break;
    case MapPartitionData:  strcpy(data_type_name, "Map partition"); break;
    case ModelData:         strcpy(data_type_name, "Model"); break;
    case LastModelData:     strcpy(data_type_name, "Last model"); break;
    case LaserData:         strcpy(data_type_name, "Laser"); break;
    case SelectedMapData:   strcpy(data_type_name, "Selected map"); break;
    case SelectedMapPartitionData:
      strcpy(data_type_name, "Selected map partition"); break;
    case SelectedModelData: strcpy(data_type_name, "Selected model"); break;
    case SelectedModelPartData:
      strcpy(data_type_name, "Selected model part"); break;
    case SelectedLaserData: strcpy(data_type_name, "Selected laser"); break;
    case SelectionBoxData:  strcpy(data_type_name, "Selection box"); break;
    case SplitLineData:     strcpy(data_type_name, "Split line"); break;
    case ExtensionLineData: strcpy(data_type_name, "Extension line"); break;
    default:
      strcpy(data_type_name, "Unknown"); break;
  }
}

void DataAppearance::SetDataTypeName(const char *name)
{
  strcpy(data_type_name, name);
}

void DataAppearance::Initialise()
{
  // Selected data should be highlighted
  line_colour = QColor(255, 184, 61);
  if (data_type >= NumNormalDataTypes && data_type < NumDataTypes) {
    highlighted = true;
    point_colour = QColor(0, 127, 0);
    line_width   = 4;
    if (data_type == SelectedMapData || data_type == SelectedMapPartitionData)
      first_point_size = 9;
  }
  else {
    highlighted = false;
    point_colour = QColor(0, 255, 0);
    line_width   = 2;
    first_point_size = 0;
  }
  point_size = 1;
  face_colour = QColor(255, 0, 0);
  point_colour_method = line_colour_method = face_colour_method = FixedColour;
  paint_first = false;
  point_colour_cycle_length = 10.0; // meter
  point_colour_cycle_phase = 0.0;   // fraction from 0.0 to 1.0
  selected_pulse_type = AnyPulse;
  switch (data_type) {
    case MapPartitionData:
    case SelectedMapPartitionData:
      line_colour = QColor(255, 0, 0);
      line_width--;
      paint_first = true;
    case MapData:
         line_width++;
    case SelectedMapData:
      show_points = show_faces = false;
      show_lines = true;
      break;
    case ModelData:
      show_lines = true;
      show_faces = true;
      face_colour_method = ColourByLabel;
      break;   
    case LastModelData:
    case SelectedModelData:
    case SelectedModelPartData:
      show_points = show_lines = false;
      show_faces = true;
      if (data_type == LastModelData) face_colour = QColor(255, 255, 255);
      else face_colour_method = ColourByLabel;
      break;
    case SelectedLaserData:
      point_size = 2;
    case LaserData:
      show_points = true;
      show_lines = show_faces = false;
      point_colour = line_colour = face_colour = QColor(255, 255, 255);
      point_colour_method = ColourBySegment;
      break;
    case SelectionBoxData:
    case SplitLineData:
    case ExtensionLineData:
      show_points = show_faces = false;
      show_lines = true;
      line_colour = QColor(0, 255, 0);
      break;
    case SelectedPointData:
      show_points = true;
      show_lines = show_faces = false;
      point_colour = QColor(255, 255, 255);
      point_size = 9;
      paint_first = true;
      break;
    default:
      break;
  }
}

void DataAppearance::Write(FILE *fd, int indent) const
{
  BNF_Write_String(fd, "dataappearance", indent, NULL);
  BNF_Write_Integer(fd, "datatype", indent+2, (int) data_type, "%d");
  BNF_Write_String(fd, "datatypename", indent+2, data_type_name);
  
  // Point appearance
  BNF_Write_Integer(fd, "showpoints", indent+2, show_points, "%d");
  BNF_Write_Integer(fd, "pointsize", indent+2, point_size, "%d");
  BNF_Write_Three_Integers(fd, "pointcolour", indent+2,
                           point_colour.red(), point_colour.green(),
                           point_colour.blue(), "%d %d %d");
  if (data_type == LaserData || data_type == SelectedLaserData) {
    BNF_Write_Integer(fd, "pointcolourmethod", indent+2, point_colour_method, "%d");
    BNF_Write_Double(fd, "pointcolourcyclelength", indent+2, point_colour_cycle_length, "%.2f");
    BNF_Write_Double(fd, "pointcolourcyclephase", indent+2, point_colour_cycle_phase, "%.2f");
    BNF_Write_Integer(fd, "selectedpulsetype", indent+2, selected_pulse_type, "%d");
  }
    
  // Line appearance
  BNF_Write_Integer(fd, "showlines", indent+2, show_lines, "%d");
  BNF_Write_Integer(fd, "linewidth", indent+2, line_width, "%d");
  if (data_type != LaserData && data_type != SelectedLaserData)
    BNF_Write_Integer(fd, "linecolourmethod", indent+2, line_colour_method, "%d");
  BNF_Write_Three_Integers(fd, "linecolour", indent+2,
                           line_colour.red(), line_colour.green(),
                           line_colour.blue(), "%d %d %d");
  BNF_Write_Integer(fd, "firstpointsize", indent+2, first_point_size, "%d");

  // Face appearance
  BNF_Write_Integer(fd, "showfaces", indent+2, show_faces, "%d");
  if (data_type != LaserData && data_type != SelectedLaserData)
    BNF_Write_Integer(fd, "facecolourmethod", indent+2, face_colour_method, "%d");
  BNF_Write_Three_Integers(fd, "facecolour", indent+2,
                           face_colour.red(), face_colour.green(),
                           face_colour.blue(), "%d %d %d");
                           
  BNF_Write_String(fd, "enddataappearance", indent, NULL);
}

void DataAppearance::Read(FILE *fd)
{
  char *buffer, *line, *keyword;
  int  keyword_length, red, green, blue;

  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "datatype", MAX(keyword_length, 8)))
          printf("Ignored second specification of \"datatype\"\n");

        else if (!strncmp(keyword, "datatypename", MAX(keyword_length, 12)))
          SetDataTypeName(BNF_String(line));

        // Point attributes
        else if (!strncmp(keyword, "showpoints", MAX(keyword_length, 10)))
          show_points = ( BNF_Integer(line) != 0 );

        else if (!strncmp(keyword, "pointsize", MAX(keyword_length, 9)))
          point_size = BNF_Integer(line);

        else if (!strncmp(keyword, "pointcolourmethod", MAX(keyword_length, 17)))
          point_colour_method = (ColourMethod) BNF_Integer(line);

        else if (!strncmp(keyword, "pointcolour", MAX(keyword_length, 11))) {
          BNF_Three_Integers(line, &red, &green, &blue);
          point_colour = QColor(red, green, blue);
        }

        else if (!strncmp(keyword, "pointcolourcyclelength",
                          MAX(keyword_length, 22)))
          point_colour_cycle_length = BNF_Double(line);

        else if (!strncmp(keyword, "pointcolourcyclephase",
                          MAX(keyword_length, 21)))
          point_colour_cycle_phase = BNF_Double(line);

        else if (!strncmp(keyword, "selectedpulsetype", MAX(keyword_length, 17)))
          selected_pulse_type = (LaserPulseType) BNF_Integer(line);

        // Line attributes
        else if (!strncmp(keyword, "showlines", MAX(keyword_length, 9)))
          show_lines = ( BNF_Integer(line) != 0 );

        else if (!strncmp(keyword, "linewidth", MAX(keyword_length, 9)))
          line_width = BNF_Integer(line);

        else if (!strncmp(keyword, "linecolourmethod", MAX(keyword_length, 16)))
          line_colour_method = (ColourMethod) BNF_Integer(line);

        else if (!strncmp(keyword, "linecolour", MAX(keyword_length, 10))) {
          BNF_Three_Integers(line, &red, &green, &blue);
          line_colour = QColor(red, green, blue);
        }

        else if (!strncmp(keyword, "firstpointsize", MAX(keyword_length, 14)))
          first_point_size = BNF_Integer(line);


        // Face attributes
        else if (!strncmp(keyword, "showfaces", MAX(keyword_length, 9)))
          show_faces = ( BNF_Integer(line) != 0 );

        else if (!strncmp(keyword, "facecolourmethod", MAX(keyword_length, 16)))
          face_colour_method = (ColourMethod) BNF_Integer(line);

        else if (!strncmp(keyword, "facecolour", MAX(keyword_length, 10))) {
          BNF_Three_Integers(line, &red, &green, &blue);
          face_colour = QColor(red, green, blue);
        }

        else if (!strncmp(keyword, "enddataappearance", MAX(keyword_length,
                          17))) {
          free(buffer);
          return;
        }

        else
          fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
      }
    }
  }
}


