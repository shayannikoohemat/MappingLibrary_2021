
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


#include "DataAppearance.h"
#include "BNF_io.h"

DataAppearance & DataAppearance::operator = (const DataAppearance &appearance)
{
  int i;
  
  if (this == &appearance) return *this;  // Check for self assignment
  data_type           = appearance.data_type;
  strcpy(data_type_name, appearance.data_type_name);
  show_points         = appearance.show_points;
  show_loose_points   = appearance.show_loose_points;
  point_size          = appearance.point_size;
  point_colour_method = appearance.point_colour_method;
  point_colour        = appearance.point_colour;
  point_colour_cycle_length = appearance.point_colour_cycle_length;
  point_colour_cycle_phase  = appearance.point_colour_cycle_phase;
  point_colour_gamma        = appearance.point_colour_gamma;
  maximum_point_spacing_in_pyramid = appearance.maximum_point_spacing_in_pyramid;
  maximum_number_of_points_memory  = appearance.maximum_number_of_points_memory;
  maximum_number_of_points_display = appearance.maximum_number_of_points_display;
  num_colours         = appearance.num_colours;
  
  use_display_filter  = appearance.use_display_filter;
  display_filter      = appearance.display_filter;
  selected_pulse_type = appearance.selected_pulse_type;

  show_lines          = appearance.show_lines;
  line_width          = appearance.line_width;
  line_colour_method  = appearance.line_colour_method;
  line_colour         = appearance.line_colour;
  only_show_lines_in_segment = appearance.only_show_lines_in_segment;
  first_point_size    = appearance.first_point_size;
  
  show_faces          = appearance.show_faces;
  face_colour_method  = appearance.face_colour_method;
  face_colour         = appearance.face_colour;
  triangulate_faces   = appearance.triangulate_faces;
  only_show_faces_in_segment = appearance.only_show_faces_in_segment;
  highlighted         = appearance.highlighted;
  paint_first         = appearance.paint_first;
  height_offset       = appearance.height_offset;
  
  threshold_attribute = appearance.threshold_attribute;
  // Check on initialisation
  if (appearance.num_thresholds < 0 ||
      appearance.num_thresholds >= PCM_MAX_NUM_THRESHOLDS) {
    printf("Warning: Invalid number of thresholds %d in DataAppearance copy assignment.\n",
	       appearance.num_thresholds);
	printf("         Number of thresholds is set to 0.\n");
	num_thresholds = 0;
  }
  else {
    num_thresholds      = appearance.num_thresholds;
    for (i=0; i<PCM_MAX_NUM_THRESHOLDS; i++) 
	  thresholds[i] = appearance.thresholds[i];
    for (i=0; i<PCM_MAX_NUM_THRESHOLDS+1; i++)
	  threshold_colours[i] = appearance.threshold_colours[i];
    colours_in_between_thresholds = appearance.colours_in_between_thresholds;
  }
  
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
    case SelectedModelFaceData:
      strcpy(data_type_name, "Selected model face"); break;
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
  show_loose_points = (data_type == LastModelData);
  point_size = 1;
  face_colour = QColor(255, 0, 0);
  point_colour_method = line_colour_method = face_colour_method = FixedColour;
  paint_first = false;
  point_colour_cycle_length = 10.0; // meter
  point_colour_cycle_phase = 0.0;   // fraction from 0.0 to 1.0
  point_colour_gamma = 1.0;
  num_colours = 216;
  maximum_point_spacing_in_pyramid = 3.0;
  maximum_number_of_points_memory = 1000000;
  maximum_number_of_points_display = 100000;
  triangulate_faces = true;
  use_display_filter = false;
  display_filter = DataBoundsLaser();
  selected_pulse_type = AnyPulse;
  only_show_lines_in_segment = only_show_faces_in_segment = false;
  height_offset = 0.0;
  num_thresholds = 0;
  switch (data_type) {
    case MapPartitionData:
    case SelectedMapPartitionData:
      line_colour = QColor(255, 0, 0);
      line_width--;
      paint_first = true;
    case MapData:
    case SelectedMapData:
      show_points = show_faces = false;
      show_lines = true;
      break;
    case ModelData:
    case LastModelData:
    case SelectedModelData:
    case SelectedModelPartData:
      show_points = show_lines = false;
      show_faces = true;
      if (data_type == LastModelData) face_colour = QColor(255, 255, 255);
      else face_colour_method = ColourByLabel;
      break;
    case SelectedModelFaceData:
      show_points = show_lines = show_faces = true;
      line_colour = QColor(0, 255, 0);
      break;
    case SelectedLaserData:
      point_size = 2;
    case LaserData:
      show_points = true;
      show_lines = show_faces = false;
      point_colour = line_colour = face_colour = QColor(255, 255, 255);
      point_colour_method = ColourByHeight;
      threshold_attribute = ZCoordinateTag;
      num_thresholds = 2;
      thresholds[0] = 30.0;
      thresholds[1] = 40.0;
      threshold_colours[0] = QColor(255, 0, 0);
      threshold_colours[1] = QColor(0, 255, 0);
      threshold_colours[2] = QColor(0, 0, 255);
      no_attribute_colour = QColor(255, 255, 255);
      colours_in_between_thresholds = true;
      for (int it=2; it<PCM_MAX_NUM_THRESHOLDS; it++) {
        thresholds[it] = 0.0;
        threshold_colours[it+1] = QColor(255, 255, 255);
      }
      break;
    case SelectionBoxData:
    case SplitLineData:
    case ExtensionLineData:
    case TileBoundaryData:
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
  char string[25];
  
  BNF_Write_String(fd, "dataappearance", indent, NULL);
  BNF_Write_Integer(fd, "datatype", indent+2, (int) data_type, "%d");
  BNF_Write_String(fd, "datatypename", indent+2, data_type_name);
  
  // Point appearance
  BNF_Write_Integer(fd, "showpoints", indent+2, show_points, "%d");
  if (data_type == LastModelData) 
    BNF_Write_Integer(fd, "showloosepoints", indent+2, show_loose_points, "%d");
  BNF_Write_Integer(fd, "pointsize", indent+2, point_size, "%d");
  BNF_Write_Three_Integers(fd, "pointcolour", indent+2,
                           point_colour.red(), point_colour.green(),
                           point_colour.blue(), "%d %d %d");
  if (data_type == LaserData || data_type == SelectedLaserData) {
    BNF_Write_Integer(fd, "pointcolourmethod", indent+2, point_colour_method, "%d");
    BNF_Write_Double(fd, "pointcolourcyclelength", indent+2, point_colour_cycle_length, "%.2f");
    BNF_Write_Double(fd, "pointcolourcyclephase", indent+2, point_colour_cycle_phase, "%.2f");
    BNF_Write_Double(fd, "pointcolourgamma", indent+2, point_colour_gamma, "%.2f");
    BNF_Write_Integer(fd, "selectedpulsetype", indent+2, selected_pulse_type, "%d");
    BNF_Write_Integer(fd, "numcolours", indent+2, num_colours, "%d");
  }
  if (data_type == LaserData) {
    BNF_Write_Double(fd, "maxpointspacingpyramid", indent+2,
                     maximum_point_spacing_in_pyramid, "%.2f");
    BNF_Write_Integer(fd,"maxnumberofpointsmemory", indent+2,
                      maximum_number_of_points_memory, "%d");
    BNF_Write_Integer(fd,"maxnumberofpointsdisplay", indent+2,
                      maximum_number_of_points_display, "%d");
    
    BNF_Write_Integer(fd,"thresholdattribute", indent+2,
                      (int) threshold_attribute, "%d");
    BNF_Write_Integer(fd,"numberofthresholds", indent+2,
                      num_thresholds, "%d");
    BNF_Write_Integer(fd, "colourinbetweenthresholds", indent+2,
                      colours_in_between_thresholds, "%d");
    for (int it=0; it<num_thresholds; it++) {
      sprintf(string, "thresholdcolour%d", it);
      BNF_Write_Three_Integers(fd, string, indent+2,
                               threshold_colours[it].red(), 
                               threshold_colours[it].green(),
                               threshold_colours[it].blue(), "%d %d %d");
      sprintf(string, "threshold%d", it);
      BNF_Write_Double(fd, string, indent+2, thresholds[it], "%.3f");
    }
    if (colours_in_between_thresholds) {
      sprintf(string, "thresholdcolour%d", num_thresholds);
      BNF_Write_Three_Integers(fd, string, indent+2,
                               threshold_colours[num_thresholds].red(), 
                               threshold_colours[num_thresholds].green(),
                               threshold_colours[num_thresholds].blue(),
                               "%d %d %d");
    }
    BNF_Write_Three_Integers(fd, "noattributecolour", indent+2,
                             no_attribute_colour.red(), 
                             no_attribute_colour.green(),
                             no_attribute_colour.blue(), "%d %d %d");
  }
  if (data_type == MapData)
    BNF_Write_Double(fd, "heightoffset", indent+2, height_offset, "%.2f");
                     
  // Line appearance
  BNF_Write_Integer(fd, "showlines", indent+2, show_lines, "%d");
  BNF_Write_Integer(fd, "linewidth", indent+2, line_width, "%d");
  if (data_type != LaserData && data_type != SelectedLaserData)
    BNF_Write_Integer(fd, "linecolourmethod", indent+2, line_colour_method, "%d");
  BNF_Write_Three_Integers(fd, "linecolour", indent+2,
                           line_colour.red(), line_colour.green(),
                           line_colour.blue(), "%d %d %d");
  BNF_Write_Integer(fd, "onlyshowlinesinsegment", indent+2, only_show_lines_in_segment, "%d");
  BNF_Write_Integer(fd, "firstpointsize", indent+2, first_point_size, "%d");

  // Face appearance
  BNF_Write_Integer(fd, "showfaces", indent+2, show_faces, "%d");
  BNF_Write_Integer(fd, "facecolourmethod", indent+2, face_colour_method, "%d");
  BNF_Write_Three_Integers(fd, "facecolour", indent+2,
                           face_colour.red(), face_colour.green(),
                           face_colour.blue(), "%d %d %d");
  BNF_Write_Integer(fd, "triangulatefaces", indent+2, triangulate_faces, "%d");
  BNF_Write_Integer(fd, "onlyshowfacesinsegment", indent+2, only_show_faces_in_segment, "%d");
                           
  BNF_Write_String(fd, "enddataappearance", indent, NULL);
}

void DataAppearance::Read(FILE *fd)
{
  char *buffer, *line, *keyword;
  int  keyword_length, red, green, blue, it;

  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "datatype", max(keyword_length, 8)))
          printf("Ignored second specification of \"datatype\"\n");

        else if (!strncmp(keyword, "datatypename", max(keyword_length, 12)))
          SetDataTypeName(BNF_String(line));

        // Point attributes
        else if (!strncmp(keyword, "showpoints", max(keyword_length, 10)))
          show_points = ( BNF_Integer(line) != 0 );

        else if (!strncmp(keyword, "showloosepoints", max(keyword_length, 15)))
          show_loose_points = ( BNF_Integer(line) != 0 );

        else if (!strncmp(keyword, "pointsize", max(keyword_length, 9)))
          point_size = BNF_Integer(line);

        else if (!strncmp(keyword, "pointcolourmethod", max(keyword_length, 17)))
          point_colour_method = (ColourMethod) BNF_Integer(line);

        else if (!strncmp(keyword, "pointcolour", max(keyword_length, 11))) {
          BNF_Three_Integers(line, &red, &green, &blue);
          point_colour = QColor(red, green, blue);
        }

        else if (!strncmp(keyword, "pointcolourcyclelength",
                          max(keyword_length, 22)))
          point_colour_cycle_length = BNF_Double(line);

        else if (!strncmp(keyword, "pointcolourcyclephase",
                          max(keyword_length, 21)))
          point_colour_cycle_phase = BNF_Double(line);

        else if (!strncmp(keyword, "pointcolourgamma", max(keyword_length, 16)))
          point_colour_gamma = BNF_Double(line);

        else if (!strncmp(keyword, "numcolours", max(keyword_length, 10)))
          num_colours = BNF_Integer(line);

        else if (!strncmp(keyword, "selectedpulsetype", max(keyword_length, 17)))
          selected_pulse_type = (LaserPulseType) BNF_Integer(line);

        else if (!strncmp(keyword, "maxpointspacingpyramid", max(keyword_length, 22)))
          maximum_point_spacing_in_pyramid = BNF_Double(line);

        else if (!strncmp(keyword, "maxnumberofpointsmemory", max(keyword_length, 23)))
          maximum_number_of_points_memory = BNF_Integer(line);

        else if (!strncmp(keyword, "maxnumberofpointsdisplay", max(keyword_length, 24)))
          maximum_number_of_points_display = BNF_Integer(line);

        else if (!strncmp(keyword, "heightoffset", max(keyword_length, 12)))
          height_offset = BNF_Double(line);
          
        else if (!strncmp(keyword, "thresholdattribute", max(keyword_length, 18)))
          threshold_attribute = (LaserPointTag) BNF_Integer(line);

        else if (!strncmp(keyword, "numberofthresholds", max(keyword_length, 18)))
          num_thresholds = BNF_Integer(line);

        else if (!strncmp(keyword, "thresholdcolour", 15)) { // All threshold colours 
          it = -1;
          if (keyword_length == 16 || keyword_length == 17) {
            sscanf(keyword+15, "%d", &it);
            if (it >= 0 && it < PCM_MAX_NUM_THRESHOLDS+1) {
              BNF_Three_Integers(line, &red, &green, &blue);
              threshold_colours[it] = QColor(red, green, blue);
            }
          }
          if (it < 0 || it >= PCM_MAX_NUM_THRESHOLDS+1)
            fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
        }

        else if (!strncmp(keyword, "threshold", 9)) { // All thresholds
          it = -1;
          if (keyword_length == 10 || keyword_length == 11) {
            sscanf(keyword+9, "%d", &it);
            if (it >= 0 && it < PCM_MAX_NUM_THRESHOLDS+1)
              thresholds[it] = BNF_Double(line);
          }
          if (it < 0 || it >= PCM_MAX_NUM_THRESHOLDS+1)
            fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
        }

        else if (!strncmp(keyword, "colourinbetweenthresholds", max(keyword_length, 25)))
          colours_in_between_thresholds = BNF_Integer(line);
         
        else if (!strncmp(keyword, "noattributecolour", max(keyword_length, 17))) {
          BNF_Three_Integers(line, &red, &green, &blue);
          no_attribute_colour = QColor(red, green, blue);
        }
         
        // Line attributes
        else if (!strncmp(keyword, "showlines", max(keyword_length, 9)))
          show_lines = ( BNF_Integer(line) != 0 );

        else if (!strncmp(keyword, "linewidth", max(keyword_length, 9)))
          line_width = BNF_Integer(line);

        else if (!strncmp(keyword, "linecolourmethod", max(keyword_length, 16)))
          line_colour_method = (ColourMethod) BNF_Integer(line);

        else if (!strncmp(keyword, "linecolour", max(keyword_length, 10))) {
          BNF_Three_Integers(line, &red, &green, &blue);
          line_colour = QColor(red, green, blue);
        }
        else if (!strncmp(keyword, "onlyshowlinesinsegment", max(keyword_length, 22)))
          only_show_lines_in_segment = ( BNF_Integer(line) != 0 );

        else if (!strncmp(keyword, "firstpointsize", max(keyword_length, 14)))
          first_point_size = BNF_Integer(line);


        // Face attributes
        else if (!strncmp(keyword, "showfaces", max(keyword_length, 9)))
          show_faces = ( BNF_Integer(line) != 0 );

        else if (!strncmp(keyword, "facecolourmethod", max(keyword_length, 16)))
          face_colour_method = (ColourMethod) BNF_Integer(line);

        else if (!strncmp(keyword, "facecolour", max(keyword_length, 10))) {
          BNF_Three_Integers(line, &red, &green, &blue);
          face_colour = QColor(red, green, blue);
        }

        else if (!strncmp(keyword, "triangulatefaces", max(keyword_length, 16)))
          triangulate_faces = ( BNF_Integer(line) != 0 );

        else if (!strncmp(keyword, "onlyshowfacesinsegment", max(keyword_length, 22)))
          only_show_faces_in_segment = ( BNF_Integer(line) != 0 );

        else if (!strncmp(keyword, "enddataappearance", max(keyword_length,
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

// Return the most suitable colouring method for a given tag

ColourMethod TagToColourMethod(const LaserPointTag tag)
{
  switch (tag) {
  	case LabelTag: return ColourByLabel;
  	case ReflectanceTag: return ColourByReflectance;
  	case ZCoordinateTag: return ColourByHeight;
  	case ResidualTag: return ColourByResidual;
  	case SegmentNumberTag: return ColourBySegment;
  	case PulseLengthTag: return GreyToneByPulseLength;
  	case ComponentNumberTag: return ColourByComponent;
  	case SegmentStartTileNumberTag: return ColourBySegmentAndTile;
  	case PulseCountTag: return ColourByPulseCount;
  	case IsFilteredTag: return ColourByFilterStatus;
  	case ScanNumberTag: return ColourByScanNumber;
  	case ScanNumberWithoutAFNTag: return ColourByScanNumberWithoutAFN;
  	case AFNTag: return ColourByAFNCode;
	case ColourTag: return ColourByColour;
	case PlaneNumberTag: return ColourByPlane;
	case ScanLineNumberTag: return ColourByScanLineNumber;
	case TimeTag: return ColourByTime;
	default: return ColourByHeight;
  }
  return ColourByHeight;
}
