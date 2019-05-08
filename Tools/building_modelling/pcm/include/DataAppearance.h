
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


#ifndef DATAAPPEARANCE_H
#define DATAAPPEARANCE_H

#define PCM_MAX_NUM_THRESHOLDS 14

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <qcolor.h>
#include "DataTypes.h"
#include "LaserPoint.h"
#include "DataBoundsLaser.h"

enum ColourMethod {FixedColour, ColourByLabel, ColourByReflectance,
                   ColourByHeight, GreyToneByHeight, ColourByResidual,
                   ColourBySegment, ColourByPlane, GreyToneByPulseLength,
                   ColourByColour, ColourByScanNumber, ColourByScanNumberWithoutAFN,
                   ColourByAFNCode,
                   ColourByPulseCount, ColourByFilterStatus,
		           ColourByStdDev_X, ColourByStdDev_Y, ColourByStdDev_Z, //Added by Markus Gerke
                   ColourByComponent, ColourByThresholding,
                   ColourBySegmentAndTile, ColourByScanLineNumber, ColourByTime,
                   ColourByLineNumber,
                   ColourByBuildingNumber, ColourByBuildingPartNumber}; 
typedef enum ColourMethod ColourMethod;

enum LabelColour {ColourRed, ColourGreen, ColourBlue, ColourYellow,
                  ColourOrange, ColourLightYellow, ColourSea,
                  ColourWhite, ColourBlack};

extern ColourMethod TagToColourMethod(const LaserPointTag tag);


class DataAppearance
{
  protected:
    /// Data type
    DataType data_type;

    /// Data type name
    char data_type_name[25];

// Point attributes

    /// Show points switch
    bool show_points;
    /// Show loose points switch
    bool show_loose_points;
    /// Point size
    int point_size;
    /// Point colour method
    ColourMethod point_colour_method;
    /// Point colour
    QColor point_colour;
    /// Point colour cycle length (for colouring by height)
    double point_colour_cycle_length;
    /// Point colour cycle phase (for colouring by height)
    double point_colour_cycle_phase;
    /// Gamma factor for contrast improvement of own point colours
    double point_colour_gamma;
    /// Point spacing for display of pyramid point clouds
    double maximum_point_spacing_in_pyramid;
    /// Maximum number of points in memory
    int maximum_number_of_points_memory;
    /// Maximum number of points to display
    int maximum_number_of_points_display;
    /// Height offset (for display only)
    double height_offset;
    /// Attribute to be thresholded
    LaserPointTag threshold_attribute;
    /// Number of thresholds
    int num_thresholds;
    /// Thresholds for colour intervals
    double thresholds[PCM_MAX_NUM_THRESHOLDS+1];
    /// Colours on or between thresholds
    QColor threshold_colours[PCM_MAX_NUM_THRESHOLDS+1];
    /// Colour for points without the selected attribute
    QColor no_attribute_colour;
    /// Colours in between thresholds
    bool colours_in_between_thresholds;
    /// Number of colours (for colouring based on labels or segment numbers)
    int num_colours;

// Point display filter

    /// Use display filter switch
    bool use_display_filter;
    /// Minima and maxima values for display filter
    DataBoundsLaser display_filter;
    /// Pulse type selection
    LaserPulseType selected_pulse_type;

// Line attributes

    /// Show lines switch
    bool show_lines;
    /// Line width
    int line_width;
    /// Line colour method
    ColourMethod line_colour_method;
    /// Line colour
    QColor line_colour;
    /// Restrict to lines in same segment
    bool only_show_lines_in_segment;
    /// Size of the first point of the line
    int first_point_size;

// Face attributes

    /// Show faces switch
    bool show_faces;
    /// Face colour method
    ColourMethod face_colour_method;
    /// Face colour
    QColor face_colour;
    /// Triangulate faces
    bool triangulate_faces;
    /// Restrict to faces in same segment
    bool only_show_faces_in_segment;
  
    /// Highlighted objects are shown with modified attributes
    bool highlighted;

    /// Paint data of this type first
    bool paint_first;

  public:
    /// Default constructor
    DataAppearance()
	  { SetDataType(LaserData); Initialise(); }

    /// Constructor
    DataAppearance(DataType new_type)
      { SetDataType(new_type); Initialise(); }

    /// Copy constructor
    DataAppearance(const DataAppearance &appearance)
      { *this = appearance; }

    /// Default destructor
    ~DataAppearance() {};

    /// Copy assignment
    DataAppearance & operator = (const DataAppearance &);

    /// Return reference
    DataAppearance & DataAppearanceRef() {return *this;}
    
    /// Set the data type
    void SetDataType(DataType new_data_type);

    /// Set the data type name
    void SetDataTypeName(const char *name);

    /// Return the data type
    DataType TypeOfData() const {return data_type;}

    /// Return the data type name with small first letter
    const char *DataTypename();

    /// Return the data type name with capital first letter
    const char *DataTypeName();

    /// Return the pointer to this class
    const DataAppearance *DataAppearancePtr() const {return this;}

    /// Return the pointer to this class
    DataAppearance *DataAppearancePtr() {return this;}

    /// Initialise all settings
    void Initialise();

    /// Return the show points switch value
    bool ShowPoints() const {return show_points;}

    /// Return the point size
    int PointSize() const {return point_size;}

    /// Return the point colour method
    ColourMethod PointColourMethod() const {return point_colour_method;}

    /// Return the point colour
    QColor PointColour() const {return point_colour;}
    
    /// Return the point colour cycle length
    double PointColourCycleLength() const {return point_colour_cycle_length;}

    /// Return the point colour cycle phase
    double PointColourCyclePhase() const {return point_colour_cycle_phase;}

    /// Return the gamma value
    double PointColourGamma() const {return point_colour_gamma;}
    
    /// Return the number of colours
    int NumberOfColours() const {return num_colours;}
    
    /// Return the threshold attribute
    LaserPointTag ThresholdAttribute() const {return threshold_attribute;};
    
    /// Return the number of thresholds
    int NumberOfThresholds() const {return num_thresholds;}
    
    /// Return the thresholds
    const double * Thresholds() const {return thresholds;}
    
    /// Set threshold
    void SetThreshold(int index, double value)
      {thresholds[index] = value;}
      
    /// Return the colours on or between thresholds
    const QColor * ThresholdColours() const {return threshold_colours;}
    
    /// Return the colour for points without the selected attribute
    const QColor NoAttributeColour() const {return no_attribute_colour;}
    
    /// Return the usage of colours (in between thresholds or on thresholds
    bool ColoursInBetweenThresholds() const {return colours_in_between_thresholds;}
        
    /// Return the use display filter switch value
    bool UseDisplayFilter() const {return use_display_filter;}

    /// Return the minima and maxima attribute values for display filtering
    DataBoundsLaser DisplayFilter() const {return display_filter;}

    /// Return the selected pulse type
    LaserPulseType SelectedPulseType() const {return selected_pulse_type;}
      
    /// Return the show lines switch value
    bool ShowLines() const {return show_lines;}

    /// Return the line width
    int LineWidth() const {return line_width;}

    /// Return the line colour method
    ColourMethod LineColourMethod() const {return line_colour_method;}

    /// Return the line colour
    QColor LineColour() const {return line_colour;}
    
    /// Return the only show lines in segment switch value
    bool OnlyShowLinesInSegment() const {return only_show_lines_in_segment;}

    /// Return the first point size
    int FirstPointSize() const {return first_point_size;}

    /// Return the show faces switch value
    bool ShowFaces() const {return show_faces;}

    /// Return the face colour method
    ColourMethod FaceColourMethod() const {return face_colour_method;}

    /// Return the face colour
    QColor FaceColour() const {return face_colour;}

    /// Return the only show faces in segment switch value
    bool OnlyShowFacesInSegment() const {return only_show_faces_in_segment;}

    /// Return the triangulate faces switch value
    bool TriangulateFaces() const {return triangulate_faces;}

    /// Return the status of the highlighted switch
    bool HighLighted() const {return highlighted;}

    /// Return the status of late painting
    bool PaintFirst() const {return paint_first;}

    /// Set the point colour method
    void SetPointColourMethod(int new_method)
      { point_colour_method = (ColourMethod) new_method; }

    /// Set the line colour method
    void SetLineColourMethod(int new_method)
      { line_colour_method = (ColourMethod) new_method; }

    /// Set the face colour method
    void SetFaceColourMethod(int new_method)
      { face_colour_method = (ColourMethod) new_method; }

    /// Set the point colour
    void SetPointColour(QColor new_colour)
      { point_colour = new_colour; }

    /// Set the line colour
    void SetLineColour(QColor new_colour)
      { line_colour = new_colour; }

    /// Set the face colour
    void SetFaceColour(QColor new_colour)
      { face_colour = new_colour; }
      
    /// Set the point colour cycle length
    void SetPointColourCycleLength(double new_length)
      { point_colour_cycle_length = new_length; }

    /// Set the point colour cycle length
    void SetPointColourCyclePhase(double new_phase)
      { point_colour_cycle_phase = new_phase; }
      
    /// Set the gamma value
    void SetPointColourGamma(double new_gamma)
      { point_colour_gamma = new_gamma; }
      
    /// Return maximum point spacing in pyramid
    double MaximumPointSpacingInPyramid() const
      { return maximum_point_spacing_in_pyramid; }
      
    /// Return maximum number of points in memory 
    int MaximumNumberOfPointsInMemory() const
      { return maximum_number_of_points_memory; }
      
    /// Return maximum number of points to display
    int MaximumNumberOfPointsToDisplay() const
      { return maximum_number_of_points_display; }
      
    /// Set the maximum point spacing in pyramid
    void SetMaximumPointSpacingInPyramid(double new_spacing)
      { maximum_point_spacing_in_pyramid = new_spacing; }
      
    /// Return height offset
    double HeightOffsetForDisplay() const
      { return height_offset; }
      
    void SetHeightOffsetForDisplay(double offset)
      { height_offset = offset; }
      
    /// Set the selected pulse type
    void SetSelectedPulseType(LaserPulseType type)
      { selected_pulse_type = type; }
      
    /// Return the writable display filter
    DataBoundsLaser &DisplayFilter() {return display_filter;}

    /// Write in BNF format
    void Write(FILE *fd, int indent) const;
    
    /// Read appearance from a BNF formatted file
    void Read(FILE *fd);
};

#endif // DATAAPPEARANCE_H
