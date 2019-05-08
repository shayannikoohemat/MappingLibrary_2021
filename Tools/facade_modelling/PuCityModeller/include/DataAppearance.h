
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


#ifndef DATAAPPEARANCE_H
#define DATAAPPEARANCE_H

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <qcolor.h>
#include "DataTypes.h"
#include "LaserPoint.h"

enum ColourMethod {FixedColour, ColourByLabel, ColourByReflectance,
                   ColourByHeight, GreyToneByHeight, ColourByResidual,
                   ColourBySegment, ColourByPlane, GreyToneByPulseLength,
                   ColourByColour, ColourByScanNumber, ColourBy10Scans};
typedef enum ColourMethod ColourMethod;

enum LabelColour {ColourRed, ColourGreen, ColourBlue, ColourYellow,
                  ColourOrange, ColourLightYellow, ColourSea,
                  ColourWhite, ColourBlack};

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
    /// Size of the first point of the line
    int first_point_size;

// Face attributes

    /// Show faces switch
    bool show_faces;
    /// Face colour method
    ColourMethod face_colour_method;
    /// Face colour
    QColor face_colour;

    /// Highlighted objects are shown with modified attributes
    bool highlighted;

    /// Paint data of this type first
    bool paint_first;

  public:
    /// Default constructor
    DataAppearance() {data_type_name[0] = 0;}

    /// Constructor
    DataAppearance(DataType new_type)
      {SetDataType(new_type); Initialise();}

    /// Copy constructor
    DataAppearance(const DataAppearance &appearance)
      { *this = appearance; }

    /// Default destructor
    ~DataAppearance() {};

    /// Copy assignment
    DataAppearance & operator = (const DataAppearance &);

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
    const DataAppearance *DataAppearancePtr() {return this;}

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

    /// Return the show lines switch value
    bool ShowLines() const {return show_lines;}

    /// Return the line width
    int LineWidth() const {return line_width;}

    /// Return the line colour method
    ColourMethod LineColourMethod() const {return line_colour_method;}

    /// Return the line colour
    QColor LineColour() const {return line_colour;}

    /// Return the first point size
    int FirstPointSize() const {return first_point_size;}

    /// Return the show faces switch value
    bool ShowFaces() const {return show_faces;}

    /// Return the face colour method
    ColourMethod FaceColourMethod() const {return face_colour_method;}

    /// Return the face colour
    QColor FaceColour() const {return face_colour;}

    /// Return the status of the highlighted switch
    bool HighLighted() const {return highlighted;}

    /// Return the status of late painting
    bool PaintFirst() const {return false;}

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
      
    /// Return the selected pulse type
    LaserPulseType SelectedPulseType() const
      { return selected_pulse_type; }
      
    /// Set the selected pulse type
    void SetSelectedPulseType(LaserPulseType type)
      { selected_pulse_type = type; }
      
    void SetPointSizeNew(int new_point_size)
    {
    if (new_point_size > 0 && new_point_size < 10) {
    point_size = new_point_size;
    }
    }
    
    /// Write in BNF format
    void Write(FILE *fd, int indent) const;
    
    /// Read appearance from a BNF formatted file
    void Read(FILE *fd);
    
      
};

#endif // DATAAPPEARANCE_H
