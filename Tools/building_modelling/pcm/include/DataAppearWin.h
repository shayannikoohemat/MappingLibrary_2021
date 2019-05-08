
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


#ifndef DATAAPPEARWIN_H
#define DATAAPPEARWIN_H

#include <stdio.h>
#include <stdlib.h>
#include "DataAppearance.h"
#include <QWidget>
#include <QPushButton>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QComboBox>

class DataAppearanceWindow : public QWidget, public DataAppearance
{
  Q_OBJECT

  protected:

// Point appearance

    /// Show points check box
    QCheckBox *show_points_toggle;

    /// Show loose points check box
    QCheckBox *show_loose_points_toggle;

    /// Point size spin box
    QSpinBox *point_size_editor;

    /// Point colour button
    QPushButton *point_colour_button;

    /// Point colour mode selector
    QComboBox *point_colour_mode;
    
    /// Colour cycle length editor
    QDoubleSpinBox *length_editor;
    
    /// Colour cycle phase editor
    QDoubleSpinBox *phase_editor;

    /// Colour gamma editor
    QDoubleSpinBox *gamma_editor;

    /// Number of colours spin box
    QSpinBox *num_colours_editor;

    /// Maximum point spacing in pyramid editor
    QDoubleSpinBox *pyramid_spacing_editor;

    /// Maximum number of points in memory spin box
    QSpinBox *maximum_number_of_points_memory_editor;

    /// Maximum number of points to display spin box
    QSpinBox *maximum_number_of_points_display_editor;

    /// Height offset (for display only)
    QDoubleSpinBox *height_offset_editor;
    
    /// Threshold attribute
    QComboBox *threshold_attribute_selector;
    
    /// Number of thresholds
    QSpinBox *number_of_thresholds_editor;
    
    /// Homogeneous colour intervals
    QCheckBox *fixed_colour_between_thresholds_toggle;

    /// Threshold editors
    QDoubleSpinBox *threshold_editor[PCM_MAX_NUM_THRESHOLDS];
    
    /// Threshold colour buttons
    QPushButton *threshold_colour_button[PCM_MAX_NUM_THRESHOLDS+1];
    
    /// No attribute colour button
    QPushButton *no_attribute_colour_button;

// Point display filter

    /// Use display filter check box
    QCheckBox *use_display_filter_toggle;

    /// Pulse selector
    QComboBox *pulse_selector;

// Line appearance

    /// Show lines check box
    QCheckBox *show_lines_toggle;

    /// Line width editor
    QSpinBox *line_width_editor;

    /// Line colour mode selector
    QComboBox *line_colour_mode;

    /// Line colour button
    QPushButton *line_colour_button;

    /// Only show lines in segment check box
    QCheckBox *only_show_lines_in_segment_toggle;
    
// Face appearance

    /// Show faces check box
    QCheckBox *show_faces_toggle;

    /// Face colour mode selector
    QComboBox *face_colour_mode;

    /// Face colour button
    QPushButton *face_colour_button;

    /// Triangulate faces check box
    QCheckBox *triangulate_faces_toggle;

    /// Only show faces in segment check box
    QCheckBox *only_show_faces_in_segment_toggle;
    
  public:

    /// Default constructor
    DataAppearanceWindow();

    /// Constructor for a data type
    DataAppearanceWindow(DataType type, QWidget *parent=NULL);

    /// Construct from an allready defined appearance
    DataAppearanceWindow(const DataAppearance &appearance,
                         QWidget *parent=NULL);

    /// Default destructor
    ~DataAppearanceWindow() {};

  private:
    void Initialise();

  protected:

    /// Paint some text
    void paintEvent(QPaintEvent *event);

  public:

    /// Return the point size spin box
    QSpinBox *PointSizeEditor() {return point_size_editor;}
    
    /// Update the settings of all boxes, toggles and other editors
    void Update();

    /// Debug
    void PrintThresholds();
    
    /// Set a new threshold value
    void SetThreshold(int index, const QString &new_threshold, bool refresh);
    
    /// Place the threshold editors and threshold colour buttons at the right location
    void PlaceThresholdAndThresholdColourEditors();
    
    /// Select a new colour (in variable and on button)
    void SelectColour(QPushButton *button, QColor &color);

    /// Set a new colour on button
    void SetButtonColour(QPushButton *button, QColor &color);

  private:
        
    /// Return the combo box index for a pulse type
    int PulseTypeIndex(LaserPulseType type);

  public slots:

    /// Edit the appearance
    void Edit();

// Point appearance

    /// Set the show points switch
    void SetShowPointsSwitch(bool new_state)
      {show_points = new_state; emit ChangedSetting();}

    /// Set the show points switch
    void SetShowLoosePointsSwitch(bool new_state)
      {show_loose_points = new_state; emit ChangedSetting();}

    /// Set the point size
    void SetPointSize(int new_point_size);

    /// Select the default point colour
    void SelectPointColour()
      {SelectColour(point_colour_button, point_colour);}

    /// Set the point colour method
    void SetPointColourMethod(int new_method);

    /// Set the threshold attribute
    void SetThresholdAttribute(int new_index);
    
    /// Return the index of a threshold attribute
    int ThresholdAttributeIndex(LaserPointTag tag);
    
    /// Set the number of thresholds
    void SetNumberOfThresholds(int new_number);
    
    /// Set fixed colours in between thresholds
    void SetFixedColourBetweenThresholdSwitch(bool new_state);
    
    /// Set threshold 0
    void SetThreshold0(const QString &new_threshold)
      {SetThreshold(0, new_threshold, true);}
      
    /// Set threshold 1
    void SetThreshold1(const QString &new_threshold)
      {SetThreshold(1, new_threshold, true);}
      
    /// Set threshold 2
    void SetThreshold2(const QString &new_threshold)
      {SetThreshold(2, new_threshold, true);}
      
    /// Set threshold 3
    void SetThreshold3(const QString &new_threshold)
      {SetThreshold(3, new_threshold, true);}
      
    /// Set threshold 4
    void SetThreshold4(const QString &new_threshold)
      {SetThreshold(4, new_threshold, true);}
      
    /// Set threshold 5
    void SetThreshold5(const QString &new_threshold)
      {SetThreshold(5, new_threshold, true);}
      
    /// Set threshold 6
    void SetThreshold6(const QString &new_threshold)
      {SetThreshold(6, new_threshold, true);}
      
    /// Set threshold 7
    void SetThreshold7(const QString &new_threshold)
      {SetThreshold(7, new_threshold, true);}
      
    /// Set threshold 8
    void SetThreshold8(const QString &new_threshold)
      {SetThreshold(8, new_threshold, true);}
      
    /// Set threshold 9
    void SetThreshold9(const QString &new_threshold)
      {SetThreshold(9, new_threshold, true);}
      
    /// Set threshold 10
    void SetThreshold10(const QString &new_threshold)
      {SetThreshold(10, new_threshold, true);}
      
    /// Set threshold 11
    void SetThreshold11(const QString &new_threshold)
      {SetThreshold(11, new_threshold, true);}
      
    /// Set threshold 12
    void SetThreshold12(const QString &new_threshold)
      {SetThreshold(12, new_threshold, true);}
      
    /// Set threshold 13
    void SetThreshold13(const QString &new_threshold)
      {SetThreshold(13, new_threshold, true);}
      
    /// Set threshold 14
    void SetThreshold14(const QString &new_threshold)
      {SetThreshold(14, new_threshold, true);}
      
    /// Select threshold colour 0
    void SelectThresholdColour0()
      {SelectColour(threshold_colour_button[0], threshold_colours[0]);}
      
    /// Select threshold colour 1
    void SelectThresholdColour1()
      {SelectColour(threshold_colour_button[1], threshold_colours[1]);}
      
    /// Select threshold colour 2
    void SelectThresholdColour2()
      {SelectColour(threshold_colour_button[2], threshold_colours[2]);}
      
    /// Select threshold colour 3
    void SelectThresholdColour3()
      {SelectColour(threshold_colour_button[3], threshold_colours[3]);}
      
    /// Select threshold colour 4
    void SelectThresholdColour4()
      {SelectColour(threshold_colour_button[4], threshold_colours[4]);}
      
    /// Select threshold colour 5
    void SelectThresholdColour5()
      {SelectColour(threshold_colour_button[5], threshold_colours[5]);}
      
    /// Select threshold colour 6
    void SelectThresholdColour6()
      {SelectColour(threshold_colour_button[6], threshold_colours[6]);}
      
    /// Select threshold colour 7
    void SelectThresholdColour7()
      {SelectColour(threshold_colour_button[7], threshold_colours[7]);}
      
    /// Select threshold colour 8
    void SelectThresholdColour8()
      {SelectColour(threshold_colour_button[8], threshold_colours[8]);}
      
    /// Select threshold colour 9
    void SelectThresholdColour9()
      {SelectColour(threshold_colour_button[9], threshold_colours[9]);}
      
    /// Select threshold colour 10
    void SelectThresholdColour10()
      {SelectColour(threshold_colour_button[10], threshold_colours[10]);}
      
    /// Select threshold colour 11
    void SelectThresholdColour11()
      {SelectColour(threshold_colour_button[11], threshold_colours[11]);}
      
    /// Select threshold colour 12
    void SelectThresholdColour12()
      {SelectColour(threshold_colour_button[12], threshold_colours[12]);}
      
    /// Select threshold colour 13
    void SelectThresholdColour13()
      {SelectColour(threshold_colour_button[13], threshold_colours[13]);}
      
    /// Select threshold colour 14
    void SelectThresholdColour14()
      {SelectColour(threshold_colour_button[14], threshold_colours[14]);}
      
    /// Select threshold colour 15
    void SelectThresholdColour15()
      {SelectColour(threshold_colour_button[15], threshold_colours[15]);}
      
    /// Select no attribute colour
    void SelectNoAttributeColour()
      {SelectColour(no_attribute_colour_button, no_attribute_colour);}
      
    /// Set the point colour cycle length
    void SetPointColourCycleLength(const QString &new_length);

    /// Set the point colour cycle length
    void SetPointColourCyclePhase(const QString &new_phase);
    
    /// Set gamma factor for own point colours
    void SetPointColourGamma(const QString &new_gamma);
    
    /// Set number of colours
    void SetNumberOfColours(int new_number);

    /// Set the maximum point spacing in pyramids
    void SetMaximumPointSpacingInPyramid(const QString &new_spacing);

    /// Set the maximum number of points in memory
    void SetMaximumNumberOfPointsInMemory(int new_max);

    /// Set the maximum number of points to display
    void SetMaximumNumberOfPointsToDisplay(int new_max);

    /// Set the height offset for display
    void SetHeightOffsetForDisplay(const QString &new_offset);
    
// Point display filter

    /// Set the use display filter switch
    void SetUseDisplayFilterSwitch(bool new_state)
      {use_display_filter = new_state; emit ChangedSetting();}

    // Switch on the display filter and force a repaint
    void UpdateDisplayFilter()
      {use_display_filter_toggle->setChecked(true); emit ChangedSetting();}
      
    /// Set the selected pulse type
    void SetSelectedPulseType(int new_type);

    /// Set the show lines switch
    void SetShowLinesSwitch(bool new_state)
      {show_lines = new_state; emit ChangedSetting();}

// Line appearance

    /// Set the line width
    void SetLineWidth(int new_line_width);

    /// Select the default line colour 
    void SelectLineColour()
      {SelectColour(line_colour_button, line_colour);}

    /// Set the line colour method
    void SetLineColourMethod(int new_method);
    
    /// Convert the line colour method to the line colour index of the pull-down menu
    int LineColourMethodIndex(int method);
    
    /// Set the only show lines in segment switch
    void SetOnlyShowLinesInSegment(bool new_state)
      {only_show_lines_in_segment = new_state; emit ChangedSetting();}

// Face appearance

    /// Set the show faces switch
    void SetShowFacesSwitch(bool new_state)
      {show_faces = new_state; emit ChangedSetting();}

    /// Select the default face colour 
    void SelectFaceColour()
      {SelectColour(face_colour_button, face_colour);}

    /// Set the face colour method
    void SetFaceColourMethod(int new_method);

    /// Convert the face colour method to the face colour index of the pull-down menu
    int FaceColourMethodIndex(int method);
    
    /// Set the only show faces in segment switch
    void SetOnlyShowFacesInSegment(bool new_state)
      {only_show_faces_in_segment = new_state; emit ChangedSetting();}

    /// Set the triangulate faces switch
    void SetTriangulateFacesSwitch(bool new_state)
      {triangulate_faces = new_state; 
       if (show_faces) emit ChangedSetting();}

  signals:
    /// Some parameter changed
    void ChangedSetting();
};
    
#endif // DATAAPPEARWIN_H
