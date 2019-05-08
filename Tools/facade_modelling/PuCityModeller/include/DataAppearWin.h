
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

    /// Show points check box
    QCheckBox *show_points_toggle;

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

    /// Pulse selector
    QComboBox *pulse_selector;

    /// Show lines check box
    QCheckBox *show_lines_toggle;

    /// Line width editor
    QSpinBox *line_width_editor;

    /// Line colour mode selector
    QComboBox *line_colour_mode;

    /// Line colour button
    QPushButton *line_colour_button;

    /// Show faces check box
    QCheckBox *show_faces_toggle;

    /// Face colour mode selector
    QComboBox *face_colour_mode;

    /// Face colour button
    QPushButton *face_colour_button;

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

  public slots:

    /// Edit the appearance
    void Edit();

    /// Set the show points switch
    void SetShowPointsSwitch(bool new_state)
      {show_points = new_state; emit ChangedSetting();}

    /// Set the point size
    void SetPointSize(int new_point_size);

    /// Set the default point colour
    void SetPointColour();

    /// Set the point colour method
    void SetPointColourMethod(int new_method);

    /// Set the show lines switch
    void SetShowLinesSwitch(bool new_state)
      {show_lines = new_state; emit ChangedSetting();}

    /// Set the line width
    void SetLineWidth(int new_line_width);

    /// Set the default line colour 
    void SetLineColour();

    /// Set the line colour method
    void SetLineColourMethod(int new_method);

    /// Set the show faces switch
    void SetShowFacesSwitch(bool new_state)
      {show_faces = new_state; emit ChangedSetting();}

    /// Set the default face colour 
    void SetFaceColour();

    /// Set the face colour method
    void SetFaceColourMethod(int new_method);

    /// Set the point colour cycle length
    void SetPointColourCycleLength(const QString &new_length);

    /// Set the point colour cycle length
    void SetPointColourCyclePhase(const QString &new_phase);
    
    /// Set the selected pulse type
    void SetSelectedPulseType(int new_type);

  signals:
    /// Some parameter changed
    void ChangedSetting();
};
    
#endif // DATAAPPEARWIN_H
