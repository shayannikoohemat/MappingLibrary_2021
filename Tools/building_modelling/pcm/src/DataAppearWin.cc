
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


#include "DataAppearWin.h"
#include "SelectWin.h"
#include <QColorDialog>
#include <QPainter>
#include <QPalette>
#include <QIcon>
#include <QPixmap>
#include <QValidator>
#include <QLineEdit>
#include "digphot_arch.h"

DataAppearanceWindow::DataAppearanceWindow() :
  QWidget(NULL), DataAppearance(MapData)
{
  Initialise();
}

DataAppearanceWindow::DataAppearanceWindow(DataType type, QWidget *parent) :
  QWidget(parent), DataAppearance(type)
{
  Initialise();
}

DataAppearanceWindow::DataAppearanceWindow
  (const DataAppearance &appearance, QWidget *parent) :
  QWidget(parent), DataAppearance(appearance)
{
  Initialise();
}

void DataAppearanceWindow::Initialise()
{
  int              y=10, y2=32, line_colour_method_index, it,
                   face_colour_method_index, threshold_attribute_index;
  QPalette         palette;

  if ((data_type == LaserData || data_type == SelectedLaserData) &&
      point_colour_method == ColourByThresholding) setFixedSize(520, 440);
  else setFixedSize(280, 440);

// Point appearance

  // Show points switch
  show_points_toggle = new QCheckBox("Show points", this);
  show_points_toggle->setGeometry(10, y, 100, 20);
  show_points_toggle->setChecked(show_points);
  show_points_toggle->setFocusPolicy(Qt::NoFocus);
  connect(show_points_toggle, SIGNAL(toggled(bool)),
          this, SLOT(SetShowPointsSwitch(bool)));

/* Loose points currently not used
  // Show loose points switch
  if (data_type == LastModelData) {
    show_loose_points_toggle = new QCheckBox("Show loose points", this);
    y += 20; show_loose_points_toggle->setGeometry(10, y, 120, 20);
    show_loose_points_toggle->setChecked(show_loose_points);
    show_loose_points_toggle->setFocusPolicy(Qt::NoFocus);
    connect(show_loose_points_toggle, SIGNAL(toggled(bool)),
            this, SLOT(SetShowLoosePointsSwitch(bool)));
  }
*/

  // Point size
  point_size_editor = new QSpinBox(this);
  point_size_editor->setMinimum(1);
  point_size_editor->setMaximum(9);
  point_size_editor->setSingleStep(1);
  point_size_editor->setValue(point_size);
  y += 22; point_size_editor->setGeometry(210, y, 30, 18);
  connect(point_size_editor, SIGNAL(valueChanged(int)),
          this, SLOT(SetPointSize(int)));

  // Point colour
  point_colour_button = new QPushButton(NULL, this);
  y += 20; // was 23
  if (data_type == LaserData || data_type == SelectedLaserData) {
    point_colour_mode = new QComboBox(this);
    point_colour_mode->setEditable(false);
    point_colour_mode->setGeometry(120, y, 120, 18);
    point_colour_mode->setFocusPolicy(Qt::NoFocus);
    point_colour_mode->addItem(QString("Fixed"));
    point_colour_mode->addItem(QString("Label"));
    point_colour_mode->addItem(QString("Reflectance"));
    point_colour_mode->addItem(QString("Height (colour)"));
    point_colour_mode->addItem(QString("Height (grey)"));
    point_colour_mode->addItem(QString("Residual"));
    point_colour_mode->addItem(QString("Segment"));
    point_colour_mode->addItem(QString("Plane"));
    point_colour_mode->addItem(QString("Pulse length"));
    point_colour_mode->addItem(QString("Own colour"));
    point_colour_mode->addItem(QString("Scan number"));
    point_colour_mode->addItem(QString("Scan number (AFN)"));
    point_colour_mode->addItem(QString("AFN code"));
    point_colour_mode->addItem(QString("Pulse count"));
    point_colour_mode->addItem(QString("Filter status"));
    point_colour_mode->addItem(QString("StdDev in X"));
    point_colour_mode->addItem(QString("StdDev in Y"));
    point_colour_mode->addItem(QString("StdDev in Z"));
    point_colour_mode->addItem(QString("Component"));
    point_colour_mode->addItem(QString("Thresholding"));
    point_colour_mode->addItem(QString("Segment+Tile"));
    point_colour_mode->addItem(QString("Scan line number"));
    point_colour_mode->addItem(QString("Time"));
    connect(point_colour_mode, SIGNAL(activated(int)),
            this, SLOT(SetPointColourMethod(int)));
    point_colour_mode->setCurrentIndex(point_colour_method);
    point_colour_button->setGeometry(250, y, 16, 16);
    SetPointColourMethod(point_colour_method);
  }
  else point_colour_button->setGeometry(110, y, 16, 16);
  SetButtonColour(point_colour_button, point_colour);    
  connect(point_colour_button, SIGNAL(clicked()),
          this, SLOT(SelectPointColour()));

  // Point colour cycle length and phase and gamma factor
  if (data_type == LaserData || data_type == SelectedLaserData) {
    // Point colour cycle length
    length_editor = new QDoubleSpinBox(this);
    length_editor->setMinimum(0.0);
    length_editor->setMaximum(1000.0);
    length_editor->setSingleStep(1.0);
    length_editor->setDecimals(1);
    length_editor->setValue(PointColourCycleLength());
    y += 20; length_editor->setGeometry(190, y, 50, 18);
    connect(length_editor, SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetPointColourCycleLength(const QString &)));
    // Point colour cycle phase
    phase_editor = new QDoubleSpinBox(this);
    phase_editor->setMinimum(0.0);
    phase_editor->setMaximum(1.0);
    phase_editor->setSingleStep(0.1);
    phase_editor->setDecimals(2);
    phase_editor->setValue(PointColourCyclePhase());
    y += 20; phase_editor->setGeometry(190, y, 50, 18);
    connect(phase_editor, SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetPointColourCyclePhase(const QString &)));
    // Point colour gamma
    gamma_editor = new QDoubleSpinBox(this);
    gamma_editor->setMinimum(0.0);
    gamma_editor->setMaximum(5.0);
    gamma_editor->setSingleStep(0.01);
    gamma_editor->setDecimals(2);
    gamma_editor->setValue(PointColourGamma());
    y += 20; gamma_editor->setGeometry(190, y, 50, 18);
    connect(gamma_editor, SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetPointColourGamma(const QString &)));
    // Number of colours
    num_colours_editor = new QSpinBox(this);
    num_colours_editor->setMinimum(2);
    num_colours_editor->setMaximum(216);
    num_colours_editor->setSingleStep(1);
    num_colours_editor->setValue(num_colours);
    y += 20; num_colours_editor->setGeometry(190, y, 50, 18);
    connect(num_colours_editor, SIGNAL(valueChanged(int)),
            this, SLOT(SetNumberOfColours(int)));
  }
  
  if (data_type == LaserData) {
    // Point spacing in pyramids
    pyramid_spacing_editor = new QDoubleSpinBox(this);
    pyramid_spacing_editor->setMinimum(0.1);
    pyramid_spacing_editor->setMaximum(1000.0);
    pyramid_spacing_editor->setSingleStep(0.5);
    pyramid_spacing_editor->setDecimals(1);
    pyramid_spacing_editor->setValue(PointColourCycleLength());
    y += 20; pyramid_spacing_editor->setGeometry(190, y, 50, 18);
    connect(pyramid_spacing_editor, SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetMaximumPointSpacingInPyramid(const QString &)));

    // Maximum number of points in memory
    maximum_number_of_points_memory_editor = new QSpinBox(this);
    maximum_number_of_points_memory_editor->setMinimum(1);
    maximum_number_of_points_memory_editor->setMaximum(10000000);
    maximum_number_of_points_memory_editor->setSingleStep(100000);
    maximum_number_of_points_memory_editor->setValue(maximum_number_of_points_memory);
    y += 20; maximum_number_of_points_memory_editor->setGeometry(170, y, 70, 18);
    connect(maximum_number_of_points_memory_editor, SIGNAL(valueChanged(int)),
            this, SLOT(SetMaximumNumberOfPointsInMemory(int)));

    // Maximum number of points to display
    maximum_number_of_points_display_editor = new QSpinBox(this);
    maximum_number_of_points_display_editor->setMinimum(1);
    maximum_number_of_points_display_editor->setMaximum(10000000);
    maximum_number_of_points_display_editor->setSingleStep(100000);
    maximum_number_of_points_display_editor->setValue(maximum_number_of_points_display);
    y += 20; maximum_number_of_points_display_editor->setGeometry(170, y, 70, 18);
    connect(maximum_number_of_points_display_editor, SIGNAL(valueChanged(int)),
            this, SLOT(SetMaximumNumberOfPointsToDisplay(int)));
    
    // Threshold attribute
    threshold_attribute_selector = new QComboBox(this);
    threshold_attribute_selector->setEditable(false);
    threshold_attribute_selector->setGeometry(400, y2, 90, 18);
    threshold_attribute_selector->setFocusPolicy(Qt::NoFocus);
    threshold_attribute_selector->addItem(QString("Z-coordinate"));
    threshold_attribute_selector->addItem(QString("Residual"));
    threshold_attribute_selector->addItem(QString("Label"));
    threshold_attribute_selector->addItem(QString("Scan number"));
    threshold_attribute_selector->addItem(QString("Scan nubmer without AFN"));
    threshold_attribute_selector->addItem(QString("Component"));
    threshold_attribute_index = ThresholdAttributeIndex(threshold_attribute);
    threshold_attribute_selector->setCurrentIndex(threshold_attribute_index);
    connect(threshold_attribute_selector, SIGNAL(activated(int)),
            this, SLOT(SetThresholdAttribute(int)));

    // Number of thresholds
    number_of_thresholds_editor = new QSpinBox(this);
    number_of_thresholds_editor->setMinimum(1);
    number_of_thresholds_editor->setMaximum(PCM_MAX_NUM_THRESHOLDS);
    number_of_thresholds_editor->setSingleStep(1);
    number_of_thresholds_editor->setValue(num_thresholds);
    y2 += 20; number_of_thresholds_editor->setGeometry(450, y2, 40, 18);
    connect(number_of_thresholds_editor, SIGNAL(valueChanged(int)),
            this, SLOT(SetNumberOfThresholds(int)));

    // Fixed colour between thresholds toggle 
    fixed_colour_between_thresholds_toggle = new QCheckBox(this);
    fixed_colour_between_thresholds_toggle->setText("Fixed colour between thresholds");
    y2 += 20; fixed_colour_between_thresholds_toggle->setGeometry(285, y2, 200, 20);
    fixed_colour_between_thresholds_toggle->setChecked(colours_in_between_thresholds);
    fixed_colour_between_thresholds_toggle->setFocusPolicy(Qt::NoFocus);
    connect(fixed_colour_between_thresholds_toggle, SIGNAL(toggled(bool)),
            this, SLOT(SetFixedColourBetweenThresholdSwitch(bool)));

    // Threshold editors
    for (it=0; it<PCM_MAX_NUM_THRESHOLDS; it++) {
      threshold_editor[it] = new QDoubleSpinBox(this);
      threshold_editor[it]->setMinimum(-1e10); // No limits
      threshold_editor[it]->setMaximum(1e10);
      threshold_editor[it]->setSingleStep(1.0);
      threshold_editor[it]->setDecimals(2);
      threshold_editor[it]->setValue(thresholds[it]);
    }
    connect(threshold_editor[0], SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetThreshold0(const QString &)));
    connect(threshold_editor[1], SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetThreshold1(const QString &)));
    connect(threshold_editor[2], SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetThreshold2(const QString &)));
    connect(threshold_editor[3], SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetThreshold3(const QString &)));
    connect(threshold_editor[4], SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetThreshold4(const QString &)));
    connect(threshold_editor[5], SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetThreshold5(const QString &)));
    connect(threshold_editor[6], SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetThreshold6(const QString &)));
    connect(threshold_editor[7], SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetThreshold7(const QString &)));
    connect(threshold_editor[8], SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetThreshold8(const QString &)));
    connect(threshold_editor[9], SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetThreshold9(const QString &)));
    connect(threshold_editor[10], SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetThreshold10(const QString &)));
    connect(threshold_editor[11], SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetThreshold11(const QString &)));
    connect(threshold_editor[12], SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetThreshold12(const QString &)));
    connect(threshold_editor[13], SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetThreshold13(const QString &)));
            
    // Threshold colour buttons
    for (it=0; it<PCM_MAX_NUM_THRESHOLDS+1; it++) {
      threshold_colour_button[it] = new QPushButton(NULL, this);
      SetButtonColour(threshold_colour_button[it], threshold_colours[it]);
    }
    connect(threshold_colour_button[0], SIGNAL(clicked()),
            this, SLOT(SelectThresholdColour0()));
    connect(threshold_colour_button[1], SIGNAL(clicked()),
            this, SLOT(SelectThresholdColour1()));
    connect(threshold_colour_button[2], SIGNAL(clicked()),
            this, SLOT(SelectThresholdColour2()));
    connect(threshold_colour_button[3], SIGNAL(clicked()),
            this, SLOT(SelectThresholdColour3()));
    connect(threshold_colour_button[4], SIGNAL(clicked()),
            this, SLOT(SelectThresholdColour4()));
    connect(threshold_colour_button[5], SIGNAL(clicked()),
            this, SLOT(SelectThresholdColour5()));
    connect(threshold_colour_button[6], SIGNAL(clicked()),
            this, SLOT(SelectThresholdColour6()));
    connect(threshold_colour_button[7], SIGNAL(clicked()),
            this, SLOT(SelectThresholdColour7()));
    connect(threshold_colour_button[8], SIGNAL(clicked()),
            this, SLOT(SelectThresholdColour8()));
    connect(threshold_colour_button[9], SIGNAL(clicked()),
            this, SLOT(SelectThresholdColour9()));
    connect(threshold_colour_button[10], SIGNAL(clicked()),
            this, SLOT(SelectThresholdColour10()));
    connect(threshold_colour_button[11], SIGNAL(clicked()),
            this, SLOT(SelectThresholdColour11()));
    connect(threshold_colour_button[12], SIGNAL(clicked()),
            this, SLOT(SelectThresholdColour12()));
    connect(threshold_colour_button[13], SIGNAL(clicked()),
            this, SLOT(SelectThresholdColour13()));
    connect(threshold_colour_button[14], SIGNAL(clicked()),
            this, SLOT(SelectThresholdColour14()));
    
    // No threshold colour
    no_attribute_colour_button = new QPushButton(NULL, this);
    SetButtonColour(no_attribute_colour_button, no_attribute_colour);
    connect(no_attribute_colour_button, SIGNAL(clicked()),
            this, SLOT(SelectNoAttributeColour()));
    
    // Place threshold fields and threshold colour buttons at right location
    PlaceThresholdAndThresholdColourEditors();
  }
    
  if (data_type == MapData) {
    height_offset_editor = new QDoubleSpinBox(this);
    height_offset_editor->setSingleStep(1.0);
    height_offset_editor->setDecimals(1);
    height_offset_editor->setValue(HeightOffsetForDisplay());
    y += 20; height_offset_editor->setGeometry(190, y, 50, 18);
    connect(height_offset_editor, SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetHeightOffsetForDisplay(const QString &)));
  }


// Display filter data
  
  if (data_type == LaserData) {
    // Use display filter switch
    y += 30;
    use_display_filter_toggle = new QCheckBox("Apply display filter", this);
    use_display_filter_toggle->setGeometry(10, y, 150, 20);
    use_display_filter_toggle->setChecked(use_display_filter);
    use_display_filter_toggle->setFocusPolicy(Qt::NoFocus);
    connect(use_display_filter_toggle, SIGNAL(toggled(bool)),
            this, SLOT(SetUseDisplayFilterSwitch(bool)));
            
    // Display filter, first create display filter window
    SelectWindow *select_window = new SelectWindow(NULL, NULL, NULL,
                                                   &(this->DataAppearanceRef()),
												   NULL);
    QPushButton *display_filter_button = new QPushButton("Change...", this);
    display_filter_button->setGeometry(180, y, 60, 20);
    connect(display_filter_button, SIGNAL(clicked()),
	        select_window, SLOT(show()));
    connect(select_window, SIGNAL(RequestDisplayFilterUpdate()),
            this, SLOT(UpdateDisplayFilter()));

    // Selected pulse types
    pulse_selector = new QComboBox(this);
    y += 20;
    pulse_selector->setEditable(false);
    pulse_selector->setGeometry(120, y, 120, 18);
    pulse_selector->setFocusPolicy(Qt::NoFocus);
    pulse_selector->setMaxCount(15);
    pulse_selector->setMaxVisibleItems(15);
    pulse_selector->addItem(QString("All pulses"));
    pulse_selector->addItem(QString("First pulse"));
    pulse_selector->addItem(QString("Second pulse"));
    pulse_selector->addItem(QString("Third pulse"));
    pulse_selector->addItem(QString("Fourth pulse"));
    pulse_selector->addItem(QString("Fifth pulse"));
    pulse_selector->addItem(QString("Last pulse"));
    pulse_selector->addItem(QString("Not first pulse"));
    pulse_selector->addItem(QString("Not last pulse"));
    pulse_selector->addItem(QString("Single pulse")); // Both first and last
    pulse_selector->addItem(QString("Multiple pulse")); // Part of multiple reflections
    pulse_selector->addItem(QString("First nor last pulse")); // Middle reflection
    connect(pulse_selector, SIGNAL(activated(int)),
            this, SLOT(SetSelectedPulseType(int)));
    pulse_selector->setCurrentIndex(PulseTypeIndex(selected_pulse_type));
    SetSelectedPulseType(PulseTypeIndex(selected_pulse_type));
  }
  
// Line appearance

  // Show lines switch
  show_lines_toggle = new QCheckBox(this);
  y += 37;
  if (data_type == LaserData || data_type == SelectedLaserData) {
    show_lines_toggle->setText("Show neighbourhood lines");
    show_lines_toggle->setGeometry(10, y, 200, 20);
  }
  else {
    show_lines_toggle->setText("Show lines");
    show_lines_toggle->setGeometry(10, y, 100, 20);
  }
  show_lines_toggle->setChecked(show_lines);
  show_lines_toggle->setFocusPolicy(Qt::NoFocus);
  connect(show_lines_toggle, SIGNAL(toggled(bool)),
          this, SLOT(SetShowLinesSwitch(bool)));
  // Line width
  line_width_editor = new QSpinBox(this);
  line_width_editor->setMinimum(1);
  line_width_editor->setMaximum(9);
  line_width_editor->setSingleStep(1);
  line_width_editor->setValue(line_width);
  y += 22; line_width_editor->setGeometry(100, y, 30, 18);
  connect(line_width_editor, SIGNAL(valueChanged(int)),
          this, SLOT(SetLineWidth(int)));
  // Line colour
  line_colour_button = new QPushButton(NULL, this);
  y += 20;
  if (data_type != LaserData && data_type != SelectedLaserData) {
    line_colour_mode = new QComboBox(this);
    line_colour_mode->setEditable(false);
    line_colour_mode->setGeometry(110, y, 120, 18);
    line_colour_mode->setFocusPolicy(Qt::NoFocus);
    line_colour_mode->addItem(QString("Fixed"));
    line_colour_button->setGeometry(240, y, 16, 16);
    line_colour_mode->addItem(QString("Label"));
    line_colour_mode->addItem(QString("Line number"));
    line_colour_mode->addItem(QString("Building number"));
    line_colour_mode->addItem(QString("Building part number"));
    line_colour_method_index = LineColourMethodIndex(line_colour_method);
    line_colour_mode->setCurrentIndex(line_colour_method_index);
    SetLineColourMethod(line_colour_method_index);
    connect(line_colour_mode, SIGNAL(activated(int)),
            this, SLOT(SetLineColourMethod(int)));
  }
  else line_colour_button->setGeometry(110, y, 16, 16);
  SetButtonColour(line_colour_button, line_colour);
  connect(line_colour_button, SIGNAL(clicked()),
          this, SLOT(SelectLineColour()));

  // Switch for only showing lines in segment
  if (data_type == LaserData) {
    only_show_lines_in_segment_toggle = new QCheckBox(this);
    only_show_lines_in_segment_toggle->setChecked(OnlyShowLinesInSegment());
    y += 20; only_show_lines_in_segment_toggle->setGeometry(32, y+2, 18, 18);
    connect(only_show_lines_in_segment_toggle, SIGNAL(toggled(bool)),
            this, SLOT(SetOnlyShowLinesInSegment(bool)));
  }

// Face appearance

  // Show faces switch
  show_faces_toggle = new QCheckBox(this);
  y += 37;
  if (data_type == LaserData || data_type == SelectedLaserData) {
    show_faces_toggle->setText("Show TIN faces");
    show_faces_toggle->setGeometry(10, y, 120, 20);
  }
  else {
    show_faces_toggle->setText("Show faces");
    show_faces_toggle->setGeometry(10, y, 100, 20);
  }
  show_faces_toggle->setChecked(show_faces);
  show_faces_toggle->setFocusPolicy(Qt::NoFocus);
  connect(show_faces_toggle, SIGNAL(toggled(bool)),
          this, SLOT(SetShowFacesSwitch(bool)));
  // Face colour
  face_colour_button = new QPushButton(NULL, this);
  y += 23;
  face_colour_mode = new QComboBox(this);
  face_colour_mode->setEditable(false);
  face_colour_mode->setFocusPolicy(Qt::NoFocus);
  face_colour_mode->addItem(QString("Fixed"));
  face_colour_mode->addItem(QString("Label"));
  if (data_type == LaserData || data_type == SelectedLaserData) {
    face_colour_mode->setGeometry(120, y, 120, 18);
    face_colour_mode->addItem(QString("Reflectance"));
    face_colour_mode->addItem(QString("Height (colour)"));
    face_colour_mode->addItem(QString("Height (grey)"));
    face_colour_mode->addItem(QString("Residual"));
    face_colour_mode->addItem(QString("Segment"));
    face_colour_mode->addItem(QString("Plane"));
    face_colour_mode->addItem(QString("Pulse length"));
    face_colour_mode->addItem(QString("Own colour"));
    face_colour_mode->addItem(QString("Scan number"));
    face_colour_mode->addItem(QString("Scan number (AFN)"));
    face_colour_mode->addItem(QString("AFN code"));
    face_colour_mode->addItem(QString("Pulse count"));
    face_colour_mode->addItem(QString("Filter status"));
    face_colour_button->setGeometry(250, y, 16, 16);
  }
  else {
    face_colour_mode->setGeometry(110, y, 120, 18);
    face_colour_mode->addItem(QString("Line number"));
    face_colour_mode->addItem(QString("Building number"));
    face_colour_mode->addItem(QString("Building part number"));
    face_colour_button->setGeometry(240, y, 16, 16);
  }
  face_colour_method_index = FaceColourMethodIndex(face_colour_method);
  face_colour_mode->setCurrentIndex(face_colour_method_index);
  SetFaceColourMethod(face_colour_method_index);
  connect(face_colour_mode, SIGNAL(activated(int)),
          this, SLOT(SetFaceColourMethod(int)));
  SetButtonColour(face_colour_button, face_colour);
  connect(face_colour_button, SIGNAL(clicked()),
          this, SLOT(SelectFaceColour()));

  // Switch for only showing faces in segment
  if (data_type == LaserData) {
    only_show_faces_in_segment_toggle = new QCheckBox(this);
    only_show_faces_in_segment_toggle->setChecked(OnlyShowFacesInSegment());
    y += 20; only_show_faces_in_segment_toggle->setGeometry(32, y+2, 18, 18);
    connect(only_show_faces_in_segment_toggle, SIGNAL(toggled(bool)),
            this, SLOT(SetOnlyShowFacesInSegment(bool)));
  }

  // Triangulate faces switch
  if (data_type != LaserData && data_type != SelectedLaserData) {
    triangulate_faces_toggle = new QCheckBox(this);
    y += 20;
    triangulate_faces_toggle->setText("Triangulate faces");
    triangulate_faces_toggle->setGeometry(32, y, 120, 20);
    triangulate_faces_toggle->setChecked(triangulate_faces);
    triangulate_faces_toggle->setFocusPolicy(Qt::NoFocus);
    connect(triangulate_faces_toggle, SIGNAL(toggled(bool)),
            this, SLOT(SetTriangulateFacesSwitch(bool)));
  }
}

void DataAppearanceWindow::PrintThresholds()
{
  printf("ColourByThresholding %d\n", (point_colour_method == ColourByThresholding));
  printf("%d thresholds\n", num_thresholds);
  printf("threshold attribute %d", (int) threshold_attribute);
  if (threshold_attribute == ZCoordinateTag) printf(" = ZCoordinateTag\n");
  else if (threshold_attribute == ResidualTag) printf(" = ResidualTag\n");
  else if (threshold_attribute == LabelTag) printf(" = LabelTag\n");
  else if (threshold_attribute == ScanNumberTag) printf(" = ScanNumberTag\n");
  else if (threshold_attribute == ScanNumberWithoutAFNTag) printf(" = ScanNumberWithoutAFNTag\n");
  else if (threshold_attribute == ComponentNumberTag) printf(" = ComponentNumberTag\n");
  else printf("unknown tag");
  printf("index  = %d\n", ThresholdAttributeIndex(threshold_attribute));
  for (int it=0; it<num_thresholds; it++)
    printf("threshold %.2f  colour %d %d %d\n",
           thresholds[it], threshold_colours[it].red(),
           threshold_colours[it].green(), threshold_colours[it].blue());
  printf("in between %d\n", colours_in_between_thresholds);
} 
  
void DataAppearanceWindow::Update()
{
  int     it;
  
  // Resize window if needed
  if (data_type == LaserData || data_type == SelectedLaserData) {
    if (point_colour_method == ColourByThresholding) setFixedSize(500, 440);
    else setFixedSize(280, 440);
  }
  
  // Point attributes
  show_points_toggle->setChecked(show_points);
/* Loose points currently not used
  if (data_type == LastModelData)
    show_loose_points_toggle->setChecked(show_loose_points);
*/
  point_size_editor->setValue(point_size);
  if (data_type == LaserData || data_type == SelectedLaserData) {
    point_colour_mode->setCurrentIndex(point_colour_method);
    SetPointColourMethod(point_colour_method);
    length_editor->setValue(PointColourCycleLength());
    phase_editor->setValue(PointColourCyclePhase());
    gamma_editor->setValue(PointColourGamma());
    num_colours_editor->setValue(num_colours);
  }
  if (data_type == LaserData) {
    pyramid_spacing_editor->setValue(MaximumPointSpacingInPyramid());
    maximum_number_of_points_memory_editor->setValue(MaximumNumberOfPointsInMemory());
    maximum_number_of_points_display_editor->setValue(MaximumNumberOfPointsToDisplay());
    threshold_attribute_selector->setCurrentIndex(ThresholdAttributeIndex(threshold_attribute));
    SetThresholdAttribute(ThresholdAttributeIndex(threshold_attribute));
    number_of_thresholds_editor->setValue(NumberOfThresholds());
    fixed_colour_between_thresholds_toggle->setChecked(colours_in_between_thresholds);
    for (it=0; it<num_thresholds; it++) {
      threshold_editor[it]->setValue(thresholds[it]);
      SetButtonColour(threshold_colour_button[it], threshold_colours[it]);
    }
    if (colours_in_between_thresholds)
      SetButtonColour(threshold_colour_button[num_thresholds],
                threshold_colours[num_thresholds]);
    SetButtonColour(no_attribute_colour_button, no_attribute_colour);
    PlaceThresholdAndThresholdColourEditors();
  }
  if (data_type == MapData)
    height_offset_editor->setValue(HeightOffsetForDisplay());
  SetButtonColour(point_colour_button, point_colour);
  
  // Point display filter
  if (data_type == LaserData) {
    use_display_filter_toggle->setChecked(use_display_filter);
  	pulse_selector->setCurrentIndex(PulseTypeIndex(selected_pulse_type)); 
    SetSelectedPulseType(PulseTypeIndex(selected_pulse_type));
  }
  
  // Line attributes
  show_lines_toggle->setChecked(show_lines);
  line_width_editor->setValue(line_width);
  if (data_type != LaserData && data_type != SelectedLaserData) {
    line_colour_mode->setCurrentIndex(LineColourMethodIndex(line_colour_method));
    SetLineColourMethod(LineColourMethodIndex(line_colour_method));
  }
  if (data_type == LaserData)
    only_show_lines_in_segment_toggle->setChecked(only_show_lines_in_segment);
  SetButtonColour(line_colour_button, line_colour);

  // Face attributes
  show_faces_toggle->setChecked(show_faces);
  if (data_type != LaserData && data_type != SelectedLaserData) {
    face_colour_mode->setCurrentIndex(FaceColourMethodIndex(face_colour_method));
    SetFaceColourMethod(FaceColourMethodIndex(face_colour_method));
  }  
  SetButtonColour(face_colour_button, face_colour);
  if (data_type == LaserData)
    only_show_faces_in_segment_toggle->setChecked(only_show_faces_in_segment);
  if (data_type != LaserData && data_type != SelectedLaserData)
    triangulate_faces_toggle->setChecked(triangulate_faces);
}

void DataAppearanceWindow::Edit()
{
  setWindowTitle("Appearance of " + QString(DataTypename()) + " data");
  showNormal();
  raise();
}

void DataAppearanceWindow::paintEvent(QPaintEvent *event)
{
  QPainter paint(this);
  int      y = 45, y2 = 25, offset_x = 0, offset_y = 0;

  if (event) y = 45; // Just to avoid a compiler warning
  
  // Point appearance
  
//  if (data_type == LastModelData) y += 20; // For loose points offset
  paint.drawText(32, y, QString("Point size:"));
  y += 20; paint.drawText(32, y, QString("Point colour:"));
  if (data_type == LaserData || data_type == SelectedLaserData) {
    y += 20; paint.drawText(32, y, QString("Colour cycle length:"));
    y += 20; paint.drawText(32, y, QString("Colour cycle phase:"));
    y += 20; paint.drawText(32, y, QString("Colour gamma factor:"));
    y += 20; paint.drawText(32, y, QString("Number of colours:"));
  }
  if (data_type == LaserData) {
    y += 20; paint.drawText(32, y, QString("Pyramid point spacing:"));
    y += 20; paint.drawText(32, y, QString("Maximum # points (memory):"));
    y += 20; paint.drawText(32, y, QString("Maximum # points (display):"));
  }
  if (data_type == MapData) {
    y += 20; paint.drawText(32, y, QString("Height offset for display:"));
  }
  if (data_type == LaserData) {
    PlaceThresholdAndThresholdColourEditors();
    paint.drawText(280, y2, QString("Colouring by thresholding"));
    y2 += 20; paint.drawText(285, y2, QString("Threshold attribute"));
    y2 += 20; paint.drawText(285, y2, QString("Number of thresholds"));
    y2 += 40; paint.drawText(285, y2, QString("Thresholds                Colours"));
    y2 += num_thresholds * 20;
    if (colours_in_between_thresholds) y2 += 20;
    paint.drawText(285, y2+20, QString("No value"));
    // Lines connecting thresholds with colours
    if (colours_in_between_thresholds) {
      offset_x = 20;
      offset_y = 10;
    }
    y2 = 121;
    for (int it=0; it<num_thresholds; it++) {
      paint.drawLine(375, y2+offset_y, 405+offset_x, y2+offset_y);
      y2 += 20;
    }
  }
  // Point display filter
  if (data_type == LaserData) {
    y += 50; paint.drawText(32, y, QString("Pulse selection:"));
  }
  
  // Line appearance
  y += 60; paint.drawText(32, y, QString("Line width:"));
  y += 20;  paint.drawText(32, y, QString("Line colour:"));
  if (data_type == LaserData) {
    y += 20; paint.drawText(52, y, QString("Only show lines in segment"));
  }
  
  // Face appearance
  y += 60;  paint.drawText(32, y, QString("Face colour:"));
  if (data_type == LaserData) {
    y += 20; paint.drawText(52, y, QString("Only show faces in segment"));
  }
}

void DataAppearanceWindow::SetPointSize(int new_point_size)
{
  if (new_point_size > 0 && new_point_size < 10) {
    point_size = new_point_size;
    emit ChangedSetting();
  }
}

void DataAppearanceWindow::SetLineWidth(int new_line_width)
{
  if (new_line_width > 0 && new_line_width < 10) {
    line_width = new_line_width;
    emit ChangedSetting();
  }
}

void DataAppearanceWindow::SetPointColourMethod(int new_method)
{
  ColourMethod old_method;
  if (new_method == FixedColour) point_colour_button->show();
  else point_colour_button->hide();
  if (new_method != point_colour_method) {
    old_method = point_colour_method;
    point_colour_method = (ColourMethod) new_method;
    if (old_method == ColourByThresholding || new_method == ColourByThresholding)
      Update(); // To adjust window size
    emit ChangedSetting();
  }
}

void DataAppearanceWindow::SetThresholdAttribute(int new_index)
{
  LaserPointTag old_attribute = threshold_attribute;
  
  switch (new_index) {
    default:
    case 0: threshold_attribute = ZCoordinateTag; break;
    case 1: threshold_attribute = ResidualTag; break;
    case 2: threshold_attribute = LabelTag; break;
    case 3: threshold_attribute = ScanNumberTag; break;
    case 4: threshold_attribute = ScanNumberWithoutAFNTag; break;
    case 5: threshold_attribute = ComponentNumberTag; break;
  }
  if (threshold_attribute != old_attribute) emit ChangedSetting();
}

int DataAppearanceWindow::ThresholdAttributeIndex(LaserPointTag tag)
{
  switch (tag) {
    default:
    case ZCoordinateTag:          return 0;
    case ResidualTag:             return 1;
    case LabelTag:                return 2;
    case ScanNumberTag:           return 3;
    case ScanNumberWithoutAFNTag: return 4;
    case ComponentNumberTag:      return 5;
  }
  return 0; 
}

void DataAppearanceWindow::SetNumberOfThresholds(int new_number)
{
  if (new_number != num_thresholds) {
    num_thresholds = new_number;
    // Set visibility of threshold edit lines and threshold colour buttons
    PlaceThresholdAndThresholdColourEditors();
    update();
    emit ChangedSetting();
  }
}


void DataAppearanceWindow::SetFixedColourBetweenThresholdSwitch(bool new_state)
{
  colours_in_between_thresholds = new_state;
  // Move colour buttons
  PlaceThresholdAndThresholdColourEditors();
  update();
  emit ChangedSetting();
}

void DataAppearanceWindow::SetThreshold(int index, const QString &new_threshold,
                                        bool refresh)
{
  if (strlen(new_threshold.toLatin1().data()) == 0) return;
  thresholds[index] = new_threshold.toDouble();
  if (refresh) emit ChangedSetting();
}

void DataAppearanceWindow::PlaceThresholdAndThresholdColourEditors()
{
  int y2 = 112, it, offset=0;
  
  if (colours_in_between_thresholds) offset=10;
  
  for (it=0; it<num_thresholds; it++) {
    threshold_editor[it]->show();
    threshold_editor[it]->setGeometry(285, y2+offset, 90, 18);
    threshold_colour_button[it]->show();
    threshold_colour_button[it]->setGeometry(395, y2+1, 16, 16);
    y2 += 20;
  }
  if (colours_in_between_thresholds) {
    threshold_colour_button[num_thresholds]->show();
    threshold_colour_button[num_thresholds]->setGeometry(395, y2+1, 16, 16);
    y2 += 20;
  }
  for (it=num_thresholds; it<PCM_MAX_NUM_THRESHOLDS; it++)
    threshold_editor[it]->hide();
  offset = (int) colours_in_between_thresholds;
  for (it=num_thresholds+offset; it<PCM_MAX_NUM_THRESHOLDS+1; it++)
    threshold_colour_button[it]->hide();
  no_attribute_colour_button->setGeometry(395, y2+1, 16, 16);
}

void DataAppearanceWindow::SelectColour(QPushButton *button, QColor &colour)
{
  QColor new_colour = QColorDialog::getColor(colour, this);

  if (new_colour.isValid()) {
    colour = new_colour;
    SetButtonColour(button, colour);
    emit ChangedSetting();
  }
}

void DataAppearanceWindow::SetButtonColour(QPushButton *button, QColor &colour)
{
  QPixmap pixmap = QPixmap(8, 8);
  QIcon   icon;

  pixmap.fill(colour);
  icon = QIcon(pixmap);
  button->setIcon(icon);
}

void DataAppearanceWindow::SetLineColourMethod(int new_index)
{
  ColourMethod new_method;
  
  // Translate index to colour method
  switch (new_index) {
    case  2: new_method = ColourByLineNumber; break;
    case  3: new_method = ColourByBuildingNumber; break;
    case  4: new_method = ColourByBuildingPartNumber; break;
    default: new_method = (ColourMethod) new_index; break;
  }
    
  // Show/hide colour button
  if (new_method == FixedColour) line_colour_button->show();
  else line_colour_button->hide();
  
  // Change colour method
  if (new_method != line_colour_method) {
    line_colour_method = (ColourMethod) new_method;
    emit ChangedSetting();
  }
}

int DataAppearanceWindow::LineColourMethodIndex(int method)
{
  if (method < 2) return method;
  if (data_type == LaserData || data_type == SelectedLaserData)
    return 0; // Should not happen
  switch (method) {
    case ColourByLineNumber: return 2;
    case ColourByBuildingNumber: return 3;
    case ColourByBuildingPartNumber: return 4;
    default: return 0;
  }
  return 0; // To satisfy compiler
}

void DataAppearanceWindow::SetFaceColourMethod(int new_index)
{
  int new_method;
  
  if (data_type == LaserData || data_type == SelectedLaserData)
    new_method = new_index;
  else {
    switch (new_index) {
      default: new_method = new_index; break;
      case 2: new_method = ColourByLineNumber; break;
      case 3: new_method = ColourByBuildingNumber; break;
      case 4: new_method = ColourByBuildingPartNumber; break;
    }
  }
  if (new_method == FixedColour) face_colour_button->show();
  else face_colour_button->hide();
  if (new_method != face_colour_method) {
    face_colour_method = (ColourMethod) new_method;
    emit ChangedSetting();
  }
}

int DataAppearanceWindow::FaceColourMethodIndex(int method)
{
  if (method < 2) return method;
  if (data_type == LaserData || data_type == SelectedLaserData)
    return method;
  switch (method) {
    case ColourByLineNumber: return 2;
    case ColourByBuildingNumber: return 3;
    case ColourByBuildingPartNumber: return 4;
    default: return 0;
  }
  return 0; // To satisfy compiler
}

void DataAppearanceWindow::SetPointColourCycleLength
  (const QString &new_length)
{
  if (strlen(new_length.toLatin1().data()) == 0) return;
  point_colour_cycle_length = new_length.toDouble();
  if (point_colour_method == ColourByHeight ||
      point_colour_method == ColourByStdDev_Z ||
	  point_colour_method == ColourByTime) emit ChangedSetting();
}

void DataAppearanceWindow::SetPointColourCyclePhase
  (const QString &new_phase)
{ 
  if (strlen(new_phase.toLatin1().data()) == 0) return;
  point_colour_cycle_phase = new_phase.toDouble();
  if (point_colour_method == ColourByHeight ||
      point_colour_method == ColourByStdDev_Z ||
	  point_colour_method == ColourByTime) emit ChangedSetting();
}

void DataAppearanceWindow::SetPointColourGamma
  (const QString &new_gamma)
{ 
  if (strlen(new_gamma.toLatin1().data()) == 0) return;
  point_colour_gamma = new_gamma.toDouble();
  if (point_colour_method == ColourByColour ||
      point_colour_method == ColourByReflectance) emit ChangedSetting();
}

void DataAppearanceWindow::SetNumberOfColours(int new_num)
{
  if (new_num > 1 && new_num < 217) {
    num_colours = new_num;
    emit ChangedSetting();
  }
}

void DataAppearanceWindow::SetSelectedPulseType(int new_index)
{
  LaserPulseType new_type;
  
  if (new_index <= 5) new_type = (LaserPulseType) new_index;
  else if (new_index == 6)  new_type = LastPulse;
  else if (new_index == 7)  new_type = NotFirstPulse;
  else if (new_index == 8)  new_type = NotLastPulse;
  else if (new_index == 9)  new_type = SinglePulse;
  else if (new_index == 10) new_type = MultiplePulse;
  else if (new_index == 11) new_type = FirstNorLastPulse;
  else {
    printf("Invalid index in DataAppearanceWindow::SetSelectedPulseType (%d)\n",
           new_index);
    exit(0);
  }
  if (new_type != selected_pulse_type) {
    selected_pulse_type = new_type;
    emit ChangedSetting();
  }
}

int DataAppearanceWindow::PulseTypeIndex(LaserPulseType type)
{
  if ((int) type <= 5) return (int) type;
  switch (type) {
    case LastPulse: return 6;
    case NotFirstPulse: return 7;
    case NotLastPulse: return 8;
    case SinglePulse: return 9;
    case MultiplePulse: return 10;
    case FirstNorLastPulse: return 11;
    default: printf("Invalid type in DataAppearanceWindow:PulseTypeIndex (%d)\n",
                    type);
  }
  return 0;
}

void DataAppearanceWindow::SetMaximumPointSpacingInPyramid
  (const QString &new_spacing)
{ 
  if (strlen(new_spacing.toLatin1().data()) == 0) return;
  maximum_point_spacing_in_pyramid = new_spacing.toDouble();
  emit ChangedSetting();
}

void DataAppearanceWindow::SetMaximumNumberOfPointsInMemory(int new_max)
{
  if (new_max > 0) {
    maximum_number_of_points_memory = new_max;
    emit ChangedSetting();
  }
}

void DataAppearanceWindow::SetMaximumNumberOfPointsToDisplay(int new_max)
{
  if (new_max > 0) {
    maximum_number_of_points_display = new_max;
    emit ChangedSetting();
  }
}

void DataAppearanceWindow::SetHeightOffsetForDisplay(const QString &new_offset)
{
  if (strlen(new_offset.toLatin1().data()) == 0) return;
  height_offset = new_offset.toDouble();
  emit ChangedSetting();
}
