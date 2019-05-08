
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


#include "DataAppearWin.h"
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
  int              y=10;
  QPalette         palette;
  QPixmap          pixmap = QPixmap(8, 8);
  QIcon            icon;

  setFixedSize(300, 300);

// Initialisations with default values

  // Show points switch
  show_points_toggle = new QCheckBox("Show points", this);
  show_points_toggle->setGeometry(10, y, 100, 20);
  show_points_toggle->setChecked(show_points);
  show_points_toggle->setFocusPolicy(Qt::NoFocus);
  connect(show_points_toggle, SIGNAL(toggled(bool)),
          this, SLOT(SetShowPointsSwitch(bool)));
  // Point size
  point_size_editor = new QSpinBox(this);
  point_size_editor->setMinimum(1);
  point_size_editor->setMaximum(9);
  point_size_editor->setSingleStep(1);
  point_size_editor->setValue(point_size);
  y += 20; point_size_editor->setGeometry(100, y, 30, 18);
  connect(point_size_editor, SIGNAL(valueChanged(int)),
          this, SLOT(SetPointSize(int)));
  // Point colour
  point_colour_button = new QPushButton(NULL, this);
  y += 23;
  if (data_type == LaserData || data_type == SelectedLaserData) {
    point_colour_mode = new QComboBox(this);
    point_colour_mode->setEditable(false);
    point_colour_mode->setGeometry(110, y, 120, 18);
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
    point_colour_mode->addItem(QString("10 scan numbers"));
    point_colour_button->setGeometry(240, y, 16, 16);
    connect(point_colour_mode, SIGNAL(activated(int)),
            this, SLOT(SetPointColourMethod(int)));
    point_colour_mode->setCurrentIndex(point_colour_method);
    SetPointColourMethod(point_colour_method);
  }
  else point_colour_button->setGeometry(110, y, 16, 16);
  // Point colour cycle length and phase
  if (data_type == LaserData || data_type == SelectedLaserData) {
    // Point colour cycle length
    length_editor = new QDoubleSpinBox(this);
    length_editor->setMinimum(0.0);
    length_editor->setMaximum(1000.0);
    length_editor->setSingleStep(1.0);
    length_editor->setDecimals(1);
    length_editor->setValue(PointColourCycleLength());
    y += 20; length_editor->setGeometry(140, y, 50, 18);
    connect(length_editor, SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetPointColourCycleLength(const QString &)));
    // Point colour cycle phase
    phase_editor = new QDoubleSpinBox(this);
    phase_editor->setMinimum(0.0);
    phase_editor->setMaximum(1.0);
    phase_editor->setSingleStep(0.1);
    phase_editor->setDecimals(2);
    phase_editor->setValue(PointColourCyclePhase());
    y += 20; phase_editor->setGeometry(140, y, 50, 18);
    connect(phase_editor, SIGNAL(valueChanged(const QString &)),
            this, SLOT(SetPointColourCyclePhase(const QString &)));
  }
  if (data_type == LaserData || data_type == SelectedLaserData) {
    pulse_selector = new QComboBox(this);
    y += 20;
    pulse_selector->setEditable(false);
    pulse_selector->setGeometry(110, y, 120, 18);
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
    pulse_selector->setCurrentIndex(0);
    SetSelectedPulseType(selected_pulse_type);
  }
    
// TODO: button colouring needs to be done through a QPalette
// setPalette and possibly setBackgroundRole
// The palette should then have a background colour

// Old code that worked for Qt 3
//#ifdef linux
//  point_colour_button->setPaletteBackgroundColor(point_colour);
//#else
//  point_colour_button->setBackgroundMode(PaletteBackground);
//  point_colour_button->setBackgroundColor(point_colour);
//#endif

// Code that should work for Qt 4 but doesn't...
//  palette.setColor(point_colour_button->backgroundRole(), point_colour);
//  point_colour_button->setPalette(palette);
  
  // Work-around using a pixmap
  pixmap.fill(point_colour);
  icon = QIcon(pixmap);
  point_colour_button->setIcon(icon);
  
  connect(point_colour_button, SIGNAL(clicked()),
          this, SLOT(SetPointColour()));

  // Show lines switch
  show_lines_toggle = new QCheckBox(this);
  y += 37;
  if (data_type == LaserData) {
    show_lines_toggle->setText("Show TIN lines");
    show_lines_toggle->setGeometry(10, y, 120, 20);
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
  y += 20; line_width_editor->setGeometry(100, y, 30, 18);
  connect(line_width_editor, SIGNAL(valueChanged(int)),
          this, SLOT(SetLineWidth(int)));
  // Line colour
  line_colour_button = new QPushButton(NULL, this);
  y += 20;
  if (data_type != LaserData) {
    line_colour_mode = new QComboBox(this);
    line_colour_mode->setEditable(false);
    line_colour_mode->setGeometry(110, y, 70, 18);
    line_colour_mode->setFocusPolicy(Qt::NoFocus);
    line_colour_mode->addItem(QString("Fixed"));
    line_colour_button->setGeometry(190, y, 16, 16);
    line_colour_mode->addItem(QString("Label"));
    line_colour_mode->setCurrentIndex(line_colour_method);
    SetLineColourMethod(line_colour_method);
    connect(line_colour_mode, SIGNAL(activated(int)),
            this, SLOT(SetLineColourMethod(int)));
  }
  else line_colour_button->setGeometry(110, y, 16, 16);
  // Work-around using a pixmap
  pixmap.fill(line_colour);
  icon = QIcon(pixmap);
  line_colour_button->setIcon(icon);
  
  connect(line_colour_button, SIGNAL(clicked()),
          this, SLOT(SetLineColour()));

  // Show faces switch
  show_faces_toggle = new QCheckBox(this);
  y += 37;
  if (data_type == LaserData) {
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
  if (data_type != LaserData) {
    face_colour_mode = new QComboBox(this);
    face_colour_mode->setEditable(false);
    face_colour_mode->setGeometry(110, y, 70, 18);
    face_colour_mode->setFocusPolicy(Qt::NoFocus);
    face_colour_mode->addItem(QString("Fixed"));
    face_colour_button->setGeometry(190, y, 16, 16);
    face_colour_mode->addItem(QString("Label"));
    face_colour_mode->setCurrentIndex(face_colour_method);
    SetFaceColourMethod(face_colour_method);
    connect(face_colour_mode, SIGNAL(activated(int)),
            this, SLOT(SetFaceColourMethod(int)));
  }
  else face_colour_button->setGeometry(110, y, 16, 16);
  // Work-around using a pixmap
  pixmap.fill(face_colour);
  icon = QIcon(pixmap);
  face_colour_button->setIcon(icon);

  connect(face_colour_button, SIGNAL(clicked()),
          this, SLOT(SetFaceColour()));
}

void DataAppearanceWindow::Update()
{
  QPixmap pixmap = QPixmap(8, 8);
  QIcon   icon;
  
  // Point attributes
  show_points_toggle->setChecked(show_points);
  point_size_editor->setValue(point_size);
  if (data_type == LaserData || data_type == SelectedLaserData) {
    point_colour_mode->setCurrentIndex(point_colour_method);
    SetPointColourMethod(point_colour_method);
    length_editor->setValue(PointColourCycleLength());
    phase_editor->setValue(PointColourCyclePhase());
    pulse_selector->setCurrentIndex(selected_pulse_type); 
    SetSelectedPulseType(selected_pulse_type);
  }
  pixmap.fill(point_colour);
  icon = QIcon(pixmap);
  point_colour_button->setIcon(icon);
  
  // Line attributes
  show_lines_toggle->setChecked(show_lines);
  line_width_editor->setValue(line_width);
  if (data_type != LaserData) {
    line_colour_mode->setCurrentIndex(line_colour_method);
    SetLineColourMethod(line_colour_method);
  }
  pixmap.fill(line_colour);
  icon = QIcon(pixmap);
  line_colour_button->setIcon(icon);

  // Face attributes
  show_faces_toggle->setChecked(show_faces);
  if (data_type != LaserData) {
    face_colour_mode->setCurrentIndex(face_colour_method);
    SetFaceColourMethod(face_colour_method);
  }  
  pixmap.fill(face_colour);
  icon = QIcon(pixmap);
  face_colour_button->setIcon(icon);
}

void DataAppearanceWindow::Edit()
{
  setWindowTitle("Appearance of " + QString(DataTypename()) + " data");
  show();
}

void DataAppearanceWindow::paintEvent(QPaintEvent *event)
{
  QPainter paint(this);
  int      yoffset = 0;

  paint.drawText(32,  45, QString("Point size:"));
  paint.drawText(32,  65, QString("Point colour:"));
  if (data_type == LaserData || data_type == SelectedLaserData) {
    yoffset = 40;
    paint.drawText(32, 85, QString("Colour cycle length:"));
    paint.drawText(32, 105, QString("Colour cycle phase:"));
    paint.drawText(32, 125, QString("Pulse selection:"));
  }
  paint.drawText(32, 125 + yoffset, QString("Line width:"));
  paint.drawText(32, 145 + yoffset, QString("Line colour:"));
  paint.drawText(32, 205 + yoffset, QString("Face colour:"));
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
  if (new_method == FixedColour) point_colour_button->show();
  else point_colour_button->hide();
  if (new_method != point_colour_method) {
    point_colour_method = (ColourMethod) new_method;
    emit ChangedSetting();
  }
}

void DataAppearanceWindow::SetPointColour()
{
  QColor  new_point_colour = QColorDialog::getColor(point_colour, this);
  QPixmap pixmap = QPixmap(8, 8);
  QIcon   icon;
  
  if (new_point_colour.isValid()) {
    point_colour = new_point_colour;
    // Work-around using a pixmap
    pixmap.fill(point_colour);
    icon = QIcon(pixmap);
    point_colour_button->setIcon(icon);
    emit ChangedSetting();
  }
}

void DataAppearanceWindow::SetLineColourMethod(int new_method)
{
  if (new_method == FixedColour) line_colour_button->show();
  else line_colour_button->hide();
  if (new_method != line_colour_method) {
    line_colour_method = (ColourMethod) new_method;
    emit ChangedSetting();
  }
}

void DataAppearanceWindow::SetLineColour()
{
  QColor new_line_colour = QColorDialog::getColor(line_colour, this);
  QPixmap pixmap = QPixmap(8, 8);
  QIcon   icon;

  if (new_line_colour.isValid()) {
    line_colour = new_line_colour;
    // Work-around using a pixmap
    pixmap.fill(line_colour);
    icon = QIcon(pixmap);
    line_colour_button->setIcon(icon);
    emit ChangedSetting();
  }
}

void DataAppearanceWindow::SetFaceColourMethod(int new_method)
{
  if (new_method == FixedColour) face_colour_button->show();
  else face_colour_button->hide();
  if (new_method != face_colour_method) {
    face_colour_method = (ColourMethod) new_method;
    emit ChangedSetting();
  }
}

void DataAppearanceWindow::SetFaceColour()
{
  QColor new_face_colour = QColorDialog::getColor(face_colour, this);
  QPixmap pixmap = QPixmap(8, 8);
  QIcon   icon;

  if (new_face_colour.isValid()) {
    face_colour = new_face_colour;
    // Work-around using a pixmap
    pixmap.fill(face_colour);
    icon = QIcon(pixmap);
    face_colour_button->setIcon(icon);
    emit ChangedSetting();
  }
}

void DataAppearanceWindow::SetPointColourCycleLength
  (const QString &new_length)
{
  point_colour_cycle_length = new_length.toDouble();
  if (point_colour_method == ColourByHeight) emit ChangedSetting();
}

void DataAppearanceWindow::SetPointColourCyclePhase
  (const QString &new_phase)
{ 
  point_colour_cycle_phase = new_phase.toDouble();
  if (point_colour_method == ColourByHeight) emit ChangedSetting();
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
