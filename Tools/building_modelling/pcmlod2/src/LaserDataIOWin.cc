
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


#include <math.h>
#include <QPainter>
#include <QProgressDialog>
#include <QLabel>
#include "LaserDataIOWin.h"

LaserDataIOWindow::LaserDataIOWindow(QApplication *app, QWidget *parent) :
      QWidget(parent)
{
  int   i, y;
  char  *field_names[15];
  QPushButton *button;
  
  // Copy pointer to main application
  main_application = app;
  
  // Initialise pointers
  filename = NULL;
  laser_points = NULL;

  // Set field names  
  field_names[ 0] = (char *) "X-coordinate";
  field_names[ 1] = (char *) "Y-coordinate";
  field_names[ 2] = (char *) "Z-coordinate";
  field_names[ 3] = (char *) "Reflectance";
  field_names[ 4] = (char *) "Red";
  field_names[ 5] = (char *) "Green";
  field_names[ 6] = (char *) "Blue";
  field_names[ 7] = (char *) "Pulse count";
  field_names[ 8] = (char *) "Label";
  field_names[ 9] = (char *) "Filter status";
  field_names[10] = (char *) "Residual";
  field_names[11] = (char *) "Selected status";
  field_names[12] = (char *) "Segment number";
  field_names[13] = (char *) "Scan number";
  field_names[14] = (char *) "Pulse length";

  setFixedSize(210, 395);

  // Header lines (for import)
  y = 5;
  headerlines_selector = new QSpinBox(this);
  headerlines_selector->setMinimum(0);
  headerlines_selector->setSingleStep(1);
  headerlines_selector->setValue(0);
  headerlines_selector->setGeometry(150, y, 50, 18);

  // Header line (for export)
  header_selector = new QCheckBox("Write number of points in first record", this);
  header_selector->setChecked(false);
  header_selector->setGeometry(5, y, 210, 18);
  header_selector->hide();
  
  // Data fields
  y = 30;
  for (i=0; i<15; i++) {
    y += 20;
    // Attribute check box (not for coordinates)
    if (i > 2) {
      attribute_selector[i] = new QCheckBox(field_names[i], this);
      attribute_selector[i]->setChecked(false);
      attribute_selector[i]->setGeometry(5, y, 100, 18);
    }
    // Column selector
    column_selector[i] = new QSpinBox(this);
    column_selector[i]->setMinimum(1);
    column_selector[i]->setMaximum(20);
    column_selector[i]->setSingleStep(1);
    column_selector[i]->setValue(min(i+1,4));
    column_selector[i]->setGeometry(150, y, 50, 18);
  }
  
  // Cancel, Import and Export buttons
  button = new QPushButton("Cancel", this); y += 30;
  button->setGeometry(5, y, 60, 25);
  connect(button, SIGNAL(clicked()), this, SLOT(hide()));
  import_button = new QPushButton("Import data", this);
  import_button->setGeometry(115, y, 90, 25);
  connect(import_button, SIGNAL(clicked()), this, SLOT(ImportLaserData()));
  export_button = new QPushButton("Export data", this);
  export_button->setGeometry(115, y, 90, 25);
  connect(export_button, SIGNAL(clicked()), this, SLOT(ExportLaserData()));
  export_button->hide();
  currently_importing = true;
}

void LaserDataIOWindow::paintEvent(QPaintEvent *event)
{
  QPainter paint(this);
  int i=5;
  
  if (event) i=5; // Just to avoid a compiler warning

  if (currently_importing)
    paint.drawText(i, 18, QString("Number of header lines"));
  paint.drawText(5, 45, QString("Field"));
  paint.drawText(150, 45, QString("Column"));
  paint.drawText(25, 62, QString("X-coordinate"));
  paint.drawText(25, 82, QString("Y-coordinate"));
  paint.drawText(25, 102, QString("Z-coordinate"));
}

void LaserDataIOWindow::Import(char *name, LaserPoints *points)
{
  // Show and hide correct widgets
  if (!currently_importing) {
    headerlines_selector->show();
    import_button->show();
    header_selector->hide();
    export_button->hide();
    currently_importing = true;
  }
  // Copy file name
  if (filename) free(filename);
  filename = (char *) malloc(strlen(name) + 1);
  strcpy(filename, name);
  // Copy pointer to laser data
  laser_points = points;
  // Show import window
  setWindowTitle("Import laser data");
  show();
}

void LaserDataIOWindow::Export(char *name, LaserPoints *points)
{
  // Show and hide correct widgets
  if (currently_importing) {
    headerlines_selector->hide();
    import_button->hide();
    header_selector->show();
    export_button->show();
    currently_importing = false;
  }
  // Copy file name
  if (filename) free(filename);
  filename = (char *) malloc(strlen(name) + 1);
  strcpy(filename, name);
  // Copy pointer to laser data
  laser_points = points;
  // Show import window
  setWindowTitle("Export laser data");
  show();
}

void LaserDataIOWindow::ImportLaserData()
{
  FILE                  *ascii;
  int                   i, num_header_lines, range;
  char                  *line, *comma;
  double                value[21];
  LaserPoint            point;
  LaserPoints::iterator laser_point;
  QLabel                *label;
  
  // Open the input file
  ascii = Open_Compressed_File(filename, "r");
  if (!ascii) {
    fprintf(stderr, "Error opening input file %s\n", filename);
    return;
  }

  // Skip the header records
  line = (char *) malloc(2048);
  num_header_lines = headerlines_selector->value();
  for (i=0; i<num_header_lines; i++) fgets(line, 2048, ascii);
  if (feof(ascii)) {
    fprintf(stderr, "Error: end of file reached after reading header lines.\n");
    return;
  }

  // Process all records
  range = 100000;
  QProgressDialog progress("Importing data... Percentage of 100000", "Cancel",
                           0, range, this);
  progress.setWindowTitle("PCM I/O progress");
  i= 0;
  do {
    if (fgets(line, 2048, ascii)) {
      // Remove the comma's
      while ((comma = strchr(line, ',')) != NULL) *comma = ' ';
      i++;
      if (1000 * (i/1000) == i) {
        if (i == range) {
          range *= 10;
          progress.setRange(0, range);
          label = new QLabel(QString("Importing data... Percentage of %1").arg(range));
          progress.setLabel(label);
        }
        progress.setValue(i);
        main_application->processEvents();
        if (progress.wasCanceled()) break;
      }

      // Read the next line
      sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
             value+ 1, value+ 2, value+ 3, value+ 4, value+ 5,
             value+ 6, value+ 7, value+ 8, value+ 9, value+10,
             value+11, value+12, value+13, value+14, value+15,
             value+16, value+17, value+18, value+19, value+20);

      // Copy coordinates
      point.X() = value[column_selector[0]->value()];
      point.Y() = value[column_selector[1]->value()];
      point.Z() = value[column_selector[2]->value()];

      // Copy attributes
      if (attribute_selector[3]->isChecked()) // Reflectance
        point.Reflectance() = (int) value[column_selector[3]->value()];
      if (attribute_selector[4]->isChecked() && // Three colour bands
          attribute_selector[5]->isChecked() &&
          attribute_selector[6]->isChecked())
        point.SetColour((int) value[column_selector[4]->value()],
                        (int) value[column_selector[5]->value()],
                        (int) value[column_selector[6]->value()]);
      else if (attribute_selector[4]->isChecked() || // Single bands
               attribute_selector[5]->isChecked() ||
               attribute_selector[6]->isChecked()) {
        point.SetColour(0, 0, 0);
        if (attribute_selector[4]->isChecked()) // Red band
          point.SetColour((int) value[column_selector[4]->value()], 0, 0);
        if (attribute_selector[5]->isChecked()) // Green band
          point.SetColour(point.Red(),
                          (int) value[column_selector[5]->value()], 0);
        if (attribute_selector[6]->isChecked()) // Blue band
          point.SetColour(point.Red(), point.Green(),
                          (int) value[column_selector[6]->value()]);
      }
      if (attribute_selector[7]->isChecked()) // Pulse count
        point.PulseCountWithFlag() = (int) value[column_selector[7]->value()];
      if (attribute_selector[8]->isChecked()) // Label
        point.Label((int) value[column_selector[8]->value()]);
      if (attribute_selector[9]->isChecked()) // Filter status
        point.Attribute(IsFilteredTag) = (int) value[column_selector[9]->value()];
      if (attribute_selector[10]->isChecked()) // Residual
        point.FloatAttribute(ResidualTag) = value[column_selector[10]->value()];
      if (attribute_selector[11]->isChecked()) // Selected status
        point.Attribute(IsSelectedTag) = (int) value[column_selector[11]->value()];
      if (attribute_selector[12]->isChecked()) // Segment number
        point.Attribute(SegmentNumberTag) = (int) value[column_selector[12]->value()];
      if (attribute_selector[13]->isChecked()) // Scan number
        point.Attribute(ScanNumberTag) = (int) value[column_selector[13]->value()];
      if (attribute_selector[14]->isChecked()) // Pulse length
        point.Attribute(PulseLengthTag) = (int) value[column_selector[14]->value()];
      
      // Add the point
      laser_points->push_back(point);
    }
  } while (!feof(ascii));
  Close_Compressed_File(ascii);

  // Set last pulse flags
  if (attribute_selector[7]->isChecked()) laser_points->SetLastPulseFlags();

  // Remove segment numbers INT_MIN
  if (attribute_selector[12]->isChecked()) {
    for (laser_point = laser_points->begin();
         laser_point != laser_points->end(); laser_point++)
      if (laser_point->Attribute(SegmentNumberTag) == INT_MIN)
        laser_point->RemoveAttribute(SegmentNumberTag);
  }
  
  // Request display of new data in main application window
  emit RequestLaserDataDisplay();
  
  // Done
  progress.setValue(range);
  hide();
}

void LaserDataIOWindow::ExportLaserData()
{
  int column, field, i;
  FILE *ascii;
  LaserPoints::const_iterator point;
  std::vector <int> field_order;
  std::vector <int>::iterator next_field;
  char *line, *string;

  // Open output file
  ascii = fopen(filename, "w");
  if (!ascii) {
    printf("Error opening file %s\n", filename);
    return;
  }
  
  // Write number of points, if requested
  if (header_selector->isChecked()) fprintf(ascii, "%d", (int) laser_points->size());
  
  // Determine field order
  for (column=1; column<=20; column++)
    for (field=0; field<15; field++)
      if (field < 3) {
        if (column_selector[field]->value() == column)
          field_order.push_back(field);
      }
      else {
        if (attribute_selector[field]->isChecked() &&
            column_selector[field]->value() == column)
          field_order.push_back(field);
      }
      
  // Write data
  QProgressDialog progress("Exporting data...", "Cancel", 0,
                           laser_points->size(), this);
  progress.setWindowTitle("PCM I/O progress");
  line = (char *) malloc(2048);
  string = (char *) malloc(128);
  for (point=laser_points->begin(), i=0; point!=laser_points->end();
       point++, i++) {
    if (1000 * (i/1000) == i) {
      progress.setValue(i);
      main_application->processEvents();
      if (progress.wasCanceled()) break;
    }
    line[0] = 0;
    for (next_field=field_order.begin(); next_field!=field_order.end();
         next_field++) {
      switch (*next_field) {
        case 0: sprintf(string, "%10.2f ", point->X()); break;
        case 1: sprintf(string, "%10.2f ", point->Y()); break;
        case 2: sprintf(string, "%6.2f ", point->Z()); break;
        case 3: sprintf(string, "%3d ", point->Reflectance()); break;
        case 4: sprintf(string, "%3d ", point->Red()); break;
        case 5: sprintf(string, "%3d ", point->Green()); break;
        case 6: sprintf(string, "%3d ", point->Blue()); break;
        case 7: sprintf(string, "%1d ", point->PulseCount()); break;
        case 8: sprintf(string, "%d ", point->Label()); break;
        case 9: sprintf(string, "%1d ", point->Attribute(IsFilteredTag)); break;
        case 10: sprintf(string, "%.2f ", point->FloatAttribute(ResidualTag)); break;
        case 11: sprintf(string, "%1d ", point->Attribute(IsSelectedTag)); break;
        case 12: sprintf(string, "%d ", point->Attribute(SegmentNumberTag)); break;
        case 13: sprintf(string, "%d ", point->Attribute(ScanNumberTag)); break;
        case 14: sprintf(string, "%d ", point->Attribute(PulseLengthTag)); break;
        default: printf("Error: invalid field number (%d) in LaserDataIOWindow::ExportLaserData\n",
                        *next_field);
                 exit(0);
      }
      strcat(line, string);
    }
    fprintf(ascii, "%s\n", line);
  }
  progress.setValue(laser_points->size());
  
  // Done
  free(line);
  free(string);
  fclose(ascii);

  // Request export message in main application window
  emit RequestExportMessage();
  hide();
}
