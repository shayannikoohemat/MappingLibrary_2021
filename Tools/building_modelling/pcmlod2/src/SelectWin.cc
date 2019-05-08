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
#include "SelectWin.h"
#include <QPainter>
#include <QValidator>
#include <QPushButton>

SelectWindow::SelectWindow(LaserPoints *laser_points,
                           LaserPoints *selected_laser_points,
                           SegmentationParameters *parameters,
                           DataAppearance *appearance,
                           QWidget *parent) : QWidget(parent)
{
  QDoubleValidator *non_negative_double;
  int              y, i;
  QPushButton      *button;

  // Determine window type
  if (laser_points != NULL) window_type = PointSelection;
  else window_type = DisplayFilter;
  
  // Copy pointers
  if (window_type == PointSelection) {
    points = laser_points;
    selected_points = selected_laser_points;
    segmentation_parameters = parameters;
  }
  else { // Display filter
  	laser_appearance = appearance;
  }
  
  // Validator
  non_negative_double = new QDoubleValidator(0.00, 1.0e10, 2, this);

  if (window_type == PointSelection) setFixedSize(400, 230);
  else setFixedSize(360, 230);
  
  y = 30;

  for (i=0; i<3; i++) {
    // Attribute selector
    point_attribute_selector[i] = new QComboBox(this);
    point_attribute_selector[i]->setEditable(false);
    point_attribute_selector[i]->setFocusPolicy(Qt::NoFocus);
    point_attribute_selector[i]->addItem("-- none --");
    point_attribute_selector[i]->addItem("X-coordinate");
    point_attribute_selector[i]->addItem("Y-coordinate");
    point_attribute_selector[i]->addItem("Z-coordinate");
    point_attribute_selector[i]->addItem("Reflectance");
    point_attribute_selector[i]->addItem("Pulse count");
    point_attribute_selector[i]->addItem("Label");
    point_attribute_selector[i]->addItem("Filter status");
    point_attribute_selector[i]->addItem("Segment number");
    point_attribute_selector[i]->addItem("Plane number");
    point_attribute_selector[i]->addItem("Scan number");
    point_attribute_selector[i]->addItem("Pulse length");
    point_attribute_selector[i]->addItem("Time stamp");
    point_attribute_selector[i]->addItem("Angle");
    point_attribute_selector[i]->addItem("Component");
    point_attribute_selector[i]->addItem("Residual");
    point_attribute_selector[i]->addItem("X-normal");
    point_attribute_selector[i]->addItem("Y-normal");
    point_attribute_selector[i]->addItem("Z-normal");
    point_attribute_selector[i]->addItem("Flatness");
    point_attribute_selector[i]->addItem("Start tile segment");
    point_attribute_selector[i]->addItem("Scan line number");
    point_attribute_selector[i]->addItem("Height above ground");

    // to be extended
    point_attribute_selector[i]->setCurrentIndex(0); // None
    point_attribute_selector[i]->setGeometry(12, y, 150, 18);

    // Minimum value
    point_minimum_line[i] = new QLineEdit("", this);
    point_minimum_line[i]->setValidator(non_negative_double);
    point_minimum_line[i]->setGeometry(172, y, 70, 18);
          
    // Maximum value
    point_maximum_line[i] = new QLineEdit("", this);
    point_maximum_line[i]->setValidator(non_negative_double);
    point_maximum_line[i]->setGeometry(252, y, 70, 18);

    // Logical operation
    if (window_type == PointSelection) {
      operation_selector[i] = new QComboBox(this);
      operation_selector[i]->setEditable(false);
      operation_selector[i]->setFocusPolicy(Qt::NoFocus);
      operation_selector[i]->addItem("AND");
      operation_selector[i]->addItem("OR");
      operation_selector[i]->addItem("END");
      operation_selector[i]->setCurrentIndex(2); // End
      operation_selector[i]->setGeometry(332, y, 50, 18);
    }
    
    // Increment row coordinate
    y += 20;
  }

  y += 30;
  for (i=0; i<3; i++) {
    // Attribute selector
    segment_attribute_selector[i] = new QComboBox(this);
    segment_attribute_selector[i]->setEditable(false);
    segment_attribute_selector[i]->setFocusPolicy(Qt::NoFocus);
    segment_attribute_selector[i]->addItem("-- none --");
    segment_attribute_selector[i]->addItem("Average X-coordinate");
    segment_attribute_selector[i]->addItem("Average Y-coordinate");
    segment_attribute_selector[i]->addItem("Average Z-coordinate");
    segment_attribute_selector[i]->addItem("Average reflectance");
    segment_attribute_selector[i]->addItem("Average filter status");
    segment_attribute_selector[i]->addItem("Average residual");
    segment_attribute_selector[i]->addItem("Average pulse length");
    segment_attribute_selector[i]->addItem("Average time stamp");
    segment_attribute_selector[i]->addItem("Average angle");
    segment_attribute_selector[i]->addItem("Minimum X-coordinate");
    segment_attribute_selector[i]->addItem("Minimum Y-coordinate");
    segment_attribute_selector[i]->addItem("Minimum Z-coordinate");
    segment_attribute_selector[i]->addItem("Minimum reflectance");
    segment_attribute_selector[i]->addItem("Minimum pulse count");
    segment_attribute_selector[i]->addItem("Minimum label");
    segment_attribute_selector[i]->addItem("Minimum residual");
    segment_attribute_selector[i]->addItem("Minimum plane number");
    segment_attribute_selector[i]->addItem("Minimum scan number");
    segment_attribute_selector[i]->addItem("Minimum pulse length");
    segment_attribute_selector[i]->addItem("Minimum polygon number");
    segment_attribute_selector[i]->addItem("Minimum scalar");
    segment_attribute_selector[i]->addItem("Minimum time stamp");
    segment_attribute_selector[i]->addItem("Minimum angle");
    segment_attribute_selector[i]->addItem("Maximum X-coordinate");
    segment_attribute_selector[i]->addItem("Maximum Y-coordinate");
    segment_attribute_selector[i]->addItem("Maximum Z-coordinate");
    segment_attribute_selector[i]->addItem("Maximum reflectance");
    segment_attribute_selector[i]->addItem("Maximum pulse count");
    segment_attribute_selector[i]->addItem("Maximum label");
    segment_attribute_selector[i]->addItem("Maximum residual");
    segment_attribute_selector[i]->addItem("Maximum plane number");
    segment_attribute_selector[i]->addItem("Maximum scan number");
    segment_attribute_selector[i]->addItem("Maximum pulse length");
    segment_attribute_selector[i]->addItem("Maximum polygon number");
    segment_attribute_selector[i]->addItem("Maximum scalar");
    segment_attribute_selector[i]->addItem("Maximum time stamp");
    segment_attribute_selector[i]->addItem("Maximum angle");
    segment_attribute_selector[i]->addItem("Segment size");
    segment_attribute_selector[i]->addItem("Component size");
    segment_attribute_selector[i]->addItem("Percentage first pulse");
    segment_attribute_selector[i]->addItem("Percentage second pulse");
    segment_attribute_selector[i]->addItem("Percentage third pulse");
    segment_attribute_selector[i]->addItem("Percentage fourth pulse");
    segment_attribute_selector[i]->addItem("Percentage last pulse");
    segment_attribute_selector[i]->addItem("Percentage not first pulse");
    segment_attribute_selector[i]->addItem("Percentage not last pulse");
    segment_attribute_selector[i]->addItem("Percentage multiple pulse");
    segment_attribute_selector[i]->addItem("Percentage filtered");
    segment_attribute_selector[i]->addItem("Inclination angle");
    segment_attribute_selector[i]->addItem("Azimuth");
    

    // to be extended
    segment_attribute_selector[i]->setCurrentIndex(0); // None
    segment_attribute_selector[i]->setGeometry(12, y, 150, 18);

    // Minimum value
    segment_minimum_line[i] = new QLineEdit("", this);
    segment_minimum_line[i]->setValidator(non_negative_double);
    segment_minimum_line[i]->setGeometry(172, y, 70, 18);
          
    // Maximum value
    segment_maximum_line[i] = new QLineEdit("", this);
    segment_maximum_line[i]->setValidator(non_negative_double);
    segment_maximum_line[i]->setGeometry(252, y, 70, 18);

    // Logical operation
    if (i < 2 && window_type == PointSelection) {
      operation_selector[i+3] = new QComboBox(this);
      operation_selector[i+3]->setEditable(false);
      operation_selector[i+3]->setFocusPolicy(Qt::NoFocus);
      operation_selector[i+3]->addItem("AND");
      operation_selector[i+3]->addItem("OR");
      operation_selector[i+3]->addItem("END");
      operation_selector[i+3]->setCurrentIndex(2); // End
      operation_selector[i+3]->setGeometry(332, y, 50, 18);
    }
          
    // Increment row coordinate
    y += 20;
  }

  y += 10;
  if (laser_points != NULL) {
    button = new QPushButton("Select", this);
    button->setGeometry(12, y, 60, 25);
    connect(button, SIGNAL(clicked()), this, SLOT(SelectData()));

    button = new QPushButton("Delete", this);
    button->setGeometry(82, y, 60, 25);
    connect(button, SIGNAL(clicked()), this, SLOT(DeleteData()));

    button = new QPushButton("Crop", this);
    button->setGeometry(152, y, 60, 25);
    connect(button, SIGNAL(clicked()), this, SLOT(CropData()));
  
    button = new QPushButton("Clear", this);
    button->setGeometry(222, y, 60, 25);
    connect(button, SIGNAL(clicked()), this, SLOT(ClearSelection()));
  }
  else {
    button = new QPushButton("Set display filter", this);
    button->setGeometry(12, y, 130, 25);
    connect(button, SIGNAL(clicked()), this, SLOT(SetDisplayFilter()));
 	
    button = new QPushButton("Clear display filter", this);
    button->setGeometry(152, y, 130, 25);
    connect(button, SIGNAL(clicked()), this, SLOT(ClearDisplayFilter()));
  }
  button = new QPushButton("Close", this);
  button->setGeometry(292, y, 60, 25);
  connect(button, SIGNAL(clicked()), this, SLOT(hide()));
  
  if (laser_points != NULL) setWindowTitle("Point and segment selection");
  else setWindowTitle("Point and segment display filter");
}

void SelectWindow::paintEvent(QPaintEvent *event)
{
  int y=14;
  QPainter paint(this);

  if (event) y = 14; // To avoid compiler warnings

  // Header text
  paint.drawText(  y, 20, QString("Point attribute"));
  paint.drawText(174, 20, QString("Minimum"));
  paint.drawText(254, 20, QString("Maximum"));
  if (window_type == PointSelection)
    paint.drawText(334, 20, QString("Operation"));
  paint.drawText( 14, 110, QString("Segment attribute"));
  paint.drawText(174, 110, QString("Minimum"));
  paint.drawText(254, 110, QString("Maximum"));
  if (window_type == PointSelection)
    paint.drawText(334, 110, QString("Operation"));
}

void SelectWindow::RetrievePointBounds(int *attributes, double *minima,
                                       double *maxima, int num)
{
  for (int i=0; i<num; i++) {
    switch (point_attribute_selector[i]->currentIndex()) {
      default:
      case  0: attributes[i] = (int) UndefinedTag; break;     // -- none --
      case  1: attributes[i] = (int) XCoordinateTag; break;   // X-coordinate
      case  2: attributes[i] = (int) YCoordinateTag; break;   // Y-coordinate
      case  3: attributes[i] = (int) ZCoordinateTag; break;   // Z-coordinate
      case  4: attributes[i] = (int) ReflectanceTag; break;   // Reflectance
      case  5: attributes[i] = (int) PulseCountTag; break;    // Pulse count
      case  6: attributes[i] = (int) LabelTag; break;         // Label
      case  7: attributes[i] = (int) IsFilteredTag; break;    // Filter status
      case  8: attributes[i] = (int) SegmentNumberTag; break; // Segment number
      case  9: attributes[i] = (int) PlaneNumberTag; break;   // Plane number
      case 10: attributes[i] = (int) ScanNumberTag; break;    // Scan number
      case 11: attributes[i] = (int) PulseLengthTag; break;   // Pulse length
      case 12: attributes[i] = (int) TimeTag; break;          // Time stamp
      case 13: attributes[i] = (int) AngleTag; break;         // Angle
      case 14: attributes[i] = (int) ComponentNumberTag; break; // Component number
      case 15: attributes[i] = (int) ResidualTag; break;      // Residual
      case 16: attributes[i] = (int) NormalXTag; break;       // X-component of normal vector
      case 17: attributes[i] = (int) NormalYTag; break;       // Y-component of normal vector
      case 18: attributes[i] = (int) NormalZTag; break;       // Z-component of normal vector
      case 19: attributes[i] = (int) FlatnessTag; break;      // Flatness of point neighbourhood
      case 20: attributes[i] = (int) SegmentStartTileNumberTag; break; // Start tile of segment
      case 21: attributes[i] = (int) ScanLineNumberTag; break; // Scan line number
      case 22: attributes[i] = (int) HeightAboveGroundTag; break; // Height above the ground
    }
    if (point_minimum_line[i]->text().length() == 0) minima[i] = (double) INT_MIN;
    else minima[i] = point_minimum_line[i]->text().toDouble();
    if (point_maximum_line[i]->text().length() == 0) maxima[i] = (double) INT_MAX;
    else maxima[i] = point_maximum_line[i]->text().toDouble();
  }
}

void SelectWindow::RetrieveSegmentBounds(int *attributes, double *minima,
                                       double *maxima, int num)
{
  for (int i=0; i<num; i++) {
    switch (segment_attribute_selector[i]->currentIndex()) {
      default:
      case  0: attributes[i] = (int) UndefinedTag; break;     // -- none --
      case  1: attributes[i] = (int) AverageXCoordinateTag; break;
      case  2: attributes[i] = (int) AverageYCoordinateTag; break;
      case  3: attributes[i] = (int) AverageZCoordinateTag; break;
      case  4: attributes[i] = (int) AverageReflectanceTag; break;
      case  5: attributes[i] = (int) AverageIsFilteredTag; break;
      case  6: attributes[i] = (int) AverageResidualTag; break;
      case  7: attributes[i] = (int) AveragePulseLengthTag; break;
      case  8: attributes[i] = (int) AverageTimeTag; break;
      case  9: attributes[i] = (int) AverageAngleTag; break;
      case 10: attributes[i] = (int) MinXCoordinateTag; break;
      case 11: attributes[i] = (int) MinYCoordinateTag; break;
      case 12: attributes[i] = (int) MinZCoordinateTag; break;
      case 13: attributes[i] = (int) MinReflectanceTag; break;
      case 14: attributes[i] = (int) MinPulseCountTag; break;
      case 15: attributes[i] = (int) MinLabelTag; break;
      case 16: attributes[i] = (int) MinResidualTag; break;
      case 17: attributes[i] = (int) MinPlaneNumberTag; break;
      case 18: attributes[i] = (int) MinScanNumberTag; break;
      case 19: attributes[i] = (int) MinPulseLengthTag; break;
      case 20: attributes[i] = (int) MinPolygonNumberTag; break;
      case 21: attributes[i] = (int) MinScalarTag; break;
      case 22: attributes[i] = (int) MinTimeTag; break;
      case 23: attributes[i] = (int) MinAngleTag; break;
      case 24: attributes[i] = (int) MaxXCoordinateTag; break;
      case 25: attributes[i] = (int) MaxYCoordinateTag; break;
      case 26: attributes[i] = (int) MaxZCoordinateTag; break;
      case 27: attributes[i] = (int) MaxReflectanceTag; break;
      case 28: attributes[i] = (int) MaxPulseCountTag; break;
      case 29: attributes[i] = (int) MaxLabelTag; break;
      case 30: attributes[i] = (int) MaxResidualTag; break;
      case 31: attributes[i] = (int) MaxPlaneNumberTag; break;
      case 32: attributes[i] = (int) MaxScanNumberTag; break;
      case 33: attributes[i] = (int) MaxPulseLengthTag; break;
      case 34: attributes[i] = (int) MaxPolygonNumberTag; break;
      case 35: attributes[i] = (int) MaxScalarTag; break;
      case 36: attributes[i] = (int) MaxTimeTag; break;
      case 37: attributes[i] = (int) MaxAngleTag; break;
      case 38: attributes[i] = (int) SegmentSizeTag; break;
      case 39: attributes[i] = (int) ComponentSizeTag; break;
      case 40: attributes[i] = (int) PercFirstPulseTag; break;
      case 41: attributes[i] = (int) PercSecondPulseTag; break;
      case 42: attributes[i] = (int) PercThirdPulseTag; break;
      case 43: attributes[i] = (int) PercFourthPulseTag; break;
      case 44: attributes[i] = (int) PercLastPulseTag; break;
      case 45: attributes[i] = (int) PercNotFirstPulseTag; break;
      case 46: attributes[i] = (int) PercNotLastPulseTag; break;
      case 47: attributes[i] = (int) PercMultiPulseTag; break;
      case 48: attributes[i] = (int) PercIsFilteredTag; break;
      case 49: attributes[i] = (int) InclinationTag; break;
      case 50: attributes[i] = (int) AzimuthTag; break;
    }
    if (segment_minimum_line[i]->text().length() == 0) minima[i] = (double) INT_MIN;
    else minima[i] = segment_minimum_line[i]->text().toDouble();
    if (segment_maximum_line[i]->text().length() == 0) maxima[i] = (double) INT_MAX;
    else maxima[i] = segment_maximum_line[i]->text().toDouble();
  }
}

void SelectWindow::SelectData()
{
  SelectData(true);
}

void SelectWindow::SelectData(bool display_selected_data)
{
  int num_attributes=6, attributes[num_attributes], operation;
  double minima[num_attributes], maxima[num_attributes];
  bool initialised=false, points_have_tile_numbers;
  
  // Clear old selection
  if (!selected_points->empty()) selected_points->ErasePoints();
  points->RemoveAttribute(IsSelectedTag);
  
  // Get selection criteria
  RetrievePointBounds(attributes, minima, maxima, 3);
  RetrieveSegmentBounds(attributes+3, minima+3, maxima+3, 3);

  // Calculate point attributes if non-existant
  for (int i=0; i<3; i++)
    if (attributes[i] != UndefinedTag)
      if (!points->HasAttribute((LaserPointTag) attributes[i]))
        points->DerivePointAttribute((LaserPointTag) attributes[i],
		                             *segmentation_parameters);
  
  // Calculate segment attributes if non-existant
  points_have_tile_numbers = points->HasAttribute(SegmentStartTileNumberTag);
  for (int i=0; i<3; i++) {
    if (attributes[i+3] != UndefinedTag) {
      if (!points->HasAttribute((LaserPointTag) attributes[i+3])) {
        if (points_have_tile_numbers) {
          // Use special version for segment numbers including tile numbers
          points->DeriveSegmentAttribute((LaserPointTag) attributes[i+3]);
        }
        else {
          // Allow selection with components
          points->DeriveSegmentAttribute((LaserPointTag) attributes[i+3],
		                           segmentation_parameters->SegmentAttribute());
	    }
	  }
    }
  }
  
  // Set new IsSelectedTag
  operation = 0;
  for (int i=0; i<num_attributes && operation!=3; i++) {
    if (attributes[i] != UndefinedTag) {
      if (!initialised) {
        operation = 0;
        initialised = true;
      }
      // Convert degrees to radians
      if (attributes[i] == InclinationTag ||
          attributes[i] == AzimuthTag) {
        minima[i] *= atan(1.0) / 45.0;
        maxima[i] *= atan(1.0) / 45.0;
      }
      points->Select((LaserPointTag) attributes[i], minima[i],
                     maxima[i], operation);
      if (i < num_attributes - 1)
        operation = operation_selector[i]->currentIndex() + 1;
    }
  }
  
  if (display_selected_data) {
    // Copy selected points to selection buffer
    selected_points->AddTaggedPoints(*points, 1, IsSelectedTag);

    // Refresh display of laser points
    emit RequestDisplayUpdate();
  }
}

void SelectWindow::DeleteData()
{
  // Select data
  SelectData(false);
  
  // Delete selected data
  points->RemoveTaggedPoints(1, IsSelectedTag);
  
  // Update bounds and TIN
  points->DeriveDataBounds(0);
  if (points->GetTIN() != NULL) points->DeriveTIN();


  // Refresh display of remaining laser points
  emit RequestDisplayUpdate();
}

void SelectWindow::CropData()
{
  // Select data
  SelectData(false);
  
  // Copy selected points to selection buffer
  selected_points->AddTaggedPoints(*points, 1, IsSelectedTag);
  
  // Replace the whole data set by the selection
  points->ErasePoints();
  points->swap(*selected_points);
  points->RemoveAttribute(IsSelectedTag);

  // Update bounds and TIN
  points->DeriveDataBounds(0);
  if (points->GetTIN() != NULL || selected_points->GetTIN() != NULL)
    points->DeriveTIN();

  // Refresh display of remaining laser points
  emit RequestDisplayUpdate();
}

void SelectWindow::ClearSelection()
{
  // Delete selected points
  selected_points->ErasePoints();;
  
  // Remove selection tags
  points->RemoveAttribute(IsSelectedTag);

  // Refresh display of remaining laser points
  emit RequestDisplayUpdate();
}

void SelectWindow::SetDisplayFilter()
{
  int    num_attributes=6, attributes[num_attributes];
  double minima[num_attributes], maxima[num_attributes];
  float  factor;

  // Remove old display filter bounds
  laser_appearance->DisplayFilter().Initialise();
 
  // Get selection criteria
  RetrievePointBounds(attributes, minima, maxima, 3);
  RetrieveSegmentBounds(attributes+3, minima+3, maxima+3, 3);
  
  // Set new filter bounds
  for (int i=0; i<num_attributes; i++) {
  	if (attributes[i] == UndefinedTag) continue;
  	switch (AttributeType((LaserPointTag) attributes[i])) {
  	  case IntegerAttributeType:
  	  	if (minima[i] != (double) INT_MIN)
  	  	  laser_appearance->DisplayFilter().Minimum().Attribute((LaserPointTag) attributes[i]) =
  	  	    (int) (minima[i]+0.01);
  	  	if (maxima[i] != (double) INT_MAX)
  	  	  laser_appearance->DisplayFilter().Maximum().Attribute((LaserPointTag) attributes[i]) =
  	  	    (int) (maxima[i]+0.01);
  	  	break;
  	  case FloatAttributeType:
  	  	if ((LaserPointTag) attributes[i] == InclinationTag ||
			(LaserPointTag) attributes[i] == AzimuthTag)
		  factor = atan(1.0) / 45.0; // Convert degrees to radians
		else
		  factor = 1.0;
		if (minima[i] != (double) INT_MIN)
  	  	  laser_appearance->DisplayFilter().Minimum().FloatAttribute((LaserPointTag) attributes[i]) =
  	  	    (float) (minima[i]) * factor;
  	  	if (maxima[i] != (double) INT_MAX)
		  laser_appearance->DisplayFilter().Maximum().FloatAttribute((LaserPointTag) attributes[i]) =
  	  	    (float) (maxima[i]) * factor;
  	  	break;
  	  case DoubleAttributeType:
  	  	if (minima[i] != (double) INT_MIN)
  	  	  laser_appearance->DisplayFilter().Minimum().DoubleAttribute((LaserPointTag) attributes[i]) =
  	  	    minima[i];
  	  	if (maxima[i] != (double) INT_MAX)
		  laser_appearance->DisplayFilter().Maximum().DoubleAttribute((LaserPointTag) attributes[i]) =
  	  	    maxima[i];
  	  	break;
  	  case LongAttributeType: // Not used yet
  	  	break;
  	}
  }

  // Refresh display of  laser points
  emit RequestDisplayFilterUpdate();
}

void SelectWindow::ClearDisplayFilter()
{
  // Remove display filter bounds
  laser_appearance->DisplayFilter().Initialise();
  
  // Reset all entries on the display window
  for (int i=0; i<3; i++) {
  	point_attribute_selector[i]->setCurrentIndex(0); // None
  	segment_attribute_selector[i]->setCurrentIndex(0); // None
  }
  
  // Refresh display of  laser points
  emit RequestDisplayFilterUpdate();
}
