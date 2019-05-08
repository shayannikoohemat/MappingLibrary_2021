
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
#include "SegmentParWin.h"
#include <QPainter>
#include <QValidator>

SegmentationParametersWindow::SegmentationParametersWindow(QWidget *parent) :
  QWidget(parent), SegmentationParameters()
{
  QDoubleValidator *positive_double, *non_negative_double, *any_double;
  int              y;
  double           degree = atan(1.0) / 45.0;

  positive_double = new QDoubleValidator(0.01, 1.0e10, 2, this);
  non_negative_double = new QDoubleValidator(0.00, 1.0e10, 2, this);
  any_double = new QDoubleValidator(this);

  setFixedSize(360, 630);

// Plane detection parameters

  // Neighbourhood storage model
  nbh_model_selector = new QComboBox(this);
  nbh_model_selector->setEditable(false);
  nbh_model_selector->setFocusPolicy(Qt::NoFocus);
  nbh_model_selector->addItem(QString("Delaunay TIN"));
  nbh_model_selector->addItem(QString("Octree"));
  nbh_model_selector->addItem(QString("Kd-tree"));
  nbh_model_selector->setCurrentIndex(NeighbourhoodStorageModel());
  y = 25; nbh_model_selector->setGeometry(115, y, 90, 18);
  connect(nbh_model_selector, SIGNAL(activated(int)),
          this, SLOT(SetNeighbourhoodStorageModel(int)));

  // Octree bin maximum number of points
  octree_bin_size_selector = new QSpinBox(this);
  octree_bin_size_selector->setMinimum(1);
  octree_bin_size_selector->setMaximum(9999);
  octree_bin_size_selector->setSingleStep(10);
  octree_bin_size_selector->setValue(OctreeBinMaxNumberOfPoints());
  y += 20; octree_bin_size_selector->setGeometry(220, y, 50, 18);
  connect(octree_bin_size_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetOctreeBinMaxNumberOfPoints(int)));

  // Octree bin overlap
  octree_overlap_line = new QLineEdit(QString("%1").arg(OctreeBinOverlap(), 0, 'f', 1),
                            this);
  octree_overlap_line->setValidator(positive_double);
  y += 20; octree_overlap_line->setGeometry(140, y, 40, 18);
  connect(octree_overlap_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetOctreeBinOverlap(const QString &)));

  // Distance metric dimension
  dim_selector = new QComboBox(this);
  dim_selector->setEditable(false);
  dim_selector->setFocusPolicy(Qt::NoFocus);
  dim_selector->addItem(QString("2D"));
  dim_selector->addItem(QString("3D"));
  dim_selector->setCurrentIndex(DistanceMetricDimension()-2);
  y += 20; dim_selector->setGeometry(140, y, 40, 18);
  connect(dim_selector, SIGNAL(activated(int)),
          this, SLOT(SetDistanceMetricDimension(int)));

  // Number of neighbours in kd-tree
  knn_selector = new QSpinBox(this);
  knn_selector->setMinimum(3);
  knn_selector->setMaximum(100);
  knn_selector->setSingleStep(1);
  knn_selector->setValue(NumberOfNeighbours());
  y += 20; knn_selector->setGeometry(220, y, 50, 18);
  connect(knn_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetNumberOfNeighbours(int)));

  // Maximum distance between two points in a component
  dist_component_line = new QLineEdit(QString("%1").arg(MaxDistanceInComponent(), 0, 'f', 1), this);
  dist_component_line->setValidator(positive_double);
  y += 60; dist_component_line->setGeometry(205, y, 40, 18);
  connect(dist_component_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxDistanceInComponent(const QString &)));

  // Minimum number of points in a component
  component_size_selector = new QSpinBox(this);
  component_size_selector->setMinimum(1);
  component_size_selector->setMaximum(999);
  component_size_selector->setSingleStep(1);
  component_size_selector->setValue(MinNumberOfPointsComponent());
  y += 20; component_size_selector->setGeometry(205, y, 40, 18);
  connect(component_size_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetMinNumberOfPointsComponent(int)));

  // Attribute for storing connected component labelling result
  component_attribute_selector = new QComboBox(this);
  component_attribute_selector->setEditable(false);
  component_attribute_selector->setFocusPolicy(Qt::NoFocus);
  component_attribute_selector->addItem(QString("Label"));
  component_attribute_selector->addItem(QString("Segment number"));
  component_attribute_selector->addItem(QString("Component number"));
  component_attribute_selector->setCurrentIndex(ComponentAttributeIndex());
  y += 20; component_attribute_selector->setGeometry(190, y, 110, 18);
  connect(component_attribute_selector, SIGNAL(activated(int)),
          this, SLOT(SetComponentAttribute(int)));
          
  // Switch for erasing old component labels
  erase_old_label_selector = new QCheckBox(this);
  erase_old_label_selector->setChecked(EraseOldLabels());
  y += 20; erase_old_label_selector->setGeometry(205, y+2, 40, 18);
  connect(erase_old_label_selector, SIGNAL(toggled(bool)),
          this, SLOT(SetEraseOldLabels(bool)));

  // Seed neighbourhood definition
  seed_nbh_selector = new QComboBox(this);
  seed_nbh_selector->setEditable(false);
  seed_nbh_selector->setFocusPolicy(Qt::NoFocus);
  seed_nbh_selector->addItem(QString("Direct neighbours"));
  seed_nbh_selector->addItem(QString("All within radius"));
  seed_nbh_selector->setCurrentIndex(SeedNeighbourhoodDefinition());
  y += 60; seed_nbh_selector->setGeometry(190, y, 110, 18);
  connect(seed_nbh_selector, SIGNAL(activated(int)),
          this, SLOT(SetSeedNeighbourhoodDefinition(int)));
          
  // Seed neighbourhood radius
  seed_radius_line = new QLineEdit(QString("%1").arg(SeedNeighbourhoodRadius(), 0, 'f', 1), this);
  seed_radius_line->setValidator(non_negative_double);
  y += 20; seed_radius_line->setGeometry(205, y, 40, 18);
  connect(seed_radius_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetSeedNeighbourhoodRadius(const QString &)));

  // Maximum slope angle of the Hough space
  max_slope_line = new QLineEdit(QString("%1").arg(MaxSlopeAngle()/degree, 0, 'f', 1), this);
  max_slope_line->setValidator(positive_double);
  y += 20; max_slope_line->setGeometry(205, y, 40, 18);
  connect(max_slope_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxSlopeAngle(const QString &)));
  
  // Bin size of the slope angle in the Hough space
  angle_bin_size_line = new QLineEdit(QString("%1").arg(BinSizeSlopeAngle()/degree, 0, 'f', 1), this);
  angle_bin_size_line->setValidator(positive_double);
  y += 20; angle_bin_size_line->setGeometry(205, y, 40, 18);
  connect(angle_bin_size_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetBinSizeSlopeAngle(const QString &)));
  
  // Bin size of the distance in the Hough space
  dist_bin_size_line = new QLineEdit(QString("%1").arg(BinSizeDistance(), 0, 'f', 1), this);
  dist_bin_size_line->setValidator(positive_double);
  y += 20; dist_bin_size_line->setGeometry(205, y, 40, 18);
  connect(dist_bin_size_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetBinSizeDistance(const QString &)));
  
  // Minimum number of points in a seed
  seed_size_selector = new QSpinBox(this);
  seed_size_selector->setMinimum(3);
  seed_size_selector->setMaximum(999);
  seed_size_selector->setSingleStep(1);
  seed_size_selector->setValue(MinNumberOfPointsSeed());
  y += 20; seed_size_selector->setGeometry(205, y, 40, 18);
  connect(seed_size_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetMinNumberOfPointsSeed(int)));

  // Maximum distance between a point and the seed plane
  dist_seed_line = new QLineEdit(QString("%1").arg(MaxDistanceSeedPlane(), 0, 'f', 2), this);
  dist_seed_line->setValidator(positive_double);
  y += 20; dist_seed_line->setGeometry(205, y, 40, 18);
  connect(dist_seed_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxDistanceSeedPlane(const QString &)));
  
  // Maximum reflectance difference between a point and the seed plane
  refl_dif_seed_line = new QLineEdit(QString("%1").arg(MaxReflectanceDifferenceSeed(), 0, 'f', 0), this);
  refl_dif_seed_line->setValidator(any_double);
  y += 20; refl_dif_seed_line->setGeometry(205, y, 40, 18);
  connect(refl_dif_seed_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxReflectanceDifferenceSeed(const QString &)));
  
  // Surface model
  surface_model_selector = new QComboBox(this);
  surface_model_selector->setEditable(false);
  surface_model_selector->setFocusPolicy(Qt::NoFocus);
  surface_model_selector->addItem(QString("Planar"));
  surface_model_selector->addItem(QString("Smooth"));
  surface_model_selector->setCurrentIndex(SurfaceModel());
  y += 60; surface_model_selector->setGeometry(185, y, 60, 18);
  connect(surface_model_selector, SIGNAL(activated(int)),
          this, SLOT(SetSurfaceModel(int)));
          
  // Growing neighbourhood definition
  growing_nbh_selector = new QComboBox(this);
  growing_nbh_selector->setEditable(false);
  growing_nbh_selector->setFocusPolicy(Qt::NoFocus);
  growing_nbh_selector->addItem(QString("Direct neighbours"));
  growing_nbh_selector->addItem(QString("All within radius"));
  growing_nbh_selector->setCurrentIndex(GrowingNeighbourhoodDefinition());
  y += 20; growing_nbh_selector->setGeometry(240, y, 110, 18);
  connect(growing_nbh_selector, SIGNAL(activated(int)),
          this, SLOT(SetGrowingNeighbourhoodDefinition(int)));
            
  // Radius for surface growing
  growing_radius_line = new QLineEdit(QString("%1").arg(GrowingRadius(), 0, 'f', 1), this);
  growing_radius_line->setValidator(non_negative_double);
  y += 20; growing_radius_line->setGeometry(205, y, 40, 18);
  connect(growing_radius_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetGrowingRadius(const QString &)));

  // Maximum distance between a point and the growing surface
  dist_growing_line = new QLineEdit(QString("%1").arg(MaxDistanceSurface(), 0, 'f', 2), this);
  dist_growing_line->setValidator(positive_double);
  y += 20; dist_growing_line->setGeometry(205, y, 40, 18);
  connect(dist_growing_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxDistanceSurface(const QString &)));
  
  // Minimum distance between a point and the surface to trigger recomputation
  // of the local plane
  dist_recompute_line = new QLineEdit(QString("%1").arg(MinDistanceRecompute(), 0, 'f', 2), this);
  dist_recompute_line->setValidator(positive_double);
  y += 20; dist_recompute_line->setGeometry(240, y, 40, 18);
  connect(dist_recompute_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMinDistanceRecompute(const QString &)));
  
  // Maximum reflectance difference between a point and the growing plane
  refl_dif_growing_line = new QLineEdit(QString("%1").arg(MaxReflectanceDifferenceGrowing(), 0, 'f', 0), this);
  refl_dif_growing_line->setValidator(any_double);
  y += 20; refl_dif_growing_line->setGeometry(205, y, 40, 18);
  connect(refl_dif_growing_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxReflectanceDifferenceGrowing(const QString &)));
  
  // Switch for letting surfaces compete
  compete_selector = new QCheckBox(this);
  compete_selector->setChecked(SurfacesCompete());
  y += 20; compete_selector->setGeometry(205, y+2, 40, 18);
  connect(compete_selector, SIGNAL(toggled(bool)),
          this, SLOT(SetSurfacesCompete(bool)));
}

void SegmentationParametersWindow::Update()
{
  double           degree = atan(1.0) / 45.0;

  nbh_model_selector->setCurrentIndex(NeighbourhoodStorageModel());
  octree_bin_size_selector->setValue(OctreeBinMaxNumberOfPoints());
  octree_overlap_line->setText(QString("%1").arg(OctreeBinOverlap(), 0, 'f', 1));
  dim_selector->setCurrentIndex(DistanceMetricDimension()-2);
  knn_selector->setValue(NumberOfNeighbours());
  dist_component_line->setText(QString("%1").arg(MaxDistanceInComponent(), 0, 'f', 1));
  component_size_selector->setValue(MinNumberOfPointsComponent());
  component_attribute_selector->setCurrentIndex(ComponentAttributeIndex());
  erase_old_label_selector->setChecked(EraseOldLabels());
  seed_nbh_selector->setCurrentIndex(SeedNeighbourhoodDefinition());
  seed_radius_line->setText(QString("%1").arg(SeedNeighbourhoodRadius(), 0, 'f', 1));
  max_slope_line->setText(QString("%1").arg(MaxSlopeAngle()/degree, 0, 'f', 1));
  angle_bin_size_line->setText(QString("%1").arg(BinSizeSlopeAngle()/degree, 0, 'f', 1));
  dist_bin_size_line->setText(QString("%1").arg(BinSizeDistance(), 0, 'f', 1));
  seed_size_selector->setValue(MinNumberOfPointsSeed());
  dist_seed_line->setText(QString("%1").arg(MaxDistanceSeedPlane(), 0, 'f', 2));
  refl_dif_seed_line->setText(QString("%1").arg(MaxReflectanceDifferenceSeed(), 0, 'f', 0));
  surface_model_selector->setCurrentIndex(SurfaceModel());
  growing_nbh_selector->setCurrentIndex(GrowingNeighbourhoodDefinition());
  growing_radius_line->setText(QString("%1").arg(GrowingRadius(), 0, 'f', 1));
  dist_growing_line->setText(QString("%1").arg(MaxDistanceSurface(), 0, 'f', 2));
  dist_recompute_line->setText(QString("%1").arg(MinDistanceRecompute(), 0, 'f', 2));
  refl_dif_growing_line->setText(QString("%1").arg(MaxReflectanceDifferenceGrowing(), 0, 'f', 0));
  compete_selector->setChecked(SurfacesCompete());
}

void SegmentationParametersWindow::paintEvent(QPaintEvent *event)
{
  QPainter paint(this);
  int      y;

  if (event) y = 20; // To avoid compiler warnings
  // Neighbourhood definitions
  y = 20; paint.drawText(12, y, QString("Neighbourhood definitions"));
  y += 20; paint.drawText(32, y, QString("Storage model:"));
  y += 20; paint.drawText(32, y, QString("Octree bin maximum number of points:"));
  y += 20; paint.drawText(32, y, QString("Octree bin overlap:"));
  y += 20; paint.drawText(32, y, QString("Distance metric:"));
  y += 20; paint.drawText(32, y, QString("Number of neighbours in kd-tree:"));
  // Connected component parameters
  y += 40; paint.drawText(12, y, QString("Connected component parameters"));
  y += 20; paint.drawText(32, y, QString("Maximum distance between points:"));
  y += 20; paint.drawText(32, y, QString("Minimum number of points:"));
  y += 20; paint.drawText(32, y, QString("Attribute for component numbers:"));
  y += 20; paint.drawText(32, y, QString("Erase old component labels:"));
  // Seed selection parameters
  y += 40; paint.drawText(12, y, QString("Seed selection parameters"));
  y += 20; paint.drawText(32, y, QString("Seed neighbourhood definition:"));
  y += 20; paint.drawText(32, y, QString("Seed neighbourhood radius:"));
  y += 20; paint.drawText(32, y, QString("Maximum slope angle:"));
  y += 20; paint.drawText(32, y, QString("Bin size slope angle:"));
  y += 20; paint.drawText(32, y, QString("Bin size distance:"));
  y += 20; paint.drawText(32, y, QString("Minimum number of seed points:"));
  y += 20; paint.drawText(32, y, QString("Maximum distance to plane:")); 
  y += 20; paint.drawText(32, y, QString("Maximum reflectance difference:"));    
  // Surface growing parameters
  y += 40; paint.drawText(12, y, QString("Surface growing parameters"));  
  y += 20; paint.drawText(32, y, QString("Surface model:"));
  y += 20; paint.drawText(32, y, QString("Surface growing neighbourhood definition:"));
  y += 20; paint.drawText(32, y, QString("Surface growing radius:"));
  y += 20; paint.drawText(32, y, QString("Maximum distance to surface:"));
  y += 20; paint.drawText(32, y, QString("Minimum distance to recompute local plane:"));
  y += 20; paint.drawText(32, y, QString("Maximum reflectance difference:"));    
  y += 20; paint.drawText(32, y, QString("Competing surfaces:"));
}

void SegmentationParametersWindow::SetNeighbourhoodStorageModel(int new_nbh_model)
{ nbh_model = new_nbh_model; }

void SegmentationParametersWindow::SetOctreeBinMaxNumberOfPoints(int new_number)
{ octree_bin_max_numpts = new_number; }

void SegmentationParametersWindow::SetOctreeBinOverlap(const QString &new_octree_bin_overlap)
{ if (strlen(new_octree_bin_overlap.toLatin1().data()) == 0) return;
  octree_bin_overlap = new_octree_bin_overlap.toDouble(); }

void SegmentationParametersWindow::SetDistanceMetricDimension(int new_dist_dimension)
{ dist_dimension = new_dist_dimension + 2; }

void SegmentationParametersWindow::SetMaxDistanceInComponent(const QString &new_max_dist_connected)
{ if (strlen(new_max_dist_connected.toLatin1().data()) == 0) return;
  max_dist_connected = new_max_dist_connected.toDouble(); }

void SegmentationParametersWindow::SetMinNumberOfPointsComponent(int new_min_numpts_component)
{ min_numpts_component = new_min_numpts_component; }

void SegmentationParametersWindow::SetNumberOfNeighbours(int new_number)
{ num_nbs = new_number; }

void SegmentationParametersWindow::SetComponentAttribute(int new_attribute)
{ 
  switch (new_attribute) {
    case 0: component_attribute = LabelTag; break;
    default:
    case 1: component_attribute = SegmentNumberTag; break;
    case 2: component_attribute = ComponentNumberTag; break;
  }
}

int SegmentationParametersWindow::ComponentAttributeIndex() const
{
  switch ((int) component_attribute) {
    case LabelTag:           return 0;
    case SegmentNumberTag:   return 1;
    case ComponentNumberTag: return 2;
    default: return -1;
  }
  return -1;
}

void SegmentationParametersWindow::SetEraseOldLabels(bool new_switch)
{ erase_old_labels = new_switch; }

void SegmentationParametersWindow::SetSeedNeighbourhoodDefinition(int new_definition)
{ seed_neighbourhood = new_definition; }

void SegmentationParametersWindow::SetSeedNeighbourhoodRadius(const QString &new_radius)
{ if (strlen(new_radius.toLatin1().data()) == 0) return;
  seed_radius = new_radius.toDouble(); }

void SegmentationParametersWindow::SetMaxSlopeAngle(const QString &new_max_slope_angle)
{ double degree = atan(1.0) / 45.0;
  if (strlen(new_max_slope_angle.toLatin1().data()) == 0) return;
  max_slope_angle = new_max_slope_angle.toDouble() * degree;
}

void SegmentationParametersWindow::SetBinSizeSlopeAngle(const QString &new_bin_size)
{ double degree = atan(1.0) / 45.0;
  if (strlen(new_bin_size.toLatin1().data()) == 0) return;
  bin_size_slope_angle = new_bin_size.toDouble() * degree;
}

void SegmentationParametersWindow::SetBinSizeDistance(const QString &new_bin_size)
{ if (strlen(new_bin_size.toLatin1().data()) == 0) return;
  bin_size_distance = new_bin_size.toDouble(); }

void SegmentationParametersWindow::SetMinNumberOfPointsSeed(int new_min_numpts)
{ min_numpts_seed = new_min_numpts; }

void SegmentationParametersWindow::SetMaxDistanceSeedPlane(const QString &new_max_distance)
{ if (strlen(new_max_distance.toLatin1().data()) == 0) return;
  max_dist_seed_plane = new_max_distance.toDouble(); }

void SegmentationParametersWindow::SetMaxReflectanceDifferenceSeed(const QString &new_max_dif)
{ if (strlen(new_max_dif.toLatin1().data()) == 0) return;
  max_refl_dif_seed = new_max_dif.toDouble(); }

void SegmentationParametersWindow::SetSurfaceModel(int new_model)
{ surface_model = new_model; }

void SegmentationParametersWindow::SetGrowingNeighbourhoodDefinition(int new_definition)
{ growing_neighbourhood = new_definition; }

void SegmentationParametersWindow::SetGrowingRadius(const QString &new_radius)
{ if (strlen(new_radius.toLatin1().data()) == 0) return;
  growing_radius = new_radius.toDouble(); }

void SegmentationParametersWindow::SetMaxDistanceSurface(const QString &new_distance)
{ if (strlen(new_distance.toLatin1().data()) == 0) return;
  max_dist_surface = new_distance.toDouble(); }

void SegmentationParametersWindow::SetMinDistanceRecompute(const QString &new_distance)
{ if (strlen(new_distance.toLatin1().data()) == 0) return;
  min_dist_recompute = new_distance.toDouble(); }

void SegmentationParametersWindow::SetMaxReflectanceDifferenceGrowing(const QString &new_max_dif)
{ if (strlen(new_max_dif.toLatin1().data()) == 0) return;
  max_refl_dif_growing = new_max_dif.toDouble(); }

void SegmentationParametersWindow::SetSurfacesCompete(bool new_switch)
{ competing_surfaces = new_switch; }







GeneralSegmentationParametersWindow::GeneralSegmentationParametersWindow(
  SegmentationParameters *par, QWidget *parent) : QWidget(parent)
{
  QDoubleValidator *positive_double;
  int              y;

  // Copy pointer to parameters
  parameters = par;
  
  // Initialise validator
  positive_double = new QDoubleValidator(0.01, 1.0e10, 2, this);

  // Set window size
  setFixedSize(360, 320);

// Neighbourhood definition parameters

  // Neighbourhood storage model
  nbh_model_selector = new QComboBox(this);
  nbh_model_selector->setEditable(false);
  nbh_model_selector->setFocusPolicy(Qt::NoFocus);
  nbh_model_selector->addItem(QString("Delaunay TIN"));
  nbh_model_selector->addItem(QString("Octree"));
  nbh_model_selector->addItem(QString("Kd-tree"));
  nbh_model_selector->setCurrentIndex(parameters->NeighbourhoodStorageModel());
  y = 25; nbh_model_selector->setGeometry(225, y, 90, 18);
  connect(nbh_model_selector, SIGNAL(activated(int)),
          this, SLOT(SetNeighbourhoodStorageModel(int)));

  // Octree bin maximum number of points
  octree_bin_size_selector = new QSpinBox(this);
  octree_bin_size_selector->setMinimum(1);
  octree_bin_size_selector->setMaximum(9999);
  octree_bin_size_selector->setSingleStep(10);
  ;octree_bin_size_selector->setValue(parameters->OctreeBinMaxNumberOfPoints());
  y += 20; octree_bin_size_selector->setGeometry(225, y, 50, 18);
  connect(octree_bin_size_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetOctreeBinMaxNumberOfPoints(int)));

  // Octree bin overlap
  octree_overlap_line = new QLineEdit(QString("%1").arg(parameters->OctreeBinOverlap(), 0, 'f', 1),
                            this);
  octree_overlap_line->setValidator(positive_double);
  y += 20; octree_overlap_line->setGeometry(225, y, 40, 18);
  connect(octree_overlap_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetOctreeBinOverlap(const QString &)));

  // Distance metric dimension
  dim_selector = new QComboBox(this);
  dim_selector->setEditable(false);
  dim_selector->setFocusPolicy(Qt::NoFocus);
  dim_selector->addItem(QString("2D"));
  dim_selector->addItem(QString("3D"));
  dim_selector->setCurrentIndex(parameters->DistanceMetricDimension()-2);
  y += 20; dim_selector->setGeometry(225, y, 40, 18);
  connect(dim_selector, SIGNAL(activated(int)),
          this, SLOT(SetDistanceMetricDimension(int)));

  // Number of neighbours in kd-tree
  knn_selector = new QSpinBox(this);
  knn_selector->setMinimum(3);
  knn_selector->setMaximum(100);
  knn_selector->setSingleStep(1);
  knn_selector->setValue(parameters->NumberOfNeighbours());
  y += 20; knn_selector->setGeometry(225, y, 50, 18);
  connect(knn_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetNumberOfNeighbours(int)));

// Segment size

  // Minimum number of points in a segment
  segment_size_selector = new QSpinBox(this);
  segment_size_selector->setMinimum(1);
  segment_size_selector->setMaximum(999);
  segment_size_selector->setSingleStep(1);
  segment_size_selector->setValue(parameters->MinNumberOfPointsSegment());
  y += 60; segment_size_selector->setGeometry(225, y, 40, 18);
  connect(segment_size_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetMinNumberOfPointsSegment(int)));


// Storage of result

  // Attribute for storing segmentation result
  segment_attribute_selector = new QComboBox(this);
  segment_attribute_selector->setEditable(false);
  segment_attribute_selector->setFocusPolicy(Qt::NoFocus);
  segment_attribute_selector->addItem(QString("Label"));
  segment_attribute_selector->addItem(QString("Segment number"));
  segment_attribute_selector->addItem(QString("Component number"));
  segment_attribute_selector->setCurrentIndex(SegmentAttributeIndex());
  y += 60; segment_attribute_selector->setGeometry(225, y, 110, 18);
  connect(segment_attribute_selector, SIGNAL(activated(int)),
          this, SLOT(SetSegmentAttribute(int)));
          
          
// Segment all points or only unsegmented points?

  // Switch for erasing old segment numbers
  erase_old_number_selector = new QCheckBox(this);
  erase_old_number_selector->setChecked(parameters->EraseOldNumbers());
  y += 60; erase_old_number_selector->setGeometry(225, y+2, 40, 18);
  connect(erase_old_number_selector, SIGNAL(toggled(bool)),
          this, SLOT(SetEraseOldNumbers(bool)));
}

void GeneralSegmentationParametersWindow::Update()
{
  nbh_model_selector->setCurrentIndex(parameters->NeighbourhoodStorageModel());
  octree_bin_size_selector->setValue(parameters->OctreeBinMaxNumberOfPoints());
  octree_overlap_line->setText(QString("%1").arg(parameters->OctreeBinOverlap(), 0, 'f', 1));
  dim_selector->setCurrentIndex(parameters->DistanceMetricDimension()-2);
  knn_selector->setValue(parameters->NumberOfNeighbours());
  segment_size_selector->setValue(parameters->MinNumberOfPointsSegment());
  segment_attribute_selector->setCurrentIndex(SegmentAttributeIndex());
  erase_old_number_selector->setChecked(parameters->EraseOldNumbers());
}

void GeneralSegmentationParametersWindow::paintEvent(QPaintEvent *event)
{
  QPainter paint(this);
  int      y;

  if (event) y = 20; // To avoid compiler warnings
  // Neighbourhood definitions
  y = 20; paint.drawText(12, y, QString("Neighbourhood definitions"));
  y += 20; paint.drawText(32, y, QString("Storage model:"));
  y += 20; paint.drawText(32, y, QString("Octree bin maximum number of points:"));
  y += 20; paint.drawText(32, y, QString("Octree bin overlap:"));
  y += 20; paint.drawText(32, y, QString("Distance metric:"));
  y += 20; paint.drawText(32, y, QString("Number of neighbours in kd-tree:"));
  // Segment size
  y += 40; paint.drawText(12, y, QString("Segment size"));
  y += 20; paint.drawText(32, y, QString("Minimum number of points:"));
  // Storage of result
  y += 40; paint.drawText(12, y, QString("Storage of segmentation result"));
  y += 20; paint.drawText(32, y, QString("Attribute for segment numbers:"));
  // Preservation of previous segmentation
  y += 40; paint.drawText(12, y, QString("Preservation of previous segmentation"));
  y += 20; paint.drawText(32, y, QString("Re-segment all points:"));
}

void GeneralSegmentationParametersWindow::SetNeighbourhoodStorageModel(int new_nbh_model)
{ parameters->NeighbourhoodStorageModel() = new_nbh_model; }

void GeneralSegmentationParametersWindow::SetOctreeBinMaxNumberOfPoints(int new_number)
{ parameters->OctreeBinMaxNumberOfPoints() = new_number; }

void GeneralSegmentationParametersWindow::SetOctreeBinOverlap(const QString &new_octree_bin_overlap)
{ if (strlen(new_octree_bin_overlap.toLatin1().data()) == 0) return;
  parameters->OctreeBinOverlap() = new_octree_bin_overlap.toDouble(); }

void GeneralSegmentationParametersWindow::SetDistanceMetricDimension(int new_dist_dimension)
{ parameters->DistanceMetricDimension() = new_dist_dimension + 2; }

void GeneralSegmentationParametersWindow::SetMinNumberOfPointsSegment(int new_min_numpts_segment)
{ parameters->MinNumberOfPointsSegment() = new_min_numpts_segment; }

void GeneralSegmentationParametersWindow::SetNumberOfNeighbours(int new_number)
{ parameters->NumberOfNeighbours() = new_number; }

void GeneralSegmentationParametersWindow::SetSegmentAttribute(int new_attribute)
{ 
  switch (new_attribute) {
    case 0: parameters->SegmentAttribute() = LabelTag; break;
    default:
    case 1: parameters->SegmentAttribute() = SegmentNumberTag; break;
    case 2: parameters->SegmentAttribute() = ComponentNumberTag; break;
  }
}

int GeneralSegmentationParametersWindow::SegmentAttributeIndex() const
{
  switch ((int) parameters->SegmentAttribute()) {
    case LabelTag:           return 0;
    case SegmentNumberTag:   return 1;
    case ComponentNumberTag: return 2;
    default: return -1;
  }
  return -1;
}

void GeneralSegmentationParametersWindow::SetEraseOldNumbers(bool new_switch)
{ parameters->EraseOldNumbers() = new_switch; }



ConnectedComponentParametersWindow::ConnectedComponentParametersWindow(
  SegmentationParameters *par, QWidget *parent) : QWidget(parent)
{
  QDoubleValidator *positive_double;
  int              y;

  // Copy pointer to parameters
  parameters = par;

  // Initialise validator
  positive_double = new QDoubleValidator(0.01, 1.0e10, 2, this);

  setFixedSize(360, 60);

  // Maximum distance between two points in a component
  dist_component_line = new QLineEdit(QString("%1").arg(parameters->MaxDistanceInComponent(), 0, 'f', 1), this);
  dist_component_line->setValidator(positive_double);
  y = 25; dist_component_line->setGeometry(205, y, 40, 18);
  connect(dist_component_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxDistanceInComponent(const QString &)));
}

void ConnectedComponentParametersWindow::Update()
{
  dist_component_line->setText(QString("%1").arg(parameters->MaxDistanceInComponent(), 0, 'f', 1));
}

void ConnectedComponentParametersWindow::paintEvent(QPaintEvent *event)
{
  QPainter paint(this);
  int      y;

  if (event) y = 20; // To avoid compiler warnings
  // Connected component parameter
  y = 20; paint.drawText(12, y, QString("Connection criterion"));
  y += 20; paint.drawText(32, y, QString("Maximum distance between points:"));
}

void ConnectedComponentParametersWindow::SetMaxDistanceInComponent(const QString &new_max_dist_connected)
{ if (strlen(new_max_dist_connected.toLatin1().data()) == 0) return;
  parameters->MaxDistanceInComponent() = new_max_dist_connected.toDouble(); }






SurfaceGrowingParametersWindow::SurfaceGrowingParametersWindow(
  SegmentationParameters *par, QWidget *parent) : QWidget(parent)
{
  QDoubleValidator *positive_double, *non_negative_double, *any_double;
  int              y;
  double           degree = atan(1.0) / 45.0;

  // Copy pointer to parameters
  parameters = par;

  // Initialise validators
  positive_double = new QDoubleValidator(0.01, 1.0e10, 2, this);
  non_negative_double = new QDoubleValidator(0.00, 1.0e10, 2, this);
  any_double = new QDoubleValidator(this);

  setFixedSize(360, 380);

  // Seed neighbourhood definition
  seed_nbh_selector = new QComboBox(this);
  seed_nbh_selector->setEditable(false);
  seed_nbh_selector->setFocusPolicy(Qt::NoFocus);
  seed_nbh_selector->addItem(QString("Direct neighbours"));
  seed_nbh_selector->addItem(QString("All within radius"));
  seed_nbh_selector->setCurrentIndex(parameters->SeedNeighbourhoodDefinition());
  y = 25; seed_nbh_selector->setGeometry(205, y, 110, 18);
  connect(seed_nbh_selector, SIGNAL(activated(int)),
          this, SLOT(SetSeedNeighbourhoodDefinition(int)));
          
  // Seed neighbourhood radius
  seed_radius_line = new QLineEdit(QString("%1").arg(parameters->SeedNeighbourhoodRadius(), 0, 'f', 1), this);
  seed_radius_line->setValidator(non_negative_double);
  y += 20; seed_radius_line->setGeometry(205, y, 40, 18);
  connect(seed_radius_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetSeedNeighbourhoodRadius(const QString &)));

  // Maximum slope angle of the Hough space
  max_slope_line = new QLineEdit(QString("%1").arg(parameters->MaxSlopeAngle()/degree, 0, 'f', 1), this);
  max_slope_line->setValidator(positive_double);
  y += 20; max_slope_line->setGeometry(205, y, 40, 18);
  connect(max_slope_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxSlopeAngle(const QString &)));
  
  // Bin size of the slope angle in the Hough space
  angle_bin_size_line = new QLineEdit(QString("%1").arg(parameters->BinSizeSlopeAngle()/degree, 0, 'f', 1), this);
  angle_bin_size_line->setValidator(positive_double);
  y += 20; angle_bin_size_line->setGeometry(205, y, 40, 18);
  connect(angle_bin_size_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetBinSizeSlopeAngle(const QString &)));
  
  // Bin size of the distance in the Hough space
  dist_bin_size_line = new QLineEdit(QString("%1").arg(parameters->BinSizeDistance(), 0, 'f', 1), this);
  dist_bin_size_line->setValidator(positive_double);
  y += 20; dist_bin_size_line->setGeometry(205, y, 40, 18);
  connect(dist_bin_size_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetBinSizeDistance(const QString &)));
  
  // Minimum number of points in a seed
  seed_size_selector = new QSpinBox(this);
  seed_size_selector->setMinimum(3);
  seed_size_selector->setMaximum(999);
  seed_size_selector->setSingleStep(1);
  seed_size_selector->setValue(parameters->MinNumberOfPointsSeed());
  y += 20; seed_size_selector->setGeometry(205, y, 40, 18);
  connect(seed_size_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetMinNumberOfPointsSeed(int)));

  // Maximum distance between a point and the seed plane
  dist_seed_line = new QLineEdit(QString("%1").arg(parameters->MaxDistanceSeedPlane(), 0, 'f', 2), this);
  dist_seed_line->setValidator(positive_double);
  y += 20; dist_seed_line->setGeometry(205, y, 40, 18);
  connect(dist_seed_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxDistanceSeedPlane(const QString &)));
  
  // Maximum reflectance difference between a point and the seed plane
  refl_dif_seed_line = new QLineEdit(QString("%1").arg(parameters->MaxReflectanceDifferenceSeed(), 0, 'f', 0), this);
  refl_dif_seed_line->setValidator(any_double);
  y += 20; refl_dif_seed_line->setGeometry(205, y, 40, 18);
  connect(refl_dif_seed_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxReflectanceDifferenceSeed(const QString &)));
  
  // Surface model
  surface_model_selector = new QComboBox(this);
  surface_model_selector->setEditable(false);
  surface_model_selector->setFocusPolicy(Qt::NoFocus);
  surface_model_selector->addItem(QString("Planar"));
  surface_model_selector->addItem(QString("Smooth"));
  surface_model_selector->setCurrentIndex(parameters->SurfaceModel());
  y += 60; surface_model_selector->setGeometry(240, y, 60, 18);
  connect(surface_model_selector, SIGNAL(activated(int)),
          this, SLOT(SetSurfaceModel(int)));
          
  // Growing neighbourhood definition
  growing_nbh_selector = new QComboBox(this);
  growing_nbh_selector->setEditable(false);
  growing_nbh_selector->setFocusPolicy(Qt::NoFocus);
  growing_nbh_selector->addItem(QString("Direct neighbours"));
  growing_nbh_selector->addItem(QString("All within radius"));
  growing_nbh_selector->setCurrentIndex(parameters->GrowingNeighbourhoodDefinition());
  y += 20; growing_nbh_selector->setGeometry(240, y, 110, 18);
  connect(growing_nbh_selector, SIGNAL(activated(int)),
          this, SLOT(SetGrowingNeighbourhoodDefinition(int)));
            
  // Radius for surface growing
  growing_radius_line = new QLineEdit(QString("%1").arg(parameters->GrowingRadius(), 0, 'f', 1), this);
  growing_radius_line->setValidator(non_negative_double);
  y += 20; growing_radius_line->setGeometry(240, y, 40, 18);
  connect(growing_radius_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetGrowingRadius(const QString &)));

  // Maximum distance between a point and the growing surface
  dist_growing_line = new QLineEdit(QString("%1").arg(parameters->MaxDistanceSurface(), 0, 'f', 2), this);
  dist_growing_line->setValidator(positive_double);
  y += 20; dist_growing_line->setGeometry(240, y, 40, 18);
  connect(dist_growing_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxDistanceSurface(const QString &)));
  
  // Minimum distance between a point and the surface to trigger recomputation
  // of the local plane
  dist_recompute_line = new QLineEdit(QString("%1").arg(parameters->MinDistanceRecompute(), 0, 'f', 2), this);
  dist_recompute_line->setValidator(positive_double);
  y += 20; dist_recompute_line->setGeometry(240, y, 40, 18);
  connect(dist_recompute_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMinDistanceRecompute(const QString &)));
  
  // Maximum reflectance difference between a point and the growing plane
  refl_dif_growing_line = new QLineEdit(QString("%1").arg(parameters->MaxReflectanceDifferenceGrowing(), 0, 'f', 0), this);
  refl_dif_growing_line->setValidator(any_double);
  y += 20; refl_dif_growing_line->setGeometry(240, y, 40, 18);
  connect(refl_dif_growing_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxReflectanceDifferenceGrowing(const QString &)));
  
  // Switch for letting surfaces compete
  compete_selector = new QCheckBox(this);
  compete_selector->setChecked(parameters->SurfacesCompete());
  y += 20; compete_selector->setGeometry(240, y+2, 40, 18);
  connect(compete_selector, SIGNAL(toggled(bool)),
          this, SLOT(SetSurfacesCompete(bool)));
}

void SurfaceGrowingParametersWindow::Update()
{
  double           degree = atan(1.0) / 45.0;

  seed_nbh_selector->setCurrentIndex(parameters->SeedNeighbourhoodDefinition());
  seed_radius_line->setText(QString("%1").arg(parameters->SeedNeighbourhoodRadius(), 0, 'f', 1));
  max_slope_line->setText(QString("%1").arg(parameters->MaxSlopeAngle()/degree, 0, 'f', 1));
  angle_bin_size_line->setText(QString("%1").arg(parameters->BinSizeSlopeAngle()/degree, 0, 'f', 1));
  dist_bin_size_line->setText(QString("%1").arg(parameters->BinSizeDistance(), 0, 'f', 1));
  seed_size_selector->setValue(parameters->MinNumberOfPointsSeed());
  dist_seed_line->setText(QString("%1").arg(parameters->MaxDistanceSeedPlane(), 0, 'f', 2));
  refl_dif_seed_line->setText(QString("%1").arg(parameters->MaxReflectanceDifferenceSeed(), 0, 'f', 0));
  surface_model_selector->setCurrentIndex(parameters->SurfaceModel());
  growing_nbh_selector->setCurrentIndex(parameters->GrowingNeighbourhoodDefinition());
  growing_radius_line->setText(QString("%1").arg(parameters->GrowingRadius(), 0, 'f', 1));
  dist_growing_line->setText(QString("%1").arg(parameters->MaxDistanceSurface(), 0, 'f', 2));
  dist_recompute_line->setText(QString("%1").arg(parameters->MinDistanceRecompute(), 0, 'f', 2));
  refl_dif_growing_line->setText(QString("%1").arg(parameters->MaxReflectanceDifferenceGrowing(), 0, 'f', 0));
  compete_selector->setChecked(parameters->SurfacesCompete());
}

void SurfaceGrowingParametersWindow::paintEvent(QPaintEvent *event)
{
  QPainter paint(this);
  int      y;

  if (event) y = 20; // To avoid compiler warnings
  // Seed selection parameters
  y = 20; paint.drawText(12, y, QString("Seed selection parameters"));
  y += 20; paint.drawText(32, y, QString("Seed neighbourhood definition:"));
  y += 20; paint.drawText(32, y, QString("Seed neighbourhood radius:"));
  y += 20; paint.drawText(32, y, QString("Maximum slope angle:"));
  y += 20; paint.drawText(32, y, QString("Bin size slope angle:"));
  y += 20; paint.drawText(32, y, QString("Bin size distance:"));
  y += 20; paint.drawText(32, y, QString("Minimum number of seed points:"));
  y += 20; paint.drawText(32, y, QString("Maximum distance to plane:")); 
  y += 20; paint.drawText(32, y, QString("Maximum reflectance difference:"));    
  // Surface growing parameters
  y += 40; paint.drawText(12, y, QString("Surface growing parameters"));  
  y += 20; paint.drawText(32, y, QString("Surface model:"));
  y += 20; paint.drawText(32, y, QString("Surface growing neighbourhood definition:"));
  y += 20; paint.drawText(32, y, QString("Surface growing radius:"));
  y += 20; paint.drawText(32, y, QString("Maximum distance to surface:"));
  y += 20; paint.drawText(32, y, QString("Minimum distance to recompute local plane:"));
  y += 20; paint.drawText(32, y, QString("Maximum reflectance difference:"));    
  y += 20; paint.drawText(32, y, QString("Competing surfaces:"));
}

void SurfaceGrowingParametersWindow::SetSeedNeighbourhoodDefinition(int new_definition)
{ parameters->SeedNeighbourhoodDefinition() = new_definition; }

void SurfaceGrowingParametersWindow::SetSeedNeighbourhoodRadius(const QString &new_radius)
{ if (strlen(new_radius.toLatin1().data()) == 0) return;
  parameters->SeedNeighbourhoodRadius() = new_radius.toDouble(); }

void SurfaceGrowingParametersWindow::SetMaxSlopeAngle(const QString &new_max_slope_angle)
{ double degree = atan(1.0) / 45.0;
  if (strlen(new_max_slope_angle.toLatin1().data()) == 0) return;
  parameters->MaxSlopeAngle() = new_max_slope_angle.toDouble() * degree;
}

void SurfaceGrowingParametersWindow::SetBinSizeSlopeAngle(const QString &new_bin_size)
{ double degree = atan(1.0) / 45.0;
  if (strlen(new_bin_size.toLatin1().data()) == 0) return;
  parameters->BinSizeSlopeAngle() = new_bin_size.toDouble() * degree;
}

void SurfaceGrowingParametersWindow::SetBinSizeDistance(const QString &new_bin_size)
{ if (strlen(new_bin_size.toLatin1().data()) == 0) return;
  parameters->BinSizeDistance() = new_bin_size.toDouble(); }

void SurfaceGrowingParametersWindow::SetMinNumberOfPointsSeed(int new_min_numpts)
{ parameters->MinNumberOfPointsSeed() = new_min_numpts; }

void SurfaceGrowingParametersWindow::SetMaxDistanceSeedPlane(const QString &new_max_distance)
{ if (strlen(new_max_distance.toLatin1().data()) == 0) return;
  parameters->MaxDistanceSeedPlane() = new_max_distance.toDouble(); }

void SurfaceGrowingParametersWindow::SetMaxReflectanceDifferenceSeed(const QString &new_max_dif)
{ if (strlen(new_max_dif.toLatin1().data()) == 0) return;
  parameters->MaxReflectanceDifferenceSeed() = new_max_dif.toDouble(); }

void SurfaceGrowingParametersWindow::SetSurfaceModel(int new_model)
{ parameters->SurfaceModel() = new_model; }

void SurfaceGrowingParametersWindow::SetGrowingNeighbourhoodDefinition(int new_definition)
{ parameters->GrowingNeighbourhoodDefinition() = new_definition; }

void SurfaceGrowingParametersWindow::SetGrowingRadius(const QString &new_radius)
{ if (strlen(new_radius.toLatin1().data()) == 0) return;
  parameters->GrowingRadius() = new_radius.toDouble(); }

void SurfaceGrowingParametersWindow::SetMaxDistanceSurface(const QString &new_distance)
{ if (strlen(new_distance.toLatin1().data()) == 0) return;
  parameters->MaxDistanceSurface() = new_distance.toDouble(); }

void SurfaceGrowingParametersWindow::SetMinDistanceRecompute(const QString &new_distance)
{ if (strlen(new_distance.toLatin1().data()) == 0) return;
  parameters->MinDistanceRecompute() = new_distance.toDouble(); }

void SurfaceGrowingParametersWindow::SetMaxReflectanceDifferenceGrowing(const QString &new_max_dif)
{ if (strlen(new_max_dif.toLatin1().data()) == 0) return;
  parameters->MaxReflectanceDifferenceGrowing() = new_max_dif.toDouble(); }

void SurfaceGrowingParametersWindow::SetSurfacesCompete(bool new_switch)
{ parameters->SurfacesCompete() = new_switch; }







SegmentGrowingParametersWindow::SegmentGrowingParametersWindow(
  SegmentationParameters *par, QWidget *parent) : QWidget(parent)
{
  QDoubleValidator *non_negative_double;
  int              y, i;

  // Copy pointer to parameters
  parameters = par;

  // Initialise validator
  non_negative_double = new QDoubleValidator(0.00, 1.0e10, 2, this);

  setFixedSize(360, 120);

  y = 10;
  for (i=0; i<3; i++) {
    // Growing attribute selector
    growing_attribute_selector[i] = new QComboBox(this);
    growing_attribute_selector[i]->setEditable(false);
    growing_attribute_selector[i]->setFocusPolicy(Qt::NoFocus);
    growing_attribute_selector[i]->addItem("-- none --");
    growing_attribute_selector[i]->addItem("Reflectance");
	growing_attribute_selector[i]->addItem("Pulse length");
    growing_attribute_selector[i]->addItem("Label");
	growing_attribute_selector[i]->addItem("Scaled X-normal");
	growing_attribute_selector[i]->addItem("Scaled Y-normal");
	growing_attribute_selector[i]->addItem("Scaled Z-normal");
	growing_attribute_selector[i]->addItem("Red");
	growing_attribute_selector[i]->addItem("Green");
	growing_attribute_selector[i]->addItem("Blue");
    y += 20; growing_attribute_selector[i]->setGeometry(12, y, 110, 18);
    connect(growing_attribute_selector[i], SIGNAL(activated(int)),
            this, SLOT(SetGrowingTolerances()));
    
    // Growing tolerance
    growing_tolerance_line[i] = new QLineEdit(this);
    growing_tolerance_line[i]->setValidator(non_negative_double);
    growing_tolerance_line[i]->setGeometry(140, y, 40, 18);
    connect(growing_tolerance_line[i], SIGNAL(textChanged(const QString &)),
            this, SLOT(SetGrowingTolerances()));
  }
  
  // Set all variables in the comboboxes and edit lines
  Update();
}

void SegmentGrowingParametersWindow::Update()
{
  const unsigned char *tolerance_tags, *tag;
  int                 num_tags, itag, attribute_index, i;

  // Temporarily disconnect data entry widgets. This ensures that the
  // setCurrentIndex and setText calls don't trigger an interfering call
  // to SetGrowingTolerances.
  for (i=0; i<3; i++) {
  	disconnect(growing_attribute_selector[i], SIGNAL(activated(int)),
               this, SLOT(SetGrowingTolerances()));
    disconnect(growing_tolerance_line[i], SIGNAL(textChanged(const QString &)),
               this, SLOT(SetGrowingTolerances()));
  }
  
  num_tags = parameters->GrowingTolerances().NumAttributes();
  tolerance_tags = parameters->GrowingTolerances().AttributeTags();
  for (itag=0, tag=tolerance_tags; itag<3; itag++, tag++) {

  	if (itag < num_tags) {
  	  if (*tag == DoubleTag) {itag--; continue;}
  	  attribute_index = AttributeIndex((LaserPointTag) *tag);
      growing_attribute_selector[itag]->setCurrentIndex(attribute_index);
  	  switch (AttributeType((LaserPointTag) *tag)) {
  	  	default:
  	  	case IntegerAttributeType:
  	  	  growing_tolerance_line[itag]->setText(QString("%1").arg((float) parameters->GrowingTolerances().Attribute(*tag), 0, 'f', 1));
  	  	  break;
  	  	case FloatAttributeType:
  	  	  growing_tolerance_line[itag]->setText(QString("%1").arg(parameters->GrowingTolerances().FloatAttribute(*tag), 0, 'f', 2));
  	  	  break;
  	  	case DoubleAttributeType:
  	  	  growing_tolerance_line[itag]->setText(QString("%1").arg(parameters->GrowingTolerances().DoubleAttribute(*tag), 0, 'f', 2));
  	  }
  	}
    else {
      growing_attribute_selector[itag]->setCurrentIndex(0);    	
      growing_tolerance_line[itag]->setText(QString("%1").arg(1.0, 0, 'f', 2));
    }
  }

  // Reconnect data entry widgets. 
  for (i=0; i<3; i++) {
  	connect(growing_attribute_selector[i], SIGNAL(activated(int)),
            this, SLOT(SetGrowingTolerances()));
    connect(growing_tolerance_line[i], SIGNAL(textChanged(const QString &)),
            this, SLOT(SetGrowingTolerances()));
  }
}

void SegmentGrowingParametersWindow::paintEvent(QPaintEvent *event)
{
  QPainter paint(this);
  int      y=20;

  if (event) y = 20; // To avoid compiler warnings
  // Segment growing tolerances
  paint.drawText(12, y, QString("Attribute"));
  paint.drawText(140, y, QString("Tolerance"));
}

void SegmentGrowingParametersWindow::SetGrowingTolerances()
{
  LaserPoint    new_tolerances;
  int           selector, index;
  double        value;
  LaserPointTag tag;
  
  for (selector=0; selector<3; selector++) {
  	index = growing_attribute_selector[selector]->currentIndex();
  	if (index > 0) {
  	  tag = AttributeTag(index);
      value = growing_tolerance_line[selector]->text().toDouble();
      switch (AttributeType(tag)) {
      	default:
      	case IntegerAttributeType:
      	  new_tolerances.Attribute(tag) = (int) value; break;
      	case FloatAttributeType:
      	  new_tolerances.FloatAttribute(tag) = (float) value; break;
      	case DoubleAttributeType:
      	  new_tolerances.DoubleAttribute(tag) = value; break;
      }
    }
  }
  parameters->GrowingTolerances() = new_tolerances;
}

int SegmentGrowingParametersWindow::AttributeIndex(LaserPointTag tag)
{
  switch (tag) {
  	default:
  	case NoTag:              return 0;
  	case ReflectanceTag:     return 1;
  	case PulseLengthTag:     return 2;
  	case LabelTag:           return 3;
  	case ScaledNormalXTag:   return 4;
  	case ScaledNormalYTag:   return 5;
  	case ScaledNormalZTag:   return 6;
  	case RedTag:             return 7;
  	case GreenTag:           return 8;
  	case BlueTag:            return 9;
  }
  return 0;
}

LaserPointTag SegmentGrowingParametersWindow::AttributeTag(int index)
{
  switch (index) {
  	default:
  	case 0: return NoTag;
  	case 1: return ReflectanceTag;
  	case 2: return PulseLengthTag;
  	case 3: return LabelTag;
  	case 4: return ScaledNormalXTag;
  	case 5: return ScaledNormalYTag;
  	case 6: return ScaledNormalZTag;
  	case 7: return RedTag;
  	case 8: return GreenTag;
  	case 9: return BlueTag;
  }
  return NoTag;
}




MeanShiftParametersWindow::MeanShiftParametersWindow(
  SegmentationParameters *par, QWidget *parent) : QWidget(parent)
{
  QDoubleValidator *non_negative_double;
  int              y, i;

  // Copy pointer to parameters
  parameters = par;

  // Initialise validator
  non_negative_double = new QDoubleValidator(0.00, 1.0e10, 2, this);

  setFixedSize(360, 200);

  y = 10;
  for (i=0; i<6; i++) {
  	y += 20; 
  	
    // Mean shift attribute selector, not for first three (coordinates)
    if (i > 2) {
      ms_attribute_selector[i-3] = new QComboBox(this);
      ms_attribute_selector[i-3]->setEditable(false);
      ms_attribute_selector[i-3]->setFocusPolicy(Qt::NoFocus);
      ms_attribute_selector[i-3]->addItem("-- none --");
      ms_attribute_selector[i-3]->addItem("Reflectance");
	  ms_attribute_selector[i-3]->addItem("Pulse length");
      ms_attribute_selector[i-3]->addItem("Label");
	  ms_attribute_selector[i-3]->addItem("Scaled X-normal");
	  ms_attribute_selector[i-3]->addItem("Scaled Y-normal");
	  ms_attribute_selector[i-3]->addItem("Scaled Z-normal");
      ms_attribute_selector[i-3]->setGeometry(12, y, 110, 18);
      connect(ms_attribute_selector[i-3], SIGNAL(activated(int)),
              this, SLOT(SetBandWidths()));
    }
    
    // Attribute band width
    ms_bandwidth_line[i] = new QLineEdit(this);
    ms_bandwidth_line[i]->setValidator(non_negative_double);
    ms_bandwidth_line[i]->setGeometry(140, y, 40, 18);
    connect(ms_bandwidth_line[i], SIGNAL(textChanged(const QString &)),
            this, SLOT(SetBandWidths()));
  }
  
  // Maximum number of iterations for the mode seeking
  ms_max_iter_selector = new QSpinBox(this);
  ms_max_iter_selector->setMinimum(1);
  ms_max_iter_selector->setMaximum(1000);
  ms_max_iter_selector->setSingleStep(1);
  ms_max_iter_selector->setValue(parameters->MaxNumIterationsModeSeeking());
  y += 20; ms_max_iter_selector->setGeometry(170, y, 50, 18);
  connect(ms_max_iter_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetMaxNumIterationsModeSeeking(int)));
  // Set all variables in the comboboxes and edit lines
  Update();
}

void MeanShiftParametersWindow::Update()
{
  const unsigned char *bandwidth_tags, *tag;
  int                 num_tags, itag, attribute_index, i;

  // Temporarily disconnect data entry widgets. This ensures that the
  // setCurrentIndex and setText calls don't trigger an interfering call
  // to SetBandwidths.
  for (i=0; i<6; i++) {
  	if (i < 3)
  	  disconnect(ms_attribute_selector[i], SIGNAL(activated(int)),
                 this, SLOT(SetBandWidths()));
    disconnect(ms_bandwidth_line[i], SIGNAL(textChanged(const QString &)),
               this, SLOT(SetBandWidths()));
  }
  
  num_tags = parameters->AttributeBandWidths().NumAttributes();
  bandwidth_tags = parameters->AttributeBandWidths().AttributeTags();
  for (itag=0, tag=bandwidth_tags; itag<3; itag++, tag++) {
  	if (itag < num_tags) {
  	  if (*tag == DoubleTag) {itag--; continue;}
  	  attribute_index = AttributeIndex((LaserPointTag) *tag);
      ms_attribute_selector[itag]->setCurrentIndex(attribute_index);
  	  switch (AttributeType((LaserPointTag) *tag)) {
  	  	default:
  	  	case IntegerAttributeType:
  	  	  ms_bandwidth_line[itag+3]->setText(QString("%1").arg((float) parameters->AttributeBandWidths().Attribute(*tag), 0, 'f', 1));
  	  	  break;
  	  	case FloatAttributeType:
  	  	  ms_bandwidth_line[itag+3]->setText(QString("%1").arg(parameters->AttributeBandWidths().FloatAttribute(*tag), 0, 'f', 2));
  	  	  break;
  	  	case DoubleAttributeType:
  	  	  ms_bandwidth_line[itag+3]->setText(QString("%1").arg(parameters->AttributeBandWidths().DoubleAttribute(*tag), 0, 'f', 2));
  	  }
  	}
    else {
      ms_attribute_selector[itag]->setCurrentIndex(0);    	
      ms_bandwidth_line[itag+3]->setText(QString("%1").arg(1.0, 0, 'f', 2));
    }
  }
  ms_bandwidth_line[0]->setText(QString("%1").arg(parameters->AttributeBandWidths().X(), 0, 'f', 2));
  ms_bandwidth_line[1]->setText(QString("%1").arg(parameters->AttributeBandWidths().Y(), 0, 'f', 2));
  ms_bandwidth_line[2]->setText(QString("%1").arg(parameters->AttributeBandWidths().Z(), 0, 'f', 2));
  
  // Reconnect data entry widgets. 
  for (i=0; i<6; i++) {
  	if (i < 3)
  	  connect(ms_attribute_selector[i], SIGNAL(activated(int)),
              this, SLOT(SetBandWidths()));
    connect(ms_bandwidth_line[i], SIGNAL(textChanged(const QString &)),
            this, SLOT(SetBandWidths()));
  }
  ms_max_iter_selector->setValue(parameters->MaxNumIterationsModeSeeking());
}

void MeanShiftParametersWindow::paintEvent(QPaintEvent *event)
{
  QPainter paint(this);
  int      y=20;

  if (event) y = 20; // To avoid compiler warnings
  // Segment growing tolerances
  paint.drawText(12, y, QString("Attribute"));
  paint.drawText(140, y, QString("Bandwidth"));
  y += 25; paint.drawText(12, y, QString("X-coordinate"));
  y += 20; paint.drawText(12, y, QString("Y-coordinate"));
  y += 20; paint.drawText(12, y, QString("Z-coordinate"));
  y += 80; paint.drawText(12, y, QString("Maximum number of iterations"));
}

void MeanShiftParametersWindow::SetBandWidths()
{
  LaserPoint    new_bandwidths;
  int           selector, index;
  double        value;
  LaserPointTag tag;
  
  for (selector=0; selector<3; selector++) {
  	index = ms_attribute_selector[selector]->currentIndex();
  	if (index > 0) {
  	  tag = AttributeTag(index);
      value = ms_bandwidth_line[selector+3]->text().toDouble();
      switch (AttributeType(tag)) {
      	default:
      	case IntegerAttributeType:
      	  new_bandwidths.Attribute(tag) = (int) value; break;
      	case FloatAttributeType:
      	  new_bandwidths.FloatAttribute(tag) = (float) value; break;
      	case DoubleAttributeType:
      	  new_bandwidths.DoubleAttribute(tag) = value; break;
      }
    }
  }
  new_bandwidths.X() = ms_bandwidth_line[0]->text().toDouble();
  new_bandwidths.Y() = ms_bandwidth_line[1]->text().toDouble();
  new_bandwidths.Z() = ms_bandwidth_line[2]->text().toDouble();
  parameters->AttributeBandWidths() = new_bandwidths;
}

void MeanShiftParametersWindow::SetMaxNumIterationsModeSeeking(int new_number)
{ parameters->MaxNumIterationsModeSeeking() = new_number; }

int MeanShiftParametersWindow::AttributeIndex(LaserPointTag tag)
{
  switch (tag) {
  	default:
  	case NoTag:              return 0;
  	case ReflectanceTag:     return 1;
  	case PulseLengthTag:     return 2;
  	case LabelTag:           return 3;
  	case ScaledNormalXTag:   return 4;
  	case ScaledNormalYTag:   return 5;
  	case ScaledNormalZTag:   return 6;
  }
  return 0;
}

LaserPointTag MeanShiftParametersWindow::AttributeTag(int index)
{
  switch (index) {
  	default:
  	case 0: return NoTag;
  	case 1: return ReflectanceTag;
  	case 2: return PulseLengthTag;
  	case 3: return LabelTag;
  	case 4: return ScaledNormalXTag;
  	case 5: return ScaledNormalYTag;
  	case 6: return ScaledNormalZTag;
  }
  return NoTag;
}





MajorityFilteringParametersWindow::MajorityFilteringParametersWindow(
  SegmentationParameters *par, QWidget *parent) : QWidget(parent)
{
  QDoubleValidator *non_negative_double;
  int              y;

  // Copy pointer to parameters
  parameters = par;

  // Initialise validator
  non_negative_double = new QDoubleValidator(0.00, 1.0e10, 2, this);

  setFixedSize(360, 140);

  // Majority filtering neighbourhood definition
  majority_nbh_selector = new QComboBox(this);
  majority_nbh_selector->setEditable(false);
  majority_nbh_selector->setFocusPolicy(Qt::NoFocus);
  majority_nbh_selector->addItem(QString("Direct neighbours"));
  majority_nbh_selector->addItem(QString("All within radius"));
  majority_nbh_selector->setCurrentIndex(parameters->MajorityNeighbourhoodDefinition());
  y = 25; majority_nbh_selector->setGeometry(220, y, 130, 18);
  connect(majority_nbh_selector, SIGNAL(activated(int)),
          this, SLOT(SetMajorityNeighbourhoodDefinition(int)));
            
  // Radius for majority filtering
  majority_radius_line = new QLineEdit(QString("%1").arg(parameters->MajorityNeighbourhoodRadius(), 0, 'f', 1), this);
  majority_radius_line->setValidator(non_negative_double);
  y += 20; majority_radius_line->setGeometry(220, y, 40, 18);
  connect(majority_radius_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMajorityRadius(const QString &)));

  // Attribute for majority filtering
  majority_attribute_selector = new QComboBox(this);
  majority_attribute_selector->setEditable(false);
  majority_attribute_selector->setFocusPolicy(Qt::NoFocus);
  majority_attribute_selector->addItem(QString("Label"));
  majority_attribute_selector->addItem(QString("Segment number"));
  majority_attribute_selector->addItem(QString("Component number"));
  majority_attribute_selector->addItem(QString("Segment+Tile number"));
  majority_attribute_selector->setCurrentIndex(MajorityAttributeIndex());
  y += 20; majority_attribute_selector->setGeometry(220, y, 130, 18);
  connect(majority_attribute_selector, SIGNAL(activated(int)),
          this, SLOT(SetMajorityAttribute(int)));
          
  // Switch for filtering all or only unlabeled points
  majority_no_attribute_only_selector = new QCheckBox(this);
  majority_no_attribute_only_selector->setChecked(parameters->MajorityNoAttributeOnly());
  y += 20; majority_no_attribute_only_selector->setGeometry(220, y+2, 40, 18);
  connect(majority_no_attribute_only_selector, SIGNAL(toggled(bool)),
          this, SLOT(SetMajorityNoAttributeOnly(bool)));
          
  // Switch for filtering using or excluding surfaces
  majority_no_surfaces_selector = new QCheckBox(this);
  majority_no_surfaces_selector->setChecked(parameters->MajorityNoSurfaces());
  y += 20; majority_no_surfaces_selector->setGeometry(220, y+2, 40, 18);
  connect(majority_no_surfaces_selector, SIGNAL(toggled(bool)),
          this, SLOT(SetMajorityNoSurfaces(bool)));
}

void MajorityFilteringParametersWindow::Update()
{
  majority_nbh_selector->setCurrentIndex(parameters->MajorityNeighbourhoodDefinition());
  majority_radius_line->setText(QString("%1").arg(parameters->MajorityNeighbourhoodRadius(), 0, 'f', 1));
  majority_attribute_selector->setCurrentIndex(MajorityAttributeIndex());
  majority_no_attribute_only_selector->setChecked(parameters->MajorityNoAttributeOnly());
  majority_no_surfaces_selector->setChecked(parameters->MajorityNoSurfaces());
}

void MajorityFilteringParametersWindow::paintEvent(QPaintEvent *event)
{
  QPainter paint(this);
  int      y;

  if (event) y = 20; // To avoid compiler warnings
  // Majority filtering parameters
  y = 20; paint.drawText(12, y, QString("Majority filtering parameters"));
  y += 20; paint.drawText(32, y, QString("Filter neighbourhood definition:"));
  y += 20; paint.drawText(32, y, QString("Filter neighbourhood radius:"));
  y += 20; paint.drawText(32, y, QString("Attribute for majority filtering:"));
  y += 20; paint.drawText(32, y, QString("Only filter points without attribute:"));
  y += 20; paint.drawText(32, y, QString("Only use non-planar segments:"));
}

void MajorityFilteringParametersWindow::SetMajorityNeighbourhoodDefinition(int new_definition)
{ parameters->MajorityNeighbourhoodDefinition() = new_definition; }

void MajorityFilteringParametersWindow::SetMajorityRadius(const QString &new_radius)
{ if (strlen(new_radius.toLatin1().data()) == 0) return;
  parameters->MajorityNeighbourhoodRadius() = new_radius.toDouble(); }

void MajorityFilteringParametersWindow::SetMajorityAttribute(int new_attribute)
{ 
  switch (new_attribute) {
    case 0: parameters->MajorityAttribute() = LabelTag; break;
    default:
    case 1: parameters->MajorityAttribute() = SegmentNumberTag; break;
    case 2: parameters->MajorityAttribute() = ComponentNumberTag; break;
    case 3: parameters->MajorityAttribute() = LongSegmentNumberTag; break;
  }
}

int MajorityFilteringParametersWindow::MajorityAttributeIndex() const
{
  switch ((int) parameters->MajorityAttribute()) {
    case LabelTag:             return 0;
    case SegmentNumberTag:     return 1;
    case ComponentNumberTag:   return 2;
    case LongSegmentNumberTag: return 3;
    default: return -1;
  }
  return -1;
}

void MajorityFilteringParametersWindow::SetMajorityNoAttributeOnly(bool new_switch)
{ parameters->MajorityNoAttributeOnly() = new_switch; }

void MajorityFilteringParametersWindow::SetMajorityNoSurfaces(bool new_switch)
{ parameters->MajorityNoSurfaces() = new_switch; }






SurfaceMergingParametersWindow::SurfaceMergingParametersWindow(
  SegmentationParameters *par, QWidget *parent) : QWidget(parent)
{
  QDoubleValidator *positive_double, *non_negative_double;
  int              y;
  double           degree = atan(1.0) / 45.0;


  // Copy pointer to parameters
  parameters = par;

  // Initialise validator
  positive_double = new QDoubleValidator(0.01, 1.0e10, 2, this);
  non_negative_double = new QDoubleValidator(0.00, 1.0e10, 2, this);

  setFixedSize(360, 150);

  // Merging neighbourhood definition
  merging_nbh_selector = new QComboBox(this);
  merging_nbh_selector->setEditable(false);
  merging_nbh_selector->setFocusPolicy(Qt::NoFocus);
  merging_nbh_selector->addItem(QString("Direct neighbours"));
  merging_nbh_selector->addItem(QString("All within radius"));
  merging_nbh_selector->setCurrentIndex(parameters->MergingNeighbourhoodDefinition());
  y = 25; merging_nbh_selector->setGeometry(240, y, 110, 18);
  connect(merging_nbh_selector, SIGNAL(activated(int)),
          this, SLOT(SetMergingNeighbourhoodDefinition(int)));
            
  // Radius for merging
  merging_radius_line = new QLineEdit(QString("%1").arg(parameters->MergingNeighbourhoodRadius(), 0, 'f', 1), this);
  merging_radius_line->setValidator(non_negative_double);
  y += 20; merging_radius_line->setGeometry(240, y, 40, 18);
  connect(merging_radius_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMergingRadius(const QString &)));

  // Maximum angle between planes
  max_angle_planes_line = new QLineEdit(QString("%1").arg(parameters->MaxAnglePlanes()/degree, 0, 'f', 1), this);
  max_angle_planes_line->setValidator(positive_double);
  y += 20; max_angle_planes_line->setGeometry(240, y, 40, 18);
  connect(max_angle_planes_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxAnglePlanes(const QString &)));

  // Maximum slope angle of planes to be merged
  max_slope_angle_line = new QLineEdit(QString("%1").arg(parameters->MergingMaxSlopeAngle()/degree, 0, 'f', 1), this);
  max_slope_angle_line->setValidator(positive_double);
  y += 20; max_slope_angle_line->setGeometry(240, y, 40, 18);
  connect(max_slope_angle_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxSlopeAngle(const QString &)));

  // Maximum distance between point and plane of other segment
  max_dist_other_segment_line = new QLineEdit(QString("%1").arg(parameters->MaxDistPointToOtherSegment(), 0, 'f', 2), this);
  max_dist_other_segment_line->setValidator(positive_double);
  y += 20; max_dist_other_segment_line->setGeometry(240, y, 40, 18);
  connect(max_dist_other_segment_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxDistPointToOtherSegment(const QString &)));

  // Minimum number of points in both planes within neighbourhood
  min_num_pts_both_planes_selector = new QSpinBox(this);
  min_num_pts_both_planes_selector->setMinimum(1);
  min_num_pts_both_planes_selector->setMaximum(100);
  min_num_pts_both_planes_selector->setSingleStep(1);
  min_num_pts_both_planes_selector->setValue(parameters->MinNumberPointsBothPlanes());
  y += 20; min_num_pts_both_planes_selector->setGeometry(240, y, 50, 18);
  connect(min_num_pts_both_planes_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetMinNumberPointsBothPlanes(int)));
}

void SurfaceMergingParametersWindow::Update()
{
  double           degree = atan(1.0) / 45.0;
  
  merging_nbh_selector->setCurrentIndex(parameters->MergingNeighbourhoodDefinition());
  merging_radius_line->setText(QString("%1").arg(parameters->MergingNeighbourhoodRadius(), 0, 'f', 1));
  max_angle_planes_line->setText(QString("%1").arg(parameters->MaxAnglePlanes()/degree, 0, 'f', 1));
  max_slope_angle_line->setText(QString("%1").arg(parameters->MergingMaxSlopeAngle()/degree, 0, 'f', 1));
  max_dist_other_segment_line->setText(QString("%1").arg(parameters->MaxDistPointToOtherSegment(), 0, 'f', 2));
  min_num_pts_both_planes_selector->setValue(parameters->MinNumberPointsBothPlanes());
}

void SurfaceMergingParametersWindow::paintEvent(QPaintEvent *event)
{
  QPainter paint(this);
  int      y;

  if (event) y = 20; // To avoid compiler warnings
  // Segment merging parameters
  y = 20; paint.drawText(12, y, QString("Merging criteria"));
  y += 20; paint.drawText(32, y, QString("Neighbourhood definition:"));
  y += 20; paint.drawText(32, y, QString("Neighbourhood radius:"));
  y += 20; paint.drawText(32, y, QString("Maximum angle between planes:"));
  y += 20; paint.drawText(32, y, QString("Maximum slope angle of planes:"));
  y += 20; paint.drawText(32, y, QString("Maximum distance of point to other plane:"));
  y += 20; paint.drawText(32, y, QString("Minimum number of points in both planes:"));
}

void SurfaceMergingParametersWindow::SetMergingNeighbourhoodDefinition(int new_definition)
{ parameters->MergingNeighbourhoodDefinition() = new_definition; }

void SurfaceMergingParametersWindow::SetMergingRadius(const QString &new_radius)
{ if (strlen(new_radius.toLatin1().data()) == 0) return;
  parameters->MergingNeighbourhoodRadius() = new_radius.toDouble(); }

void SurfaceMergingParametersWindow::SetMaxAnglePlanes(const QString &new_max_angle)
{ double degree = atan(1.0) / 45.0;
  if (strlen(new_max_angle.toLatin1().data()) == 0) return;
  parameters->MaxAnglePlanes() = new_max_angle.toDouble() * degree; }

void SurfaceMergingParametersWindow::SetMaxSlopeAngle(const QString &new_max_angle)
{ double degree = atan(1.0) / 45.0;
  if (strlen(new_max_angle.toLatin1().data()) == 0) return;
  parameters->MergingMaxSlopeAngle() = new_max_angle.toDouble() * degree; }

void SurfaceMergingParametersWindow::SetMaxDistPointToOtherSegment(const QString &new_max_distance)
{ if (strlen(new_max_distance.toLatin1().data()) == 0) return;
  parameters->MaxDistPointToOtherSegment() = new_max_distance.toDouble(); }

void SurfaceMergingParametersWindow::SetMinNumberPointsBothPlanes(int new_number)
{ parameters->MinNumberPointsBothPlanes() = new_number; }
