
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


#include <math.h>
#include "SegmentParWin.h"
#include <QPainter>
#include <QValidator>

SegmentationParametersWindow::SegmentationParametersWindow(QWidget *parent) :
  QWidget(parent), SegmentationParameters()
{
  QDoubleValidator *positive_double, *non_negative_double;
  int              y;
  double           degree = atan(1.0) / 45.0;

  positive_double = new QDoubleValidator(0.01, 1.0e10, 2, this);
  non_negative_double = new QDoubleValidator(0.00, 1.0e10, 2, this);

  setFixedSize(420, 550);

                seed_radius = 0.5;
                bin_size_distance = 0.1;
                max_dist_seed_plane = 0.1;
                growing_radius = 0.3;
                max_dist_surface = 0.1;
                competing_surfaces=false;

// Plane detection parameters

  // Neighbourhood storage model
  nbh_model_selector = new QComboBox(this);
  nbh_model_selector->setEditable(false);
  nbh_model_selector->setFocusPolicy(Qt::NoFocus);
  nbh_model_selector->addItem(QString("Delaunay TIN"));
  nbh_model_selector->addItem(QString("Octree"));
  nbh_model_selector->addItem(QString("Kd-tree"));
  nbh_model_selector->setCurrentIndex(NeighbourhoodStorageModel());
  y = 25; nbh_model_selector->setGeometry(145, y, 120, 18);
  connect(nbh_model_selector, SIGNAL(activated(int)),
          this, SLOT(SetNeighbourhoodStorageModel(int)));

  // Octree bin maximum number of points
  octree_bin_size_selector = new QSpinBox(this);
  octree_bin_size_selector->setMinimum(1);
  octree_bin_size_selector->setMaximum(9999);
  octree_bin_size_selector->setSingleStep(10);
  octree_bin_size_selector->setValue(OctreeBinMaxNumberOfPoints());
  y += 20; octree_bin_size_selector->setGeometry(270, y, 50, 18);
  connect(octree_bin_size_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetOctreeBinMaxNumberOfPoints(int)));

  // Octree bin overlap
  octree_overlap_line = new QLineEdit(QString("%1").arg(OctreeBinOverlap(), 0, 'f', 1),
                            this);
  octree_overlap_line->setValidator(positive_double);
  y += 20; octree_overlap_line->setGeometry(170, y, 40, 18);
  connect(octree_overlap_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetOctreeBinOverlap(const QString &)));

  // Distance metric dimension
  dim_selector = new QComboBox(this);
  dim_selector->setEditable(false);
  dim_selector->setFocusPolicy(Qt::NoFocus);
  dim_selector->addItem(QString("2D"));
  dim_selector->addItem(QString("3D"));
  dim_selector->setCurrentIndex(DistanceMetricDimension()-2);
  y += 20; dim_selector->setGeometry(170, y, 40, 18);
  connect(dim_selector, SIGNAL(activated(int)),
          this, SLOT(SetDistanceMetricDimension(int)));

  // Number of neighbours in kd-tree
  knn_selector = new QSpinBox(this);
  knn_selector->setMinimum(3);
  knn_selector->setMaximum(100);
  knn_selector->setSingleStep(1);
  knn_selector->setValue(NumberOfNeighbours());
  y += 20; knn_selector->setGeometry(250, y, 50, 18);
  connect(knn_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetNumberOfNeighbours(int)));

  // Maximum distance between two points in a component
  dist_component_line = new QLineEdit(QString("%1").arg(MaxDistanceInComponent(), 0, 'f', 1), this);
  dist_component_line->setValidator(positive_double);
  y += 60; dist_component_line->setGeometry(255, y, 40, 18);
  connect(dist_component_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxDistanceInComponent(const QString &)));

  // Minimum number of points in a component
  component_size_selector = new QSpinBox(this);
  component_size_selector->setMinimum(1);
  component_size_selector->setMaximum(999);
  component_size_selector->setSingleStep(1);
  component_size_selector->setValue(MinNumberOfPointsComponent());
  y += 20; component_size_selector->setGeometry(235, y, 40, 18);
  connect(component_size_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetMinNumberOfPointsComponent(int)));

  // Seed neighbourhood definition
  seed_nbh_selector = new QComboBox(this);
  seed_nbh_selector->setEditable(false);
  seed_nbh_selector->setFocusPolicy(Qt::NoFocus);
  seed_nbh_selector->addItem(QString("Direct neighbours"));
  seed_nbh_selector->addItem(QString("All within radius"));
  seed_nbh_selector->setCurrentIndex(SeedNeighbourhoodDefinition());
  y += 60; seed_nbh_selector->setGeometry(230, y, 110, 18);
  connect(seed_nbh_selector, SIGNAL(activated(int)),
          this, SLOT(SetSeedNeighbourhoodDefinition(int)));
          
  // Seed neighbourhood radius
  seed_radius_line = new QLineEdit(QString("%1").arg(SeedNeighbourhoodRadius(), 0, 'f', 1), this);
  seed_radius_line->setValidator(non_negative_double);
  y += 20; seed_radius_line->setGeometry(235, y, 40, 18);
  connect(seed_radius_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetSeedNeighbourhoodRadius(const QString &)));

  // Maximum slope angle of the Hough space
  max_slope_line = new QLineEdit(QString("%1").arg(MaxSlopeAngle()/degree, 0, 'f', 1), this);
  max_slope_line->setValidator(positive_double);
  y += 20; max_slope_line->setGeometry(235, y, 40, 18);
  connect(max_slope_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxSlopeAngle(const QString &)));
  
  // Bin size of the slope angle in the Hough space
  angle_bin_size_line = new QLineEdit(QString("%1").arg(BinSizeSlopeAngle()/degree, 0, 'f', 1), this);
  angle_bin_size_line->setValidator(positive_double);
  y += 20; angle_bin_size_line->setGeometry(235, y, 40, 18);
  connect(angle_bin_size_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetBinSizeSlopeAngle(const QString &)));
  
  // Bin size of the distance in the Hough space
  dist_bin_size_line = new QLineEdit(QString("%1").arg(BinSizeDistance(), 0, 'f', 1), this);
  dist_bin_size_line->setValidator(positive_double);
  y += 20; dist_bin_size_line->setGeometry(235, y, 40, 18);
  connect(dist_bin_size_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetBinSizeDistance(const QString &)));
  
  // Minimum number of points in a seed
  seed_size_selector = new QSpinBox(this);
  seed_size_selector->setMinimum(3);
  seed_size_selector->setMaximum(999);
  seed_size_selector->setSingleStep(1);
  seed_size_selector->setValue(MinNumberOfPointsSeed());
  y += 20; seed_size_selector->setGeometry(235, y, 40, 18);
  connect(seed_size_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetMinNumberOfPointsSeed(int)));

  // Maximum distance between a point and the seed plane
  dist_seed_line = new QLineEdit(QString("%1").arg(MaxDistanceSeedPlane(), 0, 'f', 2), this);
  dist_seed_line->setValidator(positive_double);
  y += 20; dist_seed_line->setGeometry(235, y, 40, 18);
  connect(dist_seed_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxDistanceSeedPlane(const QString &)));
  
  // Surface model
  surface_model_selector = new QComboBox(this);
  surface_model_selector->setEditable(false);
  surface_model_selector->setFocusPolicy(Qt::NoFocus);
  surface_model_selector->addItem(QString("Planar"));
  surface_model_selector->addItem(QString("Smooth"));
  surface_model_selector->setCurrentIndex(SurfaceModel());
  y += 60; surface_model_selector->setGeometry(215, y, 60, 18);
  connect(surface_model_selector, SIGNAL(activated(int)),
          this, SLOT(SetSurfaceModel(int)));
          
  // Growing neighbourhood definition
  growing_nbh_selector = new QComboBox(this);
  growing_nbh_selector->setEditable(false);
  growing_nbh_selector->setFocusPolicy(Qt::NoFocus);
  growing_nbh_selector->addItem(QString("Direct neighbours"));
  growing_nbh_selector->addItem(QString("All within radius"));
  growing_nbh_selector->setCurrentIndex(GrowingNeighbourhoodDefinition());
  y += 20; growing_nbh_selector->setGeometry(290, y, 110, 18);
  connect(growing_nbh_selector, SIGNAL(activated(int)),
          this, SLOT(SetGrowingNeighbourhoodDefinition(int)));
            
  // Radius for surface growing
  growing_radius_line = new QLineEdit(QString("%1").arg(GrowingRadius(), 0, 'f', 1), this);
  growing_radius_line->setValidator(non_negative_double);
  y += 20; growing_radius_line->setGeometry(235, y, 40, 18);
  connect(growing_radius_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetGrowingRadius(const QString &)));

  // Maximum distance between a point and the surface
  dist_growing_line = new QLineEdit(QString("%1").arg(MaxDistanceSurface(), 0, 'f', 2), this);
  dist_growing_line->setValidator(positive_double);
  y += 20; dist_growing_line->setGeometry(235, y, 40, 18);
  connect(dist_growing_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxDistanceSurface(const QString &)));
  
  // Minimum distance between a point and the surface to trigger recomputation
  // of the local plane
  dist_recompute_line = new QLineEdit(QString("%1").arg(MinDistanceRecompute(), 0, 'f', 2), this);
  dist_recompute_line->setValidator(positive_double);
  y += 20; dist_recompute_line->setGeometry(290, y, 40, 18);
  connect(dist_recompute_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMinDistanceRecompute(const QString &)));
  
  // Switch for letting surfaces compete
  compete_selector = new QCheckBox(this);
  compete_selector->setChecked(SurfacesCompete());
  y += 20; compete_selector->setGeometry(235, y+2, 40, 18);
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
  seed_nbh_selector->setCurrentIndex(SeedNeighbourhoodDefinition());
  seed_radius_line->setText(QString("%1").arg(SeedNeighbourhoodRadius(), 0, 'f', 1));
  max_slope_line->setText(QString("%1").arg(MaxSlopeAngle()/degree, 0, 'f', 1));
  angle_bin_size_line->setText(QString("%1").arg(BinSizeSlopeAngle()/degree, 0, 'f', 1));
  dist_bin_size_line->setText(QString("%1").arg(BinSizeDistance(), 0, 'f', 1));
  seed_size_selector->setValue(MinNumberOfPointsSeed());
  dist_seed_line->setText(QString("%1").arg(MaxDistanceSeedPlane(), 0, 'f', 2));
  surface_model_selector->setCurrentIndex(SurfaceModel());
  growing_nbh_selector->setCurrentIndex(GrowingNeighbourhoodDefinition());
  growing_radius_line->setText(QString("%1").arg(GrowingRadius(), 0, 'f', 1));
  dist_growing_line->setText(QString("%1").arg(MaxDistanceSurface(), 0, 'f', 2));
  dist_recompute_line->setText(QString("%1").arg(MinDistanceRecompute(), 0, 'f', 2));
  compete_selector->setChecked(SurfacesCompete());
}

void SegmentationParametersWindow::paintEvent(QPaintEvent *)
{
  QPainter paint(this);
  int      y;

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
  // Seed selection parameters
  y += 40; paint.drawText(12, y, QString("Seed selection parameters"));
  y += 20; paint.drawText(32, y, QString("Seed neighbourhood definition:"));
  y += 20; paint.drawText(32, y, QString("Seed neighbourhood radius:"));
  y += 20; paint.drawText(32, y, QString("Maximum slope angle:"));
  y += 20; paint.drawText(32, y, QString("Bin size slope angle:"));
  y += 20; paint.drawText(32, y, QString("Bin size distance:"));
  y += 20; paint.drawText(32, y, QString("Minimum number of seed points:"));
  y += 20; paint.drawText(32, y, QString("Maximum distance to plane:"));  
  // Surface growing parameters
  y += 40; paint.drawText(12, y, QString("Surface growing parameters"));  
  y += 20; paint.drawText(32, y, QString("Surface model:"));
  y += 20; paint.drawText(32, y, QString("Surface growing neighbourhood definition:"));
  y += 20; paint.drawText(32, y, QString("Surface growing radius:"));
  y += 20; paint.drawText(32, y, QString("Maximum distance to surface:"));
  y += 20; paint.drawText(32, y, QString("Minimum distance to recompute local plane:"));
  y += 20; paint.drawText(32, y, QString("Competing surfaces:"));
}

void SegmentationParametersWindow::SetNeighbourhoodStorageModel(int new_nbh_model)
{ nbh_model = new_nbh_model; }

void SegmentationParametersWindow::SetOctreeBinMaxNumberOfPoints(int new_number)
{ octree_bin_max_numpts = new_number; }

void SegmentationParametersWindow::SetOctreeBinOverlap(const QString &new_octree_bin_overlap)
{ octree_bin_overlap = new_octree_bin_overlap.toDouble(); }

void SegmentationParametersWindow::SetDistanceMetricDimension(int new_dist_dimension)
{ dist_dimension = new_dist_dimension + 2; }

void SegmentationParametersWindow::SetMaxDistanceInComponent(const QString &new_max_dist_connected)
{ max_dist_connected = new_max_dist_connected.toDouble(); }

void SegmentationParametersWindow::SetMinNumberOfPointsComponent(int new_min_numpts_component)
{ min_numpts_component = new_min_numpts_component; }

void SegmentationParametersWindow::SetNumberOfNeighbours(int new_number)
{ num_nbs = new_number; }

void SegmentationParametersWindow::SetSeedNeighbourhoodDefinition(int new_definition)
{ seed_neighbourhood = new_definition; }

void SegmentationParametersWindow::SetSeedNeighbourhoodRadius(const QString &new_radius)
{ seed_radius = new_radius.toDouble(); }

void SegmentationParametersWindow::SetMaxSlopeAngle(const QString &new_max_slope_angle)
{ double degree = atan(1.0) / 45.0;
  max_slope_angle = new_max_slope_angle.toDouble() * degree;
}

void SegmentationParametersWindow::SetBinSizeSlopeAngle(const QString &new_bin_size)
{ double degree = atan(1.0) / 45.0;
  bin_size_slope_angle = new_bin_size.toDouble() * degree;
}

void SegmentationParametersWindow::SetBinSizeDistance(const QString &new_bin_size)
{ bin_size_distance = new_bin_size.toDouble(); }

void SegmentationParametersWindow::SetMinNumberOfPointsSeed(int new_min_numpts)
{ min_numpts_seed = new_min_numpts; }

void SegmentationParametersWindow::SetMaxDistanceSeedPlane(const QString &new_max_distance)
{ max_dist_seed_plane = new_max_distance.toDouble(); }

void SegmentationParametersWindow::SetSurfaceModel(int new_model)
{ surface_model = new_model; }

void SegmentationParametersWindow::SetGrowingNeighbourhoodDefinition(int new_definition)
{ growing_neighbourhood = new_definition; }

void SegmentationParametersWindow::SetGrowingRadius(const QString &new_radius)
{ growing_radius = new_radius.toDouble(); }

void SegmentationParametersWindow::SetMaxDistanceSurface(const QString &new_distance)
{ max_dist_surface = new_distance.toDouble(); }

void SegmentationParametersWindow::SetMinDistanceRecompute(const QString &new_distance)
{ min_dist_recompute = new_distance.toDouble(); }

void SegmentationParametersWindow::SetSurfacesCompete(bool new_switch)
{ competing_surfaces = new_switch; }
