
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


#ifndef SEGMENTPARWIN_H
#define SEGMENTPARWIN_H

#include <stdio.h>
#include <stdlib.h>
#include "SegmentationParameters.h"
#include <limits.h>
#include <QWidget>
#include <QSpinBox>
#include <QLineEdit>
#include <QComboBox>
#include <QCheckBox>

class SegmentationParametersWindow : public QWidget, 
                                     public SegmentationParameters
{
  Q_OBJECT

  public:

    /// Constructor
    SegmentationParametersWindow(QWidget *parent=NULL);

    /// Default destructor
    ~SegmentationParametersWindow() {};

  protected:

    /// Selector for neighbourhood model
    QComboBox *nbh_model_selector;
  
    /// Selector for maximum number of points in an octree cell
    QSpinBox *octree_bin_size_selector;

    /// Edit line for octree overlap size
    QLineEdit *octree_overlap_line;
  
    /// Selector for distance dimension
    QComboBox *dim_selector;
  
    /// Selector for number of nearest neighbours
    QSpinBox *knn_selector;
   
    /// Edit line for maximum distance between points in a connected component
    QLineEdit *dist_component_line;
  
    /// Selector for minimum number of points in a connected component
    QSpinBox *component_size_selector;
  
    /// Selector for seed neighbourhood definition
    QComboBox *seed_nbh_selector;
  
    /// Edit line for seed radius
    QLineEdit *seed_radius_line;
  
    /// Edit line for maximum slope of seed surface
    QLineEdit *max_slope_line;
  
    /// Edit line for bin size of angle in Hough space
    QLineEdit *angle_bin_size_line;
  
    /// Edit line for bin size of distance in Hough space
    QLineEdit *dist_bin_size_line;
  
    /// Selector for minimum number of points in a seed surface
    QSpinBox *seed_size_selector;
  
    /// Edit line for maximum distance of point to seed surface
    QLineEdit *dist_seed_line;
  
    /// Selector for surface model for growing
    QComboBox *surface_model_selector;
  
    /// Selector for growing neighbourhood definition
    QComboBox *growing_nbh_selector;
  
    /// Edit line for growing radius
    QLineEdit *growing_radius_line;
  
    /// Edit line for maximum distance of point to growing surface
    QLineEdit *dist_growing_line;
  
    /// Edit line for maximum distance before recomputing local surface normal
    QLineEdit *dist_recompute_line;
  
    /// Selector for letting surfaces compete in growing phase
    QCheckBox *compete_selector;
    
    
    /// Paint some text
    void paintEvent(QPaintEvent *event);

  public slots:
         
// Parameters controlling the data organisation

    /// Set the neighbourhood storage model
    void SetNeighbourhoodStorageModel(int new_model);
    
    /// Set the octree bin size
    void SetOctreeBinMaxNumberOfPoints(int new_number);
    
    /// Set the octree bin overlap
    void SetOctreeBinOverlap(const QString &new_overlap);
    
    /// Set the distance metric dimension
    void SetDistanceMetricDimension(int new_dimension);
    
    /// Set the number of neighbours in a kd-tree
    void SetNumberOfNeighbours(int new_number);

// Parameters controlling the definition of connected components

    /// Set the maximum distance between points in a component
    void SetMaxDistanceInComponent(const QString &new_distance);
    
    /// Set the minimum number of points in a component
    void SetMinNumberOfPointsComponent(int new_number);
    
// Parameters controlling the seed selection

    /// Set the seed neighbourhood definition
    void SetSeedNeighbourhoodDefinition(int new_definition);
    
    /// Set the seed neighbourhood radius
    void SetSeedNeighbourhoodRadius(const QString &new_radius);

    /// Set the maximum slope angle of the Hough space
    void SetMaxSlopeAngle(const QString &new_max_slope_angle);

    /// Set the bin size of the slope angle in the Hough space
    void SetBinSizeSlopeAngle(const QString &new_bin_size);

    /// Set the bin size of the distance in the Hough space
    void SetBinSizeDistance(const QString &new_bin_size);

    /// Set the minimum number of points in a seed
    void SetMinNumberOfPointsSeed(int new_min_numpts);

    /// Set the maximum distance between a point and the seed plane
    void SetMaxDistanceSeedPlane(const QString &new_max_distance);

// Parameters controlling the surface growing

    /// Set the surface model
    void SetSurfaceModel(int new_model);

    /// Set the surface growing neighbourhood definition
    void SetGrowingNeighbourhoodDefinition(int new_definition);
    
    /// Set the radius for surface growing
    void SetGrowingRadius(const QString &new_radius);

    /// Set the maximum distance between a point and the surface
    void SetMaxDistanceSurface(const QString &new_distance);

    /// Set the switch for letting surfaces compete
    void SetSurfacesCompete(bool new_switch);
    
    /// Set the minimum distance to trigger recomputation of local plane
    void SetMinDistanceRecompute(const QString &new_distance);
    
    /// Update switches according to current parameter settings
    void Update();
};
#endif // SEGMENTPARWIN_H
