
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
  
    /// Selector for attribute to store component number
    QComboBox *component_attribute_selector;
  
    /// Selector for erasing old component labels
    QCheckBox *erase_old_label_selector;
  
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

    /// Edit line for maximum reflectance difference to seed surface
    QLineEdit *refl_dif_seed_line;
    
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
  
    /// Edit line for maximum reflectance difference to growing surface
    QLineEdit *refl_dif_growing_line;
    
    /// Selector for letting surfaces compete in growing phase
    QCheckBox *compete_selector;
  
  public:

    /// Constructor
    SegmentationParametersWindow(QWidget *parent=NULL);

    /// Default destructor
    ~SegmentationParametersWindow() {};

  protected:

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
    
    /// Set the component attribute
    void SetComponentAttribute(int new_attribute);
    
    /// Return index of component attribute in combobox
    int ComponentAttributeIndex() const;

    /// Set the switch for erasing old component labels
    void SetEraseOldLabels(bool new_switch);
   
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

    /// Set the maximum reflectance difference with seed plane centre
    void SetMaxReflectanceDifferenceSeed(const QString &new_max_dif);

// Parameters controlling the surface growing

    /// Set the surface model
    void SetSurfaceModel(int new_model);

    /// Set the surface growing neighbourhood definition
    void SetGrowingNeighbourhoodDefinition(int new_definition);
    
    /// Set the radius for surface growing
    void SetGrowingRadius(const QString &new_radius);

    /// Set the maximum distance between a point and the surface
    void SetMaxDistanceSurface(const QString &new_distance);
    
    /// Set the minimum distance to trigger recomputation of local plane
    void SetMinDistanceRecompute(const QString &new_distance);

    /// Set the maximum reflectance difference with in growing phase
    void SetMaxReflectanceDifferenceGrowing(const QString &new_max_dif);

    /// Set the switch for letting surfaces compete
    void SetSurfacesCompete(bool new_switch);
    
    /// Update switches according to current parameter settings
    void Update();
};



class GeneralSegmentationParametersWindow : public QWidget
{
  Q_OBJECT

  protected:

    /// Segmentation parameters
    SegmentationParameters *parameters;
    
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
   
    /// Selector for minimum number of points in a segment
    QSpinBox *segment_size_selector;
  
    /// Selector for attribute to store segment number
    QComboBox *segment_attribute_selector;
  
    /// Selector for erasing old segment numbers
    QCheckBox *erase_old_number_selector;

  public:

    /// Constructor
    GeneralSegmentationParametersWindow(SegmentationParameters *par,
	                                    QWidget *parent=NULL);

    /// Default destructor
    ~GeneralSegmentationParametersWindow() {};

  protected:

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

// Parameters controlling general segment properties

    /// Set the minimum number of points in a segment
    void SetMinNumberOfPointsSegment(int new_number);
    
    /// Set the segment attribute
    void SetSegmentAttribute(int new_attribute);
    
    /// Return index of segment attribute in combobox
    int SegmentAttributeIndex() const;

    /// Set the switch for erasing old segment numbers
    void SetEraseOldNumbers(bool new_switch);
    
    /// Update switches according to current parameter settings
    void Update();
};


class ConnectedComponentParametersWindow : public QWidget
{
  Q_OBJECT

  protected:

    /// Segmentation parameters
    SegmentationParameters *parameters;

    /// Edit line for maximum distance between points in a connected component
    QLineEdit *dist_component_line;
  
  public:

    /// Constructor
    ConnectedComponentParametersWindow(SegmentationParameters *par,
	                                   QWidget *parent=NULL);

    /// Default destructor
    ~ConnectedComponentParametersWindow() {};

  protected:

    /// Paint some text
    void paintEvent(QPaintEvent *event);

  public slots:

    /// Set the maximum distance between points in a component
    void SetMaxDistanceInComponent(const QString &new_distance);  
	
    /// Update switches according to current parameter settings
    void Update(); 
};




class SurfaceGrowingParametersWindow : public QWidget
{
  Q_OBJECT

  protected:

    /// Segmentation parameters
    SegmentationParameters *parameters;

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

    /// Edit line for maximum reflectance difference to seed surface
    QLineEdit *refl_dif_seed_line;
    
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
  
    /// Edit line for maximum reflectance difference to growing surface
    QLineEdit *refl_dif_growing_line;
    
    /// Selector for letting surfaces compete in growing phase
    QCheckBox *compete_selector;
  
  public:

    /// Constructor
    SurfaceGrowingParametersWindow(SegmentationParameters *par,
	                               QWidget *parent=NULL);

    /// Default destructor
    ~SurfaceGrowingParametersWindow() {};

  protected:

    /// Paint some text
    void paintEvent(QPaintEvent *event);

  public slots:

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

    /// Set the maximum reflectance difference with seed plane centre
    void SetMaxReflectanceDifferenceSeed(const QString &new_max_dif);

// Parameters controlling the surface growing

    /// Set the surface model
    void SetSurfaceModel(int new_model);

    /// Set the surface growing neighbourhood definition
    void SetGrowingNeighbourhoodDefinition(int new_definition);
    
    /// Set the radius for surface growing
    void SetGrowingRadius(const QString &new_radius);

    /// Set the maximum distance between a point and the surface
    void SetMaxDistanceSurface(const QString &new_distance);
    
    /// Set the minimum distance to trigger recomputation of local plane
    void SetMinDistanceRecompute(const QString &new_distance);

    /// Set the maximum reflectance difference with in growing phase
    void SetMaxReflectanceDifferenceGrowing(const QString &new_max_dif);

    /// Set the switch for letting surfaces compete
    void SetSurfacesCompete(bool new_switch);
    
    /// Update switches according to current parameter settings
    void Update();
};



class SegmentGrowingParametersWindow : public QWidget
{
  Q_OBJECT

  protected:

    /// Segmentation parameters
    SegmentationParameters *parameters;

    /// Selectors for growing attribute definition
    QComboBox *growing_attribute_selector[3];

    /// Edit line for growing tolerance values
    QLineEdit *growing_tolerance_line[3];

  public:

    /// Constructor
    SegmentGrowingParametersWindow(SegmentationParameters *par,
	                               QWidget *parent=NULL);

    /// Default destructor
    ~SegmentGrowingParametersWindow() {};

  protected:

    /// Paint some text
    void paintEvent(QPaintEvent *event);

  private:
  	
  	/// Convert tag to index in pull-down list
  	int AttributeIndex(LaserPointTag tag);
  	
  	/// Convert index in pull-down list to tag
  	LaserPointTag AttributeTag(int index);
  	
  public slots:

    /// Update switches according to current parameter settings
    void Update(); 
    
    /// Set growing tolerances
    void SetGrowingTolerances();
};



class MeanShiftParametersWindow : public QWidget
{
  Q_OBJECT

  protected:

    /// Segmentation parameters
    SegmentationParameters *parameters;

    /// Selectors for mean shift attributes
    QComboBox *ms_attribute_selector[3];

    /// Edit line for attribute band width values
    QLineEdit *ms_bandwidth_line[6];
    
    /// Selector for maximum number of iterations
    QSpinBox *ms_max_iter_selector;

  public:

    /// Constructor
    MeanShiftParametersWindow(SegmentationParameters *par,
	                          QWidget *parent=NULL);

    /// Default destructor
    ~MeanShiftParametersWindow() {};

  protected:

    /// Paint some text
    void paintEvent(QPaintEvent *event);

  private:
  	
  	/// Convert tag to index in pull-down list
  	int AttributeIndex(LaserPointTag tag);
  	
  	/// Convert index in pull-down list to tag
  	LaserPointTag AttributeTag(int index);
  	
  public slots:

    /// Update switches according to current parameter settings
    void Update(); 
    
    /// Set band widths
    void SetBandWidths();
    
    /// Set maximum number of iterations for mode seeking
    void SetMaxNumIterationsModeSeeking(int new_maximum);
};



class MajorityFilteringParametersWindow : public QWidget
{
  Q_OBJECT

  protected:

    /// Segmentation parameters
    SegmentationParameters *parameters;

    /// Selector for majority filtering neighbourhood definition
    QComboBox *majority_nbh_selector;
  
    /// Edit line for majority filtering radius
    QLineEdit *majority_radius_line;
  
    /// Selector for attribute to filter on
    QComboBox *majority_attribute_selector;
  
    /// Selector for filtering all or only unlabeled points
    QCheckBox *majority_no_attribute_only_selector;

    /// Selector for filtering using or excluding surfaces
    QCheckBox *majority_no_surfaces_selector;

  public:

    /// Constructor
    MajorityFilteringParametersWindow(SegmentationParameters *par,
	                                  QWidget *parent=NULL);

    /// Default destructor
    ~MajorityFilteringParametersWindow() {};

  protected:

    /// Paint some text
    void paintEvent(QPaintEvent *event);

  public slots:

    /// Set the majority filtering neighbourhood definition
    void SetMajorityNeighbourhoodDefinition(int new_definition);
    
    /// Set the radius for majority filtering
    void SetMajorityRadius(const QString &new_radius);

    /// Set the attribute to filter on
    void SetMajorityAttribute(int new_attribute);
    
    /// Return index of majority filtering attribute in combobox
    int MajorityAttributeIndex() const;

    /// Set the switch for filtering all or only unlabeled points
    void SetMajorityNoAttributeOnly(bool new_switch);

    /// Set the switch for filtering using or excluding surfaces
    void SetMajorityNoSurfaces(bool new_switch);

    /// Update switches according to current parameter settings
    void Update(); 
};




class SurfaceMergingParametersWindow : public QWidget
{
  Q_OBJECT

  protected:

    /// Segmentation parameters
    SegmentationParameters *parameters;

    /// Selector for merging neighbourhood definition
    QComboBox *merging_nbh_selector;
  
    /// Edit line for merging radius
    QLineEdit *merging_radius_line;
  
    /// Edit line for maximum angle between planes
    QLineEdit *max_angle_planes_line;

    /// Edit line for maximum slope angle
    QLineEdit *max_slope_angle_line;

    /// Edit line for maximum istance between point and plane of other segment
    QLineEdit *max_dist_other_segment_line;
    
    /// Edit line for minimum number of points in both planes
    QSpinBox *min_num_pts_both_planes_selector;

  public:

    /// Constructor
    SurfaceMergingParametersWindow(SegmentationParameters *par,
	                               QWidget *parent=NULL);

    /// Default destructor
    ~SurfaceMergingParametersWindow() {};

  protected:

    /// Paint some text
    void paintEvent(QPaintEvent *event);

  public slots:

    /// Set the merging neighbourhood definition
    void SetMergingNeighbourhoodDefinition(int new_definition);
    
    /// Set the radius for merging
    void SetMergingRadius(const QString &new_radius);

    /// Set the maximum angle between planes
    void SetMaxAnglePlanes(const QString &new_angle);  
    
    /// Set the maximum slope angle of planes
    void SetMaxSlopeAngle(const QString &new_angle);  
    
    /// Set the maximum distance between point and plane of other segment
    void SetMaxDistPointToOtherSegment(const QString &new_distance);

    /// Set the minimum number of points in both planes in neighbourhood
    void SetMinNumberPointsBothPlanes(int new_number);
	
    /// Update switches according to current parameter settings
    void Update(); 
};

#endif // SEGMENTPARWIN_H
