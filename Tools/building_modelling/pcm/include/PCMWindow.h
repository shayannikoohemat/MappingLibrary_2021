
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


#ifndef PCMWINDOW_H
#define PCMWINDOW_H

#include "PCMCanvas.h"
#include "ObjectPoints.h"
#include "BuildingPart.h"
#include "DataAppearance.h"
#include <QMainWindow>
#include <QToolButton>
#include <QToolBar>
#include <QTimer>
#include <QAction>

enum PCMWindowType {PCMMain, PCMCopiedDataView, PCMResult, PCMPyramidView,
                    PCMMainDataView};
typedef enum PCMWindowType PCMWindowType;

enum PCMViewAction {ViewLevel, ViewHorizontal, ViewFit, ViewFitLastModel,
                    ViewFitLaser};

class PCMWindow: public QMainWindow
{
    Q_OBJECT

  protected:
    /// PCMWindow type
    PCMWindowType window_type;

    /// The canvas
    PCMCanvas *canvas;

    /// The show data toolbar
    QToolBar *show_tools;

    /// Show data actions
    QAction *show_data_actions[NumNormalDataTypes];

    /// Show background image actions
    QAction *show_background_actions[NumBackGroundTypes];
  
    /// View toolbar
    QToolBar *view_tools;

    /// View actions
    QAction *view_actions[4];

    /// Auto rotation timer
    QTimer *rotation_timer;

    /// Reconstructed model points
    ObjectPoints last_model_points;

    /// Reconstructed model part
    BuildingPart last_model_part;

    /// Data appearance of the reconstructed model
    const DataAppearance *last_model_appearance;

    /// Laser points (local to this window)
    LaserPoints laser_points;

    /// Laser points (of the main window, only used by PCMPyramidView and PCMMainDataView)
    LaserPoints *laser_points_main;
    
    /// Data appearance of the laser points
    const DataAppearance *laserdata_appearance;

  public:
    /// Default constructor
    PCMWindow(PCMWindowType type, QWidget *parent=NULL, 
              Qt::WindowFlags flags=Qt::Window);

    /// Default destructor
    ~PCMWindow();

    /// Return the pointer
    PCMWindow * PCMWindowPtr() {return this;}

    /// Return the window type
    PCMWindowType Type() const {return window_type;}

    /// Return the canvas
    PCMCanvas * Canvas() {return canvas;}

    /// Set a show data button
    void SetShowDataButton(DataType type, bool pressed);

    /// Set a show image button
    void SetShowImageButton(BackGroundType type, bool pressed);

    /// Check if data of a specific type is shown
    bool DataIsShown(DataType type);

    /// Add reconstruction results
    void AddReconstructedModel(const ObjectPoints &points,
                               const BuildingPart &part,
                               const DataAppearance *app);

    /// Return the reconstructed points
    ObjectPoints * LastModelPoints() { return &last_model_points; }

    /// Return the reconstructed building part
    BuildingPart * LastModelPart() { return &last_model_part; }

    /// Add laser data
    void AddLaserData(LaserPoints &points, const DataAppearance *app, 
                      bool show=true);

    /// Return the laser data
    LaserPoints * PointCloud();
    
    /// Return the laser data appearance
    const DataAppearance * LaserDataAppearance() const { return laserdata_appearance; }

  private:
    /// Create the show data toolbar
    void CreateShowToolBar();
 
    /// Create the view toolbar
    void CreateViewToolBar();

  private slots:
    /// Toggle map data display
    void ToggleMapDataDisplay() {ToggleDataDisplay(MapData);}

    /// Toggle map partition data display
    void ToggleMapPartitionDataDisplay() {ToggleDataDisplay(MapPartitionData);}

    /// Toggle model data display
    void ToggleModelDataDisplay() {ToggleDataDisplay(ModelData);}

    /// Toggle last model data display
    void ToggleLastModelDataDisplay() {ToggleDataDisplay(LastModelData);}

    /// Toggle laser data display
    void ToggleLaserDataDisplay() {ToggleDataDisplay(LaserData); emit ToggledLaserDataDisplay();}

    /// Toggle height image background display
    void ToggleHeightImageDisplay() {ToggleImageDisplay(HeightImage);}

    /// Toggle shaded height image background display
    void ToggleShadedHeightImageDisplay()
      {ToggleImageDisplay(ShadedHeightImage);}

    /// Toggle ortho image background display
    void ToggleOrthoImageDisplay() {ToggleImageDisplay(OrthoImage);}

    /// Accept reconstructed model
    void AcceptReconstructedModel();

    /// Edit the reconstructed model in the main window
    void EditReconstructedModel();
    
    /// Reject reconstructed model
    void RejectReconstructedModel();
    
    /// Level view
    void LevelView() {canvas->LevelView();}

    /// Horizontal view
    void HorizontalView() {canvas->HorizontalView();}

    /// Fit view to all data
    void FitViewToData() {canvas->FitViewToData();}

    /// Fit view to laser data
    void FitViewToLaserData()
      {if (DataIsShown(LaserData)) canvas->FitViewToData(LaserData);}

    /// Fit view to reconstructed model data
    void FitViewToLastModelData()
      {if (DataIsShown(LastModelData)) canvas->FitViewToData(LastModelData);}

    /// Toggle auto rotation mode
    void ToggleAutoRotationMode();

    /// Increment scene rotation in auto rotation mode
    void IncrementRotation();

    /// Move contents of this window to the main window
    void MoveToMainWindow()
      {emit RequestMoveToMainWindow(this);}

    /// Add contents of this window to the main window
    void AddToMainWindow()
      {emit RequestAddToMainWindow(this);}

  protected:
    /// Toggle data display
    void ToggleDataDisplay(DataType type, bool remove_old_last_model=false);

    /// Toggle background image display
    void ToggleImageDisplay(BackGroundType type);

  signals:
    /// Request data display
    void RequestDataDisplay(DataType type, PCMWindow *window);

    /// Request image background display
    void RequestImageDisplay(BackGroundType type, PCMWindow *window);

    /// Data has been removed from the canvas
    void RemovedData(DataType type);

    /// Window will be closed
    void WindowWillBeClosed(PCMWindow *window);

    /// Request save of reconstructed model data
    void RequestSavingReconstructedModel(PCMWindow *window);

    /// Request to transfer data of this window to the main window
    void RequestMoveToMainWindow(PCMWindow *window);
    
    /// Request to add data of this window to the main window
    void RequestAddToMainWindow(PCMWindow *window);
    
    /// Request result to be edited in main window
    void RequestEditingBuildingModel(PCMWindow *window);
    
    /// Display of laser data has been toggled
    void ToggledLaserDataDisplay();
};
#endif /* PCMWINDOW_H */
