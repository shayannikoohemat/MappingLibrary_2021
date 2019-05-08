
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


#ifndef CITYWINDOW_H
#define CITYINDOW_H

#include "CityCanvas.h"
#include "ObjectPoints.h"
#include "BuildingPart.h"
#include "Buildings.h"
#include "DataAppearance.h"
#include "DataTypes.h"
#include "DataAppearWin.h"
#include "LaserBlock.h"
#include <QMainWindow>
#include <QToolButton>
#include <QToolBar>
#include <QTimer>
#include <QAction>
#include <QStatusBar>


enum CityWindowType {CityMain, CityView, CityResult};
typedef enum CityWindowType CityWindowType;

enum FeatureWindowType{None, Primary, Wall, Window, Ground, Door, Roof,EstimatedRoof, RoofExtrusion, Extrusion, Contour, Outline, Gap};

enum CityViewAction {ViewLevel, ViewFit, ViewFitLastModel, ViewFitLaser};

class CityWindow: public QMainWindow
{
    Q_OBJECT

  protected:
    /// CityWindow type
    CityWindowType window_type;
    
    FeatureWindowType feature_type;
 
    int type;
        /// Laser block
    LaserBlock laser_block;
    
    
    /// Mouse mode actions of the edit menu
    QAction *mode_actions[NumModes];
    
    /// The canvas
    CityCanvas *canvas;

 
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
    ObjectPoints reconstructed_points;

    /// Reconstructed model part
    BuildingPart reconstructed_part;
    
     /// Return the selection of a selected data type
    LineTopsIterVector *CorrespondingSelection(DataType type);

    /// Data appearance of the reconstructed model
    const DataAppearance *reconstruction_appearance;

    /// Laser points
    LaserPoints laser_points;
    
    int max_segment_value;

    /// Data appearance of the laser points
    const DataAppearance *laserdata_appearance;
    
     /// Selected object point
    ObjectPoints selected_point;

    /// Selected map polygons
    LineTopsIterVector selected_map_data;

    /// Selection buffer for multiple nearby map polygons
    LineTopsIterVector nearby_map_data;

    /// Selected map partition polygons
    LineTopsIterVector selected_map_part_data;

    /// Selection buffer for multiple nearby map partitions
    LineTopsIterVector nearby_map_part_data;

    /// Selected model data
    LineTopsIterVector selected_model_data;

    /// Selected buffer for multiple nearby models
    LineTopsIterVector nearby_model_data;
    
    /// Selected buffer for multiple nearby model faces
    LineTopsIterVector nearby_model_face_data;

    /// Selected model part data
    LineTopsIterVector selected_model_part_data;

    /// Selected buffer for multiple nearby model parts
    LineTopsIterVector nearby_model_part_data;

    /// Appearance of the selected point
    DataAppearance *selected_point_appearance;
    
   /// Selected model face data
    LineTopsIterVector selected_model_face_data;

    /// Model points
    ObjectPoints model_points;

    /// Show selected data actions of the show menu
    QAction *show_selection_actions[15];
    
    
    ///  Scale location actions
    QAction *show_scale_actions[3];
    
    /// Scale tick size actions
    QAction *set_scale_tick_actions[5];

// Subwindows

    vector <CityWindow *> subwindows;

  
    /// Last reconstructed model points
    ObjectPoints last_model_points;

    /// Last reconstructed model part
    BuildingPart last_model_part;

    /// Data has been modified after last save
    bool save_needed;
    
     /// File with map (partition) points
    char *map_point_file;

    /// File with map (partition) topology
    char *map_top_file;
    
    
    
      /// Set the map point file name
    void SetMapPointFile(const char *point_file);

    /// Set the map topology file name
    void SetMapTopologyFile(const char *top_file);
    
    
    /// Clear all selections
    void ClearSelections(DataType type);
    

  public:
    /// Default constructor
    CityWindow(CityWindowType type, QWidget *parent=NULL, 
              Qt::WFlags flags=Qt::Window);
              /// Default constructor
    CityWindow(CityWindowType type, FeatureWindowType ftype, QWidget *parent=NULL, 
              Qt::WFlags flags=Qt::Window);

    /// Default destructor
    ~CityWindow();

    /// Return the pointer
    CityWindow * CityWindowPtr() {return this;}

    /// Return the window type
    CityWindowType Type() const {return window_type;}

    /// Return the canvas
    CityCanvas * Canvas() {return canvas;}
    
    void SetFeatureType(FeatureWindowType type) {feature_type=type;}
    
    /// Return the window type
    FeatureWindowType FeatureType() const {return feature_type;}
    
    
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
    ObjectPoints * ReconstructedPoints() { return &reconstructed_points; }

    /// Return the reconstructed building part
    BuildingPart * ReconstructedModel() { return &reconstructed_part; }

    /// Add laser data
    void AddLaserData(const LaserPoints &points, const DataAppearance *app, 
                      bool show=true);

    /// Return the laser data
    LaserPoints * PointCloud() { return &laser_points; }
      
       /// Return the points belonging to a data type
    ObjectPoints *CorrespondingPoints(DataType type);
    
     /// Buildings
    Buildings buildings;
    
    Line2D GBKN_local;
    
       /// Map points
    ObjectPoints map_points;
    
    ObjectPoints *map_points_ptr;
    
    /// Selected laser data
    LaserPoints selected_laser_points;
    
    /// Update a data selection
    void UpdateSelection(LineTopsIterVector &selection,
                         LineTopsIterVector &nearby_data,
                         LineTopsIterVector::iterator reference_line,
                         ObjectPoints *points,
                         DataType sel_type, bool add, 
                         bool check_reference_line, bool refresh=true,
                         bool change_reference_line_only=false);


    /// Return the nearby data buffer of a selected data type
    LineTopsIterVector *CorrespondingNearbyData(DataType type);
    
    /// Show the data of a specific type
    void ShowData(CityWindow *window, DataType base_type, bool show_base_data,
                  DataType selection_type, bool show_selected_data,
                  bool refresh=true, bool data_changed=false);

    /// Select map or model data within the selection rectangle
    void SelectInsideRectangle(MouseMode mode);

    /// Initialise the map (partition) polygon split line
    void InitialiseSplitLine(MouseMode mode);

    /// Initialise the extension line
    bool InitialiseExtensionLine();

    /// Initialise the moving point lines
    bool InitialiseMoveNodeLines();
    
    /// Data availability
    bool ContainsData(DataType type) const;
    
    /// GUI for editing of appearance of different data types
    DataAppearanceWindow *appearance[NumDataTypes], *result_appearance,*map_appearance;

    
    
    
  private:
    /// Create the show data toolbar
    void CreateShowToolBar();
 
    /// Create the view toolbar
    void CreateViewToolBar();
    
   
   /// Create the edittool toolbar
     void CreateEditToolBar();
    
   
     

  public slots:
         
      
    
    
    void SelectRoofExtrusion_simple();
    
    void SelectRoofExtrusion_complex();
    
    
     /// Select object data
    void SelectObjectData(const PointNumber &number,
                          const Position3D &map_pos,
                          DataType selection_type, DataType base_type);

         
    /// Toggle map data display
    void ToggleMapDataDisplay() {ToggleDataDisplay(MapData);}

    /// Toggle map partition data display
    void ToggleMapPartitionDataDisplay() {ToggleDataDisplay(MapPartitionData);}

    /// Toggle model data display
    void ToggleModelDataDisplay() {ToggleDataDisplay(ModelData);}

    /// Toggle last model data display
    void ToggleLastModelDataDisplay() {ToggleDataDisplay(LastModelData);}

    /// Toggle laser data display
    void ToggleLaserDataDisplay() {ToggleDataDisplay(LaserData);}

    /// Toggle height image background display
    void ToggleHeightImageDisplay() {ToggleImageDisplay(HeightImage);}

    /// Toggle shaded height image background display
    void ToggleShadedHeightImageDisplay()
      {ToggleImageDisplay(ShadedHeightImage);}

    /// Toggle ortho image background display
    void ToggleOrthoImageDisplay() {ToggleImageDisplay(OrthoImage);}

    /// Accept reconstructed model
    void AcceptReconstructedModel();

    /// Level view
    void LevelView() {canvas->LevelView();}

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
      
    /// Set the mouse mode to select laser segments
    void SetSelectLaserSegmentMode() {SetMode(SelectLaserSegmentMode);}
    
    /// Set the new mouse mode
    void SetMode(MouseMode new_mode);  
       
    /// Set the mouse mode to change the pose of the data
    void SetPoseChangeMode() {SetMode(PoseChangeMode);}  
   
    void  SetSelectMapMode() {SetMode(SelectMapMode);}
    /// Set the mouse mode to select map partition lines
    void SetSelectMapPartitionMode() {SetMode(SelectMapPartitionMode);}

    /// Set the mouse mode to select models
    void SetSelectModelMode() {SetMode(SelectModelMode);}

    /// Set the mouse mode to select model parts
    void SetSelectModelPartMode() {SetMode(SelectModelPartMode);}

    /// Set the mouse mode to select laser segments
    //void SetSelectLaserSegmentMode() {SetMode(SelectLaserSegmentMode);}

    /// Set the mouse mode to select rectangle on canvas
    void SetSelectRectangleMode() {SetMode(SelectRectangleMode);}

    /// Set mode for splitting into map polygons
    void SetSplitMapMode() {SetMode(SplitMapMode);}

    /// Set mode for splitting into map partition polygons
    void SetSplitMapPartitionMode() {SetMode(SplitMapPartitionMode);}

    /// Set mode for extending a selected line
    void SetExtendLineMode() {SetMode(ExtendLineMode);}

    /// Set mode for moving a node of a selected line
    void SetMoveNodeMode() {SetMode(MoveNodeMode);}

    /// Show the map data
    void ShowMapData() {ToggleShowData(MapData);}

    /// Show the map partition data
    void ShowMapPartitionData() {ToggleShowData(MapPartitionData);}

    /// Show the model data
    void ShowModelData() {ToggleShowData(ModelData);}

    /// Show the last model data
    void ShowLastModelData() {ToggleShowData(LastModelData);}

    /// Show the laser data
    void ShowLaserData() {ToggleShowData(LaserData);}

    /// Delete a line
    void DeleteLine();
    
    /// Crop to selected lines
    void CropLines();

    /// Delete the last edge of a line
    void DeleteLastEdge();

    /// Reverse the node order of a line
    void ReverseLine();

    /// Create a new line
    void CreateNewLine();

    /// Close a line
    void CloseLine();

    /// Set new start node
    void SetNewStartNode();

    /// Split a line into two parts
    void SplitLineAtSelectedNode();

    /// Partition a building
    void PartitionBuilding();
    
    
      /// Process short cut keys received by the canvas
    void ProcessShortCutKeys(QKeyEvent *event);


      /// Set the right scale bar display options
    void SetScaleBar(CityScaleLocation new_location, double new_distance,
                     bool update=true);
                     
    /// Set up the connectors for data viewing
    void AddShowDataConnectors(CityWindow *window);

    
        /// Select laser data from the laser block
    void SelectLaserData();

    /// Select laser data based on a selected laser point on the canvas
    void SelectLaserData(const LaserPoint &selected_point, DataType type);
    
     /// Select laser data using the selection box
    void SelectLaserDataByBox(bool from_block, bool inside, bool show_data);

    /// Select laser data using selected map (partition) data
    void SelectLaserDataByMap(bool from_block, bool inside, bool show_data);
    
     /// Show the selected map data
    void ShowSelectedMapData() {ToggleShowData(SelectedMapData);}

    /// Show the selected map partition data
    void ShowSelectedMapPartitionData()
      {ToggleShowData(SelectedMapPartitionData);}

    /// Show the selected models
    void ShowSelectedModelData() {ToggleShowData(SelectedModelData);}

    /// Show the selected model parts
    void ShowSelectedModelPartData() {ToggleShowData(SelectedModelPartData);}

    /// Show the selected laser data
    void ShowSelectedLaserData() {ToggleShowData(SelectedLaserData);}

    /// Show the selected point
    void ToggleShowSelectedPoint();
        
    /// Delete laser data (within a selection if there is a selection)
    void DeleteLaserData();

    /// Crop laser data to within a selection
    void CropLaserData();
    
    /// Unify segment number of laser data selection
    void UnifySegmentNumberSelectedLaserData();
    
    
    void EmitCreateOutline();
    
     void EmitImproveOutline();
    
    void EmitCreateExtru();
    
     ///Attract map point
    void AttractMapPoint(const LaserPoint &point);


  protected:
    /// Toggle data display
    void ToggleDataDisplay(DataType type);

    /// Toggle background image display
    void ToggleImageDisplay(BackGroundType type);
    
     /// Toggle display of data of a specific type
    void ToggleShowData(DataType type, bool refresh=true);

  signals:
    /// Request data display
    void RequestDataDisplay(DataType type, CityWindow *window);

    /// Request image background display
    void RequestImageDisplay(BackGroundType type, CityWindow *window);

    /// Data has been removed from the canvas
    void RemovedData(DataType type);

/* Obsolete
    /// Texture image has been removed from the canvas
    void RemovedImage(BackGroundType type);
*/

    /// Window will be closed
    void WindowWillBeClosed(CityWindow *window);

    /// Request save of reconstructed model data
    void RequestSavingReconstructedModel(CityWindow *window);

    /// Request to transfer data of this window to the main window
    void RequestMoveToMainWindow(CityWindow *window);
    
    //change the laser in main window
    void LaserChanged(FeatureWindowType window_type);
    
    void Show_segment_info(int no,FeatureWindowType window_type);
    
    
    void CreateRoofExtru(int extru_index, int extru_type);
    
    void MapAdjusted(FeatureWindowType type);
    
    void RemoveVertex(FeatureWindowType type, int vertex_number);
    void AddVertex(FeatureWindowType type, int vertex_number);
    
    void CreateExtru();
    
    void CreateOutline();
    
    void ImproveOutlineSignal();
    
};
#endif /* CityWINDOW_H */
