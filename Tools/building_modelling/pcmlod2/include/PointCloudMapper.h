
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


#ifndef POINTCLOUDMAPPER_H
#define POINTCLOUDMAPPER_H

#include "PCMWindow.h"
#include "PCMCanvas.h"
#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "DataAppearWin.h"
#include "LaserPyramid.h"
#include "Buildings.h"
#include "LineSegment2D.h"
#include "FittingParWin.h"
#include "SegmentParWin.h"
#include "FilteringParWin.h"
#include "OutliningParWin.h"
#include "LaserDataIOWin.h"
#include "PyramidBrowseThread.h"
#include "TileInformation.h"
#include "SelectWin.h"
#include "PointInformationWin.h"
#include <QMenu>
#include <QAction>
#include <QApplication>

class PointCloudMapper: public PCMWindow
{
    Q_OBJECT

  protected:

// User interface stuff

    /// GUI for editing of appearance of different data types
    DataAppearanceWindow *appearance[NumDataTypes], *result_appearance;

    /// GUI for editing of fitting parameters
    FittingParametersWindow *fitting_parameters;

    /// GUI for editing of general segmentation parameters
    GeneralSegmentationParametersWindow *general_segmentation_parameters;

    /// GUI for editing of connected component parameters
    ConnectedComponentParametersWindow *connected_component_parameters;

    /// GUI for editing of surface growing parameters
    SurfaceGrowingParametersWindow *surface_growing_parameters;

    /// GUI for editing of segment growing parameters
    SegmentGrowingParametersWindow *segment_growing_parameters;

    /// GUI for editing of mean shift parameters
    MeanShiftParametersWindow *mean_shift_parameters;

    /// GUI for editing of majority filtering parameters
    MajorityFilteringParametersWindow *majority_filtering_parameters;

    /// GUI for editing of segment merging parameters
    SurfaceMergingParametersWindow *surface_merging_parameters;

    /// GUI for editing of outlining parameters
    OutliningParametersWindow *outlining_parameters;
    
    /// GUI for editing filtering parameters
    FilteringParametersWindow *filtering_parameters;
    
    /// GUI for importing or exporting laser data from/to ASCII files
    LaserDataIOWindow *laserdata_io_window;
    
    /// GUI for selecting laser points
    SelectWindow *select_window;
    
    /// Point information window
    PointInformationWindow *point_information_window;
    
    /// Default laser point attributes
    LaserPoint default_laser_attributes;
    
    /// Mouse mode actions of the edit menu
    QAction *mode_actions[NumModes];

    /// Show selected data actions of the show menu
    QAction *show_selection_actions[NumAllDataTypes];

    ///  Scale location actions
    QAction *show_scale_actions[3];
    
    /// Scale tick size actions
    QAction *set_scale_tick_actions[7];
    
    /// Laser data selection tag actions
    QAction *set_laser_tag_actions[11];
    
    /// Edit model action
    QAction *edit_model_action;
    
    /// Reset view when loading new points
    QAction *reset_on_loading_points;
    
// General project data

    /// Project name
    char *project_name;
    /// Meta data file with file names of point cloud mapping project
    char *project_file;

// File names

    /// File with map (partition) points
    char *map_point_file;

    /// File with map (partition) topology
    char *map_top_file;
   
    /// File with model points
    char *model_point_file;

    /// File with model topology
    char *model_top_file;

    /// File with laser meta data
    char *laser_meta_file;

    /// File with laser points
    char *laser_point_file;
    
    /// File with laser pyramid data
    char *laser_pyramid_file;

// The building data

    /// Map points
    ObjectPoints map_points;

    /// Model points
    ObjectPoints model_points;

    /// Buildings
    Buildings buildings;

    /// Laser block
    LaserBlock laser_block;
    
    /// Laser pyramid
    LaserPyramid laser_pyramid;

    /// Local ground height for wall bottom reconstruction
    double local_ground_height;
    
    /// Data has been modified after last save
    bool save_needed;

// Processing parameters

    /// Segmentation parameters
    SegmentationParameters *segmentation_parameters;
  
// Selections

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

    /// Selected model part data
    LineTopsIterVector selected_model_part_data;

    /// Selected buffer for multiple nearby model parts
    LineTopsIterVector nearby_model_part_data;

    /// Selected model face data
    LineTopsIterVector selected_model_face_data;

    /// Selected buffer for multiple nearby model faces
    LineTopsIterVector nearby_model_face_data;

    /// Appearance of the selected point
    DataAppearance *selected_point_appearance;

    /// Selected laser data
    LaserPoints selected_laser_points;
    
    /// Tag used for selecting laser data
    LaserPointTag laser_selection_tag;

// Texture data

    /// Texture images
    Image *texture_images[4];

    /// Texture bounds
    DataBounds3D texture_bounds[4];

// Subwindows

    /// Vector of all subwindows
    vector <PCMWindow *> subwindows;

    /// Window with model being edited
    PCMWindow *edit_model_window;
    
// Browsing data

    /// Thread for browsing pyramid
    PyramidBrowseThread browse_thread;
    
    /// Tile bound information
    TileInformation tile_information;
    
    /// Project number increment
    int project_number_increment;
    
// Functions

  public:
    /// Default constructor
    PointCloudMapper(QApplication *app);

    /// Default destructor
    ~PointCloudMapper() {};

    /// Open a project meta data file
    bool OpenProject(const char *data_file, const char *ini_file=NULL,
	                 bool reset_view=true);

  protected:

    /// Read meta and mapping data
    bool ReadProject(const char *filename, bool meta_data_only=false);

    /// Show the data of a specific type
    void ShowData(PCMWindow *window, DataType base_type, bool show_base_data,
                  DataType selection_type, bool show_selected_data,
                  bool refresh=true, bool data_changed=false);

    /// Toggle display of data of a specific type
    void ToggleShowData(DataType type, bool refresh=true);

    /// Data availability
    bool ContainsData(DataType type) const;

    /// Save all project data
    bool SaveProject(bool ask_file_names, bool save_map, bool save_model);

    /// Save laser data
    bool SaveLaserData(LaserPoints &points);
    
    /// Set the map point file name
    void SetMapPointFile(const char *point_file);

    /// Set the map topology file name
    void SetMapTopologyFile(const char *top_file);

    /// Set the model point file name
    void SetModelPointFile(const char *point_file);

    /// Set the model topology file name
    void SetModelTopologyFile(const char *top_file);

    /// Set the laser meta data file name
    void SetLaserMetaFile(const char *meta_file);

    /// Set the laser pyramid data file name
    void SetLaserPyramidFile(const char *pyramid_file);

    /// Set the laser point file name
    void SetLaserPointFile(const char *point_file);

    /// Close the programme after checking the need for saving data
    void closeEvent(QCloseEvent *);

    /// Update a data selection
    void UpdateSelection(LineTopsIterVector &selection,
                         LineTopsIterVector &nearby_data,
                         LineTopsIterVector::iterator reference_line,
                         ObjectPoints *points,
                         DataType sel_type, bool add, 
                         bool check_reference_line, bool refresh=true,
                         bool change_reference_line_only=false);

    /// Return the points belonging to a data type
    ObjectPoints *CorrespondingPoints(DataType type);

    /// Return the selection of a selected data type
    LineTopsIterVector *CorrespondingSelection(DataType type);

    /// Return the nearby data buffer of a selected data type
    LineTopsIterVector *CorrespondingNearbyData(DataType type);

    /// Select map or model data within the selection rectangle
    void SelectInsideRectangle(MouseMode mode);

    /// Select map partition lines of a building
    void SelectMapPartitionLines(int building_number);
    
    /// Initialise the map (partition) polygon split line
    void InitialiseSplitLine(MouseMode mode);

    /// Initialise the extension line
    bool InitialiseExtensionLine();

    /// Initialise the moving point lines
    bool InitialiseMoveNodeLines();

    /// Generate height texture image
    bool GenerateHeightTexture(PCMWindow *);

    /// Generate shaded height texture image
    bool GenerateShadedHeightTexture(PCMWindow *);

    /// Fit a roof to the laser points
    bool FitRoof(RoofType type);

    /// Set up the connectors for data viewing
    void AddShowDataConnectors(PCMWindow *window);

    /// Clear all selections
    void ClearSelections(DataType type);

    /// Set the right scale bar display options
    void SetScaleBar(PCMScaleLocation new_location, double new_distance,
                     bool update=true);

    /// Set the laser data selection tag
    void SetLaserSelectionTag(int selection_tag_index);
    
    /// Modify the label of selected laser points
    void ModifyLabel(int new_label);
    
    /// Modify an attribute of selected laser points
    void ModifyLaserAttribute(const LaserPointTag tag);
    
    /// Initialise the browsing in a pyramid
    void InitialiseBrowseMode();
    
    /// Open a pyramid
    int OpenLaserPyramid();
    
    /// Increment/decrement project number with short cut keys "n" or "p"
    void SwitchProject(int increment);
    
    /// Remove segments based on a large ratio of a certain pulse type
    void RemoveSegmentsBasedOnPulseType(LaserPulseType pulse_type,
                                        double max_ratio);
                                        
    /// Assign display colour to laser points
    void AssignDisplayColourToLaserPoints();
    
    /// Spawn a window and copy (part of) the laser points
    void SpawnWindow(PCMWindowType type, int selection=0);
    
    /// Delete (a selection of) laser points
    void DeletePoints(int selection);
    
    void CreateEditGraphToolBar();
    
  private slots:

    /// Select a project meta data file (and open it by OpenProject)
    void SelectProject();

    /// Save data
    void SaveProject() {SaveProject(false, true, true);}

    /// Save data in different files
    void SaveProjectAs() {SaveProject(true, true, true);}

    /// Save map data
    void SaveMapData() {SaveProject(false, true, false);}

    /// Save map data in different files
    void SaveMapDataAs() {SaveProject(true, true, false);}

    /// Save model data
    void SaveModelData() {SaveProject(false, false, true);}

    /// Save model data in different files
    void SaveModelDataAs() {SaveProject(true, false, true);}

    /// Save laser data in different files
    void SaveLaserDataAs() {SaveLaserData(laser_points);}
    
    /// Save selected laser data in different files
    void SaveSelectedLaserDataAs() {SaveLaserData(selected_laser_points);}
    
    /// Save settings
    void SaveSettings() {WriteDefaults("pcm.ini");}
    
    /// Save settings in different file
    void SaveSettingsAs();
    
    /// Load settings from an arbitrary file
    void LoadSettings();
    
    /// Import map data
    void ImportMapData();

    /// Import PCM map data
    void ImportPCMMapData();

    /// Import PCM model data
    void ImportPCMModelData();

    /// Import laser pyramid
    void ImportLaserPyramid();

    /// Import laser block
    void ImportLaserBlock();

    /// Import a new set of laser points
    void ImportNewLaserPoints();

    /// Import additional laser points
    void ImportAdditionalLaserPoints();

    /// All points in the existing instance which are also contained in the loaded file will be removed (so the complement will remain)
    void RemoveLoadedLaserPointsFromSet();

    /// Save current settings and quit PCM
    void QuitPCM();
    
    /// Information about this programme
    void aboutPCM();

    /// List all defined short cuts
    void PrintShortCuts();
    
    /// Information about Qt
    void aboutQt();

    /// Set the new mouse mode
    void SetMode(MouseMode new_mode);

    /// Set the mouse mode to change the pose of the data
    void SetPoseChangeMode() {SetMode(PoseChangeMode);}

    /// Set the mouse mode to change the pose of the data in perspective projection
    void SetPoseChangePerspectiveMode() {SetMode(PoseChangePerspectiveMode);}

    /// Set the mouse mode to browse the laser pyramid
    void SetBrowseMode() {SetMode(BrowseMode);}

    /// Set the mouse mode to select map lines
    void SetSelectMapMode() {SetMode(SelectMapMode);}

    /// Set the mouse mode to select map partition lines
    void SetSelectMapPartitionMode() {SetMode(SelectMapPartitionMode);}

    /// Set the mouse mode to select models
    void SetSelectModelMode() {SetMode(SelectModelMode);}

    /// Set the mouse mode to select model parts
    void SetSelectModelPartMode() {SetMode(SelectModelPartMode);}

    /// Set the mouse mode to select model faces
    void SetSelectModelFaceMode() {SetMode(SelectModelFaceMode);}

    /// Set the mouse mode to select laser points
    void SetSelectLaserPointMode() {SetMode(SelectLaserPointMode);}

    /// Set the mouse mode to select laser segments
    void SetSelectLaserSegmentMode() {SetMode(SelectLaserSegmentMode);}

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

    /// Show the selected map data
    void ShowSelectedMapData() {ToggleShowData(SelectedMapData);}

    /// Show the selected map partition data
    void ShowSelectedMapPartitionData()
      {ToggleShowData(SelectedMapPartitionData);}

    /// Show the selected models
    void ShowSelectedModelData() {ToggleShowData(SelectedModelData);}

    /// Show the selected model parts
    void ShowSelectedModelPartData() {ToggleShowData(SelectedModelPartData);}

    /// Show the selected model faces
    void ShowSelectedModelFaceData() {ToggleShowData(SelectedModelFaceData);}

    /// Show the selected laser data
    void ShowSelectedLaserData() {ToggleShowData(SelectedLaserData);}

    /// Show the selected point
    void ToggleShowSelectedPoint();

    /// Show the tile boundaries
    void ToggleShowTileBoundaries();

    /// Add laser data to a window
    void DisplayLaserDataAfterImport();

    /// Add data to a window
    void DisplayData(DataType type, PCMWindow *window, bool data_changed=false);

    /// Display message after export of laser data
    void DisplayMessageAfterExport();
    
    /// Update the show menu for removed object data
    void UpdateShowDataMenu(DataType type);

    /// Select object data
    void SelectObjectData(const PointNumber &number,
                          const Position3D &map_pos,
                          DataType selection_type, DataType base_type);

    /// Toggle a selection from the nearby data buffer
    void ToggleSelectionData(DataType selection_type);

    /// Select laser data from the laser block
    void SelectLaserData();

    /// Select laser data based on a selection on the canvas
    void ChangeLaserPointSelection(LaserPoints *selected_point, bool add);

    /// Select laser data based on a selected laser point on the canvas
    void SelectLaserData(const LaserPoint &selected_point, DataType type);

    /// Display information on selected laser point
    void DisplayPointInformation(const LaserPoint &point);
    
    /// Delete laser data (within a selection if there is a selection)
    void DeleteSelectedLaserData();

    /// Crop laser data to within a selection
    void CropToSelectedLaserData();

    /// Select laser data using the selection box
    void SelectLaserDataByBox(bool from_block, bool inside, bool show_data);

    /// Select laser data using selected map (partition) data
    void SelectLaserDataByMap(bool from_block, bool inside, bool show_data);

    /// Process short cut keys received by the canvas
    void ProcessShortCutKeys(QKeyEvent *event);

    /// Split a building (partition) outline
    void SplitOutline(MouseMode mode, const LineSegment2D &split_line);

    /// Merge two building (partition) lines
    void MergeMapLines();

    /// Read the view parameter file
    void ReadView();

    /// Save the view parameters
    void SaveView();

    /// Fit the view to selected line data
    void FitViewToSelectedData();

    /// Fit the view to selected laser data
    void FitViewToSelectedLaserData();

    /// Reset view when loading new points
    void ToggleResetOnLoading() {};
    
    /// Automatically reconstruct roof shape
    void ReconstructRoof() {FitRoof(UnknownRoof);}

    /// Fit a flat roof to the laser data
    void FitFlatRoof() {FitRoof(FlatRoof);}

    /// Fit a desk roof to the laser data
    void FitShedRoof() {FitRoof(ShedRoof);}

    /// Fit a gable roof to the laser data
    void FitGableRoof() {FitRoof(GableRoof);}

    /// Fit a rotated gable roof to the laser data
    void FitRotatedGableRoof() {FitRoof(GableRoofRot);}

    /// Fit a hip roof to the laser data
    void FitHipRoof() {FitRoof(HipRoof);}

    /// Fit a rotated hip roof to the laser data
    void FitRotatedHipRoof() {FitRoof(HipRoofRot);}

    /// Fit a double sloped hip roof to the laser data
    void FitDoubleSlopedHipRoof() {FitRoof(HipRoof2);}

    /// Fit a rotated double sloped hip roof to the laser data
    void FitRotatedDoubleSlopedHipRoof() {FitRoof(HipRoof2Rot);}

    /// Fit a gambrel roof to the laser data
    void FitGambrelRoof() {FitRoof(GambrelRoof);}

    /// Fit a rotated gambrel roof to the laser data
    void FitRotatedGambrelRoof() {FitRoof(GambrelRoofRot);}

    /// Fit a sphere roof to the laser data
    void FitSphereRoof() {FitRoof(SphereRoof);}

    /// Fit a cone roof to the laser data
    void FitConeRoof() {FitRoof(ConeRoof);}

    /// Fit a cylinder roof to the laser data
    void FitCylinderRoof() {FitRoof(CylinderRoof);}

    /// Fit a rotated cylinder roof to the laser data
    void FitRotatedCylinderRoof() {FitRoof(CylinderRoofRot);}

    /// Fit a minimum bounding rectangle to the laser data
    void FitRectangularMapLine();

    /// Fit a polygonal map line to the laser data
    void FitPolygonalMapLine();

    /// Fit a circular map line to the laser data
    void FitCircularMapLine();

    /// Save lowest point as wall bottom height
    void SetLowestPointForWallReconstruction();
    
    /// Save the last reconstructed building model data
    void SaveReconstructedModel(PCMWindow *window);

    // Copy all points to view window
	void CopyAllPoints()
	  { SpawnWindow(PCMCopiedDataView, 0); }
    
	// Copy selected points to view window
    void CopySelectedPoints()
      { SpawnWindow(PCMCopiedDataView, 1); }
    
    // Copy segmented points to view window
    void CopySegmentedPoints()
      { SpawnWindow(PCMCopiedDataView, 2); }
    
	// Copy unsegmented points to view window
    void CopyUnsegmentedPoints()
      { SpawnWindow(PCMCopiedDataView, 3); }

    // Delete all points
	void DeleteAllPoints()
	  { DeletePoints(0); }
    
	// Delete selected points
    void DeleteSelectedPoints()
      { DeletePoints(1); }
    
    // Delete segmented points
    void DeleteSegmentedPoints()
      { DeletePoints(2); }
    
	// Delete unsegmented points
    void DeleteUnsegmentedPoints()
      { DeletePoints(3); }

    /// Segment laser data
    void SegmentLaserData();

    /// Grow surfaces
    void GrowSurfaces();
    
    /// Unify segment number of laser data selection
    void UnifySegmentNumberSelectedLaserData();
    
    // Remove small segments
    void RemoveSmallSegments();
    
    // Unlabel small segments
    void UnlabelSmallSegments();
    
    // Mean-shift segmentation of connected components
    void MeanShiftSegmentation();
    
    // Segmentation by growing connected components with similar attribute values
    void SegmentGrowing();
    
    // Majority filtering of segment numbers
    void MajorityFilterSegmentation();
    
    // Merge planar segments with similar normal vectors and nearby points
    void MergeSurfaces();
    
    // Remove segments with large number of non last pulse points
    void RemoveNonLastPulseSegments()
      {RemoveSegmentsBasedOnPulseType(NotLastPulse, 0.2);}
    
    // Remove segments with large number of multiple echo points
    void RemoveMultipleEchoSegments()
      {RemoveSegmentsBasedOnPulseType(MultiplePulse, 0.2);}
    
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

    /// Delete loose points
    void DeleteLoosePoints();

    /// Display information on selected laser point
    void DisplayPointInformation(const ObjectPoint &point, DataType type);
    
    /// Set background colour
    void ChangeBackGroundColour();
    
    /// Set background image
    bool SetBackGroundImage(BackGroundType selected_type, PCMWindow *window);

    /// Set no background image
    void SetNoBackGroundImage()
      {SetBackGroundImage(NoBackGroundImage, PCMWindowPtr());}

    /// Set height background image
    void SetHeightBackGroundImage()
      {SetBackGroundImage(HeightImage, PCMWindowPtr());}

    /// Set shaded height background image
    void SetShadedBackGroundImage()
      {SetBackGroundImage(ShadedHeightImage, PCMWindowPtr());}

    /// Set (ortho)photo background image
    void SetPhotoBackGroundImage()
      {SetBackGroundImage(OrthoImage, PCMWindowPtr());}

    /// Load background image
    void LoadBackGroundImage();

    /// Remove a PCMWindow from the list of subwindows
    void RemovePCMWindow(PCMWindow *window);

    /// Spawn window with copied data
    void SpawnCopiedDataWindow()
      { SpawnWindow(PCMCopiedDataView, 0); }

    /// Spawn window with same data
    void SpawnSameDataWindow()
      { if (mode_actions[BrowseMode]->isChecked()) SpawnWindow(PCMPyramidView);
		else SpawnWindow(PCMMainDataView); }

    /// Move data of sub window to the main window
    void TransferDataFromSubWindow(PCMWindow *);
    
    /// Add data of sub window to the main window
    void AddDataFromSubWindow(PCMWindow *);
    
    /// Do not show the scale bar
    void SetNoScaleBar()
      { SetScaleBar(NoScale, canvas->ScaleBarTickDistance()); }

    /// Show the scale bar in the left bottom corner
    void SetLeftBottomScaleBar()
      { SetScaleBar(LeftBottomScale, canvas->ScaleBarTickDistance()); }

    /// Show the scale bar in the centre
    void SetCentreScaleBar()
      { SetScaleBar(CentreScale, canvas->ScaleBarTickDistance()); }
      
    /// Set the scale tick distance to 1 mm
    void SetScaleTick1mm()
      { SetScaleBar(canvas->ScaleBarLocation(), 0.001); }
      
    /// Set the scale tick distance to 1 cm
    void SetScaleTick1cm()
      { SetScaleBar(canvas->ScaleBarLocation(), 0.01); }
      
    /// Set the scale tick distance to 10 cm
    void SetScaleTick10cm()
      { SetScaleBar(canvas->ScaleBarLocation(), 0.1); }

    /// Set the scale tick distance to 1 m
    void SetScaleTick1m()
      { SetScaleBar(canvas->ScaleBarLocation(), 1.0); }

    /// Set the scale tick distance to 10 m
    void SetScaleTick10m()
      { SetScaleBar(canvas->ScaleBarLocation(), 10.0); }

    /// Set the scale tick distance to 100 m
    void SetScaleTick100m()
      { SetScaleBar(canvas->ScaleBarLocation(), 100.0); }
    
    /// Set the scale tick distance to 1 km
    void SetScaleTick1km()
      { SetScaleBar(canvas->ScaleBarLocation(), 1000.0); }
    
    /// Set laser selection tag to label
    void SetLaserSelectionTagLabel()
      { SetLaserSelectionTag(0); }
        
    /// Set laser selection tag to segment number
    void SetLaserSelectionTagSegmentNumber()
      { SetLaserSelectionTag(1); }
        
    /// Set laser selection tag to plane number
    void SetLaserSelectionTagPlaneNumber()
      { SetLaserSelectionTag(2); }
        
    /// Set laser selection tag to scan number
    void SetLaserSelectionTagScanNumber()
      { SetLaserSelectionTag(3); }
        
    /// Set laser selection tag to scan number from which the AFN code is to be removed
    void SetLaserSelectionTagScanNumberWithoutAFN()
      { SetLaserSelectionTag(7); }
        
    /// Set laser selection tag to the AFN code in the scan number
    void SetLaserSelectionTagAFNCode()
      { SetLaserSelectionTag(8); }
        
    /// Set laser selection tag to filter status
    void SetLaserSelectionTagIsFiltered()
      { SetLaserSelectionTag(4); }
        
    /// Set laser selection tag to pulse count
    void SetLaserSelectionTagPulseCount()
      { SetLaserSelectionTag(5); }
        
    /// Set laser selection tag to component number
    void SetLaserSelectionTagComponentNumber()
      { SetLaserSelectionTag(6); }
        
    /// Set laser selection tag to segment+tile number
    void SetLaserSelectionTagSegmentAndTileNumber()
      { SetLaserSelectionTag(9); }
        
    /// Set laser selection tag to scan line number
    void SetLaserSelectionTagScanLineNumber()
      { SetLaserSelectionTag(10); }
        
    /// Modify the label of selected laser points
    void ModifyAttributeLabel()
      { ModifyLaserAttribute(LabelTag); }
      
    /// Modify the filter status of selected laser points
    void ModifyAttributeIsFiltered()
      { ModifyLaserAttribute(IsFilteredTag); }
      
    /// Modify the plane number of selected laser points
    void ModifyAttributePlaneNumber()
      { ModifyLaserAttribute(PlaneNumberTag); }
      
    /// Modify the segment number of selected laser points
    void ModifyAttributeSegmentNumber()
      { ModifyLaserAttribute(SegmentNumberTag); }
      
    /// Modify the scan number of selected laser points
    void ModifyAttributeScanNumber()
      { ModifyLaserAttribute(ScanNumberTag); }
      
    /// Update the selection of data from a laser pyramid
    void UpdatePyramidSelection(const DataBounds2D &bounds);
    
    /// Store default values of appearances and parameters
    void WriteDefaults(const char *filename) const;
    
    /// Read default values of appearances and windows
    void ReadDefaults(const char *filename);
    
    /// Reconstruct a roof corner
    void ReconstructRoofCorner();

    /// Intersect two selected laser segments to reconstruct a model line
    void Reconstruct3DLineSegment();

    /// Reconstruct building walls of edited model
    void ReconstructWalls();

    /// Load updated pyramid points
    void LoadUpdatedPyramidPoints(int num_pts);
    
    /// Edit building model based on selected data on main canvas
    void EditBuildingModel();
    
    /// Edit building model based on selected result window
    void EditBuildingModel(PCMWindow *window);
    
    /// Update the last model data in the result window
    void UpdateLastModelDataInResultWindow();

    /// Update laser data display
    void UpdateLaserDataDisplay();
    
    /// Filter laser data
    void FilterLaserData();

	//////////////////////////////////////////////////////////////////////////
	//edit building topology graph tools
	void ToggelIntersectSegments();
	void ToggelDeleteIntersect();
	void ToggelMergeSegments();
	void ToggelEstimatePlane();
	void ToggelReconstrcBldModel();
	void ToggelScatterBldModel();
	void ToggelConstructPCMModel();
};

#endif /* POINTCLOUDMAPPER_H */
