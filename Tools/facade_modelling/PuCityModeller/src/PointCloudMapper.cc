
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


#include "PointCloudMapper.h"
#include "PCMCanvas.h"

#include <QApplication>
#include <QIcon>
#include <QToolBar>
#include <QMenuBar>
#include <QMenu>
#include <QFileDialog>
#include <QMessageBox>
#include <QAction>
#include <QStatusBar>
#include <QString>
#include <QCloseEvent>
#include <QFont>

#include "digphot_arch.h"

PointCloudMapper::PointCloudMapper()
    : PCMWindow(PCMMain, NULL, Qt::Window)
{
  int     initial_width=700, initial_height=700, i;
  QAction *action;

////////////////////////////////////////////////////////////////////////////////
// Data initialisations

  project_name = project_file = NULL;
  map_point_file = map_top_file = NULL;
  model_point_file = model_top_file = NULL;
  laser_meta_file = laser_point_file = NULL;
  save_needed = false;

////////////////////////////////////////////////////////////////////////////////
// Toolbar and menu for file operations

  QToolBar *file_tools = addToolBar("File operations");
  QMenu    *file_menu  = menuBar()->addMenu(tr("&File"));
#ifdef linux
  file_tools->setIconSize(QSize(20, 20));
#else
  file_tools->setIconSize(QSize(18, 18));
#endif
  // Open project
  action = new QAction(QIcon(":/buttons/fileopen.xpm"), "&Open...", this);
  action->setStatusTip(tr("Open project"));
  action->setShortcut(Qt::CTRL + Qt::Key_O);
  connect(action, SIGNAL(triggered()), this, SLOT(SelectProject()));
  file_tools->addAction(action);
  file_menu->addAction(action);

  // Save sub menu
  QMenu *save_sub_menu = file_menu->addMenu("Save");

  // Save all project data
  action = new QAction(QIcon(":/buttons/filesave.xpm"), "All", this);
  action->setStatusTip(tr("Save project"));
  action->setShortcut(Qt::CTRL + Qt::Key_S);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveProject()));
  file_tools->addAction(action);
  save_sub_menu->addAction(action);
  
  // Save all project data as
  action = new QAction("All as...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveProjectAs()));
  save_sub_menu->addAction(action);

  // Save map data
  action = new QAction("Map", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveMapData()));
  save_sub_menu->addAction(action);

  // Save map data as
  action = new QAction("Map as...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveMapDataAs()));
  save_sub_menu->addAction(action);

  // Save model data
  action = new QAction("Model", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveModelData()));
  save_sub_menu->addAction(action);

  // Save model data as
  action = new QAction("Model as...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveModelDataAs()));
  save_sub_menu->addAction(action);

  // Save laser points as
  action = new QAction("Laser points as...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveLaserDataAs()));
  save_sub_menu->addAction(action);
  
  // Save selected laser points as
  action = new QAction("Selected laser points as...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveSelectedLaserDataAs()));
  save_sub_menu->addAction(action);
  
  // Save settings
  action = new QAction("Parameter settings", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveSettings()));
  save_sub_menu->addAction(action);
  
  // Save settings as
  action = new QAction("Parameter settings as...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveSettingsAs()));
  save_sub_menu->addAction(action);
  
  // Import sub menu
  QMenu *import_sub_menu = file_menu->addMenu("Import");

  // Import map data
  action = new QAction("Map...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ImportMap()));
  import_sub_menu->addAction(action);
  
  // Import model data
  action = new QAction("Model...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ImportModel()));
  import_sub_menu->addAction(action);
  
  // Import laser pyramid 
  action = new QAction("Laser pyramid...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ImportLaserPyramid()));
  import_sub_menu->addAction(action);
  
  // Import laser block 
  action = new QAction("Laser block...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ImportLaserBlock()));
  import_sub_menu->addAction(action);
  
  // Import laser points 
  action = new QAction("Laser points...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ImportNewLaserPoints()));
  import_sub_menu->addAction(action);
  
  // Import additional laser points 
  action = new QAction("Additional laser points...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ImportAdditionalLaserPoints()));
  import_sub_menu->addAction(action);
  
  // Import background image
  action = new QAction("Background image...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(LoadBackGroundImage()));
  import_sub_menu->addAction(action);
  
  // Close and quit programme
  action = new QAction("&Quit", this);
  action->setShortcut(Qt::CTRL + Qt::Key_Q);
  connect(action, SIGNAL(triggered()), this, SLOT(QuitPCM()));
  file_menu->addAction(action);
  
////////////////////////////////////////////////////////////////////////////////
// Toolbar and menu for edit operations

  QToolBar *edit_tools = new QToolBar("Edit operations", this);
  QMenu    *edit_menu  = menuBar()->addMenu(tr("&Edit"));
  addToolBar(Qt::BottomToolBarArea, edit_tools);
#ifdef linux
  edit_tools->setIconSize(QSize(20, 20));
#else
  edit_tools->setIconSize(QSize(18, 18));
#endif

  // Mouse mode sub menu
  QMenu *mode_sub_menu = edit_menu->addMenu("Mouse mode");

  // Mouse mode pose change
  action = mode_actions[PoseChangeMode] =
    new QAction(QIcon(":/buttons/posechangemode.xpm"), "Pose change", this);
  action->setStatusTip(tr("Pose change"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetPoseChangeMode()));
  edit_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode browse
  action = mode_actions[BrowseMode] =
    new QAction(QIcon(":/buttons/browsemode.xpm"), "Browse", this);
  action->setStatusTip(tr("Browse laser pyramid"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetBrowseMode()));
  edit_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode select map line
  action = mode_actions[SelectMapMode] =
    new QAction(QIcon(":/buttons/selectmap.xpm"), "Select map line", this);
  action->setStatusTip(tr("Select map line"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectMapMode()));
  edit_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode select map partition line
  action = mode_actions[SelectMapPartitionMode] =
    new QAction(QIcon(":/buttons/selectmappart.xpm"),
                "Select map partition line", this);
  action->setStatusTip(tr("Select map partition line"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectMapPartitionMode()));
  edit_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode select model
  action = mode_actions[SelectModelMode] =
    new QAction(QIcon(":/buttons/selectmodel.xpm"), "Select model", this);
  action->setStatusTip(tr("Select model"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectModelMode()));
  edit_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode select model part
  action = mode_actions[SelectModelPartMode] =
    new QAction(QIcon(":/buttons/selectmodelpart.xpm"), "Select model part",
                this);
  action->setStatusTip(tr("Select model part"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectModelPartMode()));
  edit_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode select laser segment
  action = mode_actions[SelectLaserSegmentMode] =
    new QAction(QIcon(":/buttons/selectsegment.xpm"), "Select laser segment",
                this);
  action->setStatusTip(tr("Select laser segment"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectLaserSegmentMode()));
  edit_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode select rectangular area on canvas
  action = mode_actions[SelectRectangleMode] =
    new QAction(QIcon(":/buttons/selectrect.xpm"), "Select area on canvas",
                this);
  action->setStatusTip(tr("Select area on canvas"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectRectangleMode()));
  edit_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode split map polygon
  action = mode_actions[SplitMapMode] =
    new QAction(QIcon(":/buttons/splitmap.xpm"), "Split map line", this);
  action->setStatusTip(tr("Split map line"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSplitMapMode()));
  edit_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode split map polygon
  action = mode_actions[SplitMapPartitionMode] =
    new QAction(QIcon(":/buttons/splitmappart.xpm"), "Select map partition",
                this);
  action->setStatusTip(tr("Split map partition"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSplitMapPartitionMode()));
  edit_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode extend line
  action = mode_actions[ExtendLineMode] =
    new QAction(QIcon(":/buttons/extendline.xpm"), "Extend line", this);
  action->setStatusTip(tr("Extend line"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetExtendLineMode()));
  edit_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode move point
  action = mode_actions[MoveNodeMode] =
    new QAction(QIcon(":/buttons/movenode.xpm"), "Move point", this);
  action->setStatusTip(tr("Move point"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetMoveNodeMode()));
  edit_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Make all above buttons toggle buttons and switch them all off
  for (int mode=0; mode<NumModes; mode++) {
    mode_actions[mode]->setCheckable(true);
    mode_actions[mode]->setChecked(false);
  }

  // Start mode is pose change TODO: move to end, not related to gui
  SetMode(PoseChangeMode);

  // End of mode buttons on edit toolbar
  edit_tools->addSeparator();

//------------------------------------------------------------------------------
// Edit appearance sub menu

  QMenu *appear_sub_menu = edit_menu->addMenu("Appearence of");

  // Sub menu items and window creation for data appearances
  for (i=0; i<NumDataTypes; i++) {
    appearance[i] = new DataAppearanceWindow((DataType) i);
    action = new QAction(QString(appearance[i]->DataTypeName()) + " data...",
                         this);
    connect(action, SIGNAL(triggered()), appearance[i], SLOT(Edit()));
    appear_sub_menu->addAction(action);
  }

  // Special appearance for laser data in result window
  result_appearance = new DataAppearanceWindow(LaserData);
  action = new QAction("Laser data in result view...", this);
  connect(action, SIGNAL(triggered()), result_appearance, SLOT(Edit()));
  appear_sub_menu->addAction(action);
  
  // Background colour
  action = new QAction("Background...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ChangeBackGroundColour()));
  appear_sub_menu->addAction(action);

//------------------------------------------------------------------------------
// Laser data stuff

  // Select laser data
  action = new QAction(QIcon(":/buttons/selectlaser.xpm"), "Select laser data",
                       this);
  action->setStatusTip(tr("Select laser data"));
  connect(action, SIGNAL(triggered()), this, SLOT(SelectLaserData()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);

  // Delete laser data
  action = new QAction(QIcon(":/buttons/deletelaser.xpm"), "Delete laser data",
                       this);
  action->setStatusTip(tr("Delete laser data"));
  connect(action, SIGNAL(triggered()), this, SLOT(DeleteLaserData()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);

  // Crop laser data
  action = new QAction(QIcon(":/buttons/croplaser.xpm"), "Crop laser data",
                       this);
  action->setStatusTip(tr("Crop laser data"));
  connect(action, SIGNAL(triggered()), this, SLOT(CropLaserData()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);

  // End of laser edit buttons on edit toolbar
  edit_tools->addSeparator();

//------------------------------------------------------------------------------
// Map data stuff

  // Delete a line
  action = new QAction(QIcon(":/buttons/deleteline.xpm"), "Delete line", this);
  action->setStatusTip(tr("Delete line"));
  connect(action, SIGNAL(triggered()), this, SLOT(DeleteLine()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);

  // Crop to selected lines
  action = new QAction(QIcon(":/buttons/croplines.xpm"), "Crop lines", this);
  action->setStatusTip(tr("Crop lines"));
  connect(action, SIGNAL(triggered()), this, SLOT(CropLines()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);

  // Delete the last edge of a line
  action = new QAction(QIcon(":/buttons/deletelastedge.xpm"),
                       "Delete last edge", this);
  action->setStatusTip(tr("Delete last edge"));
  connect(action, SIGNAL(triggered()), this, SLOT(DeleteLastEdge()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);

  // Reverse the node order of a line
  action = new QAction(QIcon(":/buttons/reverseline.xpm"), "Reverse line",
                       this);
  action->setStatusTip(tr("Reverse line"));
  connect(action, SIGNAL(triggered()), this, SLOT(ReverseLine()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);

  // Create a new line
  action = new QAction(QIcon(":/buttons/newline.xpm"), "Create new line", this);
  action->setStatusTip(tr("Create new line"));
  connect(action, SIGNAL(triggered()), this, SLOT(CreateNewLine()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);

  // Close a line
  action = new QAction(QIcon(":/buttons/closeline.xpm"), "Close line", this);
  action->setStatusTip(tr("Close line"));
  connect(action, SIGNAL(triggered()), this, SLOT(CloseLine()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);

  // Set a new start node of the selected line
  action = new QAction(QIcon(":/buttons/newstartnode.xpm"), "New start node",
                       this);
  action->setStatusTip(tr("New start node"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetNewStartNode()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);

  // Merge two map (partition) lines
  action = new QAction(QIcon(":/buttons/mergemap.xpm"),
                       "Merge map (partition) lines", this);
  action->setStatusTip(tr("Merge map (partition) lines"));
  connect(action, SIGNAL(triggered()), this, SLOT(MergeMapLines()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);

  // Split a map (partition) line into two open polygons
  action = new QAction(QIcon(":/buttons/splitline.xpm"),
                       "Split map (partition) lines", this);
  action->setStatusTip(tr("Split map (partition) lines"));
  connect(action, SIGNAL(triggered()), this, SLOT(SplitLineAtSelectedNode()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);

  // Partition a building
  action = new QAction(QIcon(":/buttons/partition.xpm"), "Partition building",
                       this);
  action->setStatusTip(tr("Partition building"));
  connect(action, SIGNAL(triggered()), this, SLOT(PartitionBuilding()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);

  action = new QAction("Delete loose points", this);
  connect(action, SIGNAL(triggered()), this, SLOT(DeleteLoosePoints()));
  edit_menu->addAction(action);

////////////////////////////////////////////////////////////////////////////////
// Toolbar and menu for show operations
// Additions to toolbar for show operations

  // QToolBar show_tools already created by PCMWindow
  QMenu *show_menu  = menuBar()->addMenu(tr("&Show"));

  // Show map data
  show_menu->addAction(show_data_actions[MapData]);

  // Show map partition data
  show_menu->addAction(show_data_actions[MapPartitionData]);

  // Show model data
  show_menu->addAction(show_data_actions[ModelData]);

  // Show last model data
  show_menu->addAction(show_data_actions[LastModelData]);

  // Show laser data
  show_menu->addAction(show_data_actions[LaserData]);

  // Show selected map data
  action = show_selection_actions[SelectedMapData] =
    new QAction("Selected map data", this);
  action->setCheckable(true);
  connect(action, SIGNAL(triggered()), this, SLOT(ShowSelectedMapData()));
  show_menu->addAction(action);

  // Show selected map partition data
  action = show_selection_actions[SelectedMapPartitionData] =
    new QAction("Selected map partition data", this);
  action->setCheckable(true);
  connect(action, SIGNAL(triggered()),
          this, SLOT(ShowSelectedMapPartitionData()));
  show_menu->addAction(action);

  // Show selected model data
  action = show_selection_actions[SelectedModelData] =
    new QAction("Selected model data", this);
  action->setCheckable(true);
  connect(action, SIGNAL(triggered()), this, SLOT(ShowSelectedModelData()));
  show_menu->addAction(action);

  // Show selected model part data
  action = show_selection_actions[SelectedModelPartData] =
    new QAction("Selected model part data", this);
  action->setCheckable(true);
  connect(action, SIGNAL(triggered()), this, SLOT(ShowSelectedModelPartData()));
  show_menu->addAction(action);

  // Show selected laser data
  action = show_selection_actions[SelectedLaserData] =
    new QAction("Selected laser data", this);
  action->setCheckable(true);
  action->setChecked(false);
  connect(action, SIGNAL(triggered()), this, SLOT(ShowSelectedLaserData()));
  show_menu->addAction(action);

  // Show selected point
  action = show_selection_actions[SelectedPointData] =
    new QAction("Selected point", this);
  action->setCheckable(true);
  action->setChecked(false);
  connect(action, SIGNAL(triggered()), this, SLOT(ToggleShowSelectedPoint()));
  show_menu->addAction(action);

//------------------------------------------------------------------------------
// Background image sub menu

  QMenu *background_sub_menu = show_menu->addMenu("Background");

  // Show no background image
  action = show_background_actions[NoBackGroundImage] =
    new QAction("None", this);
  action->setCheckable(true);
  action->setChecked(true);
  connect(action, SIGNAL(triggered()), this, SLOT(SetNoBackGroundImage()));
  background_sub_menu->addAction(action);
  
  // Show height texture
  background_sub_menu->addAction(show_background_actions[HeightImage]);

  // Show shaded height texture
  background_sub_menu->addAction(show_background_actions[ShadedHeightImage]);

  // Show ortho image texture
  background_sub_menu->addAction(show_background_actions[OrthoImage]);

//------------------------------------------------------------------------------
// Scale sub menu
  QMenu *scale_sub_menu = show_menu->addMenu("Scale bar");
  
  // Scale location sub sub menu
  QMenu *scale_location_menu = scale_sub_menu->addMenu("Location");
  action = show_scale_actions[NoScale] = new QAction("None", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetNoScaleBar()));
  scale_location_menu->addAction(action);

  action = show_scale_actions[LeftBottomScale] =
    new QAction("Left bottom", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetLeftBottomScaleBar()));
  scale_location_menu->addAction(action);

  action = show_scale_actions[CentreScale] = new QAction("Centre", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetCentreScaleBar()));
  scale_location_menu->addAction(action);

  // Scale tick distance sub sub menu
  QMenu *scale_tick_menu = scale_sub_menu->addMenu("Tick distance");
  action = set_scale_tick_actions[0] = new QAction("0.01 m", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetScaleTick1cm()));
  scale_tick_menu->addAction(action);
  
  action = set_scale_tick_actions[1] = new QAction("0.1 m", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetScaleTick10cm()));
  scale_tick_menu->addAction(action);
  
  action = set_scale_tick_actions[2] = new QAction("1 m", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetScaleTick1m()));
  scale_tick_menu->addAction(action);
  
  action = set_scale_tick_actions[3] = new QAction("10 m", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetScaleTick10m()));
  scale_tick_menu->addAction(action);
  
  action = set_scale_tick_actions[4] = new QAction("100 m", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetScaleTick100m()));
  scale_tick_menu->addAction(action);  

  // Set defaults
  for (i=0; i<3; i++) show_scale_actions[i]->setCheckable(true);
  for (i=0; i<5; i++) set_scale_tick_actions[i]->setCheckable(true);
  SetScaleBar(NoScale, 1.0, false);
  
//------------------------------------------------------------------------------
// Add one button to the toolbar for spawning the window

  show_tools->addSeparator();

  action = new QAction(QIcon(":/buttons/newwindow.xpm"), "Spawn window", this);
  action->setStatusTip(tr("Spawn window"));
  connect(action, SIGNAL(triggered()), this, SLOT(SpawnWindow()));
  show_tools->addAction(action);

//------------------------------------------------------------------------------
// Switch display of normal data off and display of selected data on

  for (i=0; i<NumNormalDataTypes; i++)
    show_data_actions[i]->setChecked(false);
  for (i=SelectedMapData; i<=SelectedModelPartData; i++)
    show_selection_actions[i]->setChecked(true);

////////////////////////////////////////////////////////////////////////////////
// Toolbar and menu for view operations

  // QToolBar view_tools already created by PCMWindow
  QMenu *view_menu  = menuBar()->addMenu(tr("&View"));

  // Read view file
  action = new QAction(QIcon(":/buttons/viewopen.xpm"), "Read view file", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ReadView()));
  view_menu->addAction(action);
  
  // Save view file
  action = new QAction(QIcon(":/buttons/viewsave.xpm"), "Save view file", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveView()));
  view_menu->addAction(action);
  
  // Level view
  view_menu->addAction(view_actions[ViewLevel]);

  // Horizontal view
  view_menu->addAction(view_actions[ViewHorizontal]);

  // Fit view to data
  view_menu->addAction(view_actions[ViewFit]);

  // Fit view to last model data
  view_menu->addAction(view_actions[ViewFitLastModel]);

  // Fit view to laser data
  view_menu->addAction(view_actions[ViewFitLaser]);

  // Fit view to selected map or model data
  action = new QAction(QIcon(":/buttons/viewfitselected.xpm"),
                       "Fit view to selected line data", this);
  connect(action, SIGNAL(triggered()), this, SLOT(FitViewToSelectedData()));
  view_menu->addAction(action);
  view_tools->addAction(action);

  // Fit view to selected laser data
  action = new QAction(QIcon(":/buttons/viewfitselectedlaser.xpm"),
                       "Fit view to selected laser data", this);
  connect(action, SIGNAL(triggered()), this, SLOT(FitViewToSelectedLaserData()));
  view_menu->addAction(action);
  view_tools->addAction(action);

////////////////////////////////////////////////////////////////////////////////
// Menu for segmentation operations

  QMenu *segmentation_menu  = menuBar()->addMenu(tr("Se&gmentation"));

  // Connected component segmentation
  action = new QAction(QIcon(":/buttons/segmentlaser.xpm"),
                       "Connected components", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SegmentLaserData()));
  segmentation_menu->addAction(action);
  
  // Surface growing
  action = new QAction("Surface growing", this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(GrowSurfaces()));
  segmentation_menu->addAction(action);

  // Unify label of selected laser data
  action = new QAction("Unify segment number of selected laser data", this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(UnifySegmentNumberSelectedLaserData()));
  segmentation_menu->addAction(action);

  // Remove small segments
  action = new QAction("Remove small segments", this);
  connect(action, SIGNAL(triggered()), this, SLOT(RemoveSmallSegments()));
  segmentation_menu->addAction(action);

  // Edit segmentation parameters
  segmentation_parameters = new SegmentationParametersWindow(NULL);
  segmentation_parameters->setWindowTitle("Segmentation parameters");
  action = new QAction("Edit segmentation parameters", this);
  connect(action, SIGNAL(triggered()), segmentation_parameters, SLOT(show()));
  segmentation_menu->addAction(action);

////////////////////////////////////////////////////////////////////////////////
// Toolbar and menu for fitting operations

  QToolBar *fitting_tools = new QToolBar("Fitting operations", this);
  QMenu    *fitting_menu  = menuBar()->addMenu(tr("&Fitting"));
  addToolBarBreak(Qt::BottomToolBarArea);
  addToolBar(Qt::BottomToolBarArea, fitting_tools);
#ifdef linux
  fitting_tools->setIconSize(QSize(20, 20));
#else
  fitting_tools->setIconSize(QSize(18, 18));
#endif

  // Fit flat roof
  action = new QAction(QIcon(":/buttons/flatroof.xpm"), "Fit flat roof", this);
  action->setStatusTip(tr("Fit flat roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitFlatRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Fit shed roof
  action = new QAction(QIcon(":/buttons/shedroof.xpm"), "Fit shed roof", this);
  action->setStatusTip(tr("Fit shed roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitShedRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Fit gable roof
  action = new QAction(QIcon(":/buttons/gableroof.xpm"), "Fit gable roof",
                       this);
  action->setStatusTip(tr("Fit gable roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitGableRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Fit rotated gable roof
  action = new QAction(QIcon(":/buttons/gableroofrot.xpm"),
                       "Fit rotated gable roof", this);
  action->setStatusTip(tr("Fit rotated gable roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitRotatedGableRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Fit hip roof
  action = new QAction(QIcon(":/buttons/hiproof.xpm"), "Fit hip roof",
                       this);
  action->setStatusTip(tr("Fit hip roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitHipRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Fit rotated hip roof
  action = new QAction(QIcon(":/buttons/hiproofrot.xpm"),
                       "Fit rotated hip roof", this);
  action->setStatusTip(tr("Fit rotated hip roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitRotatedHipRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Fit double sloped hip roof
  action = new QAction(QIcon(":/buttons/hiproof2.xpm"),
                       "Fit double sloped hip roof", this);
  action->setStatusTip(tr("Fit double sloped hip roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitDoubleSlopedHipRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Fit rotated double sloped hip roof
  action = new QAction(QIcon(":/buttons/hiproof2rot.xpm"),
                       "Fit rotated double sloped hip roof", this);
  action->setStatusTip(tr("Fit rotated double sloped hip roof"));
  connect(action, SIGNAL(triggered()),
          this, SLOT(FitRotatedDoubleSlopedHipRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Fit gambrel roof
  action = new QAction(QIcon(":/buttons/gambrelroof.xpm"), "Fit gambrel roof",
                       this);
  action->setStatusTip(tr("Fit gambrel roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitGambrelRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Fit rotated gambrel roof
  action = new QAction(QIcon(":/buttons/gambrelroofrot.xpm"),
                       "Fit rotated gambrel roof", this);
  action->setStatusTip(tr("Fit rotated gambrel roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitRotatedGambrelRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Fit sphere roof
  action = new QAction(QIcon(":/buttons/sphereroof.xpm"), "Fit sphere roof",
                       this);
  action->setStatusTip(tr("Fit sphere roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitSphereRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Automatic roof reconstruction
  action = new QAction(QIcon(":/buttons/autoroof.xpm"),
                       "Automatic reconstruction", this);
  action->setStatusTip(tr("Automatic reconstruction"));
  connect(action, SIGNAL(triggered()), this, SLOT(ReconstructRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Fit minimum bounding rectangle to laser points
  action = new QAction(QIcon(":/buttons/maprectangle.xpm"),
                       "Fit rectangular map line", this);
  action->setStatusTip(tr("Fit rectangular map line"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitRectangularMapLine()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Fit polygonal map line to laser points
  action = new QAction(QIcon(":/buttons/mappolygon.xpm"),
                       "Fit polygonal map line", this);
  action->setStatusTip(tr("Fit polygonal map line"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitPolygonalMapLine()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Determine smallest enclosing circle of laser points
  action = new QAction(QIcon(":/buttons/mapcircle.xpm"),
                       "Fit circular map line", this);
  action->setStatusTip(tr("Fit circular map line"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitCircularMapLine()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Edit fitting parameters
  fitting_parameters = new FittingParametersWindow(NULL);
  fitting_parameters->setWindowTitle("Fitting parameters");
  action = new QAction("Edit fitting parameters", this);
  connect(action, SIGNAL(triggered()), fitting_parameters, SLOT(show()));
  fitting_menu->addAction(action);

////////////////////////////////////////////////////////////////////////////////
// Menu for help operations

  QMenu *help_menu  = menuBar()->addMenu(tr("&Help"));

  // About PCM
  action = new QAction("&About", this);
  action->setShortcut(Qt::Key_F1);
  connect(action, SIGNAL(triggered()), this, SLOT(about()));
  help_menu->addAction(action); 

  // About Qt
  action = new QAction("About Qt", this);
  connect(action, SIGNAL(triggered()), this, SLOT(aboutQt()));
  help_menu->addAction(action); 

////////////////////////////////////////////////////////////////////////////////
// Pop-up menu for laser point attributes

  default_laser_attributes = new LaserAttributesWindow(NULL);
  default_laser_attributes->setWindowTitle("Default laser point attributes");
  action = new QAction("Edit default attributes", this);
  connect(action, SIGNAL(triggered()), default_laser_attributes, SLOT(show()));
  Canvas()->PopUpMenu()->addAction(action);
 
  action = new QAction("Set label to default", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetAttributeLabel()));
  Canvas()->PopUpMenu()->addAction(action);
  
  action = new QAction("Set filter status to default", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetAttributeIsFiltered()));
  Canvas()->PopUpMenu()->addAction(action);
  
  action = new QAction("Set plane number to default", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetAttributePlaneNumber()));
  Canvas()->PopUpMenu()->addAction(action);
  
  action = new QAction("Set segment number to default", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetAttributeSegmentNumber()));
  Canvas()->PopUpMenu()->addAction(action);
  
////////////////////////////////////////////////////////////////////////////////

  statusBar()->showMessage("Ready", 2000);
  resize(initial_width, initial_height + 75);

////////////////////////////////////////////////////////////////////////////////
// Connect signals and slots

  // Default connectors for showing data in PCMWindows
  AddShowDataConnectors(PCMWindowPtr());
  // Update show menu if data is removed by PCMWindow
  connect(PCMWindowPtr(), SIGNAL(RemovedData(DataType)),
          this, SLOT(UpdateShowDataMenu(DataType)));
/* TODO: Remove obsolete code
  // Update show menu if image is removed by PCMWindow
  connect(PCMWindowPtr(), SIGNAL(RemovedImage(BackGroundType)),
          this, SLOT(UpdateShowImageMenu(BackGroundType)));
*/
  // Select object data if an object point has been selected on the canvas
  connect(Canvas(), SIGNAL(SelectedObjectPoint(const PointNumber &,
                                               const Position3D &,
                                               DataType, DataType)),
          this, SLOT(SelectObjectData(const PointNumber &,
                                      const Position3D &,
                                      DataType, DataType)));
  // Select laser data if a laser point has been selected on the canvas
  connect(Canvas(), SIGNAL(SelectedLaserPoint(const LaserPoint &, DataType)),
          this, SLOT(SelectLaserData(const LaserPoint &, DataType)));
  // Display information on a selected laser point
  connect(Canvas(), SIGNAL(RequestForPointInformation(const LaserPoint &)),
          this, SLOT(DisplayPointInformation(const LaserPoint &)));
  // Display information on a selected object point
  connect(Canvas(),
          SIGNAL(RequestForPointInformation(const ObjectPoint &, DataType)),
          this, SLOT(DisplayPointInformation(const ObjectPoint &, DataType)));
  // Toggle a selection from the nearby data buffer
  connect(Canvas(), SIGNAL(ToggleSelectionRequest(DataType)),
          this, SLOT(ToggleSelectionData(DataType)));
  // Short cut keys
  connect(Canvas(), SIGNAL(CanvasKeyPressed(QKeyEvent *)),
          this, SLOT(ProcessShortCutKeys(QKeyEvent *)));
  // Split a map (partition) line
  connect(Canvas(), SIGNAL(SplitRequest(MouseMode, const LineSegment2D &)),
          this, SLOT(SplitOutline(MouseMode, const LineSegment2D &)));
  // Undo last line extension
  connect(Canvas(), SIGNAL(RequestLastEdgeDelete()),
          this, SLOT(DeleteLastEdge()));
  // Change edit mode
  connect(Canvas(), SIGNAL(LineHasBeenClosed(MouseMode)),
          this, SLOT(SetMode(MouseMode)));
  // Update selection of data from laser pyramid
  connect(Canvas(), SIGNAL(NewCanvasBounds(const DataBounds2D &)),
          this, SLOT(UpdatePyramidSelection(const DataBounds2D &)));

////////////////////////////////////////////////////////////////////////////////
// Set default values

  max_point_spacing_pixels = 5.0;
  result_appearance->SetDataTypeName("Laser result data");
  result_appearance->SetPointColourMethod(ColourByResidual);
  result_appearance->SetPointSize(3);
  appearance[SelectedLaserData]->SetPointColourMethod(ColourBySegment);
  ReadDefaults("pcm.ini");
}

void PointCloudMapper::ToggleShowData(DataType type, bool refresh)
{
// This is no longer toggling. The menu or button selection already cause
// the action to change it's state. So, we just display what the current
// check states indicate.

  DataType base_type, selection_type;

  if (type == LaserData) {
    // Toggle display of laser data
    ShowData(PCMWindowPtr(), type, show_data_actions[LaserData]->isChecked(),
             type, show_data_actions[LaserData]->isChecked(), refresh);
  }
  else if (type == SelectedLaserData) {
    // Toggle display of selected laser data
    base_type = LaserData;
    ShowData(PCMWindowPtr(), base_type,
             show_data_actions[base_type]->isChecked(), type,
             show_selection_actions[type]->isChecked(), refresh);
  }
  else if ((int) type < NumObjectDataTypes) {
    // Toggle display of all data, including selected data
    selection_type = Canvas()->CorrespondingSelectedDataType(type);
    ShowData(PCMWindowPtr(), type, show_data_actions[type]->isChecked(),
             selection_type, show_data_actions[type]->isChecked(), refresh);
  }
  else { // Selected object data
    // Toggle display of selected data only
    base_type = Canvas()->CorrespondingBaseDataType(type);
    ShowData(PCMWindowPtr(), base_type,
             show_data_actions[base_type]->isChecked(), type,
             show_selection_actions[type]->isChecked(), refresh);
  }
}

void PointCloudMapper::DisplayData(DataType type, PCMWindow *window,
                                   bool data_changed)
{
  ShowData(window, type, true,
           window->Canvas()->CorrespondingSelectedDataType(type), true,
           true, data_changed);
}

void PointCloudMapper::ShowData(PCMWindow *window, DataType base_type,
                                bool show_base_data, DataType selection_type,
                                bool show_selected_data, bool refresh,
                                bool data_changed)
{
  Buildings::iterator                      building;
  ObjectPoints                             *points;
  std::vector <LineTopologies *>           *polygon_sets;
  std::vector <LineTopologies *>::iterator polygon_set;
  LineTopsIterVector                       *selected_data;
  LineTopsIterVector::iterator             polygon;
  bool                             update_bounds = !window->Canvas()->HasData();
  vector <PCMWindow *>::iterator           subwindow;

  // For base data types that should be switched on, check if there's data.
  if ((int) base_type < NumNormalDataTypes && show_base_data) {
    if (!ContainsData(base_type) && !ContainsData(selection_type)) {
      QMessageBox::information(window, "Error", "There is no " +
                               QString(appearance[base_type]->DataTypename()) +
                               " data!");
      show_data_actions[base_type]->setChecked(false); // Reset action state
      return;
    }
  }

  // Check if the required setting implies a change. If there's no change,
  // no modifications of the canvas data pointers are needed
  // Note that this is not a correct check of model parts !!
  if (window->DataIsShown(base_type) == show_base_data &&
      (window->Type() != PCMMain ||
       window->DataIsShown(selection_type) == show_selected_data) &&
      !data_changed) {
    // Update, coordinates may have changed
    if (refresh) window->Canvas()->update();
    return;
  }

  // Laser data
  if (base_type == LaserData) {
    if (show_base_data) {
      window->Canvas()->AddLaserData(window->PointCloud(),
                              appearance[base_type]->DataAppearancePtr(), true,
                              false);
    }
    else window->Canvas()->RemoveLaserData(
                             appearance[LaserData]->DataAppearancePtr(), false);
    if (selection_type == SelectedLaserData) {
      if (show_selected_data && !selected_laser_points.empty()) {
        window->Canvas()->AddLaserData(&selected_laser_points,
                             appearance[SelectedLaserData]->DataAppearancePtr(),
                             true, false);
      }
      else
        window->Canvas()->RemoveLaserData(
                             appearance[SelectedLaserData]->DataAppearancePtr(),
                             false);
    }
  }

  // Object data
  else {
    // First remove all object data
    window->Canvas()->RemoveObjectData(base_type, false);
    if (selection_type != base_type)
      window->Canvas()->RemoveObjectData(selection_type, false);
    // Also remove selected model parts
    if (selection_type == SelectedModelData)
      window->Canvas()->RemoveObjectData(SelectedModelPartData, false);

    // Display normal object data
    if (show_base_data) {
      if (base_type == LastModelData) {
        if (last_model_part.Number() != -1) { // Valid model
          window->Canvas()->AddObjectData(&last_model_points,
                                   last_model_part.RoofFaces(),
                                   appearance[base_type]->DataAppearancePtr(),
                                   false, false);
          window->Canvas()->AddObjectData(&last_model_points,
                                   last_model_part.WallFaces(),
                                   appearance[base_type]->DataAppearancePtr(),
                                   false, false);
        }
      }
      else {
        for (building=buildings.begin(); building!=buildings.end();building++) {
          points = building->Points(base_type);
          polygon_sets = building->Topology(base_type);
          for (polygon_set = polygon_sets->begin();
               polygon_set != polygon_sets->end(); polygon_set++) {
            if (points && *polygon_set)
              window->Canvas()->AddObjectData(points, *polygon_set,
                                     appearance[base_type]->DataAppearancePtr(),
                                       false, false);
          }
          delete polygon_sets;
        }
      }
    }

    // Display selected object data (only in main window)
    if (selection_type != base_type && window->Type() == PCMMain) {
      selected_data = CorrespondingSelection(selection_type);
      points        = CorrespondingPoints(selection_type);
      for (polygon=selected_data->begin(); polygon!=selected_data->end();
           polygon++) {
        if (show_base_data) {
          if (show_selected_data)
            window->Canvas()->ChangeObjectAppearance(points, &**polygon,
                                appearance[selection_type]->DataAppearancePtr(),
                                              false);
          else
            window->Canvas()->RemoveObjectData(&**polygon, false);
        }
        else if (show_selected_data)
          window->Canvas()->AddObjectData(points, &**polygon,
                                appearance[selection_type]->DataAppearancePtr(),
                                   false);
      }
    }
  }

  // Display the selected data
  if (update_bounds) window->Canvas()->InitialiseTransformation();
  if (refresh) window->Canvas()->update();

  // Update the action settings of the menu and toolbar buttons
  if (window->Type() == PCMMain) { // Only for PointCloudMapper window
    if (selection_type != LastModelData &&
        selection_type != LaserData) { // Ignore double display of LaserData
      show_selection_actions[selection_type]->setChecked(show_selected_data);
    }
  }
  window->SetShowDataButton(base_type, show_base_data);

  // If shared data was changed in the main window, also update all other
  // windows. LastModelData and LaserData are not shared.
  if (data_changed && window->Type() == PCMMain && base_type != LastModelData &&
      base_type != LaserData) {
    for (subwindow=subwindows.begin(); subwindow!=subwindows.end();
         subwindow++) {
      if ((*subwindow)->DataIsShown(base_type))
        ShowData(*subwindow, base_type, true, selection_type, false,
                 true, true);
    }
  }
}

void PointCloudMapper::UpdateShowDataMenu(DataType type)
{
  // Switch off the corresponding show selection actions
  if (type <= ModelData)
    show_selection_actions[Canvas()->CorrespondingSelectedDataType(type)]->
      setChecked(false);
  if (type == ModelData)
    show_selection_actions[SelectedModelPartData]->setChecked(false);
/*
  show_menu->setItemChecked(id_show[type], show_data[type]->isOn());
  if (type <= ModelData)
    show_menu->setItemChecked(
                id_show[canvas->CorrespondingSelectedDataType(type)],
                show_data[type]->isOn());
  if (type == ModelData)
    show_menu->setItemChecked(id_show[SelectedModelPartData],
                              show_data[type]->isOn());
*/
}

/* TODO: Remove obsolete code
void PointCloudMapper::UpdateShowImageMenu(BackGroundType type)
{
  bool image_shown;
  if (type == NoBackGroundImage) image_shown = false;
  else image_shown = show_image[type]->isOn();
  background_menu->setItemChecked(id_background[type], image_shown);
  background_menu->setItemChecked(id_background[NoBackGroundImage],
                                  !image_shown);
  show_menu->setItemChecked(id_show[NumDataTypes+1], image_shown);
}
*/

void PointCloudMapper::ToggleShowSelectedPoint()
{
  // Remove selected point from display
  if (show_selection_actions[SelectedPointData]->isChecked()) {
    Canvas()->RemoveObjectData(selected_point_appearance);
  }
  // Add selected point to display
  else {
    // Check existance of appearance structure
    if (!selected_point_appearance)
      selected_point_appearance = new DataAppearance(SelectedPointData);
    Canvas()->AddObjectData(&selected_point, (LineTopology *) NULL,
                             selected_point_appearance);
  }
// TODO: check if this toggle is done automatically
  // Toggle the switch
  show_selection_actions[SelectedPointData]->
    setChecked(!show_selection_actions[SelectedPointData]->isChecked());
}

void PointCloudMapper::SelectProject()
{
  QString new_project_file =
    QFileDialog::getOpenFileName(this, "Select project file", QString::null,
                    "Point cloud mapping data (*.pcm*);;All files (*.*)");
  if (!new_project_file.isEmpty()) {
    if (OpenProject(new_project_file.toAscii()))
      statusBar()->showMessage("Project " + new_project_file + " opened.",2000);
    else
      statusBar()->showMessage("Could not open project "+new_project_file + ".",
                               2000);
  }
}

bool PointCloudMapper::OpenProject(const char *new_project_file)
{
  bool success = ReadProject(new_project_file, false);
  int type;

  if (Canvas()->HasData()) {
    Canvas()->ClearObjectData(false);
    Canvas()->ClearLaserData(false);
  }
  for (type=0; type<NumNormalDataTypes; type++)
    show_data_actions[type]->setChecked(false);
  for (type=SelectedMapData; type<=SelectedModelPartData; type++)
    show_selection_actions[type]->setChecked(false);
  if (ContainsData(MapData)) DisplayData(MapData, PCMWindowPtr(), true);
  if (ContainsData(MapPartitionData))
    DisplayData(MapPartitionData, PCMWindowPtr(), true);
  if (ContainsData(ModelData)) DisplayData(ModelData, PCMWindowPtr(), true);
  if (ContainsData(LaserData)) DisplayData(LaserData, PCMWindowPtr(), true);
  Canvas()->InitialiseTransformation();
  Canvas()->update();
  return success;
}

bool PointCloudMapper::SaveProject(bool ask_file_names, bool save_map,
                                   bool save_model)
{
  QString filename;

  if (save_map) {
    // Save map points
    if (!map_point_file || ask_file_names) {
      filename = QFileDialog::getSaveFileName(this, "Select map point file",
                                              map_point_file,
                                      "Map points (*.objpts);;All files (*.*)");
      if (filename.isEmpty()) return false;
      SetMapPointFile(filename.toAscii());
    }
    if (!map_points.Write(map_point_file)) return false;

    // Save map topology
    if (!map_top_file || ask_file_names) {
      filename = QFileDialog::getSaveFileName(this, "Select map topology file",
                                              map_top_file,
                                       "Map topology (*.top);;All files (*.*)");
      if (filename.isEmpty()) return false;
      SetMapTopologyFile(filename.toAscii());
    }
    if (!buildings.WriteMapData(map_top_file)) return false;
  }

  if (save_model) {
    // Save model points
    if (!model_point_file || ask_file_names) {
      filename = QFileDialog::getSaveFileName(this, "Select model point file",
                                              model_point_file,
             "Model points (*.objpts);;VRML model (*.wrl);;DXF file (*.dxf);;All files (*.*)");
      if (filename.isEmpty()) return false;
      if (filename.indexOf(".wrl", 0, Qt::CaseInsensitive) != -1) {
        buildings.WriteVRML(filename.toAscii());
        return true;
      }
      else if (filename.indexOf(".dxf", 0, Qt::CaseInsensitive) != -1) {
        buildings.WriteDXF(filename.toAscii());
        return true;
      }
      else {
        // No .wrl or .dxf file
        SetModelPointFile(filename.toAscii());
      }
    }
    if (!model_points.Write(model_point_file)) return false;

    // Save model topology
    if (!model_top_file || ask_file_names) {
      filename = QFileDialog::getSaveFileName(this,
                                              "Select model topology file",
                                              model_top_file,
                                     "Model topology (*.top);;All files (*.*)");
      if (filename.isEmpty()) return false;
      SetModelTopologyFile(filename.toAscii());
    }
    if (!buildings.WriteModelData(model_top_file)) return false;
  }

  return true;
}

bool PointCloudMapper::SaveLaserData(LaserPoints &points)
{
  QString filename;

  // Check if there are any points
  if (points.empty()) {
    if (&points == &laser_points)
      QMessageBox::information(this, "Error",
                               "There are no laser points.\n");
    else {
      QMessageBox::information(this, "Error",
                               "There are no selected laser points.\n");
    }
    return false;
  }
  
  // Get the file name
  filename = QFileDialog::getSaveFileName(this, "Select laser point file",
                                          points.PointFile(),
                  "Laser points (*.laser);;VRML file (*.wrl);;All files (*.*)");
  if (filename.isEmpty()) return false;
  if (filename.indexOf(".wrl", 0, Qt::CaseInsensitive) == -1) {
    // No .wrl or .WRL file
    // Save the points in the laser point format
    points.SetPointFile(filename.toAscii());
    if (!points.Write(points.PointFile(), false)) {
      QMessageBox::information(this, "Error",
                               "Writing laser points to file",
                               points.PointFile(), "failed!");
      return false;
    }
    statusBar()->showMessage(QString("Laser points written to file ") +
                             points.PointFile(), 2000);
  }
  else { // Write points as VRML file
    LaserPoints vrml_points = Canvas()->LaserPointsWithDisplayColour(&points,
                                   appearance[LaserData]->DataAppearancePtr());
    FILE *vrml_fd;
    vrml_fd = fopen(filename.toAscii(), "w");
    if (vrml_fd == NULL) {
      QMessageBox::information(this, "Error",
                               "Opening file", filename.toAscii(), "failed!");
      return false;
    }
    vrml_points.VRML_Write(vrml_fd, 2, true);
    vrml_points.ErasePoints();
    fclose(vrml_fd);
    statusBar()->showMessage(QString("Laser points written to file ") +
                             filename.toAscii(), 2000);
  }

  return true;
}

void PointCloudMapper::SaveSettingsAs()
{
  QString filename;

  // Get the file name
  filename = QFileDialog::getSaveFileName(this, "Select parameter setting file",
                                          "pcm.ini",
                                      "Setting files (*.ini);;All files (*.*)");
  if (filename.isEmpty()) return;
  WriteDefaults(filename.toAscii());
}



void PointCloudMapper::closeEvent(QCloseEvent *event)
{
  if (!save_needed) {
    event->accept();
    return;
  }

  switch(QMessageBox::information(this, "Point Cloud Mapper",
				  "Do you want to save the changes"
				  " to map and model data?",
				  "Yes", "No", "Cancel",
				  0, 1)) {
    case 0:
      SaveProject();
      event->accept();
      break;
    case 1:
      event->accept();
      break;
    case 2:
    default: // just for sanity
      event->ignore();
      break;
  }
}


void PointCloudMapper::about()
{
  QMessageBox::about(this, "Point Cloud Mapper",
		     "Interactive programme for modelling buildings\n"
		     "in laser scanner point clouds.\n\n"
                     "Author: George Vosselman");
}

void PointCloudMapper::aboutQt()
{
  QMessageBox::aboutQt( this, "Qt Application Example" );
}

void PointCloudMapper::SetMode(MouseMode new_mode)
{
  ObjectPoints       *point_data;
  LineTopsIterVector *selected_data;
  DataType           selection_type, base_type;
  MouseMode          old_mode, real_new_mode;
  bool               remove_old_selections=false;
  int                building_number;

  real_new_mode = new_mode; // This can be overridden

  // Figure out what what to do with old object data
  old_mode = Canvas()->Mode();
  if (old_mode != SelectRectangleMode &&
      new_mode >= SelectMapMode && new_mode <= SelectRectangleMode &&
      (old_mode != new_mode || !Canvas()->ShiftKeyDown())) {
    remove_old_selections = true;
  }
  else if (old_mode == SelectRectangleMode &&
           !Canvas()->HasValidSelectionRectangle() &&
           !Canvas()->ShiftKeyDown() &&
           new_mode >= SelectMapMode && new_mode <= SelectModelPartMode) {
    remove_old_selections = true;
  }
  if (old_mode == BrowseMode) laser_pyramid.InvalidateUsedTiles();

  // Retrieve the building number if the user wants to select all partitions
  // of a map outline
  if (old_mode == SelectMapMode && new_mode == SelectMapPartitionMode) {
    if (!selected_map_data.empty())
      building_number = (*(selected_map_data.begin()))->Number() / 1000;
    else
      building_number = -1;
  }
    
  // Remove the old map and model selection
  if (remove_old_selections) {
    for (selection_type=SelectedMapData; selection_type!=SelectedLaserData;
         selection_type = (DataType) (selection_type + 1)) {
      selected_data = CorrespondingSelection(selection_type);
      if (selected_data->size()) {
        base_type  = Canvas()->CorrespondingBaseDataType(selection_type);
        point_data = CorrespondingPoints(selection_type);
        UpdateSelection(*selected_data, *selected_data, selected_data->begin(),
                        point_data, base_type, false, false, true);
      }
      if (CorrespondingNearbyData(selection_type)->size())
        CorrespondingNearbyData(selection_type)->Clear();
    }
  }

  // Remove old laser data selections
  if ((new_mode == SelectLaserSegmentMode &&
       old_mode == SelectLaserSegmentMode) ||
      (new_mode == BrowseMode && !selected_laser_points.empty())) {
    selected_laser_points.ErasePoints();
    ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, false, true,
             true);
  }

  // Remove old split line
  if (old_mode == SplitMapMode || old_mode == SplitMapPartitionMode)
    Canvas()->RemoveSplitLine();

  // Remove old extension line
  if (old_mode == ExtendLineMode) Canvas()->RemoveExtensionLine();

  // Remove old move node lines
  if (old_mode == MoveNodeMode) Canvas()->RemoveMoveNodeLines();

  // If there is a valid selection rectangle, use this to select the
  // data of the requested type
  if (old_mode == SelectRectangleMode &&
      Canvas()->HasValidSelectionRectangle()) {
    // Select data within selection rectangle
    if (new_mode >= SelectMapMode && new_mode <= SelectModelPartMode) {
      SelectInsideRectangle(new_mode);
      // Change to back to select rectangle mode
      real_new_mode = SelectRectangleMode;
    }
    // Remove the selection rectangle
    Canvas()->RemoveSelectionRectangle();
  }

  // Select all partitions of the current building if the user switches from
  // SelectMapMode to SelectMapPartitionMode
  if (old_mode == SelectMapMode && new_mode == SelectMapPartitionMode &&
      building_number != -1)
    SelectMapPartitionLines(building_number);
  
  // If something needs to be split, set this up
  if (new_mode == SplitMapMode || new_mode == SplitMapPartitionMode)
    InitialiseSplitLine(new_mode);

  // Set up the extension line
  if (new_mode == ExtendLineMode) {
    if (!InitialiseExtensionLine()) real_new_mode = old_mode;
  }

  // Set up the moving point lines (should probably not be here!!!)
  if (new_mode == MoveNodeMode)
    if (!InitialiseMoveNodeLines()) real_new_mode = old_mode;

  if (new_mode == BrowseMode) {
    if (laser_pyramid.empty()) {
      if (!OpenLaserPyramid()) real_new_mode = old_mode;
    }
    if (!laser_pyramid.empty()) InitialiseBrowseMode();
  }
    
  // Update button and menu states
  for (int mode=0; mode<NumModes; mode++)
    mode_actions[mode]->setChecked(mode == real_new_mode);

  // Inform canvas about the new mode
  Canvas()->SetMode(real_new_mode);
}

void PointCloudMapper::ImportMap()
{
  QString filename;
  filename = QFileDialog::getOpenFileName(this, "Select map point file",
                                          QString::null,
                                      "Map points (*.objpts);;All files (*.*)");
  if (filename.isEmpty()) return;
  SetMapPointFile(filename.toAscii());
  filename = QFileDialog::getOpenFileName(this, "Select map topology file",
                                          QString::null,
                                       "Map topology (*.top);;All files (*.*)");
  if (filename.isEmpty()) return;
  SetMapTopologyFile(filename.toAscii());
  buildings.ImportMap(map_point_file, map_top_file, map_points);
  ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, true, true);
}

void PointCloudMapper::ImportModel()
{
  QString filename;
  filename = QFileDialog::getOpenFileName(this, "Select model point file", 
                                          QString::null,
                                  "Object points (*.objpts*);;All files (*.*)");
  if (filename.isEmpty()) return;
  SetModelPointFile(filename.toAscii());
  filename = QFileDialog::getOpenFileName(this, "Select model topology file",
                                          QString::null,
                                   "Object topology (*.top*);;All files (*.*)");
  if (filename.isEmpty()) return;
  SetModelTopologyFile(filename.toAscii());
  buildings.ImportModel(model_point_file, model_top_file, model_points);
  ShowData(PCMWindowPtr(), ModelData, true, SelectedModelData, true, true);
}

int PointCloudMapper::OpenLaserPyramid()
{
  QString                filename;
  DataBounds2D           bounds;

  // Get the pyramid meta file
  filename = QFileDialog::getOpenFileName(this, "Select laser pyramid file",
                                          QString::null,
                                   QString("Laser pyramid files (*.pyramid)") +
                                   ";;All files (*.*)");
  if (filename.isEmpty()) return 0;
  
  // Read the meta data
  laser_pyramid.ReInitialise(); // Clear old data
  SetLaserPyramidFile(filename.toAscii());
  if (!laser_pyramid.ReadMetaData(filename.toAscii())) {
    printf("Error reading laser pyramid data from file %s\n",
           filename.toAscii().data());
    return 0;
  }
  if (laser_pyramid.empty()) return 0;
  return 1;
}

void PointCloudMapper::ImportLaserPyramid()
{
  // Try to read a laser pyramid
  if (!OpenLaserPyramid()) return;
  // Switch mode to browse
  SetMode(BrowseMode);      
}

void PointCloudMapper::ImportLaserBlock()
{
  QString filename;
  int     fileclass, numpts;

  filename = QFileDialog::getOpenFileName(this, "Select laser (meta) data file",
                                          QString::null,
                                   QString("Laser block meta files (*.block)") +
                                   ";;Laser strip meta files (*.strip)" +
                                   ";;Laser point set meta files (*.pointset)" +
                                   ";;Laser point data files (*.laser*)" +
                                   ";;All files (*.*)");
  if (filename.isEmpty()) return;
  laser_block.ErasePoints(); // Clear old data
  SetLaserMetaFile(filename.toAscii());
  laser_block.Create(filename.toAscii(), &fileclass); // Read meta data
  numpts = laser_block.ReadPoints(1000000); // Read points up to 1.000.000
  if (numpts == -1)
    printf("Block contains more than 1.000.000 laser points.\n");
  else printf("Block with %d laser points read.\n", numpts);
}

void PointCloudMapper::ImportNewLaserPoints()
{
  QString filename;
  filename = QFileDialog::getOpenFileName(this, "Select laser point file", 
                                          QString::null,
                                  QString("Laser point data files (*.laser*)") +
                                          ";;All files (*.*)");
  if (filename.isEmpty()) return;
  SetLaserPointFile(filename.toAscii());
  if (laser_points.size()) laser_points.ErasePoints();
  laser_points.Read(filename.toAscii(), false);
  laser_points.DeriveDataBounds(0);
  if (laser_points.size())
    ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, false, true);
}

void PointCloudMapper::ImportAdditionalLaserPoints()
{
  QString filename;
  LaserPoints additional_laser_points;
  
  filename = QFileDialog::getOpenFileName(this, "Select laser point file", 
                                          QString::null,
                                  QString("Laser point data files (*.laser*)") +
                                          ";;All files (*.*)");
  if (filename.isEmpty()) return;

  additional_laser_points.Read(filename.toAscii(), false);
  laser_points.AddPoints(additional_laser_points);
  additional_laser_points.ErasePoints();
  if (laser_points.size())
    ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, false, true);
}

void PointCloudMapper::ProcessShortCutKeys(QKeyEvent *event)
{
  if (!Canvas()->ShiftKeyDown()) {
    switch (event->key()) {
      case Qt::Key_Minus: // Decrease laser point size
        appearance[LaserData]->PointSizeEditor()->stepDown(); break;
      case Qt::Key_C: // Crop laser data to selected area
        CropLaserData(); break;
      case Qt::Key_D: // Delete laser data in selected area
        DeleteLaserData(); break;
      case Qt::Key_N: // Switch to next project
        SwitchProject(1); break;
      case Qt::Key_P: // Switch to previous project
        SwitchProject(-1); break;
      case Qt::Key_S: // Toggle scale bar display
        switch (canvas->ScaleBarLocation()) {
          case NoScale        : SetScaleBar(LeftBottomScale,
                                            canvas->ScaleBarTickDistance());
                                break;
          case LeftBottomScale: SetScaleBar(CentreScale,
                                            canvas->ScaleBarTickDistance());
                                break;
          default             :
          case CentreScale    : SetScaleBar(NoScale,
                                            canvas->ScaleBarTickDistance());
                                break;
        }
        break;
      case Qt::Key_Comma: // Decrease point colour cycle phase
        appearance[LaserData]->DataAppearance::SetPointColourCyclePhase
          (appearance[LaserData]->PointColourCyclePhase() - 0.1); break;
      case Qt::Key_Period: // Increase point colour cycle phase
        appearance[LaserData]->DataAppearance::SetPointColourCyclePhase
          (appearance[LaserData]->PointColourCyclePhase() + 0.1); break;
      default:
        break;
    }
  }
  else { // With shift key down
    switch (event->key()) {
      case Qt::Key_Plus: // Increase laser point size
        appearance[LaserData]->PointSizeEditor()->stepUp(); break;
      case Qt::Key_Less: // Decrease point colour cycle length
        appearance[LaserData]->DataAppearance::SetPointColourCycleLength
          (appearance[LaserData]->PointColourCycleLength() / 1.5); break;
      case Qt::Key_Greater: // Increase point colour cycle length
        appearance[LaserData]->DataAppearance::SetPointColourCycleLength
          (appearance[LaserData]->PointColourCycleLength() * 1.5); break;
      default:
        break;
    }
  }
}

void PointCloudMapper::InitialiseSplitLine(MouseMode mode)
{
  LineTopologies::iterator line;

  switch (mode) {
    case SplitMapMode:
      if (selected_map_data.empty()) {
        QMessageBox::information(this, "Error",
                                 "No building line selected.");
        return;
      }
      else if (selected_map_data.size() > 1) {
        QMessageBox::information(this, "Error",
                                 "Multiple building lines selected.");
        return;
      }
      line = *(selected_map_data.begin());
      break;
    case SplitMapPartitionMode:
      if (selected_map_data.size()) {
        if (selected_map_data.size() > 1) {
          QMessageBox::information(this, "Error",
                                   "Multiple building lines selected.");
          return;
        }
        line = *(selected_map_data.begin());
      }
      else if (selected_map_part_data.size()) {
        if (selected_map_part_data.size() > 1) {
          QMessageBox::information(this, "Error",
                              "Multiple building partition lines selected.");
          return;
        }
        line = *(selected_map_part_data.begin());
      }
      else {
        QMessageBox::information(this, "Error",
                                 "No building (partition) line selected.");
        return;
      }
      break;
    default:
      printf("Invalid mode %d in PointCloudMapper::InitialiseSplitLine\n",
             mode);
      return;
  }

  Canvas()->InitialiseSplitLine(&map_points, line);
}

void PointCloudMapper::SplitOutline(MouseMode mode,
                                    const LineSegment2D &split_line)
{
  LineTopologies::iterator outline;
  LineTopologies           new_lines;
  Building                 *building;
  int                      building_number;

  // Determine the map (partition) outline to be split
  if (selected_map_data.size()) outline = *(selected_map_data.begin());
  else outline = *(selected_map_part_data.begin());

  // Get the building
  building_number = outline->Number() / 1000;
  building = buildings.BuildingPtr(building_number);
  if (!building) {
    printf("Error locating building with number %d\n", outline->Number()/1000);
    return;
  }

  // Split the outline
  outline->SplitPolygonByLineSegment(map_points, split_line, 0.01, new_lines);

  // Update the building structure
  switch (mode) {
    case SplitMapMode: // Split a building into two new buildings
      // Check if there is any partition data and/or model data
      if (building->ContainsData(MapPartitionData) ||
          building->ContainsData(ModelData)) {
        if (QMessageBox::information(this, "Warning",
            "This building contains map partition data and/or building data.\n"
            "This data will be lost when splitting this building.\n"
            "Do you want to continue?", "Yes", "No") == 1) return;
        building->DeleteData(MapPartitionData);
        building->DeleteData(ModelData);
      }
      // Remove current building from canvas
      Canvas()->RemoveObjectData(building->MapDataPtr(), false);
      // Split the building
      buildings.SplitBuilding(building_number, new_lines);
      // Add the two new map lines to canvas
      Canvas()->AddObjectData(&map_points, building->MapDataPtr(),
                               appearance[MapData]->DataAppearancePtr(), false);
      Canvas()->AddObjectData(&map_points, (buildings.end()-1)->MapDataPtr(),
                               appearance[MapData]->DataAppearancePtr(), true);
      // Delete the selection
      selected_map_data.Clear();
      nearby_map_data.Clear();
      // Switch to map line selection mode
      SetMode(SelectMapMode);
      break;
    case SplitMapPartitionMode:
      // Split building into two building partitions
      if (selected_map_data.size()) {
        // Undo the selection of the building outline
        UpdateSelection(selected_map_data, selected_map_data,
                        selected_map_data.begin(), &map_points,
                        MapData, false, false, false);
        nearby_map_data.Clear();
        // Add the partition outlines
        building->AddMapPartitionData(new_lines[0]);
        building->AddMapPartitionData(new_lines[1]);
        // Show the map partition data
        ShowData(PCMWindowPtr(), MapPartitionData, true,
                 SelectedMapPartitionData, true, true, true);
        // Switch to map line selection mode
        SetMode(SelectMapMode);
      }
      // Split building partition into two partitions
      else {
        // Delete the selection
        selected_map_part_data.Clear();
        nearby_map_part_data.Clear();
        // Split the building partition
        building->SplitMapPartition(outline, new_lines);
        // Show the map partition data
        ShowData(PCMWindowPtr(), MapPartitionData, true,
                 SelectedMapPartitionData, true, true, true);
        // Switch to map partition selection mode
        SetMode(SelectMapPartitionMode);
      }
      break;
    default:
      break;
  }
}

bool PointCloudMapper::InitialiseExtensionLine()
{
  LineTopsIterVector::iterator selected_line;

  // Check if something is selected
  if (selected_map_data.size())
    selected_line = selected_map_data.begin();
  else if (selected_map_part_data.size())
    selected_line = selected_map_part_data.begin();
  else {
    QMessageBox::information(this, "Error",
                             "No building (partition) line selected.");
    return false;
  }

  // Check if the line is open
  if ((*selected_line)->IsClosed()) {
    QMessageBox::information(this, "Error",
                             "The selected line is a closed polygon.\n");
    return false;
  }

  // Pass data to the canvas
  Canvas()->InitialiseExtensionLine(&map_points, selected_line);
  return true;
}

bool PointCloudMapper::InitialiseMoveNodeLines()
{
  LineTopsIterVector::iterator selected_line;

  // Check if something is selected
  if (selected_map_data.size()) selected_line = selected_map_data.begin();
  else if (selected_map_part_data.size())
    selected_line = selected_map_part_data.begin();
  else {
    QMessageBox::information(this, "Error",
                             "No building (partition) line selected.");
    return false;
  }

  // Pass data to the canvas
  Canvas()->InitialiseMovingPointLines(&map_points, selected_line);
  return true;
}

void PointCloudMapper::RemovePCMWindow(PCMWindow *closed_window)
{
  vector <PCMWindow *>::iterator window;

  for (window=subwindows.begin(); window!=subwindows.end(); window++)
    if (*window == closed_window) {
      subwindows.erase(window);
      return;
    }
  printf("Did not find window in PCMWindow list, bug?\n");
}

void PointCloudMapper::AddShowDataConnectors(PCMWindow *window)
{
  // Redraw canvas if the appearance of any data type changed
  for (int j=0; j<NumDataTypes; j++)
    connect(appearance[j], SIGNAL(ChangedSetting()),
            window->Canvas(), SLOT(updateGL()));
  if (window->Type() == PCMResult)
    connect(result_appearance, SIGNAL(ChangedSetting()),
            window->Canvas(), SLOT(updateGL()));

  // Add data to a canvas on request of a PCMWindow
  connect(window, SIGNAL(RequestDataDisplay(DataType, PCMWindow *)),
          this, SLOT(DisplayData(DataType, PCMWindow *)));

  // Add background image to a canvas on request of a PCMWindow
  connect(window, SIGNAL(RequestImageDisplay(BackGroundType, PCMWindow *)),
          this, SLOT(SetBackGroundImage(BackGroundType, PCMWindow *)));
}

void PointCloudMapper::SetScaleBar(PCMScaleLocation new_location, 
                                   double new_distance, bool update)
{
  int i, item;
  
  for (i=0; i<3; i++)
    show_scale_actions[i]->setChecked(new_location == (PCMScaleLocation) i);
  item = (int) (log(new_distance) / log(10.0) + 2.01);
  for (i=0; i<5; i++)
    set_scale_tick_actions[i]->setChecked(i == item);
  canvas->SetScaleLocation(new_location);
  canvas->SetScaleTickDistance(new_distance);
  if (update) canvas->update();
}

void PointCloudMapper::InitialiseBrowseMode()
{
  DataBounds2D bounds;
  LaserPyramid::iterator top_level;
  LaserUnit::iterator    top_tile;
  
  // Reset view to vertical
  Canvas()->LevelView();
  
  // Get bounds from canvas if some data is there
  if (Canvas()->HasData()) bounds = Canvas()->BoundsXY();
  // Otherwise, get bounds from top pyramid level
  else {
    top_level = laser_pyramid.end() - 1;
    top_tile  = top_level->begin()->begin();
    bounds.SetMinimumX(top_tile->TileBounds().Minimum().X());
    bounds.SetMaximumX(top_tile->TileBounds().Maximum().X());
    bounds.SetMinimumY(top_tile->TileBounds().Minimum().Y());
    bounds.SetMaximumY(top_tile->TileBounds().Maximum().Y());
  }
  
  // Load new pyramid data
  UpdatePyramidSelection(bounds);

  // Add laser data to canvas
  ShowData(PCMWindowPtr(), LaserData, true,
           SelectedLaserData, false, true, true);
}

void PointCloudMapper::UpdatePyramidSelection(const DataBounds2D &bounds)
{
  double max_point_spacing;

  // Convert the point spacing on screen to point spacing in terrain
  max_point_spacing = max_point_spacing_pixels * bounds.XRange() /
                      Canvas()->width();

  // Load all laser data of the highest pyramid level
  laser_pyramid.UpdateSelection(bounds, max_point_spacing, laser_points,
                                1000000); 
}

void PointCloudMapper::QuitPCM()
{
  WriteDefaults("pcm.ini");
  qApp->closeAllWindows();
  qApp->quit();
}
