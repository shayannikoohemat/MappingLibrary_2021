
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

PointCloudMapper::PointCloudMapper(QApplication *app)
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
  local_ground_height = 0.0;
  project_number_increment = 1;

////////////////////////////////////////////////////////////////////////////////
// Toolbar and menu for file operations

  QToolBar *file_tools = addToolBar("File operations");
  QMenu    *file_menu  = menuBar()->addMenu(tr("&File"));
#ifdef linux
  file_tools->setIconSize(QSize(18, 18));
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
  
  // Save roof verification data
  action = new QAction("Roof verification data", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveRoofRecords()));
  save_sub_menu->addAction(action);
  
  // Save roof verification data as
  action = new QAction("Roof verification data as...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveRoofRecordsAs()));
  save_sub_menu->addAction(action);
  
  // Import sub menu
  QMenu *import_sub_menu = file_menu->addMenu("Import");

  // Import new map data
  action = new QAction("Map...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ImportMapData()));
  import_sub_menu->addAction(action);
  
  // Import PCM map data
  action = new QAction("PCM map...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ImportPCMMapData()));
  import_sub_menu->addAction(action);
  
  // Import PCM model data
  action = new QAction("PCM model...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ImportPCMModelData()));
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
  
  // Import parameter settings
  action = new QAction("Parameter settings...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(LoadSettings()));
  import_sub_menu->addAction(action);
  
  // Import roof verification data
  action = new QAction("Roof verification data...", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SelectRoofVerificationFile()));
  import_sub_menu->addAction(action);
  
  // Close and quit programme
  action = new QAction("&Quit", this);
  action->setShortcut(Qt::CTRL + Qt::Key_Q);
  connect(action, SIGNAL(triggered()), this, SLOT(QuitPCM()));
  file_menu->addAction(action);

////////////////////////////////////////////////////////////////////////////////
// Toolbar and menu for edit operations

  QToolBar *mode_tools = new QToolBar("Mouse modes", this);
  addToolBar(Qt::LeftToolBarArea, mode_tools);
  QToolBar *edit_tools = new QToolBar("Edit operations", this);
  addToolBar(Qt::BottomToolBarArea, edit_tools);
  QMenu    *edit_menu  = menuBar()->addMenu(tr("&Edit"));
#ifdef linux
  mode_tools->setIconSize(QSize(18, 18));
  edit_tools->setIconSize(QSize(18, 18));
#else
  mode_tools->setIconSize(QSize(18, 18));
  edit_tools->setIconSize(QSize(18, 18));
#endif

  // Mouse mode sub menu
  QMenu *mode_sub_menu = edit_menu->addMenu("Mouse mode");

  // Mouse mode pose change
  action = mode_actions[PoseChangeMode] =
    new QAction(QIcon(":/buttons/posechangemode.xpm"), "Pose change orthogonal", this);
  action->setStatusTip(tr("Pose change orthogonal"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetPoseChangeMode()));
  mode_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode pose change in perspective projection
  action = mode_actions[PoseChangePerspectiveMode] =
    new QAction(QIcon(":/buttons/posechangeperspectivemode.xpm"), "Pose change perspective", this);
  action->setStatusTip(tr("Pose change perspective"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetPoseChangePerspectiveMode()));
  mode_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode browse
  action = mode_actions[BrowseMode] =
    new QAction(QIcon(":/buttons/browsemode.xpm"), "Browse", this);
  action->setStatusTip(tr("Browse laser pyramid"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetBrowseMode()));
  mode_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode select map line
  action = mode_actions[SelectMapMode] =
    new QAction(QIcon(":/buttons/selectmap.xpm"), "Select map line", this);
  action->setStatusTip(tr("Select map line"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectMapMode()));
  mode_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode select map partition line
  action = mode_actions[SelectMapPartitionMode] =
    new QAction(QIcon(":/buttons/selectmappart.xpm"),
                "Select map partition line", this);
  action->setStatusTip(tr("Select map partition line"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectMapPartitionMode()));
  mode_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode select model
  action = mode_actions[SelectModelMode] =
    new QAction(QIcon(":/buttons/selectmodel.xpm"), "Select model", this);
  action->setStatusTip(tr("Select model"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectModelMode()));
  mode_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode select model part
  action = mode_actions[SelectModelPartMode] =
    new QAction(QIcon(":/buttons/selectmodelpart.xpm"), "Select model part",
                this);
  action->setStatusTip(tr("Select model part"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectModelPartMode()));
  mode_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode select model face
  action = mode_actions[SelectModelFaceMode] =
    new QAction(QIcon(":/buttons/selectmodelface.xpm"), "Select model face",
                this);
  action->setStatusTip(tr("Select model face"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectModelFaceMode()));
  mode_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode select laser segment
  action = mode_actions[SelectLaserPointMode] =
    new QAction(QIcon(":/buttons/selectpoint.xpm"), "Select laser point",
                this);
  action->setStatusTip(tr("Select laser point"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectLaserPointMode()));
  mode_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode select laser segment
  action = mode_actions[SelectLaserSegmentMode] =
    new QAction(QIcon(":/buttons/selectsegment.xpm"), "Select laser segment",
                this);
  action->setStatusTip(tr("Select laser segment"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectLaserSegmentMode()));
  mode_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode select rectangular area on canvas
  action = mode_actions[SelectRectangleMode] =
    new QAction(QIcon(":/buttons/selectrect.xpm"), "Select area on canvas",
                this);
  action->setStatusTip(tr("Select area on canvas"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectRectangleMode()));
  mode_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode split map polygon
  action = mode_actions[SplitMapMode] =
    new QAction(QIcon(":/buttons/splitmap.xpm"), "Split map line", this);
  action->setStatusTip(tr("Split map line"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSplitMapMode()));
  mode_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode split map partition polygon
  action = mode_actions[SplitMapPartitionMode] =
    new QAction(QIcon(":/buttons/splitmappart.xpm"), "Select map partition",
                this);
  action->setStatusTip(tr("Split map partition"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSplitMapPartitionMode()));
  mode_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode extend line
  action = mode_actions[ExtendLineMode] =
    new QAction(QIcon(":/buttons/extendline.xpm"), "Extend line", this);
  action->setStatusTip(tr("Extend line"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetExtendLineMode()));
  mode_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Mouse mode move point
  action = mode_actions[MoveNodeMode] =
    new QAction(QIcon(":/buttons/movenode.xpm"), "Move point", this);
  action->setStatusTip(tr("Move point"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetMoveNodeMode()));
  mode_tools->addAction(action);
  mode_sub_menu->addAction(action);

  // Make all above buttons toggle buttons and switch them all off
  for (int mode=0; mode<NumModes; mode++) {
    mode_actions[mode]->setCheckable(true);
    mode_actions[mode]->setChecked(false);
  }

  // Start mode is pose change TODO: move to end, not related to gui
  SetMode(PoseChangeMode);

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
  //Remove points to be loaded from existing set
 action = new QAction("Load points to be removed",this);
 connect(action, SIGNAL(triggered()), this, SLOT(RemoveLoadedLaserPointsFromSet()));
 edit_menu->addAction(action);

  // Select laser data
  action = new QAction(QIcon(":/buttons/selectlaser.xpm"), "Select laser data",
                       this);
  action->setStatusTip(tr("Select laser data"));
  connect(action, SIGNAL(triggered()), this, SLOT(SelectLaserData()));
  edit_tools->addAction(action);
//  edit_menu->addAction(action);  // Moved to Laser data menu

  // Delete laser data
  action = new QAction(QIcon(":/buttons/deletelaser.xpm"), "Delete selected laser data",
                       this);
  action->setStatusTip(tr("Delete selected laser data"));
  connect(action, SIGNAL(triggered()), this, SLOT(DeleteSelectedLaserData()));
  edit_tools->addAction(action);
//  edit_menu->addAction(action);  // Moved to Laser data menu

  // Crop laser data
  action = new QAction(QIcon(":/buttons/croplaser.xpm"), "Crop to selected laser data",
                       this);
  action->setStatusTip(tr("Crop to selected laser data"));
  connect(action, SIGNAL(triggered()), this, SLOT(CropToSelectedLaserData()));
  edit_tools->addAction(action);
//  edit_menu->addAction(action);  // Moved to Laser data menu

  // End of laser edit buttons on edit toolbar
  edit_tools->addSeparator();

//------------------------------------------------------------------------------
// Line edit stuff

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

  // End of line edit buttons on edit toolbar
  edit_tools->addSeparator();

//------------------------------------------------------------------------------
// Map specific line edit stuff

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

  // End of map specific line edit buttons on edit toolbar
  edit_tools->addSeparator();

//------------------------------------------------------------------------------
// Model specific line edit stuff

  // Edit model of selected map or building data
  edit_model_action = action =  new QAction(QIcon(":/buttons/editmodel.xpm"),
                                            "Edit building model", this);
  action->setStatusTip(tr("Edit building model"));
  connect(action, SIGNAL(triggered()), this, SLOT(EditBuildingModel()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);
  action->setCheckable(true);
  action->setChecked(false);

  // Reconstruct a building corner
  action =  new QAction(QIcon(":/buttons/reconstructcorner.xpm"),
                        "Reconstruct roof corner", this);
  action->setStatusTip(tr("Reconstruct roof corner"));
  connect(action, SIGNAL(triggered()), this, SLOT(ReconstructRoofCorner()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);
  
  // Reconstruct model edge by intersecting two segments
  action =  new QAction(QIcon(":/buttons/intersectsegments.xpm"),
                        "Intersect segments", this);
  action->setStatusTip(tr("Intersect segments"));
  connect(action, SIGNAL(triggered()), this, SLOT(Reconstruct3DLineSegment()));
  edit_tools->addAction(action);
  edit_menu->addAction(action);
  
  // Reconstruct building walls
  action =  new QAction(QIcon(":/buttons/reconstructwalls.xpm"),
                        "Reconstruct building walls", this);
  action->setStatusTip(tr("Reconstruct building walls"));
  connect(action, SIGNAL(triggered()), this, SLOT(ReconstructWalls()));
  edit_tools->addAction(action);
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

  // Show selected model face data
  action = show_selection_actions[SelectedModelFaceData] =
    new QAction("Selected model face data", this);
  action->setCheckable(true);
  connect(action, SIGNAL(triggered()), this, SLOT(ShowSelectedModelFaceData()));
  show_menu->addAction(action);

  // Show selected laser data
  action = show_selection_actions[SelectedLaserData] =
    new QAction(QIcon(":/buttons/showselectedlaser.xpm"),
                "Selected laser data", this);
  action->setCheckable(true);
  action->setChecked(false);
  connect(action, SIGNAL(triggered()), this, SLOT(ShowSelectedLaserData()));
  show_menu->addAction(action);
  show_tools->addAction(action);

  // Show selected point
  action = show_selection_actions[SelectedPointData] =
    new QAction("Selected point", this);
  action->setCheckable(true);
  action->setChecked(false);
  connect(action, SIGNAL(triggered()), this, SLOT(ToggleShowSelectedPoint()));
  show_menu->addAction(action);

  // Show tile boundaries
  action = show_selection_actions[TileBoundaryData] =
    new QAction("Tile boundaries", this);
  action->setCheckable(true);
  action->setChecked(false);
  connect(action, SIGNAL(triggered()), this, SLOT(ToggleShowTileBoundaries()));
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
  action = set_scale_tick_actions[0] = new QAction("0.001 m", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetScaleTick1mm()));
  scale_tick_menu->addAction(action);
  
  action = set_scale_tick_actions[1] = new QAction("0.01 m", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetScaleTick1cm()));
  scale_tick_menu->addAction(action);
  
  action = set_scale_tick_actions[2] = new QAction("0.1 m", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetScaleTick10cm()));
  scale_tick_menu->addAction(action);
  
  action = set_scale_tick_actions[3] = new QAction("1 m", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetScaleTick1m()));
  scale_tick_menu->addAction(action);
  
  action = set_scale_tick_actions[4] = new QAction("10 m", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetScaleTick10m()));
  scale_tick_menu->addAction(action);
  
  action = set_scale_tick_actions[5] = new QAction("100 m", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetScaleTick100m()));
  scale_tick_menu->addAction(action);  

  action = set_scale_tick_actions[6] = new QAction("1000 m", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SetScaleTick1km()));
  scale_tick_menu->addAction(action);  

  // Set defaults
  for (i=0; i<3; i++) show_scale_actions[i]->setCheckable(true);
  for (i=0; i<7; i++) set_scale_tick_actions[i]->setCheckable(true);
  SetScaleBar(NoScale, 1.0, false);
  
//------------------------------------------------------------------------------
// Switch display of normal data off and display of selected data on

  for (i=0; i<NumNormalDataTypes; i++)
    show_data_actions[i]->setChecked(false);
  for (i=SelectedMapData; i<=SelectedModelFaceData; i++)
    show_selection_actions[i]->setChecked(true);


////////////////////////////////////////////////////////////////////////////////
// Toolbar for spawning additional windows

  QToolBar *spawn_tools = new QToolBar(tr("Window operations"), this);
  spawn_tools->setObjectName("Window tools");
#ifdef linux
  spawn_tools->setIconSize(QSize(18, 18));
#else
  spawn_tools->setIconSize(QSize(18, 18));
#endif
  addToolBar(Qt::RightToolBarArea, spawn_tools);

//------------------------------------------------------------------------------
// Spawn a window with copied data

  action = new QAction(QIcon(":/buttons/newwindow.xpm"),
                       "Spawn window with copied data", this);
  action->setStatusTip(tr("Spawn window with copied data"));
  connect(action, SIGNAL(triggered()), this, SLOT(SpawnCopiedDataWindow()));
  spawn_tools->addAction(action);

//------------------------------------------------------------------------------
// Spawn a window with the same data
  action = new QAction(QIcon(":/buttons/newviewwindow.xpm"),
                       "Spawn window with same data", this);
  action->setStatusTip(tr("Spawn window with same data"));
  connect(action, SIGNAL(triggered()), this, SLOT(SpawnSameDataWindow()));
  spawn_tools->addAction(action);

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
  
  // Reset view when loading new point file
  action = reset_on_loading_points =
    new QAction("Reset view when loading new points", this);
  action->setCheckable(true);
  action->setChecked(false);
  connect(action, SIGNAL(triggered()), this, SLOT(ToggleResetOnLoading()));
  view_menu->addAction(action);

////////////////////////////////////////////////////////////////////////////////
// Menu for data operations

  QMenu *data_menu  = menuBar()->addMenu(tr("Laser data"));

  // Copy all points to view window
  action = new QAction("Copy all points", this);
  connect(action, SIGNAL(triggered()), this, SLOT(CopyAllPoints()));
  data_menu->addAction(action);
  
  // Copy selected points to view window
  action = new QAction("Copy selected points", this);
  connect(action, SIGNAL(triggered()), this, SLOT(CopySelectedPoints()));
  data_menu->addAction(action);
  
  // Copy segmented points to view window
  action = new QAction("Copy segmented points", this);
  connect(action, SIGNAL(triggered()), this, SLOT(CopySegmentedPoints()));
  data_menu->addAction(action);
  
  // Copy unsegmented points to view window
  action = new QAction("Copy unsegmented points", this);
  connect(action, SIGNAL(triggered()), this, SLOT(CopyUnsegmentedPoints()));
  data_menu->addAction(action);

  // Delete all points
  action = new QAction("Delete all points", this);
  connect(action, SIGNAL(triggered()), this, SLOT(DeleteAllPoints()));
  data_menu->addAction(action);
  
  // Delete selected points
  action = new QAction("Delete selected points", this);
  connect(action, SIGNAL(triggered()), this, SLOT(DeleteSelectedPoints()));
  data_menu->addAction(action);
  
  // Delete segmented points
  action = new QAction("Delete segmented points", this);
  connect(action, SIGNAL(triggered()), this, SLOT(DeleteSegmentedPoints()));
  data_menu->addAction(action);
  
  // Crop to selected points
  action = new QAction("Crop to selected points", this);
  connect(action, SIGNAL(triggered()), this, SLOT(CropToSelectedLaserData()));
  data_menu->addAction(action);

  // Crop to segmented points
  action = new QAction("Crop to segmented points", this);
  connect(action, SIGNAL(triggered()), this, SLOT(DeleteUnsegmentedPoints()));
  data_menu->addAction(action);
  
////////////////////////////////////////////////////////////////////////////////
// Menu for segmentation operations

  QMenu *segmentation_menu  = menuBar()->addMenu(tr("Segmentation"));

//------------------------------------------------------------------------------
// Segmentation parameter editing sub menu
  segmentation_parameters = new SegmentationParameters();

  QMenu *segmentation_parameter_sub_menu =
    segmentation_menu->addMenu("Edit segmentation parameters");

  // General segmentation parameters
  general_segmentation_parameters =
    new GeneralSegmentationParametersWindow(segmentation_parameters, NULL);
  general_segmentation_parameters->setWindowTitle("General segmentation parameters");
  action = new QAction("General segmentation parameters", this);
  connect(action, SIGNAL(triggered()), general_segmentation_parameters, SLOT(show()));
  segmentation_parameter_sub_menu->addAction(action);

  // Connected component parameters
  connected_component_parameters =
    new ConnectedComponentParametersWindow(segmentation_parameters, NULL);
  connected_component_parameters->setWindowTitle("Connected component parameters");
  action = new QAction("Connected component parameters", this);
  connect(action, SIGNAL(triggered()), connected_component_parameters, SLOT(show()));
  segmentation_parameter_sub_menu->addAction(action);

  // Surface growing parameters
  surface_growing_parameters =
    new SurfaceGrowingParametersWindow(segmentation_parameters, NULL);
  surface_growing_parameters->setWindowTitle("Surface growing parameters");
  action = new QAction("Surface growing parameters", this);
  connect(action, SIGNAL(triggered()), surface_growing_parameters, SLOT(show()));
  segmentation_parameter_sub_menu->addAction(action);

  // Segment growing parameters
  segment_growing_parameters =
    new SegmentGrowingParametersWindow(segmentation_parameters, NULL);
  segment_growing_parameters->setWindowTitle("Segment growing parameters");
  action = new QAction("Segment growing parameters", this);
  connect(action, SIGNAL(triggered()), segment_growing_parameters, SLOT(show()));
  segmentation_parameter_sub_menu->addAction(action);

  // Mean shift parameters
  mean_shift_parameters =
    new MeanShiftParametersWindow(segmentation_parameters, NULL);
  mean_shift_parameters->setWindowTitle("Mean shift parameters");
  action = new QAction("Mean shift parameters", this);
  connect(action, SIGNAL(triggered()), mean_shift_parameters, SLOT(show()));
  segmentation_parameter_sub_menu->addAction(action);

  // Majority filtering parameters
  majority_filtering_parameters =
    new MajorityFilteringParametersWindow(segmentation_parameters, NULL);
  majority_filtering_parameters->setWindowTitle("Majority filtering parameters");
  action = new QAction("Majority filtering parameters", this);
  connect(action, SIGNAL(triggered()), majority_filtering_parameters, SLOT(show()));
  segmentation_parameter_sub_menu->addAction(action);

  // Segment merging parameters
  surface_merging_parameters =
    new SurfaceMergingParametersWindow(segmentation_parameters, NULL);
  surface_merging_parameters->setWindowTitle("Surface merging parameters");
  action = new QAction("Surface merging parameters", this);
  connect(action, SIGNAL(triggered()), surface_merging_parameters, SLOT(show()));
  segmentation_parameter_sub_menu->addAction(action);

//------------------------------------------------------------------------------
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

  // Select points or segments
  select_window = new SelectWindow(&laser_points,
                                   &selected_laser_points, 
                                   segmentation_parameters, NULL, NULL);
  action = new QAction("Select points or segments...", this);
  connect(action, SIGNAL(triggered()),
          select_window, SLOT(show()));
  segmentation_menu->addAction(action);
  connect(select_window, SIGNAL(RequestDisplayUpdate()),
          this, SLOT(UpdateLaserDataDisplay()));
          
  // Remove small segments
  action = new QAction("Remove small segments", this);
  connect(action, SIGNAL(triggered()), this, SLOT(RemoveSmallSegments()));
  segmentation_menu->addAction(action);

  // Unlabel small segments
  action = new QAction("Unlabel small segments", this);
  connect(action, SIGNAL(triggered()), this, SLOT(UnlabelSmallSegments()));
  segmentation_menu->addAction(action);

  // Mean shift segmentation
  action = new QAction("Mean shift segmentation", this);
  connect(action, SIGNAL(triggered()), this, SLOT(MeanShiftSegmentation()));
  segmentation_menu->addAction(action);

  // Segment growing
  action = new QAction("Segment growing", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SegmentGrowing()));
  segmentation_menu->addAction(action);

  // Majority filtering
  action = new QAction("Majority filtering", this);
  connect(action, SIGNAL(triggered()), this, SLOT(MajorityFilterSegmentation()));
  segmentation_menu->addAction(action);

  // Mean shift segmentation
  action = new QAction("Merge surfaces", this);
  connect(action, SIGNAL(triggered()), this, SLOT(MergeSurfaces()));
  segmentation_menu->addAction(action);

//------------------------------------------------------------------------------
// Laser tag selection sub menu
  QMenu *laser_tag_sub_menu = segmentation_menu->addMenu("Select points by");

  action = set_laser_tag_actions[0] = new QAction("Label", this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(SetLaserSelectionTagLabel()));
  laser_tag_sub_menu->addAction(action);

  action = set_laser_tag_actions[1] = new QAction("Segment number", this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(SetLaserSelectionTagSegmentNumber()));
  laser_tag_sub_menu->addAction(action);

  action = set_laser_tag_actions[2] = new QAction("Plane number", this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(SetLaserSelectionTagPlaneNumber()));
  laser_tag_sub_menu->addAction(action);

  action = set_laser_tag_actions[3] = new QAction("Scan number", this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(SetLaserSelectionTagScanNumber()));
  laser_tag_sub_menu->addAction(action);

  action = set_laser_tag_actions[7] = new QAction("Scan number (removed AFN)", this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(SetLaserSelectionTagScanNumberWithoutAFN()));
  laser_tag_sub_menu->addAction(action);

  action = set_laser_tag_actions[8] = new QAction("AFN code", this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(SetLaserSelectionTagAFNCode()));
  laser_tag_sub_menu->addAction(action);

  action = set_laser_tag_actions[4] = new QAction("Filter status", this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(SetLaserSelectionTagIsFiltered()));
  laser_tag_sub_menu->addAction(action);

  action = set_laser_tag_actions[5] = new QAction("Pulse count", this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(SetLaserSelectionTagPulseCount()));
  laser_tag_sub_menu->addAction(action);

  action = set_laser_tag_actions[6] = new QAction("Component number", this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(SetLaserSelectionTagComponentNumber()));
  laser_tag_sub_menu->addAction(action);

  action = set_laser_tag_actions[9] = new QAction("Segment+Tile number", this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(SetLaserSelectionTagSegmentAndTileNumber()));
  laser_tag_sub_menu->addAction(action);

  action = set_laser_tag_actions[10] = new QAction("Scan line number", this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(SetLaserSelectionTagScanLineNumber()));
  laser_tag_sub_menu->addAction(action);

  // Set defaults
  for (i=0; i<11; i++) set_laser_tag_actions[i]->setCheckable(true);
  SetLaserSelectionTag(1);

////////////////////////////////////////////////////////////////////////////////
// Toolbar and menu for fitting operations

  QToolBar *fitting_tools = new QToolBar("Fitting operations", this);
  QMenu    *fitting_menu  = menuBar()->addMenu(tr("&Fitting"));
  addToolBarBreak(Qt::BottomToolBarArea);
  addToolBar(Qt::BottomToolBarArea, fitting_tools);
#ifdef linux
  fitting_tools->setIconSize(QSize(18, 18));
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

  // Fit cone roof
  action = new QAction(QIcon(":/buttons/coneroof.xpm"), "Fit cone roof",
                       this);
  action->setStatusTip(tr("Fit cone roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitConeRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Fit cylinder roof
  action = new QAction(QIcon(":/buttons/cylinderroof.xpm"), "Fit cylinder roof",
                       this);
  action->setStatusTip(tr("Fit cylinder roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitCylinderRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Fit rotated cylinder roof
  action = new QAction(QIcon(":/buttons/cylinderroofrot.xpm"),
                       "Fit rotated cylinder roof", this);
  action->setStatusTip(tr("Fit rotated cylinder roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitRotatedCylinderRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Automatic roof reconstruction
  action = new QAction(QIcon(":/buttons/autoroof.xpm"),
                       "Automatic reconstruction", this);
  action->setStatusTip(tr("Automatic reconstruction"));
  connect(action, SIGNAL(triggered()), this, SLOT(ReconstructRoof()));
  fitting_tools->addAction(action);
  fitting_menu->addAction(action);

  // Set wall bottom height
  action = new QAction("Determine local ground height", this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(SetLowestPointForWallReconstruction()));
  fitting_menu->addAction(action);

  // Edit fitting parameters
  fitting_parameters = new FittingParametersWindow(NULL);
  fitting_parameters->setWindowTitle("Fitting parameters");
  action = new QAction("Edit fitting parameters", this);
  connect(action, SIGNAL(triggered()), fitting_parameters, SLOT(show()));
  fitting_menu->addAction(action);

////////////////////////////////////////////////////////////////////////////////
// Menu for outlining operations
// Buttons are added to the fitting toolbar

  QMenu    *outlining_menu  = menuBar()->addMenu(tr("&Outlining"));

  // Fit minimum bounding rectangle to laser points
  action = new QAction(QIcon(":/buttons/maprectangle.xpm"),
                       "Fit rectangular map line", this);
  action->setStatusTip(tr("Fit rectangular map line"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitRectangularMapLine()));
  fitting_tools->addAction(action);
  outlining_menu->addAction(action);

  // Fit polygonal map line to laser points
  action = new QAction(QIcon(":/buttons/mappolygon.xpm"),
                       "Fit polygonal map line", this);
  action->setStatusTip(tr("Fit polygonal map line"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitPolygonalMapLine()));
  fitting_tools->addAction(action);
  outlining_menu->addAction(action);

  // Determine smallest enclosing circle of laser points
  action = new QAction(QIcon(":/buttons/mapcircle.xpm"),
                       "Fit circular map line", this);
  action->setStatusTip(tr("Fit circular map line"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitCircularMapLine()));
  fitting_tools->addAction(action);
  outlining_menu->addAction(action);

  // Edit outlining parameters
  outlining_parameters = new OutliningParametersWindow(NULL);
  outlining_parameters->setWindowTitle("Outlining parameters");
  action = new QAction("Edit outlining parameters", this);
  connect(action, SIGNAL(triggered()), outlining_parameters, SLOT(show()));
  outlining_menu->addAction(action);

////////////////////////////////////////////////////////////////////////////////
// Menu for filtering operations

  QMenu    *filtering_menu  = menuBar()->addMenu(tr("Filtering"));

  // Slope based filtering
  action = new QAction("Slope based filtering", this);
  connect(action, SIGNAL(triggered()), this, SLOT(FilterLaserData()));
  filtering_menu->addAction(action);

  // Edit filtering parameters
  filtering_parameters = new FilteringParametersWindow(NULL);
  filtering_parameters->setWindowTitle("Filtering parameters");
  action = new QAction("Edit filtering parameters", this);
  connect(action, SIGNAL(triggered()), filtering_parameters, SLOT(show()));
  filtering_menu->addAction(action);

////////////////////////////////////////////////////////////////////////////////
// Toolbar for roof verification operations

  roof_verification_tools = addToolBar("Roof verification operations");
  roof_verification_tools->hide();
  roof_verification_tools->setIconSize(QSize(18, 18));
  // Set roof status to approved
  action = roof_verification_button[0] =
    new QAction(QIcon(":/buttons/acceptmodel.xpm"), "&Accepted roof", this);
  action->setStatusTip(tr("Accepted roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(RoofAccepted()));
  action->setCheckable(true);
  roof_verification_tools->addAction(action);

  // Set roof status to undecided
  action = roof_verification_button[1] =
    new QAction(QIcon(":/buttons/undecided.xpm"), "&Undecided roof", this);
  action->setStatusTip(tr("Undecided roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(RoofUndecided()));
  action->setCheckable(true);
  roof_verification_tools->addAction(action);

  // Set roof status to rejected
  action = roof_verification_button[2] =
    new QAction(QIcon(":/buttons/rejectmodel.xpm"), "&Rejected roof", this);
  action->setStatusTip(tr("Rejected roof"));
  connect(action, SIGNAL(triggered()), this, SLOT(RoofRejected()));
  action->setCheckable(true);
  roof_verification_tools->addAction(action);


////////////////////////////////////////////////////////////////////////////////
// Menu for help operations

  QMenu *help_menu  = menuBar()->addMenu(tr("&Help"));

  // About PCM
  action = new QAction("&About PCM", this);
  action->setShortcut(Qt::Key_F1);
  connect(action, SIGNAL(triggered()), this, SLOT(aboutPCM()));
  help_menu->addAction(action); 

  // List short cut keys
  action = new QAction("List short cut keys", this);
  action->setShortcut(Qt::Key_Question);
  connect(action, SIGNAL(triggered()), this, SLOT(PrintShortCuts()));
  help_menu->addAction(action); 

  // About Qt
  action = new QAction("About Qt", this);
  connect(action, SIGNAL(triggered()), this, SLOT(aboutQt()));
  help_menu->addAction(action); 

////////////////////////////////////////////////////////////////////////////////
// Pop-up menu for laser point attributes

  action = new QAction("Modify label", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ModifyAttributeLabel()));
  Canvas()->PopUpMenu()->addAction(action);
  
  action = new QAction("Modify filter status", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ModifyAttributeIsFiltered()));
  Canvas()->PopUpMenu()->addAction(action);
  
  action = new QAction("Modify plane number", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ModifyAttributePlaneNumber()));
  Canvas()->PopUpMenu()->addAction(action);
  
  action = new QAction("Modify segment number", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ModifyAttributeSegmentNumber()));
  Canvas()->PopUpMenu()->addAction(action);
  
  action = new QAction("Modify scan number", this);
  connect(action, SIGNAL(triggered()), this, SLOT(ModifyAttributeScanNumber()));
  Canvas()->PopUpMenu()->addAction(action);
  
////////////////////////////////////////////////////////////////////////////////
// Laser data I/O window

  laserdata_io_window = new LaserDataIOWindow(app, NULL);
  
////////////////////////////////////////////////////////////////////////////////
// Laser point information window

  point_information_window = new PointInformationWindow(NULL);
  
////////////////////////////////////////////////////////////////////////////////

  statusBar()->showMessage("Ready", 2000);
  resize(initial_width, initial_height + 75);
  move(0, 0);

////////////////////////////////////////////////////////////////////////////////
// Connect signals and slots

  // Default connectors for showing data in PCMWindows
  AddShowDataConnectors(PCMWindowPtr());
  // Update show menu if data is removed by PCMWindow
  connect(PCMWindowPtr(), SIGNAL(RemovedData(DataType)),
          this, SLOT(UpdateShowDataMenu(DataType)));
  // Update selected laser data display if laser data display was changed
  connect(PCMWindowPtr(), SIGNAL(ToggledLaserDataDisplay()),
          this, SLOT(ShowSelectedLaserData()));
  // Select object data if an object point has been selected on the canvas
  connect(Canvas(), SIGNAL(SelectedObjectPoint(const PointNumber &,
                                               const Position3D &,
                                               DataType, DataType)),
          this, SLOT(SelectObjectData(const PointNumber &,
                                      const Position3D &,
                                      DataType, DataType)));
  // Select a single laser point
  connect(Canvas(), SIGNAL(SelectedLaserPoints(LaserPoints *, bool)),
          this, SLOT(ChangeLaserPointSelection(LaserPoints *, bool)));
  // Select laser data if a laser point has been selected on the canvas
  connect(Canvas(), SIGNAL(SelectedLaserSegment(const LaserPoint &, DataType)),
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
  // Update model data in result window 
  connect(Canvas(), SIGNAL(LastModelDataHasBeenEdited()),
          this, SLOT(UpdateLastModelDataInResultWindow()));
  // Refresh display of laser data after importing new laser data
  connect(laserdata_io_window, SIGNAL(RequestLaserDataDisplay()),
          this, SLOT(DisplayLaserDataAfterImport()));
  // Show message after exporting laser data
  connect(laserdata_io_window, SIGNAL(RequestExportMessage()),
          this, SLOT(DisplayMessageAfterExport()));
  // Upload new laser data from pyramid browse thread
  connect(&browse_thread, SIGNAL(NewSelectionReady(int)),
          this, SLOT(LoadUpdatedPyramidPoints(int)));

////////////////////////////////////////////////////////////////////////////////
// Set default values

  result_appearance->SetDataTypeName("Laser result data");
  result_appearance->SetPointColourMethod(ColourByResidual);
  result_appearance->SetPointSize(3);
  appearance[SelectedLaserData]->SetPointColourMethod(ColourBySegment);
  laser_selection_tag = SegmentNumberTag;
  ReadDefaults("pcm.ini");
}


void PointCloudMapper::ToggleShowData(DataType type, bool refresh)
{
// This is no longer toggling. The menu or button selection already cause
// the action to change it's state. So, we just display what the current
// check states indicate.

  DataType base_type, selection_type;

//  printf("ToggleShowData: type %d\n", (int) type);
  
  if (type == LaserData) {
    // Toggle display of laser data
    ShowData(PCMWindowPtr(), type, show_data_actions[LaserData]->isChecked(),
             type, show_data_actions[LaserData]->isChecked(), refresh);
  }
  else if (type == SelectedLaserData) {
    // Toggle display of selected laser data. Only allow this when the
    // display of all laser data is off
    base_type = LaserData;
    if (!show_data_actions[base_type]->isChecked()) {
      ShowData(PCMWindowPtr(), base_type,
               show_data_actions[base_type]->isChecked(), type,
               show_selection_actions[type]->isChecked(), refresh);
    }
    else { // Otherwise, refuse switching off the selected data
      show_selection_actions[type]->setChecked(false);
    }
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

void PointCloudMapper::DisplayLaserDataAfterImport()
{
  DisplayData(LaserData, PCMWindowPtr(), true);
  statusBar()->showMessage(QString("Imported %1 laser points.")
                           .arg((int) laser_points.size()));
}

void PointCloudMapper::DisplayMessageAfterExport()
{
  statusBar()->showMessage(QString("Exported %1 laser points.")
                           .arg((int) laser_points.size()));
}

void PointCloudMapper::DisplayData(DataType type, PCMWindow *window,
                                   bool data_changed)
{
//  printf("DisplayData: type %d\n", (int) type);
  
  ShowData(window, type, true,
           window->Canvas()->CorrespondingSelectedDataType(type), true,
           true, data_changed);
}

void PointCloudMapper::UpdateLaserDataDisplay()
{
//  printf("UpdateLaserDataDisplay\n");
  ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData,
           !selected_laser_points.empty(), true, true);
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
  bool           update_bounds = (!window->Canvas()->HasData()) && data_changed;
  vector <PCMWindow *>::iterator           subwindow;
  const DataAppearance                     *laserdata_appearance;
		 
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
  	// Possibly difference appearance of laser data in other views
  	if (window->Type() == PCMMainDataView ||
	    window->Type() == PCMPyramidView)
	  laserdata_appearance = window->LaserDataAppearance();
	else
	  laserdata_appearance = appearance[LaserData]->DataAppearancePtr();
    if (show_base_data) {
      window->Canvas()->AddLaserData(window->PointCloud(),
                              laserdata_appearance, true,
                              false);
    }
    else window->Canvas()->RemoveLaserData(laserdata_appearance, false);
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
  // windows. LastModelData and LaserData are not shared
  if (data_changed && window->Type() == PCMMain && base_type != LastModelData) {
    for (subwindow=subwindows.begin(); subwindow!=subwindows.end();
         subwindow++) {
      if (base_type == LaserData && (*subwindow)->Type() != PCMPyramidView &&
          (*subwindow)->Type() != PCMMainDataView) continue;
      if ((*subwindow)->DataIsShown(base_type)) {
        ShowData(*subwindow, base_type, true, selection_type, false,
                 true, true);
      }
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
    
  // If the laser data has been removed, also remove the selected laser data
  if (type == LaserData) {
  	show_selection_actions[SelectedLaserData]->setChecked(false);
  	ShowData(PCMWindowPtr(), LaserData, false, SelectedLaserData, false,
	         true, false);
  }
}

void PointCloudMapper::ToggleShowSelectedPoint()
{
  // Remove selected point from display
  if (!show_selection_actions[SelectedPointData]->isChecked()) {
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
}

void PointCloudMapper::ToggleShowTileBoundaries()
{
  // Only take action when in browsing mode
  if (Canvas()->Mode() == BrowseMode) {
    // Remove tile boundaries from display
    if (!show_selection_actions[TileBoundaryData]->isChecked())
      tile_information.Clear(Canvas(), true);
    // Add tile boundaries to display if in browsing mode
    else tile_information.Update(laser_pyramid, Canvas());
  }
  // Issue warning that tile boundaries are only displayed in browsing mode
  else
    QMessageBox::information(this, "Warning",
                          "Tile boundaries are only shown in browsing mode.\n");
}

void PointCloudMapper::SelectProject()
{
  QString new_project_file =
    QFileDialog::getOpenFileName(this, "Select project file", QString(),
                    "Point cloud mapping data (*.pcm*);;All files (*.*)");
                    
  if (!new_project_file.isEmpty()) {
    if (OpenProject(new_project_file.toLatin1(), NULL,
	                reset_on_loading_points->isChecked()))
      statusBar()->showMessage("Project " + new_project_file + " opened.",2000);
    else
      statusBar()->showMessage("Could not open project "+new_project_file + ".",
                               2000);
  }
}


bool PointCloudMapper::OpenProject(const char *new_project_file,
                                   const char *ini_file,
								   bool reset_view)
{
  bool success = ReadProject(new_project_file, false);
  int type;

  if (ini_file) ReadDefaults(ini_file);
  // Delete old data
  if (Canvas()->HasData()) {
    Canvas()->ClearLaserData(false);
    // Don't delete model data if the project file is a laser data file 
    if (strstr(new_project_file, ".laser") == NULL &&
        strstr(new_project_file, ".las") == NULL &&
	    strstr(new_project_file, ".laz") == NULL) {
      Canvas()->ClearObjectData(false);
    }
  }
  
  for (type=0; type<NumNormalDataTypes; type++)
    show_data_actions[type]->setChecked(false);
  for (type=SelectedMapData; type<=SelectedModelFaceData; type++)
    show_selection_actions[type]->setChecked(false);
  if (ContainsData(MapData)) DisplayData(MapData, PCMWindowPtr(), reset_view);
  if (ContainsData(MapPartitionData))
    DisplayData(MapPartitionData, PCMWindowPtr(), reset_view);
  if (ContainsData(ModelData)) DisplayData(ModelData, PCMWindowPtr(), reset_view);
  if (ContainsData(LaserData)) DisplayData(LaserData, PCMWindowPtr(), reset_view);
  if (reset_view) Canvas()->InitialiseTransformation();
  Canvas()->update();
  if (success) setWindowTitle( QString("Point cloud mapper - ").append(new_project_file) );
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
      SetMapPointFile(filename.toLatin1());
    }
    if (!map_points.Write(map_point_file)) return false;

    // Save map topology
    if (!map_top_file || ask_file_names) {
      filename = QFileDialog::getSaveFileName(this, "Select map topology file",
                                              map_top_file,
                                       "Map topology (*.top);;All files (*.*)");
      if (filename.isEmpty()) return false;
      SetMapTopologyFile(filename.toLatin1());
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
        buildings.WriteVRML(filename.toLatin1());
        return true;
      }
      else if (filename.indexOf(".dxf", 0, Qt::CaseInsensitive) != -1) {
        buildings.WriteDXF(filename.toLatin1());
        return true;
      }
      else {
        // No .wrl or .dxf file
        SetModelPointFile(filename.toLatin1());
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
      SetModelTopologyFile(filename.toLatin1());
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
      "Laser points (*.laser);;ASCII file (*.txt;*.pts);;VRML file (*.wrl);;All files (*.*)");
  if (filename.isEmpty()) return false;

  // Save as ASCII text file
  if (filename.indexOf(".txt", 0, Qt::CaseInsensitive) != -1 ||
      filename.indexOf(".pts", 0, Qt::CaseInsensitive) != -1) {
    laserdata_io_window->Export(filename.toLatin1().data(), &laser_points);
    return true;
  }
  // Save as VRML file
  else if (filename.indexOf(".wrl", 0, Qt::CaseInsensitive) != -1) {
    LaserPoints vrml_points = Canvas()->LaserPointsWithDisplayColour(&points,
                                   appearance[LaserData]->DataAppearancePtr());
    FILE *vrml_fd;
    vrml_fd = fopen(filename.toLatin1(), "w");
    if (vrml_fd == NULL) {
      QMessageBox::information(this, "Error",
                               "Opening file", filename.toLatin1(), "failed!");
      return false;
    }
    vrml_points.VRML_Write(vrml_fd, 2, true);
    vrml_points.ErasePoints();
    fclose(vrml_fd);
    statusBar()->showMessage(QString("Laser points written to VRML file ") +
                             filename.toLatin1(), 2000);
  }
  // Save in binary laser point format
  else {
    points.SetPointFile(filename.toLatin1());
    if (!points.Write(points.PointFile(), false)) {
      QMessageBox::information(this, "Error",
                               "Writing laser points to file",
                               points.PointFile(), "failed!");
      return false;
    }
    statusBar()->showMessage(QString("Laser points written to file ") +
                             points.PointFile(), 2000);
  }

  return true;
}

void PointCloudMapper::AssignDisplayColourToLaserPoints()
{
  LaserPoints new_points = Canvas()->LaserPointsWithDisplayColour(&laser_points,
                                   appearance[LaserData]->DataAppearancePtr());
  laser_points.swap(new_points);
  new_points.ErasePoints();
}
 
void PointCloudMapper::LoadSettings()
{
  QString new_ini_file =
    QFileDialog::getOpenFileName(this, "Select parameter file", "pcm.ini",
                    "Initialisation files (*.ini*);;All files (*.*)");
                    
  if (!new_ini_file.isEmpty()) {
    ReadDefaults(new_ini_file.toLatin1().data());
    statusBar()->showMessage("Settings loaded from " + new_ini_file, 2000);
  }
}

void PointCloudMapper::SaveSettingsAs()
{
  QString filename;

  // Get the file name
  filename = QFileDialog::getSaveFileName(this, "Select parameter setting file",
                                          "pcm.ini",
                                      "Initialisation files (*.ini);;All files (*.*)");
  if (filename.isEmpty()) return;
  WriteDefaults(filename.toLatin1().data());
  statusBar()->showMessage("Settings stored to " + filename, 2000);
}

void PointCloudMapper::SelectRoofVerificationFile()
{
  QString file_name =
    QFileDialog::getOpenFileName(this, "Select roof verification file", "statsverified.txt",
                    "Text files (*.txt);;All files (*.*)");
  
  // Check if a file is selected                  
  if (file_name.isEmpty()) {
    statusBar()->showMessage("No roof verification data read", 2000);  
	return;	
  }
  
  LoadRoofRecords(file_name.toLatin1().data());
}

void PointCloudMapper::LoadRoofRecords(char *file_name)
{
  std::vector <char *>::iterator record;
  FILE                           *roof_file;
  char                           *buffer, *line, *new_record;
  int                            roof_number, i, len;
  
  // Clear old roof verification records
  if (roof_records.size()) {
  	for (record=roof_records.begin(); record!=roof_records.end(); record++)
  	  if (*record) free(*record);
  	roof_records.erase(roof_records.begin(), roof_records.end());
  }

  // Store meta data
  if (verification_file != file_name) {
    verification_file = (char *) malloc(strlen(file_name) + 1);
    strcpy(verification_file, file_name);
  }

  // Read new data
  roof_file = fopen(file_name, "r");
  if (!roof_file) {
    QMessageBox::information(this, "Error",
                             QString("Error opening ") + file_name);
    return;
  }
  buffer = (char *) malloc(MAXCHARS);
  fgets(buffer, MAXCHARS, roof_file); // Skip header record
  while ((line = fgets(buffer, MAXCHARS, roof_file))) {
  	len = strlen(line);
  	if (len < 3) continue; // Ignore empty lines
  	if (line[len-1] == 10) {line[len-1] = 0; len--;} // Remove line feed
  	new_record = (char *) malloc((len+1) * sizeof(char));
  	if (!new_record) {
  	  printf("Error allocating memory for new roof record\n");
  	  exit(0);
  	}
  	strcpy(new_record, line);
  	roof_records.push_back(new_record);
  }
  fclose(roof_file);
  if (roof_records.size()) {
    sprintf(buffer, "%d roof records read from ", roof_records.size());
    statusBar()->showMessage(QString(buffer) + QString(file_name), 2000);
  }
  else {
    QMessageBox::information(this, "Error",
                             QString("Error: no records in file ") + file_name);
  	return;
  }
  
  // Show the roof verification menu bar
  roof_verification_tools->show();
  
  // Import the first roof data set
  current_roof_record = 0;
  sscanf(roof_records[0], "%d", &roof_number);
  sprintf(buffer, "roofv%6d.pcm", roof_number);
  for (i=4; i<10; i++)
    if (buffer[i] == ' ') buffer[i] = '0';
  // Check if this file exists, if not try with 5 digits
  if (!BNF_FileExists(buffer)) {
  	printf("Can't find file %s, trying with 5 digits.\n", buffer);
    sprintf(buffer, "roofv%5d.pcm", roof_number);
    for (i=4; i<9; i++)
      if (buffer[i] == ' ') buffer[i] = '0';
    if (!BNF_FileExists(buffer)) {
      QMessageBox::information(this, "Error",
                             QString("Can't find roof file ") + QString(buffer));
      return;
    }
  }
  reset_on_loading_points->setChecked(true);
  OpenProject(buffer);
  free(buffer);
  
  // Set buttons on current decision by calling SwitchProject without increment
  SwitchProject(0);
}

void PointCloudMapper::SaveRoofRecords()
{	  
  FILE    *roof_file;
  vector <char *>::iterator record;
  char    number_string[10];

  // Save all roof records
  roof_file = fopen(verification_file, "w");
  if (!roof_file) {
    statusBar()->showMessage("Error opening " + QString(verification_file), 2000);
    return;
  }
  fprintf(roof_file, "  bld strip1      X1         Y1      Z1     strip2      X2         Y2      Z2     dXY    dZ    stdev  manual\n");
  for (record=roof_records.begin(); record!=roof_records.end(); record++)
    fprintf(roof_file, "%s\n", *record);
  fclose(roof_file);
  sprintf(number_string, "%d", roof_records.size());
  statusBar()->showMessage(QString(number_string) + " roof records saved to " +
                           verification_file, 2000);
}

void PointCloudMapper::SaveRoofRecordsAs()
{
  QString filename;
  
  // Get the file name
  filename = QFileDialog::getSaveFileName(this, "Select roof verification file",
                                          "statsverified.txt",
                                         "Text files (*.txt);;All files (*.*)");
  if (filename.isEmpty()) return;

  // Store meta data
  if (!strcmp(verification_file, filename.toLatin1())) {
    verification_file = (char *) malloc(strlen(filename.toLatin1().data()) + 1);
    strcpy(verification_file, filename.toLatin1().data());
  }

  // Save roof records to verification_file
  SaveRoofRecords();
}

void PointCloudMapper::RoofVerification(int decision, bool move_to_next)
{
  // Store the decision in the last character of the roof record
  char *ch = roof_records[current_roof_record] +
             strlen(roof_records[current_roof_record]) - 1;
  if (*ch == 10) ch--; // Correction for line feed
  sprintf(ch, "%d", decision);
  
  // Move on to the next roof
  if (move_to_next) SwitchProject(1);
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


void PointCloudMapper::aboutPCM()
{
  QMessageBox::about(this, "Point Cloud Mapper",
		     "Program for point cloud processing\n"
		     "and building modelling.\n\n"
             "Author: George Vosselman\n\nCopyright University of Twente");
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
  old_mode = Canvas()->Mode();

  // Check if this mode can be selected
  if (new_mode == SelectModelFaceMode) {
    if (!edit_model_action->isChecked()) {
      QMessageBox::information(this, "Error",
		     "Model faces can only be selected\nwhen a building is being edited.");
      mode_actions[SelectModelFaceMode]->setChecked(false);
	  return; // Leave PCM in old mode
    }
  }
  else if (old_mode == PoseChangePerspectiveMode &&
           new_mode != PoseChangeMode &&
		   new_mode != PoseChangePerspectiveMode) {
    QMessageBox::information(this, "Warning",
	      "No selections can be done in perspective view, switching to orthogonal view.");
	SetMode(PoseChangeMode);
  }        

  // Figure out what to do with old object data selections
  if (old_mode != SelectRectangleMode &&
      new_mode >= SelectMapMode && new_mode <= SelectRectangleMode &&
      (old_mode != new_mode || !Canvas()->ShiftKeyDown())) {
    remove_old_selections = true;
  }
  else if (old_mode == SelectRectangleMode &&
           !Canvas()->HasValidSelectionRectangle() &&
           !Canvas()->ShiftKeyDown() &&
           new_mode >= SelectMapMode && new_mode <= SelectModelFaceMode) {
    remove_old_selections = true;
  }
  if (new_mode == SelectLaserSegmentMode ||
      new_mode == SelectLaserPointMode) remove_old_selections = false;
  if (old_mode == BrowseMode) {
    laser_pyramid.InvalidateUsedTiles();
    if (show_selection_actions[TileBoundaryData]->isChecked())
      tile_information.Clear(Canvas(), true);
  }
  
  // Retrieve the building number if the user wants to select all partitions
  // of a map outline
  if (old_mode == SelectMapMode && new_mode == SelectMapPartitionMode) {
    if (!selected_map_data.empty())
      building_number = (*(selected_map_data.begin()))->Attribute(BuildingNumberTag);
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
    laser_points.RemoveAttribute(IsSelectedTag);
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
    if (new_mode >= SelectMapMode && new_mode <= SelectModelFaceMode) {
      SelectInsideRectangle(new_mode);
      // Change to back to select rectangle mode
      real_new_mode = SelectRectangleMode;
    }
    // Remove the selection rectangle
    Canvas()->RemoveSelectionRectangle();
  }

  // Remove old selection circle
  if (old_mode == SelectLaserPointMode) {
  	Canvas()->setMouseTracking(false);
  	Canvas()->RemoveSelectionCircle();
  }

  // Select all partitions of the current building if the user switches from
  // SelectMapMode to SelectMapPartitionMode
  if (old_mode == SelectMapMode && new_mode == SelectMapPartitionMode &&
      building_number != -1)
    SelectMapPartitionLines(building_number);

  // Recalculate bounds if coming out of the browse mode
  if (old_mode == BrowseMode) laser_points.DeriveDataBounds(0);
    
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

  // Enable mouse movement tracking when no button is pressed
  if (new_mode == SelectLaserPointMode) Canvas()->setMouseTracking(true);

  // Initialise pyramid browser
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
  
  // Refresh in case of a different projection
  if (old_mode == PoseChangePerspectiveMode)
    Canvas()->SetOrthogonalProjection();
  else if (real_new_mode == PoseChangePerspectiveMode)
    Canvas()->SetPerspectiveProjection();
}

void PointCloudMapper::ImportMapData()
{
  QString filename;
  filename = QFileDialog::getOpenFileName(this, "Select map point file",
                                          QString(),
                                      "Map points (*.objpts);;All files (*.*)");
  if (filename.isEmpty()) return;
  SetMapPointFile(filename.toLatin1());
  filename = QFileDialog::getOpenFileName(this, "Select map topology file",
                                          QString(),
                                       "Map topology (*.top);;All files (*.*)");
  if (filename.isEmpty()) return;
  SetMapTopologyFile(filename.toLatin1());
  buildings.ImportMapData(map_point_file, map_top_file, map_points);
  ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, true, true, true);
}

void PointCloudMapper::ImportPCMMapData()
{
  QString filename;
  filename = QFileDialog::getOpenFileName(this, "Select PCM map point file",
                                          QString(),
                                      "Map points (*.objpts);;All files (*.*)");
  if (filename.isEmpty()) return;
  SetMapPointFile(filename.toLatin1());
  filename = QFileDialog::getOpenFileName(this, "Select PCM map topology file",
                                          QString(),
                                       "Map topology (*.top);;All files (*.*)");
  if (filename.isEmpty()) return;
  SetMapTopologyFile(filename.toLatin1());
  buildings.ImportPCMMapData(map_point_file, map_top_file, map_points, true);
  if (buildings.ContainsData(MapPartitionData))
    ShowData(PCMWindowPtr(), MapPartitionData, true,
             SelectedMapPartitionData, true, false, true);
  ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, true, true, true);
}

void PointCloudMapper::ImportPCMModelData()
{
  QString filename;
  filename = QFileDialog::getOpenFileName(this, "Select PCM model point file", 
                                          QString(),
                                  "Object points (*.objpts*);;All files (*.*)");
  if (filename.isEmpty()) return;
  SetModelPointFile(filename.toLatin1());
  filename = QFileDialog::getOpenFileName(this, "Select PCM model topology file",
                                          QString(),
                                   "Object topology (*.top*);;All files (*.*)");
  if (filename.isEmpty()) return;
  SetModelTopologyFile(filename.toLatin1());
  buildings.ImportPCMModelData(model_point_file, model_top_file, model_points);
  ShowData(PCMWindowPtr(), ModelData, true, SelectedModelData, true, true);
}

int PointCloudMapper::OpenLaserPyramid()
{
  QString                filename;
  DataBounds2D           bounds;

  // Get the pyramid meta file
  filename = QFileDialog::getOpenFileName(this, "Select laser pyramid file",
                                          QString(),
                                   QString("Laser pyramid files (*.pyramid)") +
                                   ";;All files (*.*)");
  if (filename.isEmpty()) return 0;
  
  // Read the meta data
  laser_pyramid.ReInitialise(); // Clear old data
  SetLaserPyramidFile(filename.toLatin1());
  if (!laser_pyramid.ReadMetaData(filename.toLatin1())) {
    printf("Error reading laser pyramid data from file %s\n",
           filename.toLatin1().data());
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
                                          QString(),
                                   QString("Laser block meta files (*.block)") +
                                   ";;Laser strip meta files (*.strip)" +
                                   ";;Laser point set meta files (*.pointset)" +
                                   ";;Laser point data files (*.laser*)" +
                                   ";;All files (*.*)");
  if (filename.isEmpty()) return;
  laser_block.ErasePoints(); // Clear old data
  SetLaserMetaFile(filename.toLatin1());
  laser_block.Create(filename.toLatin1(), &fileclass); // Read meta data
  numpts = laser_block.ReadPoints(appearance[LaserData]->MaximumNumberOfPointsInMemory());
  if (numpts == -1)
    printf("Block contains more than %d laser points.\n",
           appearance[LaserData]->MaximumNumberOfPointsInMemory());
  else printf("Block with %d laser points read.\n", numpts);
}

void PointCloudMapper::ImportNewLaserPoints()
{
  QString filename;
  FILE    *fd;
  int     file_id;
  
  filename = QFileDialog::getOpenFileName(this, "Select laser point file", 
                                          QString(),
                 QString("Laser point data files (*.laser;*.las;*.laz)") +
                    ";;ASCII point data files (*.txt;*.pts);;All files (*.*)");
  if (filename.isEmpty()) return;
  
  // Check the file type
#ifdef windows
  if ((fd = Open_Compressed_File(filename.toLatin1(), "rb")) == NULL) {
#else
  if ((fd = Open_Compressed_File(filename.toLatin1(), "r")) == NULL) {
#endif
    fprintf(stderr, "Could not open laser points file %s\n",
            filename.toLatin1().data());
    return;
  }
  fread(&file_id, sizeof(int), 1, fd);
  fclose(fd);

  // Clear old data  
  if (laser_points.size()) laser_points.ErasePoints();
  
  // Select appropriate file format
  switch (file_id) {
    // Read the points from binary file format
    case LASER_POINTS_V1:
    case LASER_POINTS_V2:
    case LASER_POINTS_V3:
    case LASER_LAS:
      SetLaserPointFile(filename.toLatin1());
      laser_points.Read(filename.toLatin1(), false);
      break;
      
    // Read the data from an ASCII file
    default:
      laserdata_io_window->Import(filename.toLatin1().data(), &laser_points);
      break;
  }
  
  laser_points.DeriveDataBounds(0);
  if (laser_points.size())
    ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, false, true, true);
}

void PointCloudMapper::ImportAdditionalLaserPoints()
{
  QString filename;
  LaserPoints additional_laser_points;
  
  filename = QFileDialog::getOpenFileName(this, "Select laser point file", 
                                          QString(),
                                  QString("Laser point data files (*.laser*;*.las;*.laz)") +
                                          ";;All files (*.*)");
  if (filename.isEmpty()) return;

  additional_laser_points.Read(filename.toLatin1(), false);
  laser_points.AddPoints(additional_laser_points);
  additional_laser_points.ErasePoints();
  if (laser_points.size())
    ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, false, true);
}

void PointCloudMapper::RemoveLoadedLaserPointsFromSet()
{
//open a file with laserpoints. All points in the existing instance which are also contained in the loaded file will be removed
  QString filename;
  LaserPoints laser_points_to_be_removed;
  
  filename = QFileDialog::getOpenFileName(this, "Select laser point file", 
                                          QString(),
                                  QString("Laser point data files (*.laser*)") +
                                          ";;All files (*.*)");
  if (filename.isEmpty()) return;

  laser_points_to_be_removed.Read(filename.toLatin1(), false);
  //iterate over the existing points and set the Tag UndefinedTag to 1001 if the same point is contained in the loaded file. Then remove all points with this Tag
  //to erase every point separately from the vector is too expensive

  //to speed up we fist sort according to coordinates so when we loop and search for corresponding points we do not start over and over again

  //STILL: this is a very expensive function!
LaserPoints::iterator ptthis, ptrem;

int count=0;
int foundnr=0;
bool startover=0;

printf("\n %d points in the loaded set\n", (int) laser_points_to_be_removed.size());

/*
printf("Sorting both sets on coordinates....\n");

laser_points_to_be_removed.SortOnCoordinates();
laser_points.SortOnCoordinates();
printf("...done\n");
*/

ptrem=laser_points_to_be_removed.begin();

for (ptthis=laser_points.begin(); ptthis!=laser_points.end(); ptthis++)
{
count++;

	if (ptrem>=laser_points_to_be_removed.end()) {ptrem=laser_points_to_be_removed.begin();}

        LaserPoint marker=*ptrem;

	for (;;)
	{
		//need only to compare the X,Y,Z coord. It is much cheaper than full comparsion of instances
		//only do it if this ptrem did not match another point yet.
		if(!ptrem->HasAttributeValue(UndefinedTag,1001))
		{
			if ((ptrem->X()==ptthis->X()) && (ptrem->Y()==ptthis->Y()) && (ptrem->Z()==ptthis->Z())) 
				{
				ptthis->SetAttribute(UndefinedTag,1001); 
				//also mark the ptrem to avoid that it will be used again later. To erase it is too expensive...
				ptrem->SetAttribute(UndefinedTag,1001);
				foundnr++;ptrem++;break;
				}		
		}
		//avoid an endless loop when no matching point was found. 
		//the startover flag is necessary, otherwise the marker would match everytime the if clause above was false				
		if (startover && marker.X()==ptrem->X() && marker.Y()==ptrem->Y() && marker.Z()==ptrem->Z()) {startover=0;break;} 

		//manually restart the loop over the points to be removed
		if (ptrem==laser_points_to_be_removed.end()-1) {ptrem=laser_points_to_be_removed.begin()-1; startover=1;}
	    			
	ptrem++;
	}//ptrem

if (foundnr==(int)laser_points_to_be_removed.size()) break;
printf("%d from %d compared, found %d points\r", count, (int) laser_points.size(),foundnr);
}//ptthis

printf("\n Found %d identical points ==> those will now be deleted from the initial set\n",foundnr);

laser_points.RemoveTaggedPoints(1001, UndefinedTag);

  if (laser_points.size())
    ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, false, true);
}

void PointCloudMapper::PrintShortCuts()
{
  printf("\nShort cut keys on canvas\n\n");
/*
  printf("1: Colour laser data by scan number (without removing AFN code)\n");
  printf("2: Colour laser data by scan number after removing AFN code\n");
  printf("3: Colour laser data by AFN code in scan number\n");
*/
  printf("0-9: Set label of selected points to 0-9\n");
  printf("a: Colour laser data by label\n");
  printf("A: Assign display colour to laser points\n");
  printf("b: Edit appearance of building model data\n");
  printf("B: Edit appearance of selected building model data\n");
  printf("c: Crop laser data to selected area\n");
  printf("d: Delete laser data in selected area\n");
  printf("e: Colour laser data by plane number\n");
  printf("f: Filter laser data with slope based filter\n");
  printf("g: Colour laser data by segment number\n");
  printf("h: Colour laser data by height\n");
  printf("i: Intersect two selected laser segments to reconstruct a model line\n");
  printf("l: Edit appearance of laser data\n");
  printf("L: Edit appearance of selected laser data\n");
  printf("m: Edit appearance of map data\n");
  printf("M: Edit apperanace of selected map data\n");
  printf("n: Switch to next project (file names should have 5 digits, e.g. roof00010.pcm)\n");
  printf("N: Increase the step size between numbered project files\n");
  printf("p: Switch to previous project (file names should have 5 digits\n");
  printf("P: Decrease the step size between numbered project files\n");
  printf("r: Reconstruct roof corner from selected map lines and laser segments\n");
  printf("s: Toggle display of scale bar (off, lower left corner, or centred\n");
  printf("t: Decrease scale bar tick distance (minimum is 1 mm)\n");
  printf("T: Increase scale bar tick distance (maximum is 1 km)\n");
  printf("x: Accepted roof verification or Clear selection of laser points\n");
  printf("v: Reject roof verification\n");
  printf("z: Zoom out by 10%s\n", "%");
  printf("Z: Zoom in by 10%s\n", "%");
  printf("-: Decrease laser point size by 1 pixel\n");
  printf("+: Increase laser point size by 1 pixel\n");
  printf(">: Increase laser data colour cycle length by a factor 1.5\n");
  printf("<: Decrease laser data colour cycle length by a factor 1.5\n");
  printf(".: Increase laser data colour cycle phase by 0.1 (maximum is 1.0)\n");
  printf(".: Decrease laser data colour cycle phase by 0.1 (minimum is 0.0)\n");
  printf("?: Print this list of short cut keys\n");
  printf("Arrow left : Move display window to the West by 10%s\n", "%");
  printf("Arrow right: Move display window to the East by 10%s\n", "%");
  printf("Arrow up   : Move display window to the North by 10%s\n", "%");
  printf("Arrow down : Move display window to the South by 10%s\n", "%");
  printf("Shift + Arrow left : Move display window to the West by 50%s\n", "%");
  printf("Shift + Arrow right: Move display window to the East by 50%s\n", "%");
  printf("Shift + Arrow up   : Move display window to the North by 50%s\n", "%");
  printf("Shift + Arrow down : Move display window to the South by 50%s\n", "%");
  printf("\n");
}

void PointCloudMapper::ProcessShortCutKeys(QKeyEvent *event)
{
  double tick_distance;
  char   number[5];
  
  if (!Canvas()->ShiftKeyDown()) {
    switch (event->key()) {
      case Qt::Key_0: ModifyLabel(0); break; // Set label of selected points to 0
      case Qt::Key_1: ModifyLabel(1); break; // Set label of selected points to 1
      case Qt::Key_2: ModifyLabel(2); break; // Set label of selected points to 2
      case Qt::Key_3: ModifyLabel(3); break; // Set label of selected points to 3
      case Qt::Key_4: ModifyLabel(4); break; // Set label of selected points to 4
      case Qt::Key_5: ModifyLabel(5); break; // Set label of selected points to 5
      case Qt::Key_6: ModifyLabel(6); break; // Set label of selected points to 6
      case Qt::Key_7: ModifyLabel(7); break; // Set label of selected points to 7
      case Qt::Key_8: ModifyLabel(8); break; // Set label of selected points to 8
      case Qt::Key_9: ModifyLabel(9); break; // Set label of selected points to 9
 
/*
      case Qt::Key_1: // Colour laser by scan number
        appearance[LaserData]->SetPointColourMethod(ColourByScanNumber);
        appearance[LaserData]->Update(); break;
      case Qt::Key_2: // Colour laser by scan number in case FLI-MAP AFN code is included
        appearance[LaserData]->SetPointColourMethod(ColourByScanNumberWithoutAFN);
        appearance[LaserData]->Update(); break;
      case Qt::Key_3: // Colour laser by FLI-MAP AFN code in scan numbers
        appearance[LaserData]->SetPointColourMethod(ColourByAFNCode);
        appearance[LaserData]->Update(); break;
*/
      case Qt::Key_Minus: // Decrease laser point size
        if (appearance[LaserData]->PointSize() > 1) {
          appearance[LaserData]->PointSizeEditor()->stepDown();
          appearance[SelectedLaserData]->PointSizeEditor()->stepDown();
          appearance[LaserData]->Update();
          appearance[SelectedLaserData]->Update();
        }
        break;
      case Qt::Key_A: // Colour laser data by label
        appearance[LaserData]->SetPointColourMethod(ColourByLabel);
        appearance[LaserData]->Update(); break;
      case Qt::Key_B: // Edit appearance of building model data
        appearance[ModelData]->Edit(); break;
      case Qt::Key_C: // Crop laser data to selected area
        CropToSelectedLaserData(); break;
      case Qt::Key_D: // Delete laser data in selected area
        DeleteSelectedLaserData(); break;
      case Qt::Key_E: // Colour laser data by plane number
        appearance[LaserData]->SetPointColourMethod(ColourByPlane);
        appearance[LaserData]->Update(); break;
      case Qt::Key_F: // Reduce focal length
        canvas->ResizeFocalLengthCanvas(0.9, true); break;
      case Qt::Key_G: // Colour laser data by segment
        appearance[LaserData]->SetPointColourMethod(ColourBySegment);
        appearance[LaserData]->Update(); break;
      case Qt::Key_H: // Colour laser data by height
        appearance[LaserData]->SetPointColourMethod(ColourByHeight);
        appearance[LaserData]->Update(); break;
      case Qt::Key_I: // Reconstruct 3D line segment
        Reconstruct3DLineSegment(); break;
      case Qt::Key_L: // Edit appearance of laser data
        appearance[LaserData]->Edit(); break;
      case Qt::Key_M: // Edit appearance of map data
        appearance[MapData]->Edit(); break;
      case Qt::Key_N: // Switch to next project
        SwitchProject(project_number_increment); break;
      case Qt::Key_P: // Switch to previous project
        SwitchProject(-project_number_increment); break;
      case Qt::Key_R: // Reconstruct roof corner
        ReconstructRoofCorner(); break;
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
      case Qt::Key_T: // Decrease scale bar tick distance
        tick_distance = canvas->ScaleBarTickDistance();
        if (tick_distance < 0.002) return; // No further decrease
        tick_distance /= 10.0;
        SetScaleBar(canvas->ScaleBarLocation(), tick_distance);
        break;
      case Qt::Key_V: 
	    // Approve verification of roof and move on to next undecided roof
        if (roof_records.size()) {
          RoofVerification(0, false);
          NextUndecidedRoof();
        }
      case Qt::Key_X:
      	// Reject verification of roofand move on to next undecided roof
      	if (roof_records.size()) {
          RoofVerification(2, false);
          NextUndecidedRoof();
      	}
	    // Clear selection of laser points
        else if (!selected_laser_points.empty()) {
          selected_laser_points.ErasePoints();
          laser_points.RemoveAttribute(IsSelectedTag);
          ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, false,
		           true, true);
        }
        break;
      case Qt::Key_Z: // Zoom out by 10%
        canvas->ScaleCanvas(0.9, true); break;
      case Qt::Key_Comma: // Decrease point colour cycle phase
        appearance[LaserData]->DataAppearance::SetPointColourCyclePhase
          (appearance[LaserData]->PointColourCyclePhase() - 0.1);
        appearance[SelectedLaserData]->DataAppearance::SetPointColourCyclePhase
          (appearance[SelectedLaserData]->PointColourCyclePhase() - 0.1);
        appearance[LaserData]->Update();
        appearance[SelectedLaserData]->Update(); break;
      case Qt::Key_Period: // Increase point colour cycle phase
        appearance[LaserData]->DataAppearance::SetPointColourCyclePhase
          (appearance[LaserData]->PointColourCyclePhase() + 0.1);
        appearance[SelectedLaserData]->DataAppearance::SetPointColourCyclePhase
          (appearance[SelectedLaserData]->PointColourCyclePhase() + 0.1);
        appearance[LaserData]->Update();
        appearance[SelectedLaserData]->Update(); break;
      case Qt::Key_Left: // Move display window to the left
        canvas->TranslateCanvas(0.1, 0.0, true); break;
      case Qt::Key_Right: // Move display window to the right
        canvas->TranslateCanvas(-0.1, 0.0, true); break;
      case Qt::Key_Up: // Move display window up
        canvas->TranslateCanvas(0.0, -0.1, true); break;
      case Qt::Key_Down: // Move display window down
        canvas->TranslateCanvas(0.0, 0.1, true); break;
      default:
        break;
    }
  }
  else { // With shift key down
    switch (event->key()) {
      case Qt::Key_A: // Assign display colour to laser points
        AssignDisplayColourToLaserPoints(); break;
      case Qt::Key_B: // Edit appearance of selected building model data
        appearance[SelectedModelData]->Edit(); break;
      case Qt::Key_F: // Increase focal length
        canvas->ResizeFocalLengthCanvas(1.0/0.9, true); break;
      case Qt::Key_L: // Edit appearance of selected laser data
        appearance[SelectedLaserData]->Edit(); break;
      case Qt::Key_M: // Edit appearance of selected map data
        appearance[SelectedMapData]->Edit(); break;
      case Qt::Key_N: // Increase project increment
        project_number_increment++;
        sprintf(number, "%d", project_number_increment);
        statusBar()->showMessage("Project increment set to " + QString(number), 2000);
		break;
      case Qt::Key_P: // Decrease project increment
        if (project_number_increment > 1) {
          project_number_increment--;
          sprintf(number, "%d", project_number_increment);
          statusBar()->showMessage("Project increment set to " + QString(number), 2000);
        }
        else {
          QMessageBox::information(this, "Error",
                                   "Project increment cannot be smaller than 1.");
        }
        break;
      case Qt::Key_T: // Increase scale bar tick distance
        tick_distance = canvas->ScaleBarTickDistance();
        if (tick_distance > 999) return; // No further increase
        tick_distance *= 10.0;
        SetScaleBar(canvas->ScaleBarLocation(), tick_distance);
        break;
      case Qt::Key_V: // Accept roof and move on the the next one
        if (roof_records.size()) RoofVerification(0, true); break;
      case Qt::Key_X: // Reject roof and move on the the next one
        if (roof_records.size()) RoofVerification(2, true); break;
      case Qt::Key_Z: // Zoom in by 10%
        canvas->ScaleCanvas(1.0/0.9, true); break;
      case Qt::Key_Plus: // Increase laser point size
        appearance[LaserData]->PointSizeEditor()->stepUp();
        appearance[SelectedLaserData]->PointSizeEditor()->stepUp();
        appearance[LaserData]->Update();
        appearance[SelectedLaserData]->Update(); break;
      case Qt::Key_Less: // Decrease point colour cycle length
        appearance[LaserData]->DataAppearance::SetPointColourCycleLength
          (appearance[LaserData]->PointColourCycleLength() / 1.5);
        appearance[SelectedLaserData]->DataAppearance::SetPointColourCycleLength
          (appearance[SelectedLaserData]->PointColourCycleLength() / 1.5);
        appearance[LaserData]->Update();
        appearance[SelectedLaserData]->Update(); break;
      case Qt::Key_Greater: // Increase point colour cycle length
        appearance[LaserData]->DataAppearance::SetPointColourCycleLength
          (appearance[LaserData]->PointColourCycleLength() * 1.5);
        appearance[SelectedLaserData]->DataAppearance::SetPointColourCycleLength
          (appearance[SelectedLaserData]->PointColourCycleLength() * 1.5);
        appearance[LaserData]->Update();
        appearance[SelectedLaserData]->Update(); break;
      case Qt::Key_Question: // Print this list of short cut keys
        PrintShortCuts();
      case Qt::Key_Left: // Move display window to the left
        canvas->TranslateCanvas(0.5, 0.0, true); break;
      case Qt::Key_Right: // Move display window to the right
        canvas->TranslateCanvas(-0.5, 0.0, true); break;
      case Qt::Key_Up: // Move display window up
        canvas->TranslateCanvas(0.0, -0.5, true); break;
      case Qt::Key_Down: // Move display window down
        canvas->TranslateCanvas(0.0, 0.5, true); break;
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
  building_number = outline->Attribute(BuildingNumberTag);
  building = buildings.BuildingPtr(building_number);
  if (!building) {
    printf("Error locating building with number %d\n", outline->Attribute(BuildingNumberTag));
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
  ObjectPoints                 *points_ptr;  

  // Check if something is selected
  if (selected_map_data.size()) {
    selected_line = selected_map_data.begin();
    points_ptr = &map_points;
  }
  else if (selected_map_part_data.size()) {
    selected_line = selected_map_part_data.begin();
    points_ptr = &map_points;
  }
  else if (selected_model_face_data.size()) {
    selected_line = selected_model_face_data.begin();
    points_ptr = &last_model_points;
  }
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
  Canvas()->InitialiseExtensionLine(points_ptr, selected_line);
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

  // If this window is used for editing a building model, disable further editing
  if (closed_window == edit_model_window) {
    edit_model_action->setChecked(false);
    edit_model_window = NULL;
  }
  
  // Remove the window from the list of sub windows
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
  
  if (canvas->ScaleBarTickDistance() != new_distance) {
    if (new_distance < 0.005)
      statusBar()->showMessage(QString("Scale bar ticks at 0.001 m"));
    else if (new_distance < 0.05)
      statusBar()->showMessage(QString("Scale bar ticks at 0.01 m"));
    else if (new_distance < 0.5)
      statusBar()->showMessage(QString("Scale bar ticks at 0.1 m"));
    else
      statusBar()->showMessage(QString("Scale bar ticks at %1 m").arg((int) (new_distance+0.01)));
  }
  for (i=0; i<3; i++)
    show_scale_actions[i]->setChecked(new_location == (PCMScaleLocation) i);
  item = (int) (log(new_distance) / log(10.0) + 3.01);
  for (i=0; i<7; i++)
    set_scale_tick_actions[i]->setChecked(i == item);
  canvas->SetScaleLocation(new_location);
  canvas->SetScaleTickDistance(new_distance);
  if (update) canvas->update();
}

void PointCloudMapper::SetLaserSelectionTag(int selection_tag_index)
{
  int i;
  
  for (i=0; i<10; i++)
    set_laser_tag_actions[i]->setChecked(selection_tag_index == i);
  switch (selection_tag_index) {
    case 0: laser_selection_tag = LabelTag;
            appearance[LaserData]->SetPointColourMethod(ColourByLabel);
            appearance[SelectedLaserData]->SetPointColourMethod(ColourByLabel);
            break;
    case 1: laser_selection_tag = SegmentNumberTag;
            appearance[LaserData]->SetPointColourMethod(ColourBySegment);
            appearance[SelectedLaserData]->SetPointColourMethod(ColourBySegment);
            break;
    case 2: laser_selection_tag = PlaneNumberTag;
            appearance[LaserData]->SetPointColourMethod(ColourByPlane);
            appearance[SelectedLaserData]->SetPointColourMethod(ColourByPlane);
            break;
    case 3: laser_selection_tag = ScanNumberTag;
            appearance[LaserData]->SetPointColourMethod(ColourByScanNumber);
            appearance[SelectedLaserData]->SetPointColourMethod(ColourByScanNumber);
            break;
    case 4: laser_selection_tag = IsFilteredTag;
            appearance[LaserData]->SetPointColourMethod(ColourByFilterStatus);
            appearance[SelectedLaserData]->SetPointColourMethod(ColourByFilterStatus);
            break;
    case 5: laser_selection_tag = PulseCountTag;
            appearance[LaserData]->SetPointColourMethod(ColourByPulseCount);
            appearance[SelectedLaserData]->SetPointColourMethod(ColourByPulseCount);
            break;
    case 6: laser_selection_tag = ComponentNumberTag;
            appearance[LaserData]->SetPointColourMethod(ColourByComponent);
            appearance[SelectedLaserData]->SetPointColourMethod(ColourByComponent);
            break;
    case 7: laser_selection_tag = ScanNumberWithoutAFNTag;
            appearance[LaserData]->SetPointColourMethod(ColourByScanNumberWithoutAFN);
            appearance[SelectedLaserData]->SetPointColourMethod(ColourByScanNumberWithoutAFN);
            break;
    case 8: laser_selection_tag = AFNTag;
            appearance[LaserData]->SetPointColourMethod(ColourByAFNCode);
            appearance[SelectedLaserData]->SetPointColourMethod(ColourByAFNCode);
            break;
    case 9: laser_selection_tag = SegmentStartTileNumberTag;
            appearance[LaserData]->SetPointColourMethod(ColourBySegmentAndTile);
            appearance[SelectedLaserData]->SetPointColourMethod(ColourBySegmentAndTile);
            break;
    case 10: laser_selection_tag = ScanLineNumberTag;
            appearance[LaserData]->SetPointColourMethod(ColourByScanLineNumber);
            appearance[SelectedLaserData]->SetPointColourMethod(ColourByScanLineNumber);
            break;
            
    default: printf("Invalid selection tag index %d\n", selection_tag_index);
  }
}


void PointCloudMapper::InitialiseBrowseMode()
{
  DataBounds2D bounds;
  LaserPyramid::iterator top_level;
  LaserUnit::iterator    top_tile;
  bool                   debug = false;
  
  // Reset view to vertical
  Canvas()->LevelView();
  
  // Get bounds from canvas if some data is there
  if (Canvas()->HasData()) {
    bounds = Canvas()->BoundsXY();
    if (debug) 
	  printf("Initiating pyramid display based on bounds of current data\n");
  }
  // Otherwise, get bounds from top pyramid level
  else {
    top_level = laser_pyramid.end() - 1;
    top_tile  = top_level->begin()->begin();
    bounds.SetMinimumX(top_tile->TileBounds().Minimum().X());
    bounds.SetMaximumX(top_tile->TileBounds().Maximum().X());
    bounds.SetMinimumY(top_tile->TileBounds().Minimum().Y());
    bounds.SetMaximumY(top_tile->TileBounds().Maximum().Y());
    if (debug)
	  printf("Initiating pyramid display based on tile bounds of top level %s\n",
	         top_level->Name());
  }
  if (debug) {
  	printf("X-range %8.3f %8.3f\n", bounds.Minimum().X(), bounds.Maximum().X());
  	printf("Y-range %8.3f %8.3f\n", bounds.Minimum().Y(), bounds.Maximum().Y());
  }
  
  // Load new pyramid data
  UpdatePyramidSelection(bounds);
}

void PointCloudMapper::UpdatePyramidSelection(const DataBounds2D &bounds)
{
  double max_point_spacing;
  int    row_min, row_max, col_min, col_max, level;
  bool   use_thread=true;

  // Convert the point spacing on screen to point spacing in terrain
  max_point_spacing =
    appearance[LaserData]->MaximumPointSpacingInPyramid() * 
    bounds.XRange() / Canvas()->width();

  if (use_thread) {
    browse_thread.UpdateRequest(bounds, max_point_spacing, &laser_points,
                                &laser_pyramid,
                                appearance[LaserData]->MaximumNumberOfPointsInMemory());
  }
  else {
    if (!laser_pyramid.UpdateSelection(bounds, max_point_spacing, laser_points,
         appearance[LaserData]->MaximumNumberOfPointsInMemory())) return; // No change

    // Add laser data to canvas
    ShowData(PCMWindowPtr(), LaserData, true,
             SelectedLaserData, false, true, true);

    laser_pyramid.SelectedTileRange(row_min, row_max, col_min, col_max, level);
    statusBar()->showMessage(QString("Selected tiles: rows %1-%2, columns %3-%4 at level %5, %6 points")
             .arg(row_min)
             .arg(row_max)
             .arg(col_min)
             .arg(col_max)
             .arg(level)
             .arg(laser_points.size()));
  }
}

void PointCloudMapper::LoadUpdatedPyramidPoints(int num_pts)
{
  int    row_min, row_max, col_min, col_max, level;

  // Update the laser points
  browse_thread.TransferNewSelection(&laser_points, &laser_pyramid);
//  laser_points.DeriveDataBounds(0);

  // Report on used tiles
  row_min = num_pts; // To avoid a compiler warning
  browse_thread.SelectedTileRange(row_min, row_max, col_min, col_max, level);
  statusBar()->showMessage(QString("Selected tiles: rows %1-%2, columns %3-%4 at level %5, %6 points")
           .arg(row_min)
           .arg(row_max)
           .arg(col_min)
           .arg(col_max)
           .arg(level)
           .arg(laser_points.size()));

  // Add laser data to canvas
  ShowData(PCMWindowPtr(), LaserData, true,
           SelectedLaserData, false, false, true);
           
  // Include drawing of tile bounds if required
  if (show_selection_actions[TileBoundaryData]->isChecked())
    tile_information.Update(laser_pyramid, Canvas(), false);
  
  // Update display
  canvas->update();
}

void PointCloudMapper::UpdateLastModelDataInResultWindow()
{
  if (!edit_model_window) return;
  edit_model_window->AddReconstructedModel(last_model_points,
                                           last_model_part,
                                appearance[LastModelData]->DataAppearancePtr());
}

void PointCloudMapper::QuitPCM()
{
  WriteDefaults("pcm.ini");
  qApp->closeAllWindows();
  qApp->quit();
}
