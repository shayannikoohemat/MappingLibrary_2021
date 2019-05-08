
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


#include "CityWindow.h"
#include <QToolBar>
#include <QSize>
#include <QStatusBar>
#include <QCloseEvent>
#include <QMessageBox>
#include "digphot_arch.h"
#include "DataTypes.h"
#include "DataAppearWin.h"

CityWindow::CityWindow(CityWindowType type, QWidget *parent, Qt::WFlags flags) :
  QMainWindow(parent, flags)
{
  // Copy the window type
  window_type = type;
  
  feature_type=None;
  for (int i=0; i<NumDataTypes; i++) {
    appearance[i] = new DataAppearanceWindow((DataType) i);
  }

  // Create the canvas
  canvas = new CityCanvas(0, this);
  canvas->setFocusPolicy(Qt::StrongFocus);
  setCentralWidget(canvas);

  // Create the show data toolbar
  CreateShowToolBar();

  // Create the view toolbar
  CreateViewToolBar();
  
  if(type!=CityResult)
  CreateEditToolBar();
  
  
  AddShowDataConnectors(CityWindowPtr());
  
  
  canvas->setMinimumSize(200, 200);
  resize(300, 300);
  
  
  
  
  statusBar()->showMessage("Ready", 2000);
  resize(300, 450);
  
}

  CityWindow::CityWindow(CityWindowType type, FeatureWindowType ftype, QWidget *parent, Qt::WFlags flags):
  QMainWindow(parent, flags)
              {
  // Copy the window type
  window_type = type;
  
  feature_type=ftype;
  for (int i=0; i<NumDataTypes; i++) {
    appearance[i] = new DataAppearanceWindow((DataType) i);
  }

  // Create the canvas
  canvas = new CityCanvas(0, this);
  canvas->setFocusPolicy(Qt::StrongFocus);
  setCentralWidget(canvas);

  // Create the show data toolbar
  CreateShowToolBar();

  // Create the view toolbar
  CreateViewToolBar();
  
  if(type!=CityResult)
  CreateEditToolBar();

  
  AddShowDataConnectors(CityWindowPtr());
  
  canvas->setMinimumSize(200, 200);
  resize(300, 300);
  
   statusBar()->showMessage("Ready", 2000);
  resize(300, 375);
  
  
                         }

void CityWindow::CreateEditToolBar()
{
QAction *action;     
QToolBar *edit_tools = new QToolBar("Edit operations", this);     
addToolBar(Qt::BottomToolBarArea, edit_tools);
#ifdef linux
  edit_tools->setIconSize(QSize(20, 20));
#else
  edit_tools->setIconSize(QSize(18, 18));
#endif 
   
   
   
   
   
   // Mouse mode pose change 
   action = mode_actions[PoseChangeMode] =
   new QAction(QIcon(":/buttons/posechangemode.xpm"), "Pose change", this);
  action->setStatusTip(tr("Pose change"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetPoseChangeMode()));
  edit_tools->addAction(action);
  
  // Select laser data
  action = new QAction(QIcon(":/buttons/selectlaser.xpm"), "Select laser data",
                       this);
  action->setStatusTip(tr("Select laser data"));
  connect(action, SIGNAL(triggered()), this, SLOT(SelectLaserData()));
  edit_tools->addAction(action);
  
  // Delete laser data
  action = new QAction(QIcon(":/buttons/deletelaser.xpm"), "Delete laser data",
                       this);
  action->setStatusTip(tr("Delete laser data"));
  connect(action, SIGNAL(triggered()), this, SLOT(DeleteLaserData()));
  edit_tools->addAction(action);
  
   // Mouse mode select laser segment
  
  //select laser segment
  action = mode_actions[SelectLaserSegmentMode] =
    new QAction(QIcon(":/buttons/selectsegment.xpm"), "Select laser segment",
                this);
  action->setStatusTip(tr("Select laser segment"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectLaserSegmentMode()));
  edit_tools->addAction(action);
  
  
 //crop laser data
  action = new QAction(QIcon(":/buttons/croplaser.xpm"), "Crop laser data",
                       this);
  action->setStatusTip(tr("Crop laser data"));
  connect(action, SIGNAL(triggered()), this, SLOT(CropLaserData()));
  edit_tools->addAction(action);
 
  
  // Merge segment
  action = 
    new QAction(QIcon(":/buttons/mappolygon.xpm"), "Merge Segment", this);
  action->setStatusTip(tr("Merge Segment"));
  connect(action, SIGNAL(triggered()), this, SLOT(UnifySegmentNumberSelectedLaserData()));
  edit_tools->addAction(action);
  
   // End of laser edit buttons on edit toolbar
  edit_tools->addSeparator();


  // Mouse mode select map line
  action = mode_actions[SelectMapMode] =
    new QAction(QIcon(":/buttons/selectmap.xpm"), "Select map line", this);
  action->setStatusTip(tr("Select map line"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectMapMode()));
  edit_tools->addAction(action);   
     
  // Mouse mode select map partition line
  action = mode_actions[SelectMapPartitionMode] =
    new QAction(QIcon(":/buttons/selectmappart.xpm"),
                "Select map partition line", this);
  action->setStatusTip(tr("Select map partition line"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectMapPartitionMode()));
  //edit_tools->addAction(action);   
  
  // Mouse mode select model
  action = mode_actions[SelectModelMode] =
    new QAction(QIcon(":/buttons/selectmodel.xpm"), "Select model", this);
  action->setStatusTip(tr("Select model"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectModelMode()));
  //edit_tools->addAction(action);
  
  // Mouse mode select model part
  action = mode_actions[SelectModelPartMode] =
    new QAction(QIcon(":/buttons/selectmodelpart.xpm"), "Select model part",
                this);
  action->setStatusTip(tr("Select model part"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectModelPartMode()));
  //edit_tools->addAction(action);


  
  // Mouse mode select rectangular area on canvas
  action = mode_actions[SelectRectangleMode] =
    new QAction(QIcon(":/buttons/selectrect.xpm"), "Select area on canvas",
                this);
  action->setStatusTip(tr("Select area on canvas"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSelectRectangleMode()));
  edit_tools->addAction(action);
  
  // Mouse mode split map polygon
  action = mode_actions[SplitMapMode] =
    new QAction(QIcon(":/buttons/splitmap.xpm"), "Split map line", this);
  action->setStatusTip(tr("Split map line"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSplitMapMode()));
  edit_tools->addAction(action);
  
  
  // Mouse mode split map polygon
  action = mode_actions[SplitMapPartitionMode] =
    new QAction(QIcon(":/buttons/splitmappart.xpm"), "Select map partition",
                this);
  action->setStatusTip(tr("Split map partition"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetSplitMapPartitionMode()));
  //edit_tools->addAction(action);
  
  // Mouse mode extend line
  action = mode_actions[ExtendLineMode] =
    new QAction(QIcon(":/buttons/extendline.xpm"), "Extend line", this);
  action->setStatusTip(tr("Extend line"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetExtendLineMode()));
  edit_tools->addAction(action);
  
   // Mouse mode move point
  action = mode_actions[MoveNodeMode] =
    new QAction(QIcon(":/buttons/movenode.xpm"), "Move point", this);
  action->setStatusTip(tr("Move point"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetMoveNodeMode()));
  edit_tools->addAction(action);
  
  
    // Start mode is pose change TODO: move to end, not related to gui
  SetMode(PoseChangeMode);

  // End of mode buttons on edit toolbar
  edit_tools->addSeparator();
  
  
  
   // Delete a line
  action = new QAction(QIcon(":/buttons/deleteline.xpm"), "Delete line", this);
  action->setStatusTip(tr("Delete line"));
  connect(action, SIGNAL(triggered()), this, SLOT(DeleteLine()));
  edit_tools->addAction(action);
  
  
  // Crop to selected lines
  action = new QAction(QIcon(":/buttons/croplines.xpm"), "Crop lines", this);
  action->setStatusTip(tr("Crop lines"));
  connect(action, SIGNAL(triggered()), this, SLOT(CropLines()));
  edit_tools->addAction(action);
  
  
  // Delete the last edge of a line
  action = new QAction(QIcon(":/buttons/deletelastedge.xpm"),
                       "Delete last edge", this);
  action->setStatusTip(tr("Delete last edge"));
  connect(action, SIGNAL(triggered()), this, SLOT(DeleteLastEdge()));
  edit_tools->addAction(action);
  
  // Reverse the node order of a line
  action = new QAction(QIcon(":/buttons/reverseline.xpm"), "Reverse line",
                       this);
  action->setStatusTip(tr("Reverse line"));
  connect(action, SIGNAL(triggered()), this, SLOT(ReverseLine()));
  edit_tools->addAction(action);
  
  
  
  // Create a new line
  action = new QAction(QIcon(":/buttons/newline.xpm"), "Create new line", this);
  action->setStatusTip(tr("Create new line"));
  connect(action, SIGNAL(triggered()), this, SLOT(CreateNewLine()));
  edit_tools->addAction(action);
  
  
    // Close a line
  action = new QAction(QIcon(":/buttons/closeline.xpm"), "Close line", this);
  action->setStatusTip(tr("Close line"));
  connect(action, SIGNAL(triggered()), this, SLOT(CloseLine()));
  edit_tools->addAction(action);
 

  // Set a new start node of the selected line
  action = new QAction(QIcon(":/buttons/newstartnode.xpm"), "New start node",
                       this);
  action->setStatusTip(tr("New start node"));
  connect(action, SIGNAL(triggered()), this, SLOT(SetNewStartNode()));
  edit_tools->addAction(action);
 

  // Merge two map (partition) lines
  action = new QAction(QIcon(":/buttons/mergemap.xpm"),
                       "Merge map (partition) lines", this);
  action->setStatusTip(tr("Merge map (partition) lines"));
  connect(action, SIGNAL(triggered()), this, SLOT(MergeMapLines()));
  edit_tools->addAction(action);

  
  
  // Select laser data if a laser point has been selected on the canvas
  connect(Canvas(), SIGNAL(SelectedLaserPoint(const LaserPoint &, DataType)),
         this, SLOT(SelectLaserData(const LaserPoint &, DataType)));
            
}

void CityWindow::CreateShowToolBar()
{
  int data_type, image_type;
  QAction *action;

  show_tools = new QToolBar(tr("Show operations"), this);
  show_tools->setObjectName("Show data tools");
#ifdef linux
  show_tools->setIconSize(QSize(20, 20));
#else
  show_tools->setIconSize(QSize(18, 18));
#endif
  
  
  for (int i=0; i<NumDataTypes; i++) {
    appearance[i] = new DataAppearanceWindow((DataType) i);
  }
  
  
  
// Add show data actions to the show toolbar
//if(feature_type==None){
          addToolBar(Qt::LeftToolBarArea, show_tools);                                                                
                                                                  //    }
  // Show map data
  action = show_data_actions[MapData] =
    new QAction(QIcon(":/buttons/showmap.xpm"), tr("All map data"), this);
  action->setStatusTip(tr("Show map data"));
  connect(action, SIGNAL(triggered()), this, SLOT(ToggleMapDataDisplay()));
  if(feature_type==None)
  show_tools->addAction(action);

  // Show map partition data
  action = show_data_actions[MapPartitionData] =
    new QAction(QIcon(":/buttons/showmapparts.xpm"), tr("All map partition data"),
                this);
  action->setStatusTip(tr("Show map partition data"));
  connect(action, SIGNAL(triggered()),
          this, SLOT(ToggleMapPartitionDataDisplay()));
  
  if(feature_type==None)
  show_tools->addAction(action);

  // Show model data
  action = show_data_actions[ModelData] =
    new QAction(QIcon(":/buttons/showmodels.xpm"), tr("All model data"), this);
  action->setStatusTip(tr("Show model data"));
  connect(action, SIGNAL(triggered()), this, SLOT(ToggleModelDataDisplay()));
  if(feature_type==None)
  show_tools->addAction(action);

  // Show last model data
  if (window_type != CityView) {
    action = show_data_actions[LastModelData] =
      new QAction(QIcon(":/buttons/showlastmodel.xpm"),tr( "Last model data"),
                  this);
    action->setStatusTip(tr("Show last model data"));
    connect(action, SIGNAL(triggered()),
            this, SLOT(ToggleLastModelDataDisplay()));
    if(feature_type==None)
    show_tools->addAction(action);
  }

  // Show laser data
  action = show_data_actions[LaserData] =
    new QAction(QIcon(":/buttons/showlaser.xpm"), tr("Laser data"), this);
  action->setStatusTip(tr("Show laser data"));
  connect(action, SIGNAL(triggered()), this, SLOT(ToggleLaserDataDisplay()));
  if(feature_type==None)
  show_tools->addAction(action);

// Add show background image actions to the show toolbar

  // Show height texture
  action = show_background_actions[HeightImage] =
    new QAction(QIcon(":/buttons/showheight.xpm"), tr("Height image"), this);
  action->setStatusTip(tr("Show height texture"));
  connect(action, SIGNAL(triggered()), this, SLOT(ToggleHeightImageDisplay()));
  if(feature_type==None)
  show_tools->addAction(action);

  // Show shaded height texture
  action = show_background_actions[ShadedHeightImage] =
    new QAction(QIcon(":/buttons/showshaded.xpm"), tr("Shaded height image"), this);
  action->setStatusTip(tr("Show shaded height texture"));
  connect(action, SIGNAL(triggered()),
          this, SLOT(ToggleShadedHeightImageDisplay()));
  if(feature_type==None)
  show_tools->addAction(action);

  // Show ortho image texture
  action = show_background_actions[OrthoImage] =
    new QAction(QIcon(":/buttons/showortho.xpm"), tr("Ortho image"), this);
  action->setStatusTip(tr("Show ortho image"));
  connect(action, SIGNAL(triggered()), this, SLOT(ToggleOrthoImageDisplay()));
  if(feature_type==None)
  show_tools->addAction(action);

  // Make all above buttons toggle buttons and switch them all off
  for (data_type=0; data_type<NumNormalDataTypes; data_type++) {
    if (window_type != CityView || data_type != LastModelData) {
      show_data_actions[data_type]->setCheckable(true);
      show_data_actions[data_type]->setChecked(false);
    }
  }
  for (image_type=1; image_type<NumBackGroundTypes; image_type++) {
    show_background_actions[image_type]->setCheckable(true);
    show_background_actions[image_type]->setChecked(false);
  }

  // Two more buttons in case of a window with reconstructed model data
  if (window_type == CityResult) {
    // Saving reconstructed data and close the window
    action = new QAction(QIcon(":/buttons/acceptmodel.xpm"),
                         tr("Accept reconstructed model"), this);
    action->setStatusTip(tr("Accept reconstructed model"));
    connect(action, SIGNAL(triggered()),
            this, SLOT(AcceptReconstructedModel()));
    if(feature_type==None)
    show_tools->addAction(action);

    // Close the window without saving reconstructed data
    action = new QAction(QIcon(":/buttons/rejectmodel.xpm"),
                         "Reject reconstructed model", this);
    action->setStatusTip(tr("Reject reconstructed model"));
    connect(action, SIGNAL(triggered()), this, SLOT(close()));
    if(feature_type==None)
    show_tools->addAction(action);
  }

  // One more button for transfering data to the main window
  if (window_type == CityResult || window_type == CityView) {
    action = new QAction(QIcon(":/buttons/mainwindow.xpm"),
                         "Transfer data to main window", this);
    action->setStatusTip(tr("Transfer to main window"));
    connect(action, SIGNAL(triggered()), this, SLOT(MoveToMainWindow()));
    if(feature_type==None)
    show_tools->addAction(action);
  }

  
  
   
   
  // Select object data if an object point has been selected on the canvas
  connect(Canvas(), SIGNAL(SelectedObjectPoint(const PointNumber &,
                                               const Position3D &,
                                               DataType, DataType)),
          this, SLOT(SelectObjectData(const PointNumber &,
                                      const Position3D &,
                                      DataType, DataType)));
  // Select laser data if a laser point has been selected on the canvas
  //connect(Canvas(), SIGNAL(SelectedLaserPoint(const LaserPoint &, DataType)),
   //       this, SLOT(SelectLaserData(const LaserPoint &, DataType)));
   
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

            
  connect(Canvas(), SIGNAL(AttractPoint(const LaserPoint &)),
          this, SLOT(AttractMapPoint(const LaserPoint &)));   

}

void CityWindow::CreateViewToolBar()
{
  QAction     *action;

  view_tools = new QToolBar(tr("View operations"), this);
  view_tools->setObjectName("View tools");
#ifdef linux
  view_tools->setIconSize(QSize(20, 20));
#else
  view_tools->setIconSize(QSize(18, 18));
#endif
  addToolBar(Qt::RightToolBarArea, view_tools);
if (window_type == CityMain || window_type == CityResult){
  // Level view
  action = view_actions[ViewLevel] = 
    new QAction(QIcon(":/buttons/viewlevel.xpm"), "Level view", this);
  action->setStatusTip(tr("Level view"));
  connect(action, SIGNAL(triggered()), this, SLOT(LevelView()));
  view_tools->addAction(action);

  // Fit to all data
  action = view_actions[ViewFit] =
    new QAction(QIcon(":/buttons/viewfit.xpm"), "Fit view to data", this);
  action->setStatusTip(tr("Fit view to data"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitViewToData()));
  view_tools->addAction(action);

  // Fit to last model data
  action = view_actions[ViewFitLastModel] =
    new QAction(QIcon(":/buttons/viewfitlastmodel.xpm"), 
                "Fit view to reconstructed model", this);
  action->setStatusTip(tr("Fit view to reconstructed model"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitViewToLastModelData()));
  view_tools->addAction(action);

  // Fit to laser data
  action = view_actions[ViewFitLaser] =
    new QAction(QIcon(":/buttons/viewfitlaser.xpm"),
                "Fit view to laser data", this);
  action->setStatusTip(tr("Fit view to laser data"));
  connect(action, SIGNAL(triggered()), this, SLOT(FitViewToLaserData()));
  view_tools->addAction(action);
}
  
  if (window_type == CityResult ) {
  // Auto rotation
  if (window_type != CityMain) {
    action = new QAction(QIcon(":/buttons/autorotate.xpm"), "Auto rotate",
                         this);
    action->setStatusTip(tr("Auto rotation"));
    connect(action, SIGNAL(triggered()), this, SLOT(ToggleAutoRotationMode()));
    view_tools->addAction(action);
    action->setCheckable(true);
    action->setChecked(false);
    rotation_timer = NULL;
  }
  }
  

  
 if (feature_type == Extrusion ) 
 { 
     action = new QAction(QIcon(":/buttons/showlastmodel.xpm"), "Make extrusion",this);
    action->setStatusTip(tr("Make extrusion")); 
    connect(action, SIGNAL(triggered()), this, SLOT(EmitCreateExtru()));
   view_tools->addAction(action);
                  
                  }
 
 if (feature_type == RoofExtrusion ) 
 { 
     action = new QAction(QIcon(":/buttons/showlastmodel.xpm"), "Make complex extrusion",this);
    action->setStatusTip(tr("Make complex extrusion")); 
    connect(action, SIGNAL(triggered()), this, SLOT(SelectRoofExtrusion_complex()));
   view_tools->addAction(action);
   
   action = new QAction(QIcon(":/buttons/showlastmodel.xpm"), tr("Make rectangle extrusion"),this);
    action->setStatusTip(tr("Make rectangle extrusion")); 
    connect(action, SIGNAL(triggered()), this, SLOT(SelectRoofExtrusion_simple()));
   view_tools->addAction(action);
                  
                  }
 if (feature_type == Wall ) 
 { 
   action = new QAction(QIcon(":/buttons/mappolygon.xpm"), tr("Generate outline"),this);
    action->setStatusTip(tr("Generate outline")); 
    connect(action, SIGNAL(triggered()), this, SLOT(EmitCreateOutline()));
   view_tools->addAction(action);
    
   action = new QAction(QIcon(":/buttons/mappolygon.xpm"), tr("Improve outline"),this);
    action->setStatusTip(tr("Improve outline")); 
    connect(action, SIGNAL(triggered()), this, SLOT(EmitImproveOutline()));
   view_tools->addAction(action); 
  
}
 
// Show selected map data
  action = show_selection_actions[SelectedMapData] =
    new QAction("Selected map data", this);
  action->setCheckable(true);
  connect(action, SIGNAL(triggered()), this, SLOT(ShowSelectedMapData()));

  // Show selected map partition data
  action = show_selection_actions[SelectedMapPartitionData] =
    new QAction("Selected map partition data", this);
  action->setCheckable(true);
  connect(action, SIGNAL(triggered()),
          this, SLOT(ShowSelectedMapPartitionData()));
 
  // Show selected model data
  action = show_selection_actions[SelectedModelData] =
    new QAction("Selected model data", this);
  action->setCheckable(true);
  connect(action, SIGNAL(triggered()), this, SLOT(ShowSelectedModelData()));


  // Show selected model part data
  action = show_selection_actions[SelectedModelPartData] =
    new QAction("Selected model part data", this);
  action->setCheckable(true);
  connect(action, SIGNAL(triggered()), this, SLOT(ShowSelectedModelPartData()));

  // Show selected laser data
  //action = show_selection_actions[SelectedLaserData] =
  //  new QAction("Selected laser data", this);
  action = show_selection_actions[SelectedLaserData]=new QAction("Selected laser data", this);
  action->setCheckable(true);
  action->setChecked(false);
  connect(action, SIGNAL(triggered()), this, SLOT(ShowSelectedLaserData()));

 
  // Show selected point
  action = show_selection_actions[SelectedPointData] =
    new QAction("Selected point", this);
  action->setCheckable(true);
  action->setChecked(true);
  connect(action, SIGNAL(triggered()), this, SLOT(ToggleShowSelectedPoint()));
  
    show_selection_actions[SelectedPointData]->setChecked(true);
 
  
}

CityWindow::~CityWindow()
{
  // Stop the auto rotation, if active
  if (rotation_timer != NULL)
    if (rotation_timer->isActive()) rotation_timer->stop();

  // Delete local data
  if (!reconstructed_points.empty())
    reconstructed_points.erase(reconstructed_points.begin(),
                               reconstructed_points.end());
  reconstructed_part.DeleteData();
  laser_points.ErasePoints();

  // Notify PointCloudMapper
  if (window_type != CityMain) emit WindowWillBeClosed(this);
}

bool CityWindow::DataIsShown(DataType type)
{
  return canvas->HasData(type);
}

void CityWindow::ToggleDataDisplay(DataType type)
{
  bool update_bounds;

  if (show_data_actions[type]->isChecked()) { // Button already in new state
    if (window_type == CityResult && type == LastModelData) {
      update_bounds = !canvas->HasData();
      canvas->AddObjectData(&reconstructed_points,
                            reconstructed_part.RoofFaces(),
                            reconstruction_appearance, false, false);
      canvas->AddObjectData(&reconstructed_points,
                            reconstructed_part.WallFaces(),
                            reconstruction_appearance, false, false);
      if (update_bounds) canvas->InitialiseTransformation();
      canvas->update();
    }
    else if ((window_type == CityResult || window_type == CityView) &&
             type == LaserData) {
      if (!laser_points.empty()) {
        update_bounds = !canvas->HasData();
        canvas->AddLaserData(&laser_points, laserdata_appearance);
        if (update_bounds) canvas->InitialiseTransformation();
        canvas->update();
      }
      else
        show_data_actions[LaserData]->setChecked(false);
    }
    else {
      // Ask PointCloudMapper to add data to this window
      emit RequestDataDisplay(type, this);
    }
  }
  else {
    // Remove data from this window
    if (type == LaserData) {
      canvas->ClearLaserData(true);
    }
    else {
      canvas->RemoveObjectData(type, false);
      if (type <= ModelData)
        canvas->RemoveObjectData(canvas->CorrespondingSelectedDataType(type),
                                 false);
      if (type == ModelData)
        canvas->RemoveObjectData(SelectedModelPartData, false);
      canvas->update();
    }
    show_data_actions[type]->setChecked(false);
    if (window_type == CityMain) emit RemovedData(type);
  }
}

void CityWindow::ToggleImageDisplay(BackGroundType type)
{
  if (show_background_actions[type]->isChecked()) { // Button in new state
    // Set if off again, it will be switched on by PointCloudMapper if
    // the image is available.
    show_background_actions[type]->setChecked(false);
    // Ask PointCloudMapper to add the image to this window.
    emit RequestImageDisplay(type, this);
  }
  else {
//  if (window_type == CityMain) emit RemovedImage(type); Obsolete
    show_background_actions[type]->setChecked(false);
  }
}

void CityWindow::SetShowDataButton(DataType type, bool pressed)
{
//if ((pressed && !show_data_actions[type]->isChecked()) ||
//    (!pressed && show_data_actions[type]->isChecked()))
//  show_data_actions[type]->toggle();
  show_data_actions[type]->setChecked(pressed);
}

void CityWindow::SetShowImageButton(BackGroundType type, bool pressed)
{
//if ((pressed && !show_background_actions[type]->isChecked()) ||
//    (!pressed && show_background_actions[type]->isChecked()))
//  show_background_actions[type]->toggle();
  show_background_actions[type]->setChecked(pressed);
}

void CityWindow::AcceptReconstructedModel()
{
  emit RequestSavingReconstructedModel(this);
}

void CityWindow::AddReconstructedModel(const ObjectPoints &points,
                                      const BuildingPart &part,
                                      const DataAppearance *app)
{
  reconstructed_points      = points;
  reconstructed_part        = part;
  reconstruction_appearance = app;
  SetShowDataButton(LastModelData, true);
  ToggleDataDisplay(LastModelData);
}

void CityWindow::AddLaserData(const LaserPoints &points,
                             const DataAppearance *app,
                             bool show)
{
  laser_points = points;
  laserdata_appearance = app;
  SetShowDataButton(LaserData, show);
  ToggleDataDisplay(LaserData);
}

void CityWindow::ToggleAutoRotationMode()
{
  // Create a timer if it does not yet exist and connect it to
  // the auto rotate function
  if (rotation_timer == NULL) {
    rotation_timer = new QTimer(this);
    connect(rotation_timer, SIGNAL(timeout()), this, SLOT(IncrementRotation()));
  }

  // Switch auto rotation off
  if (rotation_timer->isActive()) rotation_timer->stop();
  // Switch auto rotation on
  else {
    rotation_timer->setSingleShot(false); // Repeat timer
    rotation_timer->start(100);           // every 0.1 second
  }
}

void CityWindow::IncrementRotation() { canvas->IncrementKappa(1.0, true); }


void CityWindow::SetMode(MouseMode new_mode)
{
  ObjectPoints       *point_data;
  LineTopsIterVector *selected_data;
  DataType           selection_type, base_type;
  MouseMode          old_mode, real_new_mode;
  bool               remove_old_selections=false;

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
  if (new_mode == SelectLaserSegmentMode &&
      old_mode == SelectLaserSegmentMode) {
    selected_laser_points.ErasePoints();
    ShowData(CityWindowPtr(), LaserData, true, SelectedLaserData, false, true,
             true);
    //ShowData(CityWindowPtr(), LaserData, true, SelectedLaserData, false, true, true);         
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
   
  if (new_mode == SelectPointMode) 
   real_new_mode = old_mode;
   
  // Update button and menu states
  for (int mode=0; mode<NumModes; mode++)
    mode_actions[SelectLaserSegmentMode]->setChecked(true);

  // Inform canvas about the new mode
  Canvas()->SetMode(real_new_mode);

}




void CityWindow::SelectRoofExtrusion_simple()

{
int index;

if (!selected_laser_points.size())
     return;

index=selected_laser_points[0].Attribute(SegmentNumberTag);

emit CreateRoofExtru(index,0);

}


void CityWindow::SelectRoofExtrusion_complex()

{
int index;

if (!selected_laser_points.size())
     return;

index=selected_laser_points[0].Attribute(SegmentNumberTag);

emit CreateRoofExtru(index,1);

}




void CityWindow::EmitCreateExtru()
{
     
     if (!selected_laser_points.size())
     return;
     
     emit CreateExtru();
}


void CityWindow::EmitCreateOutline()
{
     
     emit CreateOutline();
}

void CityWindow::EmitImproveOutline()
{
     
     emit ImproveOutlineSignal();
}

void CityWindow::ProcessShortCutKeys(QKeyEvent *event)
{
     double x,y,z;
     double step=0.05;
     ObjectPoints *objpts;
     ObjectPoint *obj, *obj_project;
     int index;
     Line3D line;
     int size;
     Vector2D direction;
    
     
  if (!Canvas()->ShiftKeyDown()) {
    switch (event->key()) {
       case Qt::Key_Plus: // Decrease laser point size   
        appearance[LaserData]->PointSizeEditor()->stepUp(); 
        break;
      case Qt::Key_Minus: // Decrease laser point size
        appearance[LaserData]->PointSizeEditor()->stepDown(); break;

      case Qt::Key_Up:
        if(!selected_point.size())   
        break;
        objpts=CorrespondingPoints(MapData);    
        index=objpts->PointIndexByNumber(selected_point[0].Number());      
        obj=(*objpts)[index].ObjectPointPtr () ;
        z=selected_point[0].Z();
        obj->SetZ(z+step);
        //move the projected together
        if(obj->Covar(0)!=0)
        {
        obj_project=obj+(int)obj->Covar(0);        
        obj_project->SetZ(z+step);
        }
        selected_point[0]=*obj;
        Canvas()->update();
        emit MapAdjusted(feature_type);
        break;
        
      case Qt::Key_Down:
        if(!selected_point.size())   
        break;
        objpts=CorrespondingPoints(MapData);    
        index=objpts->PointIndexByNumber(selected_point[0].Number());      
        obj=(*objpts)[index].ObjectPointPtr () ;
        z=selected_point[0].Z();
        obj->SetZ(z-step);
        //move the projected together
        if(obj->Covar(0)!=0)
        {
        obj_project=obj+(int)obj->Covar(0);        
        obj_project->SetZ(z-step);
        }
        
        selected_point[0]=*obj;
        Canvas()->update();
        emit MapAdjusted(feature_type);
        break;  
        
      case Qt::Key_Left:
        if(!selected_point.size())   
        break;
        
        objpts=CorrespondingPoints(MapData);    
        size=objpts->size();
        index=objpts->PointIndexByNumber(selected_point[0].Number());      
        obj=(*objpts)[index].ObjectPointPtr () ;
        x=selected_point[0].X();y=selected_point[0].Y();
        //line=Line3D((Position3D)((*objpts)[0]), (Position3D)((*objpts)[size-1]));
        direction=GBKN_local.Direction();
        obj->SetX(x-step*direction.X()); obj->SetY(y-step*direction.Y());
        
        //move the projected together
        if(obj->Covar(0)!=0)
        {
        obj_project=obj+(int)obj->Covar(0);  
        x=obj_project->GetX();y=obj_project->GetY();    
        obj_project->SetX(x-step*direction.X());
        obj_project->SetY(y-step*direction.Y());
        }
        
        selected_point[0]=*obj;
        Canvas()->update();
        emit MapAdjusted(feature_type);
        break;    
        
      case Qt::Key_Right:
        if(!selected_point.size())   
        break;
        
        objpts=CorrespondingPoints(MapData);    
        size=objpts->size();
        index=objpts->PointIndexByNumber(selected_point[0].Number());      
        obj=(*objpts)[index].ObjectPointPtr () ;
        x=selected_point[0].X();y=selected_point[0].Y();
        //line=Line3D((Position3D)((*objpts)[0]), (Position3D)((*objpts)[size-1]));
        direction=GBKN_local.Direction();
        obj->SetX(x+step*direction.X()); obj->SetY(y+step*direction.Y());
        
        //move the projected together
        if(obj->Covar(0)!=0)
        {
        obj_project=obj+(int)obj->Covar(0);  
        x=obj_project->GetX();y=obj_project->GetY();    
        obj_project->SetX(x+step*direction.X());
        obj_project->SetY(y+step*direction.Y());
        }
        
        selected_point[0]=*obj;
        Canvas()->update();
        emit MapAdjusted(feature_type);
        break;      
        
    
      
      case Qt::Key_C: // Crop laser data to selected area
        CropLaserData(); break;
      case Qt::Key_D: // Delete laser data in selected area
        DeleteLaserData(); break;
      
      case Qt::Key_R: // Remove a map point
        if(!selected_point.size())   
        break;       
        objpts=CorrespondingPoints(MapData);          
        index= selected_point[0].Number();  
        emit RemoveVertex(feature_type, index); 
        Canvas()->update();
        break;
      
      case Qt::Key_A: // Add a map point
        if(!selected_point.size())   
          break;
        objpts=CorrespondingPoints(MapData);             
        index= selected_point[0].Number(); 
        emit AddVertex(feature_type,index);
        Canvas()->update();
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


void CityWindow::InitialiseSplitLine(MouseMode mode)
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
      printf("Invalid mode %d in City::InitialiseSplitLine\n",
             mode);
      return;
  }

  Canvas()->InitialiseSplitLine(&map_points, line);
}

  void CityWindow::AttractMapPoint(const LaserPoint &point)
{
     ObjectPoints *objpts;
     ObjectPoint *obj;
     int index;
     double x, y, z;
     
 
     if(!selected_point.size())   
        return;
     objpts=CorrespondingPoints(MapData);    
     index=objpts->PointIndexByNumber(selected_point[0].Number());      
     obj=(*objpts)[index].ObjectPointPtr () ;
     obj->SetX(point.X()); obj->SetY(point.Y()); obj->SetZ(point.Z());
     selected_point[0]=*obj; 
     emit MapAdjusted(feature_type);
}
 
