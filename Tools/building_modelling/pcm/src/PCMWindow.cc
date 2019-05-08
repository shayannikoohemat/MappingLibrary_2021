
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


#include "PCMWindow.h"
#include <QToolBar>
#include <QSize>
#include "digphot_arch.h"

PCMWindow::PCMWindow(PCMWindowType type, QWidget *parent, Qt::WindowFlags flags) :
  QMainWindow(parent, flags)
{
  // Copy the window type
  window_type = type;

  // Create the canvas
  canvas = new PCMCanvas(0, this, window_type == PCMMain);
  canvas->setFocusPolicy(Qt::StrongFocus);
  setCentralWidget(canvas);

  // Create the show data toolbar
  CreateShowToolBar(); 

  // Create the view toolbar
  CreateViewToolBar();

  // Set window sizes
  canvas->setMinimumSize(200, 200);
  resize(300, 300);
  
  // Delete data in window after close
  if (window_type != PCMMain) setAttribute(Qt::WA_DeleteOnClose);
}

void PCMWindow::CreateShowToolBar()
{
  int data_type, image_type;
  QAction *action;

  show_tools = new QToolBar(tr("Show operations"), this);
  show_tools->setObjectName("Show data tools");
#ifdef linux
  show_tools->setIconSize(QSize(18, 18));
#else
  show_tools->setIconSize(QSize(18, 18));
#endif
  addToolBar(Qt::LeftToolBarArea, show_tools);
  
// Add show data actions to the show toolbar

  // Show map data
  action = show_data_actions[MapData] =
    new QAction(QIcon(":/buttons/showmap.xpm"), "All map data", this);
  action->setStatusTip(tr("Show map data"));
  connect(action, SIGNAL(triggered()), this, SLOT(ToggleMapDataDisplay()));
  show_tools->addAction(action);

  // Show map partition data
  action = show_data_actions[MapPartitionData] =
    new QAction(QIcon(":/buttons/showmapparts.xpm"), "All map partition data",
                this);
  action->setStatusTip(tr("Show map partition data"));
  connect(action, SIGNAL(triggered()),
          this, SLOT(ToggleMapPartitionDataDisplay()));
  show_tools->addAction(action);

  // Show model data
  action = show_data_actions[ModelData] =
    new QAction(QIcon(":/buttons/showmodels.xpm"), "All model data", this);
  action->setStatusTip(tr("Show model data"));
  connect(action, SIGNAL(triggered()), this, SLOT(ToggleModelDataDisplay()));
  show_tools->addAction(action);

  // Show last model data
  if (window_type != PCMCopiedDataView && window_type != PCMPyramidView &&
      window_type != PCMMainDataView) {
    action = show_data_actions[LastModelData] =
      new QAction(QIcon(":/buttons/showlastmodel.xpm"), "Last model data",
                  this);
    action->setStatusTip(tr("Show last model data"));
    connect(action, SIGNAL(triggered()),
            this, SLOT(ToggleLastModelDataDisplay()));
    show_tools->addAction(action);
  }

  // Show laser data
  action = show_data_actions[LaserData] =
    new QAction(QIcon(":/buttons/showlaser.xpm"), "Laser data", this);
  action->setStatusTip(tr("Show laser data"));
  connect(action, SIGNAL(triggered()), this, SLOT(ToggleLaserDataDisplay()));
  show_tools->addAction(action);

// Add show background image actions to the show toolbar

  if (window_type != PCMPyramidView) {
    // Show height texture
    action = show_background_actions[HeightImage] =
      new QAction(QIcon(":/buttons/showheight.xpm"), "Height image", this);
    action->setStatusTip(tr("Show height texture"));
    connect(action, SIGNAL(triggered()), this, SLOT(ToggleHeightImageDisplay()));
    show_tools->addAction(action);

    // Show shaded height texture
    action = show_background_actions[ShadedHeightImage] =
      new QAction(QIcon(":/buttons/showshaded.xpm"), "Shaded height image", this);  
    action->setStatusTip(tr("Show shaded height texture"));
    connect(action, SIGNAL(triggered()),
            this, SLOT(ToggleShadedHeightImageDisplay()));
    show_tools->addAction(action);

    // Show ortho image texture
    action = show_background_actions[OrthoImage] =
      new QAction(QIcon(":/buttons/showortho.xpm"), "Ortho image", this);
    action->setStatusTip(tr("Show ortho image"));
    connect(action, SIGNAL(triggered()), this, SLOT(ToggleOrthoImageDisplay()));
    show_tools->addAction(action);
  }

  // Make all above buttons toggle buttons and switch them all off
  for (data_type=0; data_type<NumNormalDataTypes; data_type++) {
    if ((window_type != PCMCopiedDataView && window_type != PCMPyramidView &&
	     window_type != PCMMainDataView) || data_type != LastModelData) {
      show_data_actions[data_type]->setCheckable(true);
      show_data_actions[data_type]->setChecked(false);
    }
  }
  if (window_type != PCMPyramidView) {
    for (image_type=1; image_type<NumBackGroundTypes; image_type++) {
      show_background_actions[image_type]->setCheckable(true);
      show_background_actions[image_type]->setChecked(false);
    }
  }

  // Three more buttons in case of a window with reconstructed model data
  if (window_type == PCMResult) {
    // Saving reconstructed data and close the window
    action = new QAction(QIcon(":/buttons/acceptmodel.xpm"),
                         "Accept reconstructed model", this);
    action->setStatusTip(tr("Accept reconstructed model"));
    connect(action, SIGNAL(triggered()),
            this, SLOT(AcceptReconstructedModel()));
    show_tools->addAction(action);

    // Edit reconstructed data in main window
    action = new QAction(QIcon(":/buttons/editmodel.xpm"),
                         "Edit reconstructed model", this);
    action->setStatusTip(tr("Edit reconstructed model"));
    connect(action, SIGNAL(triggered()),
            this, SLOT(EditReconstructedModel()));
    show_tools->addAction(action);

    // Close the window without saving reconstructed data
    action = new QAction(QIcon(":/buttons/rejectmodel.xpm"),
                         "Reject reconstructed model", this);
    action->setStatusTip(tr("Reject reconstructed model"));
    connect(action, SIGNAL(triggered()), this, SLOT(RejectReconstructedModel()));
    show_tools->addAction(action);
  }

  // Two more buttons for transfering data to the main window
  if (window_type == PCMResult || window_type == PCMCopiedDataView) {
    action = new QAction(QIcon(":/buttons/mainwindow.xpm"),
                         "Move data to main window", this);
    action->setStatusTip(tr("Move data to main window"));
    connect(action, SIGNAL(triggered()), this, SLOT(MoveToMainWindow()));
    show_tools->addAction(action);

    action = new QAction(QIcon(":/buttons/newwindow.xpm"),
                         "Add data to main window", this);
    action->setStatusTip(tr("Add data to main window"));
    connect(action, SIGNAL(triggered()), this, SLOT(AddToMainWindow()));
    show_tools->addAction(action);
  }
}

void PCMWindow::CreateViewToolBar()
{
  QAction     *action;

  view_tools = new QToolBar(tr("View operations"), this);
  view_tools->setObjectName("View tools");
#ifdef linux
  view_tools->setIconSize(QSize(18, 18));
#else
  view_tools->setIconSize(QSize(18, 18));
#endif
  addToolBar(Qt::RightToolBarArea, view_tools);

  // Level view (vertical view)
  action = view_actions[ViewLevel] = 
    new QAction(QIcon(":/buttons/viewlevel.xpm"), "Level view", this);
  action->setStatusTip(tr("Level view"));
  connect(action, SIGNAL(triggered()), this, SLOT(LevelView()));
  view_tools->addAction(action);

  // Horizontal view
  action = view_actions[ViewHorizontal] = 
    new QAction(QIcon(":/buttons/viewhorizontal.xpm"), "Horizontal view", this);
  action->setStatusTip(tr("Horizontal view"));
  connect(action, SIGNAL(triggered()), this, SLOT(HorizontalView()));
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

  // Auto rotation
  if (window_type != PCMMain) {
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

PCMWindow::~PCMWindow()
{
  // Stop the auto rotation, if active
  if (rotation_timer != NULL)
    if (rotation_timer->isActive()) rotation_timer->stop();

  // Delete local data
  if (!last_model_points.empty()) last_model_points.Erase();
  last_model_part.DeleteData();
  laser_points.ErasePoints();

  // Delete data appearance in case of a main data view window
  if (window_type == PCMMainDataView) delete laserdata_appearance;
  
  // Notify PointCloudMapper
  if (window_type != PCMMain) emit WindowWillBeClosed(this);
}

LaserPoints * PCMWindow::PointCloud()
{ if (window_type == PCMPyramidView || window_type == PCMMainDataView)
    return laser_points_main;
  return &laser_points;
}

bool PCMWindow::DataIsShown(DataType type)
{
  return canvas->HasData(type);
}

void PCMWindow::ToggleDataDisplay(DataType type, bool remove_old_last_model)
{
  bool update_bounds;
  LaserPoints *laser_points_for_display;

  if (show_data_actions[type]->isChecked()) { // Button already in new state
    if (window_type == PCMResult && type == LastModelData) {
      update_bounds = !canvas->HasData();
      if (remove_old_last_model)
        canvas->RemoveObjectData(last_model_appearance, false);
      canvas->AddObjectData(&last_model_points,
                            last_model_part.RoofFaces(),
                            last_model_appearance, false, false);
      canvas->AddObjectData(&last_model_points,
                            last_model_part.WallFaces(),
                            last_model_appearance, false, false);
      if (update_bounds) canvas->InitialiseTransformation();
      canvas->update();
    }
    else if ((window_type == PCMResult || window_type == PCMCopiedDataView ||
              window_type == PCMPyramidView || window_type == PCMMainDataView) &&
             type == LaserData) {
      if (window_type == PCMPyramidView || window_type == PCMMainDataView)
	    laser_points_for_display = laser_points_main;
      else laser_points_for_display = &laser_points;
      if (!laser_points_for_display->empty()) {
        update_bounds = !canvas->HasData();
        canvas->AddLaserData(laser_points_for_display, laserdata_appearance);
        if (update_bounds) {
          if (window_type == PCMPyramidView || window_type == PCMMainDataView) {
            laser_points_main->DeriveDataBounds(0);
            printf("New bounds for data in browse view\n");
            laser_points_main->DataBounds().Print();
          }
          canvas->InitialiseTransformation();
        }
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
    if (window_type == PCMMain) emit RemovedData(type);
  }
}

void PCMWindow::ToggleImageDisplay(BackGroundType type)
{
  if (show_background_actions[type]->isChecked()) { // Button in new state
    // Set if off again, it will be switched on by PointCloudMapper if
    // the image is available.
    show_background_actions[type]->setChecked(false);
    // Ask PointCloudMapper to add the image to this window.
    emit RequestImageDisplay(type, this);
  }
  else {
    canvas->ClearTextureData(true);
    show_background_actions[type]->setChecked(false);
  }
}

void PCMWindow::SetShowDataButton(DataType type, bool pressed)
{
  show_data_actions[type]->setChecked(pressed);
}

void PCMWindow::SetShowImageButton(BackGroundType type, bool pressed)
{
  show_background_actions[type]->setChecked(pressed);
}

void PCMWindow::AcceptReconstructedModel()
{
  emit RequestSavingReconstructedModel(this);
}

void PCMWindow::EditReconstructedModel()
{
  emit RequestEditingBuildingModel(this);
}

void PCMWindow::RejectReconstructedModel()
{
  delete this;
}

void PCMWindow::AddReconstructedModel(const ObjectPoints &points,
                                      const BuildingPart &part,
                                      const DataAppearance *app)
{
  last_model_points     = points;
  last_model_part       = part;
  last_model_part.AddModelData(&last_model_points);
  last_model_appearance = app;
  SetShowDataButton(LastModelData, true);
  ToggleDataDisplay(LastModelData, true);
}

void PCMWindow::AddLaserData(LaserPoints &points,
                             const DataAppearance *app,
                             bool show)
{
  if (window_type == PCMPyramidView ||
      window_type == PCMMainDataView) laser_points_main = &points;
  else laser_points = points;
  laserdata_appearance = app;
  SetShowDataButton(LaserData, show);
  ToggleDataDisplay(LaserData);
}

void PCMWindow::ToggleAutoRotationMode()
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

void PCMWindow::IncrementRotation() { canvas->IncrementKappa(1.0, true); }
