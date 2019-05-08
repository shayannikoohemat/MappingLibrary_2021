
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


#include "PCMWindow.h"
#include <QToolBar>
#include <QSize>
#include "digphot_arch.h"

PCMWindow::PCMWindow(PCMWindowType type, QWidget *parent, Qt::WFlags flags) :
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

  canvas->setMinimumSize(200, 200);
  resize(300, 300);
}

void PCMWindow::CreateShowToolBar()
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
  if (window_type != PCMView) {
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

  // Make all above buttons toggle buttons and switch them all off
  for (data_type=0; data_type<NumNormalDataTypes; data_type++) {
    if (window_type != PCMView || data_type != LastModelData) {
      show_data_actions[data_type]->setCheckable(true);
      show_data_actions[data_type]->setChecked(false);
    }
  }
  for (image_type=1; image_type<NumBackGroundTypes; image_type++) {
    show_background_actions[image_type]->setCheckable(true);
    show_background_actions[image_type]->setChecked(false);
  }

  // Two more buttons in case of a window with reconstructed model data
  if (window_type == PCMResult) {
    // Saving reconstructed data and close the window
    action = new QAction(QIcon(":/buttons/acceptmodel.xpm"),
                         "Accept reconstructed model", this);
    action->setStatusTip(tr("Accept reconstructed model"));
    connect(action, SIGNAL(triggered()),
            this, SLOT(AcceptReconstructedModel()));
    show_tools->addAction(action);

    // Close the window without saving reconstructed data
    action = new QAction(QIcon(":/buttons/rejectmodel.xpm"),
                         "Reject reconstructed model", this);
    action->setStatusTip(tr("Reject reconstructed model"));
    connect(action, SIGNAL(triggered()), this, SLOT(close()));
    show_tools->addAction(action);
  }

  // One more button for transfering data to the main window
  if (window_type == PCMResult || window_type == PCMView) {
    action = new QAction(QIcon(":/buttons/mainwindow.xpm"),
                         "Transfer data to main window", this);
    action->setStatusTip(tr("Transfer to main window"));
    connect(action, SIGNAL(triggered()), this, SLOT(MoveToMainWindow()));
    show_tools->addAction(action);
  }
}

void PCMWindow::CreateViewToolBar()
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
  if (!reconstructed_points.empty())
    reconstructed_points.erase(reconstructed_points.begin(),
                               reconstructed_points.end());
  reconstructed_part.DeleteData();
  laser_points.ErasePoints();

  // Notify PointCloudMapper
  if (window_type != PCMMain) emit WindowWillBeClosed(this);
}

bool PCMWindow::DataIsShown(DataType type)
{
  return canvas->HasData(type);
}

void PCMWindow::ToggleDataDisplay(DataType type)
{
  bool update_bounds;

  if (show_data_actions[type]->isChecked()) { // Button already in new state
    if (window_type == PCMResult && type == LastModelData) {
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
    else if ((window_type == PCMResult || window_type == PCMView) &&
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
//  if (window_type == PCMMain) emit RemovedImage(type); Obsolete
    show_background_actions[type]->setChecked(false);
  }
}

void PCMWindow::SetShowDataButton(DataType type, bool pressed)
{
//if ((pressed && !show_data_actions[type]->isChecked()) ||
//    (!pressed && show_data_actions[type]->isChecked()))
//  show_data_actions[type]->toggle();
  show_data_actions[type]->setChecked(pressed);
}

void PCMWindow::SetShowImageButton(BackGroundType type, bool pressed)
{
//if ((pressed && !show_background_actions[type]->isChecked()) ||
//    (!pressed && show_background_actions[type]->isChecked()))
//  show_background_actions[type]->toggle();
  show_background_actions[type]->setChecked(pressed);
}

void PCMWindow::AcceptReconstructedModel()
{
  emit RequestSavingReconstructedModel(this);
}

void PCMWindow::AddReconstructedModel(const ObjectPoints &points,
                                      const BuildingPart &part,
                                      const DataAppearance *app)
{
  reconstructed_points      = points;
  reconstructed_part        = part;
  reconstruction_appearance = app;
  SetShowDataButton(LastModelData, true);
  ToggleDataDisplay(LastModelData);
}

void PCMWindow::AddLaserData(const LaserPoints &points,
                             const DataAppearance *app,
                             bool show)
{
  laser_points = points;
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
