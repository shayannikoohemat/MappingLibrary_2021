
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


#include <vector>
#include <QApplication>
#include <QIcon>
#include <QToolBar>
#include <QMenuBar>
#include <QMenu>
#include <QFileDialog>
#include <QMessageBox>
#include <QAction>
#include <QStatusBar>
#include <QTableWidget>
#include <QCloseEvent>
#include <QFont>
#include <QFileInfo>
#include <algorithm>
#include "LaserPatch.h"
#include "LaserPatches.h"
#include "Position3D.h"
#include "Line2D.h"
#include "Lines2D.h"
#include "digphot_arch.h"
#include "City.h"
#include "CityCanvas.h"
#include "TerrestrialParWin.h"
#include <string>   
#include "cv.h"
#include "highgui.h"


using namespace std;

City::City()
    : CityWindow(CityMain, Primary, NULL, Qt::Window)
{
  int     initial_width=700, initial_height=700, i;
  QAction *action;
  
  convexhulled=false;
  GBKN_OK=false;
  
////////////////////////////////////////////////////////////////////////////////
// Data initialisations

  project_name = project_file = NULL;
  map_point_file = map_top_file = NULL;
  model_point_file = model_top_file = NULL;
  laser_meta_file = laser_point_file = NULL;
  save_needed = false;

////////////////////////////////////////////////////////////////////////////////
// Toolbar and menu for file operations

  QToolBar *file_tools = addToolBar(tr("File operations"));
  QMenu    *file_menu  = menuBar()->addMenu(tr("&File"));
#ifdef linux
  file_tools->setIconSize(QSize(20, 20));
#else
  file_tools->setIconSize(QSize(18, 18));
#endif
  // Open project
  action = new QAction(QIcon(":/buttons/fileopen.xpm"), tr("Open..."), this);
  action->setStatusTip(tr("Open laser file for a building"));
  action->setShortcut(Qt::CTRL + Qt::Key_O);
  connect(action, SIGNAL(triggered()), this, SLOT(SelectProject()));
  file_tools->addAction(action);
  file_menu->addAction(action);

  // Save sub menu
  QMenu *save_sub_menu = file_menu->addMenu(tr("Save"));

  // Save all project data
  action = new QAction(QIcon(":/buttons/filesave.xpm"), tr("Save DXF"), this);
  action->setStatusTip(tr("Save DXF"));
  action->setShortcut(Qt::CTRL + Qt::Key_S);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveDXF()));
  file_tools->addAction(action);
  save_sub_menu->addAction(action);

  // Save map data
  action = new QAction(tr("Map"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveMapData()));
  save_sub_menu->addAction(action);

  // Save map data as
  action = new QAction(tr("Map as..."), this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveMapDataAs()));
  save_sub_menu->addAction(action);


  // Save laser points as
  action = new QAction(tr("Laser points as..."), this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveLaserDataAs()));
  save_sub_menu->addAction(action);
  
  // Save selected laser points as
  action = new QAction(tr("Selected laser points as..."), this);
  connect(action, SIGNAL(triggered()), this, SLOT(SaveSelectedLaserDataAs()));
  save_sub_menu->addAction(action);
  
  // Import sub menu
  QMenu *import_sub_menu = file_menu->addMenu(tr("Import"));

  // Import map data
  action = new QAction(tr("Map..."), this);
  connect(action, SIGNAL(triggered()), this, SLOT(ImportMap()));
  import_sub_menu->addAction(action);
  
   // Import map data
  action = new QAction(tr("DXF Map..."), this);
  connect(action, SIGNAL(triggered()), this, SLOT(ImportDXFMap()));
  import_sub_menu->addAction(action);
  
  
  // Import laser points 
  action = new QAction(tr("Laser points..."), this);
  connect(action, SIGNAL(triggered()), this, SLOT(ImportLaserPoints()));
  import_sub_menu->addAction(action);
  
 
  // Import background image
  action = new QAction(tr("Background image..."), this);
  connect(action, SIGNAL(triggered()), this, SLOT(LoadBackGroundImage()));
  import_sub_menu->addAction(action);
  
  // Close and quit programme
  action = new QAction(tr("&Quit"), this);
  action->setShortcut(Qt::CTRL + Qt::Key_Q);
  connect(action, SIGNAL(triggered()), qApp, SLOT(closeAllWindows()));
  file_menu->addAction(action);


////////////////////////////////////////////////////////////////////////////////
// Toolbar and menu for edit operations

  
  QMenu    *edit_menu  = menuBar()->addMenu(tr("&Edit"));

  // Make all above buttons toggle buttons and switch them all off
  for (int mode=0; mode<NumModes; mode++) {
    mode_actions[mode]->setCheckable(true);
    mode_actions[mode]->setChecked(false);
  }



//------------------------------------------------------------------------------
// Edit appearance sub menu

  QMenu *appear_sub_menu = edit_menu->addMenu(tr("Appearence of"));

  // Sub menu items and window creation for data appearances
  for (i=0; i<NumDataTypes; i++) {
    //appearance[i] = new DataAppearanceWindow((DataType) i);
    action = new QAction(QString(appearance[i]->DataTypeName()) + tr("data..."),
                         this);
    connect(action, SIGNAL(triggered()), appearance[i], SLOT(Edit()));
    appear_sub_menu->addAction(action);
  }

  // Special appearance for laser data in result window
  result_appearance = new DataAppearanceWindow(LaserData);
  action = new QAction(tr("Laser data in result view..."), this);
  connect(action, SIGNAL(triggered()), result_appearance, SLOT(Edit()));
  appear_sub_menu->addAction(action);
  
  // Background colour
  action = new QAction(tr("Background..."), this);
  connect(action, SIGNAL(triggered()), this, SLOT(ChangeBackGroundColour()));
  appear_sub_menu->addAction(action);

//------------------------------------------------------------------------------
// Laser data stuff

  
  edit_menu->addAction(action);

  
  edit_menu->addAction(action);


  
////////////////////////////////////////////////////////////////////////////////
// Toolbar and menu for show operations
// Additions to toolbar for show operations

  // QToolBar show_tools already created by CityWindow

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



  for (int i=0; i<NumNormalDataTypes; i++)
    show_data_actions[i]->setChecked(false);

  for (int i=SelectedMapData; i<=SelectedModelPartData; i++)
    show_selection_actions[i]->setChecked(true);




////////////////////////////////////////////////////////////////////////////////
// Menu for segmentation operations

  QMenu *segmentation_menu  = menuBar()->addMenu(tr("Se&gmentation"));
/*
  // Connected component segmentation
  action = new QAction(QIcon(":/buttons/segmentlaser.xpm"),
                       "Connected components", this);
  connect(action, SIGNAL(triggered()), this, SLOT(SegmentLaserData()));
  segmentation_menu->addAction(action);
*/  
  // Surface growing
  action = new QAction(tr("Surface growing"), this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(GrowSurfaces()));
  segmentation_menu->addAction(action);

  // Unify label of selected laser data
  action = new QAction(tr("Unify segment number of selected laser data"), this);
  connect(action, SIGNAL(triggered()),
          this, SLOT(UnifySegmentNumberSelectedLaserData()));
  segmentation_menu->addAction(action);

  // Remove small segments
  action = new QAction(tr("Remove small segments"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(RemoveSmallSegments()));
  segmentation_menu->addAction(action);

  // Edit segmentation parameters
  segmentation_parameters = new SegmentationParametersWindow(NULL);
  segmentation_parameters->setWindowTitle(tr("Segmentation parameters"));
  action = new QAction(tr("Edit segmentation parameters"), this);
  connect(action, SIGNAL(triggered()), segmentation_parameters, SLOT(show()));
  segmentation_menu->addAction(action);



////////////////////////////////////////////////////////////////////////////////
// Menu for Pu's City Modeller

  QMenu *preprocess_menu  = menuBar()->addMenu(tr("&Preprocess"));
  //
 /*
  action = new QAction("&Set source file", this);
  connect(action, SIGNAL(triggered()), this, SLOT(Set_Source_File()));
  preprocess_menu->addAction(action); 
  */
  //
  action = new QAction(tr("&Set strip metadata file"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(Set_Tile_Meta_File()));
  preprocess_menu->addAction(action); 
  
  
  // select reconstruction mapline
  action = new QAction(tr("&Select reconstruction mapline"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(Select_reconstruct_map()));
  preprocess_menu->addAction(action); 
  /*
  //select reconstruciton region
  action = new QAction("&Select reconstruction region", this);
  connect(action, SIGNAL(triggered()), this, SLOT(Select_reconstruct_region()));
  preprocess_menu->addAction(action); 
  */
  //select map from laser
  action = new QAction(tr("&Cut ground level laser to make mapline"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(Create_map_from_laser()));
  preprocess_menu->addAction(action); 


  QMenu *terr_menu  = menuBar()->addMenu(tr("&Terrestrial"));

  
  // Find Feature
  action = new QAction(tr("&Find Feature"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(FindFeature()));
  terr_menu->addAction(action); 
  
  // Derive contour
  //action = new QAction("&Derive Contour", this);
  //connect(action, SIGNAL(triggered()), this, SLOT(DeriveContour()));
  //terr_menu->addAction(action); 
  /*
   // Fit outline
  action = new QAction("&Generate Outline", this);
  connect(action, SIGNAL(triggered()), this, SLOT(FitWallLine()));
  terr_menu->addAction(action); 
  
  // Fit outline
 // action = new QAction("&Show final model", this);
 // connect(action, SIGNAL(triggered()), this, SLOT(fillback()));
 // terr_menu->addAction(action); 
  */
  // Find Window
  action = new QAction(tr("&Detect Window(Optional)"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(FindWindow()));
  terr_menu->addAction(action); 
  
  // Final model
  action = new QAction(tr("&Final model"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(CreateFinalModel()));
  terr_menu->addAction(action); 
  
  terr_menu->addSeparator ();

  // Edit segmentation parameters
  terrestrial_parameters = new TerrestrialParametersWindow(NULL);
  terrestrial_parameters->setWindowTitle(tr("Terrestrial related parameters"));
  action = new QAction(tr("Terrestrial related parameters"), this);
  connect(action, SIGNAL(triggered()), terrestrial_parameters, SLOT(show()));
  terr_menu->addAction(action);

  
  QMenu *image_menu  = menuBar()->addMenu(tr("&Image Processing"));
   
  
  
  // Register
  action = new QAction(tr("&Register"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(ImportImageQt()));
  image_menu->addAction(action);
  
  // Generate image
  action = new QAction(tr("&Generate image"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(GenerateImage()));
  image_menu->addAction(action);

  //Improvemodel()
  action = new QAction(tr("&Improve model"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(Improvemodel()));
  image_menu->addAction(action);
////////////////////////////////////////////////////////////////////////////////
// Menu for help operations

  QMenu *help_menu  = menuBar()->addMenu(tr("&Help"));

  // About City
  action = new QAction(tr("&About"), this);
  action->setShortcut(Qt::Key_F1);
  connect(action, SIGNAL(triggered()), this, SLOT(about()));
  help_menu->addAction(action); 

  // About Qt
  action = new QAction(tr("About Qt"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(aboutQt()));
  help_menu->addAction(action); 

////////////////////////////////////////////////////////////////////////////////

  statusBar()->showMessage(tr("Ready"), 2000);
  resize(initial_width, initial_height + 150);

////////////////////////////////////////////////////////////////////////////////
// Connect signals and slots

  // Default connectors for showing data in CityWindows
  
  // Update show menu if data is removed by CityWindow
  connect(CityWindowPtr(), SIGNAL(RemovedData(DataType)),
          this, SLOT(UpdateShowDataMenu(DataType)));
/* TODO: Remove obsolete code
  // Update show menu if image is removed by CityWindow
  connect(CityWindowPtr(), SIGNAL(RemovedImage(BackGroundType)),
          this, SLOT(UpdateShowImageMenu(BackGroundType)));
*/
   wall_win = new CityWindow(CityView, Wall, this, Qt::Window);
   ground_win = new CityWindow(CityView, Ground, this, Qt::Window);
   door_win = new CityWindow(CityView, Door, this, Qt::Window);
   roof_win = new CityWindow(CityView, Roof, this, Qt::Window);
   roof_extru_win = new CityWindow(CityView, RoofExtrusion, this, Qt::Window);
   extrusion_win = new CityWindow(CityView, Extrusion, this, Qt::Window);
   window_win = new CityWindow(CityView, Window, this, Qt::Window);  
   
    
   wall_win->setWindowTitle(tr( "Wall zone") );
   
   ground_win->setWindowTitle( tr("Ground zone") );
   
   door_win->setWindowTitle( tr("Door zone") );
   //door_win->Canvas()->SetBackGroundColour(Qt::white);
   
   roof_win->setWindowTitle( tr("Roof zone") );
   //roof_win->Canvas()->SetBackGroundColour(Qt::white);
   
   roof_extru_win->setWindowTitle( tr("Roof extrusion zone") );
   //roof_extru_win->Canvas()->SetBackGroundColour(Qt::white);
   
   extrusion_win->setWindowTitle( tr("Extrusion zone") );
   //extrusion_win->Canvas()->SetBackGroundColour(Qt::white);
   
   window_win->setWindowTitle( tr("Window zone") );
   
  
    
   subwindows.push_back(wall_win); 
   subwindows.push_back(ground_win);
   subwindows.push_back(door_win);
   subwindows.push_back(roof_win);
   subwindows.push_back(roof_extru_win);
   subwindows.push_back(extrusion_win);
   subwindows.push_back(window_win);  
   
   
   
   
   //connect laser in sub windows with global variable
   connect(wall_win->CityWindowPtr(), SIGNAL(LaserChanged(FeatureWindowType)),
          this, SLOT(ChangeLaser(FeatureWindowType)));
   connect(ground_win->CityWindowPtr(), SIGNAL(LaserChanged(FeatureWindowType)),
          this, SLOT(ChangeLaser(FeatureWindowType)));
   connect(door_win->CityWindowPtr(), SIGNAL(LaserChanged(FeatureWindowType)),
          this, SLOT(ChangeLaser(FeatureWindowType)));
   connect(roof_win->CityWindowPtr(), SIGNAL(LaserChanged(FeatureWindowType)),
          this, SLOT(ChangeLaser(FeatureWindowType)));
   connect(roof_extru_win->CityWindowPtr(), SIGNAL(LaserChanged(FeatureWindowType)),
          this, SLOT(ChangeLaser(FeatureWindowType)));
   connect(extrusion_win->CityWindowPtr(), SIGNAL(LaserChanged(FeatureWindowType)),
          this, SLOT(ChangeLaser(FeatureWindowType)));
   connect(window_win->CityWindowPtr(), SIGNAL(LaserChanged(FeatureWindowType)),
          this, SLOT(ChangeLaser(FeatureWindowType)));
          
   
   
   //map changed       
   connect(wall_win->CityWindowPtr(), SIGNAL(MapAdjusted(FeatureWindowType)),
          this, SLOT(MoveMapPoint(FeatureWindowType)));    
          
   connect(wall_win->CityWindowPtr(), SIGNAL(RemoveVertex(FeatureWindowType, int)),
          this, SLOT(RemoveVertexPoint(FeatureWindowType, int)));    
   
   connect(wall_win->CityWindowPtr(), SIGNAL(AddVertex(FeatureWindowType, int)),
          this, SLOT(AddVertexPoint(FeatureWindowType, int)));             
          
   connect(extrusion_win->CityWindowPtr(), SIGNAL(MapAdjusted(FeatureWindowType)),
          this, SLOT(MoveMapPoint(FeatureWindowType)));       
          
   connect(roof_extru_win->CityWindowPtr(), SIGNAL(MapAdjusted(FeatureWindowType)),
          this, SLOT(MoveMapPoint(FeatureWindowType)));    
     
   connect(window_win->CityWindowPtr(), SIGNAL(MapAdjusted(FeatureWindowType)),
          this, SLOT(MoveMapPoint(FeatureWindowType)));    
          
   connect(door_win->CityWindowPtr(), SIGNAL(MapAdjusted(FeatureWindowType)),
          this, SLOT(MoveMapPoint(FeatureWindowType)));           
          
    connect(extrusion_win->CityWindowPtr(), SIGNAL(MapAdjusted(FeatureWindowType)),
          this, SLOT(MoveMapPoint(FeatureWindowType)));     
          
     connect(CityWindowPtr(), SIGNAL(MapAdjusted(FeatureWindowType)),
          this, SLOT(MoveMapPoint(FeatureWindowType)));                       
          
   //Update WallOutline         
   has_outline=false;
   connect(wall_win->CityWindowPtr(), SIGNAL(CreateOutline()),
          this, SLOT(FitWallLine()));
          
    connect(wall_win->CityWindowPtr(), SIGNAL(ImproveOutlineSignal()),
          this, SLOT(ImproveWallOutline()));       
          
   //connect show laser segment information Slot      
   connect(wall_win->CityWindowPtr(), SIGNAL(Show_segment_info(int,FeatureWindowType)),
   this, SLOT(Show_Info(int,FeatureWindowType)));
   
   connect(ground_win->CityWindowPtr(), SIGNAL(Show_segment_info(int,FeatureWindowType)),
   this, SLOT(Show_Info(int,FeatureWindowType)));
   
   connect(door_win->CityWindowPtr(), SIGNAL(Show_segment_info(int,FeatureWindowType)),
   this, SLOT(Show_Info(int,FeatureWindowType)));
   
   connect(roof_win->CityWindowPtr(), SIGNAL(Show_segment_info(int,FeatureWindowType)),
   this, SLOT(Show_Info(int,FeatureWindowType)));
   
   connect(extrusion_win->CityWindowPtr(), SIGNAL(CreateExtru()),
   this, SLOT(CreateExtrusion()));
   
   connect(roof_extru_win->CityWindowPtr(), SIGNAL(Show_segment_info(int,FeatureWindowType)),
   this, SLOT(Show_Info(int,FeatureWindowType)));
   
   connect(roof_extru_win->CityWindowPtr(), SIGNAL(CreateRoofExtru(int,int)),
   this, SLOT(CreateRoofExtrusion(int,int)));
   
   
   connect(extrusion_win->CityWindowPtr(), SIGNAL(Show_segment_info(int,FeatureWindowType)),
   this, SLOT(Show_Info(int,FeatureWindowType)));
   
   connect(CityWindowPtr(), SIGNAL(Show_segment_info(int,FeatureWindowType)),
          this, SLOT(Show_Info(int,FeatureWindowType)));
          
   connect(Canvas(), SIGNAL(pop_menu(const QPoint &)),
         this, SLOT(CreateCustomMenu(const QPoint &))); 
         
   // Display information on a selected laser point
  connect(Canvas(), SIGNAL(RequestForPointInformation(const LaserPoint &)),
          this, SLOT(DisplayPointInformation(const LaserPoint &)));      
           
  //Add Tie Point
  connect(Canvas(), SIGNAL(AddCanvasTiePoint(const LaserPoint &)),
          this, SLOT(AddTiePoint(const LaserPoint &)));  
          
  connect(Canvas(), SIGNAL(DeleteCanvasTiePoint()),
           this, SLOT(DelTiePoint()));  
  
  result_appearance->SetDataTypeName("Laser result data");
  result_appearance->SetPointColourMethod(ColourByResidual);
  appearance[SelectedLaserData]->SetPointColourMethod(ColourBySegment);
  
  
 
  //CreateCustomMenu();
  cyclorama=new ImageViewer(this);      
  cyclorama->setHidden(true);
   texture_exist=false; exter_exist=false;
 
  
}





void CityWindow::ToggleShowData(DataType type, bool refresh)
{
// This is no longer toggling. The menu or button selection already cause
// the action to change it's state. So, we just display what the current
// check states indicate.

  DataType base_type, selection_type;

  if (type == LaserData) {
    // Toggle display of laser data
    ShowData(CityWindowPtr(), type, show_data_actions[LaserData]->isChecked(),
             type, show_data_actions[LaserData]->isChecked(), refresh);
  }
  else if (type == SelectedLaserData) {
    // Toggle display of selected laser data
    base_type = LaserData;
    ShowData(CityWindowPtr(), base_type,
             show_data_actions[base_type]->isChecked(), type,
             show_selection_actions[type]->isChecked(), refresh);
  }
  else if ((int) type < NumObjectDataTypes) {
    // Toggle display of all data, including selected data
    selection_type = Canvas()->CorrespondingSelectedDataType(type);
    ShowData(CityWindowPtr(), type, show_data_actions[type]->isChecked(),
             selection_type, show_data_actions[type]->isChecked(), refresh);
  }
  else { // Selected object data
    // Toggle display of selected data only
    base_type = Canvas()->CorrespondingBaseDataType(type);
    ShowData(CityWindowPtr(), base_type,
             show_data_actions[base_type]->isChecked(), type,
             show_selection_actions[type]->isChecked(), refresh);
  }
}

void City::DisplayData(DataType type, CityWindow *window,
                                   bool data_changed)
{
                                    printf("display data\n");
  ShowData(window, type, true,
           window->Canvas()->CorrespondingSelectedDataType(type), true,
           true, data_changed);
}

void CityWindow::ShowData(CityWindow *window, DataType base_type,
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
  vector <CityWindow *>::iterator           subwindow;



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
      (window->Type() != CityMain ||
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
    if (selection_type != base_type && window->Type() == CityMain) {
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
  if (window->Type() == CityMain || window->Type() == CityView) { // Only for City window
    if (selection_type != LastModelData &&
        selection_type != LaserData) { // Ignore double display of LaserData
       
      show_selection_actions[selection_type]->setChecked(show_selected_data);
   
    }
  }

  window->SetShowDataButton(base_type, show_base_data);
   
  
  // If shared data was changed in the main window, also update all other
  // windows. LastModelData and LaserData are not shared.
  if (data_changed && window->Type() == CityMain && base_type != LastModelData &&
      base_type != LaserData) {
    for (subwindow=subwindows.begin(); subwindow!=subwindows.end();
         subwindow++) {
      if ((*subwindow)->DataIsShown(base_type))
        ShowData(*subwindow, base_type, true, selection_type, false,
                 true, true);
    }
  }
}

void City::UpdateShowDataMenu(DataType type)
{
  // Switch off the corresponding show selection actions
  if (type <= ModelData)
    show_selection_actions[Canvas()->CorrespondingSelectedDataType(type)]->
      setChecked(false);
  if (type == ModelData)
    show_selection_actions[SelectedModelPartData]->setChecked(false);
}

void CityWindow::ToggleShowSelectedPoint()
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

void City::SelectProject()
{
  
  QString filename =
    QFileDialog::getOpenFileName(this, "Select laser file", QString::null,
                    "Facade TLS data (*.laser);;All files (*.*)");
  if (filename.isEmpty()) return;
  
  SetLaserPointFile(filename.toAscii());
  SetCurrentDirectory(&filename);
  
  QString qmap_obj(current_dir);
  QString qmap_top(current_dir);
  
  if (laser_points.size()) laser_points.ErasePoints();
  laser_points.Read(filename.toAscii());
  
  DataBoundsLaser db;
  db=laser_points.DeriveDataBounds(0);
  terrestrial_parameters->street_center_height=db.Minimum().GetZ()+2;
  
    vector<int> indices=laser_points.GetAttribute(SegmentNumberTag);
    max_segment_value=0;  
   for(int i=0;i<indices.size();i++)
      max_segment_value=(max_segment_value>indices[i])?max_segment_value:indices[i];                                                      
  
  if (!laser_points.size())
    return;
    
    ShowData(CityWindowPtr(), LaserData, true, SelectedLaserData, false, true);
    
    //derive region of interest in cyclorama based on mapline 
      Position3D left3d, right3d;
    ImportImageQt();
    
      qmap_obj=current_dir; qmap_obj.append("/map.objpts");
      qmap_top=current_dir; qmap_top.append("/map.top");
   
      GBKN_OK=map_obj.Read(qmap_obj.toAscii());
      map_top.Read(qmap_top.toAscii());
    
      if(!GBKN_OK)
        return;
    
          left_map=(Position2D)map_obj.PointByNumber(map_top[0][0].Number());
          right_map=(Position2D)map_obj.PointByNumber(map_top[0][1].Number());
          left3d=Position3D(left_map.GetX(), left_map.GetY(), 0);
          right3d=Position3D(right_map.GetX(), right_map.GetY(),0);
          GBKN=Line2D(left_map,right_map);
          GBKN3D=Line3D(left3d, right3d);
        
     Position2D pt((left_map.GetX()+right_map.GetX())/2,(left_map.GetY()+right_map.GetY())/2);
     cyclorama->center_point=pt;
     cyclorama->left_bound=(Position2D)left_map;
     cyclorama->right_bound=(Position2D)right_map;
     cyclorama->current_dir=current_dir;
    
}


void City::SelectProject(char *filename)
{
  QString qfilename; 
  SetLaserPointFile(filename);
  qfilename=QDir::currentPath()+QString(filename);
  SetCurrentDirectory(&qfilename);
  
  if (laser_points.size()) laser_points.ErasePoints();
  laser_points.Read(filename);
  laser_points.DeriveDataBounds(0);
  if(laser_points[0].HasAttribute(SegmentNumberTag))
    {
     vector<int> indices=laser_points.GetAttribute(SegmentNumberTag);
    max_segment_value=0;  
   for(int i=0;i<indices.size();i++)
      max_segment_value=(max_segment_value>indices[i])?max_segment_value:indices[i];                                                      
    }
  
  if (laser_points.size())
    ShowData(CityWindowPtr(), LaserData, true, SelectedLaserData, false, true);  
    
     }



bool City::SaveDXF()
{
    //Create outline
    /*
    int size, offset,max;
    
    ObjectPoints outline_dxf_obj;
    LineTopology top;
    LineTopologies outline_dxf_tops;
    ObjectPoint objpt;
    size=outline_obj.size();
    
     
    if (size==0)
    {
                   QMessageBox::information(this, "Error",
                               "There are no final model yet.\n");
                               return  false;
    }
  
    
     
    outline_dxf_obj=outline_obj;
    outline_dxf_tops=outline_tops;  
    
    max=-32768; 
    for(int m=0;m<outline_dxf_obj.size();m++)
      max=outline_dxf_obj[m].Number()>max?outline_dxf_obj[m].Number():max;
      
    offset=max+1;
    
    //estimated outline
    for(int m=0;m<outline_estimated_obj.size();m++)
    {
            objpt=outline_estimated_obj[m];
            objpt.Number()=objpt.Number()+offset;
            outline_dxf_obj.push_back(objpt);
    }
    
    for(int m=0;m<outline_estimated_tops.size();m++)
    {
            top=outline_estimated_tops[m];
             for(int j=0;j<top.size();j++)
               top[j].Number()=top[j].Number()+offset;
            top.Label()=Wall;
            outline_dxf_tops.push_back(top);
    }
   
    //roof
    max=-32768; 
    for(int m=0;m<outline_dxf_obj.size();m++)
      max=outline_dxf_obj[m].Number()>max?outline_dxf_obj[m].Number():max;
      
    offset=max+1;
   // size=outline_dxf_obj.size();
    
    for(int m=0;m<roof_triangle_obj.size();m++)
    {
            objpt=roof_triangle_obj[m];
            objpt.Number()=objpt.Number()+offset;
            outline_dxf_obj.push_back(objpt);
    }
    
    for(int m=0;m<roof_triangle_tops.size();m++)
    {
            top=roof_triangle_tops[m];
             for(int j=0;j<top.size();j++)
               top[j].Number()=top[j].Number()+offset;
            top.Label()=Roof;
            outline_dxf_tops.push_back(top);
    }
    
    //door
    max=-32768; 
    for(int m=0;m<outline_dxf_obj.size();m++)
      max=outline_dxf_obj[m].Number()>max?outline_dxf_obj[m].Number():max;
      
    offset=max+1;
    //size=outline_dxf_obj.size();
    
    for(int m=0;m<door_obj.size();m++)
    {
            objpt=door_obj[m];
            objpt.Number()=objpt.Number()+offset;
            outline_dxf_obj.push_back(objpt);
    }
    
    for(int m=0;m<door_tops.size();m++)
    {
            top=door_tops[m];
             for(int j=0;j<top.size();j++)
               top[j].Number()=top[j].Number()+offset;
            top.Label()=Door;
            outline_dxf_tops.push_back(top);
    }
    
    //window
    max=-32768; 
    for(int m=0;m<outline_dxf_obj.size();m++)
      max=outline_dxf_obj[m].Number()>max?outline_dxf_obj[m].Number():max;
      
    offset=max+1;
    //size=outline_dxf_obj.size();
    
    for(int m=0;m<window_obj.size();m++)
    {
            objpt=window_obj[m];
            objpt.Number()=objpt.Number()+offset;
            outline_dxf_obj.push_back(objpt);
    }
    
    for(int m=0;m<window_tops.size();m++)
    {
            top=window_tops[m];
             for(int j=0;j<top.size();j++)
               top[j].Number()=top[j].Number()+offset;
            top.Label()=Window;
            outline_dxf_tops.push_back(top);
    }
    
    //Extrusion
    max=-32768; 
    for(int m=0;m<outline_dxf_obj.size();m++)
      max=outline_dxf_obj[m].Number()>max?outline_dxf_obj[m].Number():max;
      
    offset=max+1;
    //size=outline_dxf_obj.size();
    
    for(int m=0;m<cwindow_objpts.size();m++)
    {
            objpt=cwindow_objpts[m];
            objpt.Number()=objpt.Number()+offset;
            outline_dxf_obj.push_back(objpt);
    }
    
    for(int m=0;m<cwindow_tops.size();m++)
    {
            top=cwindow_tops[m];
            for(int j=0;j<top.size();j++)
               top[j].Number()=top[j].Number()+offset;
            top.Label()=Extrusion;
            outline_dxf_tops.push_back(top);
    }
    
    // RoofExtrusion
    max=-32768; 
    for(int m=0;m<outline_dxf_obj.size();m++)
      max=outline_dxf_obj[m].Number()>max?outline_dxf_obj[m].Number():max;
      
    offset=max+1;
    //size=outline_dxf_obj.size();
    
    for(int m=0;m<roof_extru_obj.size();m++)
    {
            objpt=roof_extru_obj[m];
            objpt.Number()=objpt.Number()+offset;
            outline_dxf_obj.push_back(objpt);
    }
    
    for(int m=0;m<roof_extru_tops.size();m++)
    {
            top=roof_extru_tops[m];
             for(int j=0;j<top.size();j++)
               top[j].Number()=top[j].Number()+offset;
            top.Label()=RoofExtrusion;
            outline_dxf_tops.push_back(top);
    }
    //Gap between roof and wall
    max=-32768; 
    for(int m=0;m<outline_dxf_obj.size();m++)
      max=outline_dxf_obj[m].Number()>max?outline_dxf_obj[m].Number():max;
      
    offset=max+1;
    //size=outline_dxf_obj.size();
    
    for(int m=0;m<gap_obj.size();m++)
    {
            objpt=gap_obj[m];
            objpt.Number()=objpt.Number()+offset;
            outline_dxf_obj.push_back(objpt);
    }
    
    for(int m=0;m<gap_tops.size();m++)
    {
            top=gap_tops[m];
             for(int j=0;j<top.size();j++)
               top[j].Number()=top[j].Number()+offset;
            top.Label()=Gap;
            outline_dxf_tops.push_back(top);
    }
    
  
    
    
    cout<<outline_dxf_obj.size()<<" "<<outline_dxf_tops.size()<<endl;
    */
   // outline_dxf_obj.Write("outline_dxf.objpts");outline_dxf_tops.Write("outline_dxf.top");
    
    QString filename;
    
    filename = QFileDialog::getSaveFileName(this, "Save DXF file as ..",
                                          laser_points.PointFile(),
                                     "Laser points (*.dxf);;All files (*.*)");
    
    FILE *dxffile=fopen(filename.toAscii(), "w");
    
    if(dxffile)
     //outline_dxf_obj.WriteDXF(dxffile, outline_dxf_tops, true);  
     final_objpts.WriteDXF(dxffile, final_tops, true) ;                        
                                
    else
    { cout<<"Cannot open file."<<endl; return false;}
    
    fclose(dxffile);
    return true;
     
}



bool City::SaveLaserData(LaserPoints &points)
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
                                     "Laser points (*.laser);;All files (*.*)");
  if (filename.isEmpty()) return false;
  
  // Save the points
  points.SetPointFile(filename.toAscii());
  if (!points.Write(points.PointFile(), false)) {
    QMessageBox::information(this, "Error",
                             "Writing laser points to file",
                             points.PointFile(), "failed!");
    return false;
  }
  
  SetCurrentDirectory(&filename);
  
  QString qmap_obj(current_dir);
  QString qmap_top(current_dir);
  
  if(map_obj.size())
      map_obj.Write((qmap_obj.append("/map.objpts")).toAscii());
      
  if(map_top.size())
      map_top.Write((qmap_top.append("/map.top")).toAscii());

  statusBar()->showMessage(QString("Laser points written to file ") +
                           points.PointFile(), 2000);
  return true;
}

void City::closeEvent(QCloseEvent *event)
{
}




void City::about()
{
  QMessageBox::about(this, tr("City"),
		     tr("Interactive programme for modelling building facades\n Author: George Vosselman & Shi Pu (spu@itc.nl)"));
}

void City::aboutQt()
{
  QMessageBox::aboutQt( this, "Qt Application Example" );
}
void City::ImportMap()
{
  QString filename,dir;
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
  ShowData(CityWindowPtr(), MapData, true, SelectedMapData, true, true);
}

void City::ImportDXFMap()
{
LineTopologies tops;
ObjectPoints objpts;
QString filename;

const char *name;

filename = QFileDialog::getOpenFileName(this, "Select map file",
                                          QString::null,
                                      "Map points (*.dxf);;All files (*.*)");
if (filename.isEmpty()) return;

name=filename.toAscii().constData();

objpts.ReadDXF ((char *)name, tops, 3, false, 0, false, 0, false);

  buildings.ImportMap(map_point_file, map_top_file, map_points);
  ShowData(CityWindowPtr(), MapData, true, SelectedMapData, true, true);



}



void City::Set_Source_File()
{
  QString filename;
  filename = QFileDialog::getOpenFileName(this, "Set pts source file",
                                          QString::null,
                                      "pts points (*.pts);;All files (*.*)");
  if (filename.isEmpty()) return;
   source_file = (char *) malloc(strlen(filename.toAscii()) + 1);
  
  strcpy(source_file, filename.toAscii());

}

void City::Set_Tile_Meta_File()
{
  QString filename;
  filename = QFileDialog::getOpenFileName(this, "Set strip meta file",
                                          QString::null,
                                      "pts points (*.block);;All files (*.*)");
  if (filename.isEmpty()) return;
   tile_meta_file = (char *) malloc(strlen(filename.toAscii()) + 1);
  
  strcpy(tile_meta_file, filename.toAscii());

}


void City::ImportModel()
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
  ShowData(CityWindowPtr(), ModelData, true, SelectedModelData, true, true);
}

void City::ImportLaserBlock()
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

void City::ImportLaserPoints()
{
  QString filename;
 
  
  filename = QFileDialog::getOpenFileName(this, "Select laser point file", 
                                          QString::null,
                                  QString("Laser point data files (*.laser*)") +
                                          ";;All files (*.*)");
  if (filename.isEmpty()) return;
  SetLaserPointFile(filename.toAscii());
  
  if (laser_points.size()) laser_points.ErasePoints();
  laser_points.Read(filename.toAscii());
  laser_points.DeriveDataBounds(0);
  
  if (laser_points.size())
    {
    ShowData(CityWindowPtr(), LaserData, true, SelectedLaserData, false, true);
    
    }
}






void City::SplitOutline(MouseMode mode,
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
        ShowData(CityWindowPtr(), MapPartitionData, true,
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
        ShowData(CityWindowPtr(), MapPartitionData, true,
                 SelectedMapPartitionData, true, true, true);
        // Switch to map partition selection mode
        SetMode(SelectMapPartitionMode);
      }
      break;
    default:
      break;
  }
}

bool CityWindow::InitialiseExtensionLine()
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

bool CityWindow::InitialiseMoveNodeLines()
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

void City::RemoveCityWindow(CityWindow *closed_window)
{
  vector <CityWindow *>::iterator window;

  for (window=subwindows.begin(); window!=subwindows.end(); window++)
    if (*window == closed_window) {
      subwindows.erase(window);
      return;
    }
  printf("Did not find window in CityWindow list, bug?\n");
}

void CityWindow::AddShowDataConnectors(CityWindow *window)
{
     
     
  // Redraw canvas if the appearance of any data type changed
  for (int j=0; j<NumDataTypes; j++)
    connect(appearance[j], SIGNAL(ChangedSetting()),
            window->Canvas(), SLOT(updateGL()));
  if (window->Type() == CityResult)
    connect(result_appearance, SIGNAL(ChangedSetting()),
            window->Canvas(), SLOT(updateGL()));
  
  // Add data to a canvas on request of a CityWindow
  connect(window, SIGNAL(RequestDataDisplay(DataType, CityWindow *)),
          this, SLOT(DisplayData(DataType, CityWindow *)));

  // Add background image to a canvas on request of a CityWindow
  connect(window, SIGNAL(RequestImageDisplay(BackGroundType, CityWindow *)),
          this, SLOT(SetBackGroundImage(BackGroundType, CityWindow *)));
          
}

void CityWindow::SetScaleBar(CityScaleLocation new_location, 
                                   double new_distance, bool update)
{
  int i, item;
  
  for (i=0; i<3; i++)
    show_scale_actions[i]->setChecked(new_location == (CityScaleLocation) i);
  item = (int) (log(new_distance) / log(10.0) + 2.01);
  for (i=0; i<5; i++)
    set_scale_tick_actions[i]->setChecked(i == item);
  canvas->SetScaleLocation(new_location);
  canvas->SetScaleTickDistance(new_distance);
  if (update) canvas->update();
}

void City::ChangeLaser(FeatureWindowType ftype)
{
   
 switch (ftype){
        case Wall: wall_laser=*(wall_win->PointCloud()); UpdateWall(); break;
        case Ground: ground_laser=*(ground_win->PointCloud()); ground_level=ground_laser[0].GetZ();break;
        case Door: door_laser=*(door_win->PointCloud());CreateTopology(Door); 
                     door_win->Canvas()->ClearObjectData(true);
                     door_win->map_points=door_obj;
                     door_win->Canvas()->AddObjectData(&door_obj,&door_tops,appearance[MapData]->DataAppearancePtr(),false, true);
                     door_win->Canvas()->update();
                     break;
        case Roof: roof_laser=*(roof_win->PointCloud()); break;
        case RoofExtrusion: roof_extru_laser=*(roof_extru_win->PointCloud()); break;
        case Window: window_laser=*(window_win->PointCloud()); CreateTopology(Window);
                     window_win->Canvas()->ClearObjectData(true);
                     window_win->map_points=window_obj;
                     window_win->Canvas()->AddObjectData(&window_obj,&window_tops,appearance[MapData]->DataAppearancePtr(),false, true);
                     window_win->Canvas()->update();
                     break;
        case Extrusion: extrusion_laser=*(extrusion_win->PointCloud()); break;
   }
     
}

void City::Show_Info(int no,FeatureWindowType ftype)
{
     if(ftype==None&&convexhulled==false)
     return;
     
     Position3D center=building_out[no].Bounds(objpts_out).MidPoint();;
     double angle;
     QString segment_no,area, qx,qy,qz,qdistance,qangle;
     segment_no.setNum(no);
     area.setNum(abs(building_patches[no].area3D()),'f',2);
     qx.setNum(center.GetX(),'f',2);
     qy.setNum(center.GetY(),'f',2);
     qz.setNum(center.GetZ(),'f',2);
     qdistance.setNum(GBKN.DistanceToPointSigned((Position2D)center));
     angle=abs(Angle2D(GBKN.Direction(),(Vector2D)(building_patches[no].getPlane().Normal())));
     
     
     angle=abs(PI/2-angle);
     angle=angle*180/PI;
     
     
     
     qangle.setNum(angle,'f',2);
     QString str="Segment number is ";
     str+=segment_no;
     str+="; area is ";
     str+=area;
     str+="; center is (";
     str+=qx;
     str+=",";
     str+=qy;
     str+=",";
     str+=qz;
     str+=");\n";
     str+="2D distance to map line:";
     str+=qdistance;
     str+=";angle with map line:";
     str+=qangle;
     str+=" degree";
     
     

     switch(ftype){
     case Wall:             
               wall_win-> statusBar()->showMessage(str, 0);
          break;
     case Ground:
               ground_win-> statusBar()->showMessage(str, 0);
          break;   
     case Door:
               door_win-> statusBar()->showMessage(str, 0);
          break;
     case Roof:
               roof_win-> statusBar()->showMessage(str, 0);
          break;   
     case RoofExtrusion:
               roof_extru_win-> statusBar()->showMessage(str, 0);
          break;        
     case Window:
               window_win-> statusBar()->showMessage(str, 0);
          break;   
     case Extrusion:
               extrusion_win-> statusBar()->showMessage(str, 0);
          break;                              
     default:                         
               if(building_patches.size())
                 statusBar()->showMessage(str,0);
          break;                   
     }                          
}



void City::CreateCustomMenu(const QPoint &pos)
{
  QAction *action;
  QMenu *menu=new QMenu(this);
  
  
  QMenu *sub_menu = menu->addMenu(tr("Add to "));
  
  action = new QAction(tr("Wall Zone"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(AddtoWall()));
  sub_menu->addAction(action);
  
  action = new QAction(tr("Door Zone"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(AddtoDoor()));
  sub_menu->addAction(action);
  
  action = new QAction(tr("Roof Zone"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(AddtoRoof()));
  sub_menu->addAction(action);
  
  action = new QAction(tr("Roof Extrusion Zone"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(AddtoRoofExtru()));
  sub_menu->addAction(action);
  
  action = new QAction(tr("Extrusion Zone"), this);
  connect(action, SIGNAL(triggered()), this, SLOT(AddtoExtrusion()));
  sub_menu->addAction(action);
  
  menu->exec(pos);
}

void City::AddtoRoof()
{
     //roof_win->Canvas()->AddLaserData(&selected_laser_points,appearance[LaserData]->DataAppearancePtr(),true, true);
     
     int i;   
     LaserPoints *lpts;
     lpts=roof_win->PointCloud();
     
     if(roof_win->Canvas()->HasData(LaserData))
     {
       (*lpts)=(*lpts)+selected_laser_points;
     } 
     else
     {
        roof_laser=selected_laser_points;
        roof_win->AddLaserData(roof_laser.LaserPointsReference(),appearance[LaserData]->DataAppearancePtr(), true);
        roof_win->Canvas()->InitialiseTransformation();      
     }    
     roof_win->Canvas()->update(); 
}

void City::AddtoDoor()
{
     
     LaserPoints *lpts;
     lpts=door_win->PointCloud();
     
     if(door_win->Canvas()->HasData(LaserData))
     {
        (*lpts)=(*lpts)+selected_laser_points;
       
     }
     else
     {
        door_laser=selected_laser_points;
        door_win->AddLaserData(door_laser.LaserPointsReference(),appearance[LaserData]->DataAppearancePtr(), true);
        door_win->Canvas()->InitialiseTransformation();    
     }
     
     door_win->Canvas()->update(); 
}


void City::AddtoExtrusion()
{
    
     LaserPoints *lpts;
     lpts=extrusion_win->PointCloud();
     
     if(extrusion_win->Canvas()->HasData(LaserData))
     {
       (*lpts)=(*lpts)+selected_laser_points;
      
     }
     else
     {
       extrusion_laser=selected_laser_points;
       extrusion_win->AddLaserData(extrusion_laser.LaserPointsReference(),appearance[LaserData]->DataAppearancePtr(), true);
       extrusion_win->Canvas()->InitialiseTransformation();   
     }
     extrusion_win->Canvas()->update(); 
}

void City::AddtoRoofExtru()
{
     
     LaserPoints *lpts;
     lpts=roof_extru_win->PointCloud();
     
     if(roof_extru_win->Canvas()->HasData(LaserData))
     {
     (*lpts)=(*lpts)+selected_laser_points;
     }
     else
     {
       roof_extru_laser=selected_laser_points;
       roof_extru_win->AddLaserData(roof_extru_laser.LaserPointsReference(),appearance[LaserData]->DataAppearancePtr(), true);
       roof_extru_win->Canvas()->InitialiseTransformation();    
     }
    
     roof_extru_win->Canvas()->update(); 
}

void City::AddtoWall()
{
     
     LaserPoints *lpts;
     lpts=wall_win->PointCloud();
     
     if(wall_win->Canvas()->HasData(LaserData))
     {
       (*lpts)=(*lpts)+selected_laser_points;
     }
     else
     {
      wall_laser= selected_laser_points;  
      wall_win->AddLaserData(wall_laser.LaserPointsReference(),appearance[LaserData]->DataAppearancePtr(), true);
      wall_win->Canvas()->InitialiseTransformation();    
     } 
     wall_win->Canvas()->update(); 
}



void City::DisplayPointInformation(const LaserPoint &point)
{
  char                *string, *value;
  const unsigned char *tag;
  const int           *attribute;
  const float         *floatattribute;
  int                 i;
  
  string = (char *) malloc(1024);
  value  = (char *) malloc(32);
  
  sprintf(string, "XYZ (%.2f, %.2f, %.2f)", point.X(), point.Y(), point.Z());
  
  for (i=0, tag=point.AttributeTags(), attribute=point.AttributeValues();
       i<point.NumAttributes(); i++, tag++, attribute++) {
    switch (*tag) {            
      case ReflectanceTag: strcat(string, "; Reflectance "); break;
      case PulseCountTag : strcat(string, "; Pulse count ");
                           sprintf(value, "%d", point.PulseCount()); break;
      case LabelTag      : strcat(string, "; Label "); break;
      case IsFilteredTag : strcat(string, "; Is filtered "); break;
      case IsProcessedTag: strcat(string, "; Is processed "); break;
      case IsSelectedTag : strcat(string, "; Is selected "); break;
      case ColourTag     : strcat(string, "; Colour ");
                           sprintf(value, "(%d, %d, %d)",
                           point.Red(), point.Green(), point.Blue()); break; 
      case ResidualTag   : floatattribute = (const float *) attribute;
                           strcat(string, "; Residual ");
                           sprintf(value, "%7.3f", *floatattribute); break;
      case SegmentNumberTag: strcat(string, "; Segment number "); break;
      case PlaneNumberTag: strcat(string, "; Plane number "); break;
      case ScanNumberTag : strcat(string, "; Scan number "); break;
      case PointNumberTag: strcat(string, "; Point number "); break;
      case PulseLengthTag: strcat(string, "; Pulse length "); break;
      case PolygonNumberTag: strcat(string, "; Polygon number "); break;
      default            : strcat(string, "; Undefined ");
                           sprintf(value, "(tag %d) ", *tag);
                           strcat(string, value); break;
    }
    if (*tag != ColourTag && *tag != PulseCountTag && *tag != ResidualTag)
      sprintf(value, "%d", *attribute);
    strcat(string, value); 
  }
  statusBar()->showMessage(QString(string));
  free(string);
  free(value);
}


void City::AddTiePoint(const LaserPoint &point)
{
  char                *string;
  
  string = (char *) malloc(1024);

  if(cyclorama)
  {
              
               cyclorama->addRow_laser(point.X(), point.Y(), point.Z());
               sprintf(string, "Point (%.2f, %.2f, %.2f) selected as a tie point.", point.X(), point.Y(), point.Z());

               statusBar()->showMessage(QString(string));
               free(string);
   }
}

void City::DelTiePoint()
{
     char                *string;
  
  string = (char *) malloc(1024);

  if(cyclorama)
  {
               
               cyclorama->deleteRow_laser();
               
               sprintf(string, "a tie point deleted.");

               statusBar()->showMessage(QString(string));
               free(string);
  }
     
}



