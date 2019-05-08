
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


#ifndef CITY_H
#define CITY_H
#include <Vector>
#include <algorithm>
#include "CityWindow.h"
#include "CityCanvas.h"
#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "DataAppearWin.h"
#include "LaserBlock.h"
#include "Buildings.h"
#include "LineSegment2D.h"
#include "FittingParWin.h"
#include "SegmentParWin.h"
#include "LaserPatch.h"
#include "LaserPatches.h"
#include "Position3D.h"
#include "TerrestrialParWin.h"
#include "LaserAttributesWin.h"
#include <QProgressDialog>
#include <QString>
#include "cv.h"
#include "highgui.h"
#include <QMenu>
#include <QAction>
#include "imageviewer.h"

#define PI 3.1415


class City: public CityWindow
{
    Q_OBJECT

  protected:

// User interface stuff

    
   
    DataAppearance *appear_tmp;

    /// GUI for editing of fitting parameters
    FittingParametersWindow *fitting_parameters;

    /// GUI for editing of segmentation parameters
    SegmentationParametersWindow *segmentation_parameters;
    
    
    //GUI for editing of terrestrial reconstruction parameters
    TerrestrialParametersWindow *terrestrial_parameters;
    
    /// GUI for editing default laser point attributes
    LaserAttributesWindow *default_laser_attributes;


    bool has_outline;

    
// General project data

    /// Project name
    char *project_name;
    /// Meta data file with file names of point cloud mapping project
    char *project_file;

// File names

   
   
    /// File with model points
    char *model_point_file;

    /// File with model topology
    char *model_top_file;

    /// File with laser meta data
    char *laser_meta_file;

    /// File with laser points
    char *laser_point_file;
    
    /// File with .pts source file
    char *source_file;
    
    //File with strip metat data
    char *tile_meta_file;
    
    QString current_dir;
    
    bool convexhulled;

    
// The building data

  
// Selections


   ObjectPoints map_obj;
   LineTopologies map_top;
   Line2D GBKN;
   Line3D GBKN3D;
   bool GBKN_OK;
   Position2D left_map, right_map; 
    ///
    LaserPatches building_patches;
    
    LaserPoints map_laser;
    
    LaserPoints wall_laser, roof_laser, roof_extru_laser,door_laser, window_laser, ground_laser,extrusion_laser,walloutline_laser,window_tmp;
         TINEdges mytinedges;
    

    
    int window_max_no, ground_index;
    
     bool texture_exist, exter_exist;
    
    LineTopologies building_out, final_tops, wall_tops, roof_tops, door_tops, ground_tops,street_tops, extrusion_tops,window_tops, hole_out_tops, dormer_window_tops,reference_tops;
    
    ObjectPoints objpts_out,final_objpts, window_obj, hole_out_obj,roof_objpts,door_obj,extrusion_obj,dormer_window_obj,reference_obj;
    
     double ground_level;
     
      CityWindow *ground_win, *door_win, *wall_win, *window_win, *roof_win, *roof_extru_win,*extrusion_win,*outline_full_win;
 //for Derive contour     
 double dc_tolerance;
 int dc_step_time; //may set to 2, 3 or so
 LaserPoints contour;
 
 Plane wall;
 
 ObjectPoints outline_obj,outline_full_obj,gap_obj, roof_triangle_obj , roof_extru_obj, selection_obj, outline_estimated_obj,outline_drilled_obj; 
 LineTopologies outline_tops,outline_full_tops,gap_tops, roof_triangle_tops, roof_extru_tops, selection_tops,outline_estimated_tops,outline_drilled_tops;
 
 
     
    ObjectPoints cwindow_objpts; 
    LineTopologies cwindow_tops;
 //for wall roof integration
 
 LineTopologies rlist; //this is not really linetopologies, it goes along with roof_top, Linetopology i means the corresponding outline for roof i
 

// Texture data

    /// Texture images
    Image *texture_images[4];

    /// Texture bounds
    DataBounds3D texture_bounds[4];


//new ---------------------------------------------------------------------------------------

    ObjectPoints ch_obj;
    LineTopologies ch_tops;
    
    vector<Position3D>::iterator iter;
    LaserPatches building;
    LaserPatch chull;
    
    BuildingPart convexhull_bp;

    QProgressDialog *progress;

//Image-------------------------------------------------------------------------------------

      
      
      ImageViewer* cyclorama;
      
      IplImage* pContourImg;
      

// Functions

  public:
    /// Default constructor
    City();

    /// Default destructor
    ~City() {};
    
        //select laserpoints
    void SelectProject(char *filename);
    
   


  protected:
    
    /// Read meta and mapping data
    bool ReadProject(const char *filename, bool meta_data_only=false);


    /// Save laser data
    bool SaveLaserData(LaserPoints &points);
    
    /// Save all project data
    bool SaveProject(bool ask_file_names, bool save_map, bool save_model);

    /// Set the model point file name
    void SetModelPointFile(const char *point_file);

    /// Set the model topology file name
    void SetModelTopologyFile(const char *top_file);

    /// Set the laser meta data file name
    void SetLaserMetaFile(const char *meta_file);

    /// Set the laser point file name
    void SetLaserPointFile(const char *point_file);
    
    
    /// Set current Directory
    void SetCurrentDirectory(QString *dir);

    /// Close the programme after checking the need for saving data
    void closeEvent(QCloseEvent *);

  
    void setLabel(int index,int label);
   


    /// Generate height texture image
    bool GenerateHeightTexture(CityWindow *);

    /// Generate shaded height texture image
    bool GenerateShadedHeightTexture(CityWindow *);

    /// Fit a roof to the laser points
    bool FitRoof(RoofType type);


    void fillgap();
    
    void CreateTopology(FeatureWindowType);
    
    void SmoothRoof();
    
    bool AlignX(Vector2D vec1,double thres);
    
    /// Set an attribute of selected laser points to the default value
    void SetLaserAttribute(const LaserPointTag tag);

   //find holes for wall and dormer
   int FindHole(LaserPoints, LaserPoints&, LaserPoints&);
   
   void UpdateWall() ;
   
   void RemovePoint(ObjectPoints *objpts, LineTopologies *tops, int vertex_number);
    
    void AddPoint(ObjectPoints *objpts, LineTopologies *tops, int vertex_number);
    
    Position3D ProjectOnBack(Position3D point);
    Position3D ProjectOnBack(Position3D point, double dist);

    
  private slots:
          
    void CreateCustomMenu(const QPoint &);
    
    void CreateRoofExtrusion(int extru_index,int extru_type);
    
    void CreateExtrusion();
    
    void  AddtoRoof()  ;   
    
    void  AddtoDoor()  ;   
    
  
    
    void  AddtoWall()  ; 
      
    void  AddtoExtrusion()  ;   
    
    void AddtoRoofExtru();
    
    void MoveMapPoint(FeatureWindowType);
    
    void RemoveVertexPoint(FeatureWindowType, int);
    
    void AddVertexPoint(FeatureWindowType, int);
    
    
    /// Select a project meta data file (and open it by OpenProject)
    void SelectProject();


    /// Save map data
    void SaveMapData() {SaveProject(false, true, false);}

    /// Save map data in different files
    void SaveMapDataAs() {SaveProject(true, true, false);}

    /// Save laser data in different files
    void SaveLaserDataAs() {SaveLaserData(laser_points);}
    
    /// Save selected laser data in different files
    void SaveSelectedLaserDataAs() {SaveLaserData(selected_laser_points);}
    
    /// Import model data
    void ImportModel();

    /// Import map data (added to current project, if any)
    void ImportMap();
    
    /// Import DXF map data 
    void ImportDXFMap();

    /// Import laser block
    void ImportLaserBlock();

    /// Import laser points
    void ImportLaserPoints();
    
    

    /// Create Convex Hull
    void CCH();
    
    // Find features
    void FindFeature();
    
    void FitWallLine();
    
    int DeriveWallContour();
    
    void DeriveWallContour2();
    
    /// Information about this programme
    void about();

    /// Information about Qt
    void aboutQt();
    
    void Set_Source_File();
    
    void Set_Tile_Meta_File();

    void Select_reconstruct_map();
    
    void Select_reconstruct_region();
    
    void Create_map_from_laser();

    
   

    /// Add data to a window
    void DisplayData(DataType type, CityWindow *window, bool data_changed=false);

    /// Update the show menu for removed object data
    void UpdateShowDataMenu(DataType type);

   
    /// Toggle a selection from the nearby data buffer
    void ToggleSelectionData(DataType selection_type);



   /// Display information on selected laser point
    void DisplayPointInformation(const LaserPoint &point);
    
   
    
    /// Add a tie point
    void AddTiePoint(const LaserPoint &point);
    
    ///delete a tie point
    void DelTiePoint();

  
    /// Split a building (partition) outline
    void SplitOutline(MouseMode mode, const LineSegment2D &split_line);

    /// Merge two building (partition) lines
    void MergeMapLines();

    /// Read the view parameter file
    void ReadView();

    /// Save the view parameters
    void SaveView();

    /// Fit the view to selected data
    void FitViewToSelectedData();

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

    /// Fit a polygonal map line to the laser data
    void FitPolygonalMapLine();

    /// Fit a circular map line to the laser data
    void FitCircularMapLine();

    /// Segment laser data
    void SegmentLaserData();

    /// Grow surfaces
    void GrowSurfaces();
    
    
    
    // Remove small segments
    void RemoveSmallSegments();
    
    

    /// Delete loose points
    void DeleteLoosePoints();

    /// Set background colour
    void ChangeBackGroundColour();
    
    /// Set background image
    bool SetBackGroundImage(BackGroundType selected_type, CityWindow *window);

    /// Set no background image
    void SetNoBackGroundImage()
      {SetBackGroundImage(NoBackGroundImage, CityWindowPtr());}

    /// Set height background image
    void SetHeightBackGroundImage()
      {SetBackGroundImage(HeightImage, CityWindowPtr());}

    /// Set shaded height background image
    void SetShadedBackGroundImage()
      {SetBackGroundImage(ShadedHeightImage, CityWindowPtr());}

    /// Set (ortho)photo background image
    void SetPhotoBackGroundImage()
      {SetBackGroundImage(OrthoImage, CityWindowPtr());}

    /// Load background image
    void LoadBackGroundImage();

    /// Remove a CityWindow from the list of subwindows
    void RemoveCityWindow(CityWindow *window);

    /// Spawn the main window
    void SpawnWindow();

    /// Move data of sub window to the main window
    void TransferDataFromSubWindow(CityWindow *);
    
    /// Do not show the scale bar
    void SetNoScaleBar()
      { SetScaleBar(NoScale, canvas->ScaleBarTickDistance()); }

    /// Show the scale bar in the left bottom corner
    void SetLeftBottomScaleBar()
      { SetScaleBar(LeftBottomScale, canvas->ScaleBarTickDistance()); }

    /// Show the scale bar in the centre
    void SetCentreScaleBar()
      { SetScaleBar(CentreScale, canvas->ScaleBarTickDistance()); }
      
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
      
//new----------------------------------------------------------------------------------
 
    
    
    ///Whether one line align with another
    bool AlignWall(Vector2D vec1, Line2D line, double err);
    
    //show full outline
    void fillback();
    
    void ExtendRoof();
    
    void ChangeLaser(FeatureWindowType);
    
    
     void Show_Info(int,FeatureWindowType);
     
     //Find wall by checking holes in wall laser
     int FindWindow();
     
     void CreateFinalModel();
     
     
     bool SaveDXF();
     
     int AddRoofExtrusion(ObjectPoints objpts,LineTopologies tops,Plane plane);
     
     bool ConcaveHull(ObjectPoints &o, LineTopologies &, LaserPoints lpts);
     
     bool Rectangular(ObjectPoints &o, LineTopologies &, LaserPoints testpoints, int count_obj);
     
     void OffsetWindow();
     
//Photogrammetry----------------------------------------------------------------------------------     
      
      /// Import laser points
    // void ImportImage();
     
     void ImportImageQt();
    
     
     
     //int Hough_contour();
     
     void GenerateImage();
     
     
     void Image_Contour();
     
     bool Improvemodel(Position3D, Position3D, LineSegment2D & );
     
     void ImproveWallOutline();
     
     
};

#endif /* City_H */
