
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


/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : August 2003
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Class to handle display of LaserPoints on a glCanvas.
*				Derives from QGLGeneralCanvas
*
*--------------------------------------------------------------------*/
#ifndef _QGL_POINTS_CANVAS_H_
#define _QGL_POINTS_CANVAS_H_

#include "QGLGeneralCanvas.h"
#include "LaserPointsUtility.h"
#include "LaserPointsProcessing.h"
#include "LaserObjects.h"
#include "LaserSegments.h"
#include "RotationParameter.h"


#define DISPLAY_CLOUD 1
#define DISPLAY_TIN		2
#define DISPLAY_DATA_POINTS 4
#define DISPLAY_SEGMENTED_POINTS 8
#define DISPLAY_SELECTED_POINTS 16
#define DISPLAY_MODELED_OBJECTS 32

typedef void (*UserDefinedCallback)(void*);

enum ColorMappingMode{UseReflectance=1,UseRange=0,UseColor=2};

class QLaserViewer;
///Class to render and manipulate laserPoints. Although we have a pointset in GLContainerCanvas
///This class provides functionality more specific to laserPoints.
class QGLPointsCanvas: public QGLGeneralCanvas
{
	Q_OBJECT 

public:
	
	///Creates a new canvas 
	QGLPointsCanvas(QWidget* parent=NULL,const char* name="QGLPointsCanvas");
	
	///Create canvas and assign laser points.
	QGLPointsCanvas(const LaserPoints& pts, QWidget* parent=NULL,const char* name="QGLPointsCanvas");

	///This is the virtual destructor.
	~QGLPointsCanvas();

signals:
	
	//Notify that some selection has changed.
	void SegmentSelectionChanged(int size);
	void PointSelectionChanged(int size);
	void ObjectSelectionChanged(int size);
	void ControlPointSelectionChanged(int size);
	void PointTransformChanged();
	void UserTransformChanged();
	
	//Change in point size for drawing.
	void PointSizeChanged(int size);
	
	//Change in copied points or objects.
	void CopiedPointsChanged(int size);
	void CopiedObjectsChanged(int size);
	
	//change in kNN
	void KnnChanged(int knn);
	
	//Change in colorscheme
	void ColormapChanged(GLColorScheme cs);
	
	//Change in using reflectance.
	void ColorMappingModeChanged(bool useRef);
	
public slots:	
	///This slot handles menu items which are still experimental.
	void ExperimentalFunctions(int id);
	
	///This slot is for do-nothing actions.
	void DoNothing(){}
	
	///Add a control point.
	void AddControlPoint();
	
	///Load points from a file.
	bool LoadPoints();
	bool LoadPoints(const char* fileName);
	
	///Save points to laser file.
	bool SavePoints();
	
	///Loads points from ascii file.
	bool LoadAscii();
	
	///Saves points to ascii file.
	bool SaveAscii();
	
	///Load objects from a file.
	bool LoadObjects();
	
	bool LoadObjects(const char* fileName);
	
	///Save objects to laser file.
	bool SaveObjects();
	
	///Load segmentation from a file.
	bool LoadSegmentation(QString fileName);
	
	///Load segmentation from a file. Ask user for the file name too.
	bool LoadSegmentation();
	
	///Save segmentation to a file.
	bool SaveSegmentation(QString fileName);
	
	///Save segmentation to a file. Ask user for the file name too.
	bool SaveSegmentation();
	
	///Load normals from a file.
	bool LoadNormals(QString fileName);
	
	///Load normls from a file. Ask user for the file name too.
	bool LoadNormals();
	
	///Save normals to a file.
	bool SaveNormals(QString fileName);
	
	///Save normals to a file. Ask user for the file name too.
	bool SaveNormals();
		
	///Load selected indices from a file.
	bool LoadSelectedIndices(QString fileName);
	
	///Load selected indices from a file. Ask user for the file name too.
	bool LoadSelectedIndices();
	
	///Save selected indices to a file.
	bool SaveSelectedIndices(QString fileName);
	
	///Save selected indices to a file. Ask user for the file name too.
	bool SaveSelectedIndices();
		
	///Save KNN to a file.
	bool SaveKnnIndices(QString fileName);
	
	///Save KNN to a file. Ask for the filename.
	bool SaveKnnIndices();
	
	///Export vrml
	bool ExportVRML(QString fileName,bool bPointCloud, bool bTIN, bool bSegmentation,
		bool bNormals, bool bObjects, bool bAxisIndicator);
		
	///Export vrml. Ask user for file name and what to save.
	bool ExportVRML();

	///Rotate using current Opengl rotation.
	void RotatePoints();
	
	///Rotate using current Opengl rotation.
	void Print();
	
	///Sets the subsampling factor.
	void SetSubsamplingFactor(int _subsampling);
	
	///Sets the subsampling factor. - from Sender->data
	void SetSubsamplingFactor();
	
	///Gets the subsampling factor.
	int GetSubsamplingFactor()const;
	
	///Shows the segments browser.
	void ShowSegmentBrowser();
	
	///Shows the laser list browser.
	void ShowLaserListBrowser();
	
	//Stretch points.
	void StretchPoints();
	
	///Initialize the state variables.
	virtual void InitializeState();
	
	///Set fill mode.
	void SetFillMode(int);
	
	///Set fill mode - from Sender()->Data
	void SetFillMode();
	
	///Update display list.
	void UpdateDisplayList();
	
	///Refresh the view. Just call display list and do some selection related drawing.
	void Repaint();
	
	///Sets laser points.
	void SetLaserPoints(const LaserPoints& pts);
	
	///Sets count of k nearest neighbours.
	void SetKnn(int kNN);
	
	///Knn passed as sender()->Data.
	void SetKnn();
	
	///Selects points in a given window.
	///If single selection is on, just select the one closest to the user, from current view
	int WindowSelectPoints(int x1,int y1,int width,int height,IndicesVector& selectedIndices,bool bSinglePointSelection=false);
	
	///Selects objects in a given window.
	int WindowSelectObjects(int x1,int y1,int width,int height,IndicesVector& selectedIndices);
	
	///Performs free hand selection.
	int FreehandSelectPoints (IndicesVector& selectedIndices);

	///Selects segments if segmentation has been done.
	int WindowSelectSegments(int x1,int y1, int width, int height,IndicesVector& selectedSegments);
	
	///Applies transformation for picking.
	void ApplyGLTransformationForPicking(int x,int y,int width,int height);
	
	///Gets rotation from OpenGL and applies it to laserPoints.
	void ApplyGLRotationToLaserPoints(LaserPoints& laserPoints);
	
	///Turns on and off use of reflectance.
	void SetColorMappingMode(int bUse);
	
	///Sets colormapping mode - from sender()->data
	void SetColorMappingMode();
	
	///Returns current status of use reflectance flag.
	int GetColorMappingMode();
	
	///Returns laser points.
	LaserPoints& GetLaserPoints();
	
	///Returns selected laser points, or points belonging to selected segment.
	int GetSelectedLaserPoints(LaserPoints& selectedPoints);
	
	///Returns selected laser points, or points belonging to selected segment.
	LaserPoints GetSelectedLaserPoints();
	
	///Returns number of selected points.
	int PointSelectionCount();
	
	///Returns if we have point selection
	bool HasPointSelection();
	
	///Selects all points or segments.
	void SelectAll();
	
	///Deselects all points or segments.
	void DeselectAll();
	
	///Inverts the selection.
	void InvertSelection();
	
	///Crops the selection.
	void Crop();
	
	///Returns fileName of laser points.
	char* FileName();
	
	///Converts the object to a string. Useful for debugging.
	virtual string ToString(){return string("QGLPointsCanvas");}
	
	///Sets display type.
	void SetDisplayType(int d);
	
	///Sets display type - from sender()->Data.
	void SetDisplayType();
	
	///Shows a popup menu.
	void ShowPopupMenu();
	
	///Segments points using smoothness constraint.	
	void SegmentPointCloud( );
	
	///Subsample after asking user for settings.
	void Subsample();
	
	///Make normals consistent by region growing.
	void MakeNormalsConsistent();

	///Export TIN as VRML.
	void ExportTINVrml();
	
	///Switch to segment drawing mode.
	void PostSegmentationProcessing();
	
	///Find connected components.
	void SegmentIntoComponents();
	
	///Segments points into planar regions.	
	void SegmentIntoPlanes( );
	
	///Segments points into planar regions.	
	void DeleteSegmentation( );
	
	///Segments using university of bern segmentation algorithm.
	void SegmentDepthmapUB();
	
	///Calculates normals using kNN.
	void CalculateNormals();
	
	///Shows gaussian sphere in a new viewer.
	void ShowGaussianSphere();
	
	///Shows curvature in another view.
	void ShowCurvature();
	
	///Shows information.
	void ShowInformation();
	
	///Shows help.
	void ShowHelp();
	
	///Register using ICP.
	void RegisterICP();
	
	///Register using Indirect model based method.
	void RegisterIndirect();
	
	///Shows help about the viewer.
	void ShowHelpAbout();
	
	///Shows gaussian sphere in a new viewer.
	vector<double> CalculateCurvature();
	
	///Writes modeled objects to VRML.
	void WriteVRMLModeledObjects(char* fileName);
	
	///Export data to ascii file.
	void ExportAscii(const char* fileName);
	
	///Export data to vrml file.
	void ExportVrml(const char* fileName);
	
	///Cut the points.
	void CutPoints();
	
	
	///Show information about selected points or segments.
	void ShowPointInfo();
	
	///Copies currently modeled objects.
	void CopyObjects();
	
	///Pastes objects from global copied clipboard.
	void PasteObjects();
	
	///Pastes objects from global copied clipboard.
	void DeleteObjects();
	
	
	///Increases point size.
	int IncreasePointSize();
	
	///Decreases point size.
	int DecreasePointSize();
	
	///Set point size.
	int SetPointSize(int);
	
	///Set point size - from sender->data.
	int SetPointSize();
	
	///Set use of display lists.
	void SetUseDisplayList(int bUse);
	
	///Set use of display lists - from sender()->Data().
	void SetUseDisplayList();
	
	//Open selection in new window.
	QLaserViewer* InNewWindow();
	
	///Set Auto histoequalize mode.
	void SetAutoHistoequalize(int mode);
	
	///Set Auto histoequalize mode - from sender()->Data().
	void SetAutoHistoequalize();
	
	///Histoequalize laser points.
	void HistoEqualizeLaserPoints();
	
	///Copies points to global clipboard.
	void CopyPoints();
	
	///Pastes points from global clipboard.
	void PastePoints();
	
	///Draws GL Objects.
	void DrawGLObjects();
	
	///Draws GL selection
	void DrawGLSelection();
	
	///Fitting wrapper functions
	void FitCylinder();
	void FitPlane();
	void FitSphere();
	void FitTorus();
	
	///Growing functions.
	void GrowCylinders();
	void GrowPlanes();
	void GrowSpheres();
	void GrowTori();
	
	///Hough transform functions.
	void HoughCylinder();
	void HoughPlane();
	void HoughCylinderAndPlane();
		
	///Sets the specified palette.
	void SetPalette(int index);
	
	///Sets the specified palette - from sender()->data()
	void SetPalette();
	
	///Sets depth sensitive selection.
	void SetDepthSensitiveSelection(int bNew);
	
	///Sets depth sensitive selection - from sender()->data().
	void SetDepthSensitiveSelection();
	
	///Normal length factor.
	void SetNormalLengthFactor(int factor);
	
	///Normal length factor - from sender->data().
	void SetNormalLengthFactor();
	
	///Normal length factor
	void SetNormalLengthFactor(double factor);
	
	///Set the normal smoothing.
	void SetSmoothNormals(int bNew);
	
	///Set the normal smoothing - from sender()->data.
	void SetSmoothNormals();
		
	///Changes the palette.
	void ChangePalette();
	
	///Sets the color bar.
	void SetColorbar(int state);
	
	///Sets the color bar - from sender()->Data.
	void SetColorbar();
	
	QString GetHelpString();
	
	///Get the selected ControlPoints.
	LaserPoints GetSelectedControlPoints() const;
	
	///Get All ControlPoints.
	LaserPoints GetAllControlPoints() const;
	
	///Get all ControlPoints.
	LaserPoints top() const{};
	
	///Get Rotation.
	Rotation3D GetRotation() const;
	
	///Get Translation.
	Vector3D GetTranslation() const;
	
	///Set Rotation.
	void SetRotation(Rotation3D rot);
	
	///Set Translation.
	void SetTranslation(Vector3D trans);
	
	///Set Transform.
	void SetTransform(Rotation3D rot, Vector3D trans);
	
	///Apply point transformations.
	void ApplyTransform()const;
	
	void UpdateActions();
	
	void SetMenuVisible(bool status = true);
	
public:	
	void DrawAll();
	//Event handlers
	void paintGL();
	void mousePressEvent ( QMouseEvent * e );
	void mouseReleaseEvent ( QMouseEvent * e );
	void mouseDoubleClickEvent ( QMouseEvent * e );
	void mouseMoveEvent ( QMouseEvent * e );
 	void wheelEvent ( QWheelEvent * e );
 	void keyPressEvent ( QKeyEvent * e );
 	void keyReleaseEvent ( QKeyEvent * e );
 	void focusInEvent ( QFocusEvent * e);
 	void focusOutEvent ( QFocusEvent * e);
 	void enterEvent ( QEvent * e);
 	void leaveEvent ( QEvent * e);
 	void closeEvent ( QCloseEvent * e );
	void contextMenuEvent ( QContextMenuEvent * e );
	void dragEnterEvent ( QDragEnterEvent * e);
	void dragMoveEvent ( QDragMoveEvent * e);
	void dragLeaveEvent ( QDragLeaveEvent * e);
	void dropEvent ( QDropEvent * e);
	void showEvent ( QShowEvent * e);
	void hideEvent ( QHideEvent * e);
	
	///Get selected indices.
	vector<int> GetSelectedIndices()const;
	
	///Set selected indices.
	void SetSelectedIndices(const vector<int>& indices);
	
	///Assign a new callback to button press.
	void SetButtonPressCallback(UserDefinedCallback Callback,void* userData);

	///Gets information in form of a string.
	QString GetInformationString();
	
	///Calculate and return the bounds.
	void GetBoundingBox(Vector3D& minimun,Vector3D& maximum,Vector3D& range,Vector3D& middle);
	
	///Return Laser objects.
	vector<LaserObject*>& GetLaserObjects(){return laserObjects;};
	
	///Set LaserObjects.
	void SetLaserObjects(const vector<LaserObject*>& newObjects);
		
	///Return pointer to selected object.
	LaserObject* GetSelectedObject()const;
		
	///Delete Selected Object.
	void DeleteSelectedObject();
		
	///Selection related variables.
	IndicesVector selectedIndices;
	IndicesVector selectedObjectIndices;
	IndicesVector selectedSegmentIndices;
	
	///Segmented Indices vector. Its a vector of IndicesVector.
	LaserSegments segmentedVector;
	
	LaserPoints laserPoints;
	LaserPoints normalPoints;
	
protected:
	///renders segmentation results.
	void DrawSegmentedPoints();
	
	///renders Laser points.
	void DrawLaserPoints();
	
	///render selection.
	void DrawSelectedPoints();
	
	///render selected objects
	void DrawSelectedObjects();
	
	///render selected segments.
	void DrawSelectedSegments();

	///renders modeled objects.
	void DrawModeledObjects();
	
	///Allocate actions and menus.
	void MakeActionsAndMenubar();
	
	
	//Members
	GLuint pointsCanvasDisplayMode;
	char fileName[1024];
	GLuint pointSize;
	ColorMappingMode colorMappingMode;
	bool bDepthSensitiveSelection;
	double normalLengthFactor;
	int kNN;
	bool smoothNormals;
	bool autoHistoEqualize;
	bool bUseDisplayList;
	//User defined Callbacks.
	UserDefinedCallback buttonPressCallback;
	void* buttonPressUserData;
	
	int subSamplingFactor;
	
	//vector of Models pointers, result from modeling or copy paste.
	LaserObjectsVector laserObjects;
	
	//A counter for keeping track of Controlpoint id's.
	int controlPointCounter;
	
	//vector of indices accumulated during free hand selection.
	vector<float> xFreehandPoints;
	vector<float> yFreehandPoints;
	
	//Rotation and translation of the laserpoints. Its the transform that is applied.
	Rotation3D rotation;
	Vector3D translation;
	
	///Actions and menu items.
	QMenuBar* menuBar;
	QMenu* fileMenu;
	QActionGroup* fileOpenGroup;
	QAction* fileLoadLaserAction;
	QAction* fileLoadLaserAsciiAction;
	QAction* fileLoadObjectsAction;
	QAction* fileLoadSegmentationAction;
	QAction* fileLoadSelectedIndicesAction;
	QAction* fileLoadNormalsAction;
	QActionGroup* fileSaveGroup;
	QAction* fileSaveLaserAction;
	QAction* fileSaveLaserAsciiAction;
	QAction* fileSaveObjectsAction;
	QAction* fileSaveSegmentationAction;
	QAction* fileSaveSelectedIndicesAction;
	QAction* fileSaveNormalsAction;
	QAction* fileSaveKnnAction;
	QAction* fileExportVRMLAction;
	QAction* exitAction;
	QMenu* viewMenu;
	QMenu* paletteMenu;
	QActionGroup* paletteGroup;
	QAction* paletteGrayAction;
	QAction* paletteStandardAction;
	QAction* paletteJetAction;
	QAction* paletteHotAction;
	QAction* paletteCoolAction;
	QAction* paletteArmyAction;
	QAction* paletteElevationAction;
	QAction* palettePastelAction;
	QAction* paletteGrayGreenAction;
	QAction* paletteGrayBlueAction;
	QAction* paletteThresholdedAction;
	QAction* paletteRedAction;
	QAction* paletteGreenAction;
	QAction* paletteBlueAction;
	QMenu* displayMenu;
	QActionGroup* displayGroup;
	QAction* displayPointCloudAction;
	QAction* displayTINAction;
	QAction* displaySelectedPointsAction;
	QAction* displaySegmentationAction;
	QAction* displayModelsAction;
	QAction* displayColorbarAction;
	QAction* displayBkColorAction;
	QActionGroup* pointSizeGroup;
	QMenu* pointSizeMenu;
	QAction* pointSizeIncreaseAction;
	QAction* pointSizeDecreaseAction;
	QActionGroup* subSampleGroup; 
	QMenu* subSampleMenu; 
	QMenu* colorCodingMenu;
	QActionGroup* colorCodingGroup;
	QAction* useReflectanceAction;
	QAction* useRangeAction;
	QAction* useColorAction;
	QAction* autoHistoEqualizeAction;
	QActionGroup* viewPointGroup;
	QAction* viewFromXAction;
	QAction* viewFromYAction;
	QAction* viewFromZAction;
	QAction* resetViewAction;
	QMenu* viewPointMenu;
	QMenu* selectionMenu;
	QActionGroup* selectionGroup;
	QAction* selectAllAction;
	QAction* deselectAllAction;
	QAction* invertSelectionAction;
	QAction* depthSensitiveSelectionAction;
	QAction* inNewViewerAction;
	QMenu* editMenu;
	QActionGroup* copyGroup;
	QAction* copyPointsAction;
	QAction* copyObjectsAction;
	QActionGroup* pasteGroup;
	QAction* pastePointsAction;
	QAction* pasteObjectsAction;
	QActionGroup* cropGroup;
	QAction* cropPointsAction;
	QActionGroup* deleteGroup;
	QAction* deleteObjectsAction;
	QAction* histoEqualizeAction;
	QMenu* fitMenu;
	QActionGroup* fittingGroup;
	QAction* fitPlaneAction;
	QAction* fitSphereAction;
	QAction* fitCylinderAction;
	QAction* fitTorusAction;
	QActionGroup* processingGroup;
	QAction* calculateNormalsAction;
	QAction* calculateCurvatureAction;
	QAction* showGaussianSphere;
	QMenu* segmentationMenu;
	QActionGroup* segmentationGroup;
	QAction* segmentSmoothAction;
	QAction* segmentPlanarAction;
	QAction* segmentConnectedAction;
	QAction* showSegmentBrowserAction;
	QAction* showLaserListBrowserAction;
	QAction* deleteSegmentationAction;
	QMenu* settingsMenu;
	QMenu* kNNMenu;
	QActionGroup* kNNGroup;
	QAction * currentKnnAction;
	QActionGroup* normalLengthGroup;
	QMenu* normalLengthMenu;
	QAction* currentNormalLengthAction;
	QMenu* polygonFillMenu;
	QActionGroup* polygonFillGroup;
	QAction* polygonFilledAction;
	QAction* polygonLineAction;
	QAction* polygonPointAction;
	QAction* useDisplayListAction;
	QMenu* helpMenu;
	QActionGroup* helpGroup;
	QMenu* registrationMenu;
	QAction* registerICP;
	QAction* registerIndirect;

};	
			

#endif //_LASER_POINTS_CANVAS_H_


