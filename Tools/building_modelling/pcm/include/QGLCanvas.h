
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


/*!
 * \file
 * \brief Class QGLCanvas - OpenGL canvas for usage with Qt
 *
 */
/*!
 * \class QGLCanvas
 * \ingroup Photogrammetry
 * \brief Class QGLCanvas - OpenGL canvas for usage with Qt
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date          ---- (Created)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 */

#ifndef QGLCANVAS_H
#define QGLCANVAS_H

#include <limits.h>
#include "LaserPoints.h"
#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "DataAppearance.h"
#include "LineTopPtrVector.h"
#include "DataBounds2D.h"
#include <QGLWidget>
#include "glfont.h"

class QGLCanvas : public QGLWidget
{
  Q_OBJECT

  protected:

// Data pointers
    // Object data: one entry for each combination of point set and appearance
    std::vector <ObjectPoints *>         object_point_sets;
    std::vector <LineTopPtrVector>       object_top_sets;
    std::vector <const DataAppearance *> object_appearances;
    // Laser data
    std::vector <LaserPoints *>          laser_point_sets;
    std::vector <LineTopPtrVector>       laser_top_sets;
    std::vector <const DataAppearance *> laser_appearances;
    // Texture data
    std::vector <const Image *>          texture_images;
    std::vector <const DataBounds3D *>   texture_bounds;
    std::vector <GLuint>                 texture_ids;

// Transformation
    GLfloat rotation[3];
    GLfloat translation[3];
    GLfloat zoom_factor;
    GLfloat eye_distance;
    double data_range;
    double pixel_size;
    Vector3D offset;
    Rotation3D rot;
    bool perspective;

// Button information
    bool   button_down[3];
    QPoint previous_pos;
    bool   ctrl_down, shift_down;
    
// Canvas information
    QColor background_colour;

// Text font for tile number display
    GLuint text_texture_id;
    GLFont text_font;
    
  public:

    QGLCanvas()
      {InitialiseAll(true); ctrl_down = shift_down = false;
       button_down[0] = button_down[1] = button_down[2] = false;
       background_colour = QColor(0, 0, 0);}

    QGLCanvas(QWidget *parent) : QGLWidget(parent)
      {InitialiseAll(true); ctrl_down = shift_down = false;
       button_down[0] = button_down[1] = button_down[2] = false;
       background_colour = QColor(0, 0, 0);}

    ~QGLCanvas() {};

  protected:
    virtual void initializeGL();
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
    virtual void keyPressEvent(QKeyEvent *event);
    virtual void keyReleaseEvent(QKeyEvent *event);
    virtual void wheelEvent(QWheelEvent *event);

  public:
    virtual void resizeGL(int width, int height);
    void rescaleGL();
    void mouseMoveEvent(QMouseEvent *event, bool refresh);
    void wheelEvent(QWheelEvent *event, bool refresh);

//------------------------------------------------------------------------------
// Adding, removing and changing appearance of data on the canvas
//------------------------------------------------------------------------------

    /// Add object data
    void AddObjectData(ObjectPoints *object_points,
                       LineTopologies *object_top,
                       const DataAppearance *object_appearance,
                       bool check_if_present=false,
                       bool refresh=true);

    /// Add a single face of object data
    void AddObjectData(ObjectPoints *object_points,
                       LineTopology *object_top,
                       const DataAppearance *object_appearance,
                       bool check_if_present=false,
                       bool refresh=true);
                  
    /// Remove data of a specific object
    bool RemoveObjectData(LineTopologies *object_top, bool refresh=true);

    /// Remove a single face of a specific object
    bool RemoveObjectData(LineTopology *object_top, bool refresh=true);

    /// Remove all object data with a specific appaearance
    bool RemoveObjectData(const DataAppearance *object_appearance,
                          bool refresh=true);

    /// Remove all object data of a specific data type
    bool RemoveObjectData(const DataType type, bool refresh=true);

    /// Remove all object data 
    void ClearObjectData(bool refresh=true);

    /// Change appearance of a specific object
    bool ChangeObjectAppearance(ObjectPoints *object_points,
                                LineTopologies *object_top,
                                const DataAppearance *new_appearance,
                                bool refresh=true);

    /// Change appearance of a single face of a specific object
    bool ChangeObjectAppearance(ObjectPoints *object_points,
                                LineTopology *object_top,
                                const DataAppearance *new_appearance,
                                bool refresh=true);

    /// Add laser data without topology
    bool AddLaserData(LaserPoints *laser_points,
                      const DataAppearance *laser_appearance,
                      bool check_if_present=false,
                      bool refresh=true);
                      
    /// Add laser data with topology
    bool AddLaserData(LaserPoints *laser_points,
                      LineTopologies *laser_top,
                      const DataAppearance *laser_appearance,
                      bool check_if_present=false,
                      bool refresh=true);
                      
    /// Add laser data with a single polygon
    bool AddLaserData(LaserPoints *laser_points,
                      LineTopology *laser_top,
                      const DataAppearance *laser_appearance,
                      bool check_if_present=false,
                      bool refresh=true);
                      
    /// Remove data of a specific laser data set
    bool RemoveLaserData(const LaserPoints *laser_points, bool refresh=true);

    /// Remove a single face of a laser topology set
    bool RemoveLaserData(LineTopology *laser_top, bool refresh=true);

    /// Remove all faces of a laser topology set
    bool RemoveLaserData(LineTopologies *laser_top, bool refresh=true);

    /// Remove all laser data with a specific appaearance
    bool RemoveLaserData(const DataAppearance *laser_appearance,
                         bool refresh=true);

     /// Remove all laser data
    void ClearLaserData(bool refresh=true);

    /// Add texture
    void AddTextureData(const Image *texture, const DataBounds3D *bounds,
                        bool refresh=true);

    /// Remove texture
    void RemoveTextureData(const Image *texture, const DataBounds3D *bounds, 
                           bool refresh=true);

    /// Remove all texture data
    void ClearTextureData(bool refresh=true);

//------------------------------------------------------------------------------
// Painting data
//------------------------------------------------------------------------------

    /// The main paint function, invoked by Qt
    void paintGL();

    /// Paint all data
    /** Virtual function in order to allow higher classes to override this
        function or add additional painting
    */
    virtual void PaintAllData();

    /// Set background colour
    void SetBackGroundColour(const QColor &colour);
    
    /// Return the background colour
    const QColor & BackGroundColour() const
      {return background_colour;}

 private:

    /// Paint object data
    void PaintObjectData(const ObjectPoints *points,
                         std::vector <LineTopPtrVector>::const_iterator top,
                         const DataAppearance *appearance);

    /// Paint laser data
    void PaintLaserData(LaserPoints *points,
                        const DataAppearance *appearance);

    /// Paint texture
    void PaintTextureData(GLuint texture_id, const DataBounds3D *bounds);
    
//------------------------------------------------------------------------------
// Transformations
//------------------------------------------------------------------------------

  public:
    /// Transformation from the canvas to the world coordinate system
    Position3D Canvas2World(const QPoint &canvas_pos, int canvas_height=0,
                            bool apply_offset=true) const;

    /// Transformation from the canvas to a plane in the world coordinate system
    Position3D Canvas2World(const QPoint &canvas_pos, double world_height,
                            bool apply_offset=true) const;

    /// Transformation from the world to the canvas coordinate system
    QPoint World2Canvas(const Position3D &world_pos,
                        bool apply_offset=true) const;

    /// Check if data is viewed from the top
    bool VerticalView() const
      {return (rotation[0] == 0.0 && rotation[1] == 0.0);}
     
    /// View data from the top
    void SetVerticalView() {rotation[0] = rotation[1] = 0.0; update();}
    
    /// Return bounding box
    DataBounds2D & BoundsXY() const;
    
    /// Set orthogonal projection
    void SetOrthogonalProjection() {perspective = false; update();}

    /// Set perspective projection
    void SetPerspectiveProjection() {perspective = true; update();}

//------------------------------------------------------------------------------
// Key press information
//------------------------------------------------------------------------------

    /// Return Ctrl key state
    bool CtrlKeyDown() const {return ctrl_down;}

    /// Return Shift key state
    bool ShiftKeyDown() const {return shift_down;}
    
    /// Return button state
    bool ButtonDown(int button) const {return button_down[button];}
    
    /// Set button state
    void SetButtonDown(int button, bool down) { button_down[button] = down; }

//------------------------------------------------------------------------------
// Initialisation, copying and querying of canvas data and transformation
//------------------------------------------------------------------------------

    /// Copy object and laser data of another canvas
    void CopyData(const QGLCanvas &canvas);

    /// Copy transformation parameters of another canvas
    void CopyTransformation(const QGLCanvas &canvas);

    /// Copy all data of another canvas
    void CopyAll(const QGLCanvas &canvas)
      {CopyData(canvas); CopyTransformation(canvas);}

    /// Initialise object and laser data
    void InitialiseData(bool refresh);

    /// Initialise transformation data
    void InitialiseTransformation(DataType focus_on_type=NumDataTypes);

    /// Initialise colour lookup table
    void InitialiseColourLUT();
    
    /// Initialise font
    void InitialiseFont();
    
    /// Initialise all canvas data
    void InitialiseAll(bool refresh)
      {InitialiseData(refresh); InitialiseTransformation();
       InitialiseColourLUT(); InitialiseFont();}

    /// Check if canvas has some object or laser data
    bool HasData() const;

    /// Check if canvas has data of a specific type
    bool HasData(DataType type) const;

    /// Read the transformation parameters from a file
    bool ReadView(char *filename);

    /// Write the transformation parameters to a file
    bool SaveView(char *filename) const;

    /// Level the view
    void LevelView();

    /// Rotate to horizontal viewing direction
    void HorizontalView();

    /// Fit the view to the data of a specific type or all data (default)
    void FitViewToData(DataType type=NumDataTypes);

    /// Increment rotation around Z-axis
    void IncrementKappa(double angle, bool refresh=false);

//------------------------------------------------------------------------------
// Miscellaneous
//------------------------------------------------------------------------------

    /// Return the const reference to the canvas
    const QGLCanvas &QGLCanvasRef() const {return *this;}

    /// Return the pointer to the canvas
    QGLCanvas *QGLCanvasPtr() {return this;}

    /// Return the const pointer to the canvas
    const QGLCanvas *QGLCanvasPtr() const {return this;}

    /// Select object point nearest to the mouse position
    ObjectPoint *NearestObjectPoint(QMouseEvent *event, DataType type,
                                    DataType selection_type);

    /// Select laser point nearest to the mouse position
    bool NearestLaserPoint(QMouseEvent *event,
                           LaserPoints::const_iterator &nearest_point,
                           DataType &type);
                           
    /// Save the display colour explicitly in a separate copy of the points
    LaserPoints & LaserPointsWithDisplayColour(LaserPoints *laser_points,
                                               const DataAppearance *appearance) const;


    /// Select all points within a circle
    void SelectLaserPointsWithinRadius(LaserPoints &points,
                                       QMouseEvent *event, double radius);

    /// Shift the canvas
    void TranslateCanvas(float x_perc, float y_perc, bool refresh);
    
    /// Scale the canvas
    void ScaleCanvas(float scale, bool refresh);
    
    /// Scale the focal length for the perspective projection
    void ResizeFocalLengthCanvas(float scale_factor, bool refresh);
   
//------------------------------------------------------------------------------
// Slots
//------------------------------------------------------------------------------

  public slots:

    /// Offset data in canvas
    void SetDataOffsetInCanvas(double X, double Y);
    
    /// Shift the canvas
    void TranslateCanvas(float x, float y);
    
    /// Scale the canvas
    void ScaleCanvas(float scale);
    
    /// Scale the focal length for the perspective projection
    void ResizeFocalLengthCanvas(float scale_factor);
   
    /// Paint the canvas
    void PaintCanvas() {update();}
    
//------------------------------------------------------------------------------
// Signals
//------------------------------------------------------------------------------

  signals:
    /// Inform on changed canvas centre
    void CentreOfCanvasChanged(double X, double Y);
    
    /// Translation applied to canvas
    void CanvasTranslated(float x, float y);
    
    /// Scaling applied to canvas
    void CanvasScaled(float scale);
    
    /// Canvas has been painted
    void CanvasPainted();
};

#endif // QGLCANVAS_H
