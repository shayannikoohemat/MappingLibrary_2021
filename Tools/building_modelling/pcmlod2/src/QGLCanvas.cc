
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

#include <stdlib.h>
#include <string.h>
#include "GL/glu.h"
#include "QGLCanvas.h"
#include <QMessageBox>
#include <QKeyEvent>
#include <QMouseEvent>
#include "GeneralUtility.h"

// Normal colours
static GLfloat label_colours[9][3] = { {1.0, 0.3, 0.3},   // Red
                                       {0.2, 0.9, 0.2},   // Green
                                       {0.2, 0.2, 1.0},   // Blue
                                       {0.8, 0.8, 0.0},   // Yellow
                                       {1.0, 0.5, 0.0},   // Orange
                                       {1.0, 1.0, 0.8},   // Light yellow
                                       {0.0, 1.0, 1.0},   // Sea
                                       {1.0, 1.0, 1.0},   // White
                                       {0.0, 0.0, 0.0}};  // Black
        
// Bright colours
static GLfloat highlight_colours[9][3] = { { 1.0, 0.5, 0.5},   // Red
                                           {0.4, 1.0, 0.4},   // Green
                                           {0.2, 0.7, 1.0},   // Blue
                                           {1.0, 1.0, 0.0},   // Yellow
                                           {1.0, 0.7, 0.2},   // Orange
                                           {1.0, 1.0, 0.9},   // Light yellow
                                           {0.3, 1.0, 1.0},   // Sea
                                           {1.0, 1.0, 1.0},   // White
                                           {0.5, 0.5, 0.5}};  // Black

// Arbitrary colours to display segments
static GLfloat random_colours[216][3];

void QGLCanvas::keyPressEvent(QKeyEvent *event)
{
  switch (event->key()) {
    case Qt::Key_Shift  : shift_down = true; break;
    case Qt::Key_Control: ctrl_down = true; break;
    default             : break;
  }
}

void QGLCanvas::keyReleaseEvent(QKeyEvent *event)
{
  switch (event->key()) {
    case Qt::Key_Shift  : shift_down = false; break;
    case Qt::Key_Control: ctrl_down = false; break;
    default             : break;
  }
}

void QGLCanvas::mousePressEvent(QMouseEvent *event)
{
  switch (event->button()) {
    case Qt::LeftButton: button_down[0] = true; break;
    case Qt::MidButton: button_down[1] = true; break;
    case Qt::RightButton: button_down[2] = true; break;
    default:
    case Qt::NoButton: printf("No button\n"); return;
  }
  previous_pos = event->pos();
}

void QGLCanvas::mouseReleaseEvent(QMouseEvent *event)
{
  switch (event->button()) {
    case Qt::LeftButton: button_down[0] = false; break;
    case Qt::MidButton: button_down[1] = false; break;
    case Qt::RightButton: button_down[2] = false; break;
    default:
    case Qt::NoButton: printf("No button\n"); return;
  }
  update();
}

void QGLCanvas::wheelEvent(QWheelEvent *event)
{
  wheelEvent(event, true);
}

void QGLCanvas::wheelEvent(QWheelEvent *event, bool refresh)
{
  float scale;
  if (event->delta() < 0)
    scale = 1.0 - (float) event->delta() * 0.1 / 120.0;
  else
    scale = 1.0 / (1.0 + (float) event->delta() * 0.1 / 120.0);
  zoom_factor *= scale;
  emit CanvasScaled(scale);
  if (refresh) update();
}

void QGLCanvas::IncrementKappa(double angle, bool refresh)
{
  double pi = 4.0 * atan(1.0);

  rotation[1] += angle;
  rot = Rotation3D(0.0, 0.0, -rotation[1]*pi/180.0) *
        Rotation3D(-rotation[0]*pi/180.0, 0.0, 0.0);
  if (refresh) update();
}

void QGLCanvas::mouseMoveEvent(QMouseEvent *event)
{
  mouseMoveEvent(event, true);
}

void QGLCanvas::mouseMoveEvent(QMouseEvent *event, bool refresh)
{
  float rot_speed = 0.2, dx, dy, scale;
  double pi = 4.0 * atan(1.0);
  Position3D canvas_centre;

  if (button_down[1] ||
      (button_down[0] && button_down[2])) {
    dx = (float) (event->x() - previous_pos.x()) * data_range /
                  (width() * zoom_factor);
    dy = (float) (previous_pos.y() - event->y()) * data_range /
                  (height() * zoom_factor);
    translation[0] += dx;
    translation[1] += dy;
    canvas_centre = Canvas2World(QPoint(width()/2, height()/2));
    emit CentreOfCanvasChanged(canvas_centre.X(), canvas_centre.Y());
  }

  else if (button_down[0]) {
    rotation[1] = rotation[1] + rot_speed * (event->x() - previous_pos.x());
    rotation[0] = rotation[0] + rot_speed * (event->y() - previous_pos.y());
    rot = Rotation3D(0.0, 0.0, -rotation[1]*pi/180.0) *
          Rotation3D(-rotation[0]*pi/180.0, 0.0, 0.0);
  }

  else if (button_down[2]) {
    if (event->y() > previous_pos.y())
      scale = 1.0 + (float) (event->y() - previous_pos.y()) / height();
    else
      scale = 1.0 / (1.0 - (float) (event->y() - previous_pos.y()) / height());
    zoom_factor *= scale;
    emit CanvasScaled(scale);
  }

  previous_pos = event->pos();
  if (refresh) update();
}

void QGLCanvas::initializeGL()
{
  glViewport(0, 0, (GLint) width(), (GLint) height());
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_DEPTH_TEST);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

//GLfloat materialAmbient[4] = {0.2, 0.2, 0.2, 1.0};
//GLfloat materialDiffuse[4] = {1.0, 1.0, 1.0, 1.0};
  GLfloat materialSpecular[4] = {0.5, 0.5, 0.5, 1.0};
  GLfloat materialEmission[4] = {0.0, 0.0, 0.0, 1.0};
  GLfloat materialShininess[1] = {120}; // max 128

//glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, materialAmbient);
//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, materialDiffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, materialSpecular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, materialEmission);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, materialShininess);

  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

  GLfloat lightAmbient[4]  = {0.0, 0.0, 0.0, 1.0};
  GLfloat lightDiffuse[4]  = {1.0, 1.0, 1.0, 1.0};
//GLfloat lightSpecular[4] = {1.0, 1.0, 1.0, 1.0};
  GLfloat lightPosition[4] = {500.0, -500.0, 500.0, 0.0};

  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
  glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
//glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);

  glDisable(GL_LIGHT0);
  glDisable(GL_LIGHTING);
}

void QGLCanvas::resizeGL(int width, int height)
{
  glViewport(0, 0, (GLint) width, (GLint) height);
  rescaleGL();
}

void QGLCanvas::rescaleGL()
{
  pixel_size = data_range / min(width(), height());
}

void PrintMatrices(const char *string)
{
  double matmodel[16], matproj[16];
  int i, j, k, l;
  
  printf("%s\n", string);
  printf("Model view matrix                         Projection matrix\n");
  glGetDoublev(GL_MODELVIEW_MATRIX, matmodel);
  glGetDoublev(GL_PROJECTION_MATRIX, matproj);
  for (i=0, k=0, l=0; i<4; i++) {
    for (j=0; j<4; j++, k++)
      printf("%8.3f ", matmodel[k]);
    printf("   ");
    for (j=0; j<4; j++, l++)
      printf("%8.3f ", matproj[l]);
    printf("\n");
  }
}

void QGLCanvas::paintGL()
{
  float dist_to_frustum = 0.001 * data_range;
  float frustum_half_width = dist_to_frustum * data_range / (2.0 * eye_distance);

  // Scale, shift and rotate the data
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  // For perspective viewing, first rotate and translate the scene, then
  // define the eye position.
  if (perspective)
    gluLookAt(0, 0, eye_distance, 0, 0, 0, 0, 1, 0); // eye, look at, up vector
  glScalef(zoom_factor, zoom_factor, zoom_factor);
  glTranslatef(translation[0], translation[1], 0.0);
  glRotatef(rotation[0], 1.0, 0.0, 0.0);
  glRotatef(rotation[1], 0.0, 0.0, 1.0);

  // Project the data
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if (perspective) {
    glFrustum(-frustum_half_width, frustum_half_width,
              -frustum_half_width, frustum_half_width,
              dist_to_frustum, eye_distance + data_range);
  }
  else
    glOrtho(-pixel_size * width() / 2.0,  pixel_size * width() / 2.0,
            -pixel_size * height() / 2.0, pixel_size * height() / 2.0,
            -10.0 * data_range,         10.0 * data_range);

  // Clear the canvas
  glClearColor((float) background_colour.red() / 255.0,
               (float) background_colour.green() / 255.0,
               (float) background_colour.blue() / 255.0, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Redraw all data
  PaintAllData();

  emit CanvasPainted();
}

void QGLCanvas::PaintAllData()
{
  std::vector <ObjectPoints *>::const_iterator         object_point_set;
  std::vector <LineTopPtrVector>::const_iterator       object_top_set;
  std::vector <const DataAppearance *>::const_iterator appearance;
  std::vector <LaserPoints *>::iterator                laser_point_set;
  std::vector <GLuint>::const_iterator                 texture_id;
  std::vector <const DataBounds3D *>::const_iterator   texture_bound;

  for (object_point_set = object_point_sets.begin(),
       object_top_set   = object_top_sets.begin(),
       appearance       = object_appearances.begin();
       object_point_set != object_point_sets.end();
       object_point_set++, object_top_set++, appearance++)
    if ((*appearance)->PaintFirst())
      PaintObjectData(*object_point_set, object_top_set, *appearance);

  for (laser_point_set = laser_point_sets.begin(),
       appearance      = laser_appearances.begin();
       laser_point_set != laser_point_sets.end();
       laser_point_set++, appearance++)
    PaintLaserData(*laser_point_set, *appearance);

  for (object_point_set = object_point_sets.begin(),
       object_top_set   = object_top_sets.begin(),
       appearance       = object_appearances.begin();
       object_point_set != object_point_sets.end();
       object_point_set++, object_top_set++, appearance++)
    if (!(*appearance)->PaintFirst())
      PaintObjectData(*object_point_set, object_top_set, *appearance);

  for (texture_id=texture_ids.begin(), texture_bound=texture_bounds.begin();
       texture_id!=texture_ids.end();
       texture_id++, texture_bound++)
    PaintTextureData(*texture_id, *texture_bound);
}

void QGLCanvas::PaintObjectData(const ObjectPoints *points,
                             std::vector <LineTopPtrVector>::const_iterator top,
                                const DataAppearance *appearance)
{
  ObjectPoints                                 roof_points;
  LineTopologies                               roof_polygons;
  TIN                                          roof_tin;
  TIN::iterator                                roof_mesh;
  ObjectPoints::const_iterator                 point;
  std::vector <LineTopology *>::const_iterator polygon;
  LineTopology::const_iterator                 node;
  Vector3D                                     normal;
  Vector3D                                     light_dir;
  int                                          i, colour_index, tile_level,
                                               tile_row, tile_column, tile_code;
  double                                       height_offset;
  char                                         tile_name[20];

  if (!points) return;

  InitialiseFont();
  
  light_dir = Vector3D(-1.0, 1.0, 1.0).Normalize();
  height_offset = appearance->HeightOffsetForDisplay();

  if (appearance->ShowPoints()) {
    glColor3f(appearance->PointColour().red()   / 255.0,
              appearance->PointColour().green() / 255.0,
              appearance->PointColour().blue()  / 255.0);
    glPointSize((GLfloat) appearance->PointSize());
    glBegin(GL_POINTS);
    for (point=points->begin(); point!=points->end(); point++)
      glVertex3f(point->X() - offset.X(), point->Y() - offset.Y(),
                 point->Z() - offset.Z() + height_offset);
    glEnd();
  }

  if (appearance->ShowLines()) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth((GLfloat) appearance->LineWidth());
    if (appearance->LineColourMethod() == FixedColour)
      glColor3f(appearance->LineColour().red()   / 255.0,
                appearance->LineColour().green() / 255.0,
                appearance->LineColour().blue()  / 255.0);
    for (polygon=top->begin(); polygon!=top->end(); polygon++) {
      glBegin(GL_LINE_STRIP);
      if (appearance->LineColourMethod() == ColourByLabel) {
        colour_index = (*polygon)->Label() - ((*polygon)->Label() / 7) * 7;
        if (appearance->HighLighted())
          glColor3fv(highlight_colours[colour_index]);
        else glColor3fv(label_colours[colour_index]);
      }
      else if (appearance->LineColourMethod() == ColourByLineNumber) {
        colour_index = (*polygon)->Number() - ((*polygon)->Number() / appearance->NumberOfColours()) * appearance->NumberOfColours();
        glColor3fv(random_colours[colour_index]);
      }
      else if (appearance->LineColourMethod() == ColourByBuildingNumber) {
        if ((*polygon)->HasAttribute(BuildingNumberTag))
          colour_index = (*polygon)->Attribute(BuildingNumberTag) -
                         ((*polygon)->Attribute(BuildingNumberTag) / appearance->NumberOfColours()) * appearance->NumberOfColours();
        else colour_index = 0;
        glColor3fv(random_colours[colour_index]);
      }
      else if (appearance->LineColourMethod() == ColourByBuildingPartNumber) {
        if ((*polygon)->HasAttribute(BuildingPartNumberTag))
          colour_index = (*polygon)->Attribute(BuildingPartNumberTag) -
                         ((*polygon)->Attribute(BuildingPartNumberTag) / appearance->NumberOfColours()) * appearance->NumberOfColours();
        else colour_index = 0;
        glColor3fv(random_colours[colour_index]);
      }
      for (node=(*polygon)->begin(); node!=(*polygon)->end(); node++) {
        point = points->ConstPointIterator(node->Number());
        glVertex3f(point->X() - offset.X(), point->Y() - offset.Y(),
                   point->Z() - offset.Z() + height_offset);
      }
      glEnd();
      if (appearance->FirstPointSize() && !(*polygon)->empty()) {
        point = points->ConstPointIterator((*polygon)->begin()->Number());
        glPointSize(appearance->FirstPointSize());
        glBegin(GL_POINTS);
        glVertex3f(point->X() - offset.X(), point->Y() - offset.Y(),
                   point->Z() - offset.Z() + height_offset);
        glEnd();
      }
      if (appearance->TypeOfData() == TileBoundaryData) {
        QPoint canvas_point = World2Canvas(point->Position3DRef(), true);
        canvas_point.setX(canvas_point.x()+2);
        canvas_point.setY(canvas_point.y()+20);
        Position3D rounded_point = Canvas2World(canvas_point, 200.0, true);
        tile_code   = (*polygon)->Number();
        tile_level  = tile_code / 1000000;
        tile_row    = (tile_code - tile_level * 1000000) / 1000;
        tile_column = tile_code - tile_level * 1000000 - tile_row * 1000;
        sprintf(tile_name, "(%d, %d, %d)", tile_level, tile_row, tile_column);
        glEnable(GL_TEXTURE_2D);
		text_font.Begin();
		text_font.TextOutF(std::string(tile_name),
		                   (float) (rounded_point.X() - offset.X()),
					       (float) (rounded_point.Y() - offset.Y()),
					       (float) (rounded_point.Z() - offset.Z()), 
					       (2.57142854/18.0) * zoom_factor / pixel_size);
        glDisable(GL_TEXTURE_2D);
      }
    }
  }

  if (appearance->ShowFaces()) {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    if (appearance->FaceColourMethod() == FixedColour)
      glColor3f(appearance->FaceColour().red()   / 255.0,
                appearance->FaceColour().green() / 255.0,
                appearance->FaceColour().blue()  / 255.0);
    for (polygon=top->begin(); polygon!=top->end(); polygon++) {
      if ((*polygon)->size() > 1) {
        if ((*polygon)->IsClosed()) {
          glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
          glBegin(GL_POLYGON);
          normal = (*polygon)->Normal(points->ObjectPointsRef(), 1);
          if (normal.Z() < 0.01) normal *= -1.0; // Ensure upward normal
          glNormal3f(normal.X(), normal.Y(), normal.Z());
        }
        else {
          glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
          glBegin(GL_LINE_STRIP);
        }
        if (appearance->FaceColourMethod() == ColourByLabel) {
          colour_index = (*polygon)->Label() - ((*polygon)->Label() / 7) * 7;
          if (appearance->HighLighted()) 
            glColor3fv(highlight_colours[colour_index]);
          else glColor3fv(label_colours[colour_index]);
        }
        else if (appearance->FaceColourMethod() == ColourByLineNumber) {
          colour_index = (*polygon)->Number() - ((*polygon)->Number() / appearance->NumberOfColours()) * appearance->NumberOfColours();
          glColor3fv(random_colours[colour_index]);
        }
        else if (appearance->FaceColourMethod() == ColourByBuildingNumber) {
          if ((*polygon)->HasAttribute(BuildingNumberTag))
            colour_index = (*polygon)->Attribute(BuildingNumberTag) -
                           ((*polygon)->Attribute(BuildingNumberTag) / appearance->NumberOfColours()) * appearance->NumberOfColours();
          else colour_index = 0;
          glColor3fv(random_colours[colour_index]);
        }
        else if (appearance->FaceColourMethod() == ColourByBuildingPartNumber) {
          if ((*polygon)->HasAttribute(BuildingPartNumberTag))
            colour_index = (*polygon)->Attribute(BuildingPartNumberTag) -
                           ((*polygon)->Attribute(BuildingPartNumberTag) / appearance->NumberOfColours()) * appearance->NumberOfColours();
          else colour_index = 0;
          glColor3fv(random_colours[colour_index]);
        }
  
        // Triangulate non-vertical polygons of 5 or more corners
        if (normal.Z() > 0.01 && (*polygon)->IsClosed() &&
            (*polygon)->size() > 5 && appearance->TriangulateFaces()) {
          // Collect all polygon points
          roof_points.Erase();
          roof_polygons.Erase();
          for (node=(*polygon)->begin(); node!=(*polygon)->end()-1; node++) {
            point = points->ConstPointIterator(node->Number());
            roof_points.push_back(*point);
          }
          roof_polygons.push_back(**polygon);
          // Renumber points and polygon
          roof_polygons.ReNumber(roof_points);
          // Triangulate polygon
          roof_tin = roof_points.Triangulate(roof_polygons);
          // Visualise triangles
          for (roof_mesh=roof_tin.begin(); roof_mesh!=roof_tin.end(); roof_mesh++) {
            if (roof_mesh != roof_tin.begin()) {
              glBegin(GL_POLYGON);
              glNormal3f(normal.X(), normal.Y(), normal.Z());
            }
            for (i=0; i<3; i++) {
              point = roof_points.ConstPointIterator(roof_mesh->Nodes()[i]);
              glVertex3f(point->X() - offset.X(), point->Y() - offset.Y(),
                         point->Z() - offset.Z() + height_offset);
            }
            glEnd();
          }    
          // Deallocate TIN
          roof_tin.Erase();
        }
        else {
          for (node=(*polygon)->begin(); node!=(*polygon)->end()-1; node++) {
            point = points->ConstPointIterator(node->Number());
            glVertex3f(point->X() - offset.X(), point->Y() - offset.Y(),
                       point->Z() - offset.Z() + height_offset);
          }
          glEnd();
        }
      }
    }
    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);
  }
}

void QGLCanvas::SetBackGroundColour(const QColor &colour)
{
  background_colour = colour;
  update();
}

void ColourAttribute(double attribute, double colour_cycle_length,
                     double colour_cycle_phase,
                     int &red, int &green, int &blue)
{
  double attribute2 = attribute + colour_cycle_length * colour_cycle_phase;
  if (attribute2 < 0.0) 
    attribute2 += (int) ((-attribute2 / colour_cycle_length) + 1) *
                  colour_cycle_length;
  int    i = (int) (attribute2 / colour_cycle_length);
  
  attribute2 -= i * colour_cycle_length; // [ 0 - colour_cycle_lenght ]
  i = (int) (attribute2 * 600.0 / colour_cycle_length); // [ 0 - 600 ]
/*
  int i = (int) (attribute * 100.0);
  i -= (i / (colour_cycle_length * 100)) * (colour_cycle_length * 100);
  i = (i * 6) / colour_cycle_size;
*/
// i from 0 till 600
  // Red
  if (i < 200) red = 255;
  else if (i < 300) red = 255 - ((i - 200) * 255) / 100;
  else if (i >= 500) red = ((i - 500) * 255) / 100;
  else red = 0;
  // Green
  if (i >= 100 && i < 200) green = ((i - 100) * 255) / 100;
  else if (i >=200 && i < 400) green = 255;
  else if (i >=400 && i < 500)
    green = 255 - ((i - 400) * 255) / 100;
  else green = 0;
  // Blue
  if (i < 100) blue = 255 - (i * 255) / 100;
  else if (i >= 300 && i < 400) blue = ((i - 300) * 255) / 100;
  else if (i >= 400) blue = 255;
  else blue = 0;
  
  // Mix with white
  red   = (int) (0.7 * red   + 0.3 * 255);
  green = (int) (0.7 * green + 0.3 * 255);
  blue  = (int) (0.7 * blue  + 0.3 * 255);
}
        
void ColourAttribute_error(double attribute, double max_val,
                     int &red, int &green, int &blue)
{
 //from green to red, use HSV, it is easier:
	//S=V=100,H: 120=green, 0=red, scale the data approprietly and compute RGB from it
	if (attribute > max_val) attribute=max_val;
	
	double H=120-attribute*120/max_val;
	double S=1;
	double V=1;
	//Hi as integer : Hi = (H / 60.0) mod 6
	int Hi=(int) (H / 60.0);
	   //`f is a fraction of the section
	   //f as float : f = (H / 60.0) - Hi
	double f=(H / 60.0) - Hi;
	
	   //`p, q and t are temp variables used to work out the rgb values before we know where to put them
	  /* p as float : p = V * (1.0 - S)
	   q as float : q = V * (1.0 - (f * S))
	   t as float : t = V * (1.0 - ((1.0 - f) * S))
	   */
	   	double p = V * (1.0 - S);
	    double q = V * (1.0 - (f * S));
	    double t = V * (1.0 - ((1.0 - f) * S));
	    

	  // `Depending on the section worked out above, we store the rgb value appropriately...
	   //R as float : G as float : B as float
	    float R,G,B;
	    
	   switch(Hi)
	   {
	      case 0: R=V ; G=t; B=p; break;
	      case 1: R=q ; G=V; B=p; break;
	      case 2: R=p ; G=V ; B=t ; break;
	      case 3: R=p ; G=q ; B=V ; break;
	      case 4: R=t ; G=p ; B=V ; break;
	      case 5: R=V ; G=p ; B=q ; break;
	   }

	   //`Sanitize the output - just incase of floating point errors...
	   if (R < 0.0) R = 0.0; else if (R > 1.0) R = 1.0;
	   if (G < 0.0) G = 0.0; else if (G > 1.0) G = 1.0;
	   if (B < 0.0) B = 0.0; else if (B > 1.0) B = 1.0;

	   //`Convert to Bytes and then convert to a DWORD using the rgb() function
	    red= (int) (R * 255);
	   green=(int) (G * 255);
	  blue = (int) (B * 255);
	
}

QColor ColourBasedOnThresholds(LaserPoints::const_iterator point,
                               const DataAppearance *appearance)
{
  double value, diflow, difhigh;
  int    i, num_thresholds;
  
  // Return default if the point does not have the attribute
  if (!point->HasAttribute(appearance->ThresholdAttribute()))
    return appearance->NoAttributeColour();

  // Get attribute value
  switch (AttributeType(appearance->ThresholdAttribute())) {
    default:
    case IntegerAttributeType:
      value = (double) point->Attribute(appearance->ThresholdAttribute());
      break;
    case FloatAttributeType:
      value = (double) point->FloatAttribute(appearance->ThresholdAttribute());
      break;
    case DoubleAttributeType:
      value = point->DoubleAttribute(appearance->ThresholdAttribute());
      break;
  }
    
  // Fixed colour in between thresholds
  num_thresholds = appearance->NumberOfThresholds();
  if (appearance->ColoursInBetweenThresholds()) {
    for (i=0; i<num_thresholds; i++)
      if (value < appearance->Thresholds()[i]) {
        return appearance->ThresholdColours()[i];
      }
    return appearance->ThresholdColours()[num_thresholds];
  }
  else { // Gradual transition between colours on threshold values
    if (value < appearance->Thresholds()[0])
      return appearance->ThresholdColours()[0];
    for (i=1; i<num_thresholds; i++) {
      if (value < appearance->Thresholds()[i]) {
        diflow  = value - appearance->Thresholds()[i-1];
        difhigh = appearance->Thresholds()[i] - value;
        return QColor((int) (((float) appearance->ThresholdColours()[i-1].red() * difhigh +
                              (float) appearance->ThresholdColours()[i].red() * diflow) /
                             (diflow + difhigh)),
                      (int) (((float) appearance->ThresholdColours()[i-1].green() * difhigh +
                              (float) appearance->ThresholdColours()[i].green() * diflow) /
                             (diflow + difhigh)),
                      (int) (((float) appearance->ThresholdColours()[i-1].blue() * difhigh +
                              (float) appearance->ThresholdColours()[i].blue() * diflow) /
                             (diflow + difhigh)));
      }
    }
    return appearance->ThresholdColours()[num_thresholds-1];
  }
  return QColor(255, 255, 255); // Just to satisfy compiler
}

                                       

void QGLCanvas::PaintLaserData(LaserPoints *laser_points,
                               const DataAppearance *appearance)
{
  LaserPoints::const_iterator laser_point, laser_point2, laser_point3;
  GLfloat                     intensity_scale, intensity_offset, intensity,
                              height_offset, height_scale,
                              pulselength_offset, pulselength_scale,
                              flred, flgreen, flblue; 
  QColor                      colour;
  int                         i, red, green, blue, colour_index, value,
                              point_increment;
  float		                  stddev, residual;
  TIN::const_iterator         mesh;
  const PointNumber           *nodeptr;
  Plane                       plane;
  double                      gamma, dvalue;
  bool                        skip, use_filter;
  TINEdges                    *edges;
  TINEdges::iterator          edgeset;
  TINEdgeSet::iterator        node;
  SegmentationParameters      segm_par;
  long long                   lvalue;

  if (!laser_points) return;

  Vector3D        light_dir = Vector3D(-1.0, 1.0, 1.0).Normalize();
  LaserPointType  type = laser_points->Scanner().PointType();
  DataBoundsLaser bounds = laser_points->DataBounds();
  intensity_scale  = 0.75 / (float) bounds.ReflectanceRange();
  intensity_offset = 0.25 - 0.75 * (float) bounds.Minimum().Reflectance() /
                     (float) bounds.ReflectanceRange();
  height_scale  = 0.75 / (float) bounds.ZRange();
  height_offset = 0.25 - 0.75 * (float) bounds.Minimum().Z() /
                  (float) bounds.ZRange();
  pulselength_scale  = 0.75 / (float) (bounds.Maximum().PulseLength() -
                                       bounds.Minimum().PulseLength());
  pulselength_offset = 0.25 - (float) bounds.Minimum().PulseLength() *
                       pulselength_scale;
  gamma = appearance->PointColourGamma();
  use_filter = appearance->UseDisplayFilter();

  if (appearance->ShowPoints()) {
    if (appearance->PointColourMethod() == FixedColour)
      glColor3f(appearance->PointColour().red()   / 255.0,
                appearance->PointColour().green() / 255.0,
                appearance->PointColour().blue()  / 255.0);
    glPointSize((GLfloat) appearance->PointSize());
    // Check need for subsampling
    if (button_down[0] || button_down[1] || button_down[2])
      point_increment = laser_points->size() /
                        appearance->MaximumNumberOfPointsToDisplay() + 1;
    else
      point_increment = 1;
    glBegin(GL_POINTS);
    skip = false;
    for (laser_point=laser_points->begin(); laser_point<laser_points->end();
         laser_point+=point_increment) {
      if (use_filter) {
      	skip = !appearance->DisplayFilter().InsideAttributes(&*laser_point);
      	if (!skip)
      	  skip = !laser_point->IsPulseType(appearance->SelectedPulseType());
      }
      if (!skip) {
        switch (appearance->PointColourMethod()) {

          default:
          case FixedColour:
            break;
  
          case ColourByLabel:
            if (laser_point->HasLabel()) {
              colour_index = laser_point->Label() - (laser_point->Label()/7) * 7;
              glColor3fv(label_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;
  
          case ColourByReflectance:
            intensity = laser_point->Reflectance() * intensity_scale +
                        intensity_offset;
            if (gamma != 1.0) intensity = (float) pow((double) intensity, 1.0/gamma);
            glColor3f(intensity, intensity, intensity);
            break;
            
          case ColourByColour:
            if (type == ColourPoint || type == MultiColourPoint ||
                appearance->PointColourMethod() == ColourByColour) {
              flred   = laser_point->Red() / 255.0;
              flgreen = laser_point->Green() / 255.0;
              flblue  = laser_point->Blue() / 255.0;
              // Apply gamma correction
              if (gamma != 1.0) {
                flred   = (float) pow((double) flred, 1.0/gamma);
                flgreen = (float) pow((double) flgreen, 1.0/gamma);
                flblue  = (float) pow((double) flblue, 1.0/gamma);
              }
              glColor3f(flred, flgreen, flblue);
            }
            else glColor3f(1.0, 1.0, 1.0);
            break;
          
          case ColourByHeight:
          	dvalue = laser_point->Z();
          case ColourByTime:
          	if (appearance->PointColourMethod() == ColourByTime)
          	  dvalue = laser_point->DoubleAttribute(TimeTag) / 60;
            ColourAttribute(dvalue,
                            appearance->PointColourCycleLength(),
                            appearance->PointColourCyclePhase(),
                            red, green, blue);
            glColor3f(red / 255.0, green / 255.0, blue / 255.0);
            break;

          case GreyToneByHeight:
            intensity = laser_point->Z() * height_scale + height_offset;
            glColor3f(intensity, intensity, intensity);
            break;

          case ColourByResidual:
            residual = laser_point->Residual();
            if (fabs(residual) < 0.2)
              glColor3fv(label_colours[ColourGreen]);
            else if (fabs(residual) < 0.5)
              glColor3fv(label_colours[ColourYellow]);
            else glColor3fv(label_colours[ColourRed]);
            break;

          case ColourBySegment:
            if (laser_point->HasAttribute(SegmentNumberTag)) {
              value = laser_point->Attribute(SegmentNumberTag);
              colour_index = value - (value/appearance->NumberOfColours()) * appearance->NumberOfColours();
              glColor3fv(random_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;
  
          case ColourBySegmentAndTile:
            if (laser_point->HasAttribute(SegmentNumberTag) &&
			    laser_point->HasAttribute(SegmentStartTileNumberTag)) {
              lvalue = (long long) laser_point->Attribute(SegmentStartTileNumberTag) *
			           1000000 +
					   (long long) laser_point->Attribute(SegmentNumberTag);
              colour_index = (int) (lvalue - (lvalue/appearance->NumberOfColours()) * appearance->NumberOfColours());
              glColor3fv(random_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;
  
          case ColourByComponent:
            if (laser_point->HasAttribute(ComponentNumberTag)) {
              value = laser_point->Attribute(ComponentNumberTag);
              colour_index = value - (value/appearance->NumberOfColours()) * appearance->NumberOfColours();
              glColor3fv(random_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;
  
          case ColourByPlane:
            if (laser_point->HasAttribute(PlaneNumberTag)) {
              value = laser_point->Attribute(PlaneNumberTag);
              colour_index = value - (value/appearance->NumberOfColours()) * appearance->NumberOfColours();
              glColor3fv(random_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break; 

          case GreyToneByPulseLength:
            intensity = laser_point->PulseLength() * pulselength_scale + 
                        pulselength_offset;
            glColor3f(intensity, intensity, intensity);
            break;

          case ColourByScanNumber:
          case ColourByScanNumberWithoutAFN:
          case ColourByAFNCode:
            if (laser_point->HasAttribute(ScanNumberTag)) {
              value = laser_point->Attribute(ScanNumberTag);
              switch (appearance->PointColourMethod()) {
                case ColourByScanNumberWithoutAFN: value /= 10; break;
                case ColourByAFNCode: value -= 10 * (value/10); break;
                default: break;
              }
              colour_index = value - (value/appearance->NumberOfColours()) * appearance->NumberOfColours();
              glColor3fv(random_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;

          case ColourByPulseCount:
            if (laser_point->HasAttribute(PulseCountTag)) {
              value = laser_point->PulseCount();
              colour_index = max(value - (value/9) * 9 - 1, 0);
              glColor3fv(label_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;
  
          case ColourByFilterStatus:
            if (laser_point->HasAttribute(IsFilteredTag)) {
              value = laser_point->Filtered();
              if (value == 0) // Terrain
                glColor3fv(label_colours[ColourGreen]);
              else // Non-terrain
                glColor3fv(label_colours[ColourRed]);
//            glColor3fv(label_colours[laser_point->Attribute(IsFilteredTag)]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;
          
          case ColourByThresholding:
            colour = ColourBasedOnThresholds(laser_point, appearance);
            glColor3f((float) colour.red() / 255.0,
                      (float) colour.green() / 255.0,
                      (float) colour.blue() / 255.0);
            break;
            
          case ColourByScanLineNumber:
            if (laser_point->HasAttribute(ScanLineNumberTag)) {
              value = laser_point->Attribute(ScanLineNumberTag);
              colour_index = value - (value/appearance->NumberOfColours()) * appearance->NumberOfColours();
              glColor3fv(random_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;
  
           case ColourByStdDev_X:
            if (laser_point->HasAttribute(v_xTag)) {
		stddev = sqrt(laser_point->FloatAttribute(v_xTag));
		if (stddev > appearance->PointColourCycleLength()) break;
                ColourAttribute_error(stddev, appearance->PointColourCycleLength(),
                    red, green, blue);
                //ColourAttribute(variance,
                //            appearance->PointColourCycleLength(),
                //            appearance->PointColourCyclePhase(),
                //            red, green, blue);
            glColor3f(red / 255.0, green / 255.0, blue / 255.0);
	 }
 	else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
          break;
         case ColourByStdDev_Y:
            if (laser_point->HasAttribute(v_yTag)) {
		stddev = sqrt(laser_point->FloatAttribute(v_yTag));
                if (stddev > appearance->PointColourCycleLength()) break;
		ColourAttribute_error(stddev, appearance->PointColourCycleLength(),
                    red, green, blue);
                //ColourAttribute(variance,
                //            appearance->PointColourCycleLength(),
                //            appearance->PointColourCyclePhase(),
                //            red, green, blue);
            glColor3f(red / 255.0, green / 255.0, blue / 255.0);
	 }
 	else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
          break;
	case ColourByStdDev_Z:
            if (laser_point->HasAttribute(v_zTag)) {
		stddev = sqrt(laser_point->FloatAttribute(v_zTag));
                if (stddev > appearance->PointColourCycleLength()) break;
		ColourAttribute_error(stddev, appearance->PointColourCycleLength(),
                    red, green, blue);
                //ColourAttribute(variance,
                //            appearance->PointColourCycleLength(),
                //            appearance->PointColourCyclePhase(),
                //            red, green, blue);
            glColor3f(red / 255.0, green / 255.0, blue / 255.0);
	 }
 	else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
          break;
        }
        
        glVertex3f((float) (laser_point->X() - offset.X()),
                   (float) (laser_point->Y() - offset.Y()),
                   (float) (laser_point->Z() - offset.Z()));
      }
    }
    glEnd();
  }

  if (appearance->ShowLines()) {
    edges = laser_points->GetNeighbourhoodEdges();
    if (edges) { // Verify the edges are up to date
      if (edges->size() != laser_points->size()) {
        laser_points->EraseNeighbourhoodEdges();
        laser_points->EraseTIN();
        edges = NULL;
      }
    }
    if (!edges) { // Generate a TIN and its edges
      segm_par.NeighbourhoodStorageModel() = 0;
      segm_par.DistanceMetricDimension() = 2;
      segm_par.MaxDistanceInComponent() = 0.0;
      edges = laser_points->DeriveEdges(segm_par);
    }
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor3f(appearance->LineColour().red()   / 255.0,
              appearance->LineColour().green() / 255.0,
              appearance->LineColour().blue()  / 255.0);
    glLineWidth((GLfloat) appearance->LineWidth());
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    for (laser_point=laser_points->begin(), edgeset=edges->begin();
         laser_point!=laser_points->end(); laser_point++, edgeset++) {
      for (node=edgeset->begin(); node!=edgeset->end(); node++) {
        laser_point2 = laser_points->begin() + node->Number();
        skip = false;
        if (appearance->OnlyShowLinesInSegment()) {
          skip = true;
          if (laser_point->HasAttribute(SegmentNumberTag) &&
              laser_point2->HasAttribute(SegmentNumberTag))
            if (laser_point->Attribute(SegmentNumberTag) ==
                laser_point2->Attribute(SegmentNumberTag))
              skip = false;
        }
        if (!skip) {
          glBegin(GL_LINE_STRIP);
          glVertex3f((float) (laser_point->X() - offset.X()),
                     (float) (laser_point->Y() - offset.Y()),
                     (float) (laser_point->Z() - offset.Z()));
          glVertex3f((float) (laser_point2->X() - offset.X()),
                     (float) (laser_point2->Y() - offset.Y()),
                     (float) (laser_point2->Z() - offset.Z()));
          glEnd();
        }
      }
    }
/*    
    glBegin(GL_TRIANGLES);
    for (mesh=laser_points->TINReference().begin();
         mesh!=laser_points->TINReference().end(); mesh++) {
      skip = false;
      if (appearance->OnlyShowLinesInSegment()) {    
        nodeptr=mesh->Nodes();
        laser_point  = laser_points->begin() + nodeptr->Number(); nodeptr++;
        laser_point2 = laser_points->begin() + nodeptr->Number(); nodeptr++;
        laser_point3 = laser_points->begin() + nodeptr->Number();
        skip = true;
        if (laser_point->HasAttribute(SegmentNumberTag) &&
            laser_point2->HasAttribute(SegmentNumberTag) &&
            laser_point3->HasAttribute(SegmentNumberTag))
          if (laser_point->Attribute(SegmentNumberTag) ==
              laser_point2->Attribute(SegmentNumberTag) &&
              laser_point->Attribute(SegmentNumberTag) ==
              laser_point3->Attribute(SegmentNumberTag))
            skip = false;
      }
      if (!skip) {
        for (i=0, nodeptr=mesh->Nodes(); i<3; i++, nodeptr++) {
          laser_point = laser_points->begin() + nodeptr->Number();
          glVertex3f((float) (laser_point->X() - offset.X()),
                     (float) (laser_point->Y() - offset.Y()),
                     (float) (laser_point->Z() - offset.Z()));
        }
      }
    }
    glEnd();
*/
  }

  if (appearance->ShowFaces()) {
    if (!laser_points->GetTIN()) laser_points->DeriveTIN();
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    if (appearance->FaceColourMethod() == FixedColour)
      glColor3f(appearance->FaceColour().red()   / 255.0,
                appearance->FaceColour().green() / 255.0,
                appearance->FaceColour().blue()  / 255.0);
    for (mesh=laser_points->TINReference().begin();
         mesh!=laser_points->TINReference().end(); mesh++) {
      glBegin(GL_POLYGON);
      nodeptr=mesh->Nodes();
      laser_point  = laser_points->begin() + nodeptr->Number(); nodeptr++;
      laser_point2 = laser_points->begin() + nodeptr->Number(); nodeptr++;
      laser_point3 = laser_points->begin() + nodeptr->Number();
      skip = false;
      if (appearance->OnlyShowFacesInSegment()) {
        skip = true;
        if (laser_point->HasAttribute(SegmentNumberTag) &&
            laser_point2->HasAttribute(SegmentNumberTag) &&
            laser_point3->HasAttribute(SegmentNumberTag))
          if (laser_point->Attribute(SegmentNumberTag) ==
              laser_point2->Attribute(SegmentNumberTag) &&
              laser_point->Attribute(SegmentNumberTag) ==
              laser_point3->Attribute(SegmentNumberTag))
            skip = false;
      }
      if (!skip) {
        plane = Plane(laser_point->Position3DRef(),
                      laser_point2->Position3DRef(),
                      laser_point3->Position3DRef());
        switch (appearance->FaceColourMethod()) {
          case FixedColour:
            break;
  
          case ColourByLabel:
            // Use label of first point
            if (laser_point->HasLabel()) {
              colour_index = laser_point->Label() - (laser_point->Label()/7) * 7;
              glColor3fv(label_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;
  
          case ColourByReflectance:
          case ColourByColour:
            // Average colours or reflectances
            if (type == ColourPoint || type == MultiColourPoint ||
                appearance->PointColourMethod() == ColourByColour)
              glColor3f((laser_point->Red() + laser_point2->Red() + 
                         laser_point3->Red()) / 765.0,
                        (laser_point->Green() + laser_point2->Green() +
                         laser_point3->Green()) / 765.0,
                        (laser_point->Blue() + laser_point2->Blue() +
                         laser_point3->Blue()) / 765.0);
           else {
              intensity = (laser_point->Reflectance() + laser_point2->Reflectance() +
                           laser_point3->Reflectance()) * intensity_scale / 3.0 +
                          intensity_offset;
              glColor3f(intensity, intensity, intensity);
            }
            break;
        
          case ColourByHeight:
            // Average height
            dvalue = (laser_point->Z() + laser_point2->Z() +
                             laser_point3->Z()) / 3.0;
          case ColourByTime:
          	if (appearance->PointColourMethod() == ColourByTime)
          	  dvalue = (laser_point->DoubleAttribute(TimeTag) +
				        laser_point2->DoubleAttribute(TimeTag) +
                        laser_point3->DoubleAttribute(TimeTag)) / 180.0;
            ColourAttribute(dvalue,
                            appearance->PointColourCycleLength(),
                            appearance->PointColourCyclePhase(),
                            red, green, blue);
            glColor3f(red / 255.0, green / 255.0, blue / 255.0);
            break;

          case GreyToneByHeight:
            intensity = (laser_point->Z() + laser_point2->Z() + laser_point3->Z())
                        * height_scale / 3.0 + height_offset;
            glColor3f(intensity, intensity, intensity);
            break;

          case ColourByResidual:
            residual = (fabs(laser_point->Residual()) + fabs(laser_point2->Residual()) +
                        fabs(laser_point3->Residual())) / 3.0;
            if (residual < 0.2)
              glColor3fv(label_colours[ColourGreen]);
            else if (residual < 0.5)
              glColor3fv(label_colours[ColourYellow]);
            else glColor3fv(label_colours[ColourRed]);
            break;

          case ColourBySegment:
            if (laser_point->HasAttribute(SegmentNumberTag)) {
              value = laser_point->Attribute(SegmentNumberTag);
              colour_index = value - (value/appearance->NumberOfColours()) * appearance->NumberOfColours();
              glColor3fv(random_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;
  
          case ColourBySegmentAndTile:
            if (laser_point->HasAttribute(SegmentNumberTag) &&
			    laser_point->HasAttribute(SegmentStartTileNumberTag)) {
              lvalue = (long long) laser_point->Attribute(SegmentStartTileNumberTag) *
			           1000000 +
					   (long long) laser_point->Attribute(SegmentNumberTag);
              colour_index = (int) (lvalue - (lvalue/appearance->NumberOfColours()) * appearance->NumberOfColours());
              glColor3fv(random_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;
  
          case ColourByComponent:
            if (laser_point->HasAttribute(ComponentNumberTag)) {
              value = laser_point->Attribute(ComponentNumberTag);
              colour_index = value - (value/appearance->NumberOfColours()) * appearance->NumberOfColours();
              glColor3fv(random_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;
  
          case ColourByPlane:
            if (laser_point->HasAttribute(PlaneNumberTag)) {
              value = laser_point->Attribute(PlaneNumberTag);
              colour_index = value - (value/appearance->NumberOfColours()) * appearance->NumberOfColours();
              glColor3fv(random_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break; 

          case GreyToneByPulseLength:
            intensity = (laser_point->PulseLength() + laser_point2->PulseLength() +
                         laser_point3->PulseLength()) * pulselength_scale / 3.0 + 
                        pulselength_offset;
            glColor3f(intensity, intensity, intensity);
            break;

          case ColourByScanNumber:
          case ColourByScanNumberWithoutAFN:
          case ColourByAFNCode:
            if (laser_point->HasAttribute(ScanNumberTag)) {
              value = laser_point->Attribute(ScanNumberTag);
              switch (appearance->PointColourMethod()) {
                case ColourByScanNumberWithoutAFN: value /= 10; break;
                case ColourByAFNCode: value -= 10 * (value/10); break;
                default: break;
              }
              colour_index = value - (value/appearance->NumberOfColours()) * appearance->NumberOfColours();
              glColor3fv(random_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;

          case ColourByPulseCount:
            if (laser_point->HasAttribute(PulseCountTag)) {
              value = laser_point->PulseCount();
              colour_index = max(value - (value/9) * 9 - 1, 0);
              glColor3fv(label_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;
  
          case ColourByScanLineNumber:
            if (laser_point->HasAttribute(ScanLineNumberTag)) {
              value = laser_point->Attribute(ScanLineNumberTag);
              colour_index = value - (value/appearance->NumberOfColours()) * appearance->NumberOfColours();
              glColor3fv(random_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;
  
          case ColourByFilterStatus:
            if (laser_point->HasAttribute(IsFilteredTag)) {
              value = laser_point->Filtered();
              if (value == 0) // Terrain
                glColor3fv(label_colours[ColourGreen]);
              else // Non-terrain
                glColor3fv(label_colours[ColourRed]);
            }
            else // Unlabeled points in white
             glColor3f(1.0, 1.0, 1.0);
            break;
        
          // Don't have to deal with surfaces coloured by other means
          default: break;
        }
        glNormal3f(plane.Normal().X(), plane.Normal().Y(), plane.Normal().Z());
        for (i=0, nodeptr=mesh->Nodes(); i<3; i++, nodeptr++) {
          laser_point = laser_points->begin() + nodeptr->Number();
          glVertex3f((float) (laser_point->X() - offset.X()),
                     (float) (laser_point->Y() - offset.Y()),
                     (float) (laser_point->Z() - offset.Z()));
        }
        glEnd();
      }
    }
    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);
  }
}

/*
-------------------------------------------------------------------------------
                            Set data pointers
-------------------------------------------------------------------------------
*/

void QGLCanvas::AddObjectData(ObjectPoints *new_point_set,
                              LineTopologies *new_top_set,
                              const DataAppearance *new_appearance,
                              bool check_if_present,
                              bool refresh)
{
  if (new_top_set == NULL)
    AddObjectData(new_point_set, (LineTopology *) NULL, new_appearance,
                  check_if_present, false);
  else {
    for (LineTopologies::iterator polygon=new_top_set->begin();
         polygon!=new_top_set->end(); polygon++)
      AddObjectData(new_point_set, &*polygon, new_appearance,
                    check_if_present, false);
  }
  if (refresh) update();
}

void QGLCanvas::AddObjectData(ObjectPoints *new_point_set,
                              LineTopology *new_top,
                              const DataAppearance *new_appearance,
                              bool check_if_present,
                              bool refresh)
{
  std::vector <ObjectPoints *>::iterator         point_set;
  std::vector <LineTopPtrVector>::iterator       top_set, correct_top_set;
  std::vector <const DataAppearance *>::iterator appearance;
  LineTopPtrVector::iterator                     top;

  // Locate entry based on points and appearance pointers
  bool found=false;
  for (point_set  = object_point_sets.begin(),
       top_set    = object_top_sets.begin(),
       appearance = object_appearances.begin();
       point_set != object_point_sets.end() && !found;
       point_set++, top_set++, appearance++)
    if (*point_set == new_point_set && *appearance == new_appearance) {
      found = true;
      correct_top_set = top_set;
    }

  // If required, create a new entry
  if (!found) {
    object_point_sets.push_back(new_point_set);
    object_top_sets.push_back(LineTopPtrVector());
    correct_top_set = object_top_sets.end() - 1;
    object_appearances.push_back(new_appearance);
  }

  // Check on existance of polygon if required
  else if (check_if_present) {
    if (new_top != NULL) {
      for (top=correct_top_set->begin(); top!=correct_top_set->end(); top++)
        if (*top == new_top) return;
    }
  }

  // Add the polygon
  if (new_top != NULL) correct_top_set->push_back(new_top);
  if (refresh) update();
}

bool QGLCanvas::RemoveObjectData(LineTopologies *old_top_set, bool refresh)
{
  bool removed_all=true;
  for (LineTopologies::iterator polygon=old_top_set->begin();
       polygon!=old_top_set->end(); polygon++)
    removed_all = removed_all && RemoveObjectData(&*polygon, false);
  if (refresh) update();
  return removed_all;
}

bool QGLCanvas::RemoveObjectData(LineTopology *old_top, bool refresh)
{
  std::vector <ObjectPoints *>::iterator         point_set;
  std::vector <LineTopPtrVector>::iterator       top_set;
  std::vector <const DataAppearance *>::iterator appearance;
  LineTopPtrVector::iterator                     top;

  for (point_set  = object_point_sets.begin(),
       top_set    = object_top_sets.begin(),
       appearance = object_appearances.begin();
       point_set != object_point_sets.end();
       point_set++, top_set++, appearance++) {
    for (top=top_set->begin(); top!=top_set->end(); top++) {
      if (*top == old_top) {
        top_set->erase(top);
        if (top_set->empty()) {
          object_point_sets.erase(point_set);
          object_top_sets.erase(top_set);
          object_appearances.erase(appearance);
        }
        if (refresh) update();
        return true;
      }
    }
  }
  return false;
}

bool QGLCanvas::RemoveObjectData(const DataAppearance *old_appearance,
                                 bool refresh)
{
  std::vector <ObjectPoints *>::iterator         point_set;
  std::vector <LineTopPtrVector>::iterator       top_set;
  std::vector <const DataAppearance *>::iterator appearance;
  bool removed_object=false;

  for (point_set  = object_point_sets.begin(),
       top_set    = object_top_sets.begin(),
       appearance = object_appearances.begin();
       point_set != object_point_sets.end();
       point_set++, top_set++, appearance++)
    if (*appearance == old_appearance) {
      object_point_sets.erase(point_set);
      object_top_sets.erase(top_set);
      object_appearances.erase(appearance);
      removed_object = true;
      point_set--; top_set--; appearance--; 
    }
  if (refresh) update();
  return removed_object;
}

bool QGLCanvas::RemoveObjectData(const DataType type, bool refresh)
{
  std::vector <ObjectPoints *>::iterator         point_set;
  std::vector <LineTopPtrVector>::iterator       top_set;
  std::vector <const DataAppearance *>::iterator appearance;
  bool removed_object=false;

  for (point_set  = object_point_sets.begin(),
       top_set    = object_top_sets.begin(),
       appearance = object_appearances.begin();
       point_set != object_point_sets.end();
       point_set++, top_set++, appearance++)
    if (type == (*appearance)->TypeOfData()) {
      object_point_sets.erase(point_set);
      object_top_sets.erase(top_set);
      object_appearances.erase(appearance);
      removed_object = true;
      point_set--; top_set--; appearance--; 
    }
  if (refresh) update();
  return removed_object;
}

void QGLCanvas::ClearObjectData(bool refresh)
{
  std::vector <LineTopPtrVector>::iterator top_set;

  if (object_point_sets.empty()) return;
  for (top_set = object_top_sets.begin();
       top_set!=object_top_sets.end(); top_set++)
    if (!top_set->empty()) top_set->Clear();
  object_point_sets.erase(object_point_sets.begin(), object_point_sets.end());
  object_top_sets.erase(object_top_sets.begin(), object_top_sets.end());
  object_appearances.erase(object_appearances.begin(),object_appearances.end());
  if (refresh) update();
}

bool QGLCanvas::ChangeObjectAppearance(ObjectPoints *object_points,
                                       LineTopologies *object_top,
                                       const DataAppearance *new_appearance,
                                       bool refresh)
{
  bool removed = RemoveObjectData(object_top, false);
  AddObjectData(object_points, object_top, new_appearance, refresh);
  return removed;
}

bool QGLCanvas::ChangeObjectAppearance(ObjectPoints *object_points,
                                       LineTopology *object_top,
                                       const DataAppearance *new_appearance,
                                       bool refresh)
{
  bool removed = RemoveObjectData(object_top, false);
  AddObjectData(object_points, object_top, new_appearance, refresh);
  return removed;
}

bool QGLCanvas::AddLaserData(LaserPoints *new_point_set,
                             const DataAppearance *new_appearance,
                             bool check_if_present, bool refresh)
{
  LineTopology *no_line=NULL;
  
  return AddLaserData(new_point_set, no_line, new_appearance,
                      check_if_present, refresh);
}

bool QGLCanvas::AddLaserData(LaserPoints *new_point_set,
                             LineTopologies *new_top_set,
                             const DataAppearance *new_appearance,
                             bool check_if_present,
                             bool refresh)
{
  bool check_result=false;
  
  for (LineTopologies::iterator polygon=new_top_set->begin();
       polygon!=new_top_set->end(); polygon++)
    check_result = AddLaserData(new_point_set, &*polygon, new_appearance,
                                check_if_present, false);
  if (refresh) update();
  return (!check_if_present || check_result);
}

bool QGLCanvas::AddLaserData(LaserPoints *new_point_set,
                             LineTopology *new_top,
                             const DataAppearance *new_appearance,
                             bool check_if_present, bool refresh)
{
  std::vector <LaserPoints *>::iterator          point_set;
  std::vector <LineTopPtrVector>::iterator       top_set, correct_top_set;
  std::vector <const DataAppearance *>::iterator appearance;
  LineTopPtrVector::iterator                     top;

  // Locate entry based on points and appearance pointers
  bool found=false;
  for (point_set  = laser_point_sets.begin(),
       top_set    = laser_top_sets.begin(),
       appearance = laser_appearances.begin();
       point_set != laser_point_sets.end() && !found;
       point_set++, top_set++, appearance++)
    if (*point_set == new_point_set && *appearance == new_appearance) {
      found = true;
      correct_top_set = top_set;
    }

  // If required, create a new entry
  if (!found) {
    laser_point_sets.push_back(new_point_set);
    laser_top_sets.push_back(LineTopPtrVector());
    correct_top_set = laser_top_sets.end() - 1;
    laser_appearances.push_back(new_appearance);
  }

  // Check on existance of points or polygon if required
  else if (check_if_present) {
    if (new_top == NULL) return false; // Point set was already there
                                       // and no polygon was supplied
    for (top=correct_top_set->begin(); top!=correct_top_set->end(); top++)
      if (*top == new_top) return false; // Polygon was already there
  }

  // Add the polygon if there is one
  if (new_top != NULL) correct_top_set->push_back(new_top);
  if (refresh) update();
  return true;
}

bool QGLCanvas::RemoveLaserData(const LaserPoints *old_point_set, bool refresh)
{
  std::vector <LaserPoints *>::iterator          point_set;
  std::vector <LineTopPtrVector>::iterator       top_set;
  std::vector <const DataAppearance *>::iterator appearance;

  for (point_set  = laser_point_sets.begin(),
       top_set    = laser_top_sets.begin(),
       appearance = laser_appearances.begin();
       point_set != laser_point_sets.end();
       point_set++, top_set++, appearance++)
    if (*point_set == old_point_set) {
      laser_point_sets.erase(point_set);
      laser_top_sets.erase(top_set);
      laser_appearances.erase(appearance);
      if (refresh) update();
      return true;
    }
  return false;
}

bool QGLCanvas::RemoveLaserData(LineTopology *old_top, bool refresh)
{
  std::vector <LaserPoints *>::iterator          point_set;
  std::vector <LineTopPtrVector>::iterator       top_set;
  std::vector <const DataAppearance *>::iterator appearance;
  LineTopPtrVector::iterator                     top;

  for (point_set  = laser_point_sets.begin(),
       top_set    = laser_top_sets.begin(),
       appearance = laser_appearances.begin();
       point_set != laser_point_sets.end();
       point_set++, top_set++, appearance++) {
    for (top=top_set->begin(); top!=top_set->end(); top++) {
      if (*top == old_top) {
        top_set->erase(top);
        if (top_set->empty()) {
          laser_point_sets.erase(point_set);
          laser_top_sets.erase(top_set);
          laser_appearances.erase(appearance);
        }
        if (refresh) update();
        return true;
      }
    }
  }
  return false;
}

bool QGLCanvas::RemoveLaserData(LineTopologies *old_top_set, bool refresh)
{
  bool removed_all=true;
  for (LineTopologies::iterator polygon=old_top_set->begin();
       polygon!=old_top_set->end(); polygon++)
    removed_all = removed_all && RemoveLaserData(&*polygon, false);
  if (refresh) update();
  return removed_all;
}

bool QGLCanvas::RemoveLaserData(const DataAppearance *old_appearance,
                                bool refresh)
{
  std::vector <LaserPoints *>::iterator          point_set;
  std::vector <LineTopPtrVector>::iterator       top_set;
  std::vector <const DataAppearance *>::iterator appearance;
  bool removed_laser=false;

  for (point_set  = laser_point_sets.begin(),
       top_set    = laser_top_sets.begin(),
       appearance = laser_appearances.begin();
       point_set != laser_point_sets.end();
       point_set++, top_set++, appearance++)
    if (*appearance == old_appearance) {
      laser_point_sets.erase(point_set);
      laser_top_sets.erase(top_set);
      laser_appearances.erase(appearance);
      removed_laser = true;
      point_set--; top_set--; appearance--; 
    }
  if (refresh) update();
  return removed_laser;
}

void QGLCanvas::ClearLaserData(bool refresh)
{
  if (laser_point_sets.empty()) return;
  laser_point_sets.erase(laser_point_sets.begin(), laser_point_sets.end());
  laser_top_sets.erase(laser_top_sets.begin(), laser_top_sets.end());
  laser_appearances.erase(laser_appearances.begin(), laser_appearances.end());
  if (refresh) update();
}

void QGLCanvas::InitialiseData(bool refresh)
{
  ClearObjectData(false);
  ClearTextureData(false);
  ClearLaserData(refresh);
}

void QGLCanvas::InitialiseTransformation(DataType focus_on_type)
{
  std::vector <ObjectPoints *>::const_iterator   object_point_set;
  std::vector <LineTopPtrVector>::const_iterator object_top_set;
  std::vector <const DataAppearance *>::iterator appearance;
  std::vector <LaserPoints *>::iterator          laser_point_set;
  LineTopPtrVector::const_iterator               object_top;
  std::vector <const DataBounds3D *>::iterator   texture_bound;

  rotation[0] = rotation[1] = 0.0;
  translation[0] = translation[1] = 0.0;
  zoom_factor = 1.0;
  perspective = false;
  if (HasData()) {
    DataBounds3D *bounds = new DataBounds3D();
    // Loop over all object sets
    for (object_point_set = object_point_sets.begin(),
         object_top_set = object_top_sets.begin(),
         appearance = object_appearances.begin();
         object_point_set != object_point_sets.end();
         object_point_set++, object_top_set++, appearance++) {
      // Loop over all polygons of this object set
      if (focus_on_type == NumDataTypes ||
          focus_on_type == (*appearance)->TypeOfData()) {
        if ((*appearance)->TypeOfData() == LastModelData ||
            (*appearance)->TypeOfData() == SelectedModelFaceData) {
          bounds->Update((*object_point_set)->ObjectPointsRef());
        }
        else {
          for (object_top=object_top_set->begin();
               object_top!=object_top_set->end(); object_top++)
            if (*object_top)
              bounds->Update((*object_top)->Bounds(
                (*object_point_set)->ObjectPointsRef()));
        }
      }
    }
    // Loop over all laser point sets
    for (laser_point_set = laser_point_sets.begin(),
         appearance = laser_appearances.begin();
         laser_point_set != laser_point_sets.end();
         laser_point_set++, appearance++) 
      if ((focus_on_type == NumDataTypes ||
           focus_on_type == (*appearance)->TypeOfData()) &&
          !(*laser_point_set)->empty()) {
        bounds->Update((*laser_point_set)->DeriveDataBounds(1).Bounds3D());
/*
        printf("Used bounds in InitialiseTransformation\n");
        (*laser_point_set)->DataBounds().Print();
        printf("Bounds are now:\n");
        printf("Minimum %.2f, %.2f, %.2f\n", bounds->Minimum().X(),
               bounds->Minimum().Y(), bounds->Minimum().Z());
        printf("Maximum %.2f, %.2f, %.2f\n", bounds->Maximum().X(),
               bounds->Maximum().Y(), bounds->Maximum().Z());
*/
      }

    // Loop over all textures
    if (focus_on_type == NumDataTypes)
      for (texture_bound = texture_bounds.begin();
           texture_bound != texture_bounds.end(); texture_bound++)
        bounds->Update(**texture_bound);

    // Determine maximum data range
    data_range = fmax(fmax(bounds->XRange(), bounds->YRange()),
                      bounds->ZRange()) * 1.1;
    
    offset = (bounds->Minimum().vect() + bounds->Maximum().vect()) / 2.0;
    if (data_range > 0.0) {
      pixel_size = data_range / min(width(), height());
      eye_distance = 0.5 * data_range;
    }
    else 
      data_range = pixel_size = eye_distance = 1.0;
    delete bounds;
  }
  else {
    data_range = pixel_size = eye_distance = 1.0;
    offset = Vector3D(); // Null vector
  }
  rot = Rotation3D(); // Unit matrix
}

void QGLCanvas::InitialiseColourLUT()
{
  double r, g, b;
  int i;
  
  srand(123456);
  for (i=0; i<216; i++) {
    r = rand() / (double) RAND_MAX;
    g = rand() / (double) RAND_MAX;
    b = rand() / (double) RAND_MAX;
    // Demand differences between r, g, and b values to avoid grey colours
    // This also avoids very dark or bright colours
    if (fabs(r-g) > 0.2 || fabs(r-b) > 0.4 || fabs(g-b) > 0.4) {
      random_colours[i][0] = 0.1 + 0.9 * r;
      random_colours[i][1] = 0.1 + 0.9 * g;
      random_colours[i][2] = 0.1 + 0.9 * b;
    }
    else {
      i--;
    }
  }
}

void QGLCanvas::InitialiseFont()
{
  char *libdir, *fontfile;
  bool silent = true;
  
  glGenTextures(1, &text_texture_id);
  libdir = getenv("MAPPING_LIB_DIR");
  if (libdir == NULL) {
  	if (silent) return;
  	QMessageBox::information(this, "Error", "MAPPING_LIB_DIR is not defined. Tile numbers will not be shown.");
  	return;
  }
  fontfile = (char *) malloc(strlen(libdir) + 60);
  sprintf(fontfile, "%s/../../Tools/building_modelling/pcm/fonts/Arial12.glf", libdir);
  try {
    text_font.Create(fontfile, text_texture_id);
  }
  catch(GLFontError::InvalidFile) {
  	QMessageBox::information(this, "Error",
                             QString("Cannot read font file ") +
                             QString(fontfile) +
							 QString(". Tile numbers will not be shown."));

  }
  free(fontfile);
}

Position3D QGLCanvas::Canvas2World(const QPoint &canvas_pos,
                                   int canvas_Z, bool apply_offset) const
{
  Position3D world_pos;

  world_pos = rot * Vector3D((canvas_pos.x() - (double) width() / 2.0) *
                             pixel_size / zoom_factor - translation[0],
                             ((double) height() / 2.0 - canvas_pos.y()) *
                             pixel_size / zoom_factor - translation[1],
                             canvas_Z * pixel_size / zoom_factor);
  if (apply_offset) return world_pos + offset;
  return world_pos;
}

Position3D QGLCanvas::Canvas2World(const QPoint &canvas_pos,
                                   double world_height, bool apply_offset) const
{
  Position3D world_pos = Canvas2World(canvas_pos, 0, apply_offset);
  Vector3D   view_dir  = rot * Vector3D(0.0, 0.0, 1.0);

  if (view_dir.Z() != 0.0) {
    view_dir *= (world_pos.Z() - world_height) / view_dir.Z();
    world_pos = world_pos - view_dir;
  }
  return world_pos;
}


QPoint QGLCanvas::World2Canvas(const Position3D &world_pos,
                               bool apply_offset) const
{
  Position3D canvas_pos_3D;
  QPoint     canvas_pos;
  
  if (apply_offset)
    canvas_pos_3D = (rot.Transpose() * (world_pos - offset)) +
                    Vector3D(translation[0], translation[1], 0.0);
  else
    canvas_pos_3D = (rot.Transpose() * world_pos) +
                    Vector3D(translation[0], translation[1], 0.0);
  canvas_pos_3D *= zoom_factor / pixel_size;
  canvas_pos.setX((int) (canvas_pos_3D.X() + width() / 2.0));
  canvas_pos.setY((int) (-canvas_pos_3D.Y() + height() / 2.0));
  return canvas_pos;
}

DataBounds2D & QGLCanvas::BoundsXY() const
{
  DataBounds2D *bounds = new DataBounds2D();
  Position3D   canvas_corner;
  
  canvas_corner = Canvas2World(QPoint(0, 0), 0.0, true);
  bounds->Update(Position2D(canvas_corner.Position2DOnly()));
  canvas_corner = Canvas2World(QPoint(0, height()), 0.0, true);
  bounds->Update(Position2D(canvas_corner.Position2DOnly()));
  canvas_corner = Canvas2World(QPoint(width(), height()), 0.0, true);
  bounds->Update(Position2D(canvas_corner.Position2DOnly()));
  canvas_corner = Canvas2World(QPoint(width(), 0), 0.0, true);
  bounds->Update(Position2D(canvas_corner.Position2DOnly()));
  return *bounds;
}

void QGLCanvas::CopyData(const QGLCanvas &canvas)
{
  std::vector <const Image *>::const_iterator        image;
  std::vector <const DataBounds3D *>::const_iterator bounds;

  ClearObjectData(false);
  object_point_sets.insert(object_point_sets.begin(),
                           canvas.object_point_sets.begin(),
                           canvas.object_point_sets.end());
  object_top_sets.insert(object_top_sets.begin(),
                         canvas.object_top_sets.begin(),
                         canvas.object_top_sets.end());
  object_appearances.insert(object_appearances.begin(),
                            canvas.object_appearances.begin(),
                            canvas.object_appearances.end());
  ClearLaserData(false);
  laser_point_sets.insert(laser_point_sets.begin(),
                          canvas.laser_point_sets.begin(),
                          canvas.laser_point_sets.end());
  laser_appearances.insert(laser_appearances.begin(),
                           canvas.laser_appearances.begin(),
                           canvas.laser_appearances.end());
  ClearTextureData(false);
  for (image=canvas.texture_images.begin(),
       bounds=canvas.texture_bounds.begin();
       image!=canvas.texture_images.end(); image++, bounds++)
    AddTextureData(*image, *bounds, false);
  update();
}

void QGLCanvas::CopyTransformation(const QGLCanvas &canvas)
{
  int i;
  for (i=0; i<3; i++) rotation[i] = canvas.rotation[i];
  for (i=0; i<3; i++) translation[i] = canvas.translation[i];
  zoom_factor = canvas.zoom_factor;
  data_range = canvas.data_range;
  pixel_size = canvas.pixel_size;
  offset = canvas.offset;
  rot = canvas.rot;
  eye_distance = canvas.eye_distance;
  perspective = canvas.perspective;
}

bool QGLCanvas::HasData() const
{
  for (int type=0; type<NumDataTypes; type++)
    if (HasData((DataType) type)) return true;
  if (HasData(TextureData)) return true;
  return false;
}

bool QGLCanvas::HasData(DataType type) const
{
  std::vector <ObjectPoints *>::const_iterator         object_point_set;
  std::vector <LineTopPtrVector>::const_iterator       object_top_set;
  std::vector <const DataAppearance *>::const_iterator object_appearance;
  std::vector <LaserPoints *>::const_iterator          laser_point_set;

  if (type == LaserData) {
    for (laser_point_set = laser_point_sets.begin();
         laser_point_set != laser_point_sets.end(); laser_point_set++)
      if ((*laser_point_set)->size() > 0) return true;
    return false;
  }

  else if (type == TextureData) {
    if (!texture_ids.empty()) return true;
  }
  
  for (object_point_set = object_point_sets.begin(),
       object_top_set = object_top_sets.begin(),
       object_appearance = object_appearances.begin();
       object_point_set != object_point_sets.end();
       object_point_set++, object_top_set++, object_appearance++)
    if ((*object_appearance)->TypeOfData() == type)
      if (*object_point_set) {
        if (type == LastModelData || type == SelectedModelFaceData) {
          if (!(*object_point_set)->empty()) return true;
        }
        else {
          if ((*object_point_set)->size() > 1 &&
              object_top_set->size() > 0) return true;
        }
      }
  return false;
}

bool QGLCanvas::ReadView(char *viewfile)
{
  FILE *fd;

  if ((fd = fopen(viewfile, "r")) == NULL) return false;
  fread(rotation, sizeof(GLfloat), 3, fd);
  fread(translation, sizeof(GLfloat), 3, fd);
  fread(&zoom_factor, sizeof(GLfloat), 1, fd);
  fread(&data_range, sizeof(double), 1, fd);
  fread(&pixel_size, sizeof(double), 1, fd);
  fread(&offset, sizeof(Vector3D), 1, fd);
  fread(&rot, sizeof(Rotation3D), 1, fd);
  fread(&eye_distance, sizeof(GLfloat), 1, fd);
  fread(&perspective, sizeof(bool), 1, fd);
  
  fclose(fd);
  update();
  return true;
}

bool QGLCanvas::SaveView(char *viewfile) const
{
  FILE *fd;

  if ((fd = fopen(viewfile, "wb")) == NULL) return false;
  fwrite((const void *) rotation, sizeof(GLfloat), 3, fd);
  fwrite((const void *) translation, sizeof(GLfloat), 3, fd);
  fwrite((const void *) &zoom_factor, sizeof(GLfloat), 1, fd);
  fwrite((const void *) &data_range, sizeof(double), 1, fd);
  fwrite((const void *) &pixel_size, sizeof(double), 1, fd);
  fwrite((const void *) &offset, sizeof(Vector3D), 1, fd);
  fwrite((const void *) &rot, sizeof(Rotation3D), 1, fd);
  fwrite((const void *) &eye_distance, sizeof(GLfloat), 1, fd);
  fwrite((const void *) &perspective, sizeof(bool), 1, fd);
  fclose(fd);
  return true;
}

void QGLCanvas::LevelView()
{
  double pi = 4.0 * atan(1.0);

  rotation[0] = 0.0;
  rot = Rotation3D(0.0, 0.0, -rotation[1]*pi/180.0);
  update();
}

void QGLCanvas::HorizontalView()
{
  double pi = 4.0 * atan(1.0);
  rotation[0] = -90.0;
  rot = Rotation3D(0.0, 0.0, -rotation[1]*pi/180.0) *
        Rotation3D(-rotation[0]*pi/180.0, 0.0, 0.0);
  update();
}

void QGLCanvas::FitViewToData(DataType focus_on_type)
{
  InitialiseTransformation(focus_on_type);
  update();
}

void QGLCanvas::SetDataOffsetInCanvas(double X, double Y)
{
  offset.X() = X;
  offset.Y() = Y;
}

void QGLCanvas::TranslateCanvas(float x, float y)
{
  translation[0] += x;
  translation[1] += y;
}

void QGLCanvas::TranslateCanvas(float width_perc, float height_perc,
                                bool refresh)
{
  Position3D canvas_centre;

  translation[0] += width_perc * BoundsXY().XRange();
  translation[1] += height_perc * BoundsXY().YRange();
  canvas_centre = Canvas2World(QPoint(width()/2, height()/2));
  emit CentreOfCanvasChanged(canvas_centre.X(), canvas_centre.Y());
  if (refresh) update();
}

void QGLCanvas::ScaleCanvas(float scale)
{
  zoom_factor *= scale;
}

void QGLCanvas::ResizeFocalLengthCanvas(float scale)
{
  eye_distance *= scale;
}

void QGLCanvas::ScaleCanvas(float scale, bool refresh)
{
  ScaleCanvas(scale);
  emit CanvasScaled(scale);
  if (refresh) update();
}

void QGLCanvas::ResizeFocalLengthCanvas(float scale, bool refresh)
{
  ResizeFocalLengthCanvas(scale);
  emit CanvasScaled(scale);
  if (refresh) update();
}

ObjectPoint *QGLCanvas::NearestObjectPoint(QMouseEvent *event,
                                           DataType type,
                                           DataType selection_type)
{
  std::vector <ObjectPoints *>::iterator               object_point_set;
  std::vector <const DataAppearance *>::const_iterator object_appearance;
  ObjectPoints           *object_points=NULL;
  bool                   found;
  int                    dist, min_dist=1000000;
  ObjectPoints::iterator object_point, nearest_point;
  QPoint                 canvas_pos;

  // Check if there is data of the correct type
  if (!HasData(type) && !HasData(selection_type)) {
    QMessageBox::information(this, "Error",
        "There is no data of the requested type in the canvas!");
    return NULL;
  }

  // Find the object points of the correct type
  for (object_point_set = object_point_sets.begin(),
       object_appearance = object_appearances.begin(), found = false;
       object_point_set != object_point_sets.end() && !found;
       object_point_set++, object_appearance++) {
    if ((*object_appearance)->TypeOfData() == type ||
        (*object_appearance)->TypeOfData() == selection_type) {
      found = true;
      object_points = *object_point_set;
    }
  }

  // Locate the nearest point
  for (object_point=object_points->begin(); object_point!=object_points->end();
       object_point++) {
    canvas_pos = World2Canvas(object_point->Position3DRef());
    dist = abs(canvas_pos.x() - event->x()) +   // Manhattan metric
           abs(canvas_pos.y() - event->y());
    if (dist < min_dist) {
      min_dist = dist;
      nearest_point = object_point;
    }
  }

  return &*nearest_point;
}

bool QGLCanvas::NearestLaserPoint(QMouseEvent *event, 
                                  LaserPoints::const_iterator &nearest_point,
                                  DataType &type)
{
  std::vector <LaserPoints *>::iterator          point_set;
  std::vector <const DataAppearance *>::iterator appearance;
  bool                                           found;
  int                                            dist, min_dist=1000000;
  LaserPoints::const_iterator                    laser_point;
  QPoint                                         canvas_pos;

  // Check if there is laser data
  found = false;
  for (point_set=laser_point_sets.begin();
       point_set!=laser_point_sets.end() && !found; point_set++) {
    if (!(*point_set)->empty()) found = true;
  }
  if (!found) {
    QMessageBox::information(this, "Error",
        "There is no laser data in the canvas!");
    return false;
  }

  // Find the nearest laser point
  for (point_set=laser_point_sets.begin(), appearance=laser_appearances.begin();
       point_set!=laser_point_sets.end(); point_set++, appearance++) {
    for (laser_point=(*point_set)->begin(); laser_point!=(*point_set)->end();
         laser_point++) {
      canvas_pos = World2Canvas(laser_point->Position3DRef());
      dist = abs(canvas_pos.x() - event->x()) +   // Manhattan metric
             abs(canvas_pos.y() - event->y());
      if (dist < min_dist) {
        min_dist = dist;
        nearest_point = laser_point;
        type = (*appearance)->TypeOfData();
      }
    }
  }
  return true;
}

void QGLCanvas::SelectLaserPointsWithinRadius(LaserPoints &points,
                                              QMouseEvent *event,
											  double radius)
{
  std::vector <LaserPoints *>::iterator          point_set, largest_point_set;
  double                                         dx, dy, dist;
  LaserPoints::const_iterator                    laser_point;
  QPoint                                         canvas_pos;

  // Check if there is laser data
  for (point_set=laser_point_sets.begin(), largest_point_set=laser_point_sets.begin();
       point_set!=laser_point_sets.end(); point_set++) {
    if ((*point_set)->size() > (*largest_point_set)->size()) 
	  largest_point_set = point_set;
  }
  if ((*largest_point_set)->empty()) {
    QMessageBox::information(this, "Error",
        "There is no laser data in the canvas!");
    return;
  }

  // Find all points within the radius
  for (laser_point=(*largest_point_set)->begin();
       laser_point!=(*largest_point_set)->end();
       laser_point++) {
    canvas_pos = World2Canvas(laser_point->Position3DRef());
    dx = canvas_pos.x() - event->x();
	dy = canvas_pos.y() - event->y();
	dist = sqrt(dx * dx + dy * dy);
    if (dist < radius) points.push_back(*laser_point);
  }
}

void QGLCanvas::AddTextureData(const Image *image, const DataBounds3D *bounds,
                               bool refresh)
{
  GLuint texture_id;

  // Generate the texture and set its properties
  glGenTextures(1, &texture_id);
  glBindTexture(GL_TEXTURE_2D, texture_id);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image->NumColumns(), image->NumRows(),
               0, GL_RGB, GL_UNSIGNED_BYTE, image->GetImage()->imagedata);

  // Store the data
  texture_ids.push_back(texture_id);
  texture_images.push_back(image);
  texture_bounds.push_back(bounds);
  if (refresh) update();
}

void QGLCanvas::RemoveTextureData(const Image *image,
                                  const DataBounds3D *bounds, bool refresh)
{
  std::vector <GLuint>::iterator texture_id;
  std::vector <const Image *>::iterator texture_image;
  std::vector <const DataBounds3D *>::iterator texture_bound;

  for (texture_id = texture_ids.begin(),
       texture_image = texture_images.begin(),
       texture_bound = texture_bounds.begin();
       texture_id != texture_ids.end();
       texture_id++, texture_image++, texture_bound++) {
    if (*texture_image == image && *texture_bound == bounds) {
      glDeleteTextures(1, &*texture_id);
      texture_ids.erase(texture_id);
      texture_images.erase(texture_image);
      texture_bounds.erase(texture_bound);
      if (refresh) update();
      return;
    }
  }
}

void QGLCanvas::ClearTextureData(bool refresh)
{
  std::vector <GLuint>::iterator texture_id;

  for (texture_id = texture_ids.begin(); texture_id != texture_ids.end();
       texture_id++)
    glDeleteTextures(1, &*texture_id);
  texture_ids.erase(texture_ids.begin(), texture_ids.end());
  texture_images.erase(texture_images.begin(), texture_images.end());
  texture_bounds.erase(texture_bounds.begin(), texture_bounds.end());
  if (refresh) update();
}

void QGLCanvas::PaintTextureData(GLuint texture_id, const DataBounds3D *bounds)
{
  glEnable(GL_TEXTURE_2D);
  glColor3f(1.0, 1.0, 1.0);
  glBindTexture(GL_TEXTURE_2D, texture_id);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_QUADS);
  glTexCoord2f(0.0, 0.0);
  glVertex3f(bounds->Minimum().X() - offset.X(),
             bounds->Minimum().Y() - offset.Y(),
             bounds->Minimum().Z() - offset.Z());
  glTexCoord2f(1.0, 0.0);
  glVertex3f(bounds->Maximum().X() - offset.X(),
             bounds->Minimum().Y() - offset.Y(),
             bounds->Minimum().Z() - offset.Z());
  glTexCoord2f(1.0, 1.0);
  glVertex3f(bounds->Maximum().X() - offset.X(),
             bounds->Maximum().Y() - offset.Y(),
             bounds->Minimum().Z() - offset.Z());
  glTexCoord2f(0.0, 1.0);
  glVertex3f(bounds->Minimum().X() - offset.X(),
             bounds->Maximum().Y() - offset.Y(),
             bounds->Minimum().Z() - offset.Z());
  glEnd();
  glDisable(GL_TEXTURE_2D);
}

LaserPoints & QGLCanvas::LaserPointsWithDisplayColour(LaserPoints *laser_points,
                                              const DataAppearance *appearance) const
{
  LaserPoints::const_iterator laser_point, laser_point2, laser_point3;
  GLfloat                     intensity_scale, intensity_offset, intensity,
                              height_offset, height_scale,
                              pulselength_offset, pulselength_scale; 
  int                         red, green, blue, colour_index, value;
  float                       residual,stddev;
  LaserPoints                 *points;
  LaserPoint                  point;
  long long                   lvalue;
  QColor                      colour;
  double                      dvalue;

  points = new LaserPoints();
  if (!laser_points) return *points;

  LaserPointType  type = laser_points->Scanner().PointType();
  DataBoundsLaser bounds = laser_points->DataBounds();
  intensity_scale  = 0.75 / (float) bounds.ReflectanceRange();
  intensity_offset = 0.25 - 0.75 * (float) bounds.Minimum().Reflectance() /
                     (float) bounds.ReflectanceRange();
  height_scale  = 0.75 / (float) bounds.ZRange();
  height_offset = 0.25 - 0.75 * (float) bounds.Minimum().Z() /
                  (float) bounds.ZRange();
  pulselength_scale  = 0.75 / (float) (bounds.Maximum().PulseLength() -
                                       bounds.Minimum().PulseLength());
  pulselength_offset = 0.25 - (float) bounds.Minimum().PulseLength() *
                       pulselength_scale;

  for (laser_point=laser_points->begin(); laser_point!=laser_points->end();
       laser_point++) {
    if (laser_point->IsPulseType(appearance->SelectedPulseType())) {
      point.vect() = laser_point->vect();
      switch (appearance->PointColourMethod()) {
             
        default:
        case FixedColour:
          point.SetColour(appearance->PointColour().red(),
                          appearance->PointColour().green(),
                          appearance->PointColour().blue());
          break;
  
        case ColourByLabel:
          if (laser_point->HasLabel()) {
            colour_index = laser_point->Label() - (laser_point->Label()/7) * 7;
            point.SetColour((int) (label_colours[colour_index][0]*255.0),
                            (int) (label_colours[colour_index][1]*255.0),
                            (int) (label_colours[colour_index][2]*255.0));
          }
          else // Unlabeled points in white
            point.SetColour(255, 255, 255);
          break;
  
        case ColourByReflectance:
        case ColourByColour:
          if (type == ColourPoint || type == MultiColourPoint ||
              appearance->PointColourMethod() == ColourByColour)
            point.SetColour(laser_point->Red(), laser_point->Green(),
                            laser_point->Blue());    
          else {
            intensity = laser_point->Reflectance() * intensity_scale +
                        intensity_offset;
            intensity *= 255.0;
            point.SetColour((int) intensity, (int) intensity, (int) intensity);
          }
          break;
          
        case ColourByHeight:
          dvalue = laser_point->Z();
        case ColourByTime:
          if (appearance->PointColourMethod() == ColourByTime)
          	 dvalue = laser_point->DoubleAttribute(TimeTag) / 60;
          ColourAttribute(dvalue,
                          appearance->PointColourCycleLength(),
                          appearance->PointColourCyclePhase(),
                          red, green, blue);
          point.SetColour(red, green, blue);
          break;

        case GreyToneByHeight:
          intensity = laser_point->Z() * height_scale + height_offset;
          intensity *= 255.0;
          point.SetColour((int) intensity, (int) intensity, (int) intensity);
          break;

        case ColourByResidual:
          residual = laser_point->Residual();
          if (fabs(residual) < 0.2)
            point.SetColour((int) (label_colours[ColourGreen][0]*255.0),
                            (int) (label_colours[ColourGreen][1]*255.0),
                            (int) (label_colours[ColourGreen][2]*255.0));
          else if (fabs(residual) < 0.5)
            point.SetColour((int) (label_colours[ColourYellow][0]*255.0),
                            (int) (label_colours[ColourYellow][1]*255.0),
                            (int) (label_colours[ColourYellow][2]*255.0));
          else
            point.SetColour((int) (label_colours[ColourRed][0]*255.0),
                            (int) (label_colours[ColourRed][1]*255.0),
                            (int) (label_colours[ColourRed][2]*255.0));          
          break;

        case ColourBySegment:
          if (laser_point->HasAttribute(SegmentNumberTag)) {
            value = laser_point->Attribute(SegmentNumberTag);
            colour_index = value - (value/appearance->NumberOfColours()) * appearance->NumberOfColours();
            point.SetColour((int) (random_colours[colour_index][0]*255.0),
                            (int) (random_colours[colour_index][1]*255.0),
                            (int) (random_colours[colour_index][2]*255.0));
          }
          else // Unlabeled points in white
            point.SetColour(255, 255, 255);
          break;
  
        case ColourBySegmentAndTile:
          if (laser_point->HasAttribute(SegmentNumberTag) &&
		      laser_point->HasAttribute(SegmentStartTileNumberTag)) {
            lvalue = (long long) laser_point->Attribute(SegmentStartTileNumberTag) *
			         1000000 +
			         (long long) laser_point->Attribute(SegmentNumberTag);
            colour_index = (int) (lvalue - (lvalue/appearance->NumberOfColours()) * appearance->NumberOfColours());
            point.SetColour((int) (random_colours[colour_index][0]*255.0),
                            (int) (random_colours[colour_index][1]*255.0),
                            (int) (random_colours[colour_index][2]*255.0));
          }
          else // Unlabeled points in white
            point.SetColour(255, 255, 255);
          break;
  
        case ColourByComponent:
          if (laser_point->HasAttribute(ComponentNumberTag)) {
            value = laser_point->Attribute(ComponentNumberTag);
            colour_index = value - (value/appearance->NumberOfColours()) * appearance->NumberOfColours();
            point.SetColour((int) (random_colours[colour_index][0]*255.0),
                            (int) (random_colours[colour_index][1]*255.0),
                            (int) (random_colours[colour_index][2]*255.0));
          }
          else // Unlabeled points in white
            point.SetColour(255, 255, 255);
          break;
  
        case ColourByPlane:
          if (laser_point->HasAttribute(PlaneNumberTag)) {
            value = laser_point->Attribute(PlaneNumberTag);
            colour_index = value - (value/appearance->NumberOfColours()) * appearance->NumberOfColours();
            point.SetColour((int) (random_colours[colour_index][0]*255.0),
                            (int) (random_colours[colour_index][1]*255.0),
                            (int) (random_colours[colour_index][2]*255.0));
          }
          else // Unlabeled points in white
            point.SetColour(255, 255, 255);
          break;

        case GreyToneByPulseLength:
          intensity = laser_point->PulseLength() * pulselength_scale + 
                      pulselength_offset;
          intensity *= 255.0;
          point.SetColour((int) intensity, (int) intensity, (int) intensity);
          break;

        case ColourByScanNumber:
        case ColourByScanNumberWithoutAFN:
        case ColourByAFNCode:
          if (laser_point->HasAttribute(ScanNumberTag)) {
            value = laser_point->Attribute(ScanNumberTag);
            switch (appearance->PointColourMethod()) {
              case ColourByScanNumberWithoutAFN: value /= 10; break;
              case ColourByAFNCode: value -= 10 * (value/10); break;
              default: break;
            }
            colour_index = value - (value/appearance->NumberOfColours()) * appearance->NumberOfColours();
            point.SetColour((int) (random_colours[colour_index][0]*255.0),
                            (int) (random_colours[colour_index][1]*255.0),
                            (int) (random_colours[colour_index][2]*255.0));
          }
          else // Unlabeled points in white
            point.SetColour(255, 255, 255);
          break;

        case ColourByPulseCount:
          if (laser_point->HasAttribute(PulseCountTag)) {
            value = laser_point->PulseCount();
            colour_index = max(value - (value/9) * 9 - 1, 0);
            point.SetColour((int) (label_colours[colour_index][0]*255.0),
                            (int) (label_colours[colour_index][1]*255.0),
                            (int) (label_colours[colour_index][2]*255.0));
          }
          else // Unlabeled points in white
            point.SetColour(255, 255, 255);
          break;
  
        case ColourByFilterStatus:
          if (laser_point->HasAttribute(IsFilteredTag)) {
            value = laser_point->Filtered();
            if (value == 0) // Terrain
              point.SetColour((int) (label_colours[ColourGreen][0]*255.0),
                              (int) (label_colours[ColourGreen][1]*255.0),
                              (int) (label_colours[ColourGreen][2]*255.0));
            else // Non-terrain
              point.SetColour((int) (label_colours[ColourRed][0]*255.0),
                              (int) (label_colours[ColourRed][1]*255.0),
                              (int) (label_colours[ColourRed][2]*255.0));
          }
          else // Unlabeled points in white
            point.SetColour(255, 255, 255);
          break;
          
        case ColourByScanLineNumber:
          if (laser_point->HasAttribute(ScanLineNumberTag)) {
            value = laser_point->Attribute(ScanLineNumberTag);
            colour_index = value - (value/appearance->NumberOfColours()) * appearance->NumberOfColours();
            point.SetColour((int) (random_colours[colour_index][0]*255.0),
                            (int) (random_colours[colour_index][1]*255.0),
                            (int) (random_colours[colour_index][2]*255.0));
          }
          else // Unlabeled points in white
            point.SetColour(255, 255, 255);
          break;
  
        case ColourByThresholding:
          colour = ColourBasedOnThresholds(laser_point, appearance);
          point.SetColour(colour.red(), colour.green(), colour.blue());
          break;

          
          
          
	case ColourByStdDev_X:
            if (laser_point->HasAttribute(v_xTag)) {
		stddev = sqrt(laser_point->FloatAttribute(v_xTag));
                if (stddev > appearance->PointColourCycleLength()) break;
                 ColourAttribute_error(stddev, appearance->PointColourCycleLength(),
                    red, green, blue);
                //ColourAttribute(variance,
                //            appearance->PointColourCycleLength(),
                //            appearance->PointColourCyclePhase(),
                //            red, green, blue);
            point.SetColour(red, green, blue);
	 }
 	else // Unlabeled points in white
              point.SetColour(255, 255, 255);
          break;

  	case ColourByStdDev_Y:
            if (laser_point->HasAttribute(v_yTag)) {
		stddev = sqrt(laser_point->FloatAttribute(v_yTag));
                if (stddev > appearance->PointColourCycleLength()) break;
                 ColourAttribute_error(stddev, appearance->PointColourCycleLength(),
                    red, green, blue);
                //ColourAttribute(variance,
                //            appearance->PointColourCycleLength(),
                //            appearance->PointColourCyclePhase(),
                //            red, green, blue);
            point.SetColour(red, green, blue);
	 }
 	else // Unlabeled points in white
              point.SetColour(255, 255, 255);
          break;
  
  	case ColourByStdDev_Z:
            if (laser_point->HasAttribute(v_zTag)) {
		stddev = sqrt(laser_point->FloatAttribute(v_zTag));
		if (stddev > appearance->PointColourCycleLength()) break;
                 ColourAttribute_error(stddev, appearance->PointColourCycleLength(),
                    red, green, blue);
                //ColourAttribute(variance,
                //            appearance->PointColourCycleLength(),
                //            appearance->PointColourCyclePhase(),
                //            red, green, blue);
            point.SetColour(red, green, blue);
	 }
 	else // Unlabeled points in white
              point.SetColour(255, 255, 255);
          break;
      }
      points->push_back(point);        
    }
  }
  return *points;
}
