
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


#include "QGLCanvas.h"
#include <QMessageBox>
#include <QKeyEvent>
#include <QMouseEvent>
#include <stdlib.h>
#include <stdio.h>
 #include <iostream>
#include <QDir>
#ifndef CALLBACK 
#define CALLBACK
#endif
#include <GL/glut.h>
#include <QGLWidget>
#include "Positions3D.h"

#include "ExteriorOrientation.h"


// Normal colours
static GLfloat label_colours[9][3] = { 1.0, 0.3, 0.3,   // Red
                                       0.2, 0.9, 0.2,   // Green
                                       0.0, 0.5, 1.0,   // Blue
                                       0.8, 0.8, 0.0,   // Yellow
                                       1.0, 0.5, 0.0,   // Orange
                                       1.0, 1.0, 0.8,   // Light yellow
                                       0.0, 1.0, 1.0,   // Sea
                                       1.0, 1.0, 1.0,   // White
                                       0.0, 0.0, 0.0};  // Black
        
// Bright colours
static GLfloat highlight_colours[9][3] = { 1.0, 0.5, 0.5,   // Red
                                           0.4, 1.0, 0.4,   // Green
                                           0.2, 0.7, 1.0,   // Blue
                                           1.0, 1.0, 0.0,   // Yellow
                                           1.0, 0.7, 0.2,   // Orange
                                           1.0, 1.0, 0.9,   // Light yellow
                                           0.3, 1.0, 1.0,   // Sea
                                           1.0, 1.0, 1.0,   // White
                                           0.5, 0.5, 0.5};  // Black

// Arbitrary colours to display segments
static GLfloat random_colours[216][3];



void CALLBACK beginCallback(GLenum which)
{
   
   glBegin(which);
  
}

void CALLBACK errorCallback(GLenum errorCode)
{
   const GLubyte *estring;

   estring = gluErrorString(errorCode);
   printf("Tessellation Error: %s\n", estring);
   exit(0);
}

void CALLBACK endCallback(void)
{
   glEnd();
  
}

void CALLBACK vertexCallback (GLvoid *vertex) 

{ 
    const GLdouble *pointer;
     pointer=(GLdouble *)vertex;
    
  //  printf("  \n vertexCallback %f %f %f   %f %f\n", pointer[0], pointer[1], pointer[2], pointer[3], pointer[4]); 
    

    
    //cout<<vertex[0]<<" "<<vertex[1]<<" "<<vertex[2]<< " "<<vertex[3]<< " "<<vertex[4]<< " "<<tex_coord[0]<<" "<<tex_coord[1]<<endl;
    // printf("\n %f  %f   %f     %f   %f    \n",vertex[0],vertex[1],vertex[2],vertex[3],vertex[4]);
    glTexCoord2dv(pointer+3);
     glVertex3dv((GLdouble *)vertex);
    
    }

/*  combineCallback is used to create a new vertex when edges
 *  intersect.  coordinate location is trivial to calculate,
 *  but weight[4] may be used to average color, normal, or texture
 *  coordinate data.  In this program, color is weighted.
 */
void CALLBACK combineCallback(GLdouble coords[3], 
                     GLdouble *vertex_data[4],
                     GLfloat weight[4], GLdouble **dataOut )
{
   GLdouble *vertex;
   int i;

  // printf("  \n  %f %f %f   %f %f\n", coords[0], coords[1], coords[2], vertex_data[0][3], vertex_data[0][4]); 
   vertex = (GLdouble *) malloc(5 * sizeof(GLdouble));

   vertex[0] = coords[0];
   vertex[1] = coords[1];
   vertex[2] = coords[2];
   vertex[3] = vertex_data[0][3];
   vertex[4] = vertex_data[0][4];
   *dataOut = vertex;
}




void QGLCanvas::keyPressEvent(QKeyEvent *event)
{
  switch (event->key()) {
    case Qt::Key_Shift  : shift_down=1; setCursor(Qt::CrossCursor); break;
    case Qt::Key_Control: ctrl_down++; break;
    case Qt::Key_Alt    : alt_down++;  break;   
    default             : break;
  }
}

void QGLCanvas::keyReleaseEvent(QKeyEvent *event)
{
  switch (event->key()) {
    case Qt::Key_Shift  : shift_down=0; setCursor(Qt::ArrowCursor);break;
    case Qt::Key_Control: ctrl_down--; break;
    case Qt::Key_Alt:     alt_down--; break;
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
  float rot_speed = 0.2;
  double pi = 4.0 * atan(1.0);

  if (button_down[0]) {
    rotation[1] = rotation[1] + rot_speed * (event->x() - previous_pos.x());
    rotation[0] = rotation[0] + rot_speed * (event->y() - previous_pos.y());
    rot = Rotation3D(0.0, 0.0, -rotation[1]*pi/180.0) *
          Rotation3D(-rotation[0]*pi/180.0, 0.0, 0.0);
  }

  if (button_down[1]) {
    translation[0] += (float) (event->x() - previous_pos.x()) * data_range /
                              (width() * zoom_factor);
    translation[1] -= (float) (event->y() - previous_pos.y()) * data_range /
                              (height() * zoom_factor);
  }

  if (button_down[2]) {
    if (event->y() > previous_pos.y())
      zoom_factor *= 1.0 + (float) (event->y() - previous_pos.y()) / height();
    else
      zoom_factor /= 1.0 - (float) (event->y() - previous_pos.y()) / height();
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
  //GLfloat lightPosition[4] = {0, 0, 3, 0.0};

  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
  glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
//glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);

  glDisable(GL_LIGHT0);
  glDisable(GL_LIGHTING);
  
    glEnable(GL_CULL_FACE);
    glEnable(GL_TEXTURE_2D);
  QString str;
    MakeTextureData(str);  
    
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

void QGLCanvas::paintGL()
{
  glPushMatrix();
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  
  glOrtho(-pixel_size * width() / 2.0,  pixel_size * width() / 2.0,
          -pixel_size * height() / 2.0, pixel_size * height() / 2.0,
          -10.0 * data_range,         10.0 * data_range);
  glScalef(zoom_factor, zoom_factor, zoom_factor);
  glTranslatef(translation[0], translation[1], 0.0);
  glRotatef(rotation[0], 1.0, 0.0, 0.0);
  glRotatef(rotation[1], 0.0, 0.0, 1.0);
  glMatrixMode(GL_MODELVIEW);
  glClearColor((float) background_colour.red() / 255.0,
               (float) background_colour.green() / 255.0,
               (float) background_colour.blue() / 255.0, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  PaintAllData();
  glClearColor(0,
               0,
               0, 0.0);
  if(sharedObject)
  {
  qglClearColor(Qt::black);
  glEnable(GL_TEXTURE_2D);
  glCallList(sharedObject);
  glFlush();
  glDisable(GL_TEXTURE_2D);
  
  }
 
  glPopMatrix();
  
  
}

void QGLCanvas::PaintAllData()
{
  std::vector <ObjectPoints *>::const_iterator         object_point_set;
  std::vector <LineTopPtrVector>::const_iterator       object_top_set;
  std::vector <const DataAppearance *>::const_iterator appearance;
  std::vector <LaserPoints *>::iterator                laser_point_set;
  std::vector <GLuint>::const_iterator                 texture_id;
 
   

   
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
    {  
    PaintLaserData(*laser_point_set, *appearance);
}

  for (object_point_set = object_point_sets.begin(),
       object_top_set   = object_top_sets.begin(),
       appearance       = object_appearances.begin();
       object_point_set != object_point_sets.end();
       object_point_set++, object_top_set++, appearance++)
    if (!(*appearance)->PaintFirst())
      PaintObjectData(*object_point_set, object_top_set, *appearance);
      
}

void QGLCanvas::PaintObjectData(const ObjectPoints *points,
                             std::vector <LineTopPtrVector>::const_iterator top,
                                const DataAppearance *appearance)
{
  ObjectPoints::const_iterator                 point;
  std::vector <LineTopology *>::const_iterator polygon;
  LineTopology::const_iterator                 node;
  Vector3D                                     normal;
  Vector3D                                     light_dir;
  int                                          colour_index;

   if (!points) return;

  light_dir = Vector3D(-1.0, 1.0, 1.0).Normalize();

  if (appearance->ShowPoints()) {
    glColor3f(appearance->PointColour().red()   / 255.0,
              appearance->PointColour().green() / 255.0,
              appearance->PointColour().blue()  / 255.0);
    glPointSize((GLfloat) appearance->PointSize());
    glBegin(GL_POINTS);

    point=points->begin();
  
    for (point=points->begin(); point!=points->end(); point++)
      glVertex3f(point->X() - offset.X(), point->Y() - offset.Y(),
                 point->Z() - offset.Z());
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
 
      for (node=(*polygon)->begin(); node!=(*polygon)->end(); node++) {
        point = points->ConstPointIterator(node->Number());
      
        glVertex3f(point->X() - offset.X(), point->Y() - offset.Y(),
                   point->Z() - offset.Z());
                   
      }
  
      glEnd();
      if (appearance->FirstPointSize() && !(*polygon)->empty()) {
        point = points->ConstPointIterator((*polygon)->begin()->Number());
        glPointSize(appearance->FirstPointSize());
        glBegin(GL_POINTS);
        glVertex3f(point->X() - offset.X(), point->Y() - offset.Y(),
                   point->Z() - offset.Z());
        glEnd();
      }
    }
  }
  
  if (appearance->ShowFaces()) {
    glEnable(GL_LIGHTING);
    
    glEnable(GL_LIGHT0);
    //glClear(GL_COLOR_BUFFER_BIT);
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
        for (node=(*polygon)->begin(); node!=(*polygon)->end()-1; node++) {
                                          point = points->ConstPointIterator(node->Number());
                                          glVertex3f(point->X() - offset.X(), point->Y() - offset.Y(),
                                          point->Z() - offset.Z());  
                                          }
        glEnd();
        }
      }
    
    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT1);
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
}
                  

void QGLCanvas::PaintLaserData(LaserPoints *laser_points,
                               const DataAppearance *appearance)
{
  LaserPoints::const_iterator laser_point, laser_point2, laser_point3;
  GLfloat                     intensity_scale, intensity_offset, intensity,
                              height_offset, height_scale,
                              pulselength_offset, pulselength_scale; 
  int                         i, red, green, blue, colour_index, value;
  TIN::const_iterator         mesh;
  const PointNumber           *nodeptr;
  Plane                       plane;
  float                       residual;

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

  if (appearance->ShowPoints()) {
    if (appearance->PointColourMethod() == FixedColour)
      glColor3f(appearance->PointColour().red()   / 255.0,
                appearance->PointColour().green() / 255.0,
                appearance->PointColour().blue()  / 255.0);
    glPointSize((GLfloat) appearance->PointSize());
    glBegin(GL_POINTS);
    for (laser_point=laser_points->begin(); laser_point!=laser_points->end();
         laser_point++) {
      if (laser_point->IsPulseType(appearance->SelectedPulseType())) {
        switch (appearance->PointColourMethod()) {
  
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
          case ColourByColour:
            if (type == ColourPoint || type == MultiColourPoint ||
                appearance->PointColourMethod() == ColourByColour)
              glColor3f(laser_point->Red() / 255.0, laser_point->Green() / 255.0,
                        laser_point->Blue() / 255.0);
            else {
              intensity = laser_point->Reflectance() * intensity_scale +
                          intensity_offset;
              glColor3f(intensity, intensity, intensity);
            }
            break;
          
          case ColourByHeight:
            ColourAttribute(laser_point->Z(),
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
              colour_index = value - (value/216) * 216;
              glColor3fv(random_colours[colour_index]);
            }
            else // Unlabeled points in white
              glColor3f(1.0, 1.0, 1.0);
            break;
  
          case ColourByPlane:
            if (laser_point->HasAttribute(PlaneNumberTag)) {
              value = laser_point->Attribute(PlaneNumberTag);
              colour_index = value - (value/216) * 216;
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
          case ColourBy10Scans:
            if (laser_point->HasAttribute(ScanNumberTag)) {
              value = laser_point->Attribute(ScanNumberTag);
	      if (appearance->PointColourMethod() == ColourBy10Scans)
		value /= 10;
              colour_index = value - (value/216) * 216;
              glColor3fv(random_colours[colour_index]);
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
    if (!laser_points->GetTIN()) laser_points->DeriveTIN();
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor3f(appearance->LineColour().red()   / 255.0,
              appearance->LineColour().green() / 255.0,
              appearance->LineColour().blue()  / 255.0);
    glLineWidth((GLfloat) appearance->LineWidth());
    glBegin(GL_TRIANGLES);
    for (mesh=laser_points->TINReference().begin();
         mesh!=laser_points->TINReference().end(); mesh++) {
      for (i=0, nodeptr=mesh->Nodes(); i<3; i++, nodeptr++) {
        laser_point = laser_points->begin() + nodeptr->Number();
        glVertex3f((float) (laser_point->X() - offset.X()),
                   (float) (laser_point->Y() - offset.Y()),
                   (float) (laser_point->Z() - offset.Z()));
      }
    }
    glEnd();
  }

  if (appearance->ShowFaces()) {
    if (!laser_points->GetTIN()) laser_points->DeriveTIN();
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
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
      plane = Plane(laser_point->Position3DRef(),
                    laser_point2->Position3DRef(),
                    laser_point3->Position3DRef());
      glNormal3f(plane.Normal().X(), plane.Normal().Y(), plane.Normal().Z());
      for (i=0, nodeptr=mesh->Nodes(); i<3; i++, nodeptr++) {
        laser_point = laser_points->begin() + nodeptr->Number();
        glVertex3f((float) (laser_point->X() - offset.X()),
                   (float) (laser_point->Y() - offset.Y()),
                   (float) (laser_point->Z() - offset.Z()));
      }
      glEnd();
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

  for (LineTopologies::iterator polygon=new_top_set->begin();
       polygon!=new_top_set->end(); polygon++)
    AddObjectData(new_point_set, &*polygon, new_appearance,
                  check_if_present, false);

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
    for (top=correct_top_set->begin(); top!=correct_top_set->end(); top++)
      if (*top == new_top) return;
  }

  // Add the polygon
  correct_top_set->push_back(new_top);
  
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
  ClearLaserData(refresh);
}

void QGLCanvas::InitialiseTransformation(DataType focus_on_type)
{
  std::vector <ObjectPoints *>::const_iterator   object_point_set;
  std::vector <LineTopPtrVector>::const_iterator object_top_set;
  std::vector <const DataAppearance *>::iterator appearance;
  std::vector <LaserPoints *>::iterator          laser_point_set;
  LineTopPtrVector::const_iterator               object_top;

  rotation[0] = rotation[1] = 0.0;
  translation[0] = translation[1] = 0.0;
  zoom_factor = 1.0;
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
          focus_on_type == (*appearance)->TypeOfData())
        for (object_top=object_top_set->begin();
             object_top!=object_top_set->end(); object_top++)
          if (*object_top)
            bounds->Update((*object_top)->Bounds(
              (*object_point_set)->ObjectPointsRef()));
    }
    // Loop over all laser point sets
    for (laser_point_set = laser_point_sets.begin(),
         appearance = laser_appearances.begin();
         laser_point_set != laser_point_sets.end();
         laser_point_set++, appearance++) 
      if ((focus_on_type == NumDataTypes ||
           focus_on_type == (*appearance)->TypeOfData()) &&
          !(*laser_point_set)->empty())
        bounds->Update((*laser_point_set)->DeriveDataBounds(1).Bounds3D());

    // Determine maximum data range
    data_range = fmax(fmax(bounds->XRange(), bounds->YRange()),
                      bounds->ZRange()) * 1.1;
    
    if (data_range > 0.0) {
      pixel_size = data_range / min(width(), height());
      offset = (bounds->Minimum().vect() + bounds->Maximum().vect()) / 2.0;
    }
    else {
      data_range = pixel_size = 1.0;
      offset = Vector3D(); // Null vector
    }
    delete bounds;
  }
  else {
    data_range = pixel_size = 1.0;
    offset = Vector3D(); // Null vector
  }
  rot = Rotation3D(); // Unit matrix
}

void QGLCanvas::InitialiseColourLUT()
{
  int i, ir, ig, ib;
  long int j;

  j = 123456;
  for (i=0; i<216; i++) {
    j += 3111;
    j *= 1537;
    j -= (j/216)*216;
    ir = j / 36;
    j -= ir * 36;
    ig = j / 6;
    ib = j - ig * 6;
    // check for saturated colours or dark colours
    if (((ir == 0 || ir == 5) && (ig == 0 || ig == 5) && (ib == 0 || ib == 5))
        || (ir < 2 && ig < 2 && ib < 2) ) {
      i--;
    }
    else {
      random_colours[i][0] = ir * 0.2;
      random_colours[i][1] = ig * 0.2;
      random_colours[i][2] = ib * 0.2;
    }
  }
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

Line3D QGLCanvas::Canvas2World2(const QPoint &canvas_pos, bool apply_offset) const
{
  Position3D canvas_pos_3D;
  Position3D world_pos;
  Rotation3D rot_t;
  rot_t=rot.Transpose();
  
  canvas_pos_3D.SetX(canvas_pos.x()-width()/2.0);
  canvas_pos_3D.SetY(height()/2.0-canvas_pos.y());
  
  canvas_pos_3D*= pixel_size/zoom_factor ;
  canvas_pos_3D=canvas_pos_3D-Vector3D(translation[0], translation[1], 0.0);

  
  Plane plane1, plane2;
  Position3D point1, point2,footpoint; double d1, d2;
  Vector3D vec1, vec2;
  double a, b,c;
  //point1=Position3D(canvas_pos_3D.GetX()/rot_t.R(0,0),canvas_pos_3D.GetX()/rot_t.R(0,1),canvas_pos_3D.GetX()/rot_t.R(0,2));
  //point2=Position3D(canvas_pos_3D.GetY()/rot_t.R(1,0),canvas_pos_3D.GetY()/rot_t.R(1,1),canvas_pos_3D.GetY()/rot_t.R(1,2));
  vec1=rot_t.Row(0);a=vec1.X();b=vec1.Y();c=vec1.Z();
  d1=-canvas_pos_3D.GetX()/sqrt(a*a+b*b+c*c);
  plane1=Plane(); 
  plane1.SetNormal(vec1.Normalize());plane1.SetDistance(d1);
  
  vec2=rot_t.Row(1);a=vec2.X();b=vec2.Y();c=vec2.Z();
  d2=-canvas_pos_3D.GetY()/sqrt(a*a+b*b+c*c);
  plane2=Plane();
  plane2.SetNormal(vec2.Normalize());plane2.SetDistance(d2);
  
  Line3D line;
  Intersect2Planes(plane1,plane2, line);
  
  footpoint=line.FootPoint();
  footpoint=footpoint+offset;
  line.FootPoint(footpoint);
  return line;
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
}

bool QGLCanvas::HasData() const
{
  for (int type=0; type<NumDataTypes; type++)
    if (HasData((DataType) type)) return true;
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
      if (*object_point_set)
        if ((*object_point_set)->size() > 1 &&
            object_top_set->size() > 0) return true;
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
/*
void QGLCanvas::HorizontalView()
{
  double pi = 4.0 * atan(1.0);
  rotation[0] = -90.0;
  rot = Rotation3D(0.0, 0.0, -rotation[1]*pi/180.0) *
        Rotation3D(-rotation[0]*pi/180.0, 0.0, 0.0);
  update();
}
*/
void QGLCanvas::FitViewToData(DataType focus_on_type)
{
  InitialiseTransformation(focus_on_type);
  update();
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




void QGLCanvas::MakeTextureData(QString &current_dir)
{
  
   if (!texture_objpts) 
     return;            

  
  QImage *wall_image, *roof_image, *gap_image; 
  int num;
 
   GLdouble u_coord, v_coord; 
   int width, height;
 
  int holemaster;
  QString filename;
  
  GLdouble *coord;
 // double *tex_coord;
  
  //Positions3D corners;
  wall_image=new QImage(QDir::convertSeparators("c:/CITY/wall.jpg"));
  roof_image=new QImage(QDir::convertSeparators("c:/CITY/roof.jpg"));
  gap_image=new QImage(QDir::convertSeparators("c:/CITY/gap.jpg")); 
  
  texture_image=new QImage(QDir::convertSeparators(current_dir+"/texture.png"));
  QString texture_dir(current_dir+"/texture.png");

  if(texture_image->isNull())
  return;
  
  ObjectPoint obj;
  LineTopologies::const_iterator top, top2;
  
  ExteriorOrientation temp_ext;
  QString ext_dir(current_dir+"/ext_indirect.txt");
  
  cout<<"ext dir is"<<ext_dir.toStdString()<<endl;
  
  temp_ext.Read(ext_dir.toAscii());
  
 
 
  width=texture_image->width();
  height=texture_image->height();
  Line3D line;
  Position3D point;
  int counter=0;
  

 //generate and bind texture
  int success=-1;
  QImage *sub_texture;
  
  GLuint texName[texture_tops->size()];
  Positions3D corners[texture_tops->size()];
  bool texOK[texture_tops->size()];
  
  for(top=texture_tops->begin();top!=texture_tops->end();top++)
  {
  
  sub_texture=GenerateTextures(texture_objpts, *top, temp_ext, corners[top->Number()],success);
  
  if(top->HasAttribute(HoleTag))  //this is a hole, leave it to holemaster polygon
  { texOK[top->Number()]=false; continue;}
  
  if(success==0)   { texOK[top->Number()]=false; continue; }
  
  QImage temp_image;
  if(!top->HasAttribute(TextureTag)) 
   {
    switch(top->Label())
      {
      //wall
      case 2:            
           temp_image=wall_image->convertToFormat(QImage::Format_ARGB32);
           break;
      //roof
      case 6:       
           temp_image=roof_image->convertToFormat(QImage::Format_ARGB32);
           break;
      //roof extrusion
      case 8: 
          temp_image=wall_image->convertToFormat(QImage::Format_ARGB32);
               break;    
      //Gap
      case 12: 
           temp_image=gap_image->convertToFormat(QImage::Format_ARGB32);
           break;
      default:         
          temp_image=gap_image->convertToFormat(QImage::Format_ARGB32);
      }
  
  sub_texture=new QImage(temp_image);
  }
 // QImage temp_img, temp_img_buff;
  
 // temp_img.load("texture_test.jpg");
 
  
  
  glGenTextures(1, &texName[top->Number()]);  
  glBindTexture(GL_TEXTURE_2D,texName[top->Number()]);
  
  QImage temp_img_buff;
  temp_img_buff=QGLWidget::convertToGLFormat(*sub_texture);
  //glTexImage2D(GL_TEXTURE_2D, 0, 3, sub_texture->width(), sub_texture->height(), 0,
  //              GL_RGBA8, GL_UNSIGNED_BYTE, sub_texture->bits());
  
  glTexImage2D(GL_TEXTURE_2D, 0, 3, temp_img_buff.width(), temp_img_buff.height(), 0,
                GL_RGBA, GL_UNSIGNED_BYTE, temp_img_buff.bits());
 
 
  texOK[top->Number()]=true;
  
  filename=current_dir;
  filename.append("/texture");
  filename+=QString::number(top->Number());
  filename+=".jpg";
 // sub_texture->save(filename, "jpg");
  }
  
 
//tesselate and apply texture   
    GLuint list = glGenLists(1);
  glNewList(list, GL_COMPILE);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    qglColor(Qt::white);
   glShadeModel(GL_SMOOTH);
   tobj = gluNewTess();
   //gluTessCallback(tobj, GLU_TESS_VERTEX, 
   //               (GLvoid (*) ( ))&vertexCallback);
   gluTessCallback(tobj, GLU_TESS_VERTEX, 
                   (void (APIENTRY *)())vertexCallback);
   gluTessCallback(tobj, GLU_TESS_BEGIN, 
                   (void (APIENTRY *)())beginCallback);
   gluTessCallback(tobj, GLU_TESS_END, 
                   (void (APIENTRY *)())endCallback);
   gluTessCallback(tobj, GLU_TESS_ERROR, 
                   (void (APIENTRY *)())errorCallback);
   gluTessCallback(tobj, GLU_TESS_COMBINE, 
                   (void (APIENTRY *)())combineCallback);
 
   
  for(top=texture_tops->begin();top!=texture_tops->end();top++)
  {
  double scalar_u0, scalar_u1, scalar_u;
  double scalar_v0, scalar_v3, scalar_v;
   
   if(!texOK[top->Number()]) 
   {//printf(" skip %i", top->Number());
   continue;
   }
// printf("\n Bind texture for line %i \n ", top->Number());
 glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  
 glBindTexture(GL_TEXTURE_2D, texName[top->Number()]);
  
  
  gluTessProperty(tobj, GLU_TESS_WINDING_RULE,GLU_TESS_WINDING_ODD);
  gluTessBeginPolygon(tobj,NULL);
  
  gluTessBeginContour(tobj);
  
  for(int i=0;i<top->size()-1;i++)
 {  
  num=(*top)[i].Number();
  obj=(*texture_objpts).PointByNumber(num);
  
  line=Line3D(corners[top->Number()][0],corners[top->Number()][1]);
  scalar_u0=line.Scalar(corners[top->Number()][0]);scalar_u1=line.Scalar(corners[top->Number()][1]);
  point=line.Project(obj);
  scalar_u=line.Scalar(point);
  u_coord=(scalar_u-scalar_u0)/(scalar_u1-scalar_u0);
    
  line=Line3D(corners[top->Number()][0],corners[top->Number()][3]);
  scalar_v0=line.Scalar(corners[top->Number()][0]);scalar_v3=line.Scalar(corners[top->Number()][3]);
  point=line.Project(obj);
  scalar_v=line.Scalar(point);
  v_coord=(scalar_v-scalar_v3)/(scalar_v0-scalar_v3);
 
  //if(top->HasAttribute(TextureTag)) {printf("u and v coord is: %f  %f", u_coord, v_coord);}
  
  coord=(GLdouble *) malloc(5 * sizeof(GLdouble)); 
  coord[0]=obj.X() - offset.X(); coord[1]=obj.Y() - offset.Y(); coord[2]=obj.Z() - offset.Z();
  coord[3]=u_coord; coord[4]=v_coord;
  
  //printf("\n master:  %f %f %f \n ", coord[0],coord[1],coord[2]);  
  gluTessVertex(tobj,coord, coord);
  }
  gluTessEndContour(tobj);
  
   //deal with polygon with holes
  if(top->HasAttribute(HoleMasterTag))
  {
    Plane myplane(corners[top->Number()][0],corners[top->Number()][1],corners[top->Number()][2]);
    Position3D pt;
    holemaster=top->Attribute(HoleMasterTag);                                  
    for(top2=texture_tops->begin();top2!=texture_tops->end();top2++)
      {
            if(top2->HasAttribute(HoleTag)&&(top2->Attribute(HoleTag)==holemaster))  //we found a hole belong to this polygon
               {
                                                     // printf("found a hole, its line %i. its master is %i",top2->Number(), top2->Attribute(HoleTag));
               gluTessBeginContour(tobj);   
                  for(int j=0;j<top2->size()-1;j++)
                  {
                    num=(*top2)[j].Number();
                    obj=(*texture_objpts).PointByNumber(num);
  
                    pt=myplane.Project(obj);
                    obj.SetX(pt.GetX());obj.SetY(pt.GetY());obj.SetY(pt.GetY());
                    line=Line3D(corners[top->Number()][0],corners[top->Number()][1]);
                    scalar_u0=line.Scalar(corners[top->Number()][0]);scalar_u1=line.Scalar(corners[top->Number()][1]);
                    point=line.Project(obj);
                    scalar_u=line.Scalar(point);
                    u_coord=(scalar_u-scalar_u0)/(scalar_u1-scalar_u0);
    
                    line=Line3D(corners[top->Number()][0],corners[top->Number()][3]);
                    scalar_v0=line.Scalar(corners[top->Number()][0]);scalar_v3=line.Scalar(corners[top->Number()][3]);
                    point=line.Project(obj);
                    scalar_v=line.Scalar(point);
                    v_coord=(scalar_v-scalar_v3)/(scalar_v0-scalar_v3);
 
                    coord=(GLdouble *) malloc(5 * sizeof(GLdouble)); 
                    coord[0]=obj.X() - offset.X(); coord[1]=obj.Y() - offset.Y(); coord[2]=obj.Z() - offset.Z();
                    coord[3]=u_coord; coord[4]=v_coord;
  
         //       printf("\n hole:  %f %f %f \n ", coord[0],coord[1],coord[2]);   
                 
                  gluTessVertex(tobj,coord, coord);      
                          
                  }
               gluTessEndContour(tobj);
               }                                  
      }                                
  }
  
  gluTessEndPolygon(tobj);
 } 
 
 glEndList();
 sharedObject=list;
}


LaserPoints & QGLCanvas::LaserPointsWithDisplayColour(LaserPoints *laser_points,
                                              const DataAppearance *appearance) const
{
  LaserPoints::const_iterator laser_point, laser_point2, laser_point3;
  GLfloat                     intensity_scale, intensity_offset, intensity,
                              height_offset, height_scale,
                              pulselength_offset, pulselength_scale; 
  int                         red, green, blue, colour_index, value;
  float                       residual;
  LaserPoints                 *points;
  LaserPoint                  point;

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
          ColourAttribute(laser_point->Z(),
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
            colour_index = value - (value/216) * 216;
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
            colour_index = value - (value/216) * 216;
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
        case ColourBy10Scans:
          if (laser_point->HasAttribute(ScanNumberTag)) {
            value = laser_point->Attribute(ScanNumberTag);
	    if (appearance->PointColourMethod() == ColourBy10Scans)
              value /= 10;
            colour_index = value - (value/216) * 216;
            point.SetColour((int) (random_colours[colour_index][0]*255.0),
                            (int) (random_colours[colour_index][1]*255.0),
                            (int) (random_colours[colour_index][2]*255.0));
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
