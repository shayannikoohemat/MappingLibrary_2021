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
#include <stdlib.h>
#include <stdio.h>
#include "Database.h"

extern int lmodel2obj(ModelPts *, Absolute *, ObjPts **);
	
/*-----------------------------------------------------------
|
|    Written By: R.Th. Ursem
|          Date: Jul 28, 1999
| Modifications:
|
------------------------------------------------------------*/

int model2obj_c(char *model_file, char *object_file, char *absolute_file)
{
  Absolute *absolute;
  ObjPts   *object;
  ModelPts *model;
  int      error;

  absolute = Get_Absolute(absolute_file, &error);
  Print_Absolute(absolute);
  
  model    = Get_ModelPts(model_file);
  Print_ModelPts(model);

  if (lmodel2obj(model, absolute, &object)) {
    Put_ObjPts(object, object_file);
    Print_ObjPts(object);
  }
  return 1;
}
