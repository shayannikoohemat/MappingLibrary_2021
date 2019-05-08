
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


#include <cstdlib>
#include <iostream>
#include <fstream>
#include "LaserPoints.h"
#include "LaserPoint.h"
#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "Positions3D.h"
#include "Vector3D.h"
#include "DataBoundsLaser.h"
#include <stdio.h>
using namespace std;
#define PI 3.1415926
double back_offset=3.0;


int main(int argc, char *argv[])
{
    
   LaserPoints lpts;
   
   Positions3D points;
   
   if(argc<3)
   cout<<"Not enough parameters. The format is: laser2p3d input.laser output.pts"<<endl;
   
   lpts.Read(argv[1]);
   
   for(int i=0;i<lpts.size();i++)
   points.push_back(Position3D(lpts[i].GetX(), lpts[i].GetY(), lpts[i].GetZ()));
   
   
   points.Write(argv[2]);
   
   
    
 
}
