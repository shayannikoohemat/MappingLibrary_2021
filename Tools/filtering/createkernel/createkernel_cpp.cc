
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


/*
--------------------------------------------------------------------------------
 Creation of a kernel used for filtering laser data.

 Initial creation:
 Author : George Vosselman
 Date   : 24-12-1999

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <malloc.h>
#include "Image.h"

/*
--------------------------------------------------------------------------------
                         The main createkernel function
--------------------------------------------------------------------------------
*/

void createkernel_cpp(double hdif0,
                      double dist1, double hdif1,
                      double dist2, double hdif2,
                      double dist3, double hdif3,
                      double dist4, double hdif4,
                      double dist5, double hdif5,
                      double distmax, double hdifmax,
			          double distint, char *kernel_file)
{
  Image kernel;
  
  kernel.CreateFilterKernel(hdif0, dist1, hdif1, dist2, hdif2,
                            dist3, hdif3, dist4, hdif4, dist5, hdif5,
                            distmax, hdifmax, distint);

  // Write the kernel
  kernel.Write(kernel_file);
}
