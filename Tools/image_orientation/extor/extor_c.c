
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

/*
|    Written By: A van Voorden,
|                R.Th. Ursem
|                G. Vosselman
|          Date: Aug 18, 1999
| Modifications:
|
------------------------------------------------------------*/


int extor_c(char *campts_file, char *ctrlpts_file,
            char *interior_file, char *approx_exterior_file,
            int adjustment_output, char *exterior_file)
{
   int i, j, k;
   int a, b, c, d;
   int num_pts, show_output;
   int *lijstpnr, *lijstcodepasp;
   int error;

   double *lijstxpas, *lijstypas, *lijstzpas;
   double *lijstxpix, *lijstypix;
   double *lijstxpixvar, *lijstypixvar, *lijstxycovar;
   double xmod[4], ymod[4], zmod[4];
   double cc;
   
   Exterior ext, *apext;
   Interior *interior;
   CamPts *campts;
   CtrlPts *ctrlpts;
   
   void Search_Start_Points(), Exterior_Direct(), Exterior_Indirect();
   
   ctrlpts = Get_CtrlPts(ctrlpts_file);
   /*   Print_CtrlPts(ctrlpts); */

   campts = Get_CamPts(campts_file);
   /*   Print_CamPts(campts); */
   
   interior = Get_Interior(interior_file);
   Print_Interior(interior);
   
   if (ctrlpts == NULL || campts == NULL || interior == NULL)
   {
      fprintf(stderr, "extor_c: Could not read necessary input files\n");
      exit(1);
   }
   
   cc = interior->cc;
   show_output = adjustment_output;
   
   /*------ Allocatie van geheugen voor de lijsten ------*/
   num_pts = 0;
   for (i = 0; i < ctrlpts->num_pts; i++)
      for (j = 0; j < campts->num_pts; j++)
         if (ctrlpts->pts[i].num == campts->pts[j].num)
            num_pts++;

   lijstpnr      = (int *)calloc(num_pts, sizeof(int));
   lijstcodepasp = (int *)calloc(num_pts, sizeof(int));
   lijstxpas     = (double *)calloc(num_pts, sizeof(double));
   lijstypas     = (double *)calloc(num_pts, sizeof(double));
   lijstzpas     = (double *)calloc(num_pts, sizeof(double));
   lijstxpixvar  = (double *)calloc(num_pts, sizeof(double));
   lijstypixvar  = (double *)calloc(num_pts, sizeof(double));
   lijstxycovar  = (double *)calloc(num_pts, sizeof(double));
   lijstxpix     = (double *)calloc(num_pts, sizeof(double));
   lijstypix     = (double *)calloc(num_pts, sizeof(double));
      
   if (lijstpnr == NULL     || lijstcodepasp == NULL || lijstxpas == NULL    ||
       lijstypas == NULL    || lijstzpas == NULL     || lijstxpixvar == NULL ||
       lijstypixvar == NULL || lijstxycovar == NULL  || lijstxpix == NULL     ||
       lijstypix == NULL)
   {
      fprintf(stderr, "extor: Could not allocate enough memory\n");
      exit(1);
   }
   
   /* Zoek Fotopunten bij Paspunten */
   num_pts = 0;
   for (i = 0; i < ctrlpts->num_pts; i++)
      for (j = 0; j < campts->num_pts; j++)
         if (ctrlpts->pts[i].num == campts->pts[j].num &&
             ctrlpts->pts[i].status == FULL_CTRL)
         {
            lijstpnr[num_pts]      = ctrlpts->pts[i].num;
            lijstxpas[num_pts]     = ctrlpts->pts[i].x;
            lijstypas[num_pts]     = ctrlpts->pts[i].y;
            lijstzpas[num_pts]     = ctrlpts->pts[i].z;
            lijstcodepasp[num_pts] = ctrlpts->pts[i].status;
            
            lijstxpix[num_pts]     = campts->pts[j].x;
            lijstypix[num_pts]     = campts->pts[j].y;
            lijstxpixvar[num_pts]  = campts->pts[j].v_x;
            lijstypixvar[num_pts]  = campts->pts[j].v_y;
            lijstxycovar[num_pts]  = campts->pts[j].cv_xy;
            
            num_pts++;
         }
   /*
    * Nu zijn het aantal(num_pts) gemeenschappelijke punten bekend
    * De puntnrs staan in array lijstpnr. De bijbehorende coordinaten
    * codes varianties staan in de arrays lijstcodepasp,lijstxycovar,
    * lijstxpixvar,lijstypixvar, lijstxpas,lijstypas,lijstzpas,
    * lijstxpix,lijstypix.
    */
   
   printf("The following %d corresponding points were found:\n", num_pts);
   for (i = 0; i < num_pts; i++)                
      printf(" %d ", lijstpnr[i]);
   printf ("\n");       
   
/* Calculation of approximate values */

   if (approx_exterior_file == NULL) {
     printf("Calculation of approximate values with direct solution.\n");
     if (num_pts < 4) {
       printf(" extor_c: Not enough control points. Can't continue.\n");
       exit(1);
     }
     Search_Start_Points(num_pts, lijstpnr, lijstxpix, lijstypix, 
                         lijstxpas, lijstypas, lijstzpas,
                         &a, &b, &c, &d);
     Exterior_Direct(lijstxpas, lijstypas, lijstzpas, lijstxpix, lijstypix, 
                     a, b, c, d, 3.0, cc, &ext);
   }
   else {
     printf("Using approximate values from file %s\n", approx_exterior_file);
     apext = (Exterior *) Get_Exterior(approx_exterior_file, &error);
     ext = *apext;
     if (ext.w[0] == 0 && ext.f[0] == 0) {
       printf("Warning: Calculation of quaternion elements may fail if\n");
       printf("         omega and phi are zero.\n");
       printf("         Use slightly different values (e.g. 0.1 degree) instead.\n");
     }
   }

   Exterior_Indirect(num_pts, lijstpnr, lijstxpix, lijstypix, 
                     lijstxpixvar, lijstypixvar, lijstxycovar, 
                     lijstxpas, lijstypas, lijstzpas, 
                     show_output, cc, &ext);

   Put_Exterior(&ext, exterior_file);

   return 1;
}
