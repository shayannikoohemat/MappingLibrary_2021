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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "Database.h"

/*-----------------------------------------------------------
|
|  Routine Name: main() - Determination of relative orientation parameters
|
|    Written By: Geert Blommers - programming of the main body of the program,
|                R.Th. Ursem    - integration into Khoros,
|                               - Adjustment of obtained results
|          Date: Aug 16, 1999
| Modifications:
|
------------------------------------------------------------*/

int relor_c(char *left_campts_file, char *right_campts_file,
            char *interior_file, char *relative_file,
            int use_standard_approximations)
{
   extern void Coplan_direct(); /*De volgende functies zijn elders gedefinieerd (extern) */
   extern int Mod_Basis();   
   extern void Approx_Quaternion();
   extern void Coplan_indir();
   double ata[8][8], aty[8], wr[3], rg_z[3][3], bx, by, bz, e[3][3], q1, q2, q3;
   double *a, *y, *vary;
   int l, index, iter, i, j, count, num_pts;
   CamPts *L, *R;  /* pointer naar de file met beeldcoordinaten  */   
   double *qy, *qx, *err, *w, *lamx, *lamy, *naby, *par, f, varest, old_varest;
   double corr_sum, sigma_apriori;
   Exterior uitw, origin;
   Interior *inwendig;
   int retval;
   AdjustInfo info;

   inwendig = Get_Interior(interior_file);
   if (inwendig == NULL) {
     printf("relor: Error reading interior orientation from file %s\n", interior_file);
     exit(1);
   }
   f = inwendig->cc;

   L = Get_CamPts(left_campts_file);      /*  leest punten van Linker foto in */
   if ( L == NULL )
   {
      printf("relor: Error when reading left camera points from \"%s\"\n", left_campts_file); 
      exit(1);
   }
   Print_CamPts(L);               /* list de ingelezen file op het scherm */

   R = Get_CamPts(right_campts_file);       /*  leest punten van Rechter foto in */
   if ( R == NULL )
   {
      printf("relor: Error when reading right camera points from \"%s\"\n", right_campts_file); 
      exit(1);
   }
   Print_CamPts(R);               /* list de ingelezen file op het scherm */

   num_pts = 0;
   info.numbers = (int *)calloc(100, sizeof(int));
   for (i = 0; i < L->num_pts; i++)
      for (j = 0; j < R->num_pts; j++)
         if (L->pts[i].num == R->pts[j].num)
         {
            if (num_pts+1 % 100)
               info.numbers = (int *)realloc(info.numbers, (num_pts+100) * sizeof(int));
            info.numbers[num_pts++] = L->pts[i].num;
         }
   
   printf("The following numbers are used in the calculations:\n");
   for (i = 0; i < num_pts; i++)
      printf(" %d ", info.numbers[i]);
   printf("\n");

   if (num_pts >= 8 && !use_standard_approximations)
   {
      printf("Calculating approximate values with direct method.\n");
      Coplan_direct(L, R, f, ata, aty);
      
      index = Mod_Basis(ata, aty, e, wr, rg_z);
      bx = rg_z[index][0];  by = rg_z[index][1];  bz = rg_z[index][2];
      Approx_Quaternion(e, bx, by, bz, &q1, &q2, &q3); /*q1, q2 en q3 moeten retour dus adres doorgeven*/ 
   }
   else if (num_pts < 5)
   {
      fprintf(stderr, "Error: You need at least 5 corresponding points!\n");
      exit (1);
   }
   else
   {
      printf("Using default approximate values for standard aerial photography:\n");
      printf("\tbx = 1.0\n\tby = 0.0\n\tbz = 0.0\n\tUnit rotation matrix\n");
      bx = 1.0;
      by = bz = q1 = q2 = q3 = 0.0;
   }
   origin.x = origin.y = origin.z = origin.a = origin.b = origin.c = 0.0;
   Rot_From_Quat(&origin);
      

   uitw.x = bx;
   uitw.y = by;
   uitw.z = bz;

   uitw.a = q1;
   uitw.b = q2;
   uitw.c = q3;

   Rot_From_Quat(&uitw);
   Angles_From_Rot(&uitw);

   /* Allocatie van geheugenruimte voor de a-matrix, de
      waarnemingsvector y  en de vector vary met varianties van y */
   
   a    = (double *) malloc(5 * num_pts * sizeof(double));
   y    = (double *) malloc(    num_pts * sizeof(double));
   vary = (double *) malloc(    num_pts * sizeof(double));
   
   if (a == NULL || y == NULL || vary == NULL) {
      fprintf(stderr, "relor: Error in allocating space for A-matrix, y-vector or \
              vary-vector \n");
      exit(1);
   }
   
   par  = (double *)calloc( 5,   sizeof(double) );
   qy   = (double *)calloc( num_pts * (num_pts + 1) / 2, sizeof(double));
   qx   = (double *)calloc( num_pts * (num_pts + 1) / 2, sizeof(double) );
   err  = (double *)calloc( num_pts,   sizeof(double) );
   w    = (double *)calloc( num_pts,   sizeof(double) );
   lamx = (double *)calloc( num_pts,   sizeof(double) );
   lamy = (double *)calloc( num_pts,   sizeof(double) );
   naby = (double *)calloc( num_pts,   sizeof(double) );
   if (err == NULL || w == NULL || lamx == NULL || lamy == NULL || naby == NULL)
   {
      printf("relor: Not enough memory available for variables\n");
      exit( 1 );
   }

   by /= bx;
   bz /= bx;
   bx = 1.0;

   printf("Used approximate values :\n");
   printf("by = %9.6lf, bz = %9.6lf\n", by, bz);
   printf("w  = %9.6lf, f  = %9.6lf, k  = %9.6lf\n", uitw.w[0], uitw.f[0], uitw.k[0]);
   
   info.n_o_p_p = 1;
   info.order_string = (char *)calloc(80, sizeof(char));
   info.order_char = (char *)calloc(1, sizeof(char));
   info.units = (char *)calloc(20, sizeof(char));
   
   strcpy(info.order_string, "y - parallaxes");
   info.order_char[0] = 'y';
   strcpy(info.units, "millimeters");
   info.output = stdout;
   
   iter = 0;
   do 
   {
      printf("\nIteration number %d\n", iter++);
      
      Coplan_indir(L, R, by, bz, q1, q2, q3, f, a, y, vary, info.numbers, num_pts);

      old_varest = 0.0;
      for (i = 0; i < num_pts; i++)
         old_varest += vary[i];
      old_varest /= num_pts;
      for (i = 0; i < num_pts; i++)
         vary[i] /= old_varest;
      
      retval = Adjust(num_pts,      /* IN: Number of observations                 */
             5,            /* IN: Number of unknowns                              */
             a,            /* IN: Pointer to the full design matrix A[m][n]       */
             y,            /* IN: Pointer to the observations y[m]                */
             qy,           /* IN: variances of the observations (full)            */
             vary,         /* IN: variances of the observations (diagonal)        */
             no_corr,      /* IN: 0 = no correlation; 1 = unit; 2 = correlation   */
             doAll,        /* IN: depth, doAdjust or doAll (adjustment & testing) */
             old_varest,   /* IN: a priori variance factor                        */
             &varest,      /* OUT: a posteriori variance factor (sigma^)          */
             par,          /* OUT: adjusted unknowns                              */
             err,          /* OUT: least squares residuals                        */
             w,            /* OUT: w-tests datasnooping                           */
             qx,           /* OUT: variances of unknowns                          */
             naby,         /* OUT: internal reliability, nablas                   */
             lamy,         /* OUT: internal reliability, lambda-y                 */
             lamx,         /* OUT: external reliability, lamda-x                  */
             &info);       /* IN: info about printing results                     */
             
      if (retval)
      {
         printf("relor: Error in adjustment. Exiting.\n");
         exit(1);
      }
      
      corr_sum = 0;
      for (i = 0; i < 5; i++)
         corr_sum += fabs(par[i]);
      
      printf("Unknowns\t(Corrections)\n");
      printf("--------\t-------------\n");
      printf("by = %9.6lf\t(%9.6lf)\nbz = %9.6lf\t(%9.6lf)\nq1 = %9.6lf\t(%9.6lf)\
\nq2 = %9.6lf\t(%9.6lf)\nq3 = %9.6lf\t(%9.6lf)\n", 
             by, par[0], bz, par[1], q1, par[2], q2, par[3], q3, par[4]);
      
      by += par[0];
      bz += par[1];
      q1 += par[2];
      q2 += par[3];
      q3 += par[4];
      
      printf("\nSigma^ = %lf\nAbsolute sum of corrections = %lf\n", 
             varest, corr_sum);
      
   } while ( corr_sum > 0.000001 && iter < 10);
   
   uitw.x = bx;
   uitw.y = by;
   uitw.z = bz;
   
   uitw.a = q1;
   uitw.b = q2;
   uitw.c = q3;
   
   Rot_From_Quat(&uitw); 
   Angles_From_Rot(&uitw);

   Put_Relative(&origin, &uitw, relative_file);
   Print_Exterior(&uitw);


   return 1;
}
