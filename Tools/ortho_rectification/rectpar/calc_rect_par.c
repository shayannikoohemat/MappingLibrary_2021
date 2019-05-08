#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stdmath.h"
#include "Database.h"

int Calculate_Rectification_Parameters(ImgPts *imgpts, CtrlPts *ctrlpts,
                                       ImgPts *imgptsref, double *par,
                                       int min_num_pts, double max_rms,
                                       int max_iter)
{
  int    num_pts, i, j;
  double *x_org, *y_org, *x_dest, *y_dest, *var;
  char   tmpstr[80];
  AdjustInfo info;

/* Allocate coordinate, variance and point number arrays */

  if (ctrlpts) num_pts = MIN(imgpts->num_pts, ctrlpts->num_pts);
  else num_pts = MIN(imgpts->num_pts, imgptsref->num_pts);
  x_org  = (double *) malloc(num_pts * sizeof(double));
  y_org  = (double *) malloc(num_pts * sizeof(double));
  x_dest = (double *) malloc(num_pts * sizeof(double));
  y_dest = (double *) malloc(num_pts * sizeof(double));
  var    = (double *) malloc(2 * num_pts * sizeof(double));
  info.numbers = (int *) malloc(num_pts * sizeof(int));

/* Fill the arrays in case of control points */

  num_pts = 0;
  if (ctrlpts) {
    for (i = 0; i < imgpts->num_pts; i++) {
      for (j = 0; j < ctrlpts->num_pts; j++) {
        if ((imgpts->pts[i].num == ctrlpts->pts[j].num) &&
             (ctrlpts->pts[j].status == PLANI_CTRL ||
              ctrlpts->pts[j].status == FULL_CTRL)) {
          x_org[num_pts]   = imgpts->pts[i].r;
          y_org[num_pts]   = imgpts->pts[i].c;

          if (imgpts->pts[i].v_r == 0 || imgpts->pts[i].v_c == 0)
            printf("WARNING: Variance factor of point %d equals 0, replaced by 0.25\n", imgpts->pts[i].num);
          var[2*num_pts]   = (imgpts->pts[i].v_r == 0) ? 0.25 : imgpts->pts[i].v_r;
          var[2*num_pts+1] = (imgpts->pts[i].v_c == 0) ? 0.25 : imgpts->pts[i].v_c;
          x_dest[num_pts]   = ctrlpts->pts[j].x;
          y_dest[num_pts]   = ctrlpts->pts[j].y;
          info.numbers[num_pts] = imgpts->pts[i].num;
          num_pts++;
        }
      }
    }
  }

/* Fill the arrays in case of reference image points */

  else {
    for (i = 0; i < imgpts->num_pts; i++) {
      for (j = 0; j < imgptsref->num_pts; j++) {
        if (imgpts->pts[i].num == imgptsref->pts[j].num) {
          x_org[num_pts]   = imgpts->pts[i].r;
          y_org[num_pts]   = imgpts->pts[i].c;

          if (imgpts->pts[i].v_r == 0 || imgpts->pts[i].v_c == 0)
            printf("WARNING: Variance factor of point %d equals 0, replaced by 0.25\n", imgpts->pts[i].num);
          var[2*num_pts]   = (imgpts->pts[i].v_r == 0) ? 0.25 : imgpts->pts[i].v_r;
          var[2*num_pts+1] = (imgpts->pts[i].v_c == 0) ? 0.25 : imgpts->pts[i].v_c;
          x_dest[num_pts]   = imgptsref->pts[j].r;
          y_dest[num_pts]   = imgptsref->pts[j].c;
          info.numbers[num_pts] = imgpts->pts[i].num;
          num_pts++;
        }
      }
    }
  }

/* Check if we have sufficient points */

  if (num_pts < MAX(4, min_num_pts)) {
    fprintf(stderr, "Error: Only %d corresponding points, but %d required.\n",
            num_pts, MAX(4, min_num_pts));
    return(0);
  }

/* Fill the info structure for adjust */

  info.output = stdout;
  info.n_o_p_p = 2;
  info.var_num_obs = 0;
  strcpy(tmpstr, "pixel");
  info.units = (char *)malloc( sizeof(char) * (strlen(tmpstr) + 1) );
  strcpy(info.units, tmpstr);
  strcpy(tmpstr, "row and column coordinate");
  info.order_string = (char *)malloc( sizeof(char) * (strlen(tmpstr) + 1) );
  strcpy(info.order_string, tmpstr);
  info.order_char = (char *)malloc( 2 * sizeof(char) );
  info.order_char[0] = 'r';
  info.order_char[1] = 'c';

/* Do the adjustment */

  if (!Robust_Cal_Trans_Pars(x_org, y_org, x_dest, y_dest, var, num_pts,
                             &info, par, num_pts - min_num_pts, max_rms,
                             max_iter)) return(1);
  else return(0);

}
