#include <stdio.h>
#include "Database.h"

void Rectify_to_ImgPts(ImgPts *org_imgpts, double *par, ImgPts *rect_imgpts)
{
  ImgPt *org_imgpt, *rect_imgpt;
  int   i;

  void Apply_Trans();

  for (i=0, org_imgpt=org_imgpts->pts, rect_imgpt=rect_imgpts->pts;
       i<org_imgpts->num_pts;
       i++, org_imgpt++, rect_imgpt++) {
    rect_imgpt->num = org_imgpt->num;
    Apply_Trans(&(rect_imgpt->r), &(rect_imgpt->c),
                org_imgpt->r, org_imgpt->c, par);
  }
}

void Rectify_to_ObjPts2D(ImgPts *org_imgpts, double *par, ObjPts2D *rect_objpts)
{
  ImgPt   *org_imgpt;
  ObjPt2D *rect_objpt;
  int     i;

  void Apply_Trans();

  for (i=0, org_imgpt=org_imgpts->pts, rect_objpt=rect_objpts->pts;
       i<org_imgpts->num_pts;
       i++, org_imgpt++, rect_objpt++) {
    rect_objpt->num = org_imgpt->num;
    Apply_Trans(&(rect_objpt->x), &(rect_objpt->y),
                org_imgpt->r, org_imgpt->c, par);
  }
}
