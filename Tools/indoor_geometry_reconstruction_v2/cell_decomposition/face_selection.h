#include <LaserPoints.h>

#ifndef FACE_SELECTION_H
#define FACE_SELECTION_H


class FaceSelection
{
public:
    //FaceSelection();
    int faceSelection(LaserPoints segments, int min_segsize,
                                     double area_threshld, double dist_threshold,
                                     ObjectPoints faces_v,LineTopologies faces_e);
    // face selection and points association
    LaserPoints associatePointsToFace3D(LaserPoints segments, int min_segsize,
                                                       double dist_threshold,
                                                       double area_threshld,
                                                       int min_points_for_face_selection,
                                                       ObjectPoints faces_v,
                                                       LineTopologies faces_e,
                                                       LineTopologies &selected_faces);

private:
    const double PI = 3.14159;

};

#endif // FACE_SELECTION_H
